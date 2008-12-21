#include <pthread.h>
#include <signal.h>

#include "qemu-common.h"
#include "hw.h"
#include "pc.h"
#include "pci.h"
#include "console.h"
#include "vga_int.h"
#include "qemu-timer.h"
#include "sysemu.h"

#include "qxl_dev.h"
#include "qxl_interface.h"

#include "qemu-kvm.h"

#define KVM_MEM_ALIAS_QXL_RAM 0
#define KVM_MEM_ALIAS_QXL_VRAM 1
#define KVM_MEM_ALIAS_QXL_ROM 2


//#define QXL_IO_MEM

#define QXL_RAM_SIZE (32 * 1024 * 1024)
#define QXL_DEFAULT_COMPRESSION_LEVEL 2
#define QXL_SHARED_VGA_MODE FALSE
#define QXL_SAVE_VERSION 3


#define ASSERT(x) if (!(x)) { 					\
	printf("%s: ASSERT %s failed\n", __FUNCTION__, #x); 	\
	exit(-1);						\
}

#undef ALIGN
#define ALIGN(a, b) (((a) + ((b) - 1)) & ~((b) - 1))

#define TRUE 1
#define FALSE 0

#define QXL_DEV_NAME "qxl"

#define PCI_CLASS_DISPLAY 0x03
#define PCI_DISPLAY_SUBCLASS_VGA 0x00
#define PCI_DISPLAY_VGA_PROGIF 0x00
#define PCI_DISPLAY_SUBCLASS_OTHER 0x80
#define PCI_DISPLAY_OTHER_PROGIF 0x80

typedef struct __attribute__ ((__packed__)) PCIConf_s /*little-endian*/ {
    uint16_t vendor_ID;
    uint16_t device_ID;
    uint16_t command;
    uint16_t status;
    uint8_t revision;
    uint8_t class_prog_if;
    uint8_t class_sub;
    uint8_t class_base;
    uint8_t chache_line_size;
    uint8_t latency;
    uint8_t header;
    uint8_t builti_selftest;
    uint32_t base_address[6];
    uint32_t cardbus_cis_pointer;
    uint16_t subsystem_vendor_ID;
    uint16_t subsystem_ID;
    uint32_t expansion_rom_address;
    uint32_t reserved[2];
    uint8_t irq_line;
    uint8_t interrupt_pin;
    uint8_t min_grant;
    uint8_t max_letency;
} PCIConf;

enum {
    QXL_MODE_UNDEFINED,
    QXL_MODE_VGA,
    QXL_MODE_NATIVE,
};

typedef struct QXLState {
    uint32_t io_base;
    QXLRom *rom;
    QXLModes *modes;
    uint32_t rom_offset;
    uint32_t rom_size;

    uint8_t *ram_start;
    QXLRam *ram;
    uint32_t ram_offset;
    uint32_t ram_size;
    uint64_t ram_phys_addr;

    uint8_t *vram;
    unsigned long vram_offset;
    uint32_t vram_size;
#ifdef QXL_IO_MEM
    int qxl_io_memory;
#endif
    int num_free_res;
    QXLReleaseInfo *last_release;
    QXLWorkerRef worker;
    void *worker_data;
    int worker_data_size;
    int mode;
    QXLCommandRing vga_ring;
    uint32_t bits_unique;
    int running;
} QXLState;

typedef struct PCIQXLDevice {
    PCIDevice pci_dev;
    QXLState state;
    int id;
    DisplayState ds;
    struct PCIQXLDevice *dev_next;
    struct PCIQXLDevice *vga_next;
    int pipe_fd[2];
} PCIQXLDevice;

static PCIQXLDevice *dev_list = NULL;
static pthread_t main_thread;
static int compression_level = QXL_DEFAULT_COMPRESSION_LEVEL;

#define PIXEL_SIZE 0.2936875 //1280x1024 is 14.8" x 11.9" 

#define QXL_MODE(x_res, y_res, bits, orientation) \
    {0, x_res, y_res, bits, (x_res) * (bits) / 8, \
     PIXEL_SIZE * (x_res), PIXEL_SIZE * (y_res), orientation}

#define QXL_MODE_16_32(x_res, y_res, orientation) \
    QXL_MODE(x_res, y_res, 16, orientation), QXL_MODE(x_res, y_res, 32, orientation)

#define QXL_MODE_EX(x_res, y_res) \
    QXL_MODE_16_32(x_res, y_res, 0), QXL_MODE_16_32(y_res, x_res, 1), \
    QXL_MODE_16_32(x_res, y_res, 2), QXL_MODE_16_32(y_res, x_res, 3)

//#define QXL_HIRES_MODES

QXLMode qxl_modes[] = {
    QXL_MODE_EX(640, 480),
    QXL_MODE_EX(800, 600), 
    QXL_MODE_EX(832, 624), 
    QXL_MODE_EX(1024, 768),
    QXL_MODE_EX(1152, 864),
    QXL_MODE_EX(1152, 870),
    QXL_MODE_EX(1280, 720),
    QXL_MODE_EX(1280, 768),
    QXL_MODE_EX(1280, 800),
    QXL_MODE_EX(1280, 960),
    QXL_MODE_EX(1280, 1024),
    QXL_MODE_EX(1360, 768),
    QXL_MODE_EX(1366, 768),
    QXL_MODE_EX(1400, 1050),
    QXL_MODE_EX(1440, 900),
    QXL_MODE_EX(1600, 900),
    QXL_MODE_EX(1600, 1200),
    QXL_MODE_EX(1680, 1050),
    QXL_MODE_EX(1920, 1080),
#ifdef QXL_HIRES_MODES
    QXL_MODE_EX(1920, 1200),
    QXL_MODE_EX(1920, 1440),
    QXL_MODE_EX(2048, 1536),
    QXL_MODE_EX(2560, 1600),
    QXL_MODE_EX(2560, 2048),
    QXL_MODE_EX(2800, 2100),
    QXL_MODE_EX(3200, 2400),
#endif
};

typedef struct QXLVga {
    struct DisplayState *ds;
    PCIQXLDevice *clients;
    int active_clients;
    QEMUTimer *timer;
} QXLVga;

static void qxl_exit_vga_mode(PCIQXLDevice *d);
static void qxl_reset_state(PCIQXLDevice *d);

static QXLVga qxl_vga;

static QXLModeChangeNotifier qxl_mode_change_notifier = NULL;

inline void atomic_or(uint32_t *var, uint32_t add)
{
   __asm__ __volatile__ ("lock; orl %1, %0" : "+m" (*var) : "r" (add) : "memory");
}

inline uint32_t atomic_exchange(uint32_t val, uint32_t *ptr)
{
   __asm__ __volatile__("xchgl %0, %1" : "+q"(val), "+m" (*ptr) : : "memory");
   return val;
}

static void qxl_init_modes()
{
    int i;

    for (i = 0; i < sizeof(qxl_modes) / sizeof(QXLMode); i++) {
        qxl_modes[i].id = i;
    }
}

static UINT32 qxl_max_x_res()
{
    UINT32 res = 0;
    int i;

    for (i = 0; i < sizeof(qxl_modes) / sizeof(QXLMode); i++) {
        res = MAX(qxl_modes[i].x_res, res);
    }
    return res;
}

static UINT32 qxl_max_y_res()
{
    UINT32 res = 0;
    int i;

    for (i = 0; i < sizeof(qxl_modes) / sizeof(QXLMode); i++) {
        res = MAX(qxl_modes[i].y_res, res);
    }
    return res;
}

static int irq_level(PCIQXLDevice *d)
{
    return !!(d->state.ram->int_pending & d->state.ram->int_mask);

}

void qxl_update_irq()
{
    PCIQXLDevice *d = dev_list;
    while (d) {
        qemu_set_irq(d->pci_dev.irq[0], irq_level(d));
        d = d->dev_next;
    }
}

void qxl_set_compression_level(int new_compression_level)
{
    PCIQXLDevice *d = dev_list;

    compression_level = new_compression_level;
    while (d) {
        d->state.rom->compression_level = compression_level;
        d = d->dev_next;
    }
}

void do_set_qxl_log_level(int log_level)
{
    PCIQXLDevice *d = dev_list;
    while (d) {
        d->state.rom->log_level = log_level;
        d = d->dev_next;
    }
}

static void qxl_send_events(PCIQXLDevice *d, uint32_t events)
{
    ASSERT(d->state.running);
    mb();
    if ((d->state.ram->int_pending & events) == events) {
        return;
    }
    atomic_or(&d->state.ram->int_pending, events);
    if (pthread_self() == main_thread) {
        qemu_set_irq(d->pci_dev.irq[0], irq_level(d));
    } else {
        //dummy write in order to wake up the main thread
        //to update the irq line
        write(d->pipe_fd[1], d, 1);
    }
}

static void set_dreaw_area(PCIQXLDevice *d, QXLDevInfo *info)
{
    int stride = info->x_res * sizeof(uint32_t);
    info->draw_area.buf = (uint8_t *)d->state.ram_start + d->state.rom->draw_area_offset;
    info->draw_area.size = stride * info->y_res;
    info->draw_area.line_0 = info->draw_area.buf + info->draw_area.size - stride;
    info->draw_area.stride = -stride; 
    info->draw_area.width = info->x_res;
    info->draw_area.heigth = info->y_res;
}

void qxl_get_info(QXLDevRef dev_ref, QXLDevInfo *info)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    QXLState *state = &d->state;
    QXLMode *mode;

    if (state->mode == QXL_MODE_VGA) {
        info->x_res = qxl_vga.ds->width;
        info->y_res = qxl_vga.ds->height;
        info->bits = qxl_vga.ds->depth;

        info->phys_start = 0;
        info->phys_end = ~info->phys_start;
        info->phys_delta = 0;
        set_dreaw_area(d, info);
        return;
    }

    mode = &qxl_modes[d->state.rom->mode];

    info->x_res = mode->x_res;
    info->y_res = mode->y_res;
    info->bits = mode->bits;

    info->phys_start = (unsigned long)state->ram_start + state->rom->pages_offset;
    info->phys_end = (unsigned long)state->ram_start + state->ram_size;
    info->phys_delta = (long)state->ram_start - state->ram_phys_addr;
    set_dreaw_area(d, info);
}

static QXLCommandRing *qxl_active_ring(PCIQXLDevice *d)
{
    return (d->state.mode == QXL_MODE_VGA) ? &d->state.vga_ring : &d->state.ram->cmd_ring;
}

int qxl_get_command(QXLDevRef dev_ref, QXLCommand *cmd)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    QXLCommandRing *ring = qxl_active_ring(d);
    int notify;

    if (RING_IS_EMPTY(ring)) {
        return FALSE;
    }
    *cmd = *RING_CONS_ITEM(ring);
    RING_POP(ring, notify);
    if (d->state.mode != QXL_MODE_VGA && notify) {
        qxl_send_events(d, QXL_INTERRUPT_DISPLAY);
    }
    return TRUE;
}

int qxl_has_command(QXLDevRef dev_ref)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    QXLCommandRing *ring = qxl_active_ring(d);
    return !RING_IS_EMPTY(ring);
}

int qxl_get_cursor_command(QXLDevRef dev_ref, QXLCommand *cmd)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    QXLCursorRing *ring;
    int notify;

    if (d->state.mode == QXL_MODE_VGA) {
        return 0;
    }

    ring = &d->state.ram->cursor_ring;

    if (RING_IS_EMPTY(ring)) {
        return 0;
    }
    *cmd = *RING_CONS_ITEM(ring);
    RING_POP(ring, notify);
    if (d->state.mode != QXL_MODE_VGA && notify) {
        qxl_send_events(d, QXL_INTERRUPT_CURSOR);
    }
    return 1;
}

const Rect *qxl_get_update_area(QXLDevRef dev_ref)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    return &d->state.ram->update_area;
}

int qxl_req_cmd_notification(QXLDevRef dev_ref)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    int wait;

    RING_CONS_WAIT(qxl_active_ring(d), wait);
    return wait;
}

int qxl_req_cursor_notification(QXLDevRef dev_ref)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    int wait;

    if (d->state.mode == QXL_MODE_VGA) {
        return 1;
    }

    RING_CONS_WAIT(&d->state.ram->cursor_ring, wait);
    return wait;
}

#define QXL_FREE_BUNCH_SIZE 10

static inline void qxl_push_free_res(PCIQXLDevice *d)
{
    QXLState *state = &d->state;
    QXLReleaseRing *ring = &state->ram->release_ring;

    ASSERT(state->mode != QXL_MODE_VGA);
    if (RING_IS_EMPTY(ring) || (state->num_free_res == QXL_FREE_BUNCH_SIZE &&
                                ring->prod - ring->cons + 1 != ring->num_items)) {
        int notify;

        RING_PUSH(ring, notify);
        if (notify) {
            qxl_send_events(d, QXL_INTERRUPT_DISPLAY);
        }
        *RING_PROD_ITEM(ring) = 0;
        state->num_free_res = 0;
        state->last_release = NULL;
    }
}

void qxl_release_resource(QXLDevRef dev_ref, QXLReleaseInfo *release_info)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    UINT64 id = release_info->id;
    QXLState *state = &d->state;
    QXLReleaseRing *ring;
    UINT64 *item;

    if (state->mode == QXL_MODE_VGA) {
        free((void *)id);
        return;
    }
    ring = &state->ram->release_ring;
    item = RING_PROD_ITEM(ring);
    if (*item == 0) {
        release_info->next = 0;
        *item = id;
        state->last_release = release_info;
    } else {
        state->last_release->next = release_info->id;
        release_info->next = 0;
        state->last_release = release_info;
    }   
    
    state->num_free_res++;

    qxl_push_free_res(d);
}

void qxl_set_save_data(QXLDevRef dev_ref, void *data, int size)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    QXLState *state = &d->state;

    free(state->worker_data);
    state->worker_data = data;
    state->worker_data_size = size;
}

void *qxl_get_save_data(QXLDevRef dev_ref)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    QXLState *state = &d->state;

    return state->worker_data;

}

int qxl_flush_resources(QXLDevRef dev_ref)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;
    int ret;

    if (d->state.mode == QXL_MODE_VGA) {
        return 0;
    }
    ret = d->state.num_free_res; 
    if (ret) {
        qxl_push_free_res(d);
    }
    return ret;
}

void qxl_notify_update(QXLDevRef dev_ref, uint32_t update_id)
{
    PCIQXLDevice *d = (PCIQXLDevice *)dev_ref;

    if (d->state.mode == QXL_MODE_VGA) {
        return;
    }
    d->state.rom->update_id = update_id;
    qxl_send_events(d, QXL_INTERRUPT_DISPLAY);
}

static void qxl_detach(PCIQXLDevice *d)
{
    QXLCommandRing *ring;

    if (d->state.mode == QXL_MODE_UNDEFINED) {
        return;
    }

    qxl_worker_detach(d->state.worker);
    if (d->state.mode != QXL_MODE_VGA) {
        RING_INIT(&d->state.ram->cmd_ring);
        RING_INIT(&d->state.ram->cursor_ring);
        return;
    }
    
    ring = &d->state.vga_ring;
        
    while (!RING_IS_EMPTY(ring)) {
        QXLDrawable *drawable;
        QXLCommand *cmd;
        int notify;
        
        cmd = RING_CONS_ITEM(ring);
        RING_POP(ring, notify);
        ASSERT(cmd->type == QXL_CMD_DRAW);
        drawable = (QXLDrawable *)cmd->data;
        free(drawable);
    }
}

static void qxl_notify_mode_change()
{
    PCIQXLDevice *d = dev_list;
    int qxl_mode_native_counter = 0;
    int mode = 0;

    if (!qxl_mode_change_notifier){
        return;
    }
    if (qxl_vga.active_clients > 0) {
        qxl_mode_change_notifier(FALSE, 0, 0);
        return;
    }
    while (d && qxl_mode_native_counter < 2) {
        if (d->state.mode == QXL_MODE_NATIVE && ++qxl_mode_native_counter == 1) {
            mode = d->state.rom->mode;
        }
        d = d->dev_next;
    }
    if (qxl_mode_native_counter == 1) {
        qxl_mode_change_notifier(TRUE, qxl_modes[mode].x_res, qxl_modes[mode].y_res);
    } else {
        qxl_mode_change_notifier(FALSE, 0, 0);
    }
}

void qxl_register_mode_change(QXLModeChangeNotifier notifier){
    qxl_mode_change_notifier = notifier;
    qxl_notify_mode_change();
}

void qxl_set_mm_time(uint32_t stamp)
{
    PCIQXLDevice *d = dev_list;
    while (d) {
        d->state.rom->mm_clock = stamp;
        d = d->dev_next;
    }
}

static void qxl_set_mode(PCIQXLDevice *d, uint32_t mode)
{
    if (mode > sizeof(qxl_modes) / sizeof(QXLMode)) {
        printf("%s: bad mode %u\n", __FUNCTION__, mode);
        return;
    }
    printf("%s: %u\n",__FUNCTION__, mode);
    qxl_detach(d);
    ASSERT(RING_IS_EMPTY(&d->state.ram->cmd_ring));
    ASSERT(RING_IS_EMPTY(&d->state.ram->cursor_ring));
    ASSERT(RING_IS_EMPTY(&d->state.vga_ring));
    qxl_reset_state(d);
    qxl_exit_vga_mode(d);
    d->state.rom->mode = mode;
    memset(d->state.vram, 0, d->state.vram_size);
    d->state.mode = QXL_MODE_NATIVE;
    qxl_notify_mode_change();
    qxl_worker_attach(d->state.worker);
}

void *vga_context = NULL;

void qxl_set_vga_context(void *context)
{
    vga_context = context;
}

static void qxl_add_vga_client()
{
    if (qxl_vga.active_clients++ == 0) {
        qemu_mod_timer(qxl_vga.timer, qemu_get_clock(rt_clock));
    }
}

static void qxl_remove_vga_client()
{
    qxl_vga.active_clients--;
}

static void qxl_enter_vga_mode(PCIQXLDevice *d)
{
    if (d->state.mode == QXL_MODE_VGA || (!QXL_SHARED_VGA_MODE && d->id)) {
        return;
    }
    printf("%u: %s\n", d->id, __FUNCTION__);
    d->state.rom->mode = ~0;
    d->state.mode = QXL_MODE_VGA;
    qxl_notify_mode_change();
    qxl_add_vga_client();
    vga_on_qxl_enter_vga(vga_context);
}

/* reset the state (assuming the worker is detached) */
static void qxl_reset_state(PCIQXLDevice *d)
{
    QXLRam *ram = d->state.ram;
    QXLRom *rom = d->state.rom;

    ASSERT(RING_IS_EMPTY(&ram->cmd_ring));
    ASSERT(RING_IS_EMPTY(&ram->cursor_ring));
    ASSERT(RING_IS_EMPTY(&d->state.vga_ring));
    ram->magic = QXL_RAM_MAGIC;
    ram->int_pending = 0;
    ram->int_mask = 0;
    rom->update_id = 0;
    RING_INIT(&ram->cmd_ring);
    RING_INIT(&ram->cursor_ring);
    RING_INIT(&d->state.vga_ring);
    RING_INIT(&ram->release_ring);
    *RING_PROD_ITEM(&ram->release_ring) = 0;
    d->state.num_free_res = 0;
    d->state.last_release = NULL;
}

/* reset: detach, reset_state, re-attach */
static void qxl_reset(PCIQXLDevice *d)
{
    printf("%s\n", __FUNCTION__);
    qxl_detach(d);
    qxl_reset_state(d);
    if (QXL_SHARED_VGA_MODE || !d->id) {
        qxl_enter_vga_mode(d);
        qxl_worker_attach(d->state.worker);
    } else {
        d->state.mode = QXL_MODE_UNDEFINED;
        qxl_notify_mode_change();
    }
}

static void ioport_write(void *opaque, uint32_t addr, uint32_t val)
{
    PCIQXLDevice *d = (PCIQXLDevice *)opaque;
    uint32_t io_port = addr - d->state.io_base;
#ifdef DEBUG_QXL
    printf("%s: addr 0x%x val 0x%x\n", __FUNCTION__, addr, val);
#endif
    if (d->state.mode != QXL_MODE_NATIVE && io_port != QXL_IO_RESET && io_port != QXL_IO_SET_MODE) {
        printf("%s: unexpected port 0x%x in vga mode\n", __FUNCTION__, io_port);
        return;
    }
    switch (io_port) {
    case QXL_IO_UPDATE_AREA:
        qxl_worker_update_area(d->state.worker);
        break;
    case QXL_IO_NOTIFY_CMD:
        qxl_worker_wakeup(d->state.worker);
        break;
    case QXL_IO_NOTIFY_CURSOR:
        qxl_worker_wakeup(d->state.worker);
        break;
    case QXL_IO_UPDATE_IRQ:
        qemu_set_irq(d->pci_dev.irq[0], irq_level(d));
        break;
    case QXL_IO_NOTIFY_OOM:
        //todo: add counter
        if (!RING_IS_EMPTY(&d->state.ram->release_ring)) {
            break;
        }
        pthread_yield();
        if (!RING_IS_EMPTY(&d->state.ram->release_ring)) {
            break;
        }
        qxl_worker_oom(d->state.worker);
        break;
    case QXL_IO_LOG:
        printf("%u: %s", d->id, d->state.ram->log_buf);
        break;
    case QXL_IO_RESET:
        printf("%u: QXL_IO_RESET\n", d->id);
        qxl_reset(d);
        break;
    case QXL_IO_SET_MODE:
        printf("%u: QXL_IO_SET_MODE\n", d->id);
        qxl_set_mode(d, val);
        break;
    default:
        printf("%s: unexpected addr 0x%x val 0x%x\n", __FUNCTION__, addr, val);
    }
}

static uint32_t ioport_read(void *opaque, uint32_t addr)
{
    printf("%s: unexpected\n", __FUNCTION__);
    return 0xff;
}


static void ioport_map(PCIDevice *pci_dev, int region_num, 
                       uint32_t addr, uint32_t size, int type)
{
    QXLState *s = &((PCIQXLDevice *)pci_dev)->state;

    printf("%s: base 0x%x size 0x%x\n", __FUNCTION__, addr, size);
    s->io_base = addr;
    register_ioport_write(addr, size, 1, ioport_write, pci_dev);
    register_ioport_read(addr, size, 1, ioport_read, pci_dev);
}

static void rom_map(PCIDevice *d, int region_num,
                       uint32_t addr, uint32_t size, int type)
{
    QXLState *s = &((PCIQXLDevice *)d)->state;

    printf("%s: addr 0x%x size 0x%x\n", __FUNCTION__, addr, size);

    ASSERT((addr & (size - 1)) == 0);
    ASSERT(size ==  s->rom_size);

    cpu_register_physical_memory(addr, size, s->rom_offset | IO_MEM_ROM);
    if (kvm_enabled() && kvm_create_memory_alias(kvm_context,
                                               addr,
                                               size,
                                               s->rom_offset)) {
        printf("%s: create memory alias failed\n", __FUNCTION__);
        exit(-1);
    }
}

static void ram_map(PCIDevice *d, int region_num,
                     uint32_t addr, uint32_t size, int type)
{
    QXLState *s = &((PCIQXLDevice *)d)->state;

    printf("%s: addr 0x%x size 0x%x\n", __FUNCTION__, addr, size);

    ASSERT((addr & (size - 1)) == 0);
    ASSERT((addr & ~TARGET_PAGE_MASK) == 0);
    ASSERT(size ==  s->ram_size);
    ASSERT((size & ~TARGET_PAGE_MASK) == 0);
    s->ram_phys_addr = addr;
    cpu_register_physical_memory(addr, size, s->ram_offset | IO_MEM_RAM);
    if (kvm_enabled() && kvm_create_memory_alias(kvm_context,
                                               addr,
                                               size,
                                               s->ram_offset)) {
        printf("%s: create memory alias failed\n", __FUNCTION__);
        exit(-1);
    }
}

static void vram_map(PCIDevice *d, int region_num,
                     uint32_t addr, uint32_t size, int type)
{
    QXLState *s = &((PCIQXLDevice *)d)->state;

    printf("%s: addr 0x%x size 0x%x\n", __FUNCTION__, addr, size);

    ASSERT((addr & (size - 1)) == 0);
    ASSERT((addr & ~TARGET_PAGE_MASK) == 0);
    ASSERT(size ==  s->vram_size);
    ASSERT((size & ~TARGET_PAGE_MASK) == 0);
#ifdef QXL_IO_MEM
    cpu_register_physical_memory(addr, size, s->qxl_io_memory);
#else
    cpu_register_physical_memory(addr, size, s->vram_offset | IO_MEM_RAM);
#endif
    if (kvm_enabled() && kvm_create_memory_alias(kvm_context,
                                               addr,
                                               size,
                                               s->vram_offset)) {
        printf("%s: create memory alias failed\n", __FUNCTION__);
        exit(-1);
    }
}


#ifdef QXL_IO_MEM

static void qxl_mem_writeb(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    printf("%s: addr 0x%lx value 0x%x\n", __FUNCTION__, addr, value);
}

static void qxl_mem_writew(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    printf("%s: addr 0x%lx value 0x%x\n", __FUNCTION__, addr, value);
}

static void qxl_mem_writel(void *opaque, target_phys_addr_t addr, uint32_t value)
{
    printf("%s: addr 0x%lx value 0x%x\n", __FUNCTION__, addr, value);
}


static uint32_t qxl_mem_readb(void *opaque, target_phys_addr_t addr)
{
    printf("%s: addr 0x%lx\n", __FUNCTION__, addr);
    return 0x000000FF;
}

static uint32_t qxl_mem_readw(void *opaque, target_phys_addr_t addr)
{
    printf("%s: addr 0x%lx\n", __FUNCTION__, addr);
    return 0x0000FFFF;
}

static uint32_t qxl_mem_readl(void *opaque, target_phys_addr_t addr)
{
    printf("%s: addr 0x%lx\n", __FUNCTION__, addr);
    return 0xFFFFFFFF;
}


static CPUReadMemoryFunc *qxl_mem_read[3] = {
    qxl_mem_readb,
    qxl_mem_readw,
    qxl_mem_readl,
};

static CPUWriteMemoryFunc *qxl_mem_write[3] = {
    qxl_mem_writeb,
    qxl_mem_writew,
    qxl_mem_writel,
};
#endif

static uint32_t init_qxl_rom(PCIQXLDevice *d, uint8_t *buf, uint32_t vram_size,
                             uint32_t *max_fb)
{
    QXLRom *rom = (QXLRom *)buf;
    QXLModes *modes = (QXLModes *)(rom + 1);
    QXLState *s = &d->state;
    int i;

    rom->magic = QXL_ROM_MAGIC;
    rom->id = d->id;
    rom->mode = 0;
    rom->modes_offset = sizeof(QXLRom);
    rom->draw_area_size =qxl_max_x_res() * sizeof(uint32_t) * qxl_max_y_res();
    rom->compression_level = compression_level;
    rom->log_level = 0;

    *max_fb = 0;
    modes->n_modes = sizeof(qxl_modes) / sizeof(QXLMode);

    for (i = 0; i < modes->n_modes; i++) {
        *max_fb =  MAX(qxl_modes[i].y_res * qxl_modes[i].stride, *max_fb);
        modes->modes[i] = qxl_modes[i];
    }
    s->rom = rom;
    s->modes = modes;
    return (uint32_t)((uint8_t *)&modes->modes[i] - buf);
}

static void init_qxl_ram(QXLState *s, uint8_t *buf)
{
    uint32_t draw_area_size;
    uint32_t ram_header_size;

    s->ram_start = buf;

    draw_area_size = s->rom->draw_area_size;
    ram_header_size = ALIGN(sizeof(*s->ram), 8);
    ASSERT(ram_header_size + draw_area_size < QXL_RAM_SIZE);

    s->rom->ram_header_offset = QXL_RAM_SIZE - ram_header_size;
    s->ram = (QXLRam *)(buf + s->rom->ram_header_offset);
    s->ram->magic = QXL_RAM_MAGIC;
    RING_INIT(&s->ram->cmd_ring);
    RING_INIT(&s->ram->cursor_ring);
    RING_INIT(&s->ram->release_ring);
    *RING_PROD_ITEM(&s->ram->release_ring) = 0;

    s->rom->draw_area_offset = s->rom->ram_header_offset - draw_area_size;
    s->rom->pages_offset = 0;
    s->rom->num_io_pages = (QXL_RAM_SIZE - (draw_area_size + ram_header_size)) >> TARGET_PAGE_BITS;
    ASSERT((s->rom->num_io_pages << TARGET_PAGE_BITS) >= VGA_RAM_SIZE);
    printf("%s: npages %u\n", __FUNCTION__, s->rom->num_io_pages);
}

inline uint32_t msb_mask(uint32_t val)
{
    uint32_t mask;

    do {
        mask = ~(val - 1) & val;
        val &= ~mask;
    } while (mask < val);

    return mask;
}

static void qxl_display_update(struct DisplayState *ds, int x, int y, int w, int h)
{
    PCIQXLDevice *client;

    client = qxl_vga.clients;
    while (client) {
        if (client->state.mode == QXL_MODE_VGA && client->state.running) {
            QXLDrawable *drawable;
            QXLImage *image;
            QXLCommandRing *ring;
            QXLCommand *cmd;
            int wait;
            int notify;

            drawable = (QXLDrawable *)malloc(sizeof(*drawable) + sizeof(*image));
            ASSERT(drawable);
            image = (QXLImage *)(drawable + 1);
            drawable->bbox.left = x;
            drawable->bbox.right = x + w;
            drawable->bbox.top = y;
            drawable->bbox.bottom = y + h;
            drawable->clip.type = CLIP_TYPE_NONE;
            drawable->clip.data = 0;
            drawable->effect = QXL_EFFECT_OPAQUE;
            drawable->release_info.id = (UINT64)drawable;
            drawable->bitmap_offset = 0;
            drawable->type = QXL_DRAW_COPY;

            drawable->u.copy.rop_decriptor =  ROPD_OP_PUT;
            drawable->u.copy.src_bitmap = (PHYSICAL)image;
            drawable->u.copy.src_area.left = drawable->u.copy.src_area.top = 0;
            drawable->u.copy.src_area.right = w;
            drawable->u.copy.src_area.bottom = h;
            drawable->u.copy.scale_mode = 0;
            memset(&drawable->u.copy.mask, 0, sizeof(QMask));

            image->descriptor.type = IMAGE_TYPE_BITMAP;
            image->descriptor.flags = 0;
            QXL_SET_IMAGE_ID(image, QXL_IMAGE_GROUP_DEVICE, ++client->state.bits_unique);
            image->bitmap.flags = QXL_BITMAP_DIRECT | QXL_BITMAP_TOP_DOWN | QXL_BITMAP_UNSTABLE;
            image->bitmap.format = BITMAP_FMT_32BIT;
            image->bitmap.stride = ds->linesize;
            image->descriptor.width = image->bitmap.x = w;
            image->descriptor.height = image->bitmap.y = h;
            image->bitmap.data = (PHYSICAL)(ds->data + y * ds->linesize + x * 4);
            image->bitmap.palette = 0;

            ring = &client->state.vga_ring;
            for (;;) {    
                RING_PROD_WAIT(ring, wait);
                if (wait) {
                    printf("%s: wait\n", __FUNCTION__);
                    usleep(10000);
                    continue;
                }
                break;
            }

            cmd = RING_PROD_ITEM(ring);
            cmd->type = QXL_CMD_DRAW;
            cmd->data = (PHYSICAL)drawable;
            RING_PUSH(ring, notify);
            if (notify) {
                qxl_worker_wakeup(client->state.worker);
            }
        }
        client = client->vga_next;
    }
}

static void qxl_display_resize(struct DisplayState *ds, int w, int h)
{
    PCIQXLDevice *client; 
    uint8_t *data = ds->data;
    size_t buf_size;

    ds->linesize = w * 4;
    buf_size = ds->linesize * (h + 1);
    ds->data = malloc(buf_size);
    memset(ds->data, 0, buf_size);
    ds->depth = 32;
    ds->width = w;
    ds->height = h;

    client = qxl_vga.clients;
    while (client) {
        if (client->state.mode == QXL_MODE_VGA) {
            printf("%s\n", __FUNCTION__);
            qxl_reset(client);
        }
        client = client->vga_next;
    }
    free(data);
}

static void qxl_display_refresh(struct DisplayState *ds)
{
    if (qxl_vga.active_clients) {
        vga_hw_update();
    }
}

#define DISPLAY_REFRESH_INTERVAL 30

static void display_update(void *opaque)
{
    if (!qxl_vga.active_clients) {
        return;
    }
    qxl_display_refresh(qxl_vga.ds);
    qemu_mod_timer(qxl_vga.timer, qemu_get_clock(rt_clock) + DISPLAY_REFRESH_INTERVAL);
}

void qxl_init_display(DisplayState *ds)
{
    ASSERT(ds->data == NULL);
    memset(&qxl_vga, 0, sizeof(qxl_vga));
    qxl_vga.ds = ds;
    ds->dpy_update = qxl_display_update;
    ds->dpy_resize = qxl_display_resize;
    ds->dpy_refresh = qxl_display_refresh;
    qxl_display_resize(ds, 640, 400);

    qxl_vga.timer = qemu_new_timer(rt_clock, display_update, NULL);
    qemu_mod_timer(qxl_vga.timer, qemu_get_clock(rt_clock));
}

static void qxl_exit_vga_mode(PCIQXLDevice *d)
{
    if (d->state.mode != QXL_MODE_VGA) {
        return;
    }
    printf("%s\n", __FUNCTION__);
    qxl_remove_vga_client();
    d->state.mode = QXL_MODE_UNDEFINED;
    vga_on_qxl_exit_vga(vga_context);
}

int qxl_vga_touch(void)
{
    PCIQXLDevice *d = qxl_vga.clients;
    int ret = FALSE;
    while (d) {
        if (d->state.mode != QXL_MODE_VGA && (QXL_SHARED_VGA_MODE || !d->id)) {
            qxl_reset(d);
            ret = TRUE;
        }
        d = d->vga_next;
    }
    return ret;
}

static void qxl_save(QEMUFile* f, void* opaque)
{
    PCIQXLDevice* d=(PCIQXLDevice*)opaque;
    QXLState*     s=&d->state;
    uint32_t      last_release_offset;

    qxl_worker_save(d->state.worker);
    pci_device_save(&d->pci_dev, f);

    qemu_put_be32(f, s->rom->mode);
    qemu_put_be32(f, s->num_free_res);

    if (s->last_release == NULL)
        last_release_offset = 0;
    else
        last_release_offset = (uint8_t *)s->last_release - phys_ram_base;
    printf("QXL SAVE -- last_release_offset=%u \n", last_release_offset);
    ASSERT(last_release_offset < phys_ram_size);
    qemu_put_be32(f, last_release_offset);

    qemu_put_be32(f, s->mode);
    qemu_put_be32(f, s->bits_unique);

    qemu_put_buffer(f, s->ram_start, s->ram_size);

    qemu_put_be32(f, s->worker_data_size);
    if (s->worker_data_size) {
        qemu_put_buffer(f, s->worker_data, s->worker_data_size);
    }
}

static int qxl_load(QEMUFile* f,void* opaque,int version_id)
{
    PCIQXLDevice* d = (PCIQXLDevice*)opaque;
    QXLState*     s = &d->state;
    uint32_t      last_release_offset;
    int           ret;

    if (version_id != QXL_SAVE_VERSION)
        return -EINVAL;

    ret = pci_device_load(&d->pci_dev, f);
    if (ret) {
       printf("%s: error=%d\n", __FUNCTION__, ret);
       return ret;
    }

    if (d->state.mode != QXL_MODE_UNDEFINED) {
        qxl_worker_detach(d->state.worker);
    }

    if (s->mode == QXL_MODE_VGA) {
        qxl_remove_vga_client();
    }

    s->rom->mode    = qemu_get_be32(f);
    s->num_free_res = qemu_get_be32(f);

    last_release_offset = qemu_get_be32(f);
    if (last_release_offset >= phys_ram_size) {
        fprintf(stderr, "QXL LOAD: last_release_offset=%u too big\n",
               last_release_offset);
        exit(-1);
    }
    if (last_release_offset == 0)
        s->last_release = NULL;
    else
        s->last_release = (QXLReleaseInfo *)(phys_ram_base + last_release_offset);
    s->mode = qemu_get_be32(f);
    s->bits_unique = qemu_get_be32(f);

    qemu_get_buffer(f, s->ram_start, s->ram_size);

    free(s->worker_data);
    s->worker_data = NULL;
    s->worker_data_size = qemu_get_be32(f);
    if (s->worker_data_size) {
        if (!(s->worker_data = malloc(s->worker_data_size))) {
            fprintf(stderr, "QXL LOAD: alloc worker data failed %u\n", s->worker_data_size);
            exit(-1);
        }
        qemu_get_buffer(f, s->worker_data, s->worker_data_size);
    }

    if (version_id == 2) {
        qemu_get_byte(f);
    }

    if (s->mode == QXL_MODE_VGA) {
        qxl_add_vga_client();
    }
    if (s->mode != QXL_MODE_UNDEFINED) {
        qxl_worker_attach(d->state.worker);
        qxl_worker_load(d->state.worker);
    }
    qxl_notify_mode_change();
    return 0;
}

static void qxl_pipe_read(void *opaque)
{
    PCIQXLDevice *d = opaque;
    int len;
    char dummy;

    while (1) {
        len = read(d->pipe_fd[0], &dummy, sizeof(dummy));
        if (len == -1 && errno == EAGAIN)
            break;
        if (len != sizeof(dummy)) {
            printf("%s:error reading pipe_fd, len=%d\n", __FUNCTION__, len);
            break;
        }
    }
    qxl_update_irq();
}

static void qxl_vm_change_state_handler(void *opaque, int running)
{
    PCIQXLDevice* d=(PCIQXLDevice*)opaque;

    printf("QXL: %s: running=%d\n", __FUNCTION__, running);

    if (running) {
        d->state.running = TRUE;
        qemu_set_fd_handler(d->pipe_fd[0], qxl_pipe_read, NULL, d);
        qxl_worker_start(d->state.worker);
        qemu_set_irq(d->pci_dev.irq[0], irq_level(d));
        if (qxl_vga.active_clients) {
            qemu_mod_timer(qxl_vga.timer, qemu_get_clock(rt_clock));
        }
    } else {
        qemu_del_timer(qxl_vga.timer);
        qxl_worker_stop(d->state.worker);
        qemu_set_fd_handler(d->pipe_fd[0], NULL, NULL, d);
        d->state.running = FALSE;
    }
}

void init_pipe_signaling(PCIQXLDevice *d)
{
   if (pipe(d->pipe_fd) < 0) {
       printf("%s:pipe creation failed\n", __FUNCTION__);
       return;
   }
   fcntl(d->pipe_fd[0], F_SETFL, O_NONBLOCK | O_ASYNC);
   fcntl(d->pipe_fd[1], F_SETFL, O_NONBLOCK);
   fcntl(d->pipe_fd[0], F_SETOWN, getpid());
}

static void reset_handler(void *opaque)
{
    PCIQXLDevice *d = (PCIQXLDevice *)opaque;
    if (!QXL_SHARED_VGA_MODE && d->id > 0) {
        qxl_reset(d);
    }
}

static int channel_id = 0;

void qxl_init(PCIBus *bus, uint8_t *vram, unsigned long vram_offset, 
              uint32_t vram_size)
{
    PCIQXLDevice *d;
    PCIConf *pci_conf;
    uint32_t rom_size;
    uint32_t max_fb;

    d = (PCIQXLDevice*)pci_register_device(bus, QXL_DEV_NAME,
                                           sizeof(PCIQXLDevice), -1, NULL,
                                           NULL);

    d->id = channel_id;
    d->state.mode = QXL_MODE_UNDEFINED;
    if (!channel_id) {
        qxl_init_modes();
    }

    register_savevm(QXL_DEV_NAME, channel_id, QXL_SAVE_VERSION, qxl_save, qxl_load, d);
    qemu_add_vm_change_state_handler(qxl_vm_change_state_handler, d);

    pci_conf = (PCIConf *)d->pci_dev.config;

    pci_conf->vendor_ID = QUMRANET_PCI_VENDOR_ID;
    pci_conf->device_ID = QXL_DEVICE_ID;
    pci_conf->revision = QXL_REVISION;
    pci_conf->class_base = PCI_CLASS_DISPLAY;
    if (QXL_SHARED_VGA_MODE || !d->id) {
        pci_conf->class_sub = PCI_DISPLAY_SUBCLASS_VGA;
        pci_conf->class_prog_if = PCI_DISPLAY_VGA_PROGIF;
    } else {
        pci_conf->class_sub = PCI_DISPLAY_SUBCLASS_OTHER;
        pci_conf->class_prog_if = PCI_DISPLAY_OTHER_PROGIF;
    }
    pci_conf->interrupt_pin = 1;

    memset(vram, 0xff, vram_size);

    ASSERT(vram_size > QXL_RAM_SIZE);
    rom_size = init_qxl_rom(d, vram + QXL_RAM_SIZE , vram_size - QXL_RAM_SIZE, &max_fb);
    rom_size = MAX(rom_size, TARGET_PAGE_SIZE);  
    rom_size = msb_mask(rom_size * 2 - 1);
    d->state.rom_offset = vram_offset + QXL_RAM_SIZE;
    d->state.rom_size = rom_size;

    ASSERT(QXL_RAM_SIZE + rom_size < vram_size);
    init_qxl_ram(&d->state, vram);
    d->state.ram_offset = vram_offset;
    d->state.ram_size = QXL_RAM_SIZE;

    d->state.vram = vram + rom_size + QXL_RAM_SIZE;
    d->state.vram_offset = vram_offset + rom_size + QXL_RAM_SIZE;
    d->state.vram_size = msb_mask(vram_size - (QXL_RAM_SIZE + rom_size));

    printf("%s: rom(%p, 0x%x, 0x%x) ram(%p, 0x%x, 0x%x) vram(%p, 0x%lx, 0x%x)\n",
           __FUNCTION__,
           d->state.rom,
           d->state.rom_offset,
           d->state.rom_size,
           d->state.ram_start,
           d->state.ram_offset,
           d->state.ram_size,
           d->state.vram,
           d->state.vram_offset,
           d->state.vram_size);
   
    if (d->state.vram_size < max_fb) {
        printf("%s: bad vram size, vram %u rom %u max_fb %u\n",
               __FUNCTION__, vram_size, rom_size, max_fb);
        exit(-1);
    }

    pci_register_io_region(&d->pci_dev, QXL_IO_RANGE_INDEX,
                           msb_mask(QXL_IO_RANGE_SIZE * 2 - 1),
                           PCI_ADDRESS_SPACE_IO, ioport_map);

    pci_register_io_region(&d->pci_dev, QXL_ROM_RANGE_INDEX,
                           d->state.rom_size, PCI_ADDRESS_SPACE_MEM,
                           rom_map);

    pci_register_io_region(&d->pci_dev, QXL_RAM_RANGE_INDEX,
                           d->state.ram_size, PCI_ADDRESS_SPACE_MEM,
                           ram_map);
#ifdef QXL_IO_MEM
    d->state.qxl_io_memory = cpu_register_io_memory(0, qxl_mem_read, 
                                                    qxl_mem_write, NULL);
    ASSERT(d->state.qxl_io_memory > 0);
    pci_register_io_region(&d->pci_dev, QXL_VRAM_RANGE_INDEX, d->state.vram_size,
                           PCI_ADDRESS_SPACE_MEM_PREFETCH, vram_map);
#else
    pci_register_io_region(&d->pci_dev, QXL_VRAM_RANGE_INDEX, d->state.vram_size,
                           PCI_ADDRESS_SPACE_MEM_PREFETCH, vram_map);
#endif
    d->state.worker = qxl_worker_init((QXLDevRef) d, channel_id++);
    d->dev_next = dev_list;
    dev_list = d;
    d->vga_next = qxl_vga.clients;
    qxl_vga.clients = d;
    main_thread = pthread_self();
    qxl_reset_state(d);
    if (QXL_SHARED_VGA_MODE || !d->id) {
        qxl_enter_vga_mode(d);
        qxl_worker_attach(d->state.worker);
    } 
    init_pipe_signaling(d);
    qemu_register_reset(reset_handler, d);
}


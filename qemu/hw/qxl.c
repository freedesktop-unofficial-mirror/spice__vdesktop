#include <pthread.h>
#include <signal.h>

#include "qemu-common.h"
#include "hw/hw.h"
#include "hw/pc.h"
#include "hw/pci.h"
#include "console.h"
#include "hw/vga_int.h"
#include "qemu-timer.h"
#include "sysemu.h"

#include "qxl_dev.h"

#ifdef CONFIG_SPICE    
#include "interface.h"
#endif
#include "qxl_interface.h"

//#define QXL_IO_MEM

#define QXL_VRAM_SIZE 1024 * 1024 * 128
#define QXL_DEFAULT_COMPRESSION_LEVEL 0
#define QXL_SHARED_VGA_MODE FALSE
#define QXL_SAVE_VERSION 3
#define VDI_PORT_SAVE_VERSION 1

#define ASSERT(x) if (!(x)) {                               \
	printf("%s: ASSERT %s failed\n", __FUNCTION__, #x); 	\
	exit(-1);                                               \
}

#define PANIC_ON(x) if ((x)) {                         \
    printf("%s: PANIC %s failed\n", __FUNCTION__, #x); \
    exit(-1);                                          \
}

#undef ALIGN
#define ALIGN(a, b) (((a) + ((b) - 1)) & ~((b) - 1))

#define TRUE 1
#define FALSE 0

#define QXL_DEV_NAME "qxl"
#define VDI_PORT_DEV_NAME "vdi_port"

#define PCI_CLASS_DISPLAY 0x03
#define PCI_DISPLAY_SUBCLASS_VGA 0x00
#define PCI_DISPLAY_VGA_PROGIF 0x00
#define PCI_DISPLAY_SUBCLASS_OTHER 0x80
#define PCI_DISPLAY_OTHER_PROGIF 0x80


#define PCI_CLASS_COM_CONTROLLER 0x07
#define PCI_COM_CONTROLLER_SUBCLASS_OTHER 0x80
#define PCI_COM_CONTROLLER_SUBCLASS_OTHER_PROGIF 0x00


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

#define NUM_MEMSLOTS 256
#define NUM_MEMSLOTS_GROUPS 2
#define MEMSLOT_GENERATION_BITS 8
#define MEMSLOT_SLOT_BITS 8

typedef struct MemSlot {
    int active;
    unsigned long virt_start;
    unsigned long virt_end;
    int64_t address_delta;
    uint8_t generation;
} MemSlot;

enum {
    QXL_MODE_UNDEFINED,
    QXL_MODE_VGA,
    QXL_MODE_NATIVE,
};

typedef struct QXLState {
    uint32_t io_base;
    QXLRom *rom;
    QXLModes *modes;
    uint64_t rom_offset;
    uint32_t rom_size;

    uint8_t *ram_start;
    QXLRam *ram;
    uint64_t ram_offset;
    uint32_t ram_size;
    uint64_t ram_phys_addr;

    QXLAddressRange *address_ranges;
    uint8_t num_ranges;

    MemSlot mem_slots[NUM_MEMSLOTS_GROUPS][NUM_MEMSLOTS];

    uint8_t *vram;
    unsigned long vram_offset;
    uint32_t vram_size;
#ifdef QXL_IO_MEM
    int qxl_io_memory;
#endif
    int num_free_res;
    QXLReleaseInfo *last_release;
    void *worker_data;
    int worker_data_size;
    int mode;
    QXLCommandRing vga_ring;
    uint32_t bits_unique;
    int running;
    QXLCommandRing *rings_flip[2];
    int send_event_flip[2];
    int group_ids_flip[2];
    int flip_counter;
} QXLState;

typedef struct PCIQXLDevice PCIQXLDevice;

#ifdef CONFIG_SPICE
typedef struct QXLModeNotifier QXLModeNotifier;
struct QXLModeNotifier {
    void *opaque;
    qxl_mode_change_notifier_t proc;
    PCIQXLDevice *d;
    int refs;
    QXLModeNotifier *next;
};
#endif

struct PCIQXLDevice {
    PCIDevice pci_dev;
    QXLState state;
    int id;
    DisplayState ds;
    Rect dirty_rect;
    struct PCIQXLDevice *dev_next;
    struct PCIQXLDevice *vga_next;
    int pipe_fd[2];
    QXLWorker* worker;
#ifdef CONFIG_SPICE
    QXLModeNotifier *mode_notifiers;
#endif
};

typedef struct PCIVDIPortDevice {
    PCIDevice pci_dev;
    uint32_t io_base;
    uint64_t ram_offset;
    uint32_t ram_size;
    VDIPortRam *ram;
    uint32_t connected;
    int running;
    int new_gen_on_resume;
#ifdef CONFIG_SPICE
    int active_interface;
    VDIPortInterface interface;
    VDIPortPlug *plug;
    int plug_read_pos;
#endif
} PCIVDIPortDevice;

static PCIQXLDevice *dev_list = NULL;
static pthread_t main_thread;

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
    int need_update;
} QXLVga;

static void qxl_exit_vga_mode(PCIQXLDevice *d);
static void qxl_reset_state(PCIQXLDevice *d);

static QXLVga qxl_vga;

inline uint32_t msb_mask(uint32_t val);

static inline void atomic_or(uint32_t *var, uint32_t add)
{
   __asm__ __volatile__ ("lock; orl %1, %0" : "+m" (*var) : "r" (add) : "memory");
}

static inline uint32_t atomic_exchange(uint32_t val, uint32_t *ptr)
{
   __asm__ __volatile__("xchgl %0, %1" : "+q"(val), "+m" (*ptr) : : "memory");
   return val;
}

static void qxl_init_modes(void)
{
    int i;

    for (i = 0; i < sizeof(qxl_modes) / sizeof(QXLMode); i++) {
        qxl_modes[i].id = i;
    }
}

static UINT32 qxl_max_res_area(void)
{
    UINT32 area = 0;
    int i;

    for (i = 0; i < sizeof(qxl_modes) / sizeof(QXLMode); i++) {
        area = MAX(qxl_modes[i].x_res*qxl_modes[i].y_res, area);
    }
    return area;
}

static int irq_level(PCIQXLDevice *d)
{
    return !!(d->state.ram->int_pending & d->state.ram->int_mask);
}

static void qxl_update_irq(void)
{
    PCIQXLDevice *d = dev_list;
    while (d) {
        qemu_set_irq(d->pci_dev.irq[0], irq_level(d));
        d = d->dev_next;
    }
}

void qxl_do_set_log_level(int log_level)
{
    PCIQXLDevice *d = dev_list;
    while (d) {
        d->state.rom->log_level = log_level;
        d = d->dev_next;
    }
}

uint32_t qxl_get_total_mem_size(uint32_t in_ram_size)
{
    uint32_t ram_size = msb_mask(in_ram_size*2 - 1);
    uint32_t rom_size = sizeof(QXLRom) + sizeof(QXLModes) + sizeof(qxl_modes);
    uint32_t vdi_port_ram_size = msb_mask(sizeof(VDIPortRam) * 2 - 1);

    rom_size = MAX(rom_size, TARGET_PAGE_SIZE);
    rom_size = msb_mask(rom_size*2 - 1);

    return (QXL_VRAM_SIZE + rom_size + ram_size + vdi_port_ram_size);
}

uint32_t qxl_get_min_ram_size(void)
{
    return (qxl_max_res_area() * sizeof(uint32_t) * 3);
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
        if (write(d->pipe_fd[1], d, 1) != 1) {
            printf("%s: write to pipe failed\n", __FUNCTION__);
        }
    }
}

static void _qxl_get_init_info(PCIQXLDevice *d, QXLDevInitInfo *info)
{
    info->memslot_gen_bits = MEMSLOT_GENERATION_BITS;
    info->memslot_id_bits = MEMSLOT_SLOT_BITS;
    info->num_memslots = NUM_MEMSLOTS;
    info->num_memslots_groups = NUM_MEMSLOTS_GROUPS;
    info->internal_groupslot_id = 0;
    info->qxl_ram_size = d->state.rom->num_pages << TARGET_PAGE_BITS;
    info->n_surfaces = 10000;
}

static int qxl_get_ring_command(PCIQXLDevice *d, QXLCommandExt *cmd_ext, QXLCommandRing *ring,
                                int group_id, int send_event)
{
    int notify;
    QXLCommand* cmd;
    
    if (RING_IS_EMPTY(ring)) {
        return FALSE;
    }

    RING_CONS_ITEM_SAFE(ring, ring, ring + 1, cmd)
    cmd_ext->group_id = group_id;
    cmd_ext->cmd = *cmd;
    RING_POP(ring, notify);
    if (send_event && notify) {
        qxl_send_events(d, QXL_INTERRUPT_DISPLAY);
    }
    return TRUE;
}

static int _qxl_get_command(PCIQXLDevice *d, QXLCommandExt *cmd)
{
    int x = 0;

    for (; x < 2; ++x) {
        if (qxl_get_ring_command(d, cmd, d->state.rings_flip[d->state.flip_counter % 2],
                                 d->state.group_ids_flip[d->state.flip_counter % 2],
                                 d->state.send_event_flip[d->state.flip_counter % 2])) {
            d->state.flip_counter++;
            return TRUE;
        }
        d->state.flip_counter++;
    }
    return FALSE;
}

static int _qxl_has_command(PCIQXLDevice *d)
{
    return !RING_IS_EMPTY(d->state.rings_flip[0]) | !RING_IS_EMPTY(d->state.rings_flip[1]);
}

static int _qxl_get_cursor_command(PCIQXLDevice *d, QXLCommandExt *cmd_ext)
{
    QXLCursorRing *ring;
    QXLCommand* cmd;
    int notify;

    if (d->state.mode == QXL_MODE_VGA) {
        return 0;
    }

    ring = &d->state.ram->cursor_ring;

    if (RING_IS_EMPTY(ring)) {
        return 0;
    }
    RING_CONS_ITEM_SAFE(ring, ring, ring + 1, cmd);
    cmd_ext->group_id = 1;
    cmd_ext->cmd = *cmd;
    RING_POP(ring, notify);
    if (notify) {
        qxl_send_events(d, QXL_INTERRUPT_CURSOR);
    }
    return 1;
}

static void _qxl_get_update_area(PCIQXLDevice *d, const Rect **rect, UINT32 **surface_id)
{
    *rect = &d->state.ram->update_area;
    *surface_id = &d->state.ram->update_surface;
}

static int _qxl_req_cmd_notification(PCIQXLDevice *d)
{
    int wait;

    RING_CONS_WAIT(&d->state.ram->cmd_ring, wait);
    if (wait) {
        RING_CONS_WAIT(&d->state.vga_ring, wait);
        return wait;
    }
    return 0;
}

static int _qxl_req_cursor_notification(PCIQXLDevice *d)
{
    int wait;

    RING_CONS_WAIT(&d->state.ram->cursor_ring, wait);
    return wait;
}

#define QXL_FREE_BUNCH_SIZE 10

static inline void qxl_push_free_res(PCIQXLDevice *d)
{
    QXLState *state = &d->state;
    QXLReleaseRing *ring = &state->ram->release_ring;
    UINT64 *item;

    if (RING_IS_EMPTY(ring) || (state->num_free_res == QXL_FREE_BUNCH_SIZE &&
                                ring->prod - ring->cons + 1 != ring->num_items)) {
        int notify;

        RING_PUSH(ring, notify);
        if (notify) {
            qxl_send_events(d, QXL_INTERRUPT_DISPLAY);
        }
        RING_PROD_ITEM_SAFE(ring, ring, ring + 1, item);
        *item = 0;
        state->num_free_res = 0;
        state->last_release = NULL;
    }
}

static void _qxl_release_resource(PCIQXLDevice *d, QXLReleaseInfoExt *release_info_ext)
{
    QXLReleaseInfo *release_info = release_info_ext->info;
    UINT64 id = release_info->id;
    QXLState *state = &d->state;
    QXLReleaseRing *ring;
    UINT64 *item;

    if (release_info_ext->group_id == 0) {
        free((void *)id);
        return;
    }
    ring = &state->ram->release_ring;
    RING_PROD_ITEM_SAFE(ring, ring, ring + 1, item);
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

static void _qxl_set_save_data(PCIQXLDevice *d, void *data, int size)
{
    QXLState *state = &d->state;

    free(state->worker_data);
    state->worker_data = data;
    state->worker_data_size = size;
}

static void *_qxl_get_save_data(PCIQXLDevice *d)
{
    QXLState *state = &d->state;
    return state->worker_data;
}

static int _qxl_flush_resources(PCIQXLDevice *d)
{
    int ret;

    ret = d->state.num_free_res; 
    if (ret) {
        qxl_push_free_res(d);
    }
    return ret;
}

static void _qxl_notify_update(PCIQXLDevice *d, uint32_t update_id)
{
    d->state.rom->update_id = update_id;
    qxl_send_events(d, QXL_INTERRUPT_DISPLAY);
}

static void qxl_release_commands(PCIQXLDevice *d)
{
    QXLCommandRing *ring;

    if (d->state.mode != QXL_MODE_VGA) {
        RING_INIT(&d->state.ram->cmd_ring);
        RING_INIT(&d->state.ram->cursor_ring);
        return;
    }
    
    ring = &d->state.vga_ring;
        
    while (!RING_IS_EMPTY(ring)) {
        QXLCommand *cmd;
        int notify;
        
        RING_CONS_ITEM_SAFE(ring, ring, ring + 1, cmd);
        RING_POP(ring, notify);
        switch (cmd->type) {
        case QXL_CMD_DRAW: {
            QXLDrawable *drawable;
            drawable = (QXLDrawable *)cmd->data;
            free(drawable);
            break;
        }
        default:
            printf("%s: wrong type\n", __FUNCTION__);
            abort();
        }
    }

    RING_INIT(&d->state.ram->cursor_ring);
}

static void qxl_create_host_memslot(PCIQXLDevice *d)
{
    MemSlot *device_slot;
    QXLDevMemSlot qxl_dev_slot;

    printf("%s\n", __FUNCTION__);

    PANIC_ON(d->state.mem_slots[0][0].active);

    device_slot = &d->state.mem_slots[0][0];
    device_slot->active = TRUE;
    device_slot->address_delta = 0;
    device_slot->virt_start = 0;
    device_slot->virt_end = ~(unsigned long)0;

    qxl_dev_slot.slot_group_id = 0;
    qxl_dev_slot.slot_id = 0;
    qxl_dev_slot.addr_delta = device_slot->address_delta;
    qxl_dev_slot.virt_start = device_slot->virt_start;
    qxl_dev_slot.virt_end = device_slot->virt_end;
    qxl_dev_slot.generation = device_slot->generation = 0;

    d->worker->add_memslot(d->worker, &qxl_dev_slot);
}

static void qxl_init_memslots(PCIQXLDevice *d)
{
    int x;
    int i;

    for (x = 0; x < NUM_MEMSLOTS_GROUPS; ++x) {
        for (i = 1; i < NUM_MEMSLOTS; ++i) {
            d->state.mem_slots[x][i].generation = 1;
        }
    }
}

static void qxl_reset_memslots(PCIQXLDevice *d)
{
    int i;
    int x;

    for (x = 0; x < NUM_MEMSLOTS_GROUPS; ++x) {
        for (i = 0; i < NUM_MEMSLOTS; ++i) {
            d->state.mem_slots[x][i].active = FALSE;
        }
    }

    d->worker->reset_memslots(d->worker);
}

static void qxl_add_vga_client(void)
{
    if (qxl_vga.active_clients++ == 0) {
        qemu_mod_timer(qxl_vga.timer, qemu_get_clock(rt_clock));
    }
}

static void qxl_remove_vga_client(void)
{
    qxl_vga.active_clients--;
}

static void qxl_create_primary_surface(PCIQXLDevice *d)
{
    QXLDevSurfaceCreate surface;

    printf("%s\n", __FUNCTION__);

    surface.depth = 32;
    surface.width = qxl_vga.ds->width;
    surface.height = qxl_vga.ds->height;
    surface.stride = -qxl_vga.ds->width * 4;
    surface.mouse_mode = 0;
    surface.flags = 0;
    surface.type = 0;
    surface.mem = (PHYSICAL)d->state.ram_start;
    surface.group_id = 0;

    d->worker->create_primary_surface(d->worker, 0, &surface);
}

static void qxl_destroy_primary_surface(PCIQXLDevice *d)
{
    printf("%s\n", __FUNCTION__);

    d->worker->destroy_primary_surface(d->worker, 0);
}

static void qxl_enter_vga_mode(PCIQXLDevice *d)
{
    qxl_create_primary_surface(d);

    if (d->state.mode == QXL_MODE_VGA || (!QXL_SHARED_VGA_MODE && d->id)) {
        return;
    }
    printf("%u: %s\n", d->id, __FUNCTION__);
    d->state.mode = QXL_MODE_VGA;
    memset(&d->dirty_rect, 0, sizeof(d->dirty_rect));
    qxl_add_vga_client();
}

static void qxl_destroy_primary(PCIQXLDevice *d)
{
    ASSERT(d->state.mode == QXL_MODE_NATIVE);
    d->state.mode = QXL_MODE_UNDEFINED;

    printf("%s\n", __FUNCTION__);

    d->worker->destroy_primary_surface(d->worker, 0);
}

static void qxl_create_primary(PCIQXLDevice *d)
{
    QXLDevSurfaceCreate surface;

    printf("%s\n", __FUNCTION__);

    ASSERT(d->state.mode != QXL_MODE_NATIVE);
    qxl_exit_vga_mode(d);
    d->state.mode = QXL_MODE_NATIVE;

    surface.depth = d->state.ram->create_surface.depth;
    surface.height = d->state.ram->create_surface.height;
    surface.mem = d->state.ram->create_surface.mem;
    surface.mouse_mode = TRUE;
    surface.position = d->state.ram->create_surface.position;
    surface.stride = d->state.ram->create_surface.stride;
    surface.width = d->state.ram->create_surface.width;
    surface.type = d->state.ram->create_surface.type;
    surface.flags = d->state.ram->create_surface.flags;
    surface.group_id = 1;
    
    d->worker->create_primary_surface(d->worker, 0, &surface);
}

static void qxl_check_state(PCIQXLDevice *d)
{
    QXLRam *ram = d->state.ram;

    ASSERT(RING_IS_EMPTY(&ram->cmd_ring));
    ASSERT(RING_IS_EMPTY(&ram->cursor_ring));
    ASSERT(RING_IS_EMPTY(&d->state.vga_ring));
}

static void qxl_reset_state(PCIQXLDevice *d)
{
    QXLRam *ram = d->state.ram;
    QXLRom *rom = d->state.rom;
    UINT64 *item;

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
    RING_PROD_ITEM_SAFE(&ram->release_ring, &ram->release_ring, &ram->release_ring + 1, item);
    *item = 0;
    d->state.num_free_res = 0;
    d->state.last_release = NULL;
    qemu_set_irq(d->pci_dev.irq[0], irq_level(d));
}

static void qxl_reset(PCIQXLDevice *d)
{
    printf("%s\n", __FUNCTION__);
    qxl_check_state(d);

    if (QXL_SHARED_VGA_MODE || !d->id) {
        qxl_enter_vga_mode(d);
    } else {
        d->state.mode = QXL_MODE_UNDEFINED;
    }
}

static void qxl_hard_reset(PCIQXLDevice *d)
{
    printf("%s\n", __FUNCTION__);

    qxl_release_commands(d);
    d->worker->destroy_surfaces(d->worker);
    d->worker->reset_cursor(d->worker);
    d->worker->reset_image_cache(d->worker);
    qxl_reset_memslots(d);
    qxl_create_host_memslot(d);
    qxl_reset_state(d);
    qxl_reset(d);
}

static inline void vdi_port_set_dirty(PCIVDIPortDevice *d, void *start, uint32_t length)
{
    ram_addr_t addr =  (ram_addr_t)start - (ram_addr_t)d->ram + d->ram_offset;
    ram_addr_t end =  ALIGN(addr + length, TARGET_PAGE_SIZE);

    do {
        cpu_physical_memory_set_dirty(addr);
        addr += TARGET_PAGE_SIZE;
    } while ( addr < end );
}

static void vdi_port_new_gen(PCIVDIPortDevice *d)
{
    d->ram->generation = (d->ram->generation + 1 == 0) ? 1 : d->ram->generation + 1;
    vdi_port_set_dirty(d, &d->ram->generation, sizeof(d->ram->generation));
}

static int vdi_port_irq_level(PCIVDIPortDevice *d)
{
    return !!(d->ram->int_pending & d->ram->int_mask);
}

static void vdi_port_notify_guest(PCIVDIPortDevice *d)
{
    uint32_t events = VDI_PORT_INTERRUPT;

    if (!d->connected) {
        return;
    }

    mb();
    if ((d->ram->int_pending & events) == events) {
        return;
    }
    atomic_or(&d->ram->int_pending, events);
    qemu_set_irq(d->pci_dev.irq[0], vdi_port_irq_level(d));
    vdi_port_set_dirty(d, &d->ram->int_pending, sizeof(d->ram->int_pending));
}

#ifdef CONFIG_SPICE

static VDObjectRef vdi_port_interface_plug(VDIPortInterface *port, VDIPortPlug* plug)
{
    PCIVDIPortDevice *d = container_of(port, PCIVDIPortDevice, interface);

    if (d->plug) {
        return INVALID_VD_OBJECT_REF;
    }
    d->plug = plug;
    return (VDObjectRef)plug;
}

static void vdi_port_interface_unplug(VDIPortInterface *port, VDObjectRef plug)
{
    PCIVDIPortDevice *d = container_of(port, PCIVDIPortDevice, interface);
    if (!plug || plug != (VDObjectRef)d->plug) {
        return;
    }
    d->plug = NULL;
    d->plug_read_pos = 0;
    if (!d->running) {
        d->new_gen_on_resume = TRUE;
        return;
    }
    vdi_port_new_gen(d);
    vdi_port_notify_guest(d);
}

static int vdi_port_interface_write(VDIPortInterface *port, VDObjectRef plug,
                                    const uint8_t *buf, int len)
{
    PCIVDIPortDevice *d = container_of(port, PCIVDIPortDevice, interface);
    VDIPortRing *ring = &d->ram->output;
    int do_notify = FALSE;
    int actual_write = 0;

    if (!d->running) {
        return 0;
    }

    while (len) {
        VDIPortPacket *packet;
        int notify;
        int wait;

        RING_PROD_WAIT(ring, wait);
        if (wait) {
            break;
        }

        RING_PROD_ITEM_SAFE(ring, ring, ring + 1, packet);
        packet->gen = d->ram->generation;
        packet->size = MIN(len, sizeof(packet->data));
        memcpy(packet->data, buf, packet->size);
        vdi_port_set_dirty(d, packet, sizeof(*packet) - (sizeof(packet->data) - packet->size));

        RING_PUSH(ring, notify);
        do_notify = do_notify || notify;   
        len -= packet->size;
        buf += packet->size;
        actual_write += packet->size;
    }
    vdi_port_set_dirty(d, ring, sizeof(*ring) - sizeof(ring->items));

    if (do_notify) {
        vdi_port_notify_guest(d);
    }
    return actual_write;
}

static int vdi_port_interface_read(VDIPortInterface *port, VDObjectRef plug,
                                   uint8_t *buf, int len)
{
    PCIVDIPortDevice *d = container_of(port, PCIVDIPortDevice, interface);
    VDIPortRing *ring = &d->ram->input;
    uint32_t gen = d->ram->generation;
    VDIPortPacket *packet;
    int do_notify = FALSE;
    int actual_read = 0;

    if (!d->running) {
        return 0;
    }

    while (!RING_IS_EMPTY(ring)) {
        int notify;

        RING_CONS_ITEM_SAFE(ring, ring, ring + 1, packet);
        if (packet->gen == gen) {
            break;
        }

        RING_POP(ring, notify);
        do_notify = do_notify || notify;
    }
    while (len) {
        int wait;
        int now;

        RING_CONS_WAIT(ring, wait);

        if (wait) {
            break;
        }

        RING_CONS_ITEM_SAFE(ring, ring, ring + 1, packet);
        if (packet->size > sizeof(packet->data)) {
            vdi_port_set_dirty(d, ring, sizeof(*ring) - sizeof(ring->items));
            printf("%s: bad packet size\n", __FUNCTION__);
            return 0;
        }
        now = MIN(len, packet->size - d->plug_read_pos);
        memcpy(buf, packet->data + d->plug_read_pos, now);
        len -= now;
        buf += now;
        actual_read +=  now;
        if ((d->plug_read_pos += now) == packet->size) {
            int notify;

            d->plug_read_pos = 0;
            RING_POP(ring, notify);
            do_notify = do_notify || notify;
        }
    }
    vdi_port_set_dirty(d, ring, sizeof(*ring) - sizeof(ring->items));

    if (do_notify) {
        vdi_port_notify_guest(d);
    }
    return actual_read;
}

static void vdi_port_register_interface(PCIVDIPortDevice *d)
{
    VDIPortInterface *interface = &d->interface;
    static int interface_id = 0;

    if (d->active_interface ) {
        return;
    }
    
    interface->base.base_version = VM_INTERFACE_VERSION;
    interface->base.type = VD_INTERFACE_VDI_PORT;
    interface->base.id = ++interface_id;
    interface->base.description = "vdi port";
    interface->base.major_version = VD_INTERFACE_VDI_PORT_MAJOR;
    interface->base.minor_version = VD_INTERFACE_VDI_PORT_MINOR;

    interface->plug = vdi_port_interface_plug;
    interface->unplug = vdi_port_interface_unplug;
    interface->write = vdi_port_interface_write;
    interface->read = vdi_port_interface_read;

    d->active_interface = TRUE;
    add_interface(&interface->base);
}

static void vdi_port_unregister_interface(PCIVDIPortDevice *d)
{
    if (!d->active_interface ) {
        return;
    }
    d->active_interface = FALSE;
    remove_interface(&d->interface.base);
}

#endif

static uint32_t vdi_port_dev_connect(PCIVDIPortDevice *d)
{
    if (d->connected) {
        printf("%s: already connected\n", __FUNCTION__);
        return 0;
    }
    vdi_port_new_gen(d);
    d->connected = TRUE;
#ifdef CONFIG_SPICE
    vdi_port_register_interface(d);
#endif
    return d->ram->generation;
}

static void vdi_port_dev_disconnect(PCIVDIPortDevice *d)
{
    if (!d->connected) {
        printf("%s: not connected\n", __FUNCTION__);
        return;
    }
    d->connected = FALSE;
#ifdef CONFIG_SPICE
    vdi_port_unregister_interface(d);
#endif
}

static void vdi_port_dev_notify(PCIVDIPortDevice *d)
{
#ifdef CONFIG_SPICE
    if (!d->plug) {
        return;
    }
    d->plug->wakeup(d->plug);
#endif
}

static inline int in_range_start(unsigned long start, unsigned long range_start,
                                 unsigned long range_end) {
    if (start >= range_start && start < range_end) {
        return TRUE;
    }

    return FALSE;
}

static inline int in_range_end(unsigned long end, unsigned long range_start,
                               unsigned long range_end) {
    if (end > range_start && end <= range_end) {
        return TRUE;
    }

    return FALSE;
}

static inline int address_in_range(unsigned long start, unsigned long end, QXLAddressRange *range)
{
    if (in_range_start(start, range->phys_start, range->phys_end) &&
        in_range_end(end, start, range->phys_end)) {
        return TRUE;
    }
    return FALSE;
}

static inline int is_intersecting_memslot(MemSlot *slot, unsigned long vstart, unsigned long vend)
{
    return vstart < slot->virt_end && vend > slot->virt_start;
}

static void qxl_check_memslot_range(PCIQXLDevice *d, unsigned long virt_start,
                                    unsigned long virt_end)
{
    int x;
    int i;
    MemSlot *slot;

    for (i = 0; i < NUM_MEMSLOTS_GROUPS; ++i) {
        for (x = 1; x < NUM_MEMSLOTS; ++x) {
            slot = &d->state.mem_slots[i][x];

            if (slot->active) {
                PANIC_ON(is_intersecting_memslot(slot, virt_start, virt_end));
            }
        }
    }
}

static void qxl_add_memslot(PCIQXLDevice *d, uint32_t slot_id)
{
    QXLMemSlot *slot;
    QXLDevMemSlot qxl_dev_slot;
    MemSlot *device_slot;
    uint64_t start;
    uint64_t end;
    uint64_t start_offset;
    uint64_t end_offset; 
    unsigned long new_virt_start;
    unsigned long new_virt_end;
    unsigned long ram_phys_addr;
    unsigned long ram_start = 0;
    int i;

    slot = &d->state.ram->mem_slot;
    start = slot->mem_start;
    end = slot->mem_end;

    printf("%s: slot %d start: 0x%lx end: 0x%lx\n", __FUNCTION__, slot_id, start, end);
    
    PANIC_ON(slot_id >= NUM_MEMSLOTS);
    PANIC_ON(d->state.mem_slots[1][slot_id].active);

    for (i = 0; i < d->state.num_ranges; ++i) {
        if (address_in_range(start, end, &d->state.address_ranges[i])) {
            ram_phys_addr = d->state.address_ranges[i].phys_start;
            ram_start = d->state.address_ranges[i].virt_start;
            break;
        }
    }

    if (!ram_start) {
        printf("%s: invalid address range\n", __FUNCTION__);
        exit(-1);
    }

    start_offset = start - ram_phys_addr;
    end_offset = end - ram_phys_addr;

    new_virt_start = ram_start + start_offset;
    new_virt_end = ram_start + end_offset;

    qxl_check_memslot_range(d, new_virt_start, new_virt_end);

    device_slot = &d->state.mem_slots[1][slot_id];
    device_slot->active = TRUE;
    device_slot->address_delta = ram_start + start - ram_phys_addr;
    device_slot->virt_start = new_virt_start;
    device_slot->virt_end = new_virt_end;

    qxl_dev_slot.slot_id = slot_id;
    qxl_dev_slot.addr_delta = device_slot->address_delta;
    qxl_dev_slot.virt_start = device_slot->virt_start;
    qxl_dev_slot.virt_end = device_slot->virt_end;
    qxl_dev_slot.generation = d->state.rom->slot_generation = device_slot->generation++;
    qxl_dev_slot.slot_group_id = 1;

    d->worker->add_memslot(d->worker, &qxl_dev_slot);
}

static void qxl_del_memslot(PCIQXLDevice *d, uint32_t slot_id)
{
    MemSlot *device_slot;

    printf("%s: slot %d\n", __FUNCTION__, slot_id);

    PANIC_ON(slot_id >= NUM_MEMSLOTS);
    device_slot = &d->state.mem_slots[1][slot_id];
    PANIC_ON(!device_slot->active);
    device_slot->active = FALSE;

    d->worker->del_memslot(d->worker, 1, slot_id);
}

static inline int is_any_none_host_slot_active(PCIQXLDevice *d)
{
    int i;
    int x;

    for (x = 1; x < NUM_MEMSLOTS_GROUPS; ++x) {
        for (i = 0; i < NUM_MEMSLOTS; ++i) {
            if (d->state.mem_slots[x][i].active) {
                return TRUE;
            }
        }
    }
    return FALSE;
}

static void ioport_write(void *opaque, uint32_t addr, uint32_t val)
{
    PCIQXLDevice *d = (PCIQXLDevice *)opaque;
    uint32_t io_port = addr - d->state.io_base;
#ifdef DEBUG_QXL
    printf("%s: addr 0x%x val 0x%x\n", __FUNCTION__, addr, val);
#endif
    if (d->state.mode != QXL_MODE_NATIVE && io_port != QXL_IO_RESET &&
        io_port != QXL_IO_MEMSLOT_ADD && io_port != QXL_IO_MEMSLOT_DEL &&
        io_port !=  QXL_IO_CREATE_PRIMARY) {
        printf("%s: unexpected port 0x%x in vga mode\n", __FUNCTION__, io_port);
        PANIC_ON(io_port == QXL_IO_DESTROY_PRIMARY);
        return;
    }
    switch (io_port) {
    case QXL_IO_UPDATE_AREA:
        d->worker->update_area(d->worker);
        break;
    case QXL_IO_NOTIFY_CMD:
        d->worker->wakeup(d->worker);
        break;
    case QXL_IO_NOTIFY_CURSOR:
        d->worker->wakeup(d->worker);
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
        d->worker->oom(d->worker);
        break;
    case QXL_IO_LOG:
        printf("%u: %s", d->id, d->state.ram->log_buf);
        break;
    case QXL_IO_RESET:
        printf("%u: QXL_IO_RESET\n", d->id);
        qxl_hard_reset(d);
        break;
    case QXL_IO_MEMSLOT_ADD:
        qxl_add_memslot(d, val);
        break;
    case QXL_IO_MEMSLOT_DEL:
        qxl_del_memslot(d, val);
        break;
    case QXL_IO_CREATE_PRIMARY:
        PANIC_ON(val != 0);
        qxl_create_primary(d);
        break;
    case QXL_IO_DESTROY_PRIMARY:
        PANIC_ON(val != 0);
        qxl_destroy_primary(d);
        break;
    case QXL_IO_DESTROY_SURFACE_WAIT:
        d->worker->destroy_surface_wait(d->worker, val);
        break;
    case QXL_IO_DESTROY_ALL_SURFACES:
        d->worker->destroy_surfaces(d->worker);
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
    PANIC_ON(is_any_none_host_slot_active((PCIQXLDevice *)d));
    s->ram_phys_addr = addr;
    cpu_register_physical_memory(addr, size, s->ram_offset | IO_MEM_RAM);

    s->address_ranges[0].virt_start = (unsigned long)s->ram_start;
    s->address_ranges[0].virt_end = s->address_ranges[0].virt_start + size;
    s->address_ranges[0].phys_start = addr;
    s->address_ranges[0].phys_end = s->address_ranges[0].phys_start + size;
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
    s->address_ranges[1].virt_start = (unsigned long)s->vram;
    s->address_ranges[1].virt_end = s->address_ranges[1].virt_start + size;
    s->address_ranges[1].phys_start = addr;
    s->address_ranges[1].phys_end = s->address_ranges[1].phys_start + size;
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
    rom->modes_offset = sizeof(QXLRom);
    rom->surface0_area_size = qxl_max_res_area()* sizeof(uint32_t);
    rom->compression_level = QXL_DEFAULT_COMPRESSION_LEVEL;
    rom->log_level = 0;

    rom->slot_gen_bits = MEMSLOT_GENERATION_BITS;
    rom->slot_id_bits = MEMSLOT_SLOT_BITS;
    rom->slots_start = 1;
    rom->slots_end = NUM_MEMSLOTS - 1;

    rom->n_surfaces = 10000;

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

static void init_qxl_ram(QXLState *s, uint8_t *buf, uint32_t actual_ram_size)
{
    uint32_t surface0_area_size;
    uint32_t ram_header_size;
    uint64_t *item;

    s->ram_start = buf;

    surface0_area_size = ALIGN(s->rom->surface0_area_size, (1 << TARGET_PAGE_BITS));
    ram_header_size = ALIGN(sizeof(*s->ram), 8);
    ASSERT(ram_header_size + surface0_area_size < actual_ram_size);

    s->rom->ram_header_offset = actual_ram_size - ram_header_size;
    s->ram = (QXLRam *)(buf + s->rom->ram_header_offset);
    s->ram->magic = QXL_RAM_MAGIC;
    RING_INIT(&s->ram->cmd_ring);
    RING_INIT(&s->ram->cursor_ring);
    RING_INIT(&s->ram->release_ring);
    RING_PROD_ITEM_SAFE(&s->ram->release_ring, &s->ram->release_ring, &s->ram->release_ring + 1,
                        item);
    *item = 0;
    s->rom->num_pages = (actual_ram_size - (surface0_area_size + ram_header_size)) >>
                        TARGET_PAGE_BITS;

    printf("%s: npages %u\n", __FUNCTION__, s->rom->num_pages);
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

static int rect_is_empty(const Rect* r)
{
    return r->top == r->bottom || r->left == r->right;
}

static void rect_union(Rect *dest, const Rect *r)
{
    if (rect_is_empty(r)) {
        return;
    }

    if (rect_is_empty(dest)) {
        *dest = *r;
        return;
    }

    dest->top = MIN(dest->top, r->top);
    dest->left = MIN(dest->left, r->left);
    dest->bottom = MAX(dest->bottom, r->bottom);
    dest->right = MAX(dest->right, r->right);
}

static void qxl_display_update(struct DisplayState *ds, int x, int y, int w, int h)
{
    PCIQXLDevice *client;
    Rect update_area;

    qxl_vga.need_update = TRUE;

    update_area.left = x,
    update_area.right = x + w;
    update_area.top = y;
    update_area.bottom = y + h;

    for (client = qxl_vga.clients; client; client = client->vga_next) {
        if (client->state.mode == QXL_MODE_VGA && client->state.running) {
            rect_union(&client->dirty_rect, &update_area);
        }
    }
}

static void qxl_vga_update(void)
{
    PCIQXLDevice *client;

    qxl_vga.need_update = FALSE;

    for (client = qxl_vga.clients; client; client = client->vga_next) {
        if (client->state.mode == QXL_MODE_VGA && client->state.running) {
            QXLDrawable *drawable;
            QXLImage *image;
            QXLCommandRing *ring;
            QXLCommand *cmd;
            int wait;
            int notify;
            Rect *dirty_rect = &client->dirty_rect;

            if (rect_is_empty(dirty_rect)) {
                continue;
            }

            ring = &client->state.vga_ring;
            RING_PROD_WAIT(ring, wait);
            if (wait) {
                qxl_vga.need_update = TRUE;
                continue;
            }

            drawable = (QXLDrawable *)malloc(sizeof(*drawable) + sizeof(*image));
            if (!drawable) {
                printf("%s: alloc drawable failed\n", __FUNCTION__);
                abort();
            }

            image = (QXLImage *)(drawable + 1);
            drawable->surface_id = 0;
            drawable->surfaces_dest[0] = -1;
            drawable->surfaces_dest[1] = -1;
            drawable->surfaces_dest[2] = -1;
            drawable->bbox = *dirty_rect;
            drawable->clip.type = CLIP_TYPE_NONE;
            drawable->clip.data = 0;
            drawable->effect = QXL_EFFECT_OPAQUE;
            drawable->release_info.id = (UINT64)drawable;
            drawable->self_bitmap = 0;
            drawable->type = QXL_DRAW_COPY;

            drawable->u.copy.rop_decriptor =  ROPD_OP_PUT;
            drawable->u.copy.src_bitmap = (PHYSICAL)image;
            drawable->u.copy.src_area.left = drawable->u.copy.src_area.top = 0;
            drawable->u.copy.src_area.right = dirty_rect->right - dirty_rect->left;
            drawable->u.copy.src_area.bottom = dirty_rect->bottom - dirty_rect->top;
            drawable->u.copy.scale_mode = 0;
            memset(&drawable->u.copy.mask, 0, sizeof(QMask));

            image->descriptor.type = IMAGE_TYPE_BITMAP;
            image->descriptor.flags = 0;
            QXL_SET_IMAGE_ID(image, QXL_IMAGE_GROUP_DEVICE, ++client->state.bits_unique);
            image->bitmap.flags = QXL_BITMAP_DIRECT | QXL_BITMAP_TOP_DOWN | QXL_BITMAP_UNSTABLE;
            image->bitmap.format = BITMAP_FMT_32BIT;
            image->bitmap.stride = qxl_vga.ds->linesize;
            image->descriptor.width = image->bitmap.x = drawable->u.copy.src_area.right;
            image->descriptor.height = image->bitmap.y = drawable->u.copy.src_area.bottom;
            image->bitmap.data = (PHYSICAL)(qxl_vga.ds->data + dirty_rect->top * qxl_vga.ds->linesize + dirty_rect->left * 4);
            image->bitmap.palette = 0;

            RING_PROD_ITEM_SAFE(ring, ring, ring + 1, cmd);
            cmd->type = QXL_CMD_DRAW;
            cmd->data = (PHYSICAL)drawable;
            RING_PUSH(ring, notify);
            if (notify) {
                client->worker->wakeup(client->worker);
            }
            memset(dirty_rect, 0, sizeof(*dirty_rect));
        }
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
            qxl_destroy_primary_surface(client);
            qxl_create_primary_surface(client);
        }
        client = client->vga_next;
    }
    free(data);
}

static void qxl_display_refresh(struct DisplayState *ds)
{
    if (qxl_vga.active_clients) {
        vga_hw_update();
        if (qxl_vga.need_update) {
            qxl_vga_update();
        }
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
    qxl_destroy_primary_surface(d);
    qxl_remove_vga_client();
    d->state.mode = QXL_MODE_UNDEFINED;
}

int qxl_vga_touch(void)
{
    PCIQXLDevice *d = qxl_vga.clients;
    int ret = FALSE;
    while (d) {
        if (d->state.mode != QXL_MODE_VGA && (QXL_SHARED_VGA_MODE || !d->id)) {
            if (d->state.mode == QXL_MODE_NATIVE) {
                qxl_destroy_primary(d);
            }
            qxl_reset(d);
            ret = TRUE;
        }
        d = d->vga_next;
    }
    return ret;
}

//todo: use cpu_physical_memory_set_dirty instead of manual copy
static void qxl_save(QEMUFile* f, void* opaque)
{
    PCIQXLDevice* d=(PCIQXLDevice*)opaque;
    QXLState*     s=&d->state;
    uint32_t      last_release_offset;

    d->worker->save(d->worker);
    pci_device_save(&d->pci_dev, f);

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


    if (s->mode == QXL_MODE_VGA) {
        qxl_remove_vga_client();
    }

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

    if (running == d->state.running) {
        return;
    }

    if (running) {
        d->state.running = TRUE;
        qemu_set_fd_handler(d->pipe_fd[0], qxl_pipe_read, NULL, d);
        d->worker->start(d->worker);
        qemu_set_irq(d->pci_dev.irq[0], irq_level(d));
        if (qxl_vga.active_clients) {
            qemu_mod_timer(qxl_vga.timer, qemu_get_clock(rt_clock));
        }
    } else {
        qemu_del_timer(qxl_vga.timer);
        d->worker->stop(d->worker);
        qemu_set_fd_handler(d->pipe_fd[0], NULL, NULL, d);
        d->state.running = FALSE;
    }
}

static void init_pipe_signaling(PCIQXLDevice *d)
{
   if (pipe(d->pipe_fd) < 0) {
       printf("%s:pipe creation failed\n", __FUNCTION__);
       return;
   }
   fcntl(d->pipe_fd[0], F_SETFL, O_NONBLOCK | O_ASYNC);
   fcntl(d->pipe_fd[1], F_SETFL, O_NONBLOCK);
   fcntl(d->pipe_fd[0], F_SETOWN, getpid());
}

static void qxl_reset_device_address_ranges(PCIQXLDevice *d)
{
    memset(&d->state.address_ranges[0], 0, sizeof(QXLAddressRange));
}

static void reset_handler(void *opaque)
{
    PCIQXLDevice *d = (PCIQXLDevice *)opaque;

    qxl_hard_reset(d);
    qxl_reset_device_address_ranges(d);
}

void qxl_get_init_info(QXLDevRef dev_ref, QXLDevInitInfo *info)
{
    _qxl_get_init_info((PCIQXLDevice *)dev_ref, info);
}

int qxl_get_command(QXLDevRef dev_ref, struct QXLCommandExt *cmd)
{
    return _qxl_get_command((PCIQXLDevice *)dev_ref, cmd);
}

void qxl_release_resource(QXLDevRef dev_ref, QXLReleaseInfoExt release_info_ext)
{
    _qxl_release_resource((PCIQXLDevice *)dev_ref, &release_info_ext);
}

void qxl_notify_update(QXLDevRef dev_ref, uint32_t update_id)
{
    _qxl_notify_update((PCIQXLDevice *)dev_ref, update_id);
}

int qxl_req_cmd_notification(QXLDevRef dev_ref)
{
    return _qxl_req_cmd_notification((PCIQXLDevice *)dev_ref);
}

int qxl_get_cursor_command(QXLDevRef dev_ref, struct QXLCommandExt *cmd)
{
    return _qxl_get_cursor_command((PCIQXLDevice *)dev_ref, cmd);
}

int qxl_req_cursor_notification(QXLDevRef dev_ref)
{
    return _qxl_req_cursor_notification((PCIQXLDevice *)dev_ref);
}

int qxl_has_command(QXLDevRef dev_ref)
{
    return _qxl_has_command((PCIQXLDevice *)dev_ref);
}

void qxl_get_update_area(QXLDevRef dev_ref, const Rect **rect, UINT32 **surface_id)
{
    _qxl_get_update_area((PCIQXLDevice *)dev_ref, rect, surface_id);
}

int qxl_flush_resources(QXLDevRef dev_ref)
{
    return _qxl_flush_resources((PCIQXLDevice *)dev_ref);
}

void qxl_set_save_data(QXLDevRef dev_ref, void *data, int size)
{
    _qxl_set_save_data((PCIQXLDevice *)dev_ref, data, size);
}

void *qxl_get_save_data(QXLDevRef dev_ref)
{
    return _qxl_get_save_data((PCIQXLDevice *)dev_ref);
}

#ifdef CONFIG_SPICE

typedef struct Interface {
    QXLInterface vd_interface;
    PCIQXLDevice *d;
} Interface;

static void interface_attache_worker(QXLInterface *qxl, QXLWorker *qxl_worker)
{
    Interface *interface = (Interface *)qxl;
    if (interface->d->worker) {
        printf("%s: has worker\n", __FUNCTION__);
        exit(-1);
    }
    interface->d->worker = qxl_worker;
}

static void interface_set_compression_level(QXLInterface *qxl, int level)
{
    PCIQXLDevice *d = ((Interface *)qxl)->d;
    d->state.rom->compression_level = level;
}

static void interface_set_mm_time(QXLInterface *qxl, uint32_t mm_time)
{
    PCIQXLDevice *d = ((Interface *)qxl)->d;
    d->state.rom->mm_clock = mm_time;
}

static void interface_get_init_info(QXLInterface *qxl, QXLDevInitInfo *info)
{
    _qxl_get_init_info(((Interface *)qxl)->d, info);
}

static int interface_get_command(QXLInterface *qxl, struct QXLCommandExt *cmd)
{
    return _qxl_get_command(((Interface *)qxl)->d, cmd);
}

static int interface_req_cmd_notification(QXLInterface *qxl)
{
    return _qxl_req_cmd_notification(((Interface *)qxl)->d);
}

static int interface_has_command(QXLInterface *qxl)
{
    return _qxl_has_command(((Interface *)qxl)->d);
}

static void interface_release_resource(QXLInterface *qxl, QXLReleaseInfoExt release_info_ext)
{
    _qxl_release_resource(((Interface *)qxl)->d, &release_info_ext);
}

static int interface_get_cursor_command(QXLInterface *qxl, struct QXLCommandExt *cmd)
{
    return _qxl_get_cursor_command(((Interface *)qxl)->d, cmd);
}

static int interface_req_cursor_notification(QXLInterface *qxl)
{
    return _qxl_req_cursor_notification(((Interface *)qxl)->d);
}

static void interface_get_update_area(QXLInterface *qxl, const Rect **rect, UINT32 **surface_id)
{
    _qxl_get_update_area(((Interface *)qxl)->d, rect, surface_id);
}

static void interface_notify_update(QXLInterface *qxl, uint32_t update_id)
{
    _qxl_notify_update(((Interface *)qxl)->d, update_id);
}

static void interface_set_save_data(QXLInterface *qxl, void *data, int size)
{
    _qxl_set_save_data(((Interface *)qxl)->d, data, size);
}

static void *interface_get_save_data(QXLInterface *qxl)
{
    return _qxl_get_save_data(((Interface *)qxl)->d);
}

static int interface_flush_resources(QXLInterface *qxl)
{
    return _qxl_flush_resources(((Interface *)qxl)->d);
}

static void regitser_interface(PCIQXLDevice *d)
{
    Interface *interface = (Interface *)qemu_mallocz(sizeof(*interface));

    if (!interface) {
        printf("%s: malloc failed\n", __FUNCTION__);
        exit(-1);
    }
    interface->vd_interface.base.base_version = VM_INTERFACE_VERSION;
    interface->vd_interface.base.type = VD_INTERFACE_QXL;
    interface->vd_interface.base.id = d->id;
    interface->vd_interface.base.description = "QXL GPU";
    interface->vd_interface.base.major_version = VD_INTERFACE_QXL_MAJOR;
    interface->vd_interface.base.minor_version = VD_INTERFACE_QXL_MINOR;

    interface->vd_interface.pci_vendor = REDHAT_PCI_VENDOR_ID;
    interface->vd_interface.pci_id = QXL_DEVICE_ID;
    interface->vd_interface.pci_revision = QXL_REVISION;

    interface->vd_interface.attache_worker = interface_attache_worker;
    interface->vd_interface.set_compression_level = interface_set_compression_level;
    interface->vd_interface.set_mm_time = interface_set_mm_time;

    interface->vd_interface.get_init_info = interface_get_init_info;
    interface->vd_interface.get_command = interface_get_command;
    interface->vd_interface.req_cmd_notification = interface_req_cmd_notification;
    interface->vd_interface.has_command = interface_has_command;
    interface->vd_interface.release_resource = interface_release_resource;
    interface->vd_interface.get_cursor_command = interface_get_cursor_command;
    interface->vd_interface.req_cursor_notification = interface_req_cursor_notification;
    interface->vd_interface.get_update_area = interface_get_update_area;
    interface->vd_interface.notify_update = interface_notify_update;
    interface->vd_interface.set_save_data = interface_set_save_data;
    interface->vd_interface.get_save_data = interface_get_save_data;
    interface->vd_interface.flush_resources = interface_flush_resources;

    interface->d = d;
    add_interface(&interface->vd_interface.base);
}

#endif

static void creat_native_worker(PCIQXLDevice *d, int id)
{
    d->worker = qxl_interface_create_worker((QXLDevRef)d, id);
    ASSERT(d->worker);
}

static void vdi_port_write_dword(void *opaque, uint32_t addr, uint32_t val)
{
    PCIVDIPortDevice *d = (PCIVDIPortDevice *)opaque;
    uint32_t io_port = addr - d->io_base;
#ifdef DEBUG_QXL
    printf("%s: addr 0x%x val 0x%x\n", __FUNCTION__, addr, val);
#endif
    switch (io_port) {
    case VDI_PORT_IO_NOTIFY:
        if (!d->connected) {
             printf("%s: not connected\n", __FUNCTION__);
            return;
        }
        vdi_port_dev_notify(d);
        break;
    case VDI_PORT_IO_UPDATE_IRQ:
        qemu_set_irq(d->pci_dev.irq[0], vdi_port_irq_level(d));
        break;
    case VDI_PORT_IO_CONNECTION:
        vdi_port_dev_disconnect(d);
        break;
    default:
         printf("%s: unexpected addr 0x%x val 0x%x\n", __FUNCTION__, addr, val);
    };
}

static uint32_t vdi_port_read_dword(void *opaque, uint32_t addr)
{
    PCIVDIPortDevice *d = (PCIVDIPortDevice *)opaque;
    uint32_t io_port = addr - d->io_base;
#ifdef DEBUG_QXL
    printf("%s: addr 0x%x val 0x%x\n", __FUNCTION__, addr, val);
#endif
    if (io_port == VDI_PORT_IO_CONNECTION) {
        return vdi_port_dev_connect(d);
    } else {
         printf("%s: unexpected addr 0x%x\n", __FUNCTION__, addr);
    }
    return 0xffffffff;
}

static void vdi_port_io_map(PCIDevice *pci_dev, int region_num,
                             uint32_t addr, uint32_t size, int type)
{
    PCIVDIPortDevice *d = (PCIVDIPortDevice *)pci_dev;

    printf("%s: base 0x%x size 0x%x\n", __FUNCTION__, addr, size);
    d->io_base = addr;
    register_ioport_write(addr, size, 4, vdi_port_write_dword, pci_dev);
    register_ioport_read(addr, size, 4, vdi_port_read_dword, pci_dev);
}

static void vdi_port_ram_map(PCIDevice *pci_dev, int region_num,
                       uint32_t addr, uint32_t size, int type)
{
    PCIVDIPortDevice *d = (PCIVDIPortDevice *)pci_dev;

    printf("%s: addr 0x%x size 0x%x\n", __FUNCTION__, addr, size);

    ASSERT((addr & (size - 1)) == 0);
    ASSERT(size ==  d->ram_size);

    cpu_register_physical_memory(addr, size, d->ram_offset | IO_MEM_RAM);
}

static void vdi_port_reset(PCIVDIPortDevice *d)
{
    memset(d->ram, 0, sizeof(*d->ram));
    RING_INIT(&d->ram->input);
    RING_INIT(&d->ram->output);
    d->ram->magic = VDI_PORT_MAGIC;
    d->ram->generation = 0;
    d->ram->int_pending = 0;
    d->ram->int_mask = 0;
    d->connected = FALSE;
#ifdef CONFIG_SPICE
    d->plug_read_pos = 0;
#endif
     vdi_port_set_dirty(d, d->ram, sizeof(*d->ram));
}

static void vdi_port_reset_handler(void *opaque)
{
    PCIVDIPortDevice *d = (PCIVDIPortDevice *)opaque;
    if (d->connected) {
        vdi_port_dev_disconnect(d);
    }
    vdi_port_reset(d);
    qemu_set_irq(d->pci_dev.irq[0], vdi_port_irq_level(d));
}

static void vdi_port_save(QEMUFile* f, void* opaque)
{
    PCIVDIPortDevice* d = (PCIVDIPortDevice*)opaque;

    pci_device_save(opaque, f);
    qemu_put_be32(f, d->connected);
}

static int vdi_port_load(QEMUFile* f,void* opaque,int version_id)
{
    PCIVDIPortDevice* d = (PCIVDIPortDevice*)opaque;
    int ret;

#ifdef CONFIG_SPICE
    vdi_port_unregister_interface(d);
#endif
    if ((ret = pci_device_load(&d->pci_dev, f))) {
       printf("%s: error=%d\n", __FUNCTION__, ret);
       return ret;
    }
    d->connected = qemu_get_be32(f);
    if (d->connected) {
#ifdef CONFIG_SPICE
        vdi_port_register_interface(d);
#endif
    }
    return 0;
}

static void vdi_port_vm_change_state_handler(void *opaque, int running)
{
    PCIVDIPortDevice* d=(PCIVDIPortDevice*)opaque;

    if (running == d->running ) {
        return;
    }

    if (running) {
        d->running = TRUE;
        if (d->new_gen_on_resume) {
            d->new_gen_on_resume = FALSE;
            vdi_port_new_gen(d);
            vdi_port_notify_guest(d);
        }
        qemu_set_irq(d->pci_dev.irq[0], vdi_port_irq_level(d));
        vdi_port_dev_notify(d);
    } else {
        d->running = FALSE;
    }
}

static void vdi_port_init(PCIBus *bus, uint8_t *ram, unsigned long ram_offset, 
                                uint32_t ram_size)
{
    PCIVDIPortDevice *d;
    PCIConf *pci_conf;

    if (!(d = (PCIVDIPortDevice*)pci_register_device(bus, VDI_PORT_DEV_NAME,
                                           sizeof(PCIVDIPortDevice), -1, NULL,
                                           NULL))) {
        printf("%s: register devic failed\n", __FUNCTION__);
        exit(-1);
    }
    pci_conf = (PCIConf *)d->pci_dev.config;

    pci_conf->vendor_ID = REDHAT_PCI_VENDOR_ID;
    pci_conf->device_ID = VDI_PORT_DEVICE_ID;
    pci_conf->revision = VDI_PORT_REVISION;
    pci_conf->class_base = PCI_CLASS_COM_CONTROLLER;
    pci_conf->class_sub = PCI_COM_CONTROLLER_SUBCLASS_OTHER;
    pci_conf->class_prog_if = PCI_COM_CONTROLLER_SUBCLASS_OTHER_PROGIF;
    pci_conf->interrupt_pin = 1;

    d->ram = (VDIPortRam *)ram;
    d->ram_offset = ram_offset;
    vdi_port_reset(d);
    d->ram_size = ram_size;
    d->new_gen_on_resume = FALSE;
    d->running = FALSE;
    

    pci_register_io_region(&d->pci_dev, VDI_PORT_IO_RANGE_INDEX,
                           msb_mask(VDI_PORT_IO_RANGE_SIZE * 2 - 1),
                           PCI_ADDRESS_SPACE_IO, vdi_port_io_map);

    pci_register_io_region(&d->pci_dev, VDI_PORT_RAM_RANGE_INDEX,
                           d->ram_size , PCI_ADDRESS_SPACE_MEM,
                           vdi_port_ram_map);

    register_savevm(VDI_PORT_DEV_NAME, 0, VDI_PORT_SAVE_VERSION,
                    vdi_port_save, vdi_port_load, d);
    qemu_register_reset(vdi_port_reset_handler, d);
    qemu_add_vm_change_state_handler(vdi_port_vm_change_state_handler, d);
}

void qxl_init(PCIBus *bus, uint8_t *vram, unsigned long vram_offset,
              uint32_t vram_size, uint32_t in_ram_size, QXLAddressRange *address_ranges,
			  uint8_t num_ranges)
{
    PCIQXLDevice *d;
    PCIConf *pci_conf;
    uint32_t rom_size;
    uint32_t max_fb;
    uint32_t qxl_ram_size = msb_mask(in_ram_size*2 - 1);
    uint32_t vdi_port_ram_size = msb_mask(sizeof(VDIPortRam) * 2 - 1);
    int i;

    static int device_id = 0;

    if (device_id == 0) {
        vdi_port_init(bus, vram, vram_offset, vdi_port_ram_size);
    }
    vram += vdi_port_ram_size;
    vram_offset += vdi_port_ram_size;
    vram_size -= vdi_port_ram_size;
    d = (PCIQXLDevice*)pci_register_device(bus, QXL_DEV_NAME,
                                           sizeof(PCIQXLDevice), -1, NULL,
                                           NULL);

    d->id = device_id;
    d->state.mode = QXL_MODE_UNDEFINED;
    if (!device_id) {
        qxl_init_modes();
    }

    register_savevm(QXL_DEV_NAME, device_id, QXL_SAVE_VERSION, qxl_save, qxl_load, d);
    qemu_add_vm_change_state_handler(qxl_vm_change_state_handler, d);

    pci_conf = (PCIConf *)d->pci_dev.config;

    pci_conf->vendor_ID = REDHAT_PCI_VENDOR_ID;
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

    ASSERT(vram_size > qxl_ram_size);
    rom_size = init_qxl_rom(d, vram + qxl_ram_size , vram_size - qxl_ram_size, &max_fb);
    rom_size = MAX(rom_size, TARGET_PAGE_SIZE);  
    rom_size = msb_mask(rom_size * 2 - 1);
    d->state.rom_offset = vram_offset + qxl_ram_size;
    d->state.rom_size = rom_size;

    ASSERT(qxl_ram_size + rom_size < vram_size);
    init_qxl_ram(&d->state, vram, in_ram_size);
    d->state.ram_offset = vram_offset;
    d->state.ram_size = qxl_ram_size;

    d->state.vram = vram + rom_size + qxl_ram_size;
    d->state.vram_offset = vram_offset + rom_size + qxl_ram_size;
    d->state.vram_size = msb_mask(vram_size - (qxl_ram_size + rom_size));

    d->state.address_ranges = (QXLAddressRange *)malloc(sizeof(QXLAddressRange) * (num_ranges + 2));
    PANIC_ON(!d->state.address_ranges);
    qxl_reset_device_address_ranges(d);
    for (i = 0; i < num_ranges; ++i) {
        d->state.address_ranges[i + 2] = address_ranges[i];
    }
    d->state.num_ranges = num_ranges + 2;

    d->state.group_ids_flip[0] = 0;
    d->state.group_ids_flip[1] = 1;
    d->state.send_event_flip[0] = FALSE;
    d->state.send_event_flip[1] = TRUE;
    d->state.rings_flip[0] = &d->state.vga_ring;
    d->state.rings_flip[1] = &d->state.ram->cmd_ring;
    d->state.flip_counter = 0;

    printf("%s: rom(%p, 0x%lx, 0x%x) ram(%p, 0x%lx, 0x%x) vram(%p, 0x%lx, 0x%x)\n",
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
    d->dev_next = dev_list;
    dev_list = d;
    d->vga_next = qxl_vga.clients;
    qxl_vga.clients = d;
    main_thread = pthread_self();
    qxl_reset_state(d);
    qxl_init_memslots(d);
    init_pipe_signaling(d);

    
#ifdef CONFIG_SPICE
    regitser_interface(d);
#endif
    if (!d->worker) {
        creat_native_worker(d, device_id);
    }
    qxl_create_host_memslot(d);
    if (QXL_SHARED_VGA_MODE || !d->id) {
        qxl_enter_vga_mode(d);
    } 
    qemu_register_reset(reset_handler, d);
    device_id++;
}


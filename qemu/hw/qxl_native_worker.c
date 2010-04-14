#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "qemu-common.h"

#include "qxl_interface.h"
#include "qxl_dev.h"

#define qxl_error(format, ...) {                                 \
    printf("%s: " format "\n", __FUNCTION__, ## __VA_ARGS__ );   \
    exit(-1);                                                    \
}

#define qxl_printf(format, ...) \
    printf("%s: " format "\n", __FUNCTION__, ## __VA_ARGS__ )

#define ASSERT(x) if (!(x)) {                               \
    printf("%s: ASSERT %s failed\n", __FUNCTION__, #x);     \
    exit(-1);                                               \
}

#define PANIC_ON(x) if ((x)) {                                  \
    printf("%s: PANIC_ON %s failed\n", __FUNCTION__, #x);       \
    exit(-1);                                                   \
}

typedef struct MemSlot {
    int active;
    int generation;
    unsigned long virt_start_addr;
    unsigned long virt_end_addr;
    unsigned long address_delta;
} MemSlot;

typedef struct QxlDispatcher {
    QXLWorker base;
    int id;
    QXLDevRef dev_ref;
    uint32_t nmem_slots_groups;
    uint32_t nmem_slots;
    MemSlot **mem_slots;
    uint8_t generation_bits;
    uint8_t mem_slot_bits;
} QxlDispatcher;

static inline int get_memslot_id(QXLWorker *worker, unsigned long addr)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    return addr >> (64 - dispatcher->mem_slot_bits);
}

static inline int get_generation(QXLWorker *worker, unsigned long addr)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    return (addr >> (64 - (dispatcher->mem_slot_bits + dispatcher->generation_bits))) &
           ~((unsigned long)-1 << dispatcher->generation_bits);
}

static inline unsigned long __get_clean_virt(QXLWorker *worker, unsigned long addr)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    return addr & (((unsigned long)(-1)) >> (dispatcher->mem_slot_bits + dispatcher->generation_bits));
}

static inline void validate_virt(QXLWorker *worker, unsigned long virt, int group_slot_id,
                                 int slot_id, uint32_t add_size)
{
    MemSlot *slot;
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    slot = &dispatcher->mem_slots[group_slot_id][slot_id];
    if (virt < slot->virt_start_addr || (virt + add_size) > slot->virt_end_addr) {
        qxl_error("virtual address out of range 0x%lx 0x%lx", virt, slot->address_delta);
    }
}

static inline unsigned long get_virt(QXLWorker *worker, int group_slot_id,
                                     unsigned long addr, uint32_t add_size)
{
    uint32_t slot_id;
    int generation;
    unsigned long h_virt;
    MemSlot *slot;
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    if (group_slot_id > dispatcher->nmem_slots_groups) {
        qxl_error("group_slot_id too big");
    }

    slot_id = get_memslot_id(worker, addr);
    if (slot_id > dispatcher->nmem_slots) {
        qxl_error("slot_id too big");
    }

    slot = &dispatcher->mem_slots[group_slot_id][slot_id];
    if (!slot->active) {
        qxl_error("mem_slot is not avaible %d 0x%lx", slot_id, addr);
    }

    generation = get_generation(worker, addr);
    if (generation != slot->generation) {
        qxl_error("address generation is not valid");
    }

    h_virt = __get_clean_virt(worker, addr);
    h_virt += slot->address_delta;

    validate_virt(worker, h_virt, group_slot_id, slot_id, add_size);

    return h_virt;
}

static void native_qxl_worker_wakeup(QXLWorker *worker)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;
    QXLCommandExt cmd_ext;
    QXLReleaseInfoExt release_info_ext;

    qxl_printf("");

    for (;;) {
        if (qxl_get_command(dispatcher->dev_ref, &cmd_ext)) {
            switch (cmd_ext.cmd.type) {
            case QXL_CMD_DRAW: {
                QXLDrawable *draw_cmd = (QXLDrawable *)get_virt(worker, cmd_ext.group_id,
                                                                cmd_ext.cmd.data, 
                                                                sizeof(QXLDrawable));
                release_info_ext.info = &draw_cmd->release_info;
                release_info_ext.group_id = cmd_ext.group_id;
                qxl_release_resource(dispatcher->dev_ref, release_info_ext);
                break;
            }
            case QXL_CMD_UPDATE: {
                QXLUpdateCmd *update_cmd = (QXLUpdateCmd *)get_virt(worker, cmd_ext.group_id,
                                                                    cmd_ext.cmd.data, 
                                                                    sizeof(QXLUpdateCmd));
                qxl_notify_update(dispatcher->dev_ref, update_cmd->update_id);
                release_info_ext.info = &update_cmd->release_info;
                release_info_ext.group_id = cmd_ext.group_id;
                qxl_release_resource(dispatcher->dev_ref, release_info_ext);
                break;
            }
            case QXL_CMD_MESSAGE: {
                QXLMessage *message = (QXLMessage *)get_virt(worker, cmd_ext.group_id,
                                                             cmd_ext.cmd.data,
                                                             sizeof(QXLMessage));
                qxl_printf("MESSAGE: %s", message->data);
                release_info_ext.info = &message->release_info;
                release_info_ext.group_id = cmd_ext.group_id;
                qxl_release_resource(dispatcher->dev_ref, release_info_ext);
                break;
            }
            default:
                qxl_error("bad command type");
            }
            continue;
        }
        if (qxl_req_cmd_notification(dispatcher->dev_ref)) {
            break;
        }
    }
    for (;;) {
        if (qxl_get_cursor_command(dispatcher->dev_ref, &cmd_ext)) {
            switch (cmd_ext.cmd.type) {
            case QXL_CMD_CURSOR: {
                QXLReleaseInfoExt release_info_ext;
                QXLCursorCmd *cursor_cmd = (QXLCursorCmd *)get_virt(worker, cmd_ext.group_id,
                                                                    cmd_ext.cmd.data,
                                                                    sizeof(QXLCursorCmd));
                release_info_ext.group_id = cmd_ext.group_id;
                release_info_ext.info = &cursor_cmd->release_info;
                qxl_release_resource(dispatcher->dev_ref, release_info_ext);
                break;
            }
             default:
                qxl_error("bad command type");
            }
            continue;
        }
        if (qxl_req_cursor_notification(dispatcher->dev_ref)) {
            break;
        }
    }
}

static void native_qxl_worerk_reset_memslots(QXLWorker *worker)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    memset(dispatcher->mem_slots, 0, sizeof(MemSlot) * dispatcher->nmem_slots *
           dispatcher->nmem_slots_groups);
}

static void native_qxl_worker_destroy_surfaces(QXLWorker *worker)
{
}

static void native_qxl_worker_create_primary(QXLWorker *qxl_worker, uint32_t surface_id,
                                             QXLDevSurfaceCreate *surface)
{
}

static void native_qxl_worker_destroy_primary(QXLWorker *qxl_worker, uint32_t surface_id)
{
}

static void native_qxl_worker_reset_cursor(QXLWorker *worker)
{
}

static void native_qxl_worker_reset_image_cache(QXLWorker *worker)
{
}

static void native_qxl_worker_destroy_surface_wait(QXLWorker *worker, uint32_t surface_id)
{
}

static void native_qxl_worker_detach(QXLWorker *worker)
{
    qxl_printf("");

    native_qxl_worker_wakeup(worker);
}

static void native_qxl_worker_update_area(QXLWorker *worker, uint32_t surface_id,
                                          Rect *area, Rect *dirty_rects,
                                          uint32_t num_dirty_rects, uint32_t clear_dirty_region)
{
    qxl_printf("");
    native_qxl_worker_wakeup(worker);
}

static void native_qxl_worker_add_memslot(QXLWorker *worker, QXLDevMemSlot *dev_slot)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    qxl_printf("");

    ASSERT(dispatcher->nmem_slots > dev_slot->slot_id);
    ASSERT(!dispatcher->mem_slots[dev_slot->slot_group_id][dev_slot->slot_id].active);

    dispatcher->mem_slots[dev_slot->slot_group_id][dev_slot->slot_id].active = 1;
    dispatcher->mem_slots[dev_slot->slot_group_id][dev_slot->slot_id].address_delta =
                                                                               dev_slot->addr_delta;
    dispatcher->mem_slots[dev_slot->slot_group_id][dev_slot->slot_id].virt_start_addr =
                                                                               dev_slot->virt_start;
    dispatcher->mem_slots[dev_slot->slot_group_id][dev_slot->slot_id].virt_end_addr =
                                                                                 dev_slot->virt_end;
    dispatcher->mem_slots[dev_slot->slot_group_id][dev_slot->slot_id].generation =
                                                                               dev_slot->generation;
}

static void native_qxl_worker_del_memslot(QXLWorker *worker, uint32_t slot_group_id,
                                          uint32_t slot_id) {
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    qxl_printf("");


    ASSERT(dispatcher->nmem_slots > slot_id);
    ASSERT(dispatcher->nmem_slots_groups > slot_group_id);
    ASSERT(dispatcher->mem_slots[slot_group_id][slot_id].active);

    dispatcher->mem_slots[slot_group_id][slot_id].active = 0;
}

static void native_qxl_worker_oom(QXLWorker *worker)
{
    qxl_printf("");
    native_qxl_worker_wakeup(worker);
}

static void native_qxl_worker_start(QXLWorker *worker)
{
    qxl_printf("");
}

static void native_qxl_worker_stop(QXLWorker *worker)
{
    qxl_printf("");
}

static void native_qxl_worker_save(QXLWorker *worker)
{
    qxl_printf("");
}

static void native_qxl_worker_load(QXLWorker *worker)
{
    qxl_printf("");
}

QXLWorker *qxl_interface_create_worker(QXLDevRef dev_ref, int device_id)
{
    QxlDispatcher *dispatcher;
    QXLDevInitInfo init;
    uint32_t i;

    if (!(dispatcher = malloc(sizeof(QxlDispatcher)))) {
        qxl_error("malloc failed");
    }
    memset(dispatcher, 0, sizeof(*dispatcher));
    dispatcher->id = device_id;
    dispatcher->dev_ref = dev_ref;

    dispatcher->base.wakeup = native_qxl_worker_wakeup;
    dispatcher->base.oom = native_qxl_worker_oom;
    dispatcher->base.save = native_qxl_worker_save;
    dispatcher->base.load = native_qxl_worker_load;
    dispatcher->base.start = native_qxl_worker_start;
    dispatcher->base.stop = native_qxl_worker_stop;
    dispatcher->base.update_area = native_qxl_worker_update_area;
    dispatcher->base.add_memslot = native_qxl_worker_add_memslot;
    dispatcher->base.del_memslot = native_qxl_worker_del_memslot;
    dispatcher->base.reset_memslots = native_qxl_worerk_reset_memslots;
    dispatcher->base.destroy_surfaces = native_qxl_worker_destroy_surfaces;
    dispatcher->base.create_primary_surface = native_qxl_worker_create_primary;
    dispatcher->base.destroy_primary_surface = native_qxl_worker_destroy_primary;
    dispatcher->base.reset_image_cache = native_qxl_worker_reset_image_cache;
    dispatcher->base.reset_cursor = native_qxl_worker_reset_cursor;
    dispatcher->base.destroy_surface_wait = native_qxl_worker_destroy_surface_wait;

    qxl_get_init_info(dispatcher->dev_ref, &init);

    ASSERT(init.num_memslots > 0);
    ASSERT(init.num_memslots_groups > 0);
    dispatcher->mem_slots = (MemSlot **)malloc(sizeof(MemSlot *) * init.num_memslots_groups);
    PANIC_ON(!dispatcher->mem_slots);

    for (i = 0; i <  init.num_memslots_groups; ++i) {
        dispatcher->mem_slots[i] = malloc(sizeof(MemSlot) * init.num_memslots);
        PANIC_ON(!dispatcher->mem_slots[i]);
        memset(dispatcher->mem_slots[i], 0, sizeof(MemSlot) * init.num_memslots);
    }


    dispatcher->nmem_slots_groups = init.num_memslots_groups;
    dispatcher->nmem_slots = init.num_memslots;
    dispatcher->mem_slot_bits = init.memslot_id_bits;
    dispatcher->generation_bits = init.memslot_gen_bits;

    return &dispatcher->base;
}


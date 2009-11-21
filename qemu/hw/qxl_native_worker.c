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
    uint32_t nmem_slots;
    MemSlot *mem_slots;
    uint8_t generation_bits;
    uint8_t mem_slot_bits;
    QXLDevInfo dev_info;
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

static inline void validate_virt(QXLWorker *worker, unsigned long virt, int slot_id, uint32_t add_size)
{
    MemSlot *slot;
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    slot = &dispatcher->mem_slots[slot_id];
    if (virt < slot->virt_start_addr || (virt + add_size) > slot->virt_end_addr) {
        qxl_error("virtual address out of range 0x%lx 0x%lx", virt, slot->address_delta);
    }
}

static inline unsigned long get_virt(QXLWorker *worker, unsigned long addr, uint32_t add_size)
{
    uint32_t slot_id;
    int generation;
    unsigned long h_virt;
    MemSlot *slot;
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    slot_id = get_memslot_id(worker, addr);
    if (slot_id > dispatcher->nmem_slots) {
        qxl_error("slot_id too big");
    }

    slot = &dispatcher->mem_slots[slot_id];
    if (!slot->active) {
        qxl_error("mem_slot is not avaible %d 0x%lx", slot_id, addr);
    }

    generation = get_generation(worker, addr);
    if (generation != slot->generation) {
        qxl_error("address generation is not valid");
    }

    h_virt = __get_clean_virt(worker, addr);
    h_virt += slot->address_delta;

    validate_virt(worker, h_virt, slot_id, add_size);

    return h_virt;
}

static void native_qxl_worker_wakeup(QXLWorker *worker)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;
    QXLCommand cmd;

    qxl_printf("");

    for (;;) {
        if (qxl_get_command(dispatcher->dev_ref, &cmd)) {
            switch (cmd.type) {
            case QXL_CMD_DRAW: {
                QXLDrawable *draw_cmd = (QXLDrawable *)get_virt(worker, cmd.data, sizeof(QXLDrawable));
                qxl_release_resource(dispatcher->dev_ref, &draw_cmd->release_info);
                break;
            }
            case QXL_CMD_UPDATE: {
                QXLUpdateCmd *update_cmd = (QXLUpdateCmd *)get_virt(worker, cmd.data, sizeof(QXLUpdateCmd));
                qxl_notify_update(dispatcher->dev_ref, update_cmd->update_id);
                qxl_release_resource(dispatcher->dev_ref, &update_cmd->release_info);
                break;
            }
            case QXL_CMD_MESSAGE: {
                QXLMessage *message = (QXLMessage *)get_virt(worker, cmd.data, sizeof(QXLMessage));
                qxl_printf("MESSAGE: %s", message->data);
                qxl_release_resource(dispatcher->dev_ref, &message->release_info);
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
        if (qxl_get_cursor_command(dispatcher->dev_ref, &cmd)) {
            switch (cmd.type) {
            case QXL_CMD_CURSOR: {
                QXLCursorCmd *cursor_cmd = (QXLCursorCmd *)get_virt(worker, cmd.data, sizeof(QXLCursorCmd));
                qxl_release_resource(dispatcher->dev_ref, &cursor_cmd->release_info);
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

static void native_qxl_worker_attach(QXLWorker *worker)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    qxl_printf("");

    qxl_get_info(dispatcher->dev_ref, &dispatcher->dev_info);

    native_qxl_worker_wakeup(worker);
}

static void native_qxl_worerk_reset_memslots(QXLWorker *worker)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    memset(dispatcher->mem_slots, 0, sizeof(MemSlot) * dispatcher->nmem_slots);
}

static void native_qxl_worker_detach(QXLWorker *worker)
{
    qxl_printf("");

    native_qxl_worker_wakeup(worker);
}

static void native_qxl_worker_update_area(QXLWorker *worker)
{
    qxl_printf("");
    native_qxl_worker_wakeup(worker);
}

static void native_qxl_worker_add_memslot(QXLWorker *worker, QXLDevMemSlot *dev_slot)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    qxl_printf("");

    ASSERT(dispatcher->nmem_slots > dev_slot->slot_id);
    ASSERT(!dispatcher->mem_slots[dev_slot->slot_id].active);

    dispatcher->mem_slots[dev_slot->slot_id].active = 1;
    dispatcher->mem_slots[dev_slot->slot_id].address_delta = dev_slot->addr_delta;
    dispatcher->mem_slots[dev_slot->slot_id].virt_start_addr = dev_slot->virt_start;
    dispatcher->mem_slots[dev_slot->slot_id].virt_end_addr = dev_slot->virt_end;
    dispatcher->mem_slots[dev_slot->slot_id].generation = dev_slot->generation;
}

static void native_qxl_worker_del_memslot(QXLWorker *worker, uint32_t slot_id)
{
    QxlDispatcher *dispatcher = (QxlDispatcher *)worker;

    qxl_printf("");

    ASSERT(dispatcher->nmem_slots > slot_id);
    ASSERT(dispatcher->mem_slots[slot_id].active);

    dispatcher->mem_slots[slot_id].active = 0;
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

    if (!(dispatcher = malloc(sizeof(QxlDispatcher)))) {
        qxl_error("malloc failed");
    }
    memset(dispatcher, 0, sizeof(*dispatcher));
    dispatcher->id = device_id;
    dispatcher->dev_ref = dev_ref;

    dispatcher->base.attach = native_qxl_worker_attach;
    dispatcher->base.detach = native_qxl_worker_detach;
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

    qxl_get_init_info(dispatcher->dev_ref, &init);

    dispatcher->mem_slots = (MemSlot *)malloc(sizeof(MemSlot) * init.num_memslots);
    ASSERT(dispatcher->mem_slots);
    memset(dispatcher->mem_slots, 0, sizeof(MemSlot) * init.num_memslots);
    dispatcher->nmem_slots = init.num_memslots;
    dispatcher->mem_slot_bits = init.memslot_id_bits;
    dispatcher->generation_bits = init.memslot_gen_bits;

    return &dispatcher->base;
}


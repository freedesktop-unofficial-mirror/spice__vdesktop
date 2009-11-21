#ifndef _H_QXL_INTERFACE
#define _H_QXL_INTERFACE

#include <stdint.h>

#include "qxl_dev.h"

#ifdef CONFIG_SPICE

#include "vd_interface.h"

#else

typedef struct DrawArea{
    uint8_t *buf;
    uint32_t size;
    uint8_t *line_0;
    uint32_t width;
    uint32_t heigth;
    int stride;
} DrawArea;

typedef struct QXLDevInfo {
    uint32_t x_res;
    uint32_t y_res;
    int use_hardware_cursor;
    uint32_t bits;
    DrawArea draw_area;
    uint32_t ram_size;
    uint32_t num_memslots;
    uint8_t mem_generation_bits;
    uint8_t mem_slot_bits;
} QXLDevInfo;

typedef struct QXLDevInitInfo {
    uint32_t num_memslots;
    uint8_t memslot_gen_bits;
    uint8_t memslot_id_bits;
} QXLDevInitInfo;

typedef struct QXLDevMemSlot {
    uint32_t slot_id;
    uint32_t generation;
    unsigned long virt_start;
    unsigned long virt_end;
    uint64_t addr_delta;
} QXLDevMemSlot;

typedef struct QXLWorker QXLWorker;
struct QXLWorker {
    void (*attach)(QXLWorker *worker);
    void (*detach)(QXLWorker *worker);
    void (*wakeup)(QXLWorker *worker);
    void (*oom)(QXLWorker *worker);
    void (*save)(QXLWorker *worker);
    void (*load)(QXLWorker *worker);
    void (*start)(QXLWorker *worker);
    void (*stop)(QXLWorker *worker);
    void (*update_area)(QXLWorker *worker);
    void (*add_memslot)(QXLWorker *worker, QXLDevMemSlot *slot);
    void (*del_memslot)(QXLWorker *worker, uint32_t slot_id);
    void (*reset_memslots)(QXLWorker *worker);
};
#endif

typedef unsigned long QXLDevRef;

void qxl_get_init_info(QXLDevRef dev_ref, QXLDevInitInfo *info);
void qxl_get_info(QXLDevRef dev_ref, QXLDevInfo *info);
int qxl_get_command(QXLDevRef dev_ref, QXLCommand *cmd);
void qxl_release_resource(QXLDevRef dev_ref, QXLReleaseInfo *release_info);
void qxl_notify_update(QXLDevRef dev_ref, uint32_t update_id);
int qxl_req_cmd_notification(QXLDevRef dev_ref);
int qxl_get_cursor_command(QXLDevRef dev_ref, QXLCommand *cmd);
int qxl_req_cursor_notification(QXLDevRef dev_ref);
int qxl_has_command(QXLDevRef dev_ref);
const Rect *qxl_get_update_area(QXLDevRef dev_ref);
int qxl_flush_resources(QXLDevRef dev_ref);
void qxl_set_save_data(QXLDevRef dev_ref, void *data, int size);
void *qxl_get_save_data(QXLDevRef dev_ref);

QXLWorker *qxl_interface_create_worker(QXLDevRef dev, int id);

#endif


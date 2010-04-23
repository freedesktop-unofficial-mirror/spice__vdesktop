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
    uint32_t num_memslots_groups;
    uint32_t num_memslots;
    uint8_t memslot_gen_bits;
    uint8_t memslot_id_bits;
    uint32_t qxl_ram_size;
    uint8_t internal_groupslot_id;
    uint32_t n_surfaces;
} QXLDevInitInfo;

typedef struct QXLDevMemSlot {
    uint32_t slot_group_id;
    uint32_t slot_id;
    uint32_t generation;
    unsigned long virt_start;
    unsigned long virt_end;
    uint64_t addr_delta;
} QXLDevMemSlot;

typedef struct QXLDevSurfaceCreate {
    uint32_t width;
    uint32_t height;
    int32_t stride;
    uint32_t format;
    uint32_t position;
    uint32_t mouse_mode;
    uint32_t flags;
    uint32_t type;
    uint64_t mem;
    uint32_t group_id;
} QXLDevSurfaceCreate;

typedef struct QXLWorker QXLWorker;
struct QXLWorker {
    void (*wakeup)(QXLWorker *worker);
    void (*oom)(QXLWorker *worker);
    void (*save)(QXLWorker *worker);
    void (*load)(QXLWorker *worker);
    void (*start)(QXLWorker *worker);
    void (*stop)(QXLWorker *worker);
    void (*update_area)(QXLWorker *qxl_worker, uint32_t surface_id,
                        Rect *area, Rect *dirty_rects,
                        uint32_t num_dirty_rects, uint32_t clear_dirty_region);
    void (*add_memslot)(QXLWorker *worker, QXLDevMemSlot *slot);
    void (*del_memslot)(QXLWorker *worker, uint32_t slot_group_id, uint32_t slot_id);
    void (*reset_memslots)(QXLWorker *worker);
    void (*destroy_surfaces)(QXLWorker *worker);
    void (*destroy_primary_surface)(QXLWorker *worker, uint32_t surface_id);
    void (*create_primary_surface)(QXLWorker *worker, uint32_t surface_id,
                                   QXLDevSurfaceCreate *surface);
    void (*reset_image_cache)(QXLWorker *worker);
    void (*reset_cursor)(QXLWorker *worker);
    void (*destroy_surface_wait)(QXLWorker *worker, uint32_t surface_id);
};
#endif

typedef unsigned long QXLDevRef;

void qxl_get_init_info(QXLDevRef dev_ref, QXLDevInitInfo *info);
int qxl_get_command(QXLDevRef dev_ref, QXLCommandExt *cmd);
void qxl_release_resource(QXLDevRef dev_ref, QXLReleaseInfoExt release_info);
void qxl_notify_update(QXLDevRef dev_ref, uint32_t update_id);
int qxl_req_cmd_notification(QXLDevRef dev_ref);
int qxl_get_cursor_command(QXLDevRef dev_ref, QXLCommandExt *cmd);
int qxl_req_cursor_notification(QXLDevRef dev_ref);
int qxl_has_command(QXLDevRef dev_ref);
int qxl_flush_resources(QXLDevRef dev_ref);
void qxl_set_save_data(QXLDevRef dev_ref, void *data, int size);
void *qxl_get_save_data(QXLDevRef dev_ref);

QXLWorker *qxl_interface_create_worker(QXLDevRef dev, int id);

#endif


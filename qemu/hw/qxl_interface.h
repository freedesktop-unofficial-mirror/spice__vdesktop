#ifndef _H_QXL_INTERFACE
#define _H_QXL_INTERFACE

#include <stdint.h>

union QXLReleaseInfo;
struct QXLCommand;

typedef unsigned long QXLWorkerRef;
typedef unsigned long QXLDevRef;

typedef struct DrawArea{
    uint8_t *buf;
    uint32_t size;
    uint8_t *line_0;
    uint32_t width;
    uint32_t heigth;
    int stride;
} DrawArea;

typedef struct QXLDevInfo {
    long phys_delta;
    unsigned long phys_start;
    unsigned long phys_end;

    uint32_t x_res;
    uint32_t y_res;
    uint32_t bits;

    DrawArea draw_area;

} QXLDevInfo;

void qxl_get_info(QXLDevRef dev_ref, QXLDevInfo *info);
int qxl_get_command(QXLDevRef dev_ref, struct QXLCommand *cmd);
int qxl_has_command(QXLDevRef dev_ref);
int qxl_get_cursor_command(QXLDevRef dev_ref, struct QXLCommand *cmd);
const struct Rect *qxl_get_update_area(QXLDevRef dev_ref);
int qxl_req_cmd_notification(QXLDevRef dev_ref);
int qxl_req_cursor_notification(QXLDevRef dev_ref);
void qxl_release_resource(QXLDevRef dev_ref, union QXLReleaseInfo *release_info);
int qxl_flush_resources(QXLDevRef dev_ref);
void qxl_notify_update(QXLDevRef dev_ref, uint32_t update_id);
void qxl_set_save_data(QXLDevRef dev_ref, void *data, int size);
void *qxl_get_save_data(QXLDevRef dev_ref);


QXLWorkerRef qxl_worker_init(QXLDevRef dev_ref, int channel_id);
void qxl_worker_attach(QXLWorkerRef worker_ref);
void qxl_worker_detach(QXLWorkerRef worker_ref);
void qxl_worker_wakeup(QXLWorkerRef worker_ref);
void qxl_worker_oom(QXLWorkerRef worker_ref);
void qxl_worker_save(QXLWorkerRef worker_ref);
void qxl_worker_load(QXLWorkerRef worker_ref);
void qxl_worker_start(QXLWorkerRef worker_ref);
void qxl_worker_stop(QXLWorkerRef worker_ref);
void qxl_worker_update_area(QXLWorkerRef worker_ref);

#endif


/*
 * QEMU live migration
 *
 * Copyright IBM, Corp. 2008
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#include "qemu-common.h"
#include "qemu_socket.h"
#include "migration.h"
#include "qemu-char.h"
#include "sysemu.h"
#include "console.h"
#include "buffered_file.h"
#include "block.h"

#ifdef CONFIG_SPICE
#include "interface.h"
#endif

//#define DEBUG_MIGRATION_TCP

typedef struct TCPIncommingState {
    int listener;
    int fd;
    QEMUFile *file;
} TCPIncommingState;

static TCPIncommingState in_state = { -1, -1, NULL};

#define MIGRATION_MAGIC (*(uint32_t*)"QMIG")
#define MIGRATION_VERSION 3
#define MIGRATION_HOOH_END_MAGIC (*(uint32_t*)"HEND")

#define TRUE 1
#define FALSE 0

/* Migration Notifiers events */
#define    MIGRATION_NOTIFY_EVENT_NONE     0
#define    MIGRATION_NOTIFY_EVENT_STARTED  1
#define    MIGRATION_NOTIFY_EVENT_FINISHED 2

typedef void (*migration_started_t)(void *opaque, const char *cmd);
typedef void (*migration_finished_t)(void *opaque, int completed);
typedef void (*migration_recv_t)(void *opaque, int fd);

typedef struct migration_notify_record_s {
    migration_started_t migration_started;
    migration_finished_t migration_finished;
    migration_recv_t migration_recv;
    void *opaque;
    char *key;
    struct migration_notify_record_s *next;
} migration_notify_record_t;

typedef void (*end_notifiers_call_t)(FdMigrationState *s);

typedef struct {
    migration_notify_record_t *next_notifier;
    migration_notify_record_t *notifiers;
    int notify_in_progress;
    char *cmd;
    int  event; /* MIGRATION_NOTIFY_EVENT_? */
    FdMigrationState *migration_state;
    end_notifiers_call_t on_notifiers_done;
} migration_notify_data_t;

static migration_notify_data_t migration_notify_data = {
    .next_notifier = NULL,
    .notifiers = NULL,
    .notify_in_progress = FALSE,
    .cmd = NULL,
    .event = MIGRATION_NOTIFY_EVENT_NONE,
    .migration_state = NULL,
    .on_notifiers_done = NULL,
};

#ifdef DEBUG_MIGRATION_TCP
#define dprintf(fmt, ...) \
    do { printf("migration-tcp: " fmt, ## __VA_ARGS__); } while (0)
#else
#define dprintf(fmt, ...) \
    do { } while (0)
#endif

static int socket_errno(FdMigrationState *s)
{
    return socket_error();
}

static void tcp_cleanup(FdMigrationState *s)
{
    migration_notify_data_t *data = &migration_notify_data;

    migrate_fd_cleanup(s);
    data->notify_in_progress = FALSE;
    data->next_notifier = NULL;
    data->event = MIGRATION_NOTIFY_EVENT_NONE;
    data->migration_state = NULL;
    data->on_notifiers_done = NULL;
    free(data->cmd);
    data->cmd = NULL;
}

static void migration_notify_call_handler(migration_notify_record_t *notifier)
{
    migration_notify_data_t *data = &migration_notify_data;

    if (data->event == MIGRATION_NOTIFY_EVENT_STARTED) {
        printf("%s: START BEFORE curr=%p event=%d opaque=%p fd=%d cmd=%s\n",
               __FUNCTION__, notifier, data->event, notifier->opaque, data->migration_state->fd, data->cmd);
        notifier->migration_started(notifier->opaque, data->cmd);
        printf("%s: START AFTER  curr=%p event=%d opaque=%p fd=%d cmd=%s\n",
               __FUNCTION__, notifier, data->event, notifier->opaque, data->migration_state->fd, data->cmd);
    } else if (data->event == MIGRATION_NOTIFY_EVENT_FINISHED) {
        printf("%s: FINISH curr=%p event=%d opaque=%p\n", __FUNCTION__, notifier, data->event, notifier->opaque);
        notifier->migration_finished(notifier->opaque, 
                                     data->migration_state->state == MIG_STATE_COMPLETED);
    } else {
        printf("%s: bad event\n", __FUNCTION__);
        abort();
    }
}

static migration_notify_record_t *migration_notify_next(void)
{
    migration_notify_data_t *data = &migration_notify_data;

    while (data->next_notifier) {
        migration_notify_record_t *now = data->next_notifier;
        data->next_notifier = now->next;
        if ((data->event == MIGRATION_NOTIFY_EVENT_STARTED && now->migration_started ) || 
            (data->event == MIGRATION_NOTIFY_EVENT_FINISHED && now->migration_finished)) {
            return now;
        }
    }
    return NULL;
}

static void migration_notify_done(void)
{
    migration_notify_data_t *data = &migration_notify_data;
    end_notifiers_call_t on_notifiers_done;

    printf("%s\n", __FUNCTION__);
    data->notify_in_progress = FALSE;
    data->next_notifier = NULL;
    on_notifiers_done = data->on_notifiers_done;
    data->on_notifiers_done = NULL;
    free(data->cmd);
    data->cmd = NULL;
    data->event = MIGRATION_NOTIFY_EVENT_NONE;
    on_notifiers_done(data->migration_state);
}

static void migration_notify(void)
{
    migration_notify_record_t *notifier = migration_notify_next();
    if (notifier)
        migration_notify_call_handler(notifier);
    else
        migration_notify_done();
}

static void tcp_notify_finish(FdMigrationState *s)
{
    migration_notify_data_t *data = &migration_notify_data;

    data->next_notifier = data->notifiers;
    data->notify_in_progress = TRUE;
    data->event = MIGRATION_NOTIFY_EVENT_FINISHED;
    data->on_notifiers_done = tcp_cleanup;
    migration_notify();    
}

static void tcp_migration_done(FdMigrationState *s, int state)
{
    migration_notify_data_t *data = &migration_notify_data;

    if (s->state != MIG_STATE_ACTIVE) {
        return;
    }
    s->state = state;

    if (!data->migration_state) {
        tcp_cleanup(s);
        return;
    }

    if (data->notify_in_progress) {
        data->on_notifiers_done = tcp_notify_finish;
        return;
    }
    tcp_notify_finish(s);
}

static int socket_write(FdMigrationState *s, const void * buf, size_t size)
{
    return send(s->fd, buf, size, 0);
}

static int tcp_close(FdMigrationState *s)
{
    dprintf("tcp_close\n");
    if (s->fd != -1) {
        close(s->fd);
        s->fd = -1;
    }
    return 0;
}

static void tcp_begin_send_vm(FdMigrationState *s)
{
    int ret;
    qemu_put_be32(s->file, 0);
    qemu_put_be32(s->file, MIGRATION_HOOH_END_MAGIC);
    dprintf("beginning savevm\n");
    s->save_started = TRUE;
    ret = qemu_savevm_state_begin(s->file);
    if (ret < 0) {
        dprintf("failed, %d\n", ret);
        migrate_fd_error(s);
        return;
    }
     migrate_fd_put_ready(s);
}

static void tcp_send_begin(FdMigrationState *s)
{
    migration_notify_data_t *data = &migration_notify_data;

    qemu_put_be32(s->file, MIGRATION_MAGIC);
    qemu_put_be32(s->file, MIGRATION_VERSION);
    data->migration_state = s;
    data->next_notifier = data->notifiers;
    data->notify_in_progress = TRUE;
    data->event = MIGRATION_NOTIFY_EVENT_STARTED;
    data->on_notifiers_done = tcp_begin_send_vm;
    migration_notify();
}

static void tcp_connect_migrate(FdMigrationState *s)
{
    s->file = qemu_fopen_ops_buffered(s,
                                      s->bandwidth_limit,
                                      migrate_fd_put_buffer,
                                      migrate_fd_put_ready,
                                      migrate_fd_wait_for_unfreeze,
                                      migrate_fd_close);
    if (!s->file) {
        printf("%s:  qemu fopen failed\n", __FUNCTION__);
        tcp_migration_done(s, MIG_STATE_ERROR);
        return;
    }
    tcp_send_begin(s);
}

static void tcp_wait_for_connect(void *opaque)
{
    FdMigrationState *s = opaque;
    int val, ret;
    socklen_t valsize = sizeof(val);

    dprintf("connect completed\n");
    do {
        ret = getsockopt(s->fd, SOL_SOCKET, SO_ERROR, &val, &valsize);
    } while (ret == -1 && (s->get_error(s)) == EINTR);

    if (ret < 0) {
        migrate_fd_error(s);
        return;
    }

    qemu_set_fd_handler2(s->fd, NULL, NULL, NULL, NULL);

    if (val == 0)
        tcp_connect_migrate(s);
    else {
        dprintf("error connecting %d\n", val);
        migrate_fd_error(s);
    }
}

static int tcp_release(MigrationState *mig_state)
{
    FdMigrationState *s = migrate_to_fms(mig_state);

    dprintf("releasing state\n");
    if (s->state == MIG_STATE_ACTIVE) {
        tcp_migration_done(s, MIG_STATE_CANCELLED);
        return FALSE;
    }
    if (migration_notify_data.migration_state) {
        return FALSE;
    }
    free(s);
    return TRUE;
}

static int tcp_get_status(MigrationState *mig_state)
{
    FdMigrationState *s = migrate_to_fms(mig_state);

    if (s->state == MIG_STATE_COMPLETED && 
                            migration_notify_data.notify_in_progress) {
        return MIG_STATE_ACTIVE;
    }

    return s->state;
}

/*
 * set tcp-keep-alive socket-option, to detect source/destination host crash
 * timeout upon idle connection is 2 minutes (give or take 10 seconds):
 *     60 seconds idle + 10 * 6 seconds intervals
 * returns 0 on success, -1 on error (same as setsockopt)
 */
static int tcp_migration_keepalive(int fd)
{
    const int idle  =  60; /* seconds */
    const int cnt   =  10; /* try 10 times */
    const int intvl =  6; /* wait for 6 seconds between tries */

    return tcp_set_keepalive(fd, idle, cnt, intvl);
}

MigrationState *tcp_start_outgoing_migration(const char *host_port,
                                             int64_t bandwidth_limit,
                                             int async)
{
    struct sockaddr_in addr;
    FdMigrationState *s;
    int ret;

    if (parse_host_port(&addr, host_port) < 0)
        return NULL;

    free(migration_notify_data.cmd);
    migration_notify_data.cmd = strdup(host_port);
    if (!migration_notify_data.cmd) {
        perror("migration_notify_started: could not copy migration command");
        return NULL;
    }

    s = qemu_mallocz(sizeof(*s));
    if (s == NULL)
        return NULL;

    s->get_error = socket_errno;
    s->write = socket_write;
    s->close = tcp_close;
    s->done = tcp_migration_done;

    s->mig_state.cancel = migrate_fd_cancel;
    s->mig_state.get_status = tcp_get_status;
    s->mig_state.release = tcp_release;

    s->state = MIG_STATE_ACTIVE;
    s->save_started = FALSE;
    s->detach = !async;
    s->bandwidth_limit = bandwidth_limit;
    s->fd = socket(PF_INET, SOCK_STREAM, 0);
    if (s->fd == -1) {
        qemu_free(s);
        return NULL;
    }

    if (tcp_migration_keepalive(s->fd)) {
        term_printf("tcp_keepalive: setsockopt() failed -  %s\n", strerror(errno));
    }

    socket_set_nonblock(s->fd);

    if (s->detach == 1) {
        dprintf("detaching from monitor\n");
        monitor_suspend();
        s->detach = 2;
    }

    do {
        ret = connect(s->fd, (struct sockaddr *)&addr, sizeof(addr));
        if (ret == -1)
            ret = -(s->get_error(s));

        if (ret == -EINPROGRESS || ret == -EWOULDBLOCK)
            qemu_set_fd_handler2(s->fd, NULL, NULL, tcp_wait_for_connect, s);
    } while (ret == -EINTR);

    if (ret < 0 && ret != -EINPROGRESS && ret != -EWOULDBLOCK) {
        dprintf("connect failed\n");
        close(s->fd);
        qemu_free(s);
        return NULL;
    } else if (ret >= 0)
        tcp_connect_migrate(s);

    return &s->mig_state;
}

static int read_all(int fd, void *in_nuf, int in_len)
{
    uint8_t *buf = in_nuf;
    int len = in_len;

    while (len > 0) {
        int ret;

        ret = read(fd, buf, len);
        if (ret < 0) {
            if (errno != EINTR)
                return -1;
        } else if (ret == 0) {
            break;
        } else {
            buf += ret;
            len -= ret;
        }
    }
    return in_len - len;
}

static uint32_t read_be32(int fd)
{
    uint32_t ret;
    if (read_all(fd, &ret, sizeof(uint32_t)) != sizeof(uint32_t)) {
        ret = 0;
    }
    return ntohl(ret);
}

static int tcp_recive_header(void)
{
    uint32_t magic = read_be32(in_state.fd);
    uint32_t version = read_be32(in_state.fd);
    return magic == MIGRATION_MAGIC && version == MIGRATION_VERSION;
}

static migration_notify_record_t* migration_notifier_find_by_key(char *key)
{
    migration_notify_record_t* notifier = migration_notify_data.notifiers;

    for (; notifier; notifier = notifier->next) {
        if (!strcmp(notifier->key,key)) {
            return notifier;
        }
    }
    return NULL;
}

static void tcp_incoming_cleanup(void)
{
    if (in_state.file) {
        qemu_fclose(in_state.file);
        in_state.file = NULL;
    }
    if (in_state.fd != -1) {
        qemu_set_fd_handler2(in_state.fd, NULL, NULL, NULL, NULL);
        close(in_state.fd);
        in_state.fd = -1;
    }
    monitor_resume();
}

static void tcp_incoming_load_vm(void)
{
    int ret = qemu_loadvm_state(in_state.file);
    if (ret < 0) {
        fprintf(stderr, "load of migration failed\n");
        goto error;
    }
    qemu_announce_self();
    dprintf("successfully loaded vm state\n");

    /* we've successfully migrated, close the server socket */
    qemu_set_fd_handler2(in_state.listener, NULL, NULL, NULL, NULL);
    close(in_state.listener);
    in_state.listener = -1;
    term_printf_async(MIGRATION_ASYNC_EVENT, "migration: migration process finished\n");
    if (drives_reopen() != 0) {
	    fprintf(stderr, "reopening of drives failed\n");
	    goto error;
    }
    vm_start();

error:
    tcp_incoming_cleanup();
}

static void tcp_incomming_run_hook(void)
{
    uint32_t len = read_be32(in_state.fd);
    migration_notify_record_t* notifier;
    char *hook_key = NULL;

    if (!len) {
        uint32_t hooks_end_magic = qemu_get_be32(in_state.file);
        if (hooks_end_magic != MIGRATION_HOOH_END_MAGIC) {
            fprintf(stderr, "%s: bad end of hooks\n", __FUNCTION__);
            goto error;
           
        }
        tcp_incoming_load_vm();
        return;
    }
    if (!(hook_key = malloc(len)) || read_all(in_state.fd, (uint8_t *)hook_key, len) != len) {
        fprintf(stderr, "recive hook key failed\n");
        goto error;
    }
    notifier = migration_notifier_find_by_key(hook_key);
    if (!notifier || !notifier->migration_recv) {
        fprintf(stderr, "%s: bad hook %s notifier %p recv %p\n", __FUNCTION__, hook_key, 
                notifier, (notifier) ? notifier->migration_recv : NULL);
        goto error;
    }
    free(hook_key);

    notifier->migration_recv(notifier->opaque, in_state.fd);
    return;

error:
    free(hook_key);
    tcp_incoming_cleanup();
}

static void tcp_accept_incoming_migration(void *opaque)
{
    struct sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);

    in_state.listener = (unsigned long)opaque;
    monitor_suspend();

    do {
        in_state.fd = accept(in_state.listener, (struct sockaddr *)&addr, &addrlen);
    } while (in_state.fd == -1 && socket_error() == EINTR);

    dprintf("accepted migration\n");
    
    if (in_state.fd == -1) {
        fprintf(stderr, "could not accept migration connection\n");
        goto error;
    }

    in_state.file = qemu_fopen_socket(in_state.fd);
    if (in_state.file == NULL) {
        fprintf(stderr, "could not qemu_fopen socket\n");
        goto error;
    }

    if (!tcp_recive_header()) {
        fprintf(stderr, "recice migration header failed\n");
        goto error;
    }
    tcp_incomming_run_hook();
    return;

error:
    tcp_incoming_cleanup();
}

int tcp_start_incoming_migration(const char *host_port)
{
    struct sockaddr_in addr;
    int val;
    int s;

    if (parse_host_port(&addr, host_port) < 0) {
        fprintf(stderr, "invalid host/port combination: %s\n", host_port);
        return -EINVAL;
    }

    s = socket(PF_INET, SOCK_STREAM, 0);
    if (s == -1)
        return -socket_error();

    val = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *)&val, sizeof(val));

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) == -1)
        goto err;

    if (listen(s, 1) == -1)
        goto err;

    qemu_set_fd_handler2(s, NULL, tcp_accept_incoming_migration, NULL,
                         (void *)(unsigned long)s);

    return 0;

err:
    close(s);
    return -socket_error();
}

static migration_notify_record_t *migration_notify_register(void *opaque,
                                                     const char* key,
                                                     migration_started_t mig_started,
                                                     migration_finished_t mig_finished,
                                                     migration_recv_t mig_recv)
{
    migration_notify_record_t *new;
    int extra_size;

    if (!key || !strlen(key)) {
        printf("%s: bad key\n", __FUNCTION__);
        return NULL;
    }
    
    extra_size = strlen(key) + 1;
    new = (migration_notify_record_t *) qemu_mallocz(sizeof(migration_notify_record_t) + extra_size);

    if (!new)
        return NULL;
    printf("%s: record=%p opaque=%p key=%s mstarted=%p mfinished=%p mig_recv=%p\n",
           __FUNCTION__, new, opaque, key, mig_started, mig_finished, mig_recv);
    new->opaque = opaque;
    new->key = (char *)(new + 1);
    strcpy(new->key, key);
    new->migration_started  = mig_started;
    new->migration_finished = mig_finished;
    new->migration_recv = mig_recv;
    new->next = NULL;
    if (migration_notify_data.notifiers == NULL)
        migration_notify_data.notifiers = new;
    else {
        migration_notify_record_t *prev = migration_notify_data.notifiers;
        while (prev->next != NULL)
            prev = prev->next;
        prev->next = new;
    }
    return new;
}

#ifdef CONFIG_SPICE

static VDObjectRef interface_register_notifiers(MigrationInterface* mig, const char *key,
                                                migration_notify_started_t mig_started,
                                                migration_notify_finished_t mig_finished,
                                                migration_notify_recv_t mig_recv,
                                                void *opaque)
{
    return (VDObjectRef)migration_notify_register(opaque, key, mig_started, mig_finished, mig_recv);
}


static void interface_unregister_notifiers(MigrationInterface* mig, VDObjectRef notifier_ref)
{
    migration_notify_record_t* notifier = (migration_notify_record_t *)notifier_ref;
    migration_notify_record_t** now = &migration_notify_data.notifiers;

    if (!notifier) {
        return;
    }

    while (*now) {
        if (*now == notifier) {
            *now = notifier->next;
            if (notifier == migration_notify_data.next_notifier) {
                migration_notify_data.next_notifier = notifier->next;
            }
            free(notifier);
            return;
        }
    }
}

static void continue_recv_hooks(void *opaque)
{
    qemu_set_fd_handler(in_state.fd, NULL, NULL, NULL);
    tcp_incomming_run_hook();
}

static void continue_nofify(void *opaque)
{
    QEMUBH **qemu_bh_link = (QEMUBH **)opaque;
    qemu_bh_delete(*qemu_bh_link);
    free(qemu_bh_link);
    migration_notify();
}

static void interface_notifier_done(MigrationInterface *mig, VDObjectRef notifier_ref)
{
    if (!migration_notify_data.migration_state) {
        qemu_set_fd_handler(in_state.fd, continue_recv_hooks, NULL, NULL);
        return;
    }
    QEMUBH **qemu_bh_link = malloc(sizeof(QEMUBH *));
    QEMUBH *qemu_bh = qemu_bh_new(continue_nofify, qemu_bh_link);
    *qemu_bh_link = qemu_bh;
    qemu_bh_schedule(qemu_bh);
}

static int interface_begin_hook(MigrationInterface *mig, VDObjectRef notifier_ref)
{
    migration_notify_record_t* notifier = (migration_notify_record_t *)notifier_ref;
    QEMUFile *file = migration_notify_data.migration_state->file;
    int len = strlen(notifier->key) + 1;

    qemu_put_be32(file, len);
    qemu_put_buffer(file, (uint8_t *)notifier->key, len);
    qemu_fflush(file);
    return migration_notify_data.migration_state->fd;
}   

void tcp_migration_register_interface()
{
    MigrationInterface *interface = (MigrationInterface *)qemu_mallocz(sizeof(*interface));

    if (!interface) {
        printf("%s: malloc failed\n", __FUNCTION__);
        exit(-1);
    }
    interface->base.base_vertion = VM_INTERFACE_VERTION;
    interface->base.type = VD_INTERFACE_MIGRATION;
    interface->base.id = 0;
    interface->base.description = "qemue migration";
    interface->base.major_vertion = VD_INTERFACE_MIGRATION_MAJOR;
    interface->base.minor_vertion = VD_INTERFACE_MIGRATION_MINOR;
    interface->register_notifiers = interface_register_notifiers;
    interface->unregister_notifiers = interface_unregister_notifiers;
    interface->notifier_done = interface_notifier_done;
    interface->begin_hook = interface_begin_hook;
    add_interface(&interface->base);
}

#endif



#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "interface.h"
#include "qemu-common.h"
#include "qemu-timer.h"
#include "console.h"

typedef struct InterfacesNotifier InterfacesNotifier;

struct InterfacesNotifier {
    vd_interface_change_notifier_t notifier;
    void *opaque;
    int refs;
    InterfacesNotifier *next;
};

InterfacesNotifier *notifiers = NULL;

typedef struct VDInterfaceLink VDInterfaceLink;

struct VDInterfaceLink {
    VDInterface *interface;
    VDInterfaceLink *next;
};

VDInterfaceLink* interfaces = NULL;

static void hold_notifier(InterfacesNotifier* notifier)
{
    notifier->refs++;
}

static void releas_notifier(InterfacesNotifier* notifier)
{
    if (--notifier->refs) {
        return;
    }
    InterfacesNotifier **now = &notifiers;

    while (*now != notifier) {
        now = &(*now)->next;
    }
    *now = notifier->next;
    free(notifier);
}

static void run_notifiers(VDInterface *interface, VDInterfaceChangeType type)
{
    InterfacesNotifier *now;
    InterfacesNotifier *prev;

    now = notifiers;
    while (now) {
        hold_notifier(now);
        if (now->notifier) {
            now->notifier(now->opaque, interface, type);
        }
        prev = now;
        now = now->next;
        releas_notifier(prev);
    }
}

void add_interface(VDInterface *interface)
{
    VDInterfaceLink *holder;

    if (!interface) {
        return;
    }

    if (!(holder = malloc(sizeof(*holder)))) {
        printf("%s: malloc failed\n", __FUNCTION__);
        exit(-1);
    }
    memset(holder, 0, sizeof(*holder));
    holder->interface = interface;
    holder->next = interfaces;
    interfaces = holder;
    run_notifiers(interface, VD_INTERFACE_ADDING);
}

void remove_interface(VDInterface *interface)
{
    VDInterfaceLink **now;

    if (!interface) {
        return;
    }
    now = &interfaces;
    while (*now) {
        if ((*now)->interface == interface) {
            VDInterfaceLink *found = *now;
            *now = found->next;
            free(found);
            run_notifiers(interface, VD_INTERFACE_REMOVING);
            return;
        }
        now = &(*now)->next;
    }
    printf("%s: not found\n", __FUNCTION__);
}

static VDInterface *firest_valid_interface(VDInterfaceLink* interface_holder)
{
    while (interface_holder) {
        if (interface_holder->interface) {
            return interface_holder->interface;
        }
        interface_holder = interface_holder->next;
    }
    return NULL;
}

static VDInterface *core_next(CoreInterface *core, VDInterface *prev)
{
    VDInterfaceLink* now;

    if (!interfaces) {
        return NULL;
    }

    if (!prev) {
        return firest_valid_interface(interfaces);
    }

    now = interfaces;
    while (now->next) {
        if (now->interface == prev) {
            return firest_valid_interface(now->next);
        }
    }
    return NULL;
}

static void core_unregister_change_notifiers(CoreInterface *core, VDObjectRef notifier)
{
    InterfacesNotifier *to_remove = (InterfacesNotifier *)notifier;
    InterfacesNotifier *now;

    if (!to_remove) {
        return;
    }
    now = notifiers;
    while (now) {
        if (now == to_remove && now->notifier) {
            now->notifier = NULL;
            releas_notifier(now);
            return;
        }
        now = now->next;
    }
    printf("%s: not found\n", __FUNCTION__);
}

static VDObjectRef core_register_change_notifiers(CoreInterface *core, void *opaque,
                                    vd_interface_change_notifier_t notifier)
{
    InterfacesNotifier *_notifie;

    if (!notifier) {
        printf("%s: invalid args\n", __FUNCTION__);
        return 0;
    }

    _notifie = malloc(sizeof(*_notifie));
    if (!_notifie) {
        printf("%s: malloc failed\n", __FUNCTION__);
        exit(-1);
    }
    memset(_notifie, 0, sizeof(*_notifie));
    _notifie->refs = 1;
    _notifie->opaque = opaque;
    _notifie->notifier = notifier;
    _notifie->next = notifiers;
    notifiers = _notifie;

    return (VDObjectRef)_notifie;
}

static VDObjectRef core_create_timer(CoreInterface *core, timer_callback_t callback, void *opaue)
{
    return (VDObjectRef)qemu_new_timer(rt_clock, callback, opaue);
}

static void core_arm_timer(CoreInterface *core, VDObjectRef timer, uint32_t ms)
{
    qemu_mod_timer((QEMUTimer *)timer, qemu_get_clock(rt_clock) + ms);
}

static void core_disarm_timer(CoreInterface *core, VDObjectRef timer)
{
    qemu_del_timer((QEMUTimer *)timer);
}

static void core_destroy_timer(CoreInterface *core, VDObjectRef timer)
{
    qemu_del_timer((QEMUTimer *)timer);
    qemu_free_timer((QEMUTimer *)timer);
}

static int core_set_file_handlers(CoreInterface *core, int fd,
                              void (*on_read)(void *),
                              void (*on_write)(void *),
                              void *opaque)
{
    return qemu_set_fd_handler(fd, on_read, on_write, opaque);
}

static void core_term_printf(CoreInterface *core, const char* format, ...)
{
    va_list ap;

    va_start(ap, format);
    term_vprintf(format, ap);
    va_end(ap);
}

CoreInterface core_interface = {
    {VM_INTERFACE_VERTION, VD_INTERFACE_CORE, 0, "qwnu core services", VD_INTERFACE_CORE_MAJOR, VD_INTERFACE_CORE_MINOR},
    core_next,
    core_register_change_notifiers,
    core_unregister_change_notifiers,
    core_create_timer,
    core_arm_timer,
    core_disarm_timer,
    core_destroy_timer,
    core_set_file_handlers,
    core_term_printf,
};

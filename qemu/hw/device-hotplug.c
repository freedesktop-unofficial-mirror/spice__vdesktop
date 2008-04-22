#include "hw.h"
#include "boards.h"
#include "pci.h"
#include "net.h"
#include "sysemu.h"
#include "pc.h"
#include "console.h"
#include "block_int.h"

#define PCI_BASE_CLASS_STORAGE          0x01
#define PCI_BASE_CLASS_NETWORK          0x02

static PCIDevice *qemu_system_hot_add_nic(const char *opts, int bus_nr)
{
    int ret;
    char buf[4096];
    PCIBus *pci_bus;

    pci_bus = pci_find_bus (bus_nr);
    if (!pci_bus) {
        term_printf ("Can't find pci_bus %d\n", bus_nr);
        return NULL;
    }

    memset (buf, 0, sizeof (buf));

    strcpy (buf, "nic,");
    strncat (buf, opts, sizeof (buf) - strlen (buf) - 1);

    ret = net_client_init (buf);
    if (ret < 0 || !nd_table[ret].model)
        return NULL;
    return pci_nic_init (pci_bus, &nd_table[ret], -1);
}

static int add_init_drive(const char *opts)
{
    int drive_opt_idx, drive_idx;
    int ret = -1;

    drive_opt_idx = drive_add(NULL, "%s", opts);
    if (!drive_opt_idx)
        return ret;

    drive_idx = drive_init(&drives_opt[drive_opt_idx], 0, current_machine);
    if (drive_idx == -1) {
        drive_remove(drive_opt_idx);
        return ret;
    }

    return drive_idx;
}

void drive_hot_add(int pcibus, const char *devfn_string, const char *opts)
{
    int drive_idx, type, bus;
    int devfn;
    int success = 0;
    PCIDevice *dev;

    devfn = strtoul(devfn_string, NULL, 0);

    dev = pci_find_device(pcibus, PCI_SLOT(devfn));
    if (!dev) {
        term_printf("no pci device with devfn %d (slot %d)\n", devfn,
                    PCI_SLOT(devfn));
        return;
    }

    drive_idx = add_init_drive(opts);
    if (drive_idx < 0)
        return;
    type = drives_table[drive_idx].type;
    bus = drive_get_max_bus (type);

    switch (type) {
    case IF_SCSI:
        success = 1;
        lsi_scsi_attach (dev, drives_table[drive_idx].bdrv,
                         drives_table[drive_idx].unit);
        break;
    default:
        term_printf("Can't hot-add drive to type %d\n", type);
    }

    if (success)
        term_printf("OK bus %d, unit %d\n", drives_table[drive_idx].bus,
                                            drives_table[drive_idx].unit);
    return;
}

static PCIDevice *qemu_system_hot_add_storage(const char *opts, int bus_nr)
{
    void *opaque = NULL;
    PCIBus *pci_bus;
    int type = -1, drive_idx = -1;
    char buf[128];

    pci_bus = pci_find_bus(bus_nr);
    if (!pci_bus) {
        term_printf("Can't find pci_bus %d\n", bus_nr);
        return NULL;
    }

    if (get_param_value(buf, sizeof(buf), "if", opts)) {
        if (!strcmp(buf, "scsi"))
            type = IF_SCSI;
        else if (!strcmp(buf, "virtio")) {
            type = IF_VIRTIO;
        }
    } else {
        term_printf("no if= specified\n");
        return NULL;
    }

    if (get_param_value(buf, sizeof(buf), "file", opts)) {
        drive_idx = add_init_drive(opts);
        if (drive_idx < 0)
            return NULL;
    } else if (type == IF_VIRTIO) {
        term_printf("virtio requires a backing file/device.\n");
        return NULL;
    }

    switch (type) {
    case IF_SCSI:
        opaque = lsi_scsi_init (pci_bus, -1);
        if (opaque && drive_idx >= 0)
            lsi_scsi_attach (opaque, drives_table[drive_idx].bdrv,
                             drives_table[drive_idx].unit);
        break;
    case IF_VIRTIO:
        opaque = virtio_blk_init (pci_bus, 0x1AF4, 0x1001,
                                  drives_table[drive_idx].bdrv);
        break;
    default:
        term_printf ("type %s not a hotpluggable PCI device.\n", buf);
    }

    return opaque;
}

#if defined(TARGET_I386) || defined(TARGET_X86_64)
void device_hot_add(int pcibus, const char *type, const char *opts)
{
    PCIDevice *dev = NULL;

    if (strcmp(type, "nic") == 0)
        dev = qemu_system_hot_add_nic(opts, pcibus);
    else if (strcmp(type, "storage") == 0)
        dev = qemu_system_hot_add_storage(opts, pcibus);
    else
        term_printf("invalid type: %s\n", type);

    if (dev) {
        qemu_system_device_hot_add(pcibus, PCI_SLOT(dev->devfn), 1);
        term_printf("OK bus %d, slot %d, function %d (devfn %d)\n",
                    pci_bus_num(dev->bus), PCI_SLOT(dev->devfn),
                    PCI_FUNC(dev->devfn), dev->devfn);
    } else
        term_printf("failed to add %s\n", opts);
}

void device_hot_remove(int pcibus, int slot)
{
    PCIDevice *d = pci_find_device(pcibus, slot);

    if (!d) {
        term_printf("invalid slot %d\n", slot);
        return;
    }

    qemu_system_device_hot_add(pcibus, slot, 0);
}
#endif

static void destroy_nic(int slot)
{
    int i;

    for (i = 0; i < MAX_NICS; i++)
        if (nd_table[i].used &&
            PCI_SLOT(nd_table[i].devfn) == slot)
                net_client_uninit(&nd_table[i]);
}

static void destroy_bdrvs(int slot)
{
    int i;
    struct BlockDriverState *bs;

    for (i = 0; i <= MAX_DRIVES; i++) {
        bs = drives_table[i].bdrv;
        if (bs && (PCI_SLOT(bs->devfn) == slot)) {
            drive_uninit(bs);
            bdrv_delete(bs);
        }
    }
}

/*
 * OS has executed _EJ0 method, we now can remove the device
 */
void device_hot_remove_success(int pcibus, int slot)
{
    PCIDevice *d = pci_find_device(pcibus, slot);
    int class_code;

    if (!d) {
        term_printf("invalid slot %d\n", slot);
        return;
    }

    class_code = d->config_read(d, PCI_CLASS_DEVICE+1, 1);

    pci_unregister_device(d);

    switch(class_code) {
    case PCI_BASE_CLASS_STORAGE:
        destroy_bdrvs(slot);
        break;
    case PCI_BASE_CLASS_NETWORK:
        destroy_nic(slot);
        break;
    }

}


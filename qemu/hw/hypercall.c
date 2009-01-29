/*
 * QEMU-KVM Hypercall emulation
 * 
 * Copyright (c) 2003-2004 Fabrice Bellard
 * Copyright (c) 2006 Qumranet
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "hw/hw.h"
#include "sysemu.h"
#include "qemu-char.h"
#include "hw/isa.h"
#include "hw/irq.h"
#include "hw/pci.h"
#include "hypercall.h"
#include <stddef.h>

#define HYPERCALL_IOPORT_SIZE 0x100

static int use_hypercall_dev = 0;

typedef struct VmChannelCharDriverState {
    CharDriverState *vmchannel_hd;
    uint32_t deviceid;
} VmChannelCharDriverState;

static VmChannelCharDriverState vmchannel_hds[MAX_VMCHANNEL_DEVICES];

typedef struct HypercallState {
    uint32_t hcr;
    uint32_t hsr;
    uint32_t txsize;
    uint32_t txbuff;
    uint32_t rxsize;
    uint32_t version;
    uint8_t  RxBuff[HP_MEM_SIZE];
    uint8_t  txbufferaccu[HP_MEM_SIZE];
    int      txbufferaccu_offset;
    int      irq;
    PCIDevice *pci_dev;
    uint32_t index;
    uint32_t driver_state;
} HypercallState;

static HypercallState *pHypercallStates[MAX_VMCHANNEL_DEVICES] = {NULL};

//#define HYPERCALL_DEBUG 1

static void hypercall_update_irq(HypercallState *s);

static void hp_reset(HypercallState *s)
{
    s->hcr = 0;
    s->hsr = 0;
    s->txsize = 0;
    s->txbuff = 0;
    s->rxsize= 0;
    s->txbufferaccu_offset = 0;
    s->version = HP_VERSION;
    s->driver_state = 0;
    hypercall_update_irq(s);
}

static void hp_ioport_write(void *opaque, uint32_t addr, uint32_t val)
{
    HypercallState *s = opaque;

#ifdef HYPERCALL_DEBUG
    printf("%s: addr=0x%x, val=0x%x\n", __FUNCTION__, addr, val);
#endif
    addr &= 0xff;

    switch(addr)
    {
	case HSR_REGISTER: {
                s->hsr = (s->hsr & ~HSR_PENDING) + (val & HSR_PENDING);
                hypercall_update_irq(s);
        }
        break;
        case HCR_REGISTER: {
                s->hcr = (s->hcr & ~HCR_IRQ_ON) + (val & HCR_IRQ_ON);
                hypercall_update_irq(s);
        }
        break;
        case HP_VERSION_REGISTER:
                if (s->version == val)
                    s->driver_state |= HP_DRIVER_INITELIZED;
                else 
                    printf("%s:version mismatch: ours %d, driver's %d\n", __FUNCTION__, s->version, val);
        break;
        case HP_TXSIZE:
        {
            // handle the case when the we are being called when txsize is not 0
            if (s->txsize != 0) {
                printf("txsize is being set, but txsize is not 0!!!\n");
            }
            if (val > HP_MEM_SIZE) {
                printf("txsize is larger than allowed by hw!!!\n");
                break;
            }
            s->txsize = val;
            s->txbufferaccu_offset = 0;
            break;
        }

        case HP_TXBUFF:
        {
            if (s->txsize == 0) {
                printf("error with txbuff!!!\n");
                break;
            }

            s->txbufferaccu[s->txbufferaccu_offset] = val;
            s->txbufferaccu_offset++;
            if (s->txbufferaccu_offset >= s->txsize) {
		int rs;
                rs = qemu_chr_write(vmchannel_hds[s->index].vmchannel_hd, s->txbufferaccu, s->txsize);
		if (rs < 0)
			printf("%s:error from qemu_chr_write\n", __FUNCTION__);
		else if (rs != s->txsize)
			printf("%s:error from qemu_chr_write, read %d/%d bytes\n", __FUNCTION__, rs, s->txsize);
                s->txbufferaccu_offset = 0;
                s->txsize = 0;
            }
            break;
        }
        default:
        {
            printf("hp_ioport_write to unhandled address!!!\n");
        }
    }
}

static uint32_t hp_ioport_read(void *opaque, uint32_t addr)
{
    HypercallState *s = opaque;
    int ret;

    addr &= 0xff;
#ifdef HYPERCALL_DEBUG
        printf("%s: addr=0x%x", __FUNCTION__, addr);
#endif

    if (addr >= offsetof(HypercallState, RxBuff) )
    {
        int RxBuffOffset = addr - (offsetof(HypercallState, RxBuff));
        ret = s->RxBuff[RxBuffOffset];
#ifdef HYPERCALL_DEBUG
    printf(" val=%x\n", ret);
#endif
        return ret;
    }

    switch (addr)
    {
    case HP_VERSION_REGISTER:
	ret = s->version;
	break;
     case HSR_REGISTER:
        ret = s->hsr;
        break;
     case HCR_REGISTER:
        ret = s->hcr;
        break;
    case HP_RXSIZE:
        ret = s->rxsize;
        break;

    default:
        ret = 0x00;
        break;
    }
#ifdef HYPERCALL_DEBUG
    printf(" val=%x\n", ret);
#endif
    return ret;
}

/***********************************************************/
/* PCI Hypercall definitions */

typedef struct PCIHypercallState {
    PCIDevice dev;
    HypercallState hp;
} PCIHypercallState;

static void hp_map(PCIDevice *pci_dev, int region_num, 
                       uint32_t addr, uint32_t size, int type)
{
    PCIHypercallState *d = (PCIHypercallState *)pci_dev;
    HypercallState *s = &d->hp;

    register_ioport_write(addr, HYPERCALL_IOPORT_SIZE, 1, hp_ioport_write, s);
    register_ioport_read(addr, HYPERCALL_IOPORT_SIZE, 1, hp_ioport_read, s);

}


static void hypercall_update_irq(HypercallState *s)
{
    int irq_level = (s->hsr & HSR_PENDING) && (s->hcr & HCR_IRQ_ON);

    if (!(s->driver_state & HP_DRIVER_INITELIZED))
        return;

#ifdef HYPERCALL_DEBUG
    printf("%s, irq_level=%d ,hsr=%d hcr=%d\n", __FUNCTION__, irq_level, s->hsr, s->hcr);
#endif
    qemu_set_irq(s->pci_dev->irq[s->pci_dev->config[0x3d]-1], irq_level != 0);
}

static void hc_save(QEMUFile* f,void* opaque)
{
    HypercallState* s=(HypercallState*)opaque;

    pci_device_save(s->pci_dev, f);

    qemu_put_be32s(f, &s->hcr);
    qemu_put_be32s(f, &s->hsr);
    qemu_put_be32s(f, &s->txsize);
    qemu_put_be32s(f, &s->txbuff);
    qemu_put_be32s(f, &s->rxsize);
    qemu_put_buffer(f, s->RxBuff, HP_MEM_SIZE);
    qemu_put_buffer(f, s->txbufferaccu, HP_MEM_SIZE);
    qemu_put_be32s(f, &s->txbufferaccu_offset);
    qemu_put_be32s(f, &s->irq);
    qemu_put_be32s(f, &s->index);
    qemu_put_be32s(f, &s->driver_state);
}

static int hc_load(QEMUFile* f,void* opaque,int version_id)
{
    HypercallState* s=(HypercallState*)opaque;
    int ret;

    if (version_id > 2)
        return -EINVAL;

    ret = pci_device_load(s->pci_dev, f);
    if (ret < 0)
        return ret;

    qemu_get_be32s(f, &s->hcr);
    qemu_get_be32s(f, &s->hsr);
    qemu_get_be32s(f, &s->txsize);
    qemu_get_be32s(f, &s->txbuff);
    qemu_get_be32s(f, &s->rxsize);
    qemu_get_buffer(f, s->RxBuff, HP_MEM_SIZE);
    qemu_get_buffer(f, s->txbufferaccu, HP_MEM_SIZE);
    qemu_get_be32s(f, &s->txbufferaccu_offset);
    qemu_get_be32s(f, &s->irq);
    qemu_get_be32s(f, &s->index);

    if (version_id >= 2)
        qemu_get_be32s(f, &s->driver_state);
    else
        s->driver_state = 0;

    return 0;
}

static void pci_hypercall_single_init(PCIBus *bus, uint32_t deviceid, uint32_t index,int devfn)
{
    PCIHypercallState *d;
    HypercallState *s;
    uint8_t *pci_conf;
    char name[sizeof("HypercallX")];

    sprintf(name, "Hypercall%d", index);

#ifdef HYPERCALL_DEBUG
    printf("%s, devicename:%s\n", __FUNCTION__, name);
#endif

    // If the vmchannel wasn't initialized, we don't want the Hypercall device in the guest
    if (use_hypercall_dev == 0) {
        return;
    }

    d = (PCIHypercallState *)pci_register_device(bus,
                                                 name, sizeof(PCIHypercallState),
                                                 devfn,
                                                 NULL, NULL);
    if (!d) {
        fprintf(stderr, "Failed to register PCI device for vmchannel\n");
        return;
    }


    pci_conf = d->dev.config;
    pci_conf[0x00] = 0x02; // Qumranet vendor ID 0x5002
    pci_conf[0x01] = 0x50;
    pci_conf[0x02] = deviceid & 0x00ff;
    pci_conf[0x03] = (deviceid & 0xff00) >> 8;

    pci_conf[0x09] = 0x00; // ProgIf
    pci_conf[0x0a] = 0x00; // SubClass
    pci_conf[0x0b] = 0x05; // BaseClass

    pci_conf[0x0e] = 0x00; // header_type
    pci_conf[0x3d] = 2; // interrupt pin 1

    pci_register_io_region(&d->dev, 0, HYPERCALL_IOPORT_SIZE,
                           PCI_ADDRESS_SPACE_IO, hp_map);
    s = &d->hp;
    pHypercallStates[index] = s;
    s->index = index;
    s->irq = 16; /* PCI interrupt */
    s->pci_dev = &d->dev;

    hp_reset(s);
    qemu_register_reset(hp_reset, s);
    register_savevm(name, index, 2, hc_save, hc_load, s);
}

int pci_hypercall_init(PCIBus *bus,int devfn)
{
    int i;

    // loop devices & call pci_hypercall_single_init with device id's
    for(i = 0; i < MAX_VMCHANNEL_DEVICES; i++){
        if (vmchannel_hds[i].vmchannel_hd) {
            pci_hypercall_single_init(bus, vmchannel_hds[i].deviceid, i,++devfn);
        }
    }

	return devfn;
}


// If the VDR is set, data is waiting to be read by the guest.
// don't let the vmchannel device write into
// the buffer since the guest might be in the middle of a read
static int vmchannel_can_read(void *opaque)
{
    long index = (long)opaque;
    if (pHypercallStates[index]->hsr & HSR_PENDING)
        return 0;
    else
        return HP_MEM_SIZE;
}

static void vmchannel_event(void *opaque, int event)
{

#ifdef HYPERCALL_DEBUG
    // if index is to be used outside the printf, take it out of the #ifdef block!
    long index = (long)opaque;
    printf("%s index:%ld, got event %i\n", __FUNCTION__, index, event);
#endif
    
    return;
}

// input from vmchannel device in order of inject it into the guest
static void vmchannel_read(void *opaque, const uint8_t *buf, int size)
{
    int i;
    long index = (long)opaque;

#ifdef HYPERCALL_DEBUG    
    printf("vmchannel_read buf size:%d\n", size);
#endif

    // if the hypercall device is in interrupts disabled state, don't accept the data
    if (!(pHypercallStates[index]->hcr & HCR_IRQ_ON)) {
        printf("%s: error: got read during interrupt disabled\n", __FUNCTION__);
        return;
    }

    for(i = 0; i < size; i++) {
        pHypercallStates[index]->RxBuff[i] = buf[i];
    }
    pHypercallStates[index]->rxsize = size;
    pHypercallStates[index]->hsr |= HSR_PENDING;
    hypercall_update_irq(pHypercallStates[index]);
}

void vmchannel_init(CharDriverState *hd, uint32_t deviceid, uint32_t index)
{
#ifdef HYPERCALL_DEBUG
    printf("vmchannel_init, index=%d, deviceid=0x%x\n", index, deviceid);
#endif

    vmchannel_hds[index].deviceid = deviceid;
    vmchannel_hds[index].vmchannel_hd = hd;
   
    use_hypercall_dev = 1;
    qemu_chr_add_handlers(vmchannel_hds[index].vmchannel_hd, vmchannel_can_read, vmchannel_read,
                          vmchannel_event, (void *)(long)index);
}

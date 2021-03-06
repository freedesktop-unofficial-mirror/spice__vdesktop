/*
 * Virtio Network Device
 *
 * Copyright IBM, Corp. 2007
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#include "virtio.h"
#include "net.h"
#include "qemu-timer.h"
#include "virtio-net.h"
#ifdef USE_KVM
#include "qemu-kvm.h"
#endif

#define TAP_VNET_HDR

typedef struct VirtIONet
{
    VirtIODevice vdev;
    uint8_t mac[6];
    uint16_t status;
    VirtQueue *rx_vq;
    VirtQueue *tx_vq;
    VLANClientState *vc;
    QEMUBH *tx_bh;
    int tx_bh_scheduled;
    struct {
        VirtQueueElement elem;
        ssize_t len;
        struct iovec *out_sg;
        unsigned int out_num;
    } tx_queue;
    int mergeable_rx_bufs;
} VirtIONet;

/* TODO
 * - we could suppress RX interrupt if we were so inclined.
 */

static VirtIONet *to_virtio_net(VirtIODevice *vdev)
{
    return (VirtIONet *)vdev;
}

static void virtio_net_update_config(VirtIODevice *vdev, uint8_t *config)
{
    VirtIONet *n = to_virtio_net(vdev);
    struct virtio_net_config netcfg;

    netcfg.status = n->status;
    memcpy(netcfg.mac, n->mac, 6);
    memcpy(config, &netcfg, sizeof(netcfg));
}

static void virtio_net_set_link_status(VLANClientState *vc)
{
    VirtIONet *n = vc->opaque;
    uint16_t old_status = n->status;

    if (vc->link_down)
        n->status &= ~VIRTIO_NET_S_LINK_UP;
    else
        n->status |= VIRTIO_NET_S_LINK_UP;

    if (n->status != old_status)
        virtio_notify_config(&n->vdev);
}

static uint32_t virtio_net_get_features(VirtIODevice *vdev)
{
    uint32_t features = (1 << VIRTIO_NET_F_MAC) | (1 << VIRTIO_NET_F_STATUS);
#ifdef TAP_VNET_HDR
    VirtIONet *n = to_virtio_net(vdev);
    VLANClientState *host = n->vc->vlan->first_client;

    if (tap_has_vnet_hdr(host)) {
        tap_using_vnet_hdr(host, 1);
        features |= (1 << VIRTIO_NET_F_CSUM);
        features |= (1 << VIRTIO_NET_F_GUEST_CSUM);
        features |= (1 << VIRTIO_NET_F_GUEST_TSO4);
        features |= (1 << VIRTIO_NET_F_GUEST_TSO6);
        features |= (1 << VIRTIO_NET_F_GUEST_ECN);
        features |= (1 << VIRTIO_NET_F_HOST_TSO4);
        features |= (1 << VIRTIO_NET_F_HOST_TSO6);
        features |= (1 << VIRTIO_NET_F_HOST_ECN);
        features |= (1 << VIRTIO_NET_F_MRG_RXBUF);
        /* Kernel can't actually handle UFO in software currently. */
    }
#endif

    return features;
}

static uint32_t virtio_net_bad_features(VirtIODevice *vdev)
{
    uint32_t features = 0;

    /* Linux kernel 2.6.25.  It understood MAC (as everyone must),
     * but also these: */
    features |= (1 << VIRTIO_NET_F_MAC);
    features |= (1 << VIRTIO_NET_F_GUEST_CSUM);
    features |= (1 << VIRTIO_NET_F_GUEST_TSO4);
    features |= (1 << VIRTIO_NET_F_GUEST_TSO6);
    features |= (1 << VIRTIO_NET_F_GUEST_ECN);

    return features & virtio_net_get_features(vdev);
}

static void virtio_net_set_features(VirtIODevice *vdev, uint32_t features)
{
    VirtIONet *n = to_virtio_net(vdev);
#ifdef TAP_VNET_HDR
    VLANClientState *host = n->vc->vlan->first_client;
#endif

    n->mergeable_rx_bufs = !!(features & (1 << VIRTIO_NET_F_MRG_RXBUF));

#ifdef TAP_VNET_HDR
    if (!tap_has_vnet_hdr(host) || !host->set_offload)
        return;

    host->set_offload(host,
                      (features >> VIRTIO_NET_F_GUEST_CSUM) & 1,
                      (features >> VIRTIO_NET_F_GUEST_TSO4) & 1,
                      (features >> VIRTIO_NET_F_GUEST_TSO6) & 1,
                      (features >> VIRTIO_NET_F_GUEST_ECN)  & 1);
#endif
}

/* RX */

static void virtio_net_handle_rx(VirtIODevice *vdev, VirtQueue *vq)
{
#ifdef USE_KVM
    /* We now have RX buffers, signal to the IO thread to break out of the
       select to re-poll the tap file descriptor */
    if (kvm_enabled())
        qemu_kvm_notify_work();
#endif
}

static int do_virtio_net_can_receive(VirtIONet *n, int bufsize)
{
    if (!virtio_queue_ready(n->rx_vq) ||
        !(n->vdev.status & VIRTIO_CONFIG_S_DRIVER_OK))
        return 0;

    if (virtio_queue_empty(n->rx_vq) ||
        (n->mergeable_rx_bufs &&
         !virtqueue_avail_bytes(n->rx_vq, bufsize, 0))) {
        virtio_queue_set_notification(n->rx_vq, 1);
        return 0;
    }

    virtio_queue_set_notification(n->rx_vq, 0);
    return 1;
}

static int virtio_net_can_receive(void *opaque)
{
    VirtIONet *n = opaque;

    return do_virtio_net_can_receive(n, VIRTIO_NET_MAX_BUFSIZE);
}

#ifdef TAP_VNET_HDR
/* dhclient uses AF_PACKET but doesn't pass auxdata to the kernel so
 * it never finds out that the packets don't have valid checksums.  This
 * causes dhclient to get upset.  Fedora's carried a patch for ages to
 * fix this with Xen but it hasn't appeared in an upstream release of
 * dhclient yet.
 *
 * To avoid breaking existing guests, we catch udp packets and add
 * checksums.  This is terrible but it's better than hacking the guest
 * kernels.
 *
 * N.B. if we introduce a zero-copy API, this operation is no longer free so
 * we should provide a mechanism to disable it to avoid polluting the host
 * cache.
 */
static void work_around_broken_dhclient(struct virtio_net_hdr *hdr,
                                        const uint8_t *buf, size_t size)
{
    if ((hdr->flags & VIRTIO_NET_HDR_F_NEEDS_CSUM) && /* missing csum */
        (size > 27 && size < 1500) && /* normal sized MTU */
        (buf[12] == 0x08 && buf[13] == 0x00) && /* ethertype == IPv4 */
        (buf[23] == 17) && /* ip.protocol == UDP */
        (buf[34] == 0 && buf[35] == 67)) { /* udp.srcport == bootps */
        /* FIXME this cast is evil */
        net_checksum_calculate((uint8_t *)buf, size);
        hdr->flags &= ~VIRTIO_NET_HDR_F_NEEDS_CSUM;
    }
}
#endif

static int iov_fill(struct iovec *iov, int iovcnt, const void *buf, int count)
{
    int offset, i;

    offset = i = 0;
    while (offset < count && i < iovcnt) {
        int len = MIN(iov[i].iov_len, count - offset);
        memcpy(iov[i].iov_base, buf + offset, len);
        offset += len;
        i++;
    }

    return offset;
}

static int receive_header(VirtIONet *n, struct iovec *iov, int iovcnt,
                          const void *buf, size_t size, size_t hdr_len, int raw)
{
    struct virtio_net_hdr *hdr = iov[0].iov_base;
    int offset = 0;

    hdr->flags = 0;
    hdr->gso_type = VIRTIO_NET_HDR_GSO_NONE;

#ifdef TAP_VNET_HDR
    if (tap_has_vnet_hdr(n->vc->vlan->first_client)) {
        if (!raw) {
            memcpy(hdr, buf, sizeof(*hdr));
        } else {
            memset(hdr, 0, sizeof(*hdr));
        }
        offset = sizeof(*hdr);
        work_around_broken_dhclient(hdr, buf + offset, size - offset);
    }
#endif

    /* We only ever receive a struct virtio_net_hdr from the tapfd,
     * but we may be passing along a larger header to the guest.
     */
    iov[0].iov_base += hdr_len;
    iov[0].iov_len  -= hdr_len;

    return offset;
}

static void virtio_net_receive2(void *opaque, const uint8_t *buf, int size, int raw)
{
    VirtIONet *n = opaque;
    struct virtio_net_hdr_mrg_rxbuf *mhdr = NULL;
    size_t hdr_len, offset, i;

    if (!do_virtio_net_can_receive(n, size))
        return;

    /* hdr_len refers to the header we supply to the guest */
    hdr_len = n->mergeable_rx_bufs ?
        sizeof(struct virtio_net_hdr_mrg_rxbuf) : sizeof(struct virtio_net_hdr);

    offset = i = 0;

    while (offset < size) {
        VirtQueueElement elem;
        int len, total;
        struct iovec sg[VIRTQUEUE_MAX_SIZE];

        len = total = 0;

        if ((i != 0 && !n->mergeable_rx_bufs) ||
            virtqueue_pop(n->rx_vq, &elem) == 0) {
            if (i == 0)
                return;
            fprintf(stderr, "virtio-net truncating packet\n");
            exit(1);
        }

        if (elem.in_num < 1) {
            fprintf(stderr, "virtio-net receive queue contains no in buffers\n");
            exit(1);
        }

        if (!n->mergeable_rx_bufs && elem.in_sg[0].iov_len != hdr_len) {
            fprintf(stderr, "virtio-net header not in first element\n");
            exit(1);
        }

        memcpy(&sg, &elem.in_sg[0], sizeof(sg[0]) * elem.in_num);

        if (i == 0) {
            if (n->mergeable_rx_bufs)
                mhdr = (struct virtio_net_hdr_mrg_rxbuf *)sg[0].iov_base;

            offset += receive_header(n, sg, elem.in_num,
                                     buf + offset, size - offset, hdr_len, raw);
            total += hdr_len;
        }

        /* copy in packet.  ugh */
        len = iov_fill(sg, elem.in_num,
                       buf + offset, size - offset);
        total += len;

        /* signal other side */
        virtqueue_fill(n->rx_vq, &elem, total, i++);

        offset += len;
    }

    if (mhdr)
        mhdr->num_buffers = i;

    virtqueue_flush(n->rx_vq, i);
    virtio_notify(&n->vdev, n->rx_vq);
}

static void virtio_net_receive(void *opaque, const uint8_t *buf, int size)
{
    virtio_net_receive2(opaque, buf, size, 0);
}

static void virtio_net_receive_raw(void *opaque, const uint8_t *buf, int size)
{
    virtio_net_receive2(opaque, buf, size, 1);
}

/* TX */
static int virtio_net_flush_tx(VirtIONet *n, VirtQueue *vq, int enable_notify)
{
    VirtQueueElement elem;
    int num_packets = 0;
#ifdef TAP_VNET_HDR
    int has_vnet_hdr = tap_has_vnet_hdr(n->vc->vlan->first_client);
#else
    int has_vnet_hdr = 0;
#endif

    if (!(n->vdev.status & VIRTIO_CONFIG_S_DRIVER_OK))
        return num_packets;

    if (n->tx_queue.out_num) {
        virtio_queue_set_notification(n->tx_vq, 0);
        return num_packets;
    }

    while (virtqueue_pop(vq, &elem)) {
        ssize_t len = 0;
        unsigned int out_num = elem.out_num;
        struct iovec *out_sg = &elem.out_sg[0];
        unsigned hdr_len;
        int ret;

        /* hdr_len refers to the header received from the guest */
        hdr_len = n->mergeable_rx_bufs ?
            sizeof(struct virtio_net_hdr_mrg_rxbuf) :
            sizeof(struct virtio_net_hdr);

        if (out_num < 1 || out_sg->iov_len != hdr_len) {
            fprintf(stderr, "virtio-net header not in first element\n");
            exit(1);
        }

        /* ignore the header if GSO is not supported */
        if (!has_vnet_hdr) {
            out_num--;
            out_sg++;
            len += hdr_len;
        } else if (n->mergeable_rx_bufs) {
            /* tapfd expects a struct virtio_net_hdr */
            hdr_len -= sizeof(struct virtio_net_hdr);
            out_sg->iov_len -= hdr_len;
            len += hdr_len;
        }

        ret = qemu_sendv_packet(n->vc, out_sg, out_num);
        if (ret == -EAGAIN) {
            virtio_queue_set_notification(n->tx_vq, 0);
            n->tx_queue.elem = elem;
            n->tx_queue.len = len;
            n->tx_queue.out_sg = out_sg;
            n->tx_queue.out_num = out_num;
            return num_packets;
        }

        virtqueue_push(vq, &elem, len + ret);
        virtio_notify(&n->vdev, vq);

        num_packets++;
    }

    if (enable_notify) {
        virtio_queue_set_notification(vq, 1);
        num_packets += virtio_net_flush_tx(n, vq, 0);
    }

    return num_packets;
}

static void virtio_net_handle_tx(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtIONet *n = to_virtio_net(vdev);

    if (n->tx_bh_scheduled)
        return;

    virtio_queue_set_notification(n->tx_vq, 0);
    qemu_bh_schedule(n->tx_bh);
    n->tx_bh_scheduled = 1;
}

static void virtio_net_tx_bh(void *opaque)
{
    VirtIONet *n = opaque;

    n->tx_bh_scheduled = 0;

    if (virtio_net_flush_tx(n, n->tx_vq, 1)) {
        virtio_queue_set_notification(n->tx_vq, 0);
        qemu_bh_schedule(n->tx_bh);
        n->tx_bh_scheduled = 1;
    }
}

static void virtio_net_retry_tx(void *opaque)
{
    VirtIONet *n = opaque;

    if (n->tx_queue.out_num) {
        int ret;

        ret = qemu_sendv_packet(n->vc, n->tx_queue.out_sg, n->tx_queue.out_num);
        if (ret == -EAGAIN) {
            return;
        }

        virtqueue_push(n->tx_vq, &n->tx_queue.elem, n->tx_queue.len + ret);
        virtio_notify(&n->vdev, n->tx_vq);

        n->tx_queue.out_num = 0;
    }

    virtio_queue_set_notification(n->tx_vq, 1);
    virtio_net_flush_tx(n, n->tx_vq, 0);
}

static void virtio_net_save(QEMUFile *f, void *opaque)
{
    VirtIONet *n = opaque;

    virtio_save(&n->vdev, f);

    qemu_put_buffer(f, n->mac, 6);
    qemu_put_be32(f, n->tx_bh_scheduled);
    qemu_put_be32(f, n->mergeable_rx_bufs);

#ifdef TAP_VNET_HDR
    qemu_put_be32(f, tap_has_vnet_hdr(n->vc->vlan->first_client));
#endif
}

static int virtio_net_load(QEMUFile *f, void *opaque, int version_id)
{
    VirtIONet *n = opaque;

    if (version_id != 2)
        return -EINVAL;

    virtio_load(&n->vdev, f);

    qemu_get_buffer(f, n->mac, 6);
    n->tx_bh_scheduled = qemu_get_be32(f);
    n->mergeable_rx_bufs = qemu_get_be32(f);

#ifdef TAP_VNET_HDR
    if (qemu_get_be32(f))
        tap_using_vnet_hdr(n->vc->vlan->first_client, 1);
#endif

    if (n->tx_bh_scheduled) {
        qemu_bh_schedule(n->tx_bh);
    }

    return 0;
}

static void virtio_net_cleanup(VLANClientState *vc)
{
    VirtIONet *n = vc->opaque;

    unregister_savevm("virtio-net", n);

    virtio_cleanup(&n->vdev);
}

PCIDevice *virtio_net_init(PCIBus *bus, NICInfo *nd, int devfn)
{
    VirtIONet *n;
    static int virtio_net_id;

    n = (VirtIONet *)virtio_init_pci(bus, "virtio-net",
                                     PCI_VENDOR_ID_REDHAT_QUMRANET,
                                     PCI_DEVICE_ID_VIRTIO_NET,
                                     PCI_VENDOR_ID_REDHAT_QUMRANET,
                                     VIRTIO_ID_NET,
                                     0x02, 0x00, 0x00,
                                     sizeof(struct virtio_net_config),
                                     sizeof(VirtIONet));
    if (!n)
        return NULL;

    n->vdev.get_config = virtio_net_update_config;
    n->vdev.get_features = virtio_net_get_features;
    n->vdev.set_features = virtio_net_set_features;
    n->vdev.bad_features = virtio_net_bad_features;
    n->rx_vq = virtio_add_queue(&n->vdev, 256, virtio_net_handle_rx);
    n->tx_vq = virtio_add_queue(&n->vdev, 256, virtio_net_handle_tx);
    memcpy(n->mac, nd->macaddr, 6);
    n->status = VIRTIO_NET_S_LINK_UP;
    n->vc = qemu_new_vlan_client(nd->vlan, nd->model, nd->name,
                                 virtio_net_receive, virtio_net_can_receive, n);
    nd->vc = n->vc;
    n->vc->cleanup = virtio_net_cleanup;
    n->vc->link_status_changed = virtio_net_set_link_status;
    n->vc->fd_read_raw = virtio_net_receive_raw;
    n->vc->fd_writable = virtio_net_retry_tx;

    qemu_format_nic_info_str(n->vc, n->mac);

    n->tx_bh = qemu_bh_new(virtio_net_tx_bh, n);
    n->tx_bh_scheduled = 0;
    n->mergeable_rx_bufs = 0;

    register_savevm("virtio-net", virtio_net_id++, 2,
                    virtio_net_save, virtio_net_load, n);

    return (PCIDevice *)n;
}

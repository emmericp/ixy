/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _VIRTIO_RING_H_
#define _VIRTIO_RING_H_

#include <stdint.h>

// #include <rte_common.h>
#define RTE_PTR_ADD(ptr, x) ((void*)((uintptr_t)(ptr) + (x)))
#define RTE_ALIGN_FLOOR(val, align) (typeof(val))((val) & (~((typeof(val))((align)-1))))
#define RTE_ALIGN_CEIL(val, align) RTE_ALIGN_FLOOR(((val) + ((typeof(val))(align)-1)), align)
#define RTE_PTR_ALIGN_FLOOR(ptr, align) ((typeof(ptr))RTE_ALIGN_FLOOR((uintptr_t)ptr, align))
#define RTE_PTR_ALIGN_CEIL(ptr, align) RTE_PTR_ALIGN_FLOOR((typeof(ptr))RTE_PTR_ADD(ptr, (align)-1), align)

/*
 * VirtIO Header, located in BAR 0.
 */
#define VIRTIO_PCI_HOST_FEATURES 0  /* host's supported features (32bit, RO)*/
#define VIRTIO_PCI_GUEST_FEATURES 4 /* guest's supported features (32, RW) */
#define VIRTIO_PCI_QUEUE_PFN 8      /* physical address of VQ (32, RW) */
#define VIRTIO_PCI_QUEUE_NUM 12     /* number of ring entries (16, RO) */
#define VIRTIO_PCI_QUEUE_SEL 14     /* current VQ selection (16, RW) */
#define VIRTIO_PCI_QUEUE_NOTIFY 16  /* notify host regarding VQ (16, RW) */
#define VIRTIO_PCI_STATUS 18        /* device status register (8, RW) */
#define VIRTIO_PCI_ISR 19           /* interrupt status register, reading also clears the register (8, RO) */
/* Only if MSIX is enabled: */
#define VIRTIO_MSI_CONFIG_VECTOR 20 /* configuration change vector (16, RW) */
#define VIRTIO_MSI_QUEUE_VECTOR 22  /* vector for selected VQ notifications (16, RW) */

/* Status byte for guest to report progress. */
#define VIRTIO_CONFIG_STATUS_RESET 0x00
#define VIRTIO_CONFIG_STATUS_ACK 0x01
#define VIRTIO_CONFIG_STATUS_DRIVER 0x02
#define VIRTIO_CONFIG_STATUS_DRIVER_OK 0x04
#define VIRTIO_CONFIG_STATUS_FEATURES_OK 0x08
#define VIRTIO_CONFIG_STATUS_FAILED 0x80

/*
 * How many bits to shift physical queue address written to QUEUE_PFN.
 * 12 is historical, and due to x86 page size.
 */
#define VIRTIO_PCI_QUEUE_ADDR_SHIFT 12

/* This marks a buffer as continuing via the next field. */
#define VRING_DESC_F_NEXT 1
/* This marks a buffer as write-only (otherwise read-only). */
#define VRING_DESC_F_WRITE 2
/* This means the buffer contains a list of buffer descriptors. */
#define VRING_DESC_F_INDIRECT 4

/* The feature bitmap for virtio net */
#define VIRTIO_NET_F_CSUM 0            /* Host handles pkts w/ partial csum */
#define VIRTIO_NET_F_GUEST_CSUM 1      /* Guest handles pkts w/ partial csum */
#define VIRTIO_NET_F_MTU 3             /* Initial MTU advice. */
#define VIRTIO_NET_F_MAC 5             /* Host has given MAC address. */
#define VIRTIO_NET_F_GUEST_TSO4 7      /* Guest can handle TSOv4 in. */
#define VIRTIO_NET_F_GUEST_TSO6 8      /* Guest can handle TSOv6 in. */
#define VIRTIO_NET_F_GUEST_ECN 9       /* Guest can handle TSO[6] w/ ECN in. */
#define VIRTIO_NET_F_GUEST_UFO 10      /* Guest can handle UFO in. */
#define VIRTIO_NET_F_HOST_TSO4 11      /* Host can handle TSOv4 in. */
#define VIRTIO_NET_F_HOST_TSO6 12      /* Host can handle TSOv6 in. */
#define VIRTIO_NET_F_HOST_ECN 13       /* Host can handle TSO[6] w/ ECN in. */
#define VIRTIO_NET_F_HOST_UFO 14       /* Host can handle UFO in. */
#define VIRTIO_NET_F_MRG_RXBUF 15      /* Host can merge receive buffers. */
#define VIRTIO_NET_F_STATUS 16         /* virtio_net_config.status available */
#define VIRTIO_NET_F_CTRL_VQ 17        /* Control channel available */
#define VIRTIO_NET_F_CTRL_RX 18        /* Control channel RX mode support */
#define VIRTIO_NET_F_CTRL_VLAN 19      /* Control channel VLAN filtering */
#define VIRTIO_NET_F_CTRL_RX_EXTRA 20  /* Extra RX mode control support */
#define VIRTIO_NET_F_GUEST_ANNOUNCE 21 /* Guest can announce device on the network */
#define VIRTIO_NET_F_MQ 22             /* Device supports Receive Flow Steering */
#define VIRTIO_NET_F_CTRL_MAC_ADDR 23  /* Set MAC address */

/* Do we get callbacks when the ring is completely used, even if we've suppressed them? */
#define VIRTIO_F_NOTIFY_ON_EMPTY 24

/* Can the device handle any descriptor layout? */
#define VIRTIO_F_ANY_LAYOUT 27

/* We support indirect buffer descriptors */
#define VIRTIO_RING_F_INDIRECT_DESC 28

#define VIRTIO_F_VERSION_1 32
#define VIRTIO_F_IOMMU_PLATFORM 33

/**
 * Control the RX mode, ie. promiscuous, allmulti, etc...
 * All commands require an "out" sg entry containing a 1 byte
 * state value, zero = disable, non-zero = enable.  Commands
 * 0 and 1 are supported with the VIRTIO_NET_F_CTRL_RX feature.
 * Commands 2-5 are added with VIRTIO_NET_F_CTRL_RX_EXTRA.
 */
#define VIRTIO_NET_CTRL_RX 0
#define VIRTIO_NET_CTRL_RX_PROMISC 0
#define VIRTIO_NET_CTRL_RX_ALLMULTI 1
#define VIRTIO_NET_CTRL_RX_ALLUNI 2
#define VIRTIO_NET_CTRL_RX_NOMULTI 3
#define VIRTIO_NET_CTRL_RX_NOUNI 4
#define VIRTIO_NET_CTRL_RX_NOBCAST 5

struct virtio_net_ctrl_hdr {
	uint8_t class;
	uint8_t cmd;
} __attribute__((packed));

typedef uint8_t virtio_net_ctrl_ack;

#define VIRTIO_NET_OK 0
#define VIRTIO_NET_ERR 1

#define VIRTIO_MAX_CTRL_DATA 2048

struct virtio_pmd_ctrl {
	struct virtio_net_ctrl_hdr hdr;
	virtio_net_ctrl_ack status;
	uint8_t data[VIRTIO_MAX_CTRL_DATA];
};

/**
 * This is the first element of the scatter-gather list.  If you don't
 * specify GSO or CSUM features, you can simply ignore the header.
 */
struct virtio_legacy_net_hdr {
#define VIRTIO_NET_HDR_F_NEEDS_CSUM 1 /**< Use csum_start,csum_offset*/
#define VIRTIO_NET_HDR_F_DATA_VALID 2 /**< Checksum is valid */
	uint8_t flags;
#define VIRTIO_NET_HDR_GSO_NONE 0   /**< Not a GSO frame */
#define VIRTIO_NET_HDR_GSO_TCPV4 1  /**< GSO frame, IPv4 TCP (TSO) */
#define VIRTIO_NET_HDR_GSO_UDP 3    /**< GSO frame, IPv4 UDP (UFO) */
#define VIRTIO_NET_HDR_GSO_TCPV6 4  /**< GSO frame, IPv6 TCP */
#define VIRTIO_NET_HDR_GSO_ECN 0x80 /**< TCP has ECN set */
	uint8_t gso_type;
	uint16_t hdr_len;     /**< Ethernet + IP + tcp/udp hdrs */
	uint16_t gso_size;    /**< Bytes to append to hdr_len per frame */
	uint16_t csum_start;  /**< Position to start checksumming from */
	uint16_t csum_offset; /**< Offset after that to place checksum */
};

/* This marks a buffer as continuing via the next field. */
#define VRING_DESC_F_NEXT 1
/* This marks a buffer as write-only (otherwise read-only). */
#define VRING_DESC_F_WRITE 2
/* This means the buffer contains a list of buffer descriptors. */
#define VRING_DESC_F_INDIRECT 4

/* The Host uses this in used->flags to advise the Guest: don't kick me
 * when you add a buffer.  It's unreliable, so it's simply an
 * optimization.  Guest will still kick if it's out of buffers. */
#define VRING_USED_F_NO_NOTIFY 1
/* The Guest uses this in avail->flags to advise the Host: don't
 * interrupt me when you consume a buffer.  It's unreliable, so it's
 * simply an optimization. */
#define VRING_AVAIL_F_NO_INTERRUPT 1

/* VirtIO ring descriptors: 16 bytes.
 * These can chain together via "next". */
struct vring_desc {
	uint64_t addr;  /*  Address (guest-physical). */
	uint32_t len;   /* Length. */
	uint16_t flags; /* The flags as indicated above. */
	uint16_t next;  /* We chain unused descriptors via this. */
};

struct vring_avail {
	uint16_t flags;
	uint16_t idx;
	uint16_t ring[0];
};

/* id is a 16bit index. uint32_t is used here for ids for padding reasons. */
struct vring_used_elem {
	/* Index of start of used descriptor chain. */
	uint32_t id;
	/* Total length of the descriptor chain which was written to. */
	uint32_t len;
};

struct vring_used {
	uint16_t flags;
	volatile uint16_t idx;
	struct vring_used_elem ring[0];
};

struct vring {
	unsigned int num;
	struct vring_desc* desc;
	struct vring_avail* avail;
	struct vring_used* used;
};

/* The standard layout for the ring is a continuous chunk of memory which
 * looks like this.  We assume num is a power of 2.
 *
 * struct vring {
 *      // The actual descriptors (16 bytes each)
 *      struct vring_desc desc[num];
 *
 *      // A ring of available descriptor heads with free-running index.
 *      uint16_t avail_flags;
 *      uint16_t avail_idx;
 *      uint16_t available[num];
 *      uint16_t used_event_idx;
 *
 *      // Padding to the next align boundary.
 *      char pad[];
 *
 *      // A ring of used descriptor heads with free-running index.
 *      uint16_t used_flags;
 *      uint16_t used_idx;
 *      struct vring_used_elem used[num];
 *      uint16_t avail_event_idx;
 * };
 *
 * NOTE: for VirtIO PCI, align is 4096.
 */

struct virtqueue {
	struct vring vring; // The DMA'd region containing the descriptors etc.
	// Additional information for the driver only
	uint64_t notification_offset;
	uint16_t vq_used_last_idx;
	struct mempool* mempool; // Unused in Tx queues
	// virtual addresses to map descriptors back to their mbuf for freeing
	void* virtual_addresses[];
};

/*
 * We publish the used event index at the end of the available ring, and vice
 * versa. They are at the end for backwards compatibility.
 */
#define vring_used_event(vr) ((vr)->avail->ring[(vr)->num])
#define vring_avail_event(vr) (*(uint16_t*)&(vr)->used->ring[(vr)->num])

/*
 * The following is used with VIRTIO_RING_F_EVENT_IDX.
 * Assuming a given event_idx value from the other size, if we have
 * just incremented index from old to new_idx, should we trigger an
 * event?
 */
static inline int vring_need_event(uint16_t event_idx, uint16_t new_idx, uint16_t old) {
	return (uint16_t)(new_idx - event_idx - 1) < (uint16_t)(new_idx - old);
}

#endif /* _VIRTIO_RING_H_ */

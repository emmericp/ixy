#include <emmintrin.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/device.h"
#include "log.h"
#include "memory.h"
#include "pci.h"
#include "virtio.h"
#include "virtio_type.h"

static const char* driver_name = "ixy-virtio";

static inline void virtio_legacy_notify_queue(struct virtio_device* dev, uint16_t idx) {
	write_io16(dev->fd, idx, VIRTIO_PCI_QUEUE_NOTIFY);
}

static uint8_t virtio_legacy_get_status(struct virtio_device* dev) {
	return read_io8(dev->fd, VIRTIO_PCI_STATUS);
}

static void virtio_legacy_check_status(struct virtio_device* dev) {
	if (read_io8(dev->fd, VIRTIO_PCI_STATUS) == VIRTIO_CONFIG_STATUS_FAILED) {
		error("Device signaled unrecoverable error");
	}
}

static inline size_t virtio_legacy_vring_size(unsigned int num, unsigned long align) {
	size_t size;

	size = num * sizeof(struct vring_desc);
	size += sizeof(struct vring_avail) + (num * sizeof(uint16_t));
	size = RTE_ALIGN_CEIL(size, align);
	size += sizeof(struct vring_used) + (num * sizeof(struct vring_used_elem));
	return size;
}

static inline void virtio_legacy_vring_init(struct vring* vr, unsigned int num, uint8_t* p, unsigned long align) {
	vr->num = num;
	vr->desc = (struct vring_desc*)p;
	vr->avail = (struct vring_avail*)(p + num * sizeof(struct vring_desc));
	vr->used = (void*)RTE_ALIGN_CEIL((uintptr_t)(&vr->avail->ring[num]), align);
}

static void virtio_legacy_setup_tx_queue(struct virtio_device* dev, uint16_t idx) {
	if (idx != 1 && idx != 2) {
		error("Can't setup queue %u as Tx queue", idx);
	}

	// Create virt queue itself - Section 4.1.5.1.3
	write_io16(dev->fd, idx, VIRTIO_PCI_QUEUE_SEL);
	uint32_t max_queue_size = read_io32(dev->fd, VIRTIO_PCI_QUEUE_NUM);
	debug("Max queue size of tx queue #%u: %u", idx, max_queue_size);
	if (max_queue_size == 0) {
		return;
	}
	size_t virt_queue_mem_size = virtio_legacy_vring_size(max_queue_size, 4096);
	struct dma_memory mem = memory_allocate_dma(virt_queue_mem_size, true);
	memset(mem.virt, 0xab, virt_queue_mem_size);
	debug("Allocated %zu bytes for virt queue at %p", virt_queue_mem_size, mem.virt);
	write_io32(dev->fd, mem.phy >> VIRTIO_PCI_QUEUE_ADDR_SHIFT, VIRTIO_PCI_QUEUE_PFN);

	// Section 2.4.2 for layout
	struct virtqueue* vq = calloc(1, sizeof(*vq) + sizeof(void*) * max_queue_size);
	virtio_legacy_vring_init(&vq->vring, max_queue_size, mem.virt, 4096);
	debug("vring desc: %p, vring avail: %p, vring used: %p", vq->vring.desc, vq->vring.avail, vq->vring.used);
	for (size_t i = 0; i < vq->vring.num; ++i) {
		vq->vring.desc[i].len = 0;
		vq->vring.desc[i].addr = 0;
		vq->vring.desc[i].flags = 0;
		vq->vring.desc[i].next = 0;
		vq->vring.avail->ring[i] = 0;
		vq->vring.used->ring[i].id = 0;
		vq->vring.used->ring[i].len = 0;
	}
	vq->vring.used->idx = 0;
	vq->vring.avail->idx = 0;
	vq->vq_used_last_idx = 0;

	// Section 4.1.4.4
	uint32_t notify_offset = read_io16(dev->fd, VIRTIO_PCI_QUEUE_NOTIFY);
	debug("vq notifcation offset %u", notify_offset);
	vq->notification_offset = notify_offset;

	// Ctrl queue packets are not supplied by the user
	if (idx == 2) {
		vq->mempool = memory_allocate_mempool(max_queue_size, 2048);
	}

	// Disable interrupts - Section 2.4.7
	vq->vring.avail->flags = VRING_AVAIL_F_NO_INTERRUPT;
	vq->vring.used->flags = 0;

	if (idx == 1) {
		dev->tx_queue = vq;
	} else {
		dev->ctrl_queue = vq;
	}
}

static void virtio_legacy_send_command(struct virtio_device* dev, void* cmd, size_t cmd_len) {
	struct virtqueue* vq = dev->ctrl_queue;

	if (cmd_len < sizeof(struct virtio_net_ctrl_hdr)) {
		error("Command can not be shorter than control header");
	}
	if (((uint8_t*)cmd)[0] != VIRTIO_NET_CTRL_RX) {
		error("Command class is not supported");
	}

	_mm_mfence();
	// Find free desciptor slot
	uint16_t idx = 0;
	for (idx = 0; idx < vq->vring.num; ++idx) {
		struct vring_desc* desc = &vq->vring.desc[idx];
		if (desc->addr == 0) {
			break;
		}
	}
	if (idx == vq->vring.num) {
		error("command queue full");
	} else {
		debug("Found free desc slot at %u (%u)", idx, vq->vring.num);
	}

	struct pkt_buf* buf = pkt_buf_alloc(vq->mempool);
	if (!buf) {
		error("Control queue ran out of buffers");
	}
	memcpy(buf->data, cmd, cmd_len);
	vq->virtual_addresses[idx] = buf;

	/* The following descriptor setup kills QEMU, but should be allowed with VIRTIO_F_ANY_LAYOUT
	 * Error: kvm: virtio-net ctrl missing headers
	 * Version: QEMU emulator version 2.7.1 pve-qemu-kvm_2.7.1-4
	 */
	// All in one descriptor
	// vq->vring.desc[idx].len = cmd_len;
	// vq->vring.desc[idx].addr = buf->buf_addr_phy + offsetof(struct pkt_buf, data);
	// vq->vring.desc[idx].flags = VRING_DESC_F_WRITE;
	// vq->vring.desc[idx].next = 0;

	// Device-readable head: cmd header
	vq->vring.desc[idx].len = 2;
	vq->vring.desc[idx].addr = buf->buf_addr_phy + offsetof(struct pkt_buf, data);
	vq->vring.desc[idx].flags = VRING_DESC_F_NEXT;
	vq->vring.desc[idx].next = idx + 1;
	// Device-readable payload: data
	vq->vring.desc[idx + 1].len = cmd_len - 2 - 1; // Header and ack byte
	vq->vring.desc[idx + 1].addr = buf->buf_addr_phy + offsetof(struct pkt_buf, data) + 2;
	vq->vring.desc[idx + 1].flags = VRING_DESC_F_NEXT;
	vq->vring.desc[idx + 1].next = idx + 2;
	// Device-writable tail: ack flag
	vq->vring.desc[idx + 2].len = 1;
	vq->vring.desc[idx + 2].addr = buf->buf_addr_phy + offsetof(struct pkt_buf, data) + cmd_len - 1;
	vq->vring.desc[idx + 2].flags = VRING_DESC_F_WRITE;
	vq->vring.desc[idx + 2].next = 0;
	vq->vring.avail->ring[vq->vring.avail->idx % vq->vring.num] = idx;
	_mm_mfence();
	vq->vring.avail->idx++;
	_mm_mfence();

	virtio_legacy_notify_queue(dev, 2);
	_mm_mfence();

	// Wait until the buffer got processed
	while (vq->vq_used_last_idx == vq->vring.used->idx) {
		_mm_mfence();
		debug("Waiting...");
		usleep(100000);
	}
	vq->vq_used_last_idx++;
	// Check status and free buffer
	struct vring_used_elem* e = &vq->vring.used->ring[vq->vring.used->idx];
	debug("e %p: id %u len %u", e, e->id, e->len);
	if (e->id != idx) {
		error("Used buffer has different index as sent one");
	}
	if (vq->virtual_addresses[idx] != buf) {
		error("buffer differ");
	}
	pkt_buf_free(buf);
	vq->vring.desc[idx] = (struct vring_desc){};
	vq->vring.desc[idx + 1] = (struct vring_desc){};
	vq->vring.desc[idx + 2] = (struct vring_desc){};
}

static void virtio_legacy_set_promiscuous(struct virtio_device* dev, bool on) {
	struct {
		struct virtio_net_ctrl_hdr hdr;
		uint8_t on;
		uint8_t ack;
	} __attribute__((__packed__)) cmd = {};
	static_assert(sizeof(cmd) == 4, "Size of command struct wrong");

	cmd.hdr.class = VIRTIO_NET_CTRL_RX;
	cmd.hdr.cmd = VIRTIO_NET_CTRL_RX_PROMISC;
	cmd.on = on ? 1 : 0;

	virtio_legacy_send_command(dev, &cmd, sizeof(cmd));
	info("Set promisc to %u", on);
}

void virtio_set_promisc(struct ixy_device* ixy, bool enabled) {
	struct virtio_device* dev = IXY_TO_VIRTIO(ixy);
	virtio_legacy_set_promiscuous(dev, enabled);
}

uint32_t virtio_get_link_speed(const struct ixy_device* dev) {
	return 1000;
}

static const struct virtio_legacy_net_hdr net_hdr = {
	.flags = 0,
	.gso_type = VIRTIO_NET_HDR_GSO_NONE,
	.hdr_len = 14 + 20 + 8,
};

static void virtio_legacy_setup_rx_queue(struct virtio_device* dev, uint16_t idx) {
	if (idx != 0) {
		error("Can't setup Tx queue as Rx");
	}

	// Create virt queue itself - Section 4.1.5.1.3
	write_io16(dev->fd, idx, VIRTIO_PCI_QUEUE_SEL);
	uint32_t max_queue_size = read_io32(dev->fd, VIRTIO_PCI_QUEUE_NUM);
	debug("Max queue size of rx queue #%u: %u", idx, max_queue_size);
	if (max_queue_size == 0) {
		return;
	}
	uint32_t notify_offset = read_io16(dev->fd, VIRTIO_PCI_QUEUE_NOTIFY);
	debug("Notifcation offset %u", notify_offset);
	size_t virt_queue_mem_size = virtio_legacy_vring_size(max_queue_size, 4096);
	struct dma_memory mem = memory_allocate_dma(virt_queue_mem_size, true);
	memset(mem.virt, 0xab, virt_queue_mem_size);
	debug("Allocated %zu bytes for virt queue at %p", virt_queue_mem_size, mem.virt);
	write_io32(dev->fd, mem.phy >> VIRTIO_PCI_QUEUE_ADDR_SHIFT, VIRTIO_PCI_QUEUE_PFN);

	// Section 2.4.2 for layout
	struct virtqueue* vq = calloc(1, sizeof(*vq) + sizeof(void*) * max_queue_size);
	virtio_legacy_vring_init(&vq->vring, max_queue_size, mem.virt, 4096);
	debug("vring desc: %p, vring avail: %p, vring used: %p", vq->vring.desc, vq->vring.avail, vq->vring.used);
	for (size_t i = 0; i < vq->vring.num; ++i) {
		vq->vring.desc[i].len = 0;
		vq->vring.desc[i].addr = 0;
		vq->vring.desc[i].flags = 0;
		vq->vring.desc[i].next = 0;
		vq->vring.avail->ring[i] = 0;
		vq->vring.used->ring[i].id = 0;
		vq->vring.used->ring[i].len = 0;
	}
	vq->vring.used->idx = 0;
	vq->vring.avail->idx = 0;
	vq->vq_used_last_idx = 0;

	// Section 4.1.4.4
	vq->notification_offset = notify_offset;

	// Disable interrupts - Section 2.4.7
	vq->vring.avail->flags = VRING_AVAIL_F_NO_INTERRUPT;
	vq->vring.used->flags = 0;

	// Allocate buffers and fill descriptor table - Section 3.2.1
	// We allocate more bufs than what would fit in the queue,
	// because we don't want to stall rx if users hold bufs for longer
	vq->mempool = memory_allocate_mempool(max_queue_size * 4, 2048);

	dev->rx_queue = vq;
}

static void virtio_legacy_init(struct virtio_device* dev) {
	// Section 3.1
	debug("Configuring bar0");
	write_io8(dev->fd, VIRTIO_CONFIG_STATUS_RESET, VIRTIO_PCI_STATUS);
	while (read_io8(dev->fd, VIRTIO_PCI_STATUS) != VIRTIO_CONFIG_STATUS_RESET) {
		usleep(100);
	}
	write_io8(dev->fd, VIRTIO_CONFIG_STATUS_ACK, VIRTIO_PCI_STATUS);
	write_io8(dev->fd, VIRTIO_CONFIG_STATUS_DRIVER, VIRTIO_PCI_STATUS);
	// Negotiate features
	uint32_t host_features = read_io32(dev->fd, VIRTIO_PCI_HOST_FEATURES);
	debug("Host features: %x", host_features);
	if (!(host_features & VIRTIO_F_VERSION_1)) {
		error("In legacy mode but device is not legacy");
	}
	const uint32_t required_features = (1u << VIRTIO_NET_F_CSUM) | (1u << VIRTIO_NET_F_GUEST_CSUM) |
					   (1u << VIRTIO_NET_F_CTRL_VQ) | (1u << VIRTIO_F_ANY_LAYOUT) |
					   (1u << VIRTIO_NET_F_CTRL_RX) /*| (1u<<VIRTIO_NET_F_MQ)*/;
	if ((host_features & required_features) != required_features) {
		error("Device does not support required features");
	}
	debug("Guest features before negotiation: %x", read_io32(dev->fd, VIRTIO_PCI_GUEST_FEATURES));
	write_io32(dev->fd, required_features, VIRTIO_PCI_GUEST_FEATURES);
	debug("Guest features after negotiation: %x", read_io32(dev->fd, VIRTIO_PCI_GUEST_FEATURES));
	// Queue setup - Section 5.1.2 for queue index calculation
	// Legacy devices only have 3 queues
	virtio_legacy_setup_rx_queue(dev, 0); // Rx
	virtio_legacy_setup_tx_queue(dev, 1); // Tx
	virtio_legacy_setup_tx_queue(dev, 2); // Control
	_mm_mfence();
	// Signal OK
	write_io8(dev->fd, VIRTIO_CONFIG_STATUS_DRIVER_OK, VIRTIO_PCI_STATUS);
	info("Setup complete");
	// Recheck status
	virtio_legacy_check_status(dev);
	virtio_legacy_set_promiscuous(dev, true);
}

// read stat counters and accumulate in stats
// stats may be NULL to just reset the counters
// this is not thread-safe, (but we only support one queue anyways)
// a proper thread-safe implementation would collect per-queue stats
// and perform a read with relaxed memory ordering here without resetting the stats
void virtio_read_stats(struct ixy_device* ixy, struct device_stats* stats) {
	struct virtio_device* dev = IXY_TO_VIRTIO(ixy);
	if (stats) {
		stats->rx_pkts += dev->rx_pkts;
		stats->tx_pkts += dev->tx_pkts;
		stats->rx_bytes += dev->rx_bytes;
		stats->tx_bytes += dev->tx_bytes;
	}
	dev->rx_pkts = dev->tx_pkts = dev->rx_bytes = dev->tx_bytes = 0;
}


struct ixy_device* virtio_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues) {
	if (getuid()) {
		warn("Not running as root, this will probably fail");
	}
	if (rx_queues > 1) {
		error("cannot configure %d rx queues: limit is %d", rx_queues, 1);
	}
	if (tx_queues > 1) {
		error("cannot configure %d tx queues: limit is %d", tx_queues, 1);
	}
	remove_driver(pci_addr);
	enable_dma(pci_addr);
	struct virtio_device* dev = calloc(1, sizeof(*dev));
	dev->ixy.pci_addr = strdup(pci_addr);
	dev->ixy.driver_name = driver_name;
	dev->ixy.num_rx_queues = rx_queues;
	dev->ixy.num_tx_queues = tx_queues;
	dev->ixy.rx_batch = virtio_rx_batch;
	dev->ixy.tx_batch = virtio_tx_batch;
	dev->ixy.read_stats = virtio_read_stats;
	dev->ixy.set_promisc = virtio_set_promisc;
	dev->ixy.get_link_speed = virtio_get_link_speed;

	int config = pci_open_resource(pci_addr, "config");
	uint16_t device_id = read_io16(config, 2);
	close(config);
	// Check config if device is legacy network card
	if (device_id == 0x1000) {
		info("Detected virtio legacy network card");
		dev->fd = pci_open_resource(pci_addr, "resource0");
		virtio_legacy_init(dev);
	} else {
		error("Modern device not supported");
	}
	return &dev->ixy;
}

uint32_t virtio_rx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct virtio_device* dev = IXY_TO_VIRTIO(ixy);
	struct virtqueue* vq = dev->rx_queue;
	uint32_t buf_idx;

	_mm_mfence();
	// Retrieve used bufs from the device
	for (buf_idx = 0; buf_idx < num_bufs; ++buf_idx) {
		// Section 3.2.2
		if (vq->vq_used_last_idx == vq->vring.used->idx) {
			break;
		}
		// info("Rx packet: last used %u, used idx %u", vq->vq_used_last_idx,
		// vq->vring.used->idx);
		struct vring_used_elem* e = vq->vring.used->ring + (vq->vq_used_last_idx % vq->vring.num);
		// info("Used elem %p, id %u len %u", e, e->id, e->len);
		struct vring_desc* desc = &vq->vring.desc[e->id];
		vq->vq_used_last_idx++;
		// We don't support chaining or indirect descriptors
		if (desc->flags != VRING_DESC_F_WRITE) {
			error("unsupported rx flags on descriptor: %x", desc->flags);
		}
		// info("Desc %lu %u %u %u", desc->addr, desc->len, desc->flags,
		// desc->next);
		*desc = (struct vring_desc){};

		// Section 5.1.6.4
		struct pkt_buf* buf = vq->virtual_addresses[e->id];
		buf->size = e->len;
		bufs[buf_idx] = buf;
		//struct virtio_net_hdr* hdr = (void*)(buf->head_room + sizeof(buf->head_room) - sizeof(net_hdr));

		// Update rx counter
		dev->rx_bytes += buf->size;
		dev->rx_pkts++;
	}
	// Fill empty slots in desciptor table
	for (uint16_t idx = 0; idx < vq->vring.num; ++idx) {
		struct vring_desc* desc = &vq->vring.desc[idx];
		if (desc->addr != 0) { // descriptor points to something, therefore it is in use
			continue;
		}
		// info("Found free desc slot at %u (%u)", idx, vq->vring.num);
		struct pkt_buf* buf = pkt_buf_alloc(vq->mempool);
		if (!buf) {
			error("failed to allocate new mbuf for rx, you are either leaking memory or your mempool is too small");
		}
		buf->size = vq->mempool->buf_size;
		memcpy(buf->head_room + sizeof(buf->head_room) - sizeof(net_hdr), &net_hdr, sizeof(net_hdr));
		vq->vring.desc[idx].len = buf->size + sizeof(net_hdr);
		vq->vring.desc[idx].addr =
		    buf->buf_addr_phy + offsetof(struct pkt_buf, head_room) + sizeof(buf->head_room) - sizeof(net_hdr);
		vq->vring.desc[idx].flags = VRING_DESC_F_WRITE;
		vq->vring.desc[idx].next = 0;
		vq->virtual_addresses[idx] = buf;
		vq->vring.avail->ring[vq->vring.avail->idx % vq->vring.num] = idx;
		_mm_mfence(); // Make sure exposed descriptors reach device before index is updated
		vq->vring.avail->idx++;
		_mm_mfence(); // Make sure the index update reaches device before it is triggered
		virtio_legacy_notify_queue(dev, 0);
	}
	return buf_idx;
}

uint32_t virtio_tx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct virtio_device* dev = IXY_TO_VIRTIO(ixy);
	struct virtqueue* vq = dev->tx_queue;

	_mm_mfence();
	// Free sent buffers
	while (vq->vq_used_last_idx != vq->vring.used->idx) {
		// info("We can free some buffers: %u != %u", vq->vq_used_last_idx,
		// vq->vring.used->idx);
		struct vring_used_elem* e = vq->vring.used->ring + (vq->vq_used_last_idx % vq->vring.num);
		// info("e %p, id %u", e, e->id);
		struct vring_desc* desc = &vq->vring.desc[e->id];
		desc->addr = 0;
		desc->len = 0;
		pkt_buf_free(vq->virtual_addresses[e->id]);
		vq->vq_used_last_idx++;
		_mm_mfence();
	}
	// Send buffers
	uint32_t buf_idx;
	uint16_t idx = 0; // Keep index of last found free descriptor and start searching from there
	for (buf_idx = 0; buf_idx < num_bufs; ++buf_idx) {
		struct pkt_buf* buf = bufs[buf_idx];
		// Find free desc index
		for (; idx < vq->vring.num; ++idx) {
			struct vring_desc* desc = &vq->vring.desc[idx];
			if (desc->addr == 0) {
				break;
			}
		}
		if (idx == vq->vring.num) {
			break;
		}
		// info("Found free desc slot at %u (%u)", idx, vq->vring.num);

		// Update tx counter
		dev->tx_bytes += buf->size;
		dev->tx_pkts++;

		vq->virtual_addresses[idx] = buf;

		// Copy header to headroom in front of data buffer
		memcpy(buf->head_room + sizeof(buf->head_room) - sizeof(net_hdr), &net_hdr, sizeof(net_hdr));

		vq->vring.desc[idx].len = buf->size + sizeof(net_hdr);
		vq->vring.desc[idx].addr =
		    buf->buf_addr_phy + offsetof(struct pkt_buf, head_room) + sizeof(buf->head_room) - sizeof(net_hdr);
		vq->vring.desc[idx].flags = 0;
		vq->vring.desc[idx].next = 0;
		vq->vring.avail->ring[idx] = idx;
	}
	_mm_mfence();
	vq->vring.avail->idx += buf_idx;
	_mm_mfence();
	virtio_legacy_notify_queue(dev, 1);
	return buf_idx;
}


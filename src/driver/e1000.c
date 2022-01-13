#include <emmintrin.h>
#include <linux/limits.h>
#include <linux/vfio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "device.h"
#include "e1000_type.h"
#include "e1000.h"
#include "libixy-vfio.h"
#include "log.h"
#include "memory.h"
#include "interrupts.h"
#include "pci.h"
#include "stats.h"

/* General e1000 driver settings & data structure */
#define E1000_MAX_RX_RING_SIZE 4096
#define E1000_MAX_TX_RING_SIZE 4096
#define E1000_RX_RING_SIZE 512
#define E1000_TX_RING_SIZE 512
#define E1000_PKT_BUF_ENTRY_SIZE 2048
#define E1000_MIN_MEMPOOL_ENTRIES 4096
#define E1000_TX_CLEAN_BATCH 32


// allocated for each rx queue, keeps state for the receive function
struct e1000_rx_queue {
	volatile struct e1000_rx_desc* descriptors;
	struct mempool* mempool;
	uint16_t num_entries;
	// position we are reading from
	uint16_t rx_index;
	// virtual addresses to map descriptors back to their mbuf for freeing
	void* virtual_addresses[];
};

// allocated for each tx queue, keeps state for the transmit function
struct e1000_tx_queue {
	volatile struct e1000_tx_desc* descriptors;
	uint16_t num_entries;
	// position to clean up descriptors that where sent out by the nic
	uint16_t clean_index;
	// position to insert packets for transmission
	uint16_t tx_index;
	// virtual addresses to map descriptors back to their mbuf for freeing
	void* virtual_addresses[];
};

static char* driver_name = "ixy-e1000";

// see section 14.4
//  - allocate memory from hugepage to setup receive queue
//  - initialize multicast table array (MTA)
//  - setup rx control register (RCTL)
static void init_rx(struct e1000_device* dev) {
	// setup receive queue
	uint32_t ring_size_bytes = E1000_RX_RING_SIZE * sizeof(struct e1000_rx_desc);
	struct dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
	memset(mem.virt, 0, ring_size_bytes);
	struct e1000_rx_desc* rx_ring = (struct e1000_rx_desc*)mem.virt;
	struct e1000_rx_queue* queue = (struct e1000_rx_queue*)(dev->rx_queues);
	queue->num_entries = E1000_RX_RING_SIZE;
	queue->rx_index = 0;
	queue->descriptors = (struct e1000_rx_desc*) mem.virt;
	int mempool_size = E1000_RX_RING_SIZE + E1000_TX_RING_SIZE;
	queue->mempool = memory_allocate_mempool(
		mempool_size < E1000_MIN_MEMPOOL_ENTRIES ?
		E1000_MIN_MEMPOOL_ENTRIES : mempool_size, E1000_PKT_BUF_ENTRY_SIZE);

	for (int i = 0; i < queue->num_entries; i++) {
		volatile struct e1000_rx_desc *rxd = queue->descriptors + i;
		struct pkt_buf* buf = pkt_buf_alloc(queue->mempool);
		rxd->addr = (uint64_t) (buf->buf_addr_phy + offsetof(struct pkt_buf, data));
    }

	set_reg32(dev->addr, E1000_RDBAL, ((uint64_t) mem.phy) & 0xFFFFFFFFull);
	set_reg32(dev->addr, E1000_RDBAH, ((uint64_t) mem.phy) >> 32);
	set_reg32(dev->addr, E1000_RDH, 0);
	set_reg32(dev->addr, E1000_RDT, 0);
	set_reg32(dev->addr, E1000_RDLEN, sizeof(rx_ring));

	// init multicast table array
	for (int i = 0; i < 128; i++) {
		set_reg32(dev->addr, E1000_MTA + 4*i, 0);
	}

	// setup receive control register
	set_reg32(dev->addr, E1000_RCTL,
		E1000_RCTL_EN |       // enable receiver
		E1000_RCTL_BAM |      // enable broadcast
		E1000_RCTL_SZ_2048 |  // 2048-byte rx buffers
		E1000_RCTL_SECRC);    // strip CRC
}


// see section 14.5
//  - allocate memory from hugepage to setup tx queue
//  - setup tx control register (RCTL)
static void init_tx(struct e1000_device* dev) {
	uint32_t ring_size_bytes = E1000_TX_RING_SIZE * sizeof(struct e1000_tx_desc);
	struct dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
	memset(mem.virt, 0, ring_size_bytes);
	struct e1000_tx_desc *tx_ring = (struct e1000_tx_desc*)mem.virt;
		for (int i = 0; i < E1000_TX_RING_SIZE; i++) {
		tx_ring[i].status = E1000_TXD_STAT_DD;
	}
	struct e1000_tx_queue* queue = (struct e1000_tx_queue*)(dev->tx_queues);
	queue->num_entries = E1000_TX_RING_SIZE;
	queue->descriptors = (struct e1000_tx_desc*) mem.virt;

	set_reg32(dev->addr, E1000_TDBAL, ((uint64_t) mem.phy) & 0xFFFFFFFFull);
	set_reg32(dev->addr, E1000_TDBAH, ((uint64_t) mem.phy) >> 32);

	// tx queue starts out empty
	set_reg32(dev->addr,E1000_TDLEN, E1000_MAX_TX_RING_SIZE);
	set_reg32(dev->addr,E1000_TDH, 0);
	set_reg32(dev->addr,E1000_TDT, 0);

	// setup tx control register
	uint32_t flags =
		E1000_TCTL_EN |                   // enable
		E1000_TCTL_PSP |                  // pad short packets
		(0x10 << E1000_TCTL_CT_SHIFT) |   // collision stuff
		(0x40 << E1000_TCTL_COLD_SHIFT);
	set_reg32(dev->addr, E1000_TCTL, flags);
	set_reg32(dev->addr, E1000_TIPG, 10 | (8<<10) | (6<<20)); // inter-pkt gap
}

static void wait_for_link(const struct e1000_device* dev) {
	info("Waiting for link...");
	int32_t max_wait = 10000000; // 10 seconds in us
	uint32_t poll_interval = 100000; // 10 ms in us
	while (!(e1000_get_link_speed(&dev->ixy)) && max_wait > 0) {
		usleep(poll_interval);
		max_wait -= poll_interval;
	}
	info("Link speed is %d Mbit/s", e1000_get_link_speed(&dev->ixy));
}

uint32_t e1000_get_link_speed(const struct ixy_device* ixy) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t status = get_reg32(dev->addr, E1000_STATUS);

	if (!(status & E1000_STATUS_LU)) {
		return 0;
	}
	int  value = (status & E1000_LINKSPEED) >> 6;
	switch (value) {
		case E1000_1000Mbps:
			return 1000;
		case E1000_100Mbps:
			return 100;
		case E1000_10Mbps:
			return 10;
		default:
			return 0;
  }
  return 0;
}

struct mac_address e1000_get_mac_addr(const struct ixy_device* ixy) {
	struct mac_address mac;
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t rar_low = get_reg32(dev->addr, E1000_RAL(0));
	uint32_t rar_high = get_reg32(dev->addr, E1000_RAH(0)) & ~(1 << 31);

	mac.addr[0] = rar_low;
	mac.addr[1] = rar_low >> 8;
	mac.addr[2] = rar_low >> 16;
	mac.addr[3] = rar_low >> 24;
	mac.addr[4] = rar_high;
	mac.addr[5] = rar_high >> 8;
	return mac;
}

void e1000_set_mac_addr(struct ixy_device* ixy, struct mac_address mac) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t rar_low = mac.addr[0] + (mac.addr[1] << 8) + (mac.addr[2] << 16) + (mac.addr[3] << 24);
	uint32_t rar_high = mac.addr[4] + (mac.addr[5] << 8);

	set_reg32(dev->addr, E1000_RAL(0), rar_low);
	set_reg32(dev->addr, E1000_RAH(0), rar_high);
}

void e1000_set_promisc(struct ixy_device* ixy, bool enabled) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	if (enabled) {
		info("enabling promisc mode");
		set_flags32(dev->addr, E1000_RCTL, E1000_RCTL_MPE | E1000_RCTL_UPE);
	} else {
		info("disabling promisc mode");
		clear_flags32(dev->addr, E1000_RCTL, E1000_RCTL_MPE | E1000_RCTL_UPE);
	}
}

// read stat counters and accumulate in stats
// stats may be NULL to just reset the counters
void e1000_read_stats(struct ixy_device* ixy, struct device_stats* stats) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	uint32_t rx_pkts = get_reg32(dev->addr, E1000_GPRC);
	uint32_t tx_pkts = get_reg32(dev->addr, E1000_GPTC);
	uint64_t rx_bytes = get_reg32(dev->addr, E1000_GORCL) + (((uint64_t) get_reg32(dev->addr, E1000_GORCH)) << 32);
	uint64_t tx_bytes = get_reg32(dev->addr, E1000_GOTCL) + (((uint64_t) get_reg32(dev->addr, E1000_GOTCH)) << 32);

	if (stats) {
		stats->rx_pkts += rx_pkts;
		stats->tx_pkts += tx_pkts;
		stats->rx_bytes += rx_bytes;
		stats->tx_bytes += tx_bytes;
	}
}

// advance index with wrap-around, this line is the reason why we require a power of two for the queue size
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))

// see section 3.2 and 3.3 for rx ring structure and information.
// general logic is as follows:
// - when hardware receives complete data, it update status DD in rx descriptor
//   and move the head of the rx ring.
// - when driver tries to get data from rx ring, it will read status of rx descriptor
//   to determine whether rx descriptor contains a complete data.
// - rx ring structure:
//    [ <rx-desc> <rx-desc> ... <rx-desc> ... <rx-desc>]
//         |                                     |
//        tail                                  head

uint32_t e1000_rx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);

	if (queue_id > 0) {
		error("queue id must be zero, queue_id=%d", queue_id);
		return 0;
	}

	struct e1000_rx_queue* queue = (struct e1000_rx_queue*) (dev->rx_queues);
	uint16_t rx_index = queue->rx_index;
	uint16_t last_rx_index = rx_index;
	uint32_t buf_index;

	for (buf_index = 0; buf_index < num_bufs; buf_index++) {
		volatile struct e1000_rx_desc* desc_ptr = queue->descriptors + rx_index;
		uint32_t status = desc_ptr->status;
		// if rx desc contains complete data, proceed to copy it out
		if (status & E1000_RXD_STAT_DD) {
			if (!(status & E1000_RXD_STAT_EOP)) {
				error("multi-segment packets are not supported - increase buffer size or decrease MTU");
			}

			struct e1000_rx_desc desc = *desc_ptr;
			struct pkt_buf* buf = (struct pkt_buf*) queue->virtual_addresses[rx_index];
			buf->size = desc.length;

			struct pkt_buf* new_buf = pkt_buf_alloc(queue->mempool);
			if (!new_buf) {
				error("failed to allocate new mbuf for rx");
			}

			// reset the descriptor
			desc_ptr->addr = new_buf->buf_addr_phy + offsetof(struct pkt_buf, data);
			desc_ptr->status = 0;
			queue->virtual_addresses[rx_index] = new_buf;

			//store data in output buffers
			bufs[buf_index] = buf;
			last_rx_index = rx_index;
			rx_index = wrap_ring(rx_index, queue->num_entries);
		} else {
			break;
		}
	}

	if (rx_index != last_rx_index) {
		set_reg32(dev->addr, E1000_RDT, last_rx_index);
		queue->rx_index = rx_index;
	}

	return buf_index;
}

// see section 3.4 and 3.4 for tx ring structure, general logic is as follows:
//  - hardware controls head to tx ring, driver controls tail of tx ring
//  - when data to transmit is done by hardware, it updates status DD
//    in tx descriptor and move the ring head
//  - when software sending data, it tries to clean previous done transmisions
//    [clean -> clean + batch_size] and update tx descriptors at tail
//  - tx ring structure:
//    [ <tx-desc> <tx-desc> ... <tx-desc> ... <tx-desc>]
//         |                       |             |
//       head                    clean          tail
// tx batching results in good performance gains
uint32_t e1000_tx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct e1000_device* dev = IXY_TO_E1000(ixy);
	if (queue_id != 0) {
		info("queue not exist");
		return 0;
	}
	struct e1000_tx_queue* queue = (struct e1000_tx_queue*)(dev->tx_queues);

	uint16_t clean_index = queue->clean_index; // next descriptor to clean up
	while (true) {
		int32_t cleanable = queue->tx_index - clean_index;
		if (cleanable < 0) { // handle wrap-around
			cleanable = queue->num_entries + cleanable;
		}
		if (cleanable < E1000_TX_CLEAN_BATCH) {
			break;
		}
		int32_t cleanup_to = clean_index + E1000_TX_CLEAN_BATCH - 1;
		if (cleanup_to >= queue->num_entries) {
			cleanup_to -= queue->num_entries;
		}
		volatile struct e1000_tx_desc* txd = queue->descriptors + cleanup_to;
		uint32_t status = txd->status;
		if (status & E1000_TXD_STAT_DD) {
			int32_t i = clean_index;
			while (true) {
				volatile struct e1000_tx_desc* r = queue->descriptors + i;
				if (!(r->status & E1000_TXD_STAT_DD)) {
					error("status ring: i=%d, status=%x",i, r->status);
					break;
				}
				struct pkt_buf* buf = queue->virtual_addresses[i];
				if (buf) {
					pkt_buf_free(buf);
				}
				if (i == cleanup_to) {
					break;
				}
				i = wrap_ring(i, queue->num_entries);
			}
			clean_index = wrap_ring(cleanup_to, queue->num_entries);
		}
	}
	queue->clean_index = clean_index;

	uint32_t sent;
	for (sent = 0; sent < num_bufs; sent++) {
		uint32_t next_index = wrap_ring(queue->tx_index, queue->num_entries);
		if (clean_index == next_index) {
			break;
		}
		struct pkt_buf* buf = bufs[sent];
		queue->virtual_addresses[queue->tx_index] = (void*) buf;
		volatile struct e1000_tx_desc* txd = queue->descriptors + queue->tx_index;
		txd->addr = (uint64_t)buf->buf_addr_phy + offsetof(struct pkt_buf, data);
		txd->length = (uint16_t)buf->size;
		txd->cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS;
		queue->tx_index = next_index;
	}
	set_reg32(dev->addr, E1000_TDT, queue->tx_index);

	return sent;
}

static void reset_and_init(struct e1000_device* dev) {
	info("Resetting device %s", dev->ixy.pci_addr);

	// disable interrupts
	uint32_t control = get_reg32(dev->addr, E1000_CTL);
	set_reg32(dev->addr, E1000_IMS, 0);
	set_reg32(dev->addr, E1000_CTL, control | E1000_CTL_RST);
	wait_clear_reg32(dev->addr, E1000_CTL, E1000_CTL_RST);

	// redisable interrupts
	set_reg32(dev->addr, E1000_IMS, 0);

	struct mac_address mac = e1000_get_mac_addr(&dev->ixy);
	info("Initializing device %s", dev->ixy.pci_addr);
	info("MAC address %02x:%02x:%02x:%02x:%02x:%02x", mac.addr[0], mac.addr[1], mac.addr[2], mac.addr[3], mac.addr[4], mac.addr[5]);

	// reset statistics
	e1000_read_stats(&dev->ixy, NULL);

	// init rx and tx queue
	init_tx(dev);
	init_rx(dev);

	// ask e1000 for receive interrupts.
	set_reg32(dev->addr, E1000_RDTR, 0); // interrupt after every received packet (no timer)
	set_reg32(dev->addr, E1000_RADV, 0); // interrupt after every packet (no timer)
	set_reg32(dev->addr, E1000_IMS, (1 << 7)); // RXDW -- Receiver Descriptor Write Back

	e1000_set_promisc(&dev->ixy, true);
	// set link up
	set_reg32(dev->addr, E1000_CTL, get_reg32(dev->addr, E1000_CTL) | E1000_CTL_SLU);
	wait_for_link(dev);

	info("Done resetting device %s, ctrl_reg=%x", dev->ixy.pci_addr, get_reg32(dev->addr, E1000_CTL)) ;
}

/**
 * Initializes and returns the e1000 device.
 * @param pci_addr The PCI address of the device.
 * @param rx_queues The number of receiver queues.
 * @param tx_queues The number of transmitter queues.
 * @param interrupt_timeout The interrupt timeout in milliseconds
 * 	- if set to -1 the interrupt timeout is disabled
 * 	- if set to 0 the interrupt is disabled entirely)
 * @return The initialized e1000 device.
 */
struct ixy_device* e1000_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues, int interrupt_timeout) {
  debug("init e1000");
	if (getuid()) {
		warn("Not running as root, this will probably fail");
	}
	if (rx_queues > MAX_QUEUES) {
		error("cannot configure %d rx queues: limit is %d", rx_queues, MAX_QUEUES);
	}
	if (tx_queues > MAX_QUEUES) {
		error("cannot configure %d tx queues: limit is %d", tx_queues, MAX_QUEUES);
	}

	// Allocate memory for the e1000 device that will be returned
	struct e1000_device* dev = (struct e1000_device*) malloc(sizeof(struct e1000_device));
	dev->ixy.pci_addr = strdup(pci_addr);

	dev->ixy.vfio = 0;
	dev->ixy.driver_name = driver_name;
	dev->ixy.num_rx_queues = rx_queues;
	dev->ixy.num_tx_queues = tx_queues;
	dev->ixy.rx_batch = e1000_rx_batch;
	dev->ixy.tx_batch = e1000_tx_batch;
	dev->ixy.read_stats = e1000_read_stats;
	dev->ixy.set_promisc = e1000_set_promisc;
	dev->ixy.get_link_speed = e1000_get_link_speed;
	dev->ixy.get_mac_addr = e1000_get_mac_addr;
	dev->ixy.set_mac_addr = e1000_set_mac_addr;
	dev->ixy.interrupts.interrupts_enabled = false;
	dev->ixy.interrupts.itr_rate = 0;
	dev->ixy.interrupts.timeout_ms = 0;

	debug("mapping BAR0 region via pci file...");
	dev->addr = pci_map_resource(pci_addr);
	dev->rx_queues = calloc(rx_queues, sizeof(struct e1000_rx_queue) + sizeof(void*) * E1000_MAX_RX_RING_SIZE);
	dev->tx_queues = calloc(tx_queues, sizeof(struct e1000_tx_queue) + sizeof(void*) * E1000_MAX_TX_RING_SIZE);
	reset_and_init(dev);
	return &dev->ixy;
}

bool is_e1000_compatible(uint16_t vendor_id, uint16_t device_id) {
	if (vendor_id != E1000_VENDOR_ID) {
		return false;
	}

	if (device_id == E1000_82540EMA_DEVICE_ID
		|| device_id == E1000_82540EPA_DEVICE_ID) {
		return true;
	}

	return false;
}

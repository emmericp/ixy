#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/limits.h>
#include <linux/vfio.h>
#include <sys/stat.h>

#include "log.h"
#include "ixgbe.h"
#include "pci.h"
#include "memory.h"
#include "driver/ixgbe_type.h"
#include "driver/device.h"

#include "libixy-vfio.h"
#include "interrupts.h"
#include "stats.h"

const char* driver_name = "ixy-ixgbe";

const int MAX_RX_QUEUE_ENTRIES = 4096;
const int MAX_TX_QUEUE_ENTRIES = 4096;

const int NUM_RX_QUEUE_ENTRIES = 512;
const int NUM_TX_QUEUE_ENTRIES = 512;

const int PKT_BUF_ENTRY_SIZE = 2048;
const int MIN_MEMPOOL_ENTRIES = 4096;

const int TX_CLEAN_BATCH = 32;

const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;

// allocated for each rx queue, keeps state for the receive function
struct ixgbe_rx_queue {
	volatile union ixgbe_adv_rx_desc* descriptors;
	struct mempool* mempool;
	uint16_t num_entries;
	// position we are reading from
	uint16_t rx_index;
	// virtual addresses to map descriptors back to their mbuf for freeing
	void* virtual_addresses[];
};

// allocated for each tx queue, keeps state for the transmit function
struct ixgbe_tx_queue {
	volatile union ixgbe_adv_tx_desc* descriptors;
	uint16_t num_entries;
	// position to clean up descriptors that where sent out by the nic
	uint16_t clean_index;
	// position to insert packets for transmission
	uint16_t tx_index;
	// virtual addresses to map descriptors back to their mbuf for freeing
	void* virtual_addresses[];
};

/**
 * Set the IVAR registers, mapping interrupt causes to vectors
 * @param dev pointer to device
 * @param direction 0 for Rx, 1 for Tx
 * @param queue queue to map the corresponding interrupt to
 * @param msix_vector the vector to map to the corresponding queue
 */
static void set_ivar(struct ixgbe_device* dev, int8_t direction, int8_t queue, int8_t msix_vector) {
	u32 ivar, index;
	msix_vector |= IXGBE_IVAR_ALLOC_VAL;
	index = ((16 * (queue & 1)) + (8 * direction));
	ivar = get_reg32(dev->addr, IXGBE_IVAR(queue >> 1));
	ivar &= ~(0xFF << index);
	ivar |= (msix_vector << index);
	set_reg32(dev->addr, IXGBE_IVAR(queue >> 1), ivar);
}

/**
 * Clear all interrupt masks for all queues.
 * @param dev The device.
 */
static void clear_interrupts(struct ixgbe_device* dev) {
	// Clear interrupt mask
	set_reg32(dev->addr, IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);
	get_reg32(dev->addr, IXGBE_EICR);
}

/**
 * Clear interrupt for queue.
 * @param dev The device.
 * @param queue_id The ID of the queue to clear.
 */
static void clear_interrupt(struct ixgbe_device* dev, uint16_t queue_id) {
	// Clear interrupt mask
	set_reg32(dev->addr, IXGBE_EIMC, 1 << queue_id);
	get_reg32(dev->addr, IXGBE_EICR);
}

/**
 * Disable all interrupts for all queues.
 * @param dev The device.
 */
static void disable_interrupts(struct ixgbe_device* dev) {
	// Clear interrupt mask to stop from interrupts being generated
	set_reg32(dev->addr, IXGBE_EIMS, 0x00000000);
	clear_interrupts(dev);
}

/**
 * Disable interrupt for queue
 * @param dev
 * @param queue_id The ID of the queue to disable.
 */
static void disable_interrupt(struct ixgbe_device* dev, uint16_t queue_id) {
	// Clear interrupt mask to stop from interrupts being generated
	u32 mask = get_reg32(dev->addr, IXGBE_EIMS);
	mask &= ~(1 << queue_id);
	set_reg32(dev->addr, IXGBE_EIMS, mask);
	clear_interrupt(dev, queue_id);
	debug("Using polling");
}

/**
 * Enable MSI interrupt for queue.
 * @param dev The device.
 * @param queue_id The ID of the queue to enable.
 */
static void enable_msi_interrupt(struct ixgbe_device* dev, uint16_t queue_id) {
	// Step 1: The software driver associates between Tx and Rx interrupt causes and the EICR
	// register by setting the IVAR[n] registers.
	set_ivar(dev, 0, queue_id, 0);

	// Step 2: Program SRRCTL[n].RDMTS (per receive queue) if software uses the receive
	// descriptor minimum threshold interrupt
	// We don't use the minimum threshold interrupt

	// Step 3: All interrupts should be set to 0b (no auto clear in the EIAC register). Following an
	// interrupt, software might read the EICR register to check for the interrupt causes.
	set_reg32(dev->addr, IXGBE_EIAC, 0x00000000);

	// Step 4: Set the auto mask in the EIAM register according to the preferred mode of operation.
	// In our case we prefer not auto-masking the interrupts

	// Step 5: Set the interrupt throttling in EITR[n] and GPIE according to the preferred mode of operation.
	set_reg32(dev->addr, IXGBE_EITR(queue_id), dev->ixy.interrupts.itr_rate);

	// Step 6: Software clears EICR by writing all ones to clear old interrupt causes
	clear_interrupts(dev);

	// Step 7: Software enables the required interrupt causes by setting the EIMS register
	u32 mask = get_reg32(dev->addr, IXGBE_EIMS);
	mask |= (1 << queue_id);
	set_reg32(dev->addr, IXGBE_EIMS, mask);
	debug("Using MSI interrupts");
}

/**
 * Enable MSI-X interrupt for queue.
 * @param dev The device.
 * @param queue_id The ID of the queue to enable.
 */
static void enable_msix_interrupt(struct ixgbe_device* dev, uint16_t queue_id) {
	// Step 1: The software driver associates between interrupt causes and MSI-X vectors and the
	// throttling timers EITR[n] by programming the IVAR[n] and IVAR_MISC registers.
	uint32_t gpie = get_reg32(dev->addr, IXGBE_GPIE);
	gpie |= IXGBE_GPIE_MSIX_MODE | IXGBE_GPIE_PBA_SUPPORT | IXGBE_GPIE_EIAME;
	set_reg32(dev->addr, IXGBE_GPIE, gpie);
	set_ivar(dev, 0, queue_id, queue_id);

	// Step 2: Program SRRCTL[n].RDMTS (per receive queue) if software uses the receive
	// descriptor minimum threshold interrupt
	// We don't use the minimum threshold interrupt

	// Step 3: The EIAC[n] registers should be set to auto clear for transmit and receive interrupt
	// causes (for best performance). The EIAC bits that control the other and TCP timer
	// interrupt causes should be set to 0b (no auto clear).
	set_reg32(dev->addr, IXGBE_EIAC, IXGBE_EIMS_RTX_QUEUE);

	// Step 4: Set the auto mask in the EIAM register according to the preferred mode of operation.
	// In our case we prefer to not auto-mask the interrupts

	// Step 5: Set the interrupt throttling in EITR[n] and GPIE according to the preferred mode of operation.
	// 0x000 (0us) => ... INT/s
	// 0x008 (2us) => 488200 INT/s
	// 0x010 (4us) => 244000 INT/s
	// 0x028 (10us) => 97600 INT/s
	// 0x0C8 (50us) => 20000 INT/s
	// 0x190 (100us) => 9766 INT/s
	// 0x320 (200us) => 4880 INT/s
	// 0x4B0 (300us) => 3255 INT/s
	// 0x640 (400us) => 2441 INT/s
	// 0x7D0 (500us) => 2000 INT/s
	// 0x960 (600us) => 1630 INT/s
	// 0xAF0 (700us) => 1400 INT/s
	// 0xC80 (800us) => 1220 INT/s
	// 0xE10 (900us) => 1080 INT/s
	// 0xFA7 (1000us) => 980 INT/s
	// 0xFFF (1024us) => 950 INT/s
	set_reg32(dev->addr, IXGBE_EITR(queue_id), dev->ixy.interrupts.itr_rate);

	// Step 6: Software enables the required interrupt causes by setting the EIMS register
	u32 mask = get_reg32(dev->addr, IXGBE_EIMS);
	mask |= (1 << queue_id);
	set_reg32(dev->addr, IXGBE_EIMS, mask);
	debug("Using MSIX interrupts");
}

/**
 * Enable MSI or MSI-X interrupt for queue depending on which is supported (Prefer MSI-x).
 * @param dev The device.
 * @param queue_id The ID of the queue to enable.
 */
static void enable_interrupt(struct ixgbe_device* dev, uint16_t queue_id) {
	if (!dev->ixy.interrupts.interrupts_enabled) {
		return;
	}
	switch (dev->ixy.interrupts.interrupt_type) {
		case VFIO_PCI_MSIX_IRQ_INDEX:
			enable_msix_interrupt(dev, queue_id);
			break;
		case VFIO_PCI_MSI_IRQ_INDEX:
			enable_msi_interrupt(dev, queue_id);
			break;
		default:
			warn("Interrupt type not supported: %d", dev->ixy.interrupts.interrupt_type);
			return;
	}
}

/**
 * Setup interrupts by enabling VFIO interrupts.
 * @param dev The device.
 */
static void setup_interrupts(struct ixgbe_device* dev) {
	if (!dev->ixy.interrupts.interrupts_enabled) {
		return;
	}
	dev->ixy.interrupts.queues = (struct interrupt_queues*) malloc(dev->ixy.num_rx_queues * sizeof(struct interrupt_queues));
	dev->ixy.interrupts.interrupt_type = vfio_setup_interrupt(dev->ixy.vfio_fd);
	switch (dev->ixy.interrupts.interrupt_type) {
		case VFIO_PCI_MSIX_IRQ_INDEX: {
			for (uint32_t rx_queue = 0; rx_queue < dev->ixy.num_rx_queues; rx_queue++) {
				int vfio_event_fd = vfio_enable_msix(dev->ixy.vfio_fd, rx_queue);
				int vfio_epoll_fd = vfio_epoll_ctl(vfio_event_fd);
				dev->ixy.interrupts.queues[rx_queue].vfio_event_fd = vfio_event_fd;
				dev->ixy.interrupts.queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				dev->ixy.interrupts.queues[rx_queue].moving_avg.length = 0;
				dev->ixy.interrupts.queues[rx_queue].moving_avg.index = 0;
				dev->ixy.interrupts.queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		case VFIO_PCI_MSI_IRQ_INDEX: {
			int vfio_event_fd = vfio_enable_msi(dev->ixy.vfio_fd);
			int vfio_epoll_fd = vfio_epoll_ctl(vfio_event_fd);
			for (uint32_t rx_queue = 0; rx_queue < dev->ixy.num_rx_queues; rx_queue++) {
				dev->ixy.interrupts.queues[rx_queue].vfio_event_fd = vfio_event_fd;
				dev->ixy.interrupts.queues[rx_queue].vfio_epoll_fd = vfio_epoll_fd;
				dev->ixy.interrupts.queues[rx_queue].moving_avg.length = 0;
				dev->ixy.interrupts.queues[rx_queue].moving_avg.index = 0;
				dev->ixy.interrupts.queues[rx_queue].interval = INTERRUPT_INITIAL_INTERVAL;
			}
			break;
		}
		default:
			warn("Interrupt type not supported: %d", dev->ixy.interrupts.interrupt_type);
			return;
	}
}

// see section 4.6.4
static void init_link(struct ixgbe_device* dev) {
	// should already be set by the eeprom config, maybe we shouldn't override it here to support weirdo nics?
	set_reg32(dev->addr, IXGBE_AUTOC, (get_reg32(dev->addr, IXGBE_AUTOC) & ~IXGBE_AUTOC_LMS_MASK) | IXGBE_AUTOC_LMS_10G_SERIAL);
	set_reg32(dev->addr, IXGBE_AUTOC, (get_reg32(dev->addr, IXGBE_AUTOC) & ~IXGBE_AUTOC_10G_PMA_PMD_MASK) | IXGBE_AUTOC_10G_XAUI);
	// negotiate link
	set_flags32(dev->addr, IXGBE_AUTOC, IXGBE_AUTOC_AN_RESTART);
	// datasheet wants us to wait for the link here, but we can continue and wait afterwards
}

static void start_rx_queue(struct ixgbe_device* dev, int queue_id) {
	debug("starting rx queue %d", queue_id);
	struct ixgbe_rx_queue* queue = ((struct ixgbe_rx_queue*)(dev->rx_queues)) + queue_id;
	// 2048 as pktbuf size is strictly speaking incorrect:
	// we need a few headers (1 cacheline), so there's only 1984 bytes left for the device
	// but the 82599 can only handle sizes in increments of 1 kb; but this is fine since our max packet size
	// is the default MTU of 1518
	// this has to be fixed if jumbo frames are to be supported
	// mempool should be >= the number of rx and tx descriptors for a forwarding application
	int mempool_size = NUM_RX_QUEUE_ENTRIES + NUM_TX_QUEUE_ENTRIES;
	queue->mempool = memory_allocate_mempool(mempool_size < MIN_MEMPOOL_ENTRIES ? MIN_MEMPOOL_ENTRIES : mempool_size, PKT_BUF_ENTRY_SIZE);
	if (queue->num_entries & (queue->num_entries - 1)) {
		error("number of queue entries must be a power of 2");
	}
	for (int i = 0; i < queue->num_entries; i++) {
		volatile union ixgbe_adv_rx_desc* rxd = queue->descriptors + i;
		struct pkt_buf* buf = pkt_buf_alloc(queue->mempool);
		if (!buf) {
			error("failed to allocate rx descriptor");
		}
		rxd->read.pkt_addr = buf->buf_addr_phy + offsetof(struct pkt_buf, data);
		rxd->read.hdr_addr = 0;
		// we need to return the virtual address in the rx function which the descriptor doesn't know by default
		queue->virtual_addresses[i] = buf;
	}
	// enable queue and wait if necessary
	set_flags32(dev->addr, IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
	wait_set_reg32(dev->addr, IXGBE_RXDCTL(queue_id), IXGBE_RXDCTL_ENABLE);
	// rx queue starts out full
	set_reg32(dev->addr, IXGBE_RDH(queue_id), 0);
	// was set to 0 before in the init function
	set_reg32(dev->addr, IXGBE_RDT(queue_id), queue->num_entries - 1);
}

static void start_tx_queue(struct ixgbe_device* dev, int queue_id) {
	debug("starting tx queue %d", queue_id);
	struct ixgbe_tx_queue* queue = ((struct ixgbe_tx_queue*)(dev->tx_queues)) + queue_id;
	if (queue->num_entries & (queue->num_entries - 1)) {
		error("number of queue entries must be a power of 2");
	}
	// tx queue starts out empty
	set_reg32(dev->addr, IXGBE_TDH(queue_id), 0);
	set_reg32(dev->addr, IXGBE_TDT(queue_id), 0);
	// enable queue and wait if necessary
	set_flags32(dev->addr, IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
	wait_set_reg32(dev->addr, IXGBE_TXDCTL(queue_id), IXGBE_TXDCTL_ENABLE);
}

// see section 4.6.7
// it looks quite complicated in the data sheet, but it's actually really easy because we don't need fancy features
static void init_rx(struct ixgbe_device* dev) {
	// make sure that rx is disabled while re-configuring it
	// the datasheet also wants us to disable some crypto-offloading related rx paths (but we don't care about them)
	clear_flags32(dev->addr, IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
	// no fancy dcb or vt, just a single 128kb packet buffer for us
	set_reg32(dev->addr, IXGBE_RXPBSIZE(0), IXGBE_RXPBSIZE_128KB);
	for (int i = 1; i < 8; i++) {
		set_reg32(dev->addr, IXGBE_RXPBSIZE(i), 0);
	}

	// always enable CRC offloading
	set_flags32(dev->addr, IXGBE_HLREG0, IXGBE_HLREG0_RXCRCSTRP);
	set_flags32(dev->addr, IXGBE_RDRXCTL, IXGBE_RDRXCTL_CRCSTRIP);

	// accept broadcast packets
	set_flags32(dev->addr, IXGBE_FCTRL, IXGBE_FCTRL_BAM);

	// per-queue config, same for all queues
	for (uint16_t i = 0; i < dev->ixy.num_rx_queues; i++) {
		debug("initializing rx queue %d", i);
		// enable advanced rx descriptors, we could also get away with legacy descriptors, but they aren't really easier
		set_reg32(dev->addr, IXGBE_SRRCTL(i), (get_reg32(dev->addr, IXGBE_SRRCTL(i)) & ~IXGBE_SRRCTL_DESCTYPE_MASK) | IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF);
		// drop_en causes the nic to drop packets if no rx descriptors are available instead of buffering them
		// a single overflowing queue can fill up the whole buffer and impact operations if not setting this flag
		set_flags32(dev->addr, IXGBE_SRRCTL(i), IXGBE_SRRCTL_DROP_EN);
		// setup descriptor ring, see section 7.1.9
		uint32_t ring_size_bytes = NUM_RX_QUEUE_ENTRIES * sizeof(union ixgbe_adv_rx_desc);
		struct dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
		// neat trick from Snabb: initialize to 0xFF to prevent rogue memory accesses on premature DMA activation
		memset(mem.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_reg32(dev->addr, IXGBE_RDBAL(i), (uint32_t) (mem.phy & 0xFFFFFFFFull));
		set_reg32(dev->addr, IXGBE_RDBAH(i), (uint32_t) (mem.phy >> 32));
		set_reg32(dev->addr, IXGBE_RDLEN(i), ring_size_bytes);
		debug("rx ring %d phy addr:  0x%012lX", i, mem.phy);
		debug("rx ring %d virt addr: 0x%012lX", i, (uintptr_t) mem.virt);
		// set ring to empty at start
		set_reg32(dev->addr, IXGBE_RDH(i), 0);
		set_reg32(dev->addr, IXGBE_RDT(i), 0);
		// private data for the driver, 0-initialized
		struct ixgbe_rx_queue* queue = ((struct ixgbe_rx_queue*)(dev->rx_queues)) + i;
		queue->num_entries = NUM_RX_QUEUE_ENTRIES;
		queue->rx_index = 0;
		queue->descriptors = (union ixgbe_adv_rx_desc*) mem.virt;
	}

	// last step is to set some magic bits mentioned in the last sentence in 4.6.7
	set_flags32(dev->addr, IXGBE_CTRL_EXT, IXGBE_CTRL_EXT_NS_DIS);
	// this flag probably refers to a broken feature: it's reserved and initialized as '1' but it must be set to '0'
	// there isn't even a constant in ixgbe_types.h for this flag
	for (uint16_t i = 0; i < dev->ixy.num_rx_queues; i++) {
		clear_flags32(dev->addr, IXGBE_DCA_RXCTRL(i), 1 << 12);
	}

	// start RX
	set_flags32(dev->addr, IXGBE_RXCTRL, IXGBE_RXCTRL_RXEN);
}

// see section 4.6.8
static void init_tx(struct ixgbe_device* dev) {
	// crc offload and small packet padding
	set_flags32(dev->addr, IXGBE_HLREG0, IXGBE_HLREG0_TXCRCEN | IXGBE_HLREG0_TXPADEN);

	// set default buffer size allocations
	// see also: section 4.6.11.3.4, no fancy features like DCB and VTd
	set_reg32(dev->addr, IXGBE_TXPBSIZE(0), IXGBE_TXPBSIZE_40KB);
	for (int i = 1; i < 8; i++) {
		set_reg32(dev->addr, IXGBE_TXPBSIZE(i), 0);
	}
	// required when not using DCB/VTd
	set_reg32(dev->addr, IXGBE_DTXMXSZRQ, 0xFFFF);
	clear_flags32(dev->addr, IXGBE_RTTDCS, IXGBE_RTTDCS_ARBDIS);

	// per-queue config for all queues
	for (uint16_t i = 0; i < dev->ixy.num_tx_queues; i++) {
		debug("initializing tx queue %d", i);

		// setup descriptor ring, see section 7.1.9
		uint32_t ring_size_bytes = NUM_TX_QUEUE_ENTRIES * sizeof(union ixgbe_adv_tx_desc);
		struct dma_memory mem = memory_allocate_dma(ring_size_bytes, true);
		memset(mem.virt, -1, ring_size_bytes);
		// tell the device where it can write to (its iova, so its view)
		set_reg32(dev->addr, IXGBE_TDBAL(i), (uint32_t) (mem.phy & 0xFFFFFFFFull));
		set_reg32(dev->addr, IXGBE_TDBAH(i), (uint32_t) (mem.phy >> 32));
		set_reg32(dev->addr, IXGBE_TDLEN(i), ring_size_bytes);
		debug("tx ring %d phy addr:  0x%012lX", i, mem.phy);
		debug("tx ring %d virt addr: 0x%012lX", i, (uintptr_t) mem.virt);

		// descriptor writeback magic values, important to get good performance and low PCIe overhead
		// see 7.2.3.4.1 and 7.2.3.5 for an explanation of these values and how to find good ones
		// we just use the defaults from DPDK here, but this is a potentially interesting point for optimizations
		uint32_t txdctl = get_reg32(dev->addr, IXGBE_TXDCTL(i));
		// there are no defines for this in ixgbe_type.h for some reason
		// pthresh: 6:0, hthresh: 14:8, wthresh: 22:16
		txdctl &= ~(0x7F | (0x7F << 8) | (0x7F << 16)); // clear bits
		txdctl |= (36 | (8 << 8) | (4 << 16)); // from DPDK
		set_reg32(dev->addr, IXGBE_TXDCTL(i), txdctl);

		// private data for the driver, 0-initialized
		struct ixgbe_tx_queue* queue = ((struct ixgbe_tx_queue*)(dev->tx_queues)) + i;
		queue->num_entries = NUM_TX_QUEUE_ENTRIES;
		queue->descriptors = (union ixgbe_adv_tx_desc*) mem.virt;
	}
	// final step: enable DMA
	set_reg32(dev->addr, IXGBE_DMATXCTL, IXGBE_DMATXCTL_TE);
}

static void wait_for_link(const struct ixgbe_device* dev) {
	info("Waiting for link...");
	int32_t max_wait = 10000000; // 10 seconds in us
	uint32_t poll_interval = 100000; // 10 ms in us
	while (!(ixgbe_get_link_speed(&dev->ixy)) && max_wait > 0) {
		usleep(poll_interval);
		max_wait -= poll_interval;
	}
	info("Link speed is %d Mbit/s", ixgbe_get_link_speed(&dev->ixy));
}

// see section 4.6.3
static void reset_and_init(struct ixgbe_device* dev) {
	info("Resetting device %s", dev->ixy.pci_addr);

	// section 4.6.3.1 - disable all interrupts
	disable_interrupts(dev);
	// section 4.6.3.2
	set_reg32(dev->addr, IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
	wait_clear_reg32(dev->addr, IXGBE_CTRL, IXGBE_CTRL_RST_MASK);
	usleep(10000);

	// section 4.6.3.1 - disable interrupts again after reset
	disable_interrupts(dev);

	struct mac_address mac = ixgbe_get_mac_addr(&dev->ixy);

	info("Initializing device %s", dev->ixy.pci_addr);
	info("MAC address %02x:%02x:%02x:%02x:%02x:%02x", mac.addr[0], mac.addr[1], mac.addr[2], mac.addr[3], mac.addr[4], mac.addr[5]);

	// section 4.6.3 - Wait for EEPROM auto read completion
	wait_set_reg32(dev->addr, IXGBE_EEC, IXGBE_EEC_ARD);

	// section 4.6.3 - Wait for DMA initialization done (RDRXCTL.DMAIDONE)
	wait_set_reg32(dev->addr, IXGBE_RDRXCTL, IXGBE_RDRXCTL_DMAIDONE);

	// section 4.6.4 - initialize link (auto negotiation)
	init_link(dev);

	// section 4.6.5 - statistical counters
	// reset-on-read registers, just read them once
	ixgbe_read_stats(&dev->ixy, NULL);

	// section 4.6.7 - init rx
	init_rx(dev);

	// section 4.6.8 - init tx
	init_tx(dev);

	// enables queues after initializing everything
	for (uint16_t i = 0; i < dev->ixy.num_rx_queues; i++) {
		start_rx_queue(dev, i);
	}
	for (uint16_t i = 0; i < dev->ixy.num_tx_queues; i++) {
		start_tx_queue(dev, i);
	}

	// enable interrupts
	for (uint16_t queue = 0; queue < dev->ixy.num_rx_queues; queue++) {
		enable_interrupt(dev, queue);
	}

	// finally, enable promisc mode by default, it makes testing less annoying
	ixgbe_set_promisc(&dev->ixy, true);

	// wait for some time for the link to come up
	wait_for_link(dev);
}

/**
 * Initializes and returns the IXGBE device.
 * @param pci_addr The PCI address of the device.
 * @param rx_queues The number of receiver queues.
 * @param tx_queues The number of transmitter queues.
 * @param interrupt_timeout The interrupt timeout in milliseconds
 * 	- if set to -1 the interrupt timeout is disabled
 * 	- if set to 0 the interrupt is disabled entirely)
 * @return The initialized IXGBE device.
 */
struct ixy_device* ixgbe_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues, int interrupt_timeout) {
	if (getuid()) {
		warn("Not running as root, this will probably fail");
	}
	if (rx_queues > MAX_QUEUES) {
		error("cannot configure %d rx queues: limit is %d", rx_queues, MAX_QUEUES);
	}
	if (tx_queues > MAX_QUEUES) {
		error("cannot configure %d tx queues: limit is %d", tx_queues, MAX_QUEUES);
	}

	// Allocate memory for the ixgbe device that will be returned
	struct ixgbe_device* dev = (struct ixgbe_device*) malloc(sizeof(struct ixgbe_device));
	dev->ixy.pci_addr = strdup(pci_addr);

	// Check if we want the VFIO stuff
	// This is done by checking if the device is in an IOMMU group.
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/iommu_group", pci_addr);
	struct stat buffer;
	dev->ixy.vfio = stat(path, &buffer) == 0;
	if (dev->ixy.vfio) {
		// initialize the IOMMU for this device
		dev->ixy.vfio_fd = vfio_init(pci_addr);
		if (dev->ixy.vfio_fd < 0) {
			error("could not initialize the IOMMU for device %s", pci_addr);
		}
	}
	dev->ixy.driver_name = driver_name;
	dev->ixy.num_rx_queues = rx_queues;
	dev->ixy.num_tx_queues = tx_queues;
	dev->ixy.rx_batch = ixgbe_rx_batch;
	dev->ixy.tx_batch = ixgbe_tx_batch;
	dev->ixy.read_stats = ixgbe_read_stats;
	dev->ixy.set_promisc = ixgbe_set_promisc;
	dev->ixy.get_link_speed = ixgbe_get_link_speed;
	dev->ixy.get_mac_addr = ixgbe_get_mac_addr;
	dev->ixy.set_mac_addr = ixgbe_set_mac_addr;
	dev->ixy.interrupts.interrupts_enabled = interrupt_timeout != 0;
	// 0x028 (10ys) => 97600 INT/s
	dev->ixy.interrupts.itr_rate = 0x028;
	dev->ixy.interrupts.timeout_ms = interrupt_timeout;

	if (!dev->ixy.vfio && interrupt_timeout != 0) {
		warn("Interrupts requested but VFIO not available: Disabling Interrupts!");
		dev->ixy.interrupts.interrupts_enabled = false;
	}

	// Map BAR0 region
	if (dev->ixy.vfio) {
		debug("mapping BAR0 region via VFIO...");
		dev->addr = vfio_map_region(dev->ixy.vfio_fd, VFIO_PCI_BAR0_REGION_INDEX);
		// initialize interrupts for this device
		setup_interrupts(dev);
	} else {
		debug("mapping BAR0 region via pci file...");
		dev->addr = pci_map_resource(pci_addr);
	}
	dev->rx_queues = calloc(rx_queues, sizeof(struct ixgbe_rx_queue) + sizeof(void*) * MAX_RX_QUEUE_ENTRIES);
	dev->tx_queues = calloc(tx_queues, sizeof(struct ixgbe_tx_queue) + sizeof(void*) * MAX_TX_QUEUE_ENTRIES);
	reset_and_init(dev);
	return &dev->ixy;
}

uint32_t ixgbe_get_link_speed(const struct ixy_device* ixy) {
	struct ixgbe_device* dev = IXY_TO_IXGBE(ixy);
	uint32_t links = get_reg32(dev->addr, IXGBE_LINKS);
	if (!(links & IXGBE_LINKS_UP)) {
		return 0;
	}
	switch (links & IXGBE_LINKS_SPEED_82599) {
		case IXGBE_LINKS_SPEED_100_82599:
			return 100;
		case IXGBE_LINKS_SPEED_1G_82599:
			return 1000;
		case IXGBE_LINKS_SPEED_10G_82599:
			return 10000;
		default:
			return 0;
	}
}

struct mac_address ixgbe_get_mac_addr(const struct ixy_device* ixy) {
	struct mac_address mac;
	struct ixgbe_device* dev = IXY_TO_IXGBE(ixy);

	uint32_t rar_low = get_reg32(dev->addr, IXGBE_RAL(0));
	uint32_t rar_high = get_reg32(dev->addr, IXGBE_RAH(0));

	mac.addr[0] = rar_low;
	mac.addr[1] = rar_low >> 8;
	mac.addr[2] = rar_low >> 16;
	mac.addr[3] = rar_low >> 24;
	mac.addr[4] = rar_high;
	mac.addr[5] = rar_high >> 8;

	return mac;
}

void ixgbe_set_mac_addr(struct ixy_device* ixy, struct mac_address mac) {
	struct ixgbe_device* dev = IXY_TO_IXGBE(ixy);

	uint32_t rar_low = mac.addr[0] + (mac.addr[1] << 8) + (mac.addr[2] << 16) + (mac.addr[3] << 24);
	uint32_t rar_high = mac.addr[4] + (mac.addr[5] << 8);

	set_reg32(dev->addr, IXGBE_RAL(0), rar_low);
	set_reg32(dev->addr, IXGBE_RAH(0), rar_high);
}

void ixgbe_set_promisc(struct ixy_device* ixy, bool enabled) {
	struct ixgbe_device* dev = IXY_TO_IXGBE(ixy);
	if (enabled) {
		info("enabling promisc mode");
		set_flags32(dev->addr, IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
	} else {
		info("disabling promisc mode");
		clear_flags32(dev->addr, IXGBE_FCTRL, IXGBE_FCTRL_MPE | IXGBE_FCTRL_UPE);
	}
}

// read stat counters and accumulate in stats
// stats may be NULL to just reset the counters
void ixgbe_read_stats(struct ixy_device* ixy, struct device_stats* stats) {
	struct ixgbe_device* dev = IXY_TO_IXGBE(ixy);
	uint32_t rx_pkts = get_reg32(dev->addr, IXGBE_GPRC);
	uint32_t tx_pkts = get_reg32(dev->addr, IXGBE_GPTC);
	uint64_t rx_bytes = get_reg32(dev->addr, IXGBE_GORCL) + (((uint64_t) get_reg32(dev->addr, IXGBE_GORCH)) << 32);
	uint64_t tx_bytes = get_reg32(dev->addr, IXGBE_GOTCL) + (((uint64_t) get_reg32(dev->addr, IXGBE_GOTCH)) << 32);
	if (stats) {
		stats->rx_pkts += rx_pkts;
		stats->tx_pkts += tx_pkts;
		stats->rx_bytes += rx_bytes;
		stats->tx_bytes += tx_bytes;
	}
}

// advance index with wrap-around, this line is the reason why we require a power of two for the queue size
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))

// section 1.8.2 and 7.1
// try to receive a single packet if one is available, non-blocking
// see datasheet section 7.1.9 for an explanation of the rx ring structure
// tl;dr: we control the tail of the queue, the hardware the head
uint32_t ixgbe_rx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct ixgbe_device* dev = IXY_TO_IXGBE(ixy);

	struct interrupt_queues* interrupt = NULL;
	bool interrupts_enabled = ixy->interrupts.interrupts_enabled;

	if (interrupts_enabled) {
		interrupt = &ixy->interrupts.queues[queue_id];
	}

	if (interrupts_enabled && interrupt->interrupt_enabled) {
		vfio_epoll_wait(interrupt->vfio_epoll_fd, 10, dev->ixy.interrupts.timeout_ms);
	}

	struct ixgbe_rx_queue* queue = ((struct ixgbe_rx_queue*) (dev->rx_queues)) + queue_id;
	uint16_t rx_index = queue->rx_index; // rx index we checked in the last run of this function
	uint16_t last_rx_index = rx_index; // index of the descriptor we checked in the last iteration of the loop
	uint32_t buf_index;
	for (buf_index = 0; buf_index < num_bufs; buf_index++) {
		// rx descriptors are explained in 7.1.5
		volatile union ixgbe_adv_rx_desc* desc_ptr = queue->descriptors + rx_index;
		uint32_t status = desc_ptr->wb.upper.status_error;
		if (status & IXGBE_RXDADV_STAT_DD) {
			if (!(status & IXGBE_RXDADV_STAT_EOP)) {
				error("multi-segment packets are not supported - increase buffer size or decrease MTU");
			}
			// got a packet, read and copy the whole descriptor
			union ixgbe_adv_rx_desc desc = *desc_ptr;
			struct pkt_buf* buf = (struct pkt_buf*) queue->virtual_addresses[rx_index];
			buf->size = desc.wb.upper.length;
			// this would be the place to implement RX offloading by translating the device-specific flags
			// to an independent representation in the buf (similiar to how DPDK works)
			// need a new mbuf for the descriptor
			struct pkt_buf* new_buf = pkt_buf_alloc(queue->mempool);
			if (!new_buf) {
				// we could handle empty mempools more gracefully here, but it would be quite messy...
				// make your mempools large enough
				error("failed to allocate new mbuf for rx, you are either leaking memory or your mempool is too small");
			}
			// reset the descriptor
			desc_ptr->read.pkt_addr = new_buf->buf_addr_phy + offsetof(struct pkt_buf, data);
			desc_ptr->read.hdr_addr = 0; // this resets the flags
			queue->virtual_addresses[rx_index] = new_buf;
			bufs[buf_index] = buf;
			// want to read the next one in the next iteration, but we still need the last/current to update RDT later
			last_rx_index = rx_index;
			rx_index = wrap_ring(rx_index, queue->num_entries);
		} else {
			break;
		}
	}
	if (rx_index != last_rx_index) {
		// tell hardware that we are done
		// this is intentionally off by one, otherwise we'd set RDT=RDH if we are receiving faster than packets are coming in
		// RDT=RDH means queue is full
		set_reg32(dev->addr, IXGBE_RDT(queue_id), last_rx_index);
		queue->rx_index = rx_index;
	}

	if (interrupts_enabled) {
		interrupt->rx_pkts += buf_index;

		if ((interrupt->instr_counter++ & 0xFFF) == 0) {
			bool int_en = interrupt->interrupt_enabled;
			uint64_t diff = monotonic_time() - interrupt->last_time_checked;
			if (diff > interrupt->interval) {
				// every second
				check_interrupt(interrupt, diff, buf_index, num_bufs);
			}

			if (int_en != interrupt->interrupt_enabled) {
				if (interrupt->interrupt_enabled) {
					enable_interrupt(dev, queue_id);
				} else {
					disable_interrupt(dev, queue_id);
				}
			}
		}
	}

	return buf_index; // number of packets stored in bufs; buf_index points to the next index
}

// section 1.8.1 and 7.2
// we control the tail, hardware the head
// huge performance gains possible here by sending packets in batches - writing to TDT for every packet is not efficient
// returns the number of packets transmitted, will not block when the queue is full
uint32_t ixgbe_tx_batch(struct ixy_device* ixy, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	struct ixgbe_device* dev = IXY_TO_IXGBE(ixy);
	struct ixgbe_tx_queue* queue = ((struct ixgbe_tx_queue*)(dev->tx_queues)) + queue_id;
	// the descriptor is explained in section 7.2.3.2.4
	// we just use a struct copy & pasted from intel, but it basically has two formats (hence a union):
	// 1. the write-back format which is written by the NIC once sending it is finished this is used in step 1
	// 2. the read format which is read by the NIC and written by us, this is used in step 2

	uint16_t clean_index = queue->clean_index; // next descriptor to clean up

	// step 1: clean up descriptors that were sent out by the hardware and return them to the mempool
	// start by reading step 2 which is done first for each packet
	// cleaning up must be done in batches for performance reasons, so this is unfortunately somewhat complicated
	while (true) {
		// figure out how many descriptors can be cleaned up
		int32_t cleanable = queue->tx_index - clean_index; // tx_index is always ahead of clean (invariant of our queue)
		if (cleanable < 0) { // handle wrap-around
			cleanable = queue->num_entries + cleanable;
		}
		if (cleanable < TX_CLEAN_BATCH) {
			break;
		}
		// calculcate the index of the last transcriptor in the clean batch
		// we can't check all descriptors for performance reasons
		int32_t cleanup_to = clean_index + TX_CLEAN_BATCH - 1;
		if (cleanup_to >= queue->num_entries) {
			cleanup_to -= queue->num_entries;
		}
		volatile union ixgbe_adv_tx_desc* txd = queue->descriptors + cleanup_to;
		uint32_t status = txd->wb.status;
		// hardware sets this flag as soon as it's sent out, we can give back all bufs in the batch back to the mempool
		if (status & IXGBE_ADVTXD_STAT_DD) {
			int32_t i = clean_index;
			while (true) {
				struct pkt_buf* buf = queue->virtual_addresses[i];
				pkt_buf_free(buf);
				if (i == cleanup_to) {
					break;
				}
				i = wrap_ring(i, queue->num_entries);
			}
			// next descriptor to be cleaned up is one after the one we just cleaned
			clean_index = wrap_ring(cleanup_to, queue->num_entries);
		} else {
			// clean the whole batch or nothing; yes, this leaves some packets in
			// the queue forever if you stop transmitting, but that's not a real concern
			break;
		}
	}
	queue->clean_index = clean_index;

	// step 2: send out as many of our packets as possible
	uint32_t sent;
	for (sent = 0; sent < num_bufs; sent++) {
		uint32_t next_index = wrap_ring(queue->tx_index, queue->num_entries);
		// we are full if the next index is the one we are trying to reclaim
		if (clean_index == next_index) {
			break;
		}
		struct pkt_buf* buf = bufs[sent];
		// remember virtual address to clean it up later
		queue->virtual_addresses[queue->tx_index] = (void*) buf;
		volatile union ixgbe_adv_tx_desc* txd = queue->descriptors + queue->tx_index;
		queue->tx_index = next_index;
		// NIC reads from here
		txd->read.buffer_addr = buf->buf_addr_phy + offsetof(struct pkt_buf, data);
		// always the same flags: one buffer (EOP), advanced data descriptor, CRC offload, data length
		txd->read.cmd_type_len =
			IXGBE_ADVTXD_DCMD_EOP | IXGBE_ADVTXD_DCMD_RS | IXGBE_ADVTXD_DCMD_IFCS | IXGBE_ADVTXD_DCMD_DEXT | IXGBE_ADVTXD_DTYP_DATA | buf->size;
		// no fancy offloading stuff - only the total payload length
		// implement offloading flags here:
		// 	* ip checksum offloading is trivial: just set the offset
		// 	* tcp/udp checksum offloading is more annoying, you have to precalculate the pseudo-header checksum
		txd->read.olinfo_status = buf->size << IXGBE_ADVTXD_PAYLEN_SHIFT;
	}
	// send out by advancing tail, i.e., pass control of the bufs to the nic
	// this seems like a textbook case for a release memory order, but Intel's driver doesn't even use a compiler barrier here
	set_reg32(dev->addr, IXGBE_TDT(queue_id), queue->tx_index);
	return sent;
}

#ifndef IXY_DEVICE_H
#define IXY_DEVICE_H

#include <stdint.h>
#include <unistd.h>

#include "log.h"
#include "memory.h"

#define MAX_QUEUES 64

// Forward declare struct to prevent cyclic include with stats.h
struct device_stats;

/**
 * container_of - cast a member of a structure out to the containing structure
 * Adapted from the Linux kernel.
 * This allows us to expose the same struct for all drivers to the user's
 * application and cast it to a driver-specific struct in the driver.
 * A simple cast would be sufficient if we always store it at the same offset.
 * This macro looks more complicated than it is, a good explanation can be
 * found at http://www.kroah.com/log/linux/container_of.html
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({\
	const typeof(((type*)0)->member)* __mptr = (ptr);\
	(type*)((char*)__mptr - offsetof(type, member));\
})

struct ixy_device {
	const char* pci_addr;
	const char* driver_name;
	uint16_t num_rx_queues;
	uint16_t num_tx_queues;
	uint32_t (*rx_batch) (struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);
	uint32_t (*tx_batch) (struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);
	void (*read_stats) (struct ixy_device* dev, struct device_stats* stats);
	void (*set_promisc) (struct ixy_device* dev, bool enabled);
	uint32_t (*get_link_speed) (const struct ixy_device* dev);
};

struct ixy_device* ixy_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues);

// Public stubs that forward the calls to the driver-specific implementations
static inline uint32_t ixy_rx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	return dev->rx_batch(dev, queue_id, bufs, num_bufs);
}

static inline uint32_t ixy_tx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	return dev->tx_batch(dev, queue_id, bufs, num_bufs);
}

static inline void ixy_read_stats(struct ixy_device* dev, struct device_stats* stats) {
	dev->read_stats(dev, stats);
}

static inline void ixy_set_promisc(struct ixy_device* dev, bool enabled) {
	dev->set_promisc(dev, enabled);
}

static inline uint32_t get_link_speed(const struct ixy_device* dev) {
	return dev->get_link_speed(dev);
}

// calls ixy_tx_batch until all packets are queued with busy waiting
static void ixy_tx_batch_busy_wait(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	uint32_t num_sent = 0;
	while ((num_sent += ixy_tx_batch(dev, 0, bufs + num_sent, num_bufs - num_sent)) != num_bufs) {
		// busy wait
	}
}

// getters/setters for PCIe memory mapped registers
// this code looks like it's in need of some memory barrier intrinsics, but that's apparently not needed on x86
// dpdk has release/acquire memory order calls before/after the memory accesses, but they are defined as
// simple compiler barriers (i.e., the same empty asm with dependency on memory as here) on x86
// dpdk also defines an additional relaxed load/store for the registers that only uses a volatile access,  we skip that for simplicity

static inline void set_reg32(uint8_t* addr, int reg, uint32_t value) {
	__asm__ volatile ("" : : : "memory");
	*((volatile uint32_t*) (addr + reg)) = value;
}

static inline uint32_t get_reg32(const uint8_t* addr, int reg) {
	__asm__ volatile ("" : : : "memory");
	return *((volatile uint32_t*) (addr + reg));
}

static inline void set_flags32(uint8_t* addr, int reg, uint32_t flags) {
	set_reg32(addr, reg, get_reg32(addr, reg) | flags);
}

static inline void clear_flags32(uint8_t* addr, int reg, uint32_t flags) {
	set_reg32(addr, reg, get_reg32(addr, reg) & ~flags);
}

static inline void wait_clear_reg32(const uint8_t* addr, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != 0) {
		debug("waiting for flags 0x%08X in register 0x%05X to clear, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
	}
}

static inline void wait_set_reg32(const uint8_t* addr, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != mask) {
		debug("waiting for flags 0x%08X in register 0x%05X, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
	}
}

// getters/setters for pci io port resources

static inline void write_io32(int fd, uint32_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline void write_io16(int fd, uint16_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline void write_io8(int fd, uint8_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline uint32_t read_io32(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint32_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		error("pread io resource");
	return temp;
}

static inline uint16_t read_io16(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint16_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		error("pread io resource");
	return temp;
}

static inline uint8_t read_io8(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint8_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		error("pread io resource");
	return temp;
}

#endif // IXY_DEVICE_H

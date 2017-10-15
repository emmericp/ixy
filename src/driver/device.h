#ifndef IXY_DEVICE_H
#define IXY_DEVICE_H

#include <stdint.h>
#include <unistd.h>

#include "log.h"

#define MAX_QUEUES 64

struct ixy_device {
	const char* pci_addr;
	const char* driver_name;
	uint8_t* addr;
	uint16_t num_rx_queues;
	uint16_t num_tx_queues;
	// allow drivers to keep some state for queues, opaque pointer cast by the driver
	void* rx_queues;
	void* tx_queues;
};

static inline void set_reg32(struct ixy_device* dev, int reg, uint32_t value) {
	__asm__ volatile ("" : : : "memory");
	*((volatile uint32_t*) (dev->addr + reg)) = value;
}

static inline uint32_t get_reg32(const struct ixy_device* dev, int reg) {
	__asm__ volatile ("" : : : "memory");
	return *((volatile uint32_t*) (dev->addr + reg));
}

static inline void set_flags32(struct ixy_device* dev, int reg, uint32_t flags) {
	set_reg32(dev, reg, get_reg32(dev, reg) | flags);
}

static inline void clear_flags32(struct ixy_device* dev, int reg, uint32_t flags) {
	set_reg32(dev, reg, get_reg32(dev, reg) & ~flags);
}

static inline void wait_clear_reg32(const struct ixy_device* dev, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (dev->addr + reg)), (cur & mask) != 0) {
		debug("waiting for flags 0x%08X in register 0x%05X to clear, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
		__sync_synchronize();
	}
}

static inline void wait_set_reg32(const struct ixy_device* dev, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (dev->addr + reg)), (cur & mask) != mask) {
		debug("waiting for flags 0x%08X in register 0x%05X, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
	}
}


#endif //IXY_DEVICE_H

#ifndef IXY_IXGBE_H
#define IXY_IXGBE_H

#include <stdbool.h>
#include "stats.h"
#include "memory.h"

struct ixy_device* ixgbe_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues);
uint32_t ixgbe_get_link_speed(const struct ixy_device* dev);
void ixgbe_set_promisc(struct ixy_device* dev, bool enabled);
void ixgbe_read_stats(struct ixy_device* dev, struct device_stats* stats);
uint32_t ixgbe_tx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);
uint32_t ixgbe_rx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);

// calls ixgbe_tx_batch until all packets are queued with busy waiting
static void ixgbe_tx_batch_busy_wait(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs) {
	uint32_t num_sent = 0;
	while ((num_sent += ixgbe_tx_batch(dev, 0, bufs + num_sent, num_bufs - num_sent)) != num_bufs) {
		// busy wait
	}
}

#endif //IXY_IXGBE_H

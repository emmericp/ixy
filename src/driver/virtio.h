#ifndef IXY_VIRTIO_H
#define IXY_VIRTIO_H

#include <stdbool.h>
#include "stats.h"
#include "memory.h"

struct virtio_device {
	struct ixy_device ixy;
	int fd;
	void* rx_queue;
	void* tx_queue;
	void* ctrl_queue;
	uint64_t rx_pkts;
	uint64_t tx_pkts;
	uint64_t rx_bytes;
	uint64_t tx_bytes;
};

#define IXY_TO_VIRTIO(ixy_device) container_of(ixy_device, struct virtio_device, ixy)

struct ixy_device* virtio_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues);
uint32_t virtio_get_link_speed(const struct ixy_device* dev);
void virtio_set_promisc(struct ixy_device* dev, bool enabled);
void virtio_read_stats(struct ixy_device* dev, struct device_stats* stats);
uint32_t virtio_tx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);
uint32_t virtio_rx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);

#endif // IXY_VIRTIO_H

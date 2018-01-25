#ifndef IXY_IXGBE_H
#define IXY_IXGBE_H

#include <stdbool.h>
#include "stats.h"
#include "memory.h"

struct ixgbe_device {
    struct ixy_device ixy;
    uint8_t* addr;
    void* rx_queues;
    void* tx_queues;
};

#define IXY_TO_IXGBE(ixy_device) container_of(ixy_device, struct ixgbe_device, ixy)

struct ixy_device* ixgbe_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues);
uint32_t ixgbe_get_link_speed(const struct ixy_device* dev);
void ixgbe_set_promisc(struct ixy_device* dev, bool enabled);
void ixgbe_read_stats(struct ixy_device* dev, struct device_stats* stats);
uint32_t ixgbe_tx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);
uint32_t ixgbe_rx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);

#endif //IXY_IXGBE_H

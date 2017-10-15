#ifndef IXY_IXGBE_H
#define IXY_IXGBE_H

#include <stdbool.h>
#include "stats.h"

struct ixy_device* ixgbe_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues);
uint32_t ixgbe_get_link_speed(const struct ixy_device* dev);
void ixgbe_set_promisc(struct ixy_device* dev, bool enabled);
struct pkt_buf* ixgbe_rx_packet(struct ixy_device* dev, uint16_t queue_id);
void ixgbe_read_stats(struct ixy_device* dev, struct device_stats* stats);
uint16_t ixgbe_tx_packet(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* buf);


#endif //IXY_IXGBE_H

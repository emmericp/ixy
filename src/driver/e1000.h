#ifndef IXY_E1000_H
#define IXY_E1000_H

#include <stdbool.h>
#include "stats.h"
#include "memory.h"

struct e1000_device {
	struct ixy_device ixy;
	uint8_t* addr;
	void* rx_queues;
	void* tx_queues;
};

#define IXY_TO_E1000(ixy_device) container_of(ixy_device, struct e1000_device, ixy)

struct ixy_device* e1000_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues, int interrupt_timeout);
uint32_t e1000_get_link_speed(const struct ixy_device* dev);
struct mac_address e1000_get_mac_addr(const struct ixy_device* dev);
void e1000_set_mac_addr(struct ixy_device* dev, struct mac_address mac);
void e1000_set_promisc(struct ixy_device* dev, bool enabled);
void e1000_read_stats(struct ixy_device* dev, struct device_stats* stats);
uint32_t e1000_tx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);
uint32_t e1000_rx_batch(struct ixy_device* dev, uint16_t queue_id, struct pkt_buf* bufs[], uint32_t num_bufs);
bool is_e1000_compatible(uint16_t vendor_id, uint16_t device_id);

#endif //IXY_E1000_H

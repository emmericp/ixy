#include "device.h"
#include "driver/ixgbe.h"

struct ixy_device* ixy_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues) {
	if (true /* Detection magic */) {
		return ixgbe_init(pci_addr, rx_queues, tx_queues);
	} else {
		error("Unsupported device");
	}
}

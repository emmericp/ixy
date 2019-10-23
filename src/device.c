#include <sys/file.h>

#include "device.h"
#include "ixgbe.h"
#include "pci.h"

struct ixy_device* ixy_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues) {
	// Read PCI configuration space
	// every config file should be world-readable, and here we
	// only read the vendor and device id.
	int config = pci_open_resource(pci_addr, "config", O_RDONLY);
	uint32_t class_id;
	pread(config, &class_id, sizeof(class_id), 8);
	class_id >>= 24;
	close(config);
	if (class_id != 2) {
		error("Device %s is not a NIC", pci_addr);
	}
	// Our best guess is to try ixgbe
	return ixgbe_init(pci_addr, rx_queues, tx_queues);
}

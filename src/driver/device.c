#include "device.h"
#include "driver/ixgbe.h"
#include "driver/virtio.h"
#include "pci.h"

struct ixy_device* ixy_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues) {
	// Read PCI configuration space
	int config = pci_open_resource(pci_addr, "config");
	uint16_t vendor_id = read_io16(config, 0);
	uint16_t device_id = read_io16(config, 2);
	uint32_t class_id = read_io32(config, 8) >> 24;
	close(config);
	if (class_id != 2) {
		error("Device %s is not a NIC", pci_addr);
	}
	if (vendor_id == 0x1af4 && device_id >= 0x1000) {
		return virtio_init(pci_addr, rx_queues, tx_queues);
	} else {
		// Our best guess is to try ixgbe
		return ixgbe_init(pci_addr, rx_queues, tx_queues);
	}
}

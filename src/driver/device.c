#include <sys/file.h>

#include "device.h"
#include "driver/e1000.h"
#include "driver/ixgbe.h"
#include "driver/virtio.h"
#include "pci.h"

struct ixy_device* ixy_init(const char* pci_addr, uint16_t rx_queues, uint16_t tx_queues, int interrupt_timeout) {
	// Read PCI configuration space
	// For VFIO, we could access the config space another way
	// (VFIO_PCI_CONFIG_REGION_INDEX). This is not needed, though, because
	// every config file should be world-readable, and here we
	// only read the vendor and device id.
	int config = pci_open_resource(pci_addr, "config", O_RDONLY);
	uint16_t vendor_id = read_io16(config, 0);
	uint16_t device_id = read_io16(config, 2);
	uint32_t class_id = read_io32(config, 8) >> 24;
	close(config);
	if (class_id != 2) {
		error("Device %s is not a NIC", pci_addr);
	}
	if (vendor_id == 0x1af4 && device_id >= 0x1000) {
		return virtio_init(pci_addr, rx_queues, tx_queues);
	} else if (is_e1000_compatible(vendor_id, device_id)) {
		return e1000_init(pci_addr, rx_queues, tx_queues, interrupt_timeout);
	} else {
		// Our best guess is to try ixgbe
		return ixgbe_init(pci_addr, rx_queues, tx_queues, interrupt_timeout);
	}
}

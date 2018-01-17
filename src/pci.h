#ifndef IXY_PCI_H
#define IXY_PCI_H

#include <stdint.h>

void remove_driver(const char* pci_addr);
void enable_dma(const char* pci_addr);
uint8_t* pci_map_resource(const char* bus_id);
int pci_open_resource(const char* pci_addr, const char* resource);

#endif // IXY_PCI_H

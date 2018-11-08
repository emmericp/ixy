#ifndef LIBIXY_VFIO_H
#define LIBIXY_VFIO_H

#include <stdint.h>

// enables DMA on a VFIO device
void vfio_enable_dma(int device_fd);

// initializes the IOMMU for the device. returns the devices file descriptor or
// -1 on error
int vfio_init(const char* pci_addr);

// returns a uint8_t pointer to the MMAPED region or MAP_FAILED if failed.
// region_index is to be taken from linux/vfio.h
uint8_t* vfio_map_region(int vfio_fd, int region_index);

// returns iova (physical address of the DMA memory from device view) on success
// or -1 else
uint64_t vfio_map_dma(void* vaddr, uint32_t size);

// unmaps previously mapped DMA region. returns 0 on success
uint64_t vfio_unmap_dma(int fd, uint64_t iova, uint32_t size);

#endif //LIBIXY_VFIO_H

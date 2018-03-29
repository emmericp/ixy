#ifndef IXY_VFIO_H
#define IXY_VFIO_H

#include <stdint.h>

struct ixy_device;

int vfio_init(struct ixy_device* dev);
int vfio_map_dma(struct ixy_device* dev, uint64_t vaddr, uint64_t iova, uint32_t size);

#endif //IXY_VFIO_H

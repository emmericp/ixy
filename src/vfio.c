#ifdef USE_VFIO

#include "vfio.h"
#include "log.h"
#include "driver/device.h"

#include <fcntl.h>
#include <libgen.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/vfio.h>

int vfio_init(struct ixy_device* dev){
	debug("Initialize vfio");
	// find iommu group for the device
	// `readlink /sys/bus/pci/device/<segn:busn:devn.funcn>/iommu_group`
	char path[128], iommu_group_path[128];
	struct stat st;
	snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/", dev->pci_addr);
	int ret = stat(path, &st);
	if(ret < 0){
		warn("No such device: %s", path);
		return -1;
	}
	strncat(path, "iommu_group", sizeof(path) - strlen(path) - 1);

	int len = readlink(path, iommu_group_path, sizeof(iommu_group_path));
	if(len <= 0){
		warn("No iommu_group for device");
		return -1;
	}

	iommu_group_path[len] = '\0';
	char* group_name = basename(iommu_group_path);
	int groupid;
	ret = sscanf(group_name, "%d", &groupid);
	if(ret != 1){
		warn("Unkonwn group");
		return -1;
	}

	// open vfio file
	dev->vfio_cfd = open("/dev/vfio/vfio", O_RDWR);
	if(dev->vfio_cfd < 0){
		warn("Failed to open /dev/vfio/vfio");
		return -1;
	}

	snprintf(path, sizeof(path), "/dev/vfio/%d", groupid);
	dev->vfio_gfd = open(path, O_RDWR);
	if(dev->vfio_gfd < 0){
		warn("Failed to open %s", path);
		return -1;
	}

	struct vfio_group_status group_status;
	ret = ioctl(dev->vfio_gfd, VFIO_GROUP_GET_STATUS, &group_status);
	if(ret != 0 || !group_status.flags & VFIO_GROUP_FLAGS_VIABLE){
		warn("VFIO group is not visible");
		return -1;
	}

	// set container
	ret = ioctl(dev->vfio_gfd, VFIO_GROUP_SET_CONTAINER, &dev->vfio_cfd);
	if(ret != 0){
		warn("Failed to set container");
		return -1;
	}
	// set vfio type (type1 is for IOMMU like VT-d or AMD-Vi)
	ret = ioctl(dev->vfio_cfd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
	if(ret != 0){
		warn("Failed to set iommu type");
		return -1;
	}

	// get device descriptor
	dev->vfio_fd = ioctl(dev->vfio_gfd, VFIO_GROUP_GET_DEVICE_FD, dev->pci_addr);
	if(dev->vfio_fd < 0){
		warn("Cannot get device fd");
		return -1;
	}
	return 0;
}

int vfio_map_dma(struct ixy_device* dev, uint64_t vaddr, uint64_t iova, uint32_t size){
	struct vfio_iommu_type1_dma_map dma_map = {
		.vaddr = vaddr,
		.iova = iova,
		.size = size,
		.argsz = sizeof(dma_map),
		.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE};
	return ioctl(dev->vfio_cfd, VFIO_IOMMU_MAP_DMA, &dma_map);
}

#endif

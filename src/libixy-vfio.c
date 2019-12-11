#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <linux/limits.h>
#include <linux/vfio.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>

#include <driver/device.h>

#define IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int))
#define MAX_INTERRUPT_VECTORS 32
#define MSIX_IRQ_SET_BUF_LEN (sizeof(struct vfio_irq_set) + sizeof(int) * (MAX_INTERRUPT_VECTORS + 1))

ssize_t MIN_DMA_MEMORY = 4096; // we can not allocate less than page_size memory

void vfio_enable_dma(int device_fd) {
	// write to the command register (offset 4) in the PCIe config space
	int command_register_offset = 4;
	// bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
	int bus_master_enable_bit = 2;
	// Get region info for config region
	struct vfio_region_info conf_reg = {.argsz = sizeof(conf_reg)};
	conf_reg.index = VFIO_PCI_CONFIG_REGION_INDEX;
	check_err(ioctl(device_fd, VFIO_DEVICE_GET_REGION_INFO, &conf_reg), "get vfio config region info");
	uint16_t dma = 0;
	assert(pread(device_fd, &dma, 2, conf_reg.offset + command_register_offset) == 2);
	dma |= 1 << bus_master_enable_bit;
	assert(pwrite(device_fd, &dma, 2, conf_reg.offset + command_register_offset) == 2);
}

/**
 * Enable VFIO MSI interrupts.
 * @param device_fd The VFIO file descriptor.
 * @return The event file descriptor.
 */
int vfio_enable_msi(int device_fd) {
	info("Enable MSI Interrupts");
	char irq_set_buf[IRQ_SET_BUF_LEN];
	int* fd_ptr;

	// setup event fd
	int event_fd = eventfd(0, 0);

	struct vfio_irq_set* irq_set = (struct vfio_irq_set*) irq_set_buf;
	irq_set->argsz = sizeof(irq_set_buf);
	irq_set->count = 1;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSI_IRQ_INDEX;
	irq_set->start = 0;
	fd_ptr = (int*) &irq_set->data;
	*fd_ptr = event_fd;

	check_err(ioctl(device_fd, VFIO_DEVICE_SET_IRQS, irq_set), "enable MSI interrupts");

	return event_fd;
}

/**
 * Disable VFIO MSI interrupts.
 * @param device_fd The VFIO file descriptor.
 * @return 0 on success.
 */
int vfio_disable_msi(int device_fd) {
	info("Disable MSI Interrupts");
	char irq_set_buf[IRQ_SET_BUF_LEN];

	struct vfio_irq_set* irq_set = (struct vfio_irq_set*) irq_set_buf;
	irq_set->argsz = sizeof(irq_set_buf);
	irq_set->count = 0;
	irq_set->flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSI_IRQ_INDEX;
	irq_set->start = 0;

	check_err(ioctl(device_fd, VFIO_DEVICE_SET_IRQS, irq_set), "disable MSI interrupts");

	return 0;
}

/**
 * Enable VFIO MSI-X interrupts.
 * @param device_fd The VFIO file descriptor.
 * @return The event file descriptor.
 */
int vfio_enable_msix(int device_fd, uint32_t interrupt_vector) {
	info("Enable MSIX Interrupts");
	char irq_set_buf[MSIX_IRQ_SET_BUF_LEN];
	struct vfio_irq_set* irq_set;
	int* fd_ptr;

	// setup event fd
	int event_fd = eventfd(0, 0);

	irq_set = (struct vfio_irq_set*) irq_set_buf;
	irq_set->argsz = sizeof(irq_set_buf);
	if (!interrupt_vector) {
		interrupt_vector = 1;
	} else if (interrupt_vector > MAX_INTERRUPT_VECTORS)
		interrupt_vector = MAX_INTERRUPT_VECTORS + 1;

	irq_set->count = interrupt_vector;
	irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSIX_IRQ_INDEX;
	irq_set->start = 0;
	fd_ptr = (int*) &irq_set->data;
	fd_ptr[0] = event_fd;

	check_err(ioctl(device_fd, VFIO_DEVICE_SET_IRQS, irq_set), "enable MSIX interrupt");

	return event_fd;
}

/**
 * Disable VFIO MSI-X interrupts.
 * @param device_fd The VFIO file descriptor.
 * @return 0 on success.
 */
int vfio_disable_msix(int device_fd) {
	info("Disable MSIX Interrupts");
	struct vfio_irq_set* irq_set;
	char irq_set_buf[MSIX_IRQ_SET_BUF_LEN];

	irq_set = (struct vfio_irq_set*) irq_set_buf;
	irq_set->argsz = sizeof(struct vfio_irq_set);
	irq_set->count = 0;
	irq_set->flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER;
	irq_set->index = VFIO_PCI_MSIX_IRQ_INDEX;
	irq_set->start = 0;

	check_err(ioctl(device_fd, VFIO_DEVICE_SET_IRQS, irq_set), "disable MSIX interrupt");

	return 0;
}

/**
 * Setup VFIO interrupts by detecting which interrupts this device supports.
 * @param device_fd The VFIO file descriptor.
 * @return The supported interrupt.
 */
int vfio_setup_interrupt(int device_fd) {
	info("Setup VFIO Interrupts");

	for (int i = VFIO_PCI_MSIX_IRQ_INDEX; i >= 0; i--) {
		struct vfio_irq_info irq = {.argsz = sizeof(irq), .index = i};

		check_err(ioctl(device_fd, VFIO_DEVICE_GET_IRQ_INFO, &irq), "get IRQ Info");

		/* if this vector cannot be used with eventfd continue with next*/
		if ((irq.flags & VFIO_IRQ_INFO_EVENTFD) == 0) {
			debug("IRQ doesn't support Event FD");
			continue;
		}

		return i;
	}

	return -1;
}

/**
 * Waits for events on the epoll instance referred to by the file descriptor epoll_fd.
 * The memory area pointed to by events will contain the events that will be available for the caller.
 * Up to maxevents are returned by epoll_wait.
 * @param epoll_fd The epoll file descriptor.
 * @param maxevents The maximum number of events to return. The maxevents argument must be greater than zero.
 * @param timeout The timeout argument specifies the minimum number of milliseconds that epoll_wait will block.
 * Specifying a timeout of -1 causes epoll_wait to block indefinitely,
 * while specifying a timeout equal to zero cause epoll_wait to return immediately, even if no events are available.
 * @return Number of ready file descriptors.
 */
int vfio_epoll_wait(int epoll_fd, int maxevents, int timeout) {
	struct epoll_event events[maxevents];
	int rc;

	while (1) {
		// Waiting for packets
		rc = (int) check_err(epoll_wait(epoll_fd, events, maxevents, timeout), "to handle epoll wait");
		if (rc > 0) {
			/* epoll_wait has at least one fd ready to read */
			for (int i = 0; i < rc; i++) {
				uint64_t val;
				// read event file descriptor to clear interrupt.
				check_err(read(events[i].data.fd, &val, sizeof(val)), "to read event");
			}
			break;
		} else {
			/* rc == 0, epoll_wait timed out */
			break;
		}
	}

	return rc;
}

/**
 * Add event file descriptor to epoll.
 * @param event_fd The event file descriptor to add.
 * @return The epoll file descriptor.
 */
int vfio_epoll_ctl(int event_fd) {
	struct epoll_event event;
	event.events = EPOLLIN;
	event.data.fd = event_fd;

	int epoll_fd = (int) check_err(epoll_create1(0), "to created epoll");

	check_err(epoll_ctl(epoll_fd, EPOLL_CTL_ADD, event_fd, &event), "to initialize epoll");

	return epoll_fd;
}

// returns the devices file descriptor or -1 on error
int vfio_init(const char* pci_addr) {
	// find iommu group for the device
	// `readlink /sys/bus/pci/device/<segn:busn:devn.funcn>/iommu_group`
	char path[PATH_MAX], iommu_group_path[PATH_MAX];
	struct stat st;
	snprintf(path, sizeof(path), "/sys/bus/pci/devices/%s/", pci_addr);
	int ret = stat(path, &st);
	if (ret < 0) {
		// No such device
		return -1;
	}
	strncat(path, "iommu_group", sizeof(path) - strlen(path) - 1);

	int len = check_err(readlink(path, iommu_group_path, sizeof(iommu_group_path)), "find the iommu_group for the device");

	iommu_group_path[len] = '\0'; // append 0x00 to the string to end it
	char* group_name = basename(iommu_group_path);
	int groupid;
	check_err(sscanf(group_name, "%d", &groupid), "convert group id to int");

	int firstsetup = 0; // Need to set up the container exactly once
	int cfd = get_vfio_container();
	if (cfd == -1) {
		firstsetup = 1;
		// open vfio file to create new vfio container
		cfd = check_err(open("/dev/vfio/vfio", O_RDWR), "open /dev/vfio/vfio");
		set_vfio_container(cfd);

		// check if the container's API version is the same as the VFIO API's
		check_err((ioctl(cfd, VFIO_GET_API_VERSION) == VFIO_API_VERSION) - 1, "get a valid API version from the container");

		// check if type1 is supported
		check_err((ioctl(cfd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) == 1) - 1, "get Type1 IOMMU support from the IOMMU container");
	}

	// open VFIO group containing the device
	snprintf(path, sizeof(path), "/dev/vfio/%d", groupid);
	int vfio_gfd = check_err(open(path, O_RDWR), "open vfio group");

	// check if group is viable
	struct vfio_group_status group_status = {.argsz = sizeof(group_status)};
	check_err(ioctl(vfio_gfd, VFIO_GROUP_GET_STATUS, &group_status), "get VFIO group status");
	check_err(((group_status.flags & VFIO_GROUP_FLAGS_VIABLE) > 0) - 1, "get viable VFIO group - are all devices in the group bound to the VFIO driver?");

	// Add group to container
	check_err(ioctl(vfio_gfd, VFIO_GROUP_SET_CONTAINER, &cfd), "set container");

	if (firstsetup != 0) {
		// Set vfio type (type1 is for IOMMU like VT-d or AMD-Vi) for the
		// container.
		// This can only be done after at least one group is in the container.
		ret = check_err(ioctl(cfd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU), "set IOMMU type");
	}

	// get device file descriptor
	int vfio_fd = check_err(ioctl(vfio_gfd, VFIO_GROUP_GET_DEVICE_FD, pci_addr), "get device fd");

	// enable DMA
	vfio_enable_dma(vfio_fd);

	return vfio_fd;
}

// returns a uint8_t pointer to the MMAPED region or MAP_FAILED if failed
uint8_t* vfio_map_region(int vfio_fd, int region_index) {
	struct vfio_region_info region_info = {.argsz = sizeof(region_info)};
	region_info.index = region_index;
	int ret = ioctl(vfio_fd, VFIO_DEVICE_GET_REGION_INFO, &region_info);
	if (ret == -1) {
		// Failed to set iommu type
		return MAP_FAILED; // MAP_FAILED == ((void *) -1)
	}
	return (uint8_t*) check_err(mmap(NULL, region_info.size, PROT_READ | PROT_WRITE, MAP_SHARED, vfio_fd, region_info.offset), "mmap vfio bar0 resource");
}

// returns iova (physical address of the DMA memory from device view) on success
uint64_t vfio_map_dma(void* vaddr, uint32_t size) {
	uint64_t iova = (uint64_t) vaddr; // map iova to process virtual address
	struct vfio_iommu_type1_dma_map dma_map = {
		.vaddr = (uint64_t) vaddr,
		.iova = iova,
		.size = size < MIN_DMA_MEMORY ? MIN_DMA_MEMORY : size,
		.argsz = sizeof(dma_map),
		.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE};
	int cfd = get_vfio_container();
	check_err(ioctl(cfd, VFIO_IOMMU_MAP_DMA, &dma_map), "IOMMU Map DMA Memory");
	return iova;
}

// unmaps previously mapped DMA region. returns 0 on success
uint64_t vfio_unmap_dma(int fd, uint64_t iova, uint32_t size) {
	struct vfio_iommu_type1_dma_unmap dma_unmap = {
		.argsz = sizeof(dma_unmap),
		.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
		.iova = iova,
		.size = size
	};
	int cfd = get_vfio_container();
	int ret = ioctl(cfd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap);
	if (ret == -1) {
		// Failed to unmap DMA region
		return -1;
	}
	return ret;
}

#include <assert.h>
#include <errno.h>
#include <linux/limits.h>
#include <stdio.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "pci.h"
#include "log.h"


void remove_driver(const char* pci_addr) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/driver/unbind", pci_addr);
	int fd = open(path, O_WRONLY);
	if (fd == -1) {
		debug("no driver loaded");
		return;
	}
	if (write(fd, pci_addr, strlen(pci_addr)) != (ssize_t) strlen(pci_addr)) {
		warn("failed to unload driver for device %s", pci_addr);
	}
	check_err(close(fd), "close");
}

void enable_dma(const char* pci_addr) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/config", pci_addr);
	int fd = check_err(open(path, O_RDWR), "open pci config");
	// write to the command register (offset 4) in the PCIe config space
	// bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
	assert(lseek(fd, 4, SEEK_SET) == 4);
	uint16_t dma = 0;
	assert(read(fd, &dma, 2) == 2);
	dma |= 1 << 2;
	assert(lseek(fd, 4, SEEK_SET) == 4);
	assert(write(fd, &dma, 2) == 2);
	check_err(close(fd), "close");
}

uint8_t* pci_map_resource(const char* pci_addr) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/resource0", pci_addr);
	debug("Mapping PCI resource at %s", path);
	remove_driver(pci_addr);
	enable_dma(pci_addr);
	int fd = check_err(open(path, O_RDWR), "open pci resource");
	struct stat stat;
	check_err(fstat(fd, &stat), "stat pci resource");
	return (uint8_t*) check_err(mmap(NULL, stat.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0), "mmap pci resource");
}

int pci_open_resource(const char* pci_addr, const char* resource) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/%s", pci_addr, resource);
	debug("Opening PCI resource at %s", path);
	int fd = check_err(open(path, O_RDWR), "open pci resource");
	return fd;
}

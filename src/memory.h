#ifndef IXY_MEMORY_H
#define IXY_MEMORY_H

#include <stdint.h>
#include <unistd.h>

struct pkt_buf {
	// physical address to pass a buffer to a nic
	uintptr_t buf_addr_phy;
	struct mempool* mempool;
	uint32_t mempool_idx;
	uint32_t size;
	uint8_t data[] __attribute__((aligned(64)));
};

struct mempool {
	void* base_addr;
	uintptr_t base_addr_phy;
	uint32_t buf_size;
	uint32_t num_entries;
	// memory is managed via a simple stack
	// replacing this with a lock-free queue (or stack) makes this thread-safe
	uint32_t free_stack_top;
	uint32_t free_stack[];
};

struct dma_memory {
	void* virt;
	uintptr_t phy;
};

struct dma_memory memory_allocate_dma(size_t size);

struct mempool* memory_allocate_mempool(uint32_t num_entries, uint32_t entry_size);
struct pkt_buf* pkt_buf_alloc(struct mempool* mempool);
void pkt_buf_free(struct pkt_buf* buf);

#endif //IXY_MEMORY_H

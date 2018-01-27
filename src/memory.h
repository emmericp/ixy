#ifndef IXY_MEMORY_H
#define IXY_MEMORY_H

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stddef.h>
#include <assert.h>

#define HUGE_PAGE_BITS 21
#define HUGE_PAGE_SIZE (1 << HUGE_PAGE_BITS)
#define SIZE_PKT_BUF_HEADROOM 40

struct pkt_buf {
	// physical address to pass a buffer to a nic
	uintptr_t buf_addr_phy;
	struct mempool* mempool;
	uint32_t mempool_idx;
	uint32_t size;
	uint8_t head_room[SIZE_PKT_BUF_HEADROOM];
	uint8_t data[] __attribute__((aligned(64)));
};

static_assert(sizeof(struct pkt_buf) == 64, "pkt_buf too large");
static_assert(offsetof(struct pkt_buf, data) == 64, "data at unexpected position");
static_assert(offsetof(struct pkt_buf, head_room) + SIZE_PKT_BUF_HEADROOM == offsetof(struct pkt_buf, data), "head room not immediately before data");

// everything here contains virtual addresses, the mapping to physical addresses are in the pkt_buf
struct mempool {
	void* base_addr;
	uint32_t buf_size;
	uint32_t num_entries;
	// memory is managed via a simple stack
	// replacing this with a lock-free queue (or stack) makes this thread-safe
	uint32_t free_stack_top;
	// the stack contains the entry id, i.e., base_addr + entry_id * buf_size is the address of the buf
	uint32_t free_stack[];
};

struct dma_memory {
	void* virt;
	uintptr_t phy;
};

struct dma_memory memory_allocate_dma(size_t size, bool require_contiguous);

struct mempool* memory_allocate_mempool(uint32_t num_entries, uint32_t entry_size);
uint32_t pkt_buf_alloc_batch(struct mempool* mempool, struct pkt_buf* bufs[], uint32_t num_bufs);
struct pkt_buf* pkt_buf_alloc(struct mempool* mempool);
void pkt_buf_free(struct pkt_buf* buf);

#endif //IXY_MEMORY_H

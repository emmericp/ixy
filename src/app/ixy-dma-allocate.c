#include "memory.h"

int main() {
    memory_allocate_dma(HUGE_PAGE_SIZE * 1, true);
    memory_allocate_dma(HUGE_PAGE_SIZE * 2, true);
    memory_allocate_dma(HUGE_PAGE_SIZE * 4, true);
    memory_allocate_dma(HUGE_PAGE_SIZE * 8, true);
    memory_allocate_dma(HUGE_PAGE_SIZE * 16, true);
}

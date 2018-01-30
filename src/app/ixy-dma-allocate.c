#include <stdbool.h>

#include "memory.h"

int main() {
    memory_allocate_dma(4096, true);
    memory_allocate_dma(4096 * 4, true);
    memory_allocate_dma(4096 * 16, true);
    memory_allocate_dma(4096 * 64, true);
}

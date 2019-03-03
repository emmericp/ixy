#include <stdbool.h>

#include "memory.h"

int main() {
    memory_allocate_dma(4096, REQUIRE_CONTIGUOUS);
    memory_allocate_dma(4096 * 4, REQUIRE_CONTIGUOUS);
    memory_allocate_dma(4096 * 16, REQUIRE_CONTIGUOUS);
    memory_allocate_dma(4096 * 64, REQUIRE_CONTIGUOUS);
    memory_allocate_dma(4096 * 128, REQUIRE_CONTIGUOUS);
}

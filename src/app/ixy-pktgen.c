#include <stdio.h>
#include <unistd.h>

#include "stats.h"
#include "log.h"
#include "memory.h"
#include "driver/ixgbe.h"

// excluding CRC (offloaded by default)
static const int PKT_SIZE = 60;

// number of packets sent simultaneously to our driver
static const uint32_t BATCH_SIZE = 64;

static struct mempool* init_mempool() {
	const int NUM_BUFS = 2048;
	struct mempool* mempool = memory_allocate_mempool(NUM_BUFS, 0);

	// pre-fill all our packet buffers with some templates that can be modified later
	// we have to do it like this because sending is async in the hardware; we cannot re-use a buffer immediately
	struct pkt_buf* bufs[NUM_BUFS];
	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		struct pkt_buf* buf = pkt_buf_alloc(mempool);
		buf->size = PKT_SIZE;
		// TODO: initialize packet with something else here
		for (int i = 0; i < PKT_SIZE; i++) {
			buf->data[i] = 0xFF;
		}
		bufs[buf_id] = buf;
	}
	// return them all to the mempool, all future allocations will return bufs with the data set above
	for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
		pkt_buf_free(bufs[buf_id]);
	}

	return mempool;
}

int main(int argc, char* argv[]) {
	if (argc != 2) {
		printf("Usage: %s <pci bus id>\n", argv[0]);
		return 1;
	}

	struct mempool* mempool = init_mempool();
	struct ixy_device* dev = ixgbe_init(argv[1], 1, 1);

	uint64_t last_stats_printed = monotonic_time();
	uint64_t counter = 0;
	struct device_stats stats_old, stats;
	stats_init(&stats, dev);
	stats_init(&stats_old, dev);

	// array of bufs sent out in a batch
	struct pkt_buf* bufs[BATCH_SIZE];

	// tx loop
	while (true) {
		// we cannot immediately recycle packets, we need to allocate new packets every time
		// the old packets might still be used by the NIC: tx is async
		pkt_buf_alloc_batch(mempool, bufs, BATCH_SIZE);
		// the packets could be modified here to generate multiple flows
		ixgbe_tx_batch_busy_wait(dev, 0, bufs, BATCH_SIZE);

		// don't check time for every packet, this yields +10% performance :)
		if ((counter++ & 0xFFF) == 0) {
			uint64_t time = monotonic_time();
			if (time - last_stats_printed > 1000 * 1000 * 1000) {
				// every second
				ixgbe_read_stats(dev, &stats);
				print_stats_diff(&stats, &stats_old, time - last_stats_printed);
				stats_old = stats;
				last_stats_printed = time;
			}
		}
		// track stats
	}
	return 0;
}


#include <stdio.h>
#include <unistd.h>

#include "stats.h"
#include "log.h"
#include "memory.h"
#include "driver/device.h"

const int BATCH_SIZE = 32;

static void forward(struct ixy_device* rx_dev, uint16_t rx_queue, struct ixy_device* tx_dev, uint16_t tx_queue) {
	struct pkt_buf* bufs[BATCH_SIZE];
	uint32_t num_rx = ixy_rx_batch(rx_dev, rx_queue, bufs, BATCH_SIZE);
	if (num_rx > 0) {
		// touch all packets, otherwise it's a completely unrealistic workload if the packet just stays in L3
		for (uint32_t i = 0; i < num_rx; i++) {
			bufs[i]->data[1]++;
		}
		uint32_t num_tx = ixy_tx_batch(tx_dev, tx_queue, bufs, num_rx);
		// there are two ways to handle the case that packets are not being sent out:
		// either wait on tx or drop them; in this case it's better to drop them, otherwise we accumulate latency
		for (uint32_t i = num_tx; i < num_rx; i++) {
			pkt_buf_free(bufs[i]);
		}
	}
}

int main(int argc, char* argv[]) {
	if (argc != 3) {
		printf("%s forwards packets between two ports.\n", argv[0]);
		printf("Usage: %s <pci bus id2> <pci bus id1>\n", argv[0]);
		return 1;
	}

	struct ixy_device* dev1 = ixy_init(argv[1], 1, 1);
	struct ixy_device* dev2 = ixy_init(argv[2], 1, 1);

	uint64_t last_stats_printed = monotonic_time();
	struct device_stats stats1, stats1_old;
	struct device_stats stats2, stats2_old;
	stats_init(&stats1, dev1);
	stats_init(&stats1_old, dev1);
	stats_init(&stats2, dev2);
	stats_init(&stats2_old, dev2);

	uint64_t counter = 0;
	while (true) {
		forward(dev1, 0, dev2, 0);
		forward(dev2, 0, dev1, 0);

		// don't poll the time unnecessarily
		if ((counter++ & 0xFFF) == 0) {
			uint64_t time = monotonic_time();
			if (time - last_stats_printed > 1000 * 1000 * 1000) {
				// every second
				ixy_read_stats(dev1, &stats1);
				print_stats_diff(&stats1, &stats1_old, time - last_stats_printed);
				stats1_old = stats1;
				if (dev1 != dev2) {
					ixy_read_stats(dev2, &stats2);
					print_stats_diff(&stats2, &stats2_old, time - last_stats_printed);
					stats2_old = stats2;
				}
				last_stats_printed = time;
			}
		}
	}
}


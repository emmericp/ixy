#include <stdio.h>
#include <unistd.h>

#include "stats.h"
#include "log.h"
#include "memory.h"
#include "driver/ixgbe.h"


int main(int argc, char* argv[]) {
	if (argc != 3) {
		printf("%s forwards packets between two ports.\n", argv[0]);
		printf("Usage: %s <pci bus id2> <pci bus id1>\n", argv[0]);
		return 1;
	}

	struct ixy_device* dev1 = ixgbe_init(argv[1], 1, 1);
	struct ixy_device* dev2;
	if (strcmp(argv[1], argv[2])) {
		dev2 = ixgbe_init(argv[2], 1, 1);
	} else {
		// same device, cannot be initialized twice
		// this effectively turns this into an echo server
		dev2 = dev1;
	}

	uint64_t last_stats_printed = monotonic_time();
	struct device_stats stats1, stats1_old;
	struct device_stats stats2, stats2_old;
	stats_init(&stats1, dev1);
	stats_init(&stats1_old, dev1);
	stats_init(&stats2, dev2);
	stats_init(&stats2_old, dev2);

	uint64_t counter = 0;

	while (true) {
		struct pkt_buf* buf = ixgbe_rx_packet(dev1, 0);
		if (buf) {
			// transmit function takes care of freeing the packet
			while (!ixgbe_tx_batch(dev2, 0, &buf, 1)) {
				// busy wait until we can send out that packet
			}
		}

		// don't poll the time unnecessarily
		if ((counter++ & 0xFFF) == 0) {
			uint64_t time = monotonic_time();
			if (time - last_stats_printed > 1000 * 1000 * 1000) {
				// every second
				ixgbe_read_stats(dev1, &stats1);
				print_stats_diff(&stats1, &stats1_old, time - last_stats_printed);
				stats1_old = stats1;
				if (dev1 != dev2) {
					ixgbe_read_stats(dev2, &stats2);
					print_stats_diff(&stats2, &stats2_old, time - last_stats_printed);
					stats2_old = stats2;
				}
				last_stats_printed = time;
			}
		}
	}
}


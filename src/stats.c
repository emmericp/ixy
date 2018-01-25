#include "stats.h"

#include <stdio.h>

void print_stats(struct device_stats* stats) {
	printf("[%s] RX: %zu bytes %zu packets\n", stats->device ? stats->device->pci_addr : "???", stats->rx_bytes, stats->rx_pkts);
	printf("[%s] TX: %zu bytes %zu packets\n", stats->device ? stats->device->pci_addr : "???", stats->tx_bytes, stats->tx_pkts);
}

static double diff_mpps(uint64_t pkts_new, uint64_t pkts_old, uint64_t nanos) {
	return (double) (pkts_new - pkts_old) / 1000000.0 / ((double) nanos / 1000000000.0);
}

static uint32_t diff_mbit(uint64_t bytes_new, uint64_t bytes_old, uint64_t pkts_new, uint64_t pkts_old, uint64_t nanos) {
	// take stuff on the wire into account, i.e., the preamble, SFD and IFG (20 bytes)
	// otherwise it won't show up as 10000 mbit/s with small packets which is confusing
	return (uint32_t) (((bytes_new - bytes_old) / 1000000.0 / ((double) nanos / 1000000000.0)) * 8
		+ diff_mpps(pkts_new, pkts_old, nanos) * 20 * 8);
}

void print_stats_diff(struct device_stats* stats_new, struct device_stats* stats_old, uint64_t nanos) {
	printf("[%s] RX: %d Mbit/s %.2f Mpps\n", stats_new->device ? stats_new->device->pci_addr : "???",
		diff_mbit(stats_new->rx_bytes, stats_old->rx_bytes, stats_new->rx_pkts, stats_old->rx_pkts, nanos),
		diff_mpps(stats_new->rx_pkts, stats_old->rx_pkts, nanos)
	);
	printf("[%s] TX: %d Mbit/s %.2f Mpps\n", stats_new->device ? stats_new->device->pci_addr : "???",
		diff_mbit(stats_new->tx_bytes, stats_old->tx_bytes, stats_new->tx_pkts, stats_old->tx_pkts, nanos),
		diff_mpps(stats_new->tx_pkts, stats_old->tx_pkts, nanos)
	);
}


// returns a timestamp in nanoseconds
// based on rdtsc on reasonably configured systems and is hence fast
uint64_t monotonic_time() {
	struct timespec timespec;
	clock_gettime(CLOCK_MONOTONIC, &timespec);
	return timespec.tv_sec * 1000 * 1000 * 1000 + timespec.tv_nsec;
}

// initializes a stat struct and clears the stats on the device
void stats_init(struct device_stats* stats, struct ixy_device* dev) {
	// might require device-specific initialization
	stats->rx_pkts = 0;
	stats->tx_pkts = 0;
	stats->rx_bytes = 0;
	stats->tx_bytes = 0;
	stats->device = dev;
	if (dev) {
		ixy_read_stats(dev, NULL);
	}
}

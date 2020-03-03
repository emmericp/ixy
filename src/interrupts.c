#include "interrupts.h"
#include "libixy-vfio.h"
#include "log.h"
#include "stats.h"

#include <stdio.h>

/**
 * Calculate packets per millisecond based on the received number of packets and the elapsed time in nanoseconds since the
 * last calculation.
 * @param received_pkts Number of received packets.
 * @param elapsed_time_nanos Time elapsed in nanoseconds since the last calculation.
 * @return Packets per millisecond.
 */
static uint64_t ppms(uint64_t received_pkts, uint64_t elapsed_time_nanos) {
	return received_pkts / (elapsed_time_nanos / 1000000);
}

/**
 * Check if interrupts or polling should be used based on the current number of received packets per seconds.
 * @param interrupts The interrupt handler.
 * @param diff The difference since the last call in nanoseconds.
 * @param buf_index The current buffer index.
 * @param buf_size The maximum buffer size.
 * @return Whether to disable NIC interrupts or not.
 */
void check_interrupt(struct interrupt_queues* interrupt, uint64_t diff, uint32_t buf_index, uint32_t buf_size) {
	struct interrupt_moving_avg* avg = &interrupt->moving_avg;
	avg->sum -= avg->measured_rates[avg->index];
	avg->measured_rates[avg->index] = ppms(interrupt->rx_pkts, diff);
	avg->sum += avg->measured_rates[avg->index];
	if (avg->length < MOVING_AVERAGE_RANGE) {
		avg->length++;
	}
	avg->index = (avg->index + 1) % MOVING_AVERAGE_RANGE;
	interrupt->rx_pkts = 0;
	uint64_t average = avg->sum / avg->length;
	if (average > INTERRUPT_THRESHOLD) {
		interrupt->interrupt_enabled = false;
	} else if (buf_index == buf_size) {
		interrupt->interrupt_enabled = false;
	} else {
		interrupt->interrupt_enabled = true;
	}
	interrupt->last_time_checked = monotonic_time();
}

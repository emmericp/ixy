#ifndef IXY_INTERRUPTS_H
#define IXY_INTERRUPTS_H

#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <stdbool.h>

#define MOVING_AVERAGE_RANGE 5
#define INTERRUPT_THRESHOLD 1200

struct interrupt_moving_avg {
	uint32_t index; // The current index
	uint32_t length; // The moving average length
	uint64_t sum; // The moving average sum
	uint64_t measured_rates[MOVING_AVERAGE_RANGE]; // The moving average window
};

struct interrupt_queues {
	int vfio_event_fd; // event fd
	int vfio_epoll_fd; // epoll fd
	bool interrupt_enabled; // Whether interrupt for this queue is enabled or not
	uint64_t last_time_checked; // Last time the interrupt flag was checked
	uint64_t instr_counter; // Instruction counter to avoid unnecessary calls to monotonic_time
	uint64_t rx_pkts; // The number of received packets since the last check
	uint64_t interval; // The interval to check the interrupt flag
	struct interrupt_moving_avg moving_avg; // The moving average of the hybrid interrupt
};

struct interrupts {
	bool interrupts_enabled; // Whether interrupts for this device are enabled or disabled.
	uint32_t itr_rate; // The Interrupt Throttling Rate
	struct interrupt_queues* queues; // Interrupt settings per queue
	uint8_t interrupt_type; // MSI or MSIX
	int timeout_ms; // interrupt timeout in milliseconds (-1 to disable the timeout)
};

void check_interrupt(struct interrupt_queues* interrupt, uint64_t diff, uint32_t buf_index, uint32_t buf_size);

#endif //IXY_INTERRUPTS_H

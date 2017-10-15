# ixy - a userspace network driver in 1000 lines of code

ixy is a simple userspace packet processing framework.
It takes exclusive control of a network device and implements the *whole driver* in userspace.
Its architecture is similar to [DPDK](http://dpdk.org/) and [Snabb](http://snabb.co) and completely different from (seemingly similar) frameworks such as netmap, pfq, pf_ring, or XDP (all of which rely on kernel components).

In fact, reading both DPDK and Snabb drivers was crucial to understand some parts of the [Intel 82599 datasheet](https://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82599-10-gbe-controller-datasheet.pdf) better.


ixy is designed for educational purposes to learn how a network card works at the driver level.
Lack of kernel code and a allows you to look through the whole code from startup to the lowest level of the driver.
Low-level functions like handling DMA descriptors are rarely more than a single function call away from your application logic.

A whole ixy app, including the whole driver, is only ~1000 lines of C code.
Check out the `ixy-fwd` and `ixy-pktgen` example apps and look through the code.
The code often references sections in the [Intel 82599 datasheet](https://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82599-10-gbe-controller-datasheet.pdf), so keep it open while reading the code.
You will be surprised how simple a full driver for a network card can be.



# Features
* Driver for Intel NICs in the `ixgbe` family (e.g., Intel X520/82599)
* Less than 1000 lines of C code for the full framework *including the driver*
* No kernel modules needed
* Simple API with memory management, similar to DPDK, easier to use than APIs based on a ring interface (e.g., netmap)
* Support for multiple device queues and multiple threads
* Super fast, easily achieves 10 Mpps (million packets per second) per CPU core. Performance will be improved further once we have a batched API (see wish list).
* Super simple to use: no dependencies, no annoying drivers to load, bind, or manage - see step-by-step tutorial below
* BSD license

# Supported hardware
Only tested on an Intel 82599ES (Intel X520-T2) at the moment.
Support/testing on X540, X550, and X552/X557 (Xeon D embedded NIC) is planned.

# How does it work?
TODO. For now: read the code, it has comments referring to relevant sections of the [Intel 82599 datasheet](https://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82599-10-gbe-controller-datasheet.pdf).

# Compiling ixy and running the examples

### Caution
**Running ixy will unbind the driver of the given PCIe device without checking if it is in use**. This means the NIC will disappear from the system. Do not run this on NICs that you need.

1. Install the following dependencies
	* gcc >= 4.8
	* make
	* cmake
	
	Run this Debian/Ubuntu to install them:
	
	```
	sudo apt-get install -y build-essential cmake
	```
2. Configure 2MB hugepages in /mnt/huge using our script:

	```
	cd ixy
	sudo ./setup-hugetlbfs.sh
	```
	
3. Run cmake and make

	```
	cmake .
	make
	```
4. That's it! You can now run the included examples, e.g.:

	```
	sudo ./ixy-pktgen 0000:03:00.0
	```
	
	Replace the PCI address as needed. All examples expect fully qualified PCIe bus addresses, i.e., typically prefixed with `0000:`, as arguments.
	You can use `lspci` from the `pciutils` (Debian/Ubuntu) package to find the bus address.
	For example, `lspci` shows my 82599ES NIC as
	
	`03:00.0 Ethernet controller: Intel Corporation 82599ES 10-Gigabit SFI/SFP+ Network Connection (rev 01)`
	
	which means that I have to pass `0000:03:00.0` as parameter to use it.

# Wish list
It's not the plan to implement every single feature, but a few more things would be nice to have.
The list is in no particular order.

### Implement at least one other driver beside ixgbe

To showcase how to make the framework more independent from the used hardware.
Various 1 Gbit/s NICs are good candidates.

NICs that rely too much on firmware (e.g., Intel XL710) are not fun, because you end up only talking to a firmware that does everything.
The same is true for NICs like the ones by Mellanox that keep a lot of magic in kernel modules, even when being used by frameworks like DPDK.

### Batched RX/TX APIs

TX performance is currently bottlenecked at around 10 Mpps because we need to access the PCIe address space for every packet.
This is a hardware bottleneck in the NICs, multiple threads/queues don't help.
DPDK hits the same bottleneck when using a batch size of 1.

A batch API adds some complexity to the rx/tx functions.
It should probably be based on explicit batching by the user and look similar to the DPDK API.
Implicit batching requires regular callbacks to make sure that no packets get stuck in it.

### NUMA support
DMA memory should be pinned to the correct NUMA node.
Threads handling packet reception should also be pinned to the same NUMA node.
Less important for transmission.

### RSS support
What's the point of having multiple rx queues if there is no good way to distribute the traffic to them?

Shouldn't be too hard.

### `tcpdump`-like example
A simple rx-only app that writes packets to a `.pcap` file based on `mmap` and `fallocate`.
Most of the code can be re-used from [libmoon's pcap.lua](https://github.com/libmoon/libmoon/blob/master/lua/pcap.lua).

### Multi-threaded mempools
A mempool can currently only be accessed by a single thread, i.e., a packet must be allocated and free'd by the same thread.
This prevents apps that implement different processing steps on different cores, a common multi-threading model for complex chains.

The current limitation is the list of free buffers in the mempool, it's a simple stack at the moment.
Replacing this with a lock-free stack or queue makes it safe for use with multiple threads.

The good news is that multi-threaded mempools are essentially the same problem as passing packets between threads: both can be implemented with the same queue/ring buffer data structure.


# FAQ

## Why C and not a more reasonable language?
It's the lowest common denominator that everyone should be able to understand -- this is for educational purposes only.
I've taken care to keep the code simple and understandable.

## It's more than ~1000 lines! There is a huge `ixgbe_type.h` file.
`ixgbe_type.h` is copied from the Intel driver, it's only used as a machine-readable version of the datasheet.
ixy only uses `#define` definitions for registers and the two relatively simple structs for the DMA descriptors.
Overall, ixy uses less than 100 lines of the file and we could remove the remainder.

But it's nice to have all the struct definitions right there when implementing a new driver feature. Copy & pasting magic values from the datasheet is significantly less fun.
Another interesting approach to making these values available is writing a parser for the tables in the datasheet.
[Snabb does this.](https://github.com/snabbco/snabb/blob/master/src/lib/hardware/register.lua)

## Should I use this for my app?
No.

## When should I use this?
To understand how a NIC works and to understand how a framework like [DPDK](http://dpdk.org/) or [Snabb](http://snabb.co) works.

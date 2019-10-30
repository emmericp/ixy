# ixy - a userspace network driver in 1000 lines of code

ixy is a simple userspace packet processing framework.
It takes exclusive control of a network adapter and implements the *whole driver* in userspace.
Its architecture is similar to [DPDK](http://dpdk.org/) and [Snabb](http://snabb.co) and completely different from (seemingly similar) frameworks such as netmap, pfq, pf_ring, or XDP (all of which rely on kernel components).
In fact, reading both DPDK and Snabb drivers was crucial to understand some parts of the [Intel 82599 datasheet](https://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82599-10-gbe-controller-datasheet.pdf) better.

Check out our research paper ["User Space Network Drivers"](https://www.net.in.tum.de/fileadmin/bibtex/publications/papers/ixy-writing-user-space-network-drivers.pdf) [[BibTeX](https://www.net.in.tum.de/publications/bibtex/ixy-user-space-drivers.bib)] or [watch the recording of our talk at 34C3](https://media.ccc.de/v/34c3-9159-demystifying_network_cards) to learn more.

ixy is designed for educational purposes to learn how a network card works at the driver level.
Lack of kernel code and external libraries allows you to look through the whole code from startup to the lowest level of the driver.
Low-level functions like handling DMA descriptors are rarely more than a single function call away from your application logic.

A whole ixy app, including the whole driver, is only ~1000 lines of C code.
Check out the `ixy-fwd` and `ixy-pktgen` example apps and look through the code.
The code often references sections in the [Intel 82599 datasheet](https://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82599-10-gbe-controller-datasheet.pdf) or the [VirtIO specification](http://docs.oasis-open.org/virtio/virtio/v1.0/virtio-v1.0.pdf), so keep them open while reading the code.
You will be surprised how simple a full driver for a network card can be.

Don't like C? We also have [implementations in other languages](https://github.com/ixy-languages/ixy-languages) (Rust, Go, C#, Java, OCaml, Haskell, Swift, JavaScript, and Python).


# Features
* Driver for Intel NICs in the `ixgbe` family, i.e., the 82599ES family (aka Intel X520)
* Driver for paravirtualized virtio NICs
* Less than 1000 lines of C code for a packet forwarder including the whole driver (w/o virtio and VFIO support, see [minimal branch](https://github.com/emmericp/ixy/tree/minimal-ixgbe-fwd))
* No kernel modules needed (except `vfio-pci` when using the IOMMU / VFIO)
* Can run without root privileges (when using the IOMMU / VFIO)
* IOMMU support (see Using the IOMMU / VFIO)
* Interrupt support (when using VFIO)
* Simple API with memory management, similar to DPDK, easier to use than APIs based on a ring interface (e.g., netmap)
* Support for multiple device queues and multiple threads
* Super fast, can forward > 25 million packets per second on a single 3.0 GHz CPU core
* Super simple to use (when not using VFIO): no dependencies, no annoying drivers to load, bind, or manage - see step-by-step tutorial below
* BSD license

# Supported hardware
Tested on an Intel 82599ES (aka Intel X520), X540, and X550. Might not work on all variants of these NICs because our link setup code is a little bit dodgy.

# How does it work?
Check out our research paper ["User Space Network Drivers"](https://www.net.in.tum.de/fileadmin/bibtex/publications/papers/ixy-writing-user-space-network-drivers.pdf) [[BibTeX](https://www.net.in.tum.de/publications/bibtex/ixy-user-space-drivers.bib)] for a detailed evaluation.

If you prefer to dive into the code: Start by reading the apps in [src/app](https://github.com/emmericp/ixy/tree/master/src/app) then follow the function calls into the driver. The comments in the code refer to the [Intel 82599 datasheet](https://www.intel.com/content/dam/www/public/us/en/documents/datasheets/82599-10-gbe-controller-datasheet.pdf) (Revision 3.3, March 2016).



# Compiling ixy and running the examples

### Caution
**Your NIC has full DMA access to your memory. A misconfigured NIC will cause memory corruptions that might crash your server or even destroy your filesystem**. Do not run this on any systems that have anything remotely important on them if you want to modify the driver. Our version is also not necessarily safe and might be buggy. You have been warned.

**Running ixy will unbind the driver of the given PCIe device without checking if it is in use.** This means the NIC will disappear from the system. Do not run this on NICs that you need.
We currently have a simple check if the device is actually a NIC, but trying to use another device could crash your system.

1. Install the following dependencies
	* gcc >= 4.8
	* make
	* cmake
	
	Run this on Debian/Ubuntu to install them:
	
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
	sudo ./ixy-pktgen 0000:XX:YY.Z
	```
	
	Replace the PCI address as needed. All examples expect fully qualified PCIe bus addresses, i.e., typically prefixed with `0000:`, as arguments.
	You can use `lspci` from the `pciutils` (Debian/Ubuntu) package to find the bus address.
	For example, `lspci` shows my 82599ES NIC as
	
	`03:00.0 Ethernet controller: Intel Corporation 82599ES 10-Gigabit SFI/SFP+ Network Connection (rev 01)`
	
	which means that I have to pass `0000:03:00.0` as parameter to use it.

## Using the IOMMU / VFIO
The usage of the IOMMU via the `vfio-pci` driver is implemented for ixgbe devices (Intel X520, X540, and X550).
Using VFIO will also enable interrupt support.
To use it, you have to:

0. Enable the IOMMU in the BIOS.
	On most Intel machines, the BIOS entry is called `VT-d` and has to be enabled in addition to any other virtualization technique.

1. Enable the IOMMU in the linux kernel.
	Add `intel_iommu=on` to your cmdline (if you are running a grub, the file `/etc/default/grub.cfg` contains a `GRUB_CMDLINE_LINUX` where you can add it).

2. Get the PCI address, vendor and device ID:
	`lspci -nn | grep Ether` returns something like `05:00.0 Ethernet controller [0200]: Intel Corporation Ethernet Controller 10-Gigabit X540-AT2 [8086:1528] (rev 01)`.
	In this case, `0000:05:00.0` is our PCI Address, and `8086` and `1528` are the vendor and device id, respectively.

3. Unbind the device from the `ixgbe` driver.
	`echo $PCI_ADDRESS > /sys/bus/pci/devices/$PCI_ADDRESS/driver/unbind`

4. Enable the `vfio-pci` driver.
	`modprobe vfio-pci`

5. Bind the device to the `vfio-pci` driver.
	`echo $VENDOR_ID $DEVICE_ID > /sys/bus/pci/drivers/vfio-pci/new_id`

6. Chown the device to the user.
	`chown $USER:$GROUP /dev/vfio/*`

6. That's it!
	Now you can compile and run ixy as stated above!

# Wish list
It's not the plan to implement every single feature, but a few more things would be nice to have.
The list is in no particular order.

### Implement at least one other driver beside ixgbe and VirtIO

NICs that rely too much on firmware (e.g., Intel XL710) are not fun, because you end up only talking to a firmware that does everything.
The same is true for NICs like the ones by Mellanox that keep a lot of magic in kernel modules, even when being used by frameworks like DPDK.

Interesting candidates would be NICs from the Intel igb and e1000e families as they quite common and reasonably cheap.

### Better NUMA support
PCIe devices are attached to a specific CPU in NUMA systems.
DMA memory should be pinned to the correct NUMA node.
Threads handling packet reception should also be pinned to the same NUMA node.

NUMA handling must currently be done via `numactl` outside of ixy. 
Implementing it within ixy is annoying without depending on `libnuma`, so it's not implemented here.

### RSS support
What's the point of having multiple rx queues if there is no good way to distribute the traffic to them?
Shouldn't be too hard. See Section 7.1.2.8 in the datasheet.

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

Check out our [implementations in other languages](https://github.com/ixy-languages/ixy-languages) (Rust, Go, C#, Java, OCaml, Haskell, Swift, JavaScript, and Python).


## I can't get line rate :(
There's a weird problem on some systems that causes it to slow down if the CPU is too fast. DPDK had the same problem in the past. Try applying bidirectional traffic to the forwarder and/or *underclock* your CPU to speed up ixy.

## It's more than ~1000 lines! There is a huge `ixgbe_type.h` file.
`ixgbe_type.h` is copied from the Intel driver, it's only used as a machine-readable version of the datasheet.
ixy only uses `#define` definitions for registers and the two relatively simple structs for the DMA descriptors.
Overall, ixy uses less than 100 lines of the file and we could remove the remainder.

But it's nice to have all the struct definitions right there when implementing a new driver feature. Copy & pasting magic values from the datasheet is significantly less fun.
Another interesting approach to making these values available is writing a parser for the tables in the datasheet.
[Snabb does this.](https://github.com/snabbco/snabb/blob/master/src/lib/hardware/register.lua)

## Should I use this for my production app?
No.

## When should I use this?
To understand how a NIC works and to understand how a framework like [DPDK](http://dpdk.org/) or [Snabb](http://snabb.co) works.

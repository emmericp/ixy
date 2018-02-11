Run ixy in VirtualBox with Vagrant
============================

Tested with Vagrant 2.0.2 and VirtualBox 5.2.6 on macOS and Debian Stretch.

We only tested VirtualBox, other providers might require changes to the network setup in the Vagrantfile.

Starting VMs
------------

```
vagrant up pktgen
vagrant up fwd
```

This will spawn two Debian VMs interconnected with two links with VirtIO NICs on each side.
Ixy is compiled in `/home/vagrant/ixy`.

**Caution:** VMs should only be stopped and started with with `vagrant halt <vmname>` and `vagrant up <vmname>`, otherwise the network setup might not work after restarts.

You can always get back to a clean state with `vagrant destroy -f <vmname>`. This destroys all data stored in the VM.

Connecting to VMs
-----------------

```
vagrant ssh pktgen
vagrant ssh fwd
```

Running Ixy
------------

The first step is to figure out the PCI bus addresses of our VirtIO NICs:

```
vagrant@stretch:~$ lspci |grep Ethernet
00:03.0 Ethernet controller: Intel Corporation 82540EM Gigabit Ethernet Controller (rev 02)
00:08.0 Ethernet controller: Red Hat, Inc Virtio network device
00:09.0 Ethernet controller: Red Hat, Inc Virtio network device
```

This means our NICs are connected at `0000:00:08.0` and `0000:00:09.0`.
You are connected via the emulated Intel NIC.
Do not try to use it with Ixy.

Let's send some packets (from VM `pktgen`)
```
sudo ./ixy/ixy-pktgen 0000:00:08.0
```

And forward them on VM `fwd`
```
sudo ./ixy/ixy-fwd 0000:00:08.0 0000:00:09.0
```

**Caution:** The receive counter of the `ixy-pktgen` application does not work in this setup: it only counts packets actually retrieved by the application when using VirtIO due to the lack of statistics registers. 

Is it fast?
============

No. It's incredibly slow. But that's not ixy's fault, the performance is lost somewhere in the VirtualBox stack. Other drivers like DPDK aren't faster.
#!/bin/bash
mkdir -p /mnt/huge
(mount | grep /mnt/huge) > /dev/null || mount -t hugetlbfs hugetlbfs /mnt/huge
for i in /sys/devices/system/node/node[0-9]*
do
	echo 512 > "$i"/hugepages/hugepages-2048kB/nr_hugepages
done

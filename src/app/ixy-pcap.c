#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include "driver/device.h"

const int BATCH_SIZE = 32;

// From https://wiki.wireshark.org/Development/LibpcapFileFormat
typedef struct pcap_hdr_s {
	uint32_t magic_number;  /* magic number */
	uint16_t version_major; /* major version number */
	uint16_t version_minor; /* minor version number */
	int32_t  thiszone;      /* GMT to local correction */
	uint32_t sigfigs;       /* accuracy of timestamps */
	uint32_t snaplen;       /* max length of captured packets, in octets */
	uint32_t network;       /* data link type */
} pcap_hdr_t;

typedef struct pcaprec_hdr_s {
	uint32_t ts_sec;        /* timestamp seconds */
	uint32_t ts_usec;       /* timestamp microseconds */
	uint32_t incl_len;      /* number of octets of packet saved in file */
	uint32_t orig_len;      /* actual length of packet */
} pcaprec_hdr_t;

int main(int argc, char* argv[]) {
	if (argc < 3 || argc > 4) {
		printf("Usage: %s <pci bus id> <output file> [n packets]\n", argv[0]);
		return 1;
	}

	struct ixy_device* dev = ixy_init(argv[1], 1, 1, 0);

	FILE* pcap = fopen(argv[2], "wb");
	if (pcap == NULL) {
		error("failed to open file %s", argv[2]);
	}

	int64_t n_packets = -1;
	if (argc == 4) {
		n_packets = atol(argv[3]);
		printf("Capturing %ld packets...\n", n_packets);
	} else {
		printf("Capturing packets...\n");
	}

	pcap_hdr_t header = {
		.magic_number =  0xa1b2c3d4,
		.version_major = 2,
		.version_minor = 4,
		.thiszone = 0,
		.sigfigs = 0,
		.snaplen = 65535,
		.network = 1, // Ethernet
	};
	fwrite(&header, sizeof(header), 1, pcap);

	struct pkt_buf* bufs[BATCH_SIZE];
	while (n_packets != 0) {
		uint32_t num_rx = ixy_rx_batch(dev, 0, bufs, BATCH_SIZE);
		struct timeval tv;
		gettimeofday(&tv, NULL);

		for (uint32_t i = 0; i < num_rx && n_packets != 0; i++) {
			pcaprec_hdr_t rec_header = {
				.ts_sec = tv.tv_sec,
				.ts_usec = tv.tv_usec,
				.incl_len = bufs[i]->size,
				.orig_len = bufs[i]->size
			};
			fwrite(&rec_header, sizeof(pcaprec_hdr_t), 1, pcap);

			fwrite(bufs[i]->data, bufs[i]->size, 1, pcap);

			pkt_buf_free(bufs[i]);
			// n_packets == -1 indicates unbounded capture
			if (n_packets > 0) {
				n_packets--;
			}
		}
	}

	fclose(pcap);
	return 0;
}

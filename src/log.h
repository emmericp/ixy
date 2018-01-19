#ifndef IXY_LOG_H
#define IXY_LOG_H

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <assert.h>

#ifndef NDEBUG
#define debug(fmt, ...) do {\
	fprintf(stderr, "[DEBUG] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)
#else
#define debug(fmt, ...) do {} while(0)
#undef assert
#define assert(expr) (void) (expr)
#endif

#define info(fmt, ...) do {\
	fprintf(stdout, "[INFO ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define warn(fmt, ...) do {\
	fprintf(stderr, "[WARN ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define error(fmt, ...) do {\
	fprintf(stderr, "[ERROR] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
	abort();\
} while(0)

#define check_err(expr, op) ({\
	int64_t result = (int64_t) (expr);\
	if ((int64_t) result == -1LL) {\
		int err = errno;\
		char buf[512];\
		strerror_r(err, buf, sizeof(buf));\
		fprintf(stderr, "[ERROR] %s:%d %s(): Failed to %s: %s\n", __FILE__, __LINE__, __func__, op, buf);\
		exit(err);\
	}\
	result;\
})

static void hexdump(void* void_ptr, size_t len) {
	uint8_t* ptr = (uint8_t*) void_ptr;
	char ascii[17];
	for (uint32_t i = 0; i < len; i += 16) {
		printf("%06x: ", i);
		int j = 0;
		for (; j < 16 && i + j < len; j++) {
			printf("%02x", ptr[i + j]);
			if (j % 2) {
				printf(" ");
			}
			ascii[j] = isprint(ptr[i + j]) ? ptr[i + j] : '.';
		}
		ascii[j] = '\0';
		if (j < 16) {
			for (; j < 16; j++) {
				printf("  ");
				if (j % 2) {
					printf(" ");
				}
			}
		}
		printf("  %s\n", ascii);
	}
}

#endif //IXY_LOG_H

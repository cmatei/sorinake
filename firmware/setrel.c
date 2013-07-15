#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static char *device = "/dev/i2c-1";

#define REGS_LEN 144
static uint8_t buffer[REGS_LEN];

static void print_buffer(uint8_t *buffer, size_t len)
{
	int i;

	for (i = 0; i < len; i++) {
		if ((i % 16 == 0) && i)
			printf("\n");

		if ((i % 16 == 0))
			printf("%04x: ", i);

		printf("%02x ", buffer[i]);
	}

	printf("\n");
}

int main(int argc, char **argv)
{
	int i, fd;
	uint8_t val = 0;
	int tx = 0;

	while ((i = getopt(argc, argv, "d:")) != -1) {
		switch (i) {
		case 'd':
			device = optarg;
			break;
		}
	}

	if (argc > 1) {
		tx = 1;
		val = atoi(argv[1]);
	}

	if ((fd = open(device, O_RDWR)) < 0) {
		perror("open");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE, 0x32) < 0) {
		perror("ioctl");
		exit(1);
	}

	if (tx) {
		buffer[0] = 1;
		buffer[1] = val;

		write(fd, buffer, 2);
	}

	printf("read: %d\n", read(fd, buffer, REGS_LEN));

	print_buffer(buffer, REGS_LEN);

	close(fd);

	return 0;
}

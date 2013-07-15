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
static uint8_t val = 0xff;

static uint8_t buffer[256];

int main(int argc, char **argv)
{
	int i, fd;

	while ((i = getopt(argc, argv, "d:")) != -1) {
		switch (i) {
		case 'd':
			device = optarg;
			break;
		}
	}

	if (argc > 1)
		val = atoi(argv[1]);

	if ((fd = open(device, O_RDWR)) < 0) {
		perror("open");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE, 0x32) < 0) {
		perror("ioctl");
		exit(1);
	}

	buffer[0] = 16;
	buffer[1] = val;
	write(fd, buffer, 2);

	close(fd);

	return 0;
}

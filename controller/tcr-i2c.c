#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/i2c-dev.h>

#include "tcr-i2c.h"

#ifndef SIMULATE

#define DEVNAMELEN 128
static int i2c_open(int bus, int address)
{
	char device[DEVNAMELEN];
	int fd;

	snprintf(device, DEVNAMELEN, "/dev/i2c-%d", bus);
	if ((fd = open(device, O_RDWR)) < 0)
		return -1;

	if (ioctl(fd, I2C_SLAVE, address) < 0) {
		close(fd);
		return -2;
	}

	return fd;
}

int i2c_read(int bus, int address, unsigned char *buffer, size_t len)
{
	int fd, r = 0;

	if ((fd = i2c_open(bus, address)) < 0)
		return fd;

	if (read(fd, buffer, len) != len)
		r = -1;

	close(fd);

	return r;
}

int i2c_write(int bus, int address, unsigned char *buffer, size_t len)
{
	int fd, r = 0;

	if ((fd = i2c_open(bus, address)) < 0)
		return fd;

	if (write(fd, buffer, len) != len)
		r = -1;

	close(fd);

	return r;
}

#else

// stub test functions
int i2c_read(int bus, int address, unsigned char *buffer, size_t len)
{
	static unsigned char data[] = {
		0x04, 0x01, 0x3a, 0x00, 0x4b, 0x46, 0xff, 0xff, 0x08, 0x10, 0x41, 0x00, 0x0c, 0x00, 0x00, 0x00,
		0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x00, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0x12, 0x00, 0x00, 0x00,
		0x03, 0x00, 0xe6, 0x02, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0x12, 0x00, 0x00, 0x00,
		0x02, 0x01, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0xef, 0x02, 0x11, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xd9, 0x00, 0xd9, 0x00, 0xd9, 0x00, 0xd9, 0x00, 0xd9, 0x00, 0x11, 0x00, 0x00, 0x00,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x00, 0x00, 0x1b, 0x00, 0x00, 0x00, 0x55, 0xaa
	};

	memcpy(buffer, data, (len <= 144) ? len : 144);
	return 0;
}

int i2c_write(int bus, int address, unsigned char *buffer, size_t len)
{
	return 0;
}

#endif


/*---------------------------------------------------------------------------
   Function :   kth_smallest()
   In       :   array of elements, # of elements in the array, rank k
   Out      :   one element
   Job      :   find the kth smallest element in the array

                Reference:

                  Author: Wirth, Niklaus
                   Title: Algorithms + data structures = programs
               Publisher: Englewood Cliffs: Prentice-Hall, 1976
    Physical description: 366 p.
                  Series: Prentice-Hall Series in Automatic Computation

 ---------------------------------------------------------------------------*/

static inline double dkth_smallest(double a[], int n, int k)
{
    int i,j,l,m ;
    double t, x ;

    l=0 ; m=n-1 ;
    while (l<m) {
        x=a[k] ;
        i=l ;
        j=m ;
        do {
            while (a[i]<x) i++ ;
            while (x<a[j]) j-- ;
            if (i<=j) {
                t=a[i]; a[i]=a[j]; a[j]=t;
                i++ ; j-- ;
            }
        } while (i<=j) ;
        if (j<k) l=i ;
        if (k<i) m=j ;
    }
    return a[k] ;
}


/* find median of n double array.
 * N.B. array is rearranged IN PLACE.
 */
double dmedian(double a[], int n)
{
        return (dkth_smallest (a, n, n/2));
}

#define REGS_LEN 144
int slave_get_data(int bus, int address, struct slave_data *data)
{
	unsigned char buffer[REGS_LEN];
	int r, i, j;
	int16_t TEMPERATURE_LSB, TEMPERATURE_MSB; // sign extended below
	unsigned int COUNT_REMAIN, COUNT_PER_C;
	unsigned int ADCL, ADCH;
	int temp, base;
	double tbuf[5];

	if ((r = i2c_read(bus, address, buffer, REGS_LEN)) != 0)
		return r;

	/* check the marker */
	if ((buffer[142] != 0x55) || (buffer[143] != 0xAA))
		return -1;

	for (i = 0; i < 4; i++) {

		/* DS1820 */
		base = i * 16;

		TEMPERATURE_LSB = buffer[base + 2];
		TEMPERATURE_MSB = buffer[base + 3];
		COUNT_REMAIN    = buffer[base + 8];
		COUNT_PER_C     = buffer[base + 9];

		/* take care as it is signed */
		temp = (int16_t) ((TEMPERATURE_MSB << 8) | TEMPERATURE_LSB);
		if (COUNT_PER_C != 0) {
			temp = temp >> 1; // truncate 0.5C bit
			data->t1[i] = temp - 0.25 + (COUNT_PER_C - COUNT_REMAIN) / (1.0 * COUNT_PER_C);
		} else {
			data->t1[i] =  temp / 2.0;
		}

		data->t1_count[i] =
			((unsigned int) buffer[base + 12] << 0 ) |
			((unsigned int) buffer[base + 13] << 8 ) |
			((unsigned int) buffer[base + 14] << 16) |
			((unsigned int) buffer[base + 15] << 24) ;

		// printf("%f %u\n", data->t1[i], data->t1_count[i]);

		/* PT1000 */
		base = (i + 4) * 16;
		for (j = 0; j < 5; j++) {
			ADCL = buffer[base + j * 2 + 2];
			ADCH = buffer[base + j * 2 + 3];

			data->t2_raw[i][j] = (unsigned int) ((ADCH << 8) | ADCL);
			tbuf[j] = data->t2_raw[i][j];

			// printf("%.2f ", tbuf[j]);
		}
		data->t2[i] = dmedian(tbuf, 5);
		// printf("-> %.2f ", data->t2[i]);

		data->t2_count[i] =
			((unsigned int) buffer[base + 12] << 0 ) |
			((unsigned int) buffer[base + 13] << 8 ) |
			((unsigned int) buffer[base + 14] << 16) |
			((unsigned int) buffer[base + 15] << 24) ;

		// printf("count %u\n", data->t2_count[i]);
	}

	data->relays[0] = (buffer[128] & 0x01) ? 1 : 0;
	data->relays[1] = (buffer[128] & 0x02) ? 1 : 0;
	data->relays[2] = (buffer[128] & 0x04) ? 1 : 0;
	data->relays[3] = (buffer[128] & 0x08) ? 1 : 0;


	data->uptime =
		((unsigned int) buffer[138] << 0 ) |
		((unsigned int) buffer[139] << 8 ) |
		((unsigned int) buffer[140] << 16) |
		((unsigned int) buffer[141] << 24) ;


	return 0;
}

int slave_set_relays(int bus, int address, unsigned relays)
{
	unsigned char buffer[2];

	buffer[0] = 1;
	buffer[1] = (unsigned char) (relays & 0x0F);

	return i2c_write(bus, address, buffer, 2);
}

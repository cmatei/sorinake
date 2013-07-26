#ifndef __TCR_I2C_H
#define __TCR_I2C_H

struct slave_data {
	double t1[4];
	unsigned int t1_count[4];

	double t2[4];
	unsigned int t2_raw[4][5];
	unsigned int t2_count[4];

	unsigned char relays[4];

	unsigned int uptime;
	unsigned int errcnt;
};

extern int i2c_read(int bus, int address, unsigned char *buffer, size_t len);
extern int i2c_write(int bus, int address, unsigned char *buffer, size_t len);

extern int slave_get_data(int bus, int address, struct slave_data *data);
extern int slave_set_relays(int bus, int address, unsigned relays);

#endif

CC	= gcc -Wall -O2 -g -DSIMULATE=1
CFLAGS	= $(shell pkg-config --cflags lua5.1)
LIBS	= $(shell pkg-config --libs lua5.1)


all: tcr

.c.o:
	$(CC) $(CFLAGS) -c -o $@ $^

tcr: tcr.o tcr-i2c.o
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o tcr

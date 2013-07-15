CC	= avr-gcc -mmcu=atmega8 -fomit-frame-pointer
OBJCOPY	= avr-objcopy
CFLAGS	= -Wall -Os -g -Wall -I.

all: main.hex

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $^

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $^ $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $^ $@

main.elf: main.o crc8.o
	$(CC) -o $@ $^

install: main.hex
	sudo avrdude -pm8 -cusbtiny -U flash:w:main.hex:i

clean:
	rm -f *.hex *.bin *.o *.elf

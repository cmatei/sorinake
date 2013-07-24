
#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

#include <string.h>

#include "crc8.h"

/* after this time we open relays if nothing heard on I2C */
#define SAFEMODE_SECONDS 60

/* I2C */
#define TWI_ADDRESS 0x32

#define TWI_CMD_SET_RELAYS 1

/* relay pins */
#define RELAY_1 _BV(PD2)
#define RELAY_2 _BV(PD3)
#define RELAY_3 _BV(PD6)
#define RELAY_4 _BV(PD7)

#define RELAY_MASK (RELAY_1 | RELAY_2 | RELAY_3 | RELAY_4)

/* power LED */
#define PWR_LED _BV(PD5)

/* DS1820 pins */
#define DS1820_1 _BV(PB2)
#define DS1820_2 _BV(PB3)
#define DS1820_3 _BV(PB4)
#define DS1820_4 _BV(PB5)

#define DS1820_MASK (DS1820_1 | DS1820_2 | DS1820_3 | DS1820_4)

#define DS1820_OUT PORTB
#define DS1820_IN  PINB
#define DS1820_DDR DDRB

#define DS1820_CMD_SKIP_ROM        0xCC
#define DS1820_CMD_CONVERT_T       0x44
#define DS1820_CMD_READ            0xBE


/* Analog multiplexer */
#define AMUX_1 _BV(PB0)
#define AMUX_2 _BV(PB1)

#define AMUX_MASK (AMUX_1 | AMUX_2)

enum {
	DS1820_IDLE = 1,
	DS1820_CONVERT,
	DS1820_READ,
};

struct ds1820 {
	uint8_t pin;
	uint8_t state;
	uint8_t data[9];
	uint8_t pad;
	uint32_t read_count;
} __attribute__ ((packed));

struct pt1000 {
	uint8_t amux;
	uint8_t converting;
	uint16_t raw[5];
	uint32_t count;
} __attribute__ ((packed));

struct i2c_registers {
	struct ds1820 ds1820[4];
	struct pt1000 pt1000[4];
	uint8_t  relays;
	uint8_t  pad[5];
	uint32_t twi_timestamp;
	uint32_t timestamp;
	uint16_t marker;
} __attribute__ ((packed));

static struct i2c_registers regs = {
	.ds1820 = {
		{ .pin = DS1820_1, .state = DS1820_IDLE },
		{ .pin = DS1820_2, .state = DS1820_IDLE },
		{ .pin = DS1820_3, .state = DS1820_IDLE },
		{ .pin = DS1820_4, .state = DS1820_IDLE }
	},

	.pt1000 = {
		{ .amux = 1 },
		{ .amux = 3 },
		{ .amux = 2 },
		{ .amux = 0 }
	},

	.marker = 0xAA55,
};

#define TWI_TXBUFSIZE (sizeof(struct i2c_registers))
static volatile uint8_t *twi_txbuf = (uint8_t *) &regs;
static volatile uint8_t twi_txaddr;

#define TWI_RXBUFSIZE 2
static volatile uint8_t twi_rxbuf[TWI_RXBUFSIZE];
static volatile uint8_t twi_rxaddr;

uint8_t ds1820_reset(struct ds1820 *sensor)
{
	uint8_t pin = sensor->pin;
	uint8_t err = 0;

	DS1820_OUT &= ~pin;
	DS1820_DDR |= pin;
	_delay_us(500);
	cli();
	DS1820_OUT |= pin; // pullup
	DS1820_DDR &= ~pin;
	_delay_us(65);
	err = DS1820_IN & pin;		// no presence detect
	sei();
	_delay_us(490 - 65);
	if ((DS1820_IN & pin) == 0)		// short circuit
		err = 1;

	return err;
}


uint8_t ds1820_bit_io(struct ds1820 *sensor, uint8_t b)
{
	uint8_t pin = sensor->pin;

	cli();
	DS1820_OUT &= ~pin;		     // drive low
	DS1820_DDR |= pin;
	_delay_us(2);
	if (b) {
		DS1820_DDR &= ~pin;
		DS1820_OUT |= pin; 	     // pullup
	}
	_delay_us(15-2);
	if ((DS1820_IN & pin) == 0 )
		b = 0;
	_delay_us(60-15);
	DS1820_DDR &= ~pin;
	DS1820_OUT |= pin;
	sei();
	return b;
}

uint8_t ds1820_write(struct ds1820 *sensor, uint8_t b)
{
	uint8_t i = 8, j;
	do {
		j = ds1820_bit_io(sensor, b & 1);
		b >>= 1;
		if (j)
			b |= 0x80;
	} while (--i);
	return b;
}


uint8_t ds1820_read(struct ds1820 *sensor)
{
	return ds1820_write(sensor, 0xFF);
}

uint8_t ds1820_read_scratchpad(struct ds1820 *sensor)
{
        int i;

        if (ds1820_reset(sensor))
                return 1;

        ds1820_write(sensor, DS1820_CMD_SKIP_ROM);
        ds1820_write(sensor, DS1820_CMD_READ);

        for (i = 0; i < 9; i++)
                sensor->data[i] = ds1820_read(sensor);

        if (crc8(sensor->data, 9))
                return 1;

        return 0;
}

uint8_t ds1820_convert(struct ds1820 *sensor)
{
        if (ds1820_reset(sensor))
                return 1;

        ds1820_write(sensor, DS1820_CMD_SKIP_ROM);
        ds1820_write(sensor, DS1820_CMD_CONVERT_T);

        return 0;
}

static void ds1820_state_machine(struct ds1820 *sensor)
{
	switch (sensor->state) {

	case DS1820_IDLE:
		if (!ds1820_convert(sensor))
			sensor->state = DS1820_CONVERT;
		break;

	case DS1820_CONVERT:
		if (ds1820_read(sensor) == 0xFF)
			sensor->state = DS1820_READ;
		break;

	case DS1820_READ:
		if (!ds1820_read_scratchpad(sensor)) {
			sensor->read_count++;
		}

		sensor->state = DS1820_IDLE;
		break;
	}
}

static int pt1000_read_sensor(struct pt1000 *sensor)
{
	if (sensor->converting) {
		if ((ADCSRA & (1 << ADSC)) != 0)
			return 0;

		/* conversion complete */
		sensor->converting = 0;

		/* shift values */
		memmove(&sensor->raw[1], &sensor->raw[0], 4 * sizeof(uint16_t));

		sensor->raw[0]  = ADCL;
		sensor->raw[0] += ((uint16_t) ADCH) << 8;

		sensor->count++;

		/* move to next amux channel */
		return 1;
	} else {
		sensor->converting = 1;
		ADCSRA |= (1 << ADSC);
	}

	return 0;
}

static void pt1000_set_mux(uint8_t index)
{
	struct pt1000 *sensor = &regs.pt1000[index];

	PORTB &= ~AMUX_MASK;
	PORTB |= sensor->amux;
}

static void relays_set(uint8_t mask)
{
	regs.relays = mask & 0x0f;
	PORTD = (PORTD & ~RELAY_MASK) |
		((regs.relays & _BV(0)) ? RELAY_1 : 0) |
		((regs.relays & _BV(1)) ? RELAY_2 : 0) |
		((regs.relays & _BV(2)) ? RELAY_3 : 0) |
		((regs.relays & _BV(3)) ? RELAY_4 : 0);
}


#define TWACK   TWCR=(1<<TWINT)|(1<<TWIE)|(1<<TWEN)|(1<<TWEA)
#define TWNACK  TWCR=(1<<TWINT)|(1<<TWIE)|(1<<TWEN)
#define TWRESET TWCR=(1<<TWINT)|(1<<TWIE)|(1<<TWEN)|(1<<TWSTO)|(1<<TWEA)

void TWI_init(uint8_t address)
{
	TWAR = address;
	TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) |
	       (1<<TWEA) | (0<<TWSTA) | (0<<TWSTO) |
	       (0<<TWWC);
}

ISR(TWI_vect)
{
	switch (TWSR) {

	/* receive from master */
	case TW_SR_SLA_ACK:
		TWACK;
		twi_rxaddr = 0;
		regs.twi_timestamp = regs.timestamp;
		break;
	case TW_SR_DATA_ACK:
		twi_rxbuf[twi_rxaddr++] = TWDR;
		if (twi_rxaddr >= TWI_RXBUFSIZE) {
			TWNACK;

			/* full buffer, process command */
			switch (twi_rxbuf[0]) {
			case TWI_CMD_SET_RELAYS:
				relays_set(twi_rxbuf[1]);
				break;
			}
		} else {
			TWACK;
		}
		break;
	case TW_SR_ARB_LOST_SLA_ACK:
		TWRESET;
		break;
	case TW_SR_STOP:
		TWACK;
		break;
	case TW_SR_DATA_NACK:
		TWACK;
		break;

        /* transmit to master */
	case TW_ST_SLA_ACK:
		TWDR = twi_txbuf[0];
		twi_txaddr = 1;
		TWACK;
		regs.twi_timestamp = regs.timestamp;
		break;
	case TW_ST_DATA_ACK:
		TWDR = twi_txbuf[twi_txaddr++];
		if (twi_txaddr >= TWI_TXBUFSIZE) {
			TWNACK;
		} else {
			TWACK;
		}
		break;

	case TW_ST_LAST_DATA:
	case TW_ST_DATA_NACK:
		TWACK;
		break;

	default:
		TWRESET;
		break;
	}
}

ISR(TIMER0_OVF_vect)
{
	static uint8_t count = 0;

	cli();

	/* 12 Mhz / 1024 / 256 = 45.78 */
	if (count++ == 46) {
		count = 0;
		regs.timestamp++;

		/* turn on power led */
		PORTD &= ~PWR_LED;

		/* wraps in ~136 years :P */
		if (regs.timestamp - regs.twi_timestamp > SAFEMODE_SECONDS)
			relays_set(0);
	}

	sei();

	/* turn off power led */
	if (count > 5)
		PORTD |= PWR_LED;
}


static void init_hardware()
{
	/* port D: relays */
	DDRD = RELAY_MASK | PWR_LED;
	PORTD &= ~(RELAY_MASK | PWR_LED);    // disable relays

	/* port B: AMUX + DS1820 */
	DDRB = AMUX_MASK;
	PORTB = DS1820_MASK;		     // DS1820 input pull-ups, mux 00

	DDRC = 0; 			     // all inputs
	PORTC = _BV(PC4) | _BV(PC5);	     // internal pull-ups on I2C

	/* initialize ADC, clock = F_CPU/64 */
	ADCSRA = (1 << ADEN) |
		 (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);

	/* AVCC reference, mux ADC0 */
	ADMUX = (0 << REFS1) | (1 << REFS0) |
		(0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);

	/* dummy conversion to initialize ADC analog sections */
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));


	/* set up timer, interrupt on overflow, clk/1024 */
	TCCR0 = (1 << CS02) | (0 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);

	/* init I2C */
	TWI_init(TWI_ADDRESS << 1);
}

int main()
{
	uint8_t ds1820_current = 0;
	uint8_t pt1000_current = 0;

	init_hardware();

	sei();

	for (;;) {
		ds1820_state_machine(&regs.ds1820[ds1820_current]);
		ds1820_current = (ds1820_current + 1) & 3;

		if (pt1000_read_sensor(&regs.pt1000[pt1000_current])) {
			/* can move to next amux channel */
			pt1000_current = (pt1000_current + 1) & 3;

			/* there's a large 220k resistor in front of the ADC.
			   Some delay is needed after switching voltages to
			   allow it to stabilise */
			pt1000_set_mux(pt1000_current);
		}

		_delay_ms(200);
	}


	return 0;
}

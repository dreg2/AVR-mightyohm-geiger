/*
    Title: Geiger Counter with Serial Data Reporting
    Description: This is the firmware for the mightyohm.com Geiger Counter.
        There is more information at http://mightyohm.com/geiger

    Author:     Jeff Keyzer
    Company:    MightyOhm Engineering
    Website:    http://mightyohm.com/
    Contact:    jeff <at> mightyohm.com

    This firmware controls the ATtiny2313 AVR microcontroller on board the Geiger Counter kit.

    When an impulse from the GM tube is detected, the firmware flashes the LED and produces a short
    beep on the piezo speaker.  It also outputs an active-high pulse (default 100us) on the PULSE pin.

    A pushbutton on the PCB can be used to mute the beep.

    A running average of the detected counts per second (CPS) and counts per minute (CPM)
    is output on the serial port once per second.

    The serial port is configured for BAUD baud, 8-N-1 (default 9600).

    The data is reported in comma separated value (CSV) format:
    CPS, #####*, CPM, #####*

    There are two compile-time modes.
    - normal CPM-maximum mode: allows CPS values up to 255 and CPM values up to 15,300
    - high CPM-maximum mode  : allows CPS values up to 65,535 and CPM values up to 1,966,050 at the
      cost of reduced resolution for the CPM value

    If a CPS value overflows a '*' is appended to the output CPS value. If the CPM value is calculated
    using an overflowed CPS value a '*' is appended to the output CPM value.

    ***** WARNING *****
    This Geiger Counter kit is for EDUCATIONAL PURPOSES ONLY.  Don't even think about using it to monitor radiation in
    life-threatening situations, or in any environment where you may expose yourself to dangerous levels of radiation.
    Don't rely on the collected data to be an accurate measure of radiation exposure! Be safe!


        Change log:
        8/4/11  1.00: Initial release for Chaos Camp 2011!
        1/25/20 1.10: dreg2 version


                Copyright 2011 Jeff Keyzer, MightyOhm Engineering

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define VERSION         "1.10"
#define URL             "http://mightyohm.com/geiger"

#define OUT_BUF_LEN     12     // serial output buffer length
#define PULSE_WIDTH     100    // length of the pulse pin output in us
#define FLASH_SHORT     10     // length of led/piezo short flash in ms
#define FLASH_LONG      250    // length of led/piezo long flash in ms

#ifndef BAUD
	// default baud rate if not defined in the Makefile
	#define BAUD            9600
#endif

#ifdef HIGH_CPM
	// high cpm-maximum mode (1,966,050), reduced resolution
	#define CPM_PERIOD      30         // number of samples to keep for cpm calculations
	#define CPM_T           uint32_t   // type for running total
	#define CPS_T           uint16_t   // type for sample array
	#define CPS_MAX         UINT16_MAX // maximum value for sample array
	#define CPM_UTOA        ultoa      // int to ascii conversion
#else
	// high resolution mode, reduced cpm-maximum (15,300)
	#define CPM_PERIOD      60         // number of samples to keep for cpm calculations
	#define CPM_T           uint16_t   // type for running total
	#define CPS_T           uint8_t    // type for sample array
	#define CPS_MAX         UINT8_MAX  // maximum value for sample array
	#define CPM_UTOA        utoa       // int to ascii conversion
#endif

#define DATA_FLASH      0      // data located in flash memory
#define DATA_SRAM       1      // data located in sram

#define PIN_PIEZO       PB2    // piezo pin
#define PIN_LED         PB4    // LED pin
#define PIN_BUTTON      PD3    // button pin
#define PIN_PULSE       PD6    // pulse pin

uint8_t  volatile event_flag;  // GM event flag
uint8_t  volatile tick_flag;   // timer tick flag
uint8_t  volatile mute_flag;   // mute beeper flag
uint16_t volatile cps;         // number of GM events that have occurred between timer ticks


//----------------------------------------------------------------------------------------------------
// INT0_vect - GM pulse
//----------------------------------------------------------------------------------------------------
ISR(INT0_vect)
	{
	// set event flag
	event_flag = 1;

	// increment counter if not at maximum
	if (cps < UINT16_MAX)
		cps++;

	// send a pulse to the pulse pin
	PORTD |= (uint8_t)(1 << PIN_PULSE);
	_delay_us(PULSE_WIDTH);
	PORTD &= (uint8_t)~(1 << PIN_PULSE);
	}

//----------------------------------------------------------------------------------------------------
// INT1_vect - push button
//----------------------------------------------------------------------------------------------------
ISR(INT1_vect)
	{
	// delay as a crude debounce
	_delay_ms(25);

	// toggle beep mode if button still pressed
	if ((PIND & (uint8_t)(1 << PIN_BUTTON)) == 0)
		mute_flag ^= 1;

	// clear interrupt flag to avoid executing ISR again due to switch bounce
	EIFR |= (uint8_t)(1 << INTF1);
	}

//----------------------------------------------------------------------------------------------------
// TIMER_COMPA_vect - timer tick
//----------------------------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
	{
	// set tick flag
	tick_flag = 1;
	}

//----------------------------------------------------------------------------------------------------
// signal_event - flash led and beep the piezo
//----------------------------------------------------------------------------------------------------
void signal_event(uint16_t flash_length)
	{
	// turn on led
	PORTB |= (uint8_t)(1 << PIN_LED);

	// if not muted, start beep
	if (!mute_flag)
		{
		TCCR0A |= (uint8_t)(1 << COM0A0);    // enable OCR0A output on pin PIN_PIEZO
		TCCR0B |= (uint8_t)(1 << CS01);      // set prescaler to clk/8 (1Mhz) or 1us/count
		OCR0A   = 160;                       // 160 = toggle OCR0A every 160ms, period = 320us, freq= 3.125kHz
		}

	// delay
	if (flash_length == FLASH_LONG)
		_delay_ms(FLASH_LONG);
	else
		_delay_ms(FLASH_SHORT);

	// turn off led
	PORTB  &= (uint8_t)~(1 << PIN_LED);  // turn off the led

	// stop beep
	TCCR0B  = 0;                         // disable timer0
	TCCR0A &= (uint8_t)~(1 << COM0A0);   // disconnect OCR0A from timer0, this avoids occasional HVPS whine after beep
	}

//----------------------------------------------------------------------------------------------------
// uart_putchar - send a character to the uart
//----------------------------------------------------------------------------------------------------
void uart_putchar(const char c)
	{
	// add return to newline for Windows
	if (c == '\n') uart_putchar('\r');

	// wait for uart ready
	loop_until_bit_is_set(UCSRA, UDRE);

	// send character to uart
	UDR = (uint8_t)c;
	}

//----------------------------------------------------------------------------------------------------
// uart_putstring - send a string to the uart
//----------------------------------------------------------------------------------------------------
void uart_putstring(const char *data, uint8_t data_location)
	{
	if (data_location == DATA_FLASH)
		{
		// send string from flash
		while (pgm_read_byte(data) != '\0')
			uart_putchar((char)pgm_read_byte(data++));
		}
	else
		{
		// send string from sram
		while (*data != '\0')
			uart_putchar(*data++);
		}
	}

//----------------------------------------------------------------------------------------------------
// calc_values - calculate output data values and send to uart
//----------------------------------------------------------------------------------------------------
void calc_values(void)
	{
	uint16_t        save_cps;                   // saved current GM cps
	CPM_T           cpm;                        // calculated GM event counts per minute 
	static CPM_T    running_total;              // running total of GM event counts per second
	static CPS_T    cps_samples[CPM_PERIOD];    // array of cps samples
	static uint8_t  sample = 0;                 // cps array index
	char            out_buf[OUT_BUF_LEN];       // serial output buffer

	#define         OVF_NONE 255
	static uint8_t  sample_ovf = OVF_NONE;      // cps array index for most recent overflow element

	// save current cps and reset cps, ready for next GM interrupt
	save_cps = cps;
	cps = 0;

	// subtract oldest sample from running total
	running_total -= cps_samples[sample]; 

	// save current sample to cps array
	if (save_cps < CPS_MAX)
		{
		// save current sample to cps array
		cps_samples[sample] = (CPS_T)save_cps;
		if (sample == sample_ovf)
			sample_ovf = OVF_NONE;           // clear overflow index
		}
	else
		{
		// save max value to cps array
		cps_samples[sample] = CPS_MAX;
		sample_ovf = sample;                     // set overflow index
		}

	// add current sample to running total
	running_total += cps_samples[sample];

	// calculate cpm
	cpm = running_total * (60 / CPM_PERIOD);

	// send cps value to the uart
	uart_putstring(PSTR("CPS, "), DATA_FLASH);
	utoa(save_cps, out_buf, 10);
	uart_putstring(out_buf, DATA_SRAM);
	if (sample_ovf == sample)
		uart_putchar('*');    // add overflow indicator

	// send cpm value to the uart
	uart_putstring(PSTR(", CPM, "), DATA_FLASH);
	CPM_UTOA(cpm, out_buf, 10);
	uart_putstring(out_buf, DATA_SRAM);
	if (sample_ovf != OVF_NONE)
		uart_putchar('*');    // add overflow indicator

	// output a newline.
	uart_putchar('\n');

	// adjust cps array index to next entry
	sample++;
	if (sample >= CPM_PERIOD)
		sample = 0;
	}

//----------------------------------------------------------------------------------------------------
// Start of main program
//----------------------------------------------------------------------------------------------------
int main(void)
	{
	// configure the uart
	#include <util/setbaud.h>    // defines UBRR register value constants based on BAUD and F_CPU
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	UCSRB = (1 << RXEN) | (1 << TXEN);

	// send title string
	uart_putstring(PSTR("mightyohm.com Geiger Counter " VERSION "\n" URL "\n"), DATA_FLASH);

	// configure IO pins
	DDRB   = (uint8_t)(1 << PIN_LED) | (uint8_t)(1 << PIN_PIEZO);  // set led pin and piezo pin as outputs
	DDRD   = (uint8_t)(1 << PIN_PULSE);                            // configure pulse pin output
	PORTD |= (uint8_t)(1 << PIN_BUTTON);                           // enable pull up resistor on button pin

	// configure external interrupts - INT0 = GM pulse, INT1 = button
	MCUCR |= (uint8_t)(1 << ISC01) | (uint8_t)(1 << ISC11);    // configure interrupts on falling edge of INT0 and INT1
	GIMSK |= (uint8_t)(1 << INT0)  | (uint8_t)(1 << INT1);     // enable external interrupts on pins INT0 and INT1

	// configure timer0 for tone generation
	TCCR0A = (0 << COM0A1) | (1 << COM0A0) | (0 << WGM02) | (1 << WGM01) | (0 << WGM00);
	TCCR0B = 0;

	// configure timer1 for 1 second interrupts
	TCCR1B = (uint8_t)(1 << WGM12) | (uint8_t)(1 << CS12);  // CTC mode, prescaler = 256 (32us ticks)
	OCR1A  = 31249;                                         // 32us * 31250 = 1 sec
	TIMSK  = (uint8_t)(1 << OCIE1A);                        // Timer1 overflow interrupt enable

	// enable interrupts
	sei();

	// signal powered-up and ready
	signal_event(FLASH_LONG);

	// infinite loop
	while(1)
		{
		// configure for sleep-mode-idle and go to sleep, sleep ends on interrupt
		MCUCR &= (uint8_t)~((1 << SM1) | (1 << SM0));
		sleep_mode();

		// handle clock tick
		if (tick_flag)
			{
			// clear tick flag
			tick_flag = 0;

			// calculate values and send to uart
			calc_values();
			}

		// handle GM event
		while (event_flag)
			{
			// clear event flag
			event_flag = 0;

			// flash led and beep
			signal_event(FLASH_SHORT);
			}
		}

	return 0;
	}

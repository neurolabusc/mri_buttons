/* Keyboard example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2008 PJRC.COM
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *  
 * +New debounce method added by Chris Rorden. Same conditions apply
 * +New analog input, comparing sensor on PF0 to potentiometer reference on PF1
 *   http://www.mccauslandcenter.sc.edu/CRNL/tools/screensync
 * Assumes digital button switches that close PD0, PD1, PD2, PD3 to ground
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_keyboard.h"
#include "analog.h"
#include "analog.c"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define LED_CONFIG	(DDRD |= (1<<6))

#define kDebounceTime 3 //how many interrupts for debounce: e.g. if 3 and 2.048ms/per interrupt then 6.144ms minimum repeat 
#define kAnalogDebounceTime 240 //how many interrupts for debounce: e.g. if 3 and 2.048ms/per interrupt then 6.144ms minimum repeat 

#define nKey 4		// number of digital inputs to track
volatile  uint8_t keys_D[nKey]= {KEY_0,KEY_1,KEY_2,KEY_3};//key mapping for port D
volatile uint8_t debounce_D[nKey]={0,0,0,0};//debounce information for port D

//#define nKey_A 1	// number of analog inputs to track
//int16_t ref_A[nKey_A]= {KEY_A};//key mapping for ANALOG port 
//int16_t pin_A[nKey_A]= {0};//key mapping for ANALOG port 
//uint8_t keys_A[nKey_A]= {KEY_A};//key mapping for ANALOG port 
//volatile uint8_t debounce_A[nKey_A]={0};//debounce information for ANALOG port
//uint8_t reverse_A[nKey_A]={0};//polarity for analog ports

#define nKey_A 2	// number of analog inputs to track
int16_t ref_A[nKey_A]= {KEY_A, KEY_B};//key mapping for ANALOG port 
int16_t pin_A[nKey_A]= {0, 1};//key mapping for ANALOG port 
uint8_t keys_A[nKey_A]= {KEY_A, KEY_B};//key mapping for ANALOG port 
volatile uint8_t debounce_A[nKey_A]={0,0};//debounce information for ANALOG port
uint8_t reverse_A[nKey_A]={0,0};//polarity for analog ports

int main(void)
{
	uint8_t current_d, current_a, mask, i;
	uint8_t prev_d=0xFF;
	uint8_t prev_a=0xFF;
	int16_t sensor_analog, reference_analog;     
	//CPU_PRESCALE(0); // set for 16 MHz clock - requires makefile statement F_CPU = 16000000
	CPU_PRESCALE(1); // set for 8 MHz clock - requires makefile statement F_CPU = 8000000
	// Configure all port B pins as inputs with pullup resistors.
	// See the "Using I/O Pins" page for details.
	// http://www.pjrc.com/teensy/pins.html

    	DDRD = 0x00; //set port D as inputs
    	PORTD = 0xFF; //pullup all

    	DDRC = 0x00; //set port C as inputs
    	PORTC = 64; //pullup PC6 
	LED_OFF;
	//int reversePolarity = 0;
	//if ((PINC & 64) != 64) {
	//	reversePolarity = 1 - reversePolarity;
	//}
	
	

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

    //TCCR0B http://www.protostack.com/blog/2010/09/timer-interrupts-on-an-atmega168/ 
    //TIMSK0 http://www2.tech.purdue.edu/eet/courses/referencematerial/atmel/Timer_Stuff/TIMSK.htm
	// Configure timer 0 to generate a timer overflow interrupt every
	// 256*64 clock cycles, or every 1.024ms using 16 MHz clock (every 16000000/(256*64)sec)
    //         or every 2.048ms using 8 MHz clock (every 8000000/(256*64)sec)
	// This demonstrates how to use interrupts to implement a simple
	// inactivity timeout.
	TCCR0A = 0x00; //Counter 0, Control register A
	TCCR0B = 0x03; //set CS0x bits set prescaling 0=stopped, 1=x1, 2=x8, 3=x64, 4=x256, 5=x1024 
	TIMSK0 = (1<<TOIE0); //OIE="Overflow Interrupt Enable", this 8-bit timer will create interrupt every 256 ticks
	for (int p=0; p < nKey_A; p++) {
		ref_A[p] = analogRead(pin_A[p]);
		for (i=0; i<100; i++) {
			sensor_analog = analogRead(pin_A[p]);
			if (reverse_A[p] == 1) {
        		if (sensor_analog < ref_A[p]) reference_analog = ref_A[p];
        	} else {
				if (sensor_analog > ref_A[p]) reference_analog = ref_A[p];
			}
		}	
		if (reverse_A[p] == 1) {
			if (ref_A[p] > 0) ref_A[p] = ref_A[p]-1;
			if (ref_A[p] > 9) ref_A[p] = ref_A[p]-9;
			ref_A[p] = 400;
		} else {
			ref_A[p] = ref_A[p]+1;
			ref_A[p] = ref_A[p]+1;
		}	
	}

	//reference_analog = analogRead(0);
	//for (i=0; i<100; i++) {
	//	sensor_analog = analogRead(0);
	//	if (sensor_analog > reference_analog) reference_analog = sensor_analog;
	//}
	while (1) {
		// read all port D pins
		current_d = PIND;
		// check if any pins are low, but were high previously
		mask = 1;

        	//note we do not stop interrupts when we read volatile debounce_D, but errors will be caught in next cycle
		for (i=0; i<nKey; i++) {
			if (  ((prev_d & mask) != (current_d & mask))  && (debounce_D[i] == 0) ) {
				if ((current_d & mask) == 0) {
                    			usb_keyboard_press(keys_D[i], 0);
                		}
                		cli();
                		debounce_D[i] = kDebounceTime;
                		sei();
			} //Port D
			mask = mask << 1;
		}
		prev_d = current_d;

		//read analog signals
		mask = 1;
		current_a = 0;
        for (i=0; i<nKey_A; i++) {
        	sensor_analog = analogRead(pin_A[i]);
        	if (reverse_A[i] == 1) {
        		if (sensor_analog <  ref_A[i]) current_a = current_a + mask;
        	} else {
				if (sensor_analog >  ref_A[i]) current_a = current_a + mask;	
			}
			mask = mask << 1;
		}
		
		//if (analogRead(0)  > reference_analog) {
		//	current_a = 1;
		//} else {
		//	current_a = 0;
		//}

		mask = 1;
        for (i=0; i<nKey_A; i++) {
			if (  ((prev_a & mask) != (current_a & mask))  && (debounce_A[i] == 0) ) {
				if ((current_a & mask) != 0) {
                    		usb_keyboard_press(keys_A[i], 0);
							LED_ON;
                			cli();
                			debounce_A[i] = kAnalogDebounceTime;
                			sei();
                		}

			} //Port A
			mask = mask << 1;
		}
		prev_a = current_a;
		
	}
}


// This interrupt routine is run approx every 2.048ms at 8 Mhz.
//  http://tom-itx.dyndns.org:81/~webpage/abcminiuser/articles/avr_interrupts_index.php
ISR(TIMER0_OVF_vect)
{
    uint8_t i;
    for (i=0; i<nKey; i++) {
        if (debounce_D[i] > 0) {
            debounce_D[i]--;
        }
    }
    for (i=0; i<nKey_A; i++) {
        if (debounce_A[i] > 0) {
        	debounce_A[i]--;
		if (debounce_A[i] == 0) {
			LED_OFF;
		}
        }
    } //for each analog key


}
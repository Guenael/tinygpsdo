/*
 * FreeBSD License
 * Copyright (c) 2016, Guenael
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


// AD5061
// http://www.analog.com/media/en/technical-documentation/data-sheets/AD5061.pdf


#include "config.h"
#include "dac.h"

#include <avr/io.h>
#include <util/delay.h>


void dacTransmitByte(uint8_t data) {
    /* Enable PLL LE */
    PORTB &= ~_BV(PORTB2);

    /* Start transmission */
    SPDR = data;

    /* Wait for transmission complete */
    while(!(SPSR & _BV(SPIF)));

    /* Disable PLL LE */
    PORTB |= _BV(PORTB2);
}


void dacTransmitWord(uint32_t data) {
    /* Enable PLL LE */
    PORTB &= ~_BV(PORTB2);

    uint8_t *p = (uint8_t*)&data;
    for (uint8_t i=0; i<4; i++) {
        /* Start transmission */
        SPDR = p[3-i];  // Little endian

        /* Wait for transmission complete */
        while(!(SPSR & _BV(SPIF)));
    }

    /* Disable PLL LE */
    PORTB |= _BV(PORTB2);
}


// FIXME !!!
// 012
// 210
void dacTransmit24bits(uint32_t data) {
    /* Enable PLL LE */
    PORTB &= ~_BV(PORTB2);
    _delay_us(1); 

    uint8_t *p = (uint8_t*)&data;
    SPDR = p[3];  // Little endian
    while(!(SPSR & _BV(SPIF)));  /* Wait for transmission complete */

    SPDR = p[2];  // Little endian
    while(!(SPSR & _BV(SPIF)));  /* Wait for transmission complete */

    SPDR = p[1];  // Little endian
    while(!(SPSR & _BV(SPIF)));  /* Wait for transmission complete */

    _delay_us(1); 

    /* Disable PLL LE */
    PORTB |= _BV(PORTB2);
}


void dacTransmitAlt(uint32_t data) {
	PORTB &= ~_BV(PORTB2);
	_delay_us(1);

	for(int i = 23; i >= 0; i--) {
		if ((data >> i) & 0x1) {
			PORTB |= _BV(PORTB3);
			_delay_us(1);
		} else {
			PORTB &= ~_BV(PORTB3);
			_delay_us(1);
		}

		PORTB &= ~_BV(PORTB5);
		_delay_us(1);

		PORTB |= _BV(PORTB5);
		_delay_us(1);
	}

	PORTB |= _BV(PORTB2);
	_delay_us(1);
}


void dacInit() {
    DDRB   |= _BV(DDB3);       /* MOSI   - Enable output */
    DDRB   |= _BV(DDB5);       /* SCK    - Enable output */

    DDRB   |= _BV(DDB2);       /* DAC_LE - Enable output */
    PORTB  |= _BV(PORTB2);     /* DAC_LE disable */

    PORTB  |= _BV(PORTB5);  // FIXME

    /* Enable SPI, as Master, prescaler = Fosc/16 */
    //SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);

    /* First initialisation of the DAC, with zero value */
    dacTransmitAlt(0);
    _delay_ms(100);
}


void dacShutdown() {
}

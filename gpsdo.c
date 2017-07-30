/*
 * FreeBSD License
 * Copyright (c) 2016, 2017, Guenael
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


/* |                                                                   |
   |  TinyBeacon project                                               |
   |                                                                   |
   |  - 10 MHz oscillator stabilized by GPS (GPSDO)                    |
   |  - 4 multiplexed output and 1 mixed output                        |
   |  - Compact design / Credit card size                              |
   |  - DC-DC Power supply within 10-15V, 0.5A max                     |
   |                                                                   |
   |                                                                   |
   |  IO Mapping uController, rev.A                                    |
   |                                                                   |
   |  - PC4/SDA  (pin 27) | I2C SDA                                    |
   |  - PC5/SCL  (pin 28) | I2C SCL                                    |
   |  - PD0/RXD  (pin 30) | USART RX                                   |
   |  - PD1/TXD  (pin 31) | USART TX                                   |
   |  - PD5      (pin  9) | GPS INT                                    |
   |  - PD6      (pin 10) | PA EN                                      |
   |  - PD7      (pin 11) | INFO LED                                   |
   |  - PB0      (pin 12) | PLL LOCK                                   |
   |  - PB2      (pin 14) | DAC LE                                     |
   |  ! PB3               | DAC MOSI (old design)                      |
   |  ! PB4      (pin 16) | DAC MISO (this board)                      |
   |  - PB5      (pin 17) | DAC SCK                                    |
   |                                                                   |
   |                                                                   | */


#include "config.h"

#include "twi.h"
#include "gps.h"
#include "dac.h"
#include "usart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


int main (void) {
    /* CKDIV8 fuse is set -- Frequency is divided by 8 at startup : 2.5MHz */
    cli();
    CLKPR = _BV(CLKPCE);  // Enable change of CLKPS bits
    CLKPR = 0;            // Set prescaler to 0 = Restore system clock to 10 MHz
    sei();

    /* LED : Set pin 11 of PORT-PD7 for output*/
    DDRD |= _BV(DDD7);

    /* For now, used for DEBUG purpose only. Future : CLI for freq settings & modes */
    usartInit();
    _delay_ms(10);

    /* Peform I2C modules init */
    twi_init();
    _delay_ms(10);

    /* uBlox : GPS IO init & Set the default I2C address of the GPS */
    gpsInit(0x42);  // I2C Have to be init before the PLL !

    /* uBlox : Rstrict DDC port only */
    gpsSet_CFG_PRT();

    /* uBlox : Wait on a full GPS sync (+ info req. for message encoding)*/
    gpsGetPVT();
    gpsExtractStrings();
    gpsGetTime();

    /* uBlox : 100kHz timing setup */
    gpsSet_CFG_TP5();

    /* uBlox : Refresh rate for internal GPSDO alignment */
    gpsSet_CFG_RATE();

    /* ADF4355 PLL Init, conf & settings */
    dacInit();

    /* End of init sequence : Turn on the LED (pin 11) */
    PORTD |= _BV(PORTD7);

    uint32_t dacValue;
    while(1) {
    	/* Get GPS data for the next time sync */
    	//gpsGetTime();

    	/* Align on an minute for the next message */
    	//gpsTimeAling1Mb();

        // TODO digital PLL ajust
        void dacTransmit24bits(dacValue);
        _delay_us(100); 
    }

    /* This case never happens :) Useless without powermanagement... */
    return 0;
}


// Fix CFG_TP5 avec Checksum
// Fix CFG_RATE
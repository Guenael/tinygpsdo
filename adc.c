/* 
 * FreeBSD License
* Copyright (c) 2015, 2016, 2017 Guenael 
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


#include "cpu.h"
#include "sensors.h"

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Vcc voltage (regulated output) */
#define VCC 3.3

/* Interal 1.1V reference voltage */
#define VREF 1.1


void adcInit() { 
    /* Disable power saving */
    PRR &= ~_BV(PRADC);

    /* Disable the digital buffer for the 4 first ADC */
    DIDR0 = _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D) | _BV(ADC3D);  

    /* Configure the ADC */
    ADMUX  |= _BV(REFS0);                             // Select Vref=AVcc, by default
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);   // Set Prescaler = 128, take your time...
    ADCSRA |= _BV(ADEN);                              // Enable ADC
    //ADCSRA |= _BV(ADLAR);                           // Bug, do not use...
}


void adcShutdown() {
    /* Disable ADC */
    ADCSRA &= ~_BV(ADEN);   
    
    /* Enable powersaving for ADC sub-system */
    PRR |= _BV(PRADC);      
}


void adcSetRef(uint8_t ref) {  // 0 = ARef, 1 = Internal 1.1V
    ADMUX = (ADMUX & 0x3f);
    if (ref)
        ADMUX  |= _BV(REFS1) | _BV(REFS0);
    else
        ADMUX  |= _BV(REFS0);

}


uint16_t adcRead(uint8_t pinSelect) {
    ADMUX = (ADMUX & 0xf0) | (pinSelect & 0x0f);  // Select the input pin & keep previous settings
    ADCSRA |= _BV(ADSC);                          // Start the conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);        // Wait the end off the conversion

    return ADC;
}


float getAdcPllVoltage() {
    float value;

    adcSetRef(0);
    value = (float)adcRead(6);
    return (value * VCC) / 1024.0;
}

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


#include "config.h"
#include "usart.h"

#include <avr/io.h>


/* UART baud rate for GPS */
#define USART_BAUDRATE       9600

/* UART CPU timing config */
#define USART_BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) -1 )


/* Initialize the USART @ 9600 bauds (8N1), RX&TX */
void usartInit() {
    /* Disable power saving */
    PRR &= ~_BV(PRUSART0);

    UBRR0L = USART_BAUD_PRESCALE;        // Prescaler for 9600 bauds
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);    // Enable RX and TX
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);  // 8-bit data
}


void usartShutdown() {
    /* Disable USART RX/TX */
    UCSR0B &= ~_BV(TXEN0);

    /* Enable powersaving for USART sub-system */
    PRR |= _BV(PRUSART0);
}


/* Receive a char on serial port (USART) */
void usartGetChar(char *c) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    *c = UDR0;
}


/* Send a char on serial port (USART) */
void usartSendChar(char *c) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = *c;
}


/* Receive a string on serial port (USART) */
void usartGetString(char* str) {
    char tmp;
    do {
        loop_until_bit_is_set(UCSR0A, RXC0);
        tmp = UDR0;
        *str++ = tmp;  // CAUTION : No buffer overflow check - Fixed length !!
    } while (tmp != '\x0A' && tmp != '\x0D' && tmp != '\x00');
    *str = '\x00';
}


/* Receive a GPS specific string on serial port (USART) */
void usartGetStringGPS(char* str) {
    do {
        loop_until_bit_is_set(UCSR0A, RXC0);
        // FIXME
        // while ( (!(UCSR0A & _BV(RXC0))) && (wdGetCounter() < 60) );  //  "(wdGetCounter() < 60)" ==> Avoid a GPS deadlock if nothing is received
        *str++ = UDR0;  // CAUTION : No buffer overflow check - Fixed length !!
    } while ((*(str-2) != '\x0D') && (*(str-1) != '\x0A'));  // Wait a full line (/n/n on Linx GPS, and /r/n on Ublox)
    *(str-1) = '\x00';  // Mark the end of the string
}


/* Send a string on serial port (USART) */
void usartSendString(char* str) {
    while (*str != '\x00') {
        loop_until_bit_is_set(UCSR0A, UDRE0);  // while ( !( UCSRnA & (1<<UDREn)) );
        UDR0 = *str++;  // CAUTION : Possible buffer overflow -- String must be terminated by zero !!
    };
}

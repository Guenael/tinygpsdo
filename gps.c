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


#include "config.h"
#include "gps.h"

#include "twi.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usart.h"


/* Global definition for I2C GPS address */
static uint8_t gpsAddr;


/* Local/Private structrues for this processing code */
typedef struct gpsDataStruct {
    uint32_t  itow;
    uint16_t  year;
    uint8_t   month;
    uint8_t   day;
    uint8_t   hours;
    uint8_t   minutes;
    uint8_t   seconds;
    int32_t   nano;
    int32_t   lon;     // factor 1e-7
    int32_t   lat;     // factor 1e-7
    int32_t   height;  // in mm, factor 1e-3
    int32_t   speed;   // in mm/s, factor 1e-3
    uint8_t   numsat;
    int32_t   head;    // factor 1e-5, in deg.
    uint32_t  leapsec;
} gpsData;


typedef struct gpsStringStruct {
    // Extracted / Calculated outputs
    char posLocator[7];
    char posLat[8];
    char posLatDir[2];
    char posLon[9];
    char posLonDir[2];
    char day[3];
    char hours[3];
    char minutes[3];
    char seconds[3];
    char altitude[7];
    char speed[4];
    char head[4];
    char numsat[4];
    char seq[4];
} gpsString;


/* Goblal structrues & variables declaration */
static gpsData    lGpsData;    // Structure with all raw GPS data
static gpsString  lGpsString;  // Structure with all extracted/computed GPS data
// FIXME : global def, is static req. ?


static const uint8_t PROGMEM CFG_TP5[] = {
    0xB5, 0x62,             // Header
    0x06, 0x31,             // ID
    0x20, 0x00,             // Length
    0x00,                   // tpIdx
    0x01,                   // version
    0x00, 0x00,             // reserved1
    0x00, 0x00,             // antCableDelay
    0x00, 0x00,             // rfGroupDelay
    0x01, 0x00, 0x00, 0x00, // freqPeriod = 1 Hz
    0xA0, 0x86, 0x01, 0x00, // freqPeriodLock = 100kHz
    0x00, 0x00, 0x00, 0x00, // pulseLenRatio = 0% (disable)
    0x00, 0x00, 0x00, 0x80, // pulseLenRatioLock = 50%
    0x00, 0x00, 0x00, 0x00, // userConfigDelay
    0xEF, 0x08, 0x00, 0x00, // flags
    0xF7, 0xDA              // CRC
};


static const uint8_t PROGMEM CFG_RATE[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x0A, 0x00, 0x01, 0x00, 0x83, 0x36 //  100ms / 10cyc / GPS Time
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x4B, 0x00, 0x14, 0x00, 0x00, 0x00, 0x73, 0xC6 //   75ms   / 20cyc
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x82, 0x34 //  100ms   / 10cyc
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x88, 0x13, 0x01, 0x00, 0x00, 0x00, 0xB0, 0x47 //  5000ms  / 1cyc
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x60, 0xEA, 0x01, 0x00, 0x00, 0x00, 0x5F, 0x8A //  60000ms / 1cyc
};


static const uint8_t PROGMEM CFG_PRT[] = {
    0xB5, 0x62,             // Header
    0x06, 0x00,             // ID
    0x14, 0x00,             // Length
    0x00,                   // Port ID (0 = DDC)
    0x00,                   // Reserved1
    0x00, 0x00,             // txReady (0 = Disable)
    0x84, 0x00, 0x00, 0x00, // mode // Slave address (0x42)
    0x00, 0x00, 0x00, 0x00, // reserved2
    0x01, 0x00,             // inProtoMask  (1 = in UBX only)
    0x01, 0x00,             // outProtoMask (1 = out UBX only)
    0x00, 0x00,             // flags (0 = Extended TX timeout )
    0x00, 0x00,             // reserved5
    0xA0, 0x96              // CRC
};


static const uint8_t PROGMEM NAV_PVT[] = {
    0xB5, 0x62,             // Header
    0x01, 0x07,             // ID
    0x00, 0x00,             // Length
    0x08, 0x19              // CRC
};


static const uint8_t PROGMEM NAV_TIMEGPS[] = {
    0xB5, 0x62,             // Header
    0x01, 0x20,             // ID
    0x00, 0x00,             // Length
    0x21, 0x64              // CRC
};


void gpsInit(uint8_t addr) {
    /* GPS EXTINT : set pin 9 of PORT-PD5 for output*/
    DDRD |= _BV(DDD5);

    /* GPS EXTINT down for now (energy saver feature) */
    PORTD &= ~_BV(PORTD5);

    _delay_ms(100);

    /* Define I2C address */
    gpsAddr = addr;

    /* Note : I2C bus Init is done by i2c.c */
}


void gpsShutdown() {
}


void gpsSet_CFG_TP5() {
    // No gpsCrcUpdate -- CRC Hardcoded
    twi_writeToPgm(gpsAddr, CFG_TP5, sizeof(CFG_TP5), 1, 0);
    _delay_ms(100);
}


void gpsSet_CFG_RATE() {
    twi_writeToPgm(gpsAddr, CFG_RATE, sizeof(CFG_RATE), 1, 0);
    _delay_ms(100);
}


void gpsSet_CFG_PRT() {
    twi_writeToPgm(gpsAddr, CFG_PRT, sizeof(CFG_PRT), 1, 0);
    _delay_ms(100);
}


void gpsPoll_NAV_PVT() {
    twi_writeToPgm(gpsAddr, NAV_PVT, sizeof(NAV_PVT), 1, 0);
    _delay_ms(100);
}


void gpsPoll_NAV_TIMEGPS() {
    twi_writeToPgm(gpsAddr, NAV_TIMEGPS, sizeof(NAV_TIMEGPS), 1, 0);
    _delay_ms(100);
}


void gpsCrcUpdate(uint8_t *payload, uint8_t payloadSize) {
    /* https://www.ietf.org/rfc/rfc1145.txt */
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (uint8_t i=2; i<(payloadSize-2); i++) { // Skip the header for the checksum
        ck_a = ck_a + payload[i];
        ck_b = ck_b + ck_a;
    }

    payload[payloadSize-1] = ck_b;
    payload[payloadSize-2] = ck_a;
}


void gpsFlushBuffer() {
    uint16_t byteToRead;
    uint8_t  garbage;

    uint8_t  cmd = 0xFD;

    /* Point on the bytes available (Register Adressing) */
    twi_writeTo(gpsAddr, &cmd, 1, 1, 0);  // 0xFD & 0xFE for the 16 bit register
    _delay_ms(1);

    /* Read the effective buffered byte on the GPS uProcessor */
    if(!twi_readFrom(gpsAddr, (uint8_t*) &byteToRead, 2, 0))
        return;

    byteToRead = ((byteToRead>>8) | (byteToRead<<8));  // Little endian conversion

    while (byteToRead--)
        twi_readFrom(gpsAddr, &garbage, 1, 0);
}


void gpsGetPVT() {
    uint16_t byteToRead;
    uint8_t  *data;

    uint8_t  cmd = 0xFD;
    uint8_t  cmd2 = 0xFF;

    /* GPS locked flag (incremented at each valid line) */
    uint8_t  valid = 0;

    /* Free space for the raw input */
    data = malloc(100 * sizeof(uint8_t));

    while (!valid) {
        gpsFlushBuffer();

        gpsPoll_NAV_PVT();

        /* Point on the bytes available (Register Adressing) */
        twi_writeTo(gpsAddr, &cmd, 1, 1, 0);  // 0xFD & 0xFE for the 16 bit register
        _delay_ms(1);

        /* Read the effective buffered byte on the GPS uProcessor */
        if(!twi_readFrom(gpsAddr, (uint8_t*) &byteToRead, 2, 0))
            continue;

        byteToRead = ((byteToRead>>8) | (byteToRead<<8));  // Little endian conversion

        /* Free the buffer if unexpected size */
        if (byteToRead != 100)
            continue;

        /* Point on the stream buffer (Register Adressing) */
        twi_writeTo(gpsAddr, &cmd2, 1, 1, 0);  // 0xFF = StreamBuffer
        _delay_ms(1);

        /* Read all the buffer */
        for (uint8_t i=0; i<byteToRead; i++) {
            twi_readFrom(gpsAddr, &data[i], 1, 0);
        }

        /* Check the validity of the data */
        if ( (data[0] == 0xB5) &&
                (data[1] == 0x62) &&
                (data[2] == 0x01) &&
                (data[3] == 0x07) &&
                (data[4] == 0x5C) &&
                (data[5] == 0x00) &&
                ((data[17] & 0x07) == 0x07) ) {

            /* Extract usefull data */
            lGpsData.itow    = *(uint32_t*) &data[6];
            lGpsData.year    = *(uint16_t*) &data[10];
            lGpsData.month   =               data[12];
            lGpsData.day     =               data[13];
            lGpsData.hours   =               data[14];
            lGpsData.minutes =               data[15];
            lGpsData.seconds =               data[16];
            lGpsData.nano    = *(int32_t*)  &data[22];
            lGpsData.lon     = *(int32_t*)  &data[30];
            lGpsData.lat     = *(int32_t*)  &data[34];
            lGpsData.height  = *(int32_t*)  &data[38];
            lGpsData.speed   = *(int32_t*)  &data[66];
            lGpsData.numsat  =               data[29];
            lGpsData.head    = *(int32_t*)  &data[70];

            /* Set the stop condition */
            valid++;
        } else {
            _delay_ms(100);
        }
    }
    /* Raw data no longer needed */
    free(data);
}


void gpsGetTime() {
    uint16_t byteToRead;
    uint8_t  *data;
    uint8_t  valid = 0;

    uint8_t  cmd = 0xFD;
    uint8_t  cmd2 = 0xFF;

    /* Free space for the raw input */
    data = malloc(100 * sizeof(uint8_t));

    while (!valid) {  // 3 minute max to get a full sync
        gpsFlushBuffer();

        gpsPoll_NAV_TIMEGPS();

        /* Point on the bytes available (Register Adressing) */
        twi_writeTo(gpsAddr, &cmd, 1, 1, 0);  // 0xFD & 0xFE for the 16 bit register
        _delay_ms(1);

        /* Read the effective buffered byte on the GPS uProcessor */
        if(!twi_readFrom(gpsAddr, (uint8_t*) &byteToRead, 2, 0))
            continue;

        byteToRead = ((byteToRead>>8) | (byteToRead<<8));  // Little endian conversion

        /* Free the buffer if unexpected size */
        if (byteToRead != 24)
            continue;

        /* Point on the stream buffer (Register Adressing) */
        twi_writeTo(gpsAddr, &cmd2, 1, 1, 0);  // 0xFF = StreamBuffer
        _delay_ms(1);

        /* Read all the buffer */
        for (uint8_t i=0; i<byteToRead; i++) {
            twi_readFrom(gpsAddr, &data[i], 1, 0);
        }

        /* Check the validity of the data */
        if ( (data[0] == 0xB5) &&
                (data[1] == 0x62) &&
                (data[2] == 0x01) &&
                (data[3] == 0x20) &&
                (data[4] == 0x10) &&
                (data[5] == 0x00) &&
                ((data[17] & 0x04) == 0x04) ) {  // Check leapsecond flag validity

            /* Extract usefull data (ITOW & LeapSecodn)*/
            lGpsData.itow    = *(uint32_t*) &data[6];    // FIXME : assignment makes integer from pointer without a cast
            lGpsData.leapsec = 1000 * (uint32_t)*(int8_t*) &data[16]; // leap seconds in ms
            valid++;
        } else {
            _delay_ms(100);
        }
    }
    /* Raw data no longer needed */
    free(data);
}


void gpsExtractStrings() {
    /* Clean the structure */
    memset(&lGpsString, 0, sizeof(lGpsString));

    /* Latitude conversion deg -> ddmm.mm */
    uint32_t latInt  = labs(lGpsData.lat / 10000000);
    uint32_t latFrac = labs(lGpsData.lat) - (latInt * 10000000);

    sprintf(lGpsString.posLat, "%02lu", latInt);
    sprintf(lGpsString.posLat+2, "%04lu", 6 * latFrac / 1000);

    /* sprintf do no work with float on avr... dumb trick */
    lGpsString.posLat[6] = lGpsString.posLat[5];
    lGpsString.posLat[5] = lGpsString.posLat[4];
    lGpsString.posLat[4] = '.';

    /* Longitude conversion deg -> dddmm.mm */
    uint32_t lonInt  = labs(lGpsData.lon / 10000000);
    uint32_t lonFrac = labs(lGpsData.lon) - (lonInt * 10000000);

    sprintf(lGpsString.posLon, "%03lu", lonInt);
    sprintf(lGpsString.posLon+3, "%04lu", 6 * lonFrac / 10000);

    /* sprintf do no work with float on avr... dumb trick */
    lGpsString.posLon[7] = lGpsString.posLon[6];
    lGpsString.posLon[6] = lGpsString.posLon[5];
    lGpsString.posLon[5] = '.';

    /* N/S E/W flags */
    lGpsString.posLatDir[0] = (lGpsData.lat >= 0)? 'N' : 'S';
    lGpsString.posLonDir[0] = (lGpsData.lon >= 0)? 'E' : 'W';

    /* Basic other convesion */
    sprintf(lGpsString.day, "%02d", lGpsData.day);
    sprintf(lGpsString.hours, "%02d", lGpsData.hours);
    sprintf(lGpsString.minutes, "%02d", lGpsData.minutes);
    sprintf(lGpsString.seconds, "%02d", lGpsData.seconds);
    sprintf(lGpsString.altitude, "%06d", (int)lGpsData.height/1000);
    sprintf(lGpsString.speed, "%03d", (int)lGpsData.speed/1000);
    sprintf(lGpsString.head, "%03li", (long int)lGpsData.head/100000);
    sprintf(lGpsString.numsat, "%03d", lGpsData.numsat);
}


void gpsExtractLocator() {
    // latitude
    int32_t iLat = lGpsData.lat + 900000000;
    char lat_M = (char)(iLat/100000000) + 'A';
    char lat_D = (char)((iLat%100000000) / 10000000) + '0';
    char lat_m = (char)(((iLat%10000000) * 24 ) / 10000000) + 'A';

    // longitude
    int32_t iLong = lGpsData.lon + 1800000000;
    iLong /= 2;
    char long_M = (char)(iLong/100000000) + 'A';
    char long_D = (char)((iLong%100000000) / 10000000) + '0';
    char long_m = (char)(((iLong%10000000) * 24 ) / 10000000) + 'A';

    lGpsString.posLocator[0] = long_M;
    lGpsString.posLocator[1] = lat_M;
    lGpsString.posLocator[2] = long_D;
    lGpsString.posLocator[3] = lat_D;
    lGpsString.posLocator[4] = long_m;
    lGpsString.posLocator[5] = lat_m;
    lGpsString.posLocator[6] = 0x00;
}


void gpsTimeAling1M() {
    uint8_t sec = 59 - lGpsData.seconds;
    int32_t nano = 1000 - (lGpsData.nano/1000000);

    // FIXME - CHECK
    if(sec > 60) sec=60;
    if(nano > 1000) nano=1000;

    // _delay_ms function support only const... :(
    while (sec--)
        _delay_ms(1000);

    while (nano--)
        _delay_ms(1);
}


void gpsTimeAling2M() {
    uint8_t min = lGpsData.minutes;
    uint8_t sec = 59 - lGpsData.seconds;
    int32_t nano = 1000 - (lGpsData.nano/1000000);

    // FIXME - CHECK
    if(sec > 60) sec=60;
    if(nano > 1000) nano=1000;

    if (!(min % 2))
        _delay_ms(60000);

    while (sec--)
        _delay_ms(1000);

    while (nano--)
        _delay_ms(1);
}


void gpsTimeAling1Mb() {
    uint32_t milli = 60000 - ((lGpsData.itow - lGpsData.leapsec)% 60000);

    while (milli--)
        _delay_ms(1);
}


void gpsTimeAling2Mb() {
    uint32_t milli = 120000 - ((lGpsData.itow - lGpsData.leapsec)% 120000);

    while (milli--)
        _delay_ms(1);
}


char* getLocator() {
    return lGpsString.posLocator;
}


char* getLatitude() {
    return lGpsString.posLat;
}


char* getLongitude() {
    return lGpsString.posLon;
}


char* getLatitudeDir() {
    return lGpsString.posLatDir;
}


char* getLongitudeDir() {
    return lGpsString.posLonDir;
}


char* getAltitude() {
    return lGpsString.altitude;
}


char* getSpeed() {
    return lGpsString.speed;
}


char* getHead() {
    return lGpsString.head;
}


char* getDay() {
    return lGpsString.day;
}


char* getHours() {
    return lGpsString.hours;
}


char* getMinutes() {
    return lGpsString.minutes;
}


char* getSeconds() {
    return lGpsString.seconds;
}


char* getNumSat() {
    return lGpsString.numsat;
}


char* getSeq() {
    return lGpsString.seq;
}


/* DEBUG

#include "usart.h"
usartSendString("IN!!!\n");
char tmp[64];
sprintf(tmp, "%x-%x-%x-%x-%x-%x\n", data[0], data[1], data[2], data[3], data[4], data[5]);
usartSendString(tmp);
sprintf(tmp, "DEBUG leapSec = %d\n", data[16]);
usartSendString(tmp);
sprintf(tmp, "flag= %d\n", data[17]);
usartSendString(tmp);

#include "usart.h"
usartSendString("TEST\n");
char tmp[64];
sprintf(tmp, "DEBUG leapSec = %d\n", lGpsData.leapsec);
usartSendString(tmp);

*/
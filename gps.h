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


#pragma once


#include "config.h"


void gpsInit(uint8_t addr);
void gpsShutdown();
void gpsSet_CFG_TP5();
void gpsSet_CFG_RATE();
void gpsSet_CFG_PRT();
void gpsPoll_NAV_PVT();
void gpsPoll_NAV_TIMEGPS();
void gpsCrcUpdate(uint8_t *payload, uint8_t payloadSize);
void gpsGetPVT();
void gpsGetTime();
void gpsExtractLocator();
void gpsExtractStrings();
void gpsTimeAling1Mb();
void gpsTimeAling2Mb();
void gpsTimeAling1M();
void gpsTimeAling2M();
char* getLocator();
char* getLatitude();
char* getLongitude();
char* getLatitudeDir();
char* getLongitudeDir();
char* getAltitude();
char* getSpeed();
char* getHead();
char* getDay();
char* getHours();
char* getMinutes();
char* getSeconds();
char* getNumSat();
char* getSeq();

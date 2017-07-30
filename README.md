# TinyGPSDO Project -- GPS Disciplined oscillator, 10MHz

*Work in progress -- rev.A*

This driver was written for a dedicated board, able to deliver a reference signal of 10MHz (low noise & exact frequency). The board inclue 4 multiplexed 10MHz signal, 4 multiplexed sync signal and a SMA mixed output. The Sync signal is set on every odd minutes (could by modifier by editing the code). This minute sync signal was choosen to drive WSPR & PI4 protocols.
Schematic and PCB layout is available on CircuitMaker, using a credit card format size.

<h2>Keywords:</h2>
gpsdo, reference clock, oscillator, gps

<h3>Basically, this application :</h3>
- Configure the GPS output clock on 100kHz
- Adjust continually the digital PLL by:
  - Getting the 10bits values of the analog PLL
  - Calculating the value of the DAC
  - Pushitng the 16bits value of on the DAC
  
<h2>Hardware features:</h2>
- Compact design / Credit card size
- 10 MHz oscillator stabilized by GPS (GPSDO)
- 4 small footprint signal output (10MHz) and 1 SMA
- 4 small footprint sync output (minute odd) and mixed 1 SMA
- DC-DC Power supply within 10-15V, 0.5A max

<h2>Firmware feature:</h2>
- Run on a common ATMEGA328p (like Arduino)
- Drive GPS (uBlox Max-8)
- Drive 10MHz signal multiplexer 1 minute sync signal

<h3>Howto:</h3>
1. Install avr-gcc & avr-dude on your Unix disto
2. Use "make" to build the firmware
3. Use "make burn" to flash the firmware, with a "AVR Pocket Programmer"
4. Disconnect the programming cable & cut the power supply (the programmer could interfere with SPI port)

<h2>Links for this project:</h2>
- Hardware : https://circuitmaker.com/Projects/Details/Guenael-VA2GKA/TinyGPSDO-10M-RevA
- Twitter : https://twitter.com/guenael_jouchet
- Blog : https://www.guenael.ca/

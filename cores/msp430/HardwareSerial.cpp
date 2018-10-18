/*
  ************************************************************************
  *	HardwareSerial.cpp
  *
  *	Arduino core files for MSP430
  *		Copyright (c) 2012 Robert Wessels. All right reserved.
  *
  *
  ***********************************************************************
  Derived from:
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Energia.h"
#include "wiring_private.h"
#include "usci_isr_handler.h"

#include "HardwareSerial.h"

#define UCAxCTLW0     UCA0CTLW0 
#define UCAxCTL0      UCA0CTL0
#define UCAxCTL1      UCA0CTL1
#define UCAxBRW       UCA0BRW
#define UCAxBR0       UCA0BR0
#define UCAxBR1       UCA0BR1
#define UCAxMCTL      UCA0MCTL
#define UCAxMCTLW     UCA0MCTLW
#define UCAxMCTLW_L   UCA0MCTLW_L
#define UCAxMCTLW_H   UCA0MCTLW_H
#define UCAxSTAT      UCA0STAT
#define UCAxRXBUF     UCA0RXBUF
#define UCAxTXBUF     UCA0TXBUF
#define UCAxABCTL     UCA0ABCTL
#define UCAxIRCTL     UCA0IRCTL
#define UCAxIRTCTL    UCA0IRTCTL
#define UCAxIRRCTL    UCA0IRRCTL
#define UCAxICTL      UCA0ICTL
#define UCAxIE        UCA0IE
#define UCAxIFG       UCA0IFG
#define UCAxIV        UCA0IV

#define ARDUINO_SERIAL1_INPUT_BUFFER_SIZE 512
#define ARDUINO_SERIAL1_OUTPUT_BUFFER_SIZE 16

struct ring_buffer
{
	unsigned char* buffer;
	unsigned int size;
	volatile unsigned int head;
	volatile unsigned int tail;
};

static unsigned char rx_backend_buffer[ARDUINO_SERIAL1_INPUT_BUFFER_SIZE];
static unsigned char tx_backend_buffer[ARDUINO_SERIAL1_OUTPUT_BUFFER_SIZE];
static ring_buffer rx_buffer  =  { rx_backend_buffer, ARDUINO_SERIAL1_INPUT_BUFFER_SIZE, 0, 0 };
static ring_buffer tx_buffer  =  { tx_backend_buffer, ARDUINO_SERIAL1_OUTPUT_BUFFER_SIZE, 0, 0 };

inline void store_char(unsigned char c, ring_buffer *buffer)
{
	unsigned int i = (unsigned int)(buffer->head + 1) % buffer->size;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != buffer->tail) {
		buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}

void serialEvent() __attribute__((weak));
void serialEvent() {}

void serialEventRun(void)
{
	if (Serial1.available()) serialEvent();
}

// Public Methods //////////////////////////////////////////////////////////////
#define SMCLK F_CPU //SMCLK = F_CPU for now

void HardwareSerial::begin(unsigned long baud)
{
	unsigned int mod;
	unsigned long divider;
	unsigned char oversampling;
	
	/* Calling this dummy function prevents the linker
	 * from stripping the USCI interupt vectors.*/ 
	usci_isr_install();
	if (SMCLK/baud>=48) {                                                // requires SMCLK for oversampling
		oversampling = 1;
	}
	else {
		oversampling= 0;
	}

	divider=(SMCLK<<4)/baud;

	pinMode_int(rxPin, rxPinMode);
	pinMode_int(txPin, txPinMode);

	delay(10);

	*(&(UCAxCTL1) + uartOffset) = UCSWRST;
	*(&(UCAxCTL1) + uartOffset) |= UCSSEL_2;                                // SMCLK
	*(&(UCAxCTL0) + uartOffset) = 0;
	*(&(UCAxABCTL) + uartOffset) = 0;
	if(!oversampling) {
		mod = ((divider&0xF)+1)&0xE;                    // UCBRSx (bit 1-3)
		divider >>=4;
	} else {
		mod = ((divider&0xf8)+0x8)&0xf0;                // UCBRFx (bit 4-7)
		divider>>=8;
	}
	*(&(UCAxBR0) + uartOffset)= divider;
	*(&(UCAxBR1) + uartOffset) = divider>>8;
	*(&(UCAxMCTL) + uartOffset) = (unsigned char)(oversampling ? UCOS16:0) | mod;
	*(&(UCAxCTL1) + uartOffset) &= ~UCSWRST;
	*(&(UCAxIE) + uartOffset) |= UCRXIE;
}

void HardwareSerial::end()
{
	// wait for transmission of outgoing data
	while (_tx_buffer->head != _tx_buffer->tail);

	_rx_buffer->head = _rx_buffer->tail;
}

int HardwareSerial::available(void)
{
	return (unsigned int)(_rx_buffer->size + _rx_buffer->head - _rx_buffer->tail) % _rx_buffer->size;
}

int HardwareSerial::peek(void)
{
	if (_rx_buffer->head == _rx_buffer->tail) {
		return -1;
	} else {
		return _rx_buffer->buffer[_rx_buffer->tail];
	}
}

int HardwareSerial::read(void)
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (_rx_buffer->head == _rx_buffer->tail) {
		return -1;
	} else {
		unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
		_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % _rx_buffer->size;
		return c;
	}
}

void HardwareSerial::flush()
{
	while (_tx_buffer->head != _tx_buffer->tail);
}

size_t HardwareSerial::write(uint8_t c)
{
	unsigned int i = (_tx_buffer->head + 1) % _tx_buffer->size;
	
	// If the output buffer is full, there's nothing for it other than to
	// wait for the interrupt handler to empty it a bit
	// ???: return 0 here instead?
	while (i == _tx_buffer->tail);
	
	_tx_buffer->buffer[_tx_buffer->head] = c;
	_tx_buffer->head = i;

	*(&(UCAxIE) + uartOffset) |= UCTXIE;
	return 1;
}

HardwareSerial::operator bool() {
	return true;
}

void uart_rx_isr(uint8_t offset)
{
	ring_buffer *rx_buffer_ptr = &rx_buffer;
	unsigned char c = *(&(UCAxRXBUF) + offset);
	store_char(c, rx_buffer_ptr);
}

void uart_tx_isr(uint8_t offset)
{
	ring_buffer *tx_buffer_ptr = &tx_buffer;
	if (tx_buffer_ptr->head == tx_buffer_ptr->tail) {
		// Buffer empty, so disable interrupts
		*(&(UCAxIE) + offset) &= ~UCTXIE;
		*(&(UCAxIFG) + offset) |= UCTXIFG;    // Set Flag again
		return;
	}

	unsigned char c = tx_buffer_ptr->buffer[tx_buffer_ptr->tail];
	tx_buffer_ptr->tail = (tx_buffer_ptr->tail + 1) % tx_buffer_ptr->size;
	*(&(UCAxTXBUF) + offset) = c;
}
// Preinstantiate Objects //////////////////////////////////////////////////////

HardwareSerial Serial1(&rx_buffer, &tx_buffer, AUX_UART_MODULE_OFFSET, AUX_UARTRXD_SET_MODE, AUX_UARTTXD_SET_MODE, AUX_UARTRXD, AUX_UARTTXD);

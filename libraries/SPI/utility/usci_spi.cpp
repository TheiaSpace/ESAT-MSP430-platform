/**
 * File: usci_spi.c - msp430 USCI SPI implementation
 *
 * Copyright (c) 2012 by Rick Kimball <rick@kimballsoftware.com>
 * spi abstraction api for msp430
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 */

#include <msp430.h>
#include <stdint.h>
#include "spi_430.h"
#include <Energia.h>

#if defined(SPI_AVAILABLE)

#if defined(__MSP430_HAS_USCI_B0__) || defined(__MSP430_HAS_USCI_B1__) || defined(__MSP430_HAS_USCI__)

/**
 * USCI flags for various the SPI MODEs
 *
 * Note: The msp430 UCCKPL tracks the CPOL value. However,
 * the UCCKPH flag is inverted when compared to the CPHA
 * value described in Motorola documentation.
 */

#define SPI_MODE_0 (UCCKPH)			    /* CPOL=0 CPHA=0 */
#define SPI_MODE_1 (0)                 	/* CPOL=0 CPHA=1 */
#define SPI_MODE_2 (UCCKPL | UCCKPH)    /* CPOL=1 CPHA=0 */
#define SPI_MODE_3 (UCCKPL)			    /* CPOL=1 CPHA=1 */

#define SPI_MODE_MASK (UCCKPL | UCCKPH)

/* Calculate divisor to keep SPI clock close to 4MHz but never over */
#ifndef SPI_CLOCK_SPEED
#define SPI_CLOCK_SPEED 4000000L
#endif

#if F_CPU < 4000000L
#define SPI_CLOCK_DIV() 1 
#else
#define SPI_CLOCK_DIV() ((F_CPU / SPI_CLOCK_SPEED) + (F_CPU % SPI_CLOCK_SPEED == 0 ? 0:1))
#endif

#define SPI_CLOCK_DIV_DEFAULT (F_CPU / 4)

/* Pointers to USCI registers */
volatile unsigned char* UCBxCTL0;
volatile unsigned char* UCBxCTL1;
volatile unsigned char* UCBxBR0;
volatile unsigned char* UCBxBR1;
volatile unsigned char* UCBxSTAT;
const volatile unsigned char* UCBxRXBUF;
volatile unsigned char* UCBxTXBUF;
volatile unsigned char* UCBxIE;
volatile unsigned char* UCBxIFG;
volatile unsigned int* UCBxIV;
volatile unsigned int* UCBxI2COA;
volatile unsigned int* UCBxI2CSA;

/**
 * spi_initialize() - Configure USCI UCBx for SPI mode
 */
void spi_initialize(const uint8_t module)
{
    /* Set pointers to correct USCI registers based on which module is being used */
    UCBxCTL0 =     &UCB0CTL0;
    UCBxCTL1 =     &UCB0CTL1;
    UCBxBR0 =      &UCB0BR0;
    UCBxBR1 =      &UCB0BR1;
    UCBxSTAT =     &UCB0STAT;
    UCBxRXBUF =    &UCB0RXBUF;
    UCBxTXBUF =    &UCB0TXBUF;
    UCBxIE =       &UCB0IE;
    UCBxIFG =      &UCB0IFG;
    UCBxI2COA =    &UCB0I2COA;
    UCBxI2CSA =    &UCB0I2CSA;
#if defined(__MSP430_HAS_USCI_B1__)
    if(module == 1) {
        UCBxCTL0 =     &UCB1CTL0;
        UCBxCTL1 =     &UCB1CTL1;
        UCBxBR0 =      &UCB1BR0;
        UCBxBR1 =      &UCB1BR1;
        UCBxSTAT =     &UCB1STAT;
        UCBxRXBUF =    &UCB1RXBUF;
        UCBxTXBUF =    &UCB1TXBUF;
        UCBxIE =       &UCB1IE;
        UCBxIFG =      &UCB1IFG;
        UCBxI2COA =    &UCB1I2COA;
        UCBxI2CSA =    &UCB1I2CSA;

    }
#endif

	*UCBxCTL1 = UCSWRST | UCSSEL_2;      // Put USCI in reset mode, source USCI clock from SMCLK
	*UCBxCTL0 = SPI_MODE_0 | UCMSB | UCSYNC | UCMST;  // Use SPI MODE 0 - CPOL=0 CPHA=0

	pinMode(SCK, SPISCK_SET_MODE);
	pinMode(MOSI, SPIMOSI_SET_MODE);
	pinMode(MISO, SPIMISO_SET_MODE);

	*UCBxBR0 = SPI_CLOCK_DIV() & 0xFF;   // set initial speed to 4MHz
	*UCBxBR1 = (SPI_CLOCK_DIV() >> 8 ) & 0xFF;

	*UCBxCTL1 &= ~UCSWRST;			    // release USCI for operation
	//*UCBxIE |= UCRXIE;
}

/**
 * spi_disable() - put USCI into reset mode
 */
void spi_disable(void)
{
    *UCBxCTL1 |= UCSWRST;                // Put USCI in reset mode
}

/**
 * spi_send() - send a byte and recv response
 */
uint8_t spi_send(const uint8_t _data)
{
	*UCBxTXBUF = _data; // setting TXBUF clears the TXIFG flag
	while (*UCBxSTAT & UCBUSY)
		; // wait for SPI TX/RX to finish

	return *UCBxRXBUF; // reading clears RXIFG flag
}

/***SPI_MODE_0
 * spi_set_divisor() - set new clock divider for USCI
 *
 * USCI speed is based on the SMCLK divided by BR0 and BR1
 *
 */
void spi_set_divisor(const uint16_t clkdiv)
{
	*UCBxCTL1 |= UCSWRST;		// go into reset state
	*UCBxBR0 = clkdiv & 0xFF;
	*UCBxBR1 = (clkdiv >> 8 ) & 0xFF;
	*UCBxCTL1 &= ~UCSWRST;		// release for operation
}

/**
 * spi_set_bitorder(LSBFIRST=0 | MSBFIRST=1)
 */
void spi_set_bitorder(const uint8_t order)
{
    *UCBxCTL1 |= UCSWRST;        // go into reset state
    *UCBxCTL0 = (*UCBxCTL0 & ~UCMSB) | ((order == 1 /*MSBFIRST*/) ? UCMSB : 0); /* MSBFIRST = 1 */
    *UCBxCTL1 &= ~UCSWRST;       // release for operation
}

/**
 * spi_set_datamode() - mode 0 - 3
 */
void spi_set_datamode(const uint8_t mode)
{
    *UCBxCTL1 |= UCSWRST;        // go into reset state
    switch(mode) {
    case 0: /* SPI_MODE0 */
        *UCBxCTL0 = (*UCBxCTL0 & ~SPI_MODE_MASK) | SPI_MODE_0;
        break;
    case 1: /* SPI_MODE1 */
        *UCBxCTL0 = (*UCBxCTL0 & ~SPI_MODE_MASK) | SPI_MODE_1;
        break;
    case 2: /* SPI_MODE2 */
        *UCBxCTL0 = (*UCBxCTL0 & ~SPI_MODE_MASK) | SPI_MODE_2;
        break;
    case 4: /* SPI_MODE3 */
        *UCBxCTL0 = (*UCBxCTL0 & ~SPI_MODE_MASK) | SPI_MODE_3;
        break;
    default:
        break;
    }
    *UCBxCTL1 &= ~UCSWRST;       // release for operation
}
#else
    //#error "Error! This device doesn't have a USCI peripheral"
#endif

#endif

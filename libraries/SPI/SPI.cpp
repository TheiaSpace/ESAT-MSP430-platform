/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 * 2012-04-29 rick@kimballsoftware.com - added msp430 support.
 *
 */

#include <Energia.h>

#include "SPI.h"

SPIClass SPI;

#if defined(__MSP430_HAS_USCI_B1__)
uint8_t SPIClass::spiModule = 0;
#endif

void SPIClass::begin()
{
#if defined(__MSP430_HAS_USCI_B1__)
    spi_initialize(spiModule);
#else
    spi_initialize(0);
#endif
}

void SPIClass::end()
{
    spi_disable();
}

void SPIClass::setBitOrder(uint8_t bitOrder)
{
    spi_set_bitorder(bitOrder);
}

void SPIClass::setDataMode(uint8_t mode)
{
    spi_set_datamode(mode);
}

void SPIClass::setClockDivider(uint8_t rate)
{
    spi_set_divisor(rate);
}

void SPIClass::setModule(uint8_t module)
{
#if defined(__MSP430_HAS_USCI_B1__)
    spiModule = module;
#endif
}


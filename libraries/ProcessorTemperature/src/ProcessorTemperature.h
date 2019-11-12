/*
 * Copyright (C) 2019 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's Processor Temperature (MSP430)
 * library.
 *
 * Theia Space's Processor Temperature (MSP430) is free software: you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Theia Space's Processor Temperature (MSP430) library is distributed
 * in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's Processor Temperature (MSP430) library.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ProcessorTemperature_h
#define ProcessorTemperature_h

#include <Arduino.h>

// Access to the internal thermometer found in MSP430 microcontrollers.
// Use the global instance ProcessorTemperature.
class ProcessorTemperatureClass
{
  public:
    // Return the processor temperature in degrees Celsius.
    float read();

    // Return the raw (uncalibrated) reading of the processor
    // temperature analog-to-digital conversion.  This is an unsigned
    // integer number ranging from 0 to 4095.  The raw reading versus
    // actual temperature plot is a straight line.
    word readRaw();
};

// Global instance of the library.
extern ProcessorTemperatureClass ProcessorTemperature;

#endif /* ProcessorTemperature_h */

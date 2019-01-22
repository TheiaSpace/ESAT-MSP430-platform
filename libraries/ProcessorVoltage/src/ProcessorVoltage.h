/*
 * Copyright (C) 2019 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's Processor Voltage (MSP430)
 * library.
 *
 * Theia Space's Processor Voltage (MSP430) is free software: you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Theia Space's Processor Voltage (MSP430) library is distributed
 * in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's Processor Voltage (MSP430) library.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ProcessorVoltage_h
#define ProcessorVoltage_h

#include <Arduino.h>

// Access to the supply voltage of MSP430 microcontrollers.
// Use the global instance ProcessorVoltage.
class ProcessorVoltageClass
{
  public:
    // Return the processor supply voltage in volts.
    float read();
};

// Global instance of the library.
extern ProcessorVoltageClass ProcessorVoltage;

#endif /* ProcessorVoltage_h */

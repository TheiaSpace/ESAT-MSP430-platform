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

#include "ProcessorTemperature.h"

float ProcessorTemperatureClass::read()
{
  // Calling readRaw() returns the raw reading of the internal
  // thermometer.  This raw reading can be converted to degrees
  // Celsius with a simple linear interpolation between the raw
  // reading at 30 degress Celsius (stored as a 16-bit unsigned
  // integer number in memory address 0x1A1A) and the raw reading
  // reading at 85 degrees Celsius (stored as a 16-bit unsigned
  // integer number in memory address 0x1A1C).
  const float calibration30 = float(*((word*) 0x1A1A));
  const float calibration85 = float(*((word*) 0x1A1C));
  const float scale = (85.f - 30.f) / (calibration85 - calibration30);
  const float rawTemperatureReading = float(analogRead(TEMPSENSOR));
  return 30.f + scale * (rawTemperatureReading - calibration30);
}

word ProcessorTemperatureClass::readRaw()
{
  return analogRead(TEMPSENSOR);
}

// Global instance of the library.
ProcessorTemperatureClass ProcessorTemperature;

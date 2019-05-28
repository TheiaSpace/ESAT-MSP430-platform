/*
 * ProcessorTemperature example program version 1.0.0
 * Copyright (C) 2019 Theia Space, Universidad Politécnica de Madrid
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ProcessorTemperature.h>

// This program reads the processor temperature from the internal
// thermometer and writes it to the Serial interface.

// Initial configuration tasks:
// - Begin the Serial interface.
void setup()
{
  Serial.begin();
}

// Body of the main loop of the program:
// - Read the processor temperature from the internal thermometer.
// - Write the processor temperature to the Serial interface.
void loop()
{
  (void) Serial.print("Processor temperature: ");
  (void) Serial.print(ProcessorTemperature.read());
  (void) Serial.println(" degC");
  (void) Serial.print("Processor temperature (raw): ");
  (void) Serial.println(ProcessorTemperature.readRaw());
}

/*
 * RAMStatistics example program version 1.0.0
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

#include <RAMStatistics.h>

// This program prints RAM usage statistics.

// Initial configuration tasks:
// - Begin the Serial interface.
void setup()
{
  RAMStatistics.begin();
  Serial.begin(9600);
}

// Body of the main loop of the program:
// - Print RAM usage statistics.
void loop()
{
  (void) Serial.print(F("Total RAM size: "));
  (void) Serial.print(RAMStatistics.size(), DEC);
  (void) Serial.println(F(" bytes."));
  (void) Serial.print(F("Current stack usage: "));
  (void) Serial.print(RAMStatistics.currentStackUsage(), DEC);
  (void) Serial.println(F(" bytes."));
  (void) Serial.print(F("Current RAM usage: "));
  (void) Serial.print(RAMStatistics.currentUsage(), DEC);
  (void) Serial.println(F(" bytes."));
  (void) Serial.print(F("Current usage percentage: "));
  (void) Serial.print(RAMStatistics.currentUsagePercentage());
  (void) Serial.println(F(" %."));
  (void) Serial.print(F("Maximum dynamic memory usage: "));
  (void) Serial.print(RAMStatistics.maximumDynamicMemoryUsage(), DEC);
  (void) Serial.println(F(" bytes."));
  (void) Serial.print(F("Maximum stack usage: "));
  (void) Serial.print(RAMStatistics.maximumStackUsage(), DEC);
  (void) Serial.println(F(" bytes."));
  (void) Serial.print(F("Maximum usage. "));
  (void) Serial.print(RAMStatistics.maximumUsage(), DEC);
  (void) Serial.println(F(" bytes."));
  (void) Serial.print(F("Maximum RAM usage percentage: "));
  (void) Serial.print(RAMStatistics.maximumUsagePercentage());
  (void) Serial.println(F(" %."));
}

/*
 * RTC example program version 1.0.0
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

#include <RTC.h>

// This program reads the internal RTC and writes the current time
// to the Serial interface.

// Initial configuration tasks:
// - Begin the Serial interface.
// - Begin the RTC.
void setup()
{
  Serial.begin();
  RTC.begin();
  RtcTimestamp timestamp;
  timestamp.year = 1957;
  timestamp.month = 10;
  timestamp.dayOfMonth = 4;
  timestamp.dayOfWeek = 4;
  timestamp.hours = 19;
  timestamp.minutes = 28;
  timestamp.seconds = 34;
  RTC.write(timestamp);
}

// Body of the main loop of the program:
// - Read the current time.
// - Write the current time to the Serial interface.
// - Wait one second.
void loop()
{
  const RtcTimestamp timestamp = RTC.read();
  (void) Serial.print(timestamp.year);
  (void) Serial.print("-");
  (void) Serial.print(timestamp.month);
  (void) Serial.print("-");
  (void) Serial.print(timestamp.dayOfMonth);
  (void) Serial.print(" ");
  (void) Serial.print(timestamp.hours);
  (void) Serial.print(":");
  (void) Serial.print(timestamp.minutes);
  (void) Serial.print(":");
  (void) Serial.println(timestamp.seconds);
  delay(1000);
}

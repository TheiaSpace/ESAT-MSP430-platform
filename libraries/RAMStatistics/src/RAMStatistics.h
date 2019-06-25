/*
 * Copyright (C) 2019 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's RAM Statistics (MSP430)
 * library.
 *
 * Theia Space's RAM Statistics (MSP430) is free software: you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Theia Space's RAM Statistics (MSP430) library is distributed
 * in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's RAM Statistics (MSP430) library.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RAMStatistics_h
#define RAMStatistics_h

#include <Arduino.h>

// Access to the RAM statistics of the MSP430F5529 microcontroller.
// Use the global instance RAMStatistics.
class RAMStatisticsClass
{
  public:
    // Mark the currently-unused memory with a bit pattern
    // so that the high water mark of stack usage can be detected.
    void begin(byte bitPattern = 0);

    // Return the current stack usage in bytes.
    unsigned long currentStackUsage() const;

    // Return the current RAM usage in bytes.
    unsigned long currentUsage() const;

    // Return the current RAM usage percentage.
    float currentUsagePercentage() const;

    // Return the maximum dynamic memory usage in bytes.
    // This includes the RAM used for static data and the heap.
    unsigned long maximumDynamicMemoryUsage() const;

    // Return the maximum stack usage in bytes.
    unsigned long maximumStackUsage() const;

    // Return the maximum RAM usage in bytes.
    unsigned long maximumUsage() const;

    // Return the maximum RAM usage percentage.
    float maximumUsagePercentage() const;

    // Return the total RAM size.
    unsigned long size() const;

  private:
    // RAM starts at this memory address.
    static const unsigned long ramStart = 0x2400;

    // RAM ends at this memory address.
    static const unsigned long ramEnd = 0x43FF;

    // Bit pattern used for maximum stack usage detection.
    byte pattern;
};

// Global instance of the library.
extern RAMStatisticsClass RAMStatistics;

#endif /* RAMStatistics_h */

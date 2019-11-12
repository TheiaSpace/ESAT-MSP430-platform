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

#include "RAMStatistics.h"

extern "C"
{
  void* sbrk(intptr_t increment);
}

void RAMStatisticsClass::begin(const byte bitPattern)
{
  // We mark all unused memory (from the end of the heap to the end
  // of the stack) with a bit pattern so that we can find the high
  // water mark of stack usage later on.
  void* const heapEnd = sbrk(0);
  const intptr_t guardDistance = 5;
  const intptr_t stackEnd = intptr_t(&bitPattern) - guardDistance;
  const intptr_t size = stackEnd - intptr_t(heapEnd);
  if (size > 0)
  {
    (void) memset(heapEnd, bitPattern, size);
  }
  pattern = bitPattern;
}

unsigned long RAMStatisticsClass::currentStackUsage() const
{
  byte stackEnd;
  return ramEnd + 1 - intptr_t(&stackEnd);
}

unsigned long RAMStatisticsClass::currentUsage() const
{
  return maximumDynamicMemoryUsage() + currentStackUsage();
}

float RAMStatisticsClass::currentUsagePercentage() const
{
  return 100 * float(currentUsage()) / size();
}

unsigned long RAMStatisticsClass::maximumDynamicMemoryUsage() const
{
  return (word) intptr_t(sbrk(0)) - ramStart;
}

unsigned long RAMStatisticsClass::maximumStackUsage() const
{
  // We can't know for sure the maximum stack size used,
  // but we can estimate it.
  // The stack starts at the end of the RAM and goes down from there
  // towards the end of the dynamic memory.
  // As memory is set to a pattern, we can get a bound of the
  // deepest point reached by the stack by finding the first
  // memory byte above the end of the dynamic memory that doesn't
  // match the pattern.
  byte* stackEnd = (byte*) sbrk(0);
  while ((((unsigned long) stackEnd) <= ramEnd) && (*stackEnd == pattern))
  {
    stackEnd = stackEnd + 1;
  }
  return ramEnd + 1 - ((unsigned long) stackEnd);
}

unsigned long RAMStatisticsClass::maximumUsage() const
{
  return maximumDynamicMemoryUsage() + maximumStackUsage();
}

float RAMStatisticsClass::maximumUsagePercentage() const
{
  return 100 * float(maximumUsage()) / float(size());
}

unsigned long RAMStatisticsClass::size() const
{
  return ramEnd + 1 - ramStart;
}

// Global instance of the library.
RAMStatisticsClass RAMStatistics;

/*
 * Copyright (C) 2019 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's RTC (MSP430) library.
 *
 * Theia Space's RTC (MSP430) is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's RTC (MSP430) library is distributed in the hope that
 * it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's RTC (MSP430) library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "RTC.h"

// This library provides read and write access to the real-time clock
// time registers, but with a small level of indirection and
// complexity:
// * Naive direct reads of the real-time clock time registers work
//   most of the time, but they are not atomic (there are several time
//   registers: one for the seconds, one for the minutes, one for the
//   hour...), so they can result in a mangled timestamp if the clock
//   ticks in the middle of a read.  We could poll the clock-is-ready
//   bit, but naive polling will fail too with wrong timing.  There
//   are several strategies for getting reliable reads; we opted for
//   double-buffering the time, which is a well-known method for
//   achieving this end.  The buffered time is updated once every
//   clock tick.  Also, during time reads, we postpone possible
//   updates until the read finishes, so the user always sees a
//   correct time.
// * Setting the time is simpler: to avoid clock ticks while in the
//   middle of the process, we just stop the clock while we
//   change the time registers and update the buffered time, and
//   then start the clock back again.

void RTCClass::begin()
{
  RTCCTL01 = RTCMODE; // Also clears RTCHOLD to enable RTC.
  enableTickInterrupt();
}

void RTCClass::disable()
{
  RTCCTL01 = RTCHOLD; // Sets whole RTCCTL01 to reset state.
}

void RTCClass::disableCalibrationOutput()
{
  // Pin peripheral selection shared with ESP0 GPIO0 on OBC.
  bitClear(P2DIR, 6);
  bitClear(P2SEL, 6);
  RTCCTL23_H = 0;
}

void RTCClass::disableTickInterrupt()
{
  // The clock tick interrupt is disabled when the RTCRDYIE bit is clear.
  RTCCTL01 = RTCCTL01 & (~RTCRDYIE);
}

void RTCClass::enableCalibrationOutput(const RTCClass::CalibrationOutputFrequency frequency)
{
  // Pin peripheral selection shared with ESP0 GPIO0 on OBC.
  bitSet(P2DIR, 6);
  bitSet(P2SEL, 6);
  switch (frequency)
  {
	case CALIBRATION_OUTPUT_512HZ:
	{
	RTCCTL23_H = 0x01;
	return;
	}
	case CALIBRATION_OUTPUT_256HZ:
	{
	RTCCTL23_H = 0x02;
	return;
	}
	case CALIBRATION_OUTPUT_1HZ:
	{
	RTCCTL23_H = 0x03;
	return;
	}
	default:
	{
	disableCalibrationOutput();
	return;
	}
  }
  return;
}

void RTCClass::enableTickInterrupt()
{
  // The clock tick interrupt is enabled when the RTCRDYIE bit is set.
  RTCCTL01 = RTCCTL01 | RTCRDYIE;
}

RtcTimestamp RTCClass::read()
{
  const boolean tickInterruptWasEnabled = tickInterruptEnabled();
  // The tick interrupt must be stopped while we copy the current time
  // in order to avoid it from changing and leaving us with a
  // potentially incorrect time.
  disableTickInterrupt();
  // We can safely copy the current time when the clock tick interrupt
  // is disabled.
  RtcTimestamp readTime;
  readTime.seconds = currentTime.seconds;
  readTime.minutes = currentTime.minutes;
  readTime.hours = currentTime.hours;
  readTime.dayOfWeek = currentTime.dayOfWeek;
  readTime.dayOfMonth = currentTime.dayOfMonth;
  readTime.month = currentTime.month;
  readTime.year = currentTime.year;
  // The tick interrupt must be enabled again if it was enabled
  // before.
  if (tickInterruptWasEnabled)
  {
    enableTickInterrupt();
  }
  return readTime;
}

boolean RTCClass::running() const
{
  // The clock runs when the RTCHOLD bit is clear.
  if (RTCCTL1 & (~RTCHOLD))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RTCClass::setBCDMode()
{
  // The clock is in BCD mode when the RTCBCD bit is set.
  RTCCTL01 = RTCCTL01 | RTCBCD;
}

void RTCClass::setBinaryMode()
{
  // The clock is in binary mode when the RTCBCD bit is clear.
  RTCCTL01 = RTCCTL01 & (~RTCBCD);
}

void RTCClass::setCalibration(const int8_t calibrationValue)
{
  // This way allows you to set the desired calibration value directly.
  // If value is negative, a decrement is desired (if it is zero,
  // calibration is disabled).
  if (calibrationValue <= 0)
  {
    // Change to positive, disable positive sign bit, mask and divide
    // by two because each step is -2 ppm.
    RTCCTL23_L =
      (RTCCAL5_L
       | RTCCAL4_L
       | RTCCAL3_L
       | RTCCAL2_L
       | RTCCAL1_L
       | RTCCAL0_L)
      & (((uint8_t) (-calibrationValue)) / 2);
  }
  else
  {
    // Enable positive bit, mask and divide by 4 because each step is +4 ppm.
    RTCCTL23_L =
      RTCCALS_L | ((RTCCAL5_L
                    | RTCCAL4_L
                    | RTCCAL3_L
                    | RTCCAL2_L
                    | RTCCAL1_L
                    | RTCCAL0_L)
                   & (((uint8_t) calibrationValue) / 4));
  }
}

void RTCClass::start()
{
  // The clock runs when the RTCHOLD bit is clear.
  RTCCTL01 = RTCCTL01 & (~RTCHOLD);
}

void RTCClass::stop()
{
  // The clock halts when the RTCHOLD bit is set.
  RTCCTL01 = RTCCTL01 | RTCHOLD;
}

boolean RTCClass::tickInterruptEnabled()
{
  // The clock tick interrupt is enabled when the RTCRDYIE bit is set.
  if (RTCCTL01 & RTCRDYIE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RTCClass::updateReading()
{
  currentTime.seconds = RTCSEC;
  currentTime.minutes = RTCMIN;
  currentTime.hours = RTCHOUR;
  currentTime.dayOfWeek = RTCDOW;
  currentTime.dayOfMonth = RTCDAY;
  currentTime.month = RTCMON;
  currentTime.year = (RTCYEAR & 0x0FFF);
}

void RTCClass::write(const RtcTimestamp timeToSet)
{
  const boolean clockWasRunning = running();
  // The clock must be stopped while we set the time in order to avoid
  // it from ticking and leaving us with a potentially incorrect time.
  stop();
  // We can safely set the time when the clock is stopped.
  RTCYEAR = timeToSet.year & 0x0FFF;
  RTCMON = timeToSet.month;
  RTCDAY = timeToSet.dayOfMonth;
  RTCDOW = timeToSet.dayOfWeek;
  RTCHOUR = timeToSet.hours;
  RTCMIN = timeToSet.minutes;
  RTCSEC = timeToSet.seconds;
  // Update the real-time clock reading with the time we just set.
  updateReading();
  // Finally, don't forget to start the clock if it was running
  // before.
  if (clockWasRunning)
  {
    start();
  }
}

extern "C"
{
  // Real-time clock interrupt handler.
  __attribute__((interrupt(RTC_VECTOR)))
  void RTC_ISR(void)
  {
    // We have a clock tick interrupt when the RTCIV_RTCRDYIFG bit is
    // set.  We must update the time reading buffer when the clock
    // ticks.
    if (RTCIV & RTCIV_RTCRDYIFG)
    {
      RTC.updateReading();
    }
  }
}

RTCClass RTC;

/*
  ************************************************************************
  *	RTC.cpp
  * 
  *
  *
  ***********************************************************************
  
  Created in 2019 by Theia Space.
*/

//Only RTC C by now

#include "RTC.h"
#include <Arduino.h>

void RealTimeClock::begin()
{
  UCSCTL6_L &= (~XCAP_3); // Sets minimum capacitance for XT1 RTC crystal oscillator.
  RTCCTL01 = RTCMODE; // Also clears RTCHOLD to enable RTC.
  enableTickInterrupt();
}

void RealTimeClock::disableTickInterrupt()
{
  // The clock tick interrupt is disabled when the RTCRDYIE bit is clear.
  RTCCTL01 = RTCCTL01 & (~RTCRDYIE);
}

void RealTimeClock::enableTickInterrupt()
{
  // The clock tick interrupt is enabled when the RTCRDYIE bit is set.
  RTCCTL01 = RTCCTL01 | RTCRDYIE;
}

void RealTimeClock::start()
{
  // The clock runs when the RTCHOLD bit is clear.
  RTCCTL01 = RTCCTL01 & (~RTCHOLD);
}

void RealTimeClock::stop()
{
  // The clock halts when the RTCHOLD bit is set.
  RTCCTL01 = RTCCTL01 | RTCHOLD;
}

boolean RealTimeClock::tickInterruptEnabled()
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

void RealTimeClock::updateReading()
{
  currentTime.seconds = RTCSEC;
  currentTime.minutes = RTCMIN;
  currentTime.hours = RTCHOUR;
  currentTime.dayOfWeek = RTCDOW;
  currentTime.dayOfMonth = RTCDAY;
  currentTime.month = RTCMON;
  currentTime.year = (RTCYEAR & 0x0FFF);
}

void RealTimeClock::write(RtcTimestamp timeToSet)
{
  const boolean clockWasRunning = !!(status());
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

RtcTimestamp RealTimeClock::read(void)
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

RtcTimestamp RealTimeClock::readUnreliable(void)
{
  RtcTimestamp readTime;
  readTime.seconds = RTCSEC;
  readTime.minutes = RTCMIN;
  readTime.hours = RTCHOUR;
  readTime.dayOfWeek = RTCDOW;
  readTime.dayOfMonth = RTCDAY;
  readTime.month = RTCMON;
  readTime.year = (RTCYEAR & 0x0FFF);
  return readTime;
}

uint8_t RealTimeClock::available()
{
  if (RTCCTL01 & RTCRDY)
  {
    return RTC_SAFE_TO_READ;
  }
  return RTC_UNSAFE_TO_READ;
}

uint8_t RealTimeClock::status()
{
  if (RTCCTL01 & RTCHOLD)
  {
    return 0;
  }
  return 1;
}

void RealTimeClock::disable()
{
  RTCCTL01 = RTCHOLD; // Sets whole RTCCTL01 to reset state.
  return;
}

void RealTimeClock::setMode(uint8_t mode)
{
  switch (mode) // Only changes if proper value is set.
  {
    case RTC_BCD:
      RTCCTL01 |= RTCBCD;
      return;
    case RTC_DECIMAL:
      RTCCTL01 &= (~RTCBCD);
      return;
    default:
      return;
  }
  return;
}

void RealTimeClock::setCalibration(int8_t calibrationValue)
{
  // This way allows you to set the desired calibration value directly.
  // If value is negative, a decrement is desired (if it is zero,
  // calibration is disabled).
  if (calibrationValue <= 0)
  {
    // Change to positive, disable positive sign bit, mask and divide
    // by two because each step is -2 ppm.
    calibrationValue *= -1;
    RTCCTL23_L =
      (RTCCAL5_L
       | RTCCAL4_L
       | RTCCAL3_L
       | RTCCAL2_L
       | RTCCAL1_L
       | RTCCAL0_L)
      & (((uint8_t) calibrationValue) / 2);
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
  return;
}

void RealTimeClock::setCalibrationOutput(uint8_t frequency)
{
  if ((frequency == RTC_CALIBRATION_SIGNAL_512HZ)
      || (frequency == RTC_CALIBRATION_SIGNAL_256HZ)
      || (frequency == RTC_CALIBRATION_SIGNAL_1HZ))
  {
    // Pin peripheral selection
    // shared with ESP0 GPIO0 on OBC.
    P2DIR |= (1 << 6);
    P2SEL |= (1 << 6);
    RTCCTL23_H = frequency;
    return;
  }
  // Pin peripheral disable
  // Shared with ESP0 GPIO0 on OBC.
  P2SEL &= ~(1<<6);
  P2DIR &= ~(1<<6);
  RTCCTL23_H = 0;
  return;
}

extern "C"
{
  // Real-time clock interrupt handler.
  __attribute__((interrupt(RTC_VECTOR)))
  void RTC_ISR(void)
  {
    if ((RTCIV & RTCIV_RTCRDYIFG) != 0)
    {
      RTC.updateReading();
    }
  }
}

RealTimeClock RTC;

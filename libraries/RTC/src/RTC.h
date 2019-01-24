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

#ifndef RTC_h
#define RTC_h

#include <Arduino.h>

// Timestamp used by the real-time clock.
// It can hold either binary numbers or BCD numbers.
struct RtcTimestamp
{
  uint16_t year;
  uint8_t month;
  uint8_t dayOfMonth;
  uint8_t dayOfWeek;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
};

// Driver for the real-time clock found in MSP430 processors.
// Use the global instance RTC.
class RTCClass
{
  public:
    // Calibration output frequency used by RTC.enableCalibrationOutput()
    enum CalibrationOutputFrequency
    {
      CALIBRATION_OUTPUT_512HZ = 1,
      CALIBRATION_OUTPUT_256HZ = 2,
      CALIBRATION_OUTPUT_1HZ = 3
    };

    // Start the clock.
    // The clock runs in binary mode by default; use RTC.setBCDMode()
    // if you need it to run in BCD mode.
    void begin();

    // Turns RTC off.
    void disable();

    // Disable the calibration output signal.
    void disableCalibrationOutput();

    // Enable the calibration output signal at the given frequency
    // on pin P2.6 (RTCCLK).
    void enableCalibrationOutput(CalibrationOutputFrequency frequency);

    // Return the current timestamp.
    RtcTimestamp read();

    // Set the clock in BCD mode.
    // Time will be encoded as BCD numbers.
    void setBCDMode();

    // Set the clock in binary mode.
    // Time will be encoded as binary numbers.
    // This is the default mode.
    void setBinaryMode();

    // Allows user to trim RTC frequency.
    void setCalibration(int8_t calibrationValue);

    // Update the reading of the real-time clock driver in
    // response to real-time clock tick interrupts.  User code
    // shouldn't care about this.
    void updateReading();

    // Writes timestamp into RTC registers.
    void write(RtcTimestamp timeToSet);

  private:
    // Current timestamp.
    // Updated on calls to updateReading(), which happen automatically
    // every time the real-time clock ticks.
    // We need to double-buffer the current timestamp like this
    // instead of reading the real-time clock time registers directly
    // in order to avoid problems when the time registers change in
    // the middle of a read operation.
    volatile RtcTimestamp currentTime;

    // Disable the clock tick interrupt.
    void disableTickInterrupt();

    // Enable the clock tick interrupt.
    void enableTickInterrupt();

    // Return true if the real-time clock is running; otherwise return
    // false.
    boolean running();

    // Start the clock.
    void start();

    // Stop the clock.
    void stop();

    // Return true if clock tick interrupt is enabled; otherwise
    // return false.
    boolean tickInterruptEnabled();
};

// Global instance of RTCClass.
extern RTCClass RTC;

#endif /* RTC_h */

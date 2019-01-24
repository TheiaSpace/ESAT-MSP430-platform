/*
  ************************************************************************
  *	RTC.h
  * 
  *
  *
  ***********************************************************************
  
  *Created in 2019 by Theia Space.
*/

#ifndef RTC_h
#define RTC_h

#include <Energia.h>
#include <inttypes.h>
#include <msp430.h>

#define DEFAULT_YEAR    2019
#define DEFAULT_MONTH   1
#define DEFAULT_DAY     1
#define DEFAULT_DOW     0
#define DEFAULT_HOURS   0
#define DEFAULT_MINUTES 0
#define DEFAULT_SECONDS 0

#define RTC_DECIMAL 0
#define RTC_BCD     1

#define RTC_UNSAFE_TO_READ 0
#define RTC_SAFE_TO_READ   1

#define RTC_CALIBRATION_SIGNAL_DISABLED 0
#define RTC_CALIBRATION_SIGNAL_512HZ    1
#define RTC_CALIBRATION_SIGNAL_256HZ    2
#define RTC_CALIBRATION_SIGNAL_1HZ      3

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

class RealTimeClock
{
  public:
    // Enables RTC.
    void begin();

    // Writes timestamp into RTC registers.
    void write(RtcTimestamp timeToSet);

    // Retrieves timestamp from RTC registers, checking if no
    // registers are changing.
    RtcTimestamp read();

    // Checks if RTC can be read safely.
    uint8_t available();

    // Turns RTC off.
    void disable();

    // Sets RTC into BCD or decimal mode.
    void setMode(uint8_t mode);

    // Allows user to trim RTC frequency.
    void setCalibration(int8_t calibrationValue);

    // Enables or disables calibration output waverform.
    void setCalibrationOutput(uint8_t frequency);

    // Update the reading of the real-time clock driver in
    // response to real-time clock tick interrupts.  User code
    // shouldn't care about this.
    void updateReading();

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

extern RealTimeClock RTC;

#endif

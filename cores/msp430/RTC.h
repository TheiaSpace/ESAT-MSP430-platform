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

    // Retrieves tiemstamp without checking if the registers are changing.
    RtcTimestamp readUnreliable();

    // Checks if RTC can be read safely.
    uint8_t available();

    // Checks if RTC is enabled.
    uint8_t status();

    // Turns RTC off.
    void disable();

    // Sets RTC into BCD or decimal mode.
    void setMode(uint8_t mode);

    // Allows user to trim RTC frequency.
    void setCalibration(int8_t calibrationValue);

    // Enables or disables calibration output waverform.
    void setCalibrationOutput(uint8_t frequency);
};

extern RealTimeClock RTC;

#endif

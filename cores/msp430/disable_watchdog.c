#include <Arduino.h>

__attribute__((naked, section(".crt_0001_disable_watchdog")))
void __disable_watchdog()
{
  disableWatchDog();
}

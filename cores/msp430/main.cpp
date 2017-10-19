#include <Energia.h>

__attribute__((naked, section(".crt_0001_disable_watchdog")))
void __disable_watchdog()
{
	disableWatchDog();
}

int main(void)
{
	init();

	setup();

	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}

	return 0;
}



#include <cassert>
#include "gpio/gpio_pin.h"
#include "timer/timer.h"

// Make it use C linkage so we can call it from sdk_unit_test.
extern "C" void gpio_buzzer(int);
extern "C" void gpio_beep(void);

#define GPIO_BUZZER_PIN	GPIO_MAKE_PIN(5, 12)

//! This test simply blinks the debug LED at a 1 Hz rate.
void gpio_buzzer(int on)
{
    GpioOutput buzzer(GPIO_BUZZER_PIN);

    buzzer = on;
}

void gpio_beep(void)
{
    GpioOutput buzzer(GPIO_BUZZER_PIN);

    buzzer = !buzzer;

    buzzer = true;
    hal_delay_us(500000);
    buzzer = false;
}

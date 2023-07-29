#include <stdbool.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#define LED_PIN GPIO_NUM_2

int main (void)
{
    ulp_riscv_gpio_init(LED_PIN);
    ulp_riscv_gpio_output_enable(LED_PIN);

    bool led_on = 0;
    while(1) {
        led_on = !led_on;
        ulp_riscv_gpio_output_level(LED_PIN, led_on);

        ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_MS);
    }

    return 0;
}

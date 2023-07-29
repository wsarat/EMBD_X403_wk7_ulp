#include <stdbool.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#include "ulp_riscv_print.h"
#include "ulp_riscv_uart_ulp_core.h"
#include "sdkconfig.h" // for UART baudrate

static ulp_riscv_uart_t s_print_uart;

#define LED_PIN     GPIO_NUM_2
#define UART_TX_PIN GPIO_NUM_14

int main (void)
{
    // LED
    bool led_on = 0;
    ulp_riscv_gpio_init(LED_PIN);
    ulp_riscv_gpio_output_enable(LED_PIN);

    // UART
    ulp_riscv_uart_cfg_t cfg = {
        .tx_pin = UART_TX_PIN,
    };

    ulp_riscv_uart_init(&s_print_uart, &cfg);
    ulp_riscv_print_install((putc_fn_t)ulp_riscv_uart_putc, &s_print_uart);
    
    int count = 0;
    char buf[32];
    while(1) {
        led_on = !led_on;
        ulp_riscv_gpio_output_level(LED_PIN, led_on);

        ulp_riscv_print_str("ULP-RISCV count: ");
        ulp_riscv_print_hex(count++);
        ulp_riscv_print_str("\n");

        ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_MS);
    }

    return 0;
}

#include <stdbool.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#include "ulp_riscv_print.h"
#include "ulp_riscv_uart_ulp_core.h"
#include "sdkconfig.h" // for UART baudrate

#include "ulp_riscv_i2c_ulp_core.h"

static ulp_riscv_uart_t s_print_uart;

#define LED_PIN     GPIO_NUM_2
#define UART_TX_PIN GPIO_NUM_14

#define MPU6050_ADDR    0x68
#define PWR_MGMT_1     0x6B
#define WHO_AM_I        117

void uart_print(char* key, uint8_t value) {
    ulp_riscv_print_str(key);
    ulp_riscv_print_str(" -> ");
    ulp_riscv_print_hex(value);
    ulp_riscv_print_str("\n");
}

void read_i2c(int8_t i2c_addr, int8_t i2c_reg_addr, int8_t *data)
{
    /* Set slave register address to the control register */
    ulp_riscv_i2c_master_set_slave_addr(i2c_addr);
    ulp_riscv_i2c_master_set_slave_reg_addr(i2c_reg_addr);
    ulp_riscv_i2c_master_read_from_device(data, 1);
}

void write_i2c(int8_t i2c_addr, int8_t i2c_reg_addr, int8_t *data)
{
    /* Set slave register address to the control register */
    ulp_riscv_i2c_master_set_slave_addr(i2c_addr);
    ulp_riscv_i2c_master_set_slave_reg_addr(i2c_reg_addr);
    ulp_riscv_i2c_master_write_to_device(data, 1);
}

void scan_i2c() {
    int ret;
    uint8_t data;

    ulp_riscv_print_str("i2c scan start.\n");
    
    for (uint8_t addr=0; addr<0x7f; addr++) {     
        data = 0;
        read_i2c(addr, 0, &data);
        if (data > 0) {
            uart_print("Found i2c at Address", addr);
            uart_print("Read", data);
            break;
        }
    }    
    ulp_riscv_print_str("i2c scan end.\n\n");           
}

void mpu6050_init() {
    ulp_riscv_print_str("mpu6050 init (write PWR_MGMT_1 = 0)\n");
    //write_i2c(MPU6050_ADDR, PWR_MGMT_1, 0);
    uint8_t data = 0;
    write_i2c(MPU6050_ADDR, PWR_MGMT_1, &data);
    write_i2c(0x6A, PWR_MGMT_1, &data);
}

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
    
    char buf[32];
    int8_t data;

    ulp_riscv_delay_cycles(3000 * ULP_RISCV_CYCLES_PER_MS);
    mpu6050_init();
    ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_MS);

    scan_i2c();

    ulp_riscv_print_str("mpu6050 at 0x68\n");
    data = 0;
    read_i2c(MPU6050_ADDR, WHO_AM_I, &data);
    uart_print("WHO_AM_I", data);
    ulp_riscv_print_str("\n");

    ulp_riscv_print_str("mpu6050 at 0x6a\n");
    data = 0;
    read_i2c(0x6a, WHO_AM_I, &data);
    uart_print("WHO_AM_I", data);
    ulp_riscv_print_str("\n");

    while(1) {
        led_on = !led_on;
        ulp_riscv_gpio_output_level(LED_PIN, led_on);

        ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_MS);
    }

    return 0;
}

#include <stdio.h>
#include <esp_err.h>
#include "esp_sleep.h"

#include "ulp_riscv.h"
#include "ulp_riscv_i2c.h"
#include "ulp_riscv_lock.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "ulp_main.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

#define UART_PORT   UART_NUM_2
#define UART_RX_PIN  GPIO_NUM_4     // hard wired to ulp uart tx  
#define UART_TX_PIN  GPIO_NUM_5     // NC 
#define UART_BUF_SIZE    128

static void init_ulp_program(void)
{
    esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

    ulp_set_wakeup_period(0, 20000);
    err = ulp_riscv_run();
    ESP_ERROR_CHECK(err);
}

static void uart_monitor(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = 9600, // ulp serial baudrate
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, -1, -1));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_PORT, data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        // Write data back to the UART
        data[len] = 0x0;
        if (len)
            printf("%s", data);
    }
}

void rtc_i2c_init(uint8_t sda_pin, u_int8_t scl_pin) {
    printf("Initializing RTC I2C ...\n");

    ulp_riscv_i2c_cfg_t i2c_cfg = {
        ULP_RISCV_I2C_DEFAULT_GPIO_CONFIG()
        ULP_RISCV_I2C_FAST_MODE_CONFIG()
    };
    i2c_cfg.i2c_pin_cfg.sda_io_num = sda_pin;
    i2c_cfg.i2c_pin_cfg.scl_io_num = scl_pin;

    printf("RTC i2c SDA pin: %ld, SCL pin: %ld Fast Mode.\n",
        i2c_cfg.i2c_pin_cfg.sda_io_num,
        i2c_cfg.i2c_pin_cfg.scl_io_num);
    if (ulp_riscv_i2c_master_init(&i2c_cfg) != ESP_OK)
        printf("RTC i2c init failed.\n");
}

void rtc_i2c_read_byte(uint8_t addr, uint8_t reg_addr, u_int8_t *data) {
    ulp_riscv_i2c_master_set_slave_addr(addr);
    ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr);
    ulp_riscv_i2c_master_read_from_device(data, 1);
}

void rtc_i2c_scan() {
    for (int addr=0; addr<0x7F; addr++) {
        if (addr % 8 == 0)
            printf("\n");

        uint8_t data = 0;
        rtc_i2c_read_byte(addr, 0x00, &data);
        printf("0x%02X ", data);
    }
}

void init_system() {
    rtc_i2c_init(GPIO_NUM_3, GPIO_NUM_0);
    //rtc_i2c_scan(); // error when change device address

    uint8_t data = 0;
    rtc_i2c_read_byte(0x68, 0x75, &data);
    printf("MPU6050 addr: 0x68 register:0x75 (WHO_AM_I) read: 0x%X <-- First time mostly failed.\n", data);
    // https://github.com/espressif/esp-idf/issues/11608

    rtc_i2c_read_byte(0x68, 0x75, &data);
    printf("MPU6050 addr: 0x68 register:0x75 (WHO_AM_I) read: 0x%X\n", data);

    init_ulp_program();

    if (false) {
        xTaskCreate(uart_monitor, "uart_monitor", 2048, NULL, 1, NULL);
        ulp_riscv_lock_t *lock = (ulp_riscv_lock_t*)&ulp_lock;
        while (true) {
            ulp_riscv_lock_acquire(lock);
            if (ulp_mpu6050_ready) {
                printf("Temp = %.02f C ", ((int16_t)ulp_mpu6050_temp)/340 + 35.53);
                printf("accX = %.02f "  , ((int16_t)ulp_mpu6050_accX)/16384.0);
                printf("accY = %.02f "  , ((int16_t)ulp_mpu6050_accY)/16384.0);
                printf("accZ = %.02f \n", ((int16_t)ulp_mpu6050_accZ)/16384.0);
            }
            ulp_riscv_lock_release(lock);

            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

struct acc_t {
    float accX;
    float accY;
    float accZ;
};

struct acc_t acc_data[10];

void app_main(void)
{   
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    /* not a wakeup from ULP, load the firmware */
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Normal Start Up (%d).\n", cause);
        init_system();
    }

    /* ULP Risc-V read and detected a temperature above the limit */
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        ulp_riscv_lock_t *lock = (ulp_riscv_lock_t*)&ulp_lock;
        for (int i=0; i<10; i++) {
            ulp_riscv_lock_acquire(lock);
            acc_data[i].accX = ((int16_t)ulp_mpu6050_accX)/16384.0;
            acc_data[i].accY = ((int16_t)ulp_mpu6050_accY)/16384.0;
            acc_data[i].accZ = ((int16_t)ulp_mpu6050_accZ)/16384.0;
            ulp_riscv_lock_release(lock);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        printf("Motion Recorded!\n");
    }
    /* Go back to sleep, only the ULP Risc-V will run */
    printf("Entering in deep sleep\n\n");
    vTaskDelay(100);
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup());
    esp_deep_sleep_start();
}

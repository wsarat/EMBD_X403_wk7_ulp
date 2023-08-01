#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
//#include "ulp_riscv_i2c.h"
#include "ulp_riscv_lock_ulp_core.h"

#include "ulp_riscv_print.h"
#include "ulp_riscv_uart_ulp_core.h"
#include "sdkconfig.h" // for UART baudrate

static ulp_riscv_uart_t s_print_uart;
ulp_riscv_lock_t lock;

#define LED_PIN     GPIO_NUM_2
#define UART_TX_PIN GPIO_NUM_14

#define MPU6050_ADDR    0x68

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

uint16_t mpu6050_ready;
uint16_t mpu6050_temp;
uint16_t mpu6050_accX;
uint16_t mpu6050_accY;
uint16_t mpu6050_accZ;

void uart_print(char* key, uint16_t value) {
    ulp_riscv_print_str(key);
    ulp_riscv_print_str(" -> ");
    ulp_riscv_print_hex(value);
    ulp_riscv_print_str("\n");
}

void ulp_i2c_read(int8_t addr, int8_t reg_addr, int8_t *data)
{
    ulp_riscv_i2c_master_set_slave_addr(addr);
    ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr);
    ulp_riscv_i2c_master_read_from_device(data, 1);
}

void ulp_i2c_read2(int8_t addr, int8_t reg_addr, int16_t *data)
{
    uint8_t temp = 0;
    ulp_riscv_i2c_master_set_slave_addr(addr);
    ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr);
    ulp_riscv_i2c_master_read_from_device(&temp, 1);
    *data = temp << 8;

    ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr+1);
    ulp_riscv_i2c_master_read_from_device(&temp, 1);
    *data |= temp;
}

void ulp_i2c_write(int8_t addr, int8_t reg_addr, int8_t data)
{
    ulp_riscv_i2c_master_set_slave_addr(addr);
    ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr);
    ulp_riscv_i2c_master_write_to_device(&data, 1);
}

void ulp_i2c_scan() {
    int ret;
    uint8_t data;

    ulp_riscv_print_str("ulp i2c scan start.\n");
    
    for (uint8_t addr=0; addr<0x7f; addr++) {  
        data = 0;
        ulp_i2c_read(addr, 0, &data);
        if (data > 0)
            uart_print("Found i2c at ", addr);
    }    
    ulp_riscv_print_str("ulp i2c scan end.\n\n");           
}

void mpu6050_init(uint8_t addr) {
    ulp_riscv_print_str("mpu6050 init.\n");
    
    ulp_i2c_write(addr, MPU6050_PWR_MGMT_1, 0);
    ulp_i2c_write(addr, MPU6050_SMPLRT_DIV, 0x00);
    ulp_i2c_write(addr, MPU6050_CONFIG, 0x00);
    ulp_i2c_write(addr, MPU6050_GYRO_CONFIG, 0x08);
    ulp_i2c_write(addr, MPU6050_ACCEL_CONFIG, 0x00);
    ulp_i2c_write(addr, MPU6050_PWR_MGMT_1, 0x01);    
}

int main (void)
{   
    mpu6050_ready = 0;
    mpu6050_temp = 0;
    mpu6050_accX = 0;
    mpu6050_accY = 0;
    mpu6050_accZ = 0;

    // UART
    ulp_riscv_uart_cfg_t cfg = {
        .tx_pin = UART_TX_PIN,
    };
    ulp_riscv_uart_init(&s_print_uart, &cfg);
    ulp_riscv_print_install((putc_fn_t)ulp_riscv_uart_putc, &s_print_uart);   

    ulp_riscv_delay_cycles(3000 * ULP_RISCV_CYCLES_PER_MS); // just a delay, not to missed first uart print.
    ulp_riscv_print_str("ULP riscV started!\n");

    // ulp_i2c_scan(); //not working

    // LED
    ulp_riscv_gpio_init(LED_PIN);
    ulp_riscv_gpio_output_enable(LED_PIN);
    ulp_riscv_gpio_output_level(LED_PIN, 1);

    mpu6050_init(MPU6050_ADDR);

    uint8_t data = 0;
    uint16_t data2 = 0;
    while (true) {
        ulp_riscv_gpio_output_level(LED_PIN, 1);
        
        ulp_i2c_read(MPU6050_ADDR, MPU6050_PWR_MGMT_1, &data);
        if (data & 1 << 6) {
            ulp_riscv_print_str("MPU6050 Sleep!!!!\n");
        }

        ulp_i2c_read(MPU6050_ADDR, MPU6050_WHO_AM_I, &data);
        if (data != 0x68)
            ulp_riscv_print_str("something wrong!!!!\n");
        //uart_print("MPU6050_WHO_AM_I", data);

        // ulp_i2c_read(MPU6050_ADDR, MPU6050_TEMP_H, &data);   
        // uart_print("MPU6050_TEMP_H ", data);
        // mpu6050_temp = data << 8;

        // ulp_i2c_read(MPU6050_ADDR, MPU6050_TEMP_L, &data);   
        // uart_print("MPU6050_TEMP_L ", data);   
        // mpu6050_temp |= data;  
        ulp_i2c_read2(MPU6050_ADDR, MPU6050_TEMP_H, &data2);  
        ulp_riscv_lock_acquire(&lock);  
        mpu6050_temp = data2;  
        ulp_riscv_lock_release(&lock); 

        ulp_i2c_read2(MPU6050_ADDR, 0x3B, &data2);   
        ulp_riscv_lock_acquire(&lock);   
        mpu6050_accX = data2;    
        ulp_riscv_lock_release(&lock);    

        ulp_i2c_read2(MPU6050_ADDR, 0x3D, &data2);  
        ulp_riscv_lock_acquire(&lock);    
        mpu6050_accY = data2; 
        ulp_riscv_lock_release(&lock);   

        ulp_i2c_read2(MPU6050_ADDR, 0x3F, &data2);    
        ulp_riscv_lock_acquire(&lock);  
        mpu6050_accZ = data2;   
        ulp_riscv_lock_release(&lock);   

        ulp_riscv_gpio_output_level(LED_PIN, 0);
        ulp_riscv_lock_acquire(&lock); 
        mpu6050_ready = 1;
        ulp_riscv_lock_release(&lock);

        ulp_riscv_delay_cycles(50 * ULP_RISCV_CYCLES_PER_MS);
    }

    return 0;
}

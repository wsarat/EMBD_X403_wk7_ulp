#include <stdbool.h>
#include <stdlib.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
//#include "ulp_riscv_i2c.h"
#include "ulp_riscv_lock_ulp_core.h"

#include "ulp_riscv_print.h"
#include "ulp_riscv_uart_ulp_core.h"
#include "sdkconfig.h" // for UART baudrate

#include "ulp_riscv.h"

static ulp_riscv_uart_t s_print_uart;
ulp_riscv_lock_t lock;


#define LED_PIN     GPIO_NUM_2
#define UART_TX_PIN GPIO_NUM_14

#define MPU6050_ADDR    0x68

#define MPU6050_INT          GPIO_NUM_13
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

#define SIGNAL_PATH_RESET  0x68
#define I2C_SLV0_ADDR      0x37
#define ACCEL_CONFIG       0x1C 
#define MOT_THR            0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR            0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL    0x69
#define INT_ENABLE         0x38
#define INT_STATUS          0x3A

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

    // Interrupt
    ulp_i2c_write(addr, SIGNAL_PATH_RESET, 0x07); //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
    ulp_i2c_write(addr, I2C_SLV0_ADDR, 0x20); 
    //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
    ulp_i2c_write(addr, ACCEL_CONFIG, 0x01); 
    //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
    ulp_i2c_write(addr, MOT_THR, 10); 
    //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
    ulp_i2c_write(addr, MOT_DUR, 40); 
    //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
    ulp_i2c_write(addr, MOT_DETECT_CTRL, 0x15); 
    //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
    ulp_i2c_write(addr, INT_ENABLE, 0x40); 
    //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
    ulp_i2c_write(addr, 0x37, 160); 
    // now INT pin is active low
}

uint16_t last_temp = 0;
uint16_t last_accX = 0;
uint16_t last_accY = 0;
uint16_t last_accZ = 0;
int8_t motion_count = 0;

void read_mpu6050() {
    uint16_t raw_temp = 0;
    uint16_t raw_accX = 0;
    uint16_t raw_accY = 0;
    uint16_t raw_accZ = 0;

    uint8_t data = 0;
    uint16_t data2 = 0;

    ulp_i2c_read(MPU6050_ADDR, MPU6050_PWR_MGMT_1, &data);
    if (data & 1 << 6) {
        ulp_riscv_print_str("MPU6050 Sleep!!!!\n");
        return;
    }

    ulp_i2c_read(MPU6050_ADDR, MPU6050_WHO_AM_I, &data);
    if (data != 0x68) {
        ulp_riscv_print_str("something wrong!!!!\n"); 
        return;
    }
    
    ulp_riscv_gpio_output_level(LED_PIN, 1);

    ulp_i2c_read2(MPU6050_ADDR, MPU6050_TEMP_H, &data2);  
    raw_temp = data2;  

    ulp_i2c_read2(MPU6050_ADDR, 0x3B, &data2);   
    raw_accX = data2;       

    ulp_i2c_read2(MPU6050_ADDR, 0x3D, &data2);    
    raw_accY = data2;   

    ulp_i2c_read2(MPU6050_ADDR, 0x3F, &data2);     
    raw_accZ = data2;    

    ulp_riscv_gpio_output_level(LED_PIN, 0); 

    // dirty motion lol
    int sum_diff = 0;
    sum_diff += abs(last_accX - raw_accX);
    sum_diff += abs(last_accY - raw_accY);
    sum_diff += abs(last_accZ - raw_accZ);

    last_accX = raw_accX;
    last_accY = raw_accY;
    last_accZ = raw_accZ;

    if (sum_diff > 1000)
        motion_count += 1;
    else
        motion_count = 0;

    if (motion_count > 2) {
        motion_count = -10;
        ulp_riscv_print_str("Motion!\n"); 
        ulp_riscv_wakeup_main_processor();
    }

    ulp_riscv_lock_acquire(&lock); 
    mpu6050_ready = 1;
    mpu6050_temp = raw_temp;
    mpu6050_accX = raw_accX;
    mpu6050_accY = raw_accY;
    mpu6050_accZ = raw_accZ;
    ulp_riscv_lock_release(&lock); 

    ulp_riscv_delay_cycles(50 * ULP_RISCV_CYCLES_PER_MS);
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

    ulp_riscv_delay_cycles(1000 * ULP_RISCV_CYCLES_PER_MS); // just a delay, not to missed first uart print.
    ulp_riscv_print_str("ULP riscV started!\n");

    // ulp_i2c_scan(); //not working

    // LED
    ulp_riscv_gpio_init(LED_PIN);
    ulp_riscv_gpio_output_enable(LED_PIN);
    //ulp_riscv_gpio_output_level(LED_PIN, 1);

    // GPIO
    ulp_riscv_gpio_init(MPU6050_INT);
    ulp_riscv_gpio_input_enable(MPU6050_INT);

    mpu6050_init(MPU6050_ADDR);

    bool gpio_level;
    bool gpio_level_previous = (bool)ulp_riscv_gpio_get_level(MPU6050_INT);
    
    int count = 0;
    while (true) {
        gpio_level = (bool)ulp_riscv_gpio_get_level(MPU6050_INT);
        if (gpio_level != gpio_level_previous) {
            if (gpio_level == 0) {
                uart_print("Got Interrupt!", count);
                count += 1;
            }
            gpio_level_previous = gpio_level;
        }

        read_mpu6050();
        //ulp_riscv_delay_cycles(50 * ULP_RISCV_CYCLES_PER_MS);
    }

    return 0;
}

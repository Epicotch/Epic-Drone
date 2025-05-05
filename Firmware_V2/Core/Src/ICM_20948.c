#include "ICM_20948.h"

I2C_HandleTypeDef *_icm20948_i2c;

uint8_t ICM_20948_READ = (ICM_20948_ADDR << 1) | I2C_READ;
uint8_t ICM_20948_WRITE = (ICM_20948_ADDR << 1) | I2C_WRITE;

/* Function prototypes */
uint8_t icm20948_init(I2C_HandleTypeDef *i2c);
uint8_t icm20948_data_ready();
HAL_I2C_StateTypeDef icm20948_write(uint8_t reg, uint8_t data);
HAL_I2C_StateTypeDef icm20948_read(uint8_t reg, uint8_t *data, uint8_t length);
void icm20948_reset();

// initializes ICM20948 I2C, and confirms by reading ICM_20948_WHOAMI
uint8_t icm20948_init(I2C_HandleTypeDef *i2c) {
    _icm20948_i2c = i2c;
    uint8_t out;
    icm20948_read(ICM_20948_WHOAMI, &out, 1);
    return (out == 0xEA);
}

void icm20948_reset() {
    icm20948_write(ICM_20948_PWR_MGMT_1, 0x80);
    osDelay(700);
}

Vector3 icm20948_get_accel() {
    Vector3 out;
    HAL_I2C_Master_Transmit(&_icm20948_i2c, ICM_20948_WRITE, ); // TODO: make burst read.
}

/* FreeRTOS functions */

/* STM32 HAL functions */

// writes single bit to ICM20948
HAL_I2C_StateTypeDef icm20948_write(uint8_t reg, uint8_t data) {
    uint8_t transmitData[2] = {reg, data};
    return HAL_I2C_Master_Transmit(&_icm20948_i2c, ICM_20948_WRITE, transmitData, sizeof(transmitData), 100);
}

// reads arbitrary number of bits from ICM20948
HAL_I2C_StateTypeDef icm20948_read(uint8_t reg, uint8_t *data, uint8_t length) {
    HAL_I2C_StateTypeDef writeStatus = HAL_I2C_Master_Transmit(&_icm20948_i2c, ICM_20948_WRITE, reg, 1, 100);
    if (writeStatus != HAL_OK)
        return writeStatus;
    return HAL_I2C_Master_Receive(&_icm20948_i2c, ICM_20948_READ, data, length, 100);
}


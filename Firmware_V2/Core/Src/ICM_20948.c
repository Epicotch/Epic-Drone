#include "ICM_20948.h"

uint8_t ICM_20948_READ = (ICM_20948_ADDR << 1) | I2C_READ;
uint8_t ICM_20948_WRITE = (ICM_20948_ADDR << 1) | I2C_WRITE;

// initializes ICM20948 I2C, and confirms by reading ICM_20948_WHOAMI
bool icm20948_init(ICM20948 *imu, I2C_HandleTypeDef *i2c, uint8_t addr) {
    imu->i2c = i2c;
    imu->writeAddr = (addr << 1) | I2C_WRITE;
    imu->readAddr = (addr << 1) | I2C_READ;
    uint8_t out;
    icm20948_read(imu, ICM_20948_WHOAMI, &out, 1);
    return (out == 0xEA);
}

// set the relevant bank
void icm20948_set_reg_bank(ICM20948 *imu, uint8_t bank) {
    icm20948_write(imu, ICM_20948_REG_BANK_SEL, bank << 4);
    imu->reg_bank = bank;
}

// sets IMU accelerometer settings
void icm20948_set_accel_settings(ICM20948 *imu, ICM20948_ACCEL_SETTINGS *settings) {
    icm20948_set_reg_bank(imu, 2);

    uint8_t data1 = 0;
    uint8_t data2 = 0;

    data1 |= settings->dlpfcfg << 3;
    data1 |= settings->fullScale << 1;
    data1 |= settings->fchoice;

    data2 |= settings->x_self_test << 4;
    data2 |= settings->y_self_test << 3;
    data2 |= settings->z_self_test << 2;
    data2 |= settings->dec3cfg;

    switch(settings->fullScale) {
        case ICM20948_ACCEL_FS_2G:
            imu->accelCoeff = 16.0;
            break;
        case ICM20948_ACCEL_FS_4G:
            imu->accelCoeff = 8.0;
            break;
        case ICM20948_ACCEL_FS_8G:
            imu->accelCoeff = 4.0;
            break;
        case ICM20948_ACCEL_FS_16G:
            imu->accelCoeff = 2.0;
            break;
    }

    icm20948_write(imu, ICM_20948_ACCEL_CONFIG, data1);
    icm20948_write(imu, ICM_20948_ACCEL_CONFIG_2, data2);

    icm20948_set_reg_bank(imu, 0);
}

// sets IMU gyroscope settings
void icm20948_set_gyro_settings(ICM20948 *imu, ICM20948_GYRO_SETTINGS *settings) {
    icm20948_set_reg_bank(imu, 2);

    uint8_t data1 = 0;
    uint8_t data2 = 0;

    data1 |= settings->dlpfcfg << 3;
    data1 |= settings->fullScale << 1;
    data1 |= settings->fchoice;

    data2 |= settings->x_self_test << 5;
    data2 |= settings->y_self_test << 4;
    data2 |= settings->z_self_test << 3;
    data2 |= settings->avgcfg;

    switch(settings->fullScale) {
        case ICM20948_GYRO_FS_250DPS:
            imu->accelCoeff = 131.0;
            break;
        case ICM20948_GYRO_FS_500DPS:
            imu->accelCoeff = 65.5;
            break;
        case ICM20948_GYRO_FS_1000DPS:
            imu->accelCoeff = 32.75;
            break;
        case ICM20948_GYRO_FS_2000DPS:
            imu->accelCoeff = 16.375;
            break;
    }

    icm20948_write(imu, ICM_20948_GYRO_CONFIG_1, data1);
    icm20948_write(imu, ICM_20948_GYRO_CONFIG_2, data2);

    icm20948_set_reg_bank(imu, 0);
}

// resets IMU via PWR_MGMT_1 register
void icm20948_reset(ICM20948 *imu) {
    icm20948_write(imu, ICM_20948_PWR_MGMT_1, 0x80);
    osDelay(700);
}

bool icm20948_data_ready(ICM20948 *imu) {
    uint8_t data;
    icm20948_read(imu, ICM_20948_DATA_RDY_STATUS, &data, 1);
    return data & 1;
}

// gets acceleration data from sensor and puts it into imu.accel
HAL_I2C_StateTypeDef icm20948_get_accel(ICM20948 *imu) {
    HAL_I2C_StateTypeDef out = HAL_I2C_Master_Transmit(imu->i2c, imu->writeAddr, ICM_20948_ACCEL_XOUT_H, 1, 100);
    if (out != HAL_OK) {
        return out;
    }
    uint8_t data[6];
    out = HAL_I2C_Master_Recieve(imu->i2c, imu->readAddr, data, sizeof(data), 100);
    float temp[3];
    
    for (int i = 0; i < 3; i++) {
        temp[i] = ((int16_t*)data)[i]/imu->gyroCoeff;
    }

    imu->accel.x = temp[0];
    imu->accel.y = temp[1];
    imu->accel.z = temp[2];

    return out;
}

// gets gyro data from sensor and puts it into imu.accel
HAL_I2C_StateTypeDef icm20948_get_gyro(ICM20948 *imu) {
    HAL_I2C_StateTypeDef out = HAL_I2C_Master_Transmit(imu->i2c, imu->writeAddr, ICM_20948_GYRO_XOUT_H, 1, 100);
    if (out != HAL_OK) {
        return out;
    }
    uint8_t data[6];
    out = HAL_I2C_Master_Recieve(imu->i2c, imu->readAddr, data, sizeof(data), 100);
    float temp[3];
    
    for (int i = 0; i < 3; i++) {
        temp[i] = ((int16_t*)data)[i]/imu->gyroCoeff;
    }

    imu->gyro.x = temp[0];
    imu->gyro.y = temp[1];
    imu->gyro.z = temp[2];

    return out;
}

/* FreeRTOS functions */

/* STM32 HAL functions */

// writes single byte to ICM20948
HAL_I2C_StateTypeDef icm20948_write(ICM20948 *imu, uint8_t reg, uint8_t data) {
    uint8_t transmitData[2] = {reg, data};
    return HAL_I2C_Master_Transmit(imu->i2c, imu->writeAddr, transmitData, sizeof(transmitData), 100);
}

// reads arbitrary number of bits from ICM20948
HAL_I2C_StateTypeDef icm20948_read(ICM20948 *imu, uint8_t reg, uint8_t *data, uint8_t length) {
    HAL_I2C_StateTypeDef writeStatus = HAL_I2C_Master_Transmit(imu->i2c, imu->writeAddr, reg, 1, 100); // make sure to see if we need &reg instead of reg
    if (writeStatus != HAL_OK)
        return writeStatus;
    return HAL_I2C_Master_Receive(imu->i2c, imu->readAddr, data, length, 100);
}


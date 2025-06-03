#include "imu.h"
#include "../../lib/BNO055_SensorAPI/bno055.h"

#define BNO055_TAG      "BNO055"
#define BNO055_DEV_ADDR (BNO055_I2C_ADDR1 << 1)

struct bno055_t bno055dev;
struct bno055_accel_t bno055_accel;
struct bno055_gyro_t bno055_gyro;

int8_t bno055_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt) {
    if(furi_hal_i2c_write_mem(&furi_hal_i2c_handle_external, dev_addr, reg_addr, reg_data, cnt, 50))
        return BNO055_SUCCESS;
    return BNO055_ERROR;
}

int8_t bno055_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt) {
    if(furi_hal_i2c_read_mem(&furi_hal_i2c_handle_external, dev_addr, reg_addr, reg_data, cnt, 50))
        return BNO055_SUCCESS;
    return BNO055_ERROR;
}

void bno055_delay_msec(u32 msec) {
    furi_delay_ms(msec);
}

bool bno055_begin() {
    FURI_LOG_I(BNO055_TAG, "Init BNO055");

    if(!furi_hal_i2c_is_device_ready(&furi_hal_i2c_handle_external, BNO055_DEV_ADDR, 50)) {
        FURI_LOG_E(BNO055_TAG, "Not ready");
        return false;
    }

    bno055dev.bus_write = bno055_i2c_bus_write;
    bno055dev.bus_read = bno055_i2c_bus_read;
    bno055dev.delay_msec = bno055_delay_msec;
    bno055dev.dev_addr = BNO055_DEV_ADDR;

    if(bno055_init(&bno055dev) != BNO055_SUCCESS) {
        FURI_LOG_E(BNO055_TAG, "Init failed");
        return false;
    }

    if(bno055_set_power_mode(BNO055_POWER_MODE_NORMAL) != BNO055_SUCCESS) {
        FURI_LOG_E(BNO055_TAG, "Power mode set failed");
        return false;
    }

    if(bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF) != BNO055_SUCCESS) {
        FURI_LOG_E(BNO055_TAG, "Operation mode set failed");
        return false;
    }

    FURI_LOG_I(BNO055_TAG, "Init OK");
    return true;
}

void bno055_end() {
    bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);
}

int bno055_read(double* vec) {
    int ret = 0;
    if(bno055_read_accel_xyz(&bno055_accel) == BNO055_SUCCESS) {
        vec[0] = bno055_accel.x / 100.0; // m/s^2
        vec[1] = bno055_accel.y / 100.0;
        vec[2] = bno055_accel.z / 100.0;
        ret |= ACC_DATA_READY;
    }
    if(bno055_read_gyro_xyz(&bno055_gyro) == BNO055_SUCCESS) {
        vec[3] = bno055_gyro.x * (float)DEG_TO_RAD / 16.0; // rad/s
        vec[4] = bno055_gyro.y * (float)DEG_TO_RAD / 16.0;
        vec[5] = bno055_gyro.z * (float)DEG_TO_RAD / 16.0;
        ret |= GYR_DATA_READY;
    }
    return ret;
}

struct imu_t imu_bno055 = {BNO055_DEV_ADDR, bno055_begin, bno055_end, bno055_read, BNO055_TAG};

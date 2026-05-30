#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

class MPU6050 {
public:
    MPU6050();

    bool init();
    
    void update();
    
    bool recalibrate();
    
    float get_pitch_angle() const { return pitch_angle; }
    float get_roll_angle() const { return roll_angle; }
    
    bool is_healthy() const { return healthy; }
    
    uint8_t get_error_code() const { return error_code; }

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    
    static constexpr uint8_t MPU6050_ADDR = 0x68;
    static constexpr uint8_t MPU6050_SMPLRT_DIV = 0x19;
    static constexpr uint8_t MPU6050_CONFIG = 0x1A;
    static constexpr uint8_t MPU6050_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t MPU6050_INT_ENABLE = 0x38;
    static constexpr uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t MPU6050_GYRO_XOUT_H = 0x43;
    static constexpr uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t MPU6050_WHO_AM_I = 0x75;
    
    static constexpr float GYRO_SCALE = 1.0f / 65.5f * DEG_TO_RAD;
    static constexpr float ACCEL_SCALE = 1.0f / 16384.0f;
    static constexpr float DT = 0.01f;
    
    bool healthy;
    float pitch_angle;
    float roll_angle;
    uint8_t error_code;
    
    float q0, q1, q2, q3;
    float integral_x, integral_y, integral_z;
    uint32_t update_count;
    
    int16_t gyro_zero_x;
    int16_t gyro_zero_y;
    int16_t gyro_zero_z;
    
    uint8_t read_error_count;
    bool calibrating;  // 校准进行中标志
    uint32_t calibration_complete_ms;  // 校准完成时间戳
    
    float inv_sqrt(float x);
    
    bool calibrate_gyro();
    bool read_raw_data(int16_t &acc_x, int16_t &acc_y, int16_t &acc_z, 
                       int16_t &gyro_x, int16_t &gyro_y, int16_t &gyro_z);
    
    bool write_register(uint8_t reg, uint8_t value);
    bool read_registers(uint8_t reg, uint8_t *data, uint8_t len);
};

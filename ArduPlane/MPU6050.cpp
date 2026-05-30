#include "MPU6050.h"
#include "Plane.h"

extern const AP_HAL::HAL& hal;

MPU6050::MPU6050() :
    healthy(false),
    pitch_angle(0.0f),
    roll_angle(0.0f),
    error_code(0),
    q0(1.0f),
    q1(0.0f),
    q2(0.0f),
    q3(0.0f),
    integral_x(0.0f),
    integral_y(0.0f),
    integral_z(0.0f),
    update_count(0),
    gyro_zero_x(0),
    gyro_zero_y(0),
    gyro_zero_z(0),
    read_error_count(0),
    calibrating(false),
    calibration_complete_ms(0)
{
}

bool MPU6050::init()
{
    error_code = 0;
    dev = hal.i2c_mgr->get_device(0, MPU6050_ADDR);
    
    if (!dev) {
        error_code = 1;
        return false;
    }
    
    dev->get_semaphore()->take_blocking();
    
    uint8_t chip_id = 0;
    if (!read_registers(MPU6050_WHO_AM_I, &chip_id, 1)) {
        error_code = 2;
        dev->get_semaphore()->give();
        return false;
    }
    
    if (chip_id != 0x68) {
        error_code = 3;
        dev->get_semaphore()->give();
        return false;
    }
    
    if (!write_register(MPU6050_PWR_MGMT_1, 0x80)) {
        dev->get_semaphore()->give();
        return false;
    }
    hal.scheduler->delay(100);
    
    if (!write_register(MPU6050_PWR_MGMT_1, 0x00)) {
        dev->get_semaphore()->give();
        return false;
    }
    hal.scheduler->delay(10);
    
    uint8_t smplrt_div = (uint8_t)(1000.0f / 200.0f - 1);
    if (!write_register(MPU6050_SMPLRT_DIV, smplrt_div)) {
        dev->get_semaphore()->give();
        return false;
    }
    
    if (!write_register(0x37, 0x00)) {
        dev->get_semaphore()->give();
        return false;
    }
    
    if (!write_register(MPU6050_INT_ENABLE, 0x01)) {
        dev->get_semaphore()->give();
        return false;
    }
    
    if (!write_register(MPU6050_CONFIG, 0x04)) {
        dev->get_semaphore()->give();
        return false;
    }
    
    if (!write_register(MPU6050_GYRO_CONFIG, 0x08)) {
        dev->get_semaphore()->give();
        return false;
    }
    
    if (!write_register(MPU6050_ACCEL_CONFIG, 0x00)) {
        dev->get_semaphore()->give();
        return false;
    }
    
    if (!write_register(MPU6050_PWR_MGMT_1, 0x01)) {
        dev->get_semaphore()->give();
        return false;
    }
    
    hal.scheduler->delay(100);
    
    if (!calibrate_gyro()) {
        dev->get_semaphore()->give();
        return false;
    }
    
    dev->get_semaphore()->give();
    
    healthy = true;
    return true;
}

bool MPU6050::recalibrate()
{
    if (!dev) {
        return false;
    }

    
    init();
    // 设置校准标志，暂停update
    calibrating = true;
    
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        calibrating = false;
        return false;
    }
    
    bool success = calibrate_gyro();
    dev->get_semaphore()->give();
    // 清除校准标志，记录完成时间
    calibrating = false;
    if (success) {
        calibration_complete_ms = AP_HAL::millis();
    }
    
    if (success) {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        integral_x = 0.0f;
        integral_y = 0.0f;
        integral_z = 0.0f;
        update_count = 0;
        pitch_angle = 0.0f;
        roll_angle = 0.0f;
    }
    
    return success;
}

bool MPU6050::calibrate_gyro()
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    const uint16_t samples = 2000;
    
    for (uint16_t i = 0; i < samples; i++) {
        uint8_t data[6];
        if (!read_registers(MPU6050_GYRO_XOUT_H, data, 6)) {
            return false;
        }
        
        int16_t gx = (int16_t)((data[0] << 8) | data[1]);
        int16_t gy = (int16_t)((data[2] << 8) | data[3]);
        int16_t gz = (int16_t)((data[4] << 8) | data[5]);
        
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        
        hal.scheduler->delay(5);
    }
    
    gyro_zero_x = sum_x / samples;
    gyro_zero_y = sum_y / samples;
    gyro_zero_z = sum_z / samples;
    
    return true;
}

float MPU6050::inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void MPU6050::update()
{
    // 如果正在校准，暂停更新
    if (calibrating) {
        return;
    }
    
    // 校准完成后等待2秒再恢复更新
    if (calibration_complete_ms != 0) {
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - calibration_complete_ms < 2000) {
            return;  // 还在等待期内，暂停更新
        }
        calibration_complete_ms = 0;  // 清除时间戳，恢复正常
    }
    
    if (!healthy || !dev) {
        return;
    }
    
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    
    if (!read_raw_data(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)) {
        read_error_count++;
        if (read_error_count >= 10) {
            healthy = false;
        }
        return;
    }
    
    read_error_count = 0;
    
    float ax = acc_x * ACCEL_SCALE;
    float ay = acc_y * ACCEL_SCALE;
    float az = acc_z * ACCEL_SCALE;
    float gx = gyro_x * GYRO_SCALE;
    float gy = gyro_y * GYRO_SCALE;
    float gz = gyro_z * GYRO_SCALE;
    
    float acc_mag = ax*ax + ay*ay + az*az;
    
    float kp, ki;
    if (update_count < 400) {
        update_count++;
        kp = 8.0f;
        ki = 0.002f;
    } else {
        if (acc_mag > 1.44f || acc_mag < 0.64f) {
            kp = 3.6f;
            ki = 0.001f;
        } else {
            kp = 4.8f;
            ki = 0.0015f;
        }
    }
    
    if (acc_mag > 0.01f) {
        float recip_norm = inv_sqrt(acc_mag);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;
        
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;
        
        if (ki > 0.0f) {
            integral_x += ex * DT;
            integral_y += ey * DT;
            integral_z += ez * DT;
            gx += ki * integral_x;
            gy += ki * integral_y;
            gz += ki * integral_z;
        }
        
        gx += kp * ex;
        gy += kp * ey;
        gz += kp * ez;
    }
    
    float q_dot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float q_dot1 = 0.5f * (q0*gx + q2*gz - q3*gy);
    float q_dot2 = 0.5f * (q0*gy - q1*gz + q3*gx);
    float q_dot3 = 0.5f * (q0*gz + q1*gy - q2*gx);
    
    q0 += q_dot0 * DT;
    q1 += q_dot1 * DT;
    q2 += q_dot2 * DT;
    q3 += q_dot3 * DT;
    
    float norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
    
    roll_angle = degrees(atan2f(2.0f * (q0*q1 + q2*q3), 
                                1.0f - 2.0f * (q1*q1 + q2*q2)));
    pitch_angle = degrees(asinf(2.0f * (q0*q2 - q3*q1)));
}

bool MPU6050::read_raw_data(int16_t &acc_x, int16_t &acc_y, int16_t &acc_z,
                            int16_t &gyro_x, int16_t &gyro_y, int16_t &gyro_z)
{
    uint8_t data[14];
    
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    
    bool success = read_registers(MPU6050_ACCEL_XOUT_H, data, 14);
    
    dev->get_semaphore()->give();
    
    if (!success) {
        return false;
    }
    
    acc_x = (int16_t)((data[0] << 8) | data[1]);
    acc_y = (int16_t)((data[2] << 8) | data[3]);
    acc_z = (int16_t)((data[4] << 8) | data[5]);
    
    gyro_x = (int16_t)((data[8] << 8) | data[9]) - gyro_zero_x;
    gyro_y = (int16_t)((data[10] << 8) | data[11]) - gyro_zero_y;
    gyro_z = (int16_t)((data[12] << 8) | data[13]) - gyro_zero_z;
    
    return true;
}

bool MPU6050::write_register(uint8_t reg, uint8_t value)
{
    return dev->write_register(reg, value);
}

bool MPU6050::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    return dev->read_registers(reg, data, len);
}

#include "Plane.h"

void Plane::mpu6050_init(void)
{
    if (mpu6050.init()) {
        gcs().send_text(MAV_SEVERITY_INFO, "MPU6050: Initialized");
    } else {
        uint8_t err = mpu6050.get_error_code();
        switch (err) {
            case 1:
                gcs().send_text(MAV_SEVERITY_ERROR, "MPU6050: Failed to get I2C device (bus=0, addr=0x68)");
                break;
            case 2:
                gcs().send_text(MAV_SEVERITY_ERROR, "MPU6050: Failed to read WHO_AM_I register");
                break;
            case 3:
                gcs().send_text(MAV_SEVERITY_ERROR, "MPU6050: Wrong chip ID (check I2C address)");
                break;
            default:
                gcs().send_text(MAV_SEVERITY_ERROR, "MPU6050: Init failed (error=%d)", err);
                break;
        }
    }
}

void Plane::mpu6050_update(void)
{
    // mpu6050.update();
    
// #if HAL_QUADPLANE_ENABLED
//     if (quadplane.available() && mpu6050.is_healthy()) {
//         quadplane.mpu6050_angle_pitch = mpu6050.get_pitch_angle();
//         quadplane.mpu6050_angle_roll = mpu6050.get_roll_angle();
        
//         static uint32_t last_print_ms = 0;
//         uint32_t now_ms = AP_HAL::millis();
//         if (now_ms - last_print_ms > 1000) {
//             last_print_ms = now_ms;
//             gcs().send_text(MAV_SEVERITY_INFO, 
//                           "MPU6050: P=%.2f R=%.2f", 
//                           (double)quadplane.mpu6050_angle_pitch,
//                           (double)quadplane.mpu6050_angle_roll);
//         }
//     } else {
//         static uint32_t last_print_ms = 0;
//         uint32_t now_ms = AP_HAL::millis();
//         if (now_ms - last_print_ms > 1000) {
//             last_print_ms = now_ms;
//             gcs().send_text(MAV_SEVERITY_WARNING, 
//                           "MPU6050: Not updating (avail=%d, healthy=%d)", 
//                           quadplane.available(), 
//                           mpu6050.is_healthy());
//         }
//     }
// #endif
}

void Plane::mpu6050_recalibrate(void)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "MPU6050: Starting recalibration...");
    
    // if (mpu6050.recalibrate()) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "MPU6050: Recalibration successful");
    // } else {
    //     gcs().send_text(MAV_SEVERITY_WARNING, "MPU6050: Recalibration failed");
    // }
}

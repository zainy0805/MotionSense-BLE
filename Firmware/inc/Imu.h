/**
 * ============================================================================
 * File Name   : Imu.h
 * Author      : 
 * Created On  : 29-08-2026
 * Description : LSM6DS3TR-C raw-I2C driver + IDLE/MOVING/WALKING activity
 *               state machine for the Seeed XIAO BLE Sense onboard IMU.
 * ============================================================================
 */

#ifndef IMU_H
#define IMU_H

/* ---------------------------- Includes ------------------------------------ */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdint.h>

/* ---------------------------- Macros / Constants --------------------------- */
#define IMU_I2C_ADDR              0x6A

#define IMU_IDLE_POLL_MS          1000  /* slow accel-only check while quiet   */
#define IMU_MOVING_POLL_MS        100   /* fast full read/print while active   */
#define IMU_IDLE_CONFIRM_SAMPLES  10    /* consecutive quiet samples (@ MOVING
                                          * rate) before ->IDLE                 */
#define IMU_WALKING_TIMEOUT_SAMPLES 10  /* consecutive samples with no new
                                          * step (@ MOVING rate) before
                                          * WALKING -> MOVING                   */
#define IMU_MOTION_THRESHOLD_MPS2 1.0   /* |accel| deviation from 1 g counted
                                          * as motion -- tune to taste          */

/* ---------------------------- Type Definitions ----------------------------- */

/**
 * @brief States of the IMU activity state machine.
 *
 *   IDLE    - sitting still, slow accel-only polling.
 *   MOVING  - accel magnitude says something is happening, but the chip's
 *             pedometer hasn't recognized a rhythmic step pattern (yet).
 *   WALKING - MOVING *and* the chip's step counter is actively incrementing.
 */
typedef enum
{
    IMU_STATE_IDLE = 0,
    IMU_STATE_MOVING,
    IMU_STATE_WALKING,
} ImuStateType;

/**
 * @brief Handle holding the runtime state for one IMU instance. Caller fills
 *        in i2cDev and ledGreen (from devicetree) before calling Imu_Init().
 */
typedef struct
{
    const struct device *i2cDev;
    struct gpio_dt_spec   ledGreen;
    ImuStateType          state;
    int                   quietStreak;
    int                   noStepStreak;
    uint16_t              lastStepCount;
} ImuHandleType;

/* ---------------------------- Function Prototypes --------------------------- */

int Imu_Init(ImuHandleType *imu);
int Imu_Update(ImuHandleType *imu);
uint32_t Imu_GetPollIntervalMs(const ImuHandleType *imu);
void Imu_Cleanup(ImuHandleType *imu);

#endif /* IMU_H */
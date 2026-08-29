/**
 * ============================================================================
 * File Name   : Imu.h
 * Author      : 
 * Created On  : DD-MM-YYYY
 * Description : LSM6DS3TR-C raw-I2C driver + IDLE/MOVING motion state machine
 *               for the Seeed XIAO BLE Sense onboard IMU.
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
                                          * rate) before MOVING -> IDLE         */
#define IMU_MOTION_THRESHOLD_MPS2 1.0   /* |accel| deviation from 1 g counted
                                          * as motion -- tune to taste          */

/* ---------------------------- Type Definitions ----------------------------- */

/**
 * @brief States of the IMU motion state machine.
 */
typedef enum
{
    IMU_STATE_IDLE = 0,
    IMU_STATE_MOVING,
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
} ImuHandleType;

/* ---------------------------- Function Prototypes --------------------------- */

int Imu_Init(ImuHandleType *imu);
int Imu_Update(ImuHandleType *imu);
uint32_t Imu_GetPollIntervalMs(const ImuHandleType *imu);
void Imu_Cleanup(ImuHandleType *imu);

#endif /* IMU_H */
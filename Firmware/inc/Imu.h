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

/**
 * @brief   Probes the LSM6DS3TR-C over I2C (WHO_AM_I) and configures the
 *          accel/gyro ODR and BDU/IF_INC. Initializes imu->state to
 *          IMU_STATE_IDLE.
 * @param   imu  Pointer to a handle with i2cDev/ledGreen already set.
 * @return  0 on success, -1 on failure (NULL pointer, device not ready, or
 *          WHO_AM_I mismatch).
 */
int Imu_Init(ImuHandleType *imu);

/**
 * @brief   Runs one iteration of the IDLE/MOVING state machine: reads accel
 *          to check for motion, and while MOVING also reads gyro/temp and
 *          prints a full sample. Updates imu->state as needed.
 * @param   imu  Pointer to an initialized handle.
 * @return  0 on success, -1 on I2C failure or NULL pointer.
 */
int Imu_Update(ImuHandleType *imu);

/**
 * @brief   Returns how long the caller should sleep before the next
 *          Imu_Update() call, based on the current state.
 * @param   imu  Pointer to an initialized handle.
 * @return  Milliseconds to sleep (IMU_IDLE_POLL_MS or IMU_MOVING_POLL_MS).
 */
uint32_t Imu_GetPollIntervalMs(const ImuHandleType *imu);

/**
 * @brief   Releases any resources owned by the handle (turns the LED off).
 *          Provided for symmetry with Imu_Init.
 * @param   imu  Pointer to the handle to clean up.
 * @return  void
 */
void Imu_Cleanup(ImuHandleType *imu);

#endif /* IMU_H */
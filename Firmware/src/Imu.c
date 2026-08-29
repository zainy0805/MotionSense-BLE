/**
 * ============================================================================
 * File Name   : Imu.c
 * Author      : 
 * Created On  : DD-MM-YYYY
 * Description : LSM6DS3TR-C raw-I2C driver + IDLE/MOVING motion state machine
 *               for the Seeed XIAO BLE Sense onboard IMU.
 * ============================================================================
 */

#include "Imu.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>
#include <math.h>

/* ---------------------------- Register Map (private) ----------------------- */
#define IMU_REG_WHO_AM_I        0x0F
#define IMU_REG_CTRL1_XL        0x10   /* accel ODR/scale */
#define IMU_REG_CTRL2_G         0x11   /* gyro ODR/scale  */
#define IMU_REG_CTRL3_C         0x12   /* BDU, IF_INC     */
#define IMU_REG_OUT_TEMP_L      0x20   /* temp: 2 bytes from here  */
#define IMU_REG_OUTX_L_G        0x22   /* gyro: 6 bytes from here  */
#define IMU_REG_OUTX_L_XL       0x28   /* accel: 6 bytes from here */

#define IMU_WHO_AM_I_VALUE       0x6A

/* ODR=104 Hz (0100), FS=+-2g (00), BW default (00) -> 0x40 */
#define IMU_CTRL1_XL_104HZ_2G      0x40
/* ODR=104 Hz (0100), FS=245 dps (00) -> 0x40 */
#define IMU_CTRL2_G_104HZ_245DPS   0x40
/* BDU=1 (bit6), IF_INC=1 (bit2) */
#define IMU_CTRL3_C_BDU_IFINC      0x44

/* sensitivities at these full-scale settings */
#define IMU_ACCEL_SENS_G_PER_LSB   0.000061  /* g per LSB at +-2g   */
#define IMU_GYRO_SENS_DPS_PER_LSB  0.00875   /* dps per LSB at 245  */
#define IMU_TEMP_LSB_PER_DEGC      256.0     /* LSB per deg C, 0 LSB @ 25 C */
#define IMU_TEMP_OFFSET_DEGC       25.0
#define IMU_GRAVITY_MPS2           9.80665

/* ---------------------------- Static (private) Helpers --------------------- */
/* Prefix private/helper functions with a lowercase module tag. */

static int16_t imu_toS16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((hi << 8) | lo);
}

static int imu_readAccel(const ImuHandleType *imu, double *ax, double *ay, double *az)
{
    uint8_t buf[6];
    int ret = i2c_burst_read(imu->i2cDev, IMU_I2C_ADDR, IMU_REG_OUTX_L_XL, buf, 6);

    if (ret != 0)
    {
        return ret;
    }

    *ax = imu_toS16(buf[0], buf[1]) * IMU_ACCEL_SENS_G_PER_LSB * IMU_GRAVITY_MPS2;
    *ay = imu_toS16(buf[2], buf[3]) * IMU_ACCEL_SENS_G_PER_LSB * IMU_GRAVITY_MPS2;
    *az = imu_toS16(buf[4], buf[5]) * IMU_ACCEL_SENS_G_PER_LSB * IMU_GRAVITY_MPS2;
    return 0;
}

static void imu_printGyro(const ImuHandleType *imu)
{
    uint8_t buf[6];
    int ret = i2c_burst_read(imu->i2cDev, IMU_I2C_ADDR, IMU_REG_OUTX_L_G, buf, 6);

    if (ret == 0)
    {
        double gx = imu_toS16(buf[0], buf[1]) * IMU_GYRO_SENS_DPS_PER_LSB;
        double gy = imu_toS16(buf[2], buf[3]) * IMU_GYRO_SENS_DPS_PER_LSB;
        double gz = imu_toS16(buf[4], buf[5]) * IMU_GYRO_SENS_DPS_PER_LSB;

        printk("gyro x:%.3f dps y:%.3f dps z:%.3f dps\n", gx, gy, gz);
    }
    else
    {
        printk("gyro read failed: %d\n", ret);
    }
}

static void imu_printTemp(const ImuHandleType *imu)
{
    uint8_t tbuf[2];
    int ret = i2c_burst_read(imu->i2cDev, IMU_I2C_ADDR, IMU_REG_OUT_TEMP_L, tbuf, 2);

    if (ret == 0)
    {
        double tempC = IMU_TEMP_OFFSET_DEGC +
                       (imu_toS16(tbuf[0], tbuf[1]) / IMU_TEMP_LSB_PER_DEGC);

        printk("temp: %.2f C\n", tempC);
    }
    else
    {
        printk("temp read failed: %d\n", ret);
    }
}

/**
 * @brief   Combines ax/ay/az into a single vector magnitude and compares it
 *          against 1 g -- orientation-independent motion check (see chat
 *          explanation: this is the same idea the chip's own hardware
 *          Significant-Motion engine uses, just run in software).
 */
static bool imu_isMoving(double ax, double ay, double az)
{
    double mag = sqrt(ax * ax + ay * ay + az * az);

    return fabs(mag - IMU_GRAVITY_MPS2) > IMU_MOTION_THRESHOLD_MPS2;
}

/* ---------------------------- Function Definitions -------------------------- */

int Imu_Init(ImuHandleType *imu)
{
    uint8_t who = 0;
    int ret;

    if (imu == NULL)
    {
        return -1;
    }

    if (!device_is_ready(imu->i2cDev))
    {
        printk("i2c0 not ready\n");
        return -1;
    }

    if (!gpio_is_ready_dt(&imu->ledGreen))
    {
        printk("green LED not ready\n");
        return -1;
    }
    gpio_pin_configure_dt(&imu->ledGreen, GPIO_OUTPUT_INACTIVE);

    ret = i2c_reg_read_byte(imu->i2cDev, IMU_I2C_ADDR, IMU_REG_WHO_AM_I, &who);
    printk("who_am_i ret=%d val=0x%02x\n", ret, who);
    if (ret != 0 || who != IMU_WHO_AM_I_VALUE)
    {
        printk("IMU not responding as expected\n");
        return -1;
    }

    i2c_reg_write_byte(imu->i2cDev, IMU_I2C_ADDR, IMU_REG_CTRL1_XL, IMU_CTRL1_XL_104HZ_2G);
    i2c_reg_write_byte(imu->i2cDev, IMU_I2C_ADDR, IMU_REG_CTRL2_G, IMU_CTRL2_G_104HZ_245DPS);
    i2c_reg_write_byte(imu->i2cDev, IMU_I2C_ADDR, IMU_REG_CTRL3_C, IMU_CTRL3_C_BDU_IFINC);

    imu->state       = IMU_STATE_IDLE;
    imu->quietStreak = 0;

    printk("State machine armed, starting in IDLE (checking every %d ms)\n",
           IMU_IDLE_POLL_MS);

    return 0;
}

int Imu_Update(ImuHandleType *imu)
{
    double ax, ay, az;
    bool movingNow;
    int ret;

    if (imu == NULL)
    {
        return -1;
    }

    ret = imu_readAccel(imu, &ax, &ay, &az);
    if (ret != 0)
    {
        printk("accel read failed: %d\n", ret);
        return -1;
    }

    movingNow = imu_isMoving(ax, ay, az);

    if (imu->state == IMU_STATE_IDLE)
    {
        if (!movingNow)
        {
            /* Still quiet -- nothing else to do this cycle. */
            return 0;
        }

        printk("[state] IDLE -> MOVING (|accel|=%.3f m/s2)\n",
               sqrt(ax * ax + ay * ay + az * az));
        imu->state       = IMU_STATE_MOVING;
        imu->quietStreak = 0;
    }

    /* IMU_STATE_MOVING: full read + print at the fast rate. */
    gpio_pin_set_dt(&imu->ledGreen, 1);

    printk("accel x:%.3f m/s2 y:%.3f m/s2 z:%.3f m/s2\n", ax, ay, az);
    imu_printGyro(imu);
    imu_printTemp(imu);

    gpio_pin_set_dt(&imu->ledGreen, 0);

    if (movingNow)
    {
        imu->quietStreak = 0;
    }
    else
    {
        imu->quietStreak++;
        if (imu->quietStreak >= IMU_IDLE_CONFIRM_SAMPLES)
        {
            printk("[state] MOVING -> IDLE (settled for %d samples)\n",
                   imu->quietStreak);
            imu->state = IMU_STATE_IDLE;
        }
    }

    return 0;
}

uint32_t Imu_GetPollIntervalMs(const ImuHandleType *imu)
{
    if (imu == NULL)
    {
        return IMU_IDLE_POLL_MS;
    }

    return (imu->state == IMU_STATE_IDLE) ? IMU_IDLE_POLL_MS : IMU_MOVING_POLL_MS;
}

void Imu_Cleanup(ImuHandleType *imu)
{
    if (imu == NULL)
    {
        return;
    }

    gpio_pin_set_dt(&imu->ledGreen, 0);
}
/**
 * ============================================================================
 * File Name   : main.c
 * Author      : 
 * Created On  : DD-MM-YYYY
 * Description : Entry point. Wires up devicetree handles for the IMU module
 *               and drives the IDLE/MOVING poll loop.
 * ============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "Imu.h"

/* GPIO_DT_SPEC_GET expands to a bare brace initializer list, so it can only
 * be used here -- in a declaration's initializer -- not as a plain
 * assignment further down. Same reasoning applies to DEVICE_DT_GET, which
 * happens to also be safe as a direct assignment, but keeping both as
 * file-scope statics is the consistent, always-safe pattern. */
static const struct device *const g_i2cDev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct gpio_dt_spec g_ledGreen = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

int main(void)
{
    ImuHandleType imu;

    imu.i2cDev   = g_i2cDev;
    imu.ledGreen = g_ledGreen;

    if (Imu_Init(&imu) != 0)
    {
        return 0;
    }

    while (1)
    {
        Imu_Update(&imu);
        k_sleep(K_MSEC(Imu_GetPollIntervalMs(&imu)));
    }

    /* Unreachable in this build, kept for symmetry with Imu_Init(). */
    Imu_Cleanup(&imu);
    return 0;
}
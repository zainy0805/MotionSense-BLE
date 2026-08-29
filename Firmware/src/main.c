#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>

#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

#define LSM6DS3_ADDR       0x6A

#define REG_WHO_AM_I       0x0F
#define REG_CTRL1_XL       0x10   /* accel ODR/scale */
#define REG_CTRL2_G        0x11   /* gyro ODR/scale  */
#define REG_CTRL3_C        0x12   /* BDU, IF_INC      */
#define REG_OUTX_L_G       0x22   /* gyro: 6 bytes from here  */
#define REG_OUTX_L_XL      0x28   /* accel: 6 bytes from here */

/* ODR=104 Hz (0100), FS=+-2g (00), BW default (00) -> 0x40 */
#define CTRL1_XL_104HZ_2G  0x40
/* ODR=104 Hz (0100), FS=245 dps (00) -> 0x40 */
#define CTRL2_G_104HZ_245DPS 0x40
/* BDU=1 (bit6), IF_INC=1 (bit2, usually already default) */
#define CTRL3_C_BDU_IFINC  0x44

/* sensitivities at these full-scale settings */
#define ACCEL_SENS_G_PER_LSB   0.000061  /* g per LSB at +-2g   */
#define GYRO_SENS_DPS_PER_LSB  0.00875   /* dps per LSB at 245  */

static int16_t to_s16(uint8_t lo, uint8_t hi)
{
	return (int16_t)((hi << 8) | lo);
}

int main(void)
{
	uint8_t who = 0, buf[6];
	int ret;

	if (!device_is_ready(i2c_dev)) {
		printk("i2c0 not ready\n");
		return 0;
	}

	if (!gpio_is_ready_dt(&led_green)) {
		printk("green LED not ready\n");
		return 0;
	}
	gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);

	ret = i2c_reg_read_byte(i2c_dev, LSM6DS3_ADDR, REG_WHO_AM_I, &who);
	printk("who_am_i ret=%d val=0x%02x\n", ret, who);
	if (ret || who != 0x6A) {
		printk("IMU not responding as expected\n");
		return 0;
	}

	i2c_reg_write_byte(i2c_dev, LSM6DS3_ADDR, REG_CTRL1_XL, CTRL1_XL_104HZ_2G);
	i2c_reg_write_byte(i2c_dev, LSM6DS3_ADDR, REG_CTRL2_G, CTRL2_G_104HZ_245DPS);
	i2c_reg_write_byte(i2c_dev, LSM6DS3_ADDR, REG_CTRL3_C, CTRL3_C_BDU_IFINC);

	while (1) {
		char out_str[96];

		gpio_pin_set_dt(&led_green, 1);   /* LED on for this read cycle */

		/* gyro: 6 bytes starting at OUTX_L_G */
		ret = i2c_burst_read(i2c_dev, LSM6DS3_ADDR, REG_OUTX_L_G, buf, 6);
		if (ret == 0) {
			double gx = to_s16(buf[0], buf[1]) * GYRO_SENS_DPS_PER_LSB;
			double gy = to_s16(buf[2], buf[3]) * GYRO_SENS_DPS_PER_LSB;
			double gz = to_s16(buf[4], buf[5]) * GYRO_SENS_DPS_PER_LSB;

			sprintf(out_str, "gyro x:%.3f dps y:%.3f dps z:%.3f dps", gx, gy, gz);
			printk("%s\n", out_str);
		} else {
			printk("gyro read failed: %d\n", ret);
		}

		/* accel: 6 bytes starting at OUTX_L_XL */
		ret = i2c_burst_read(i2c_dev, LSM6DS3_ADDR, REG_OUTX_L_XL, buf, 6);
		if (ret == 0) {
			double ax = to_s16(buf[0], buf[1]) * ACCEL_SENS_G_PER_LSB * 9.80665;
			double ay = to_s16(buf[2], buf[3]) * ACCEL_SENS_G_PER_LSB * 9.80665;
			double az = to_s16(buf[4], buf[5]) * ACCEL_SENS_G_PER_LSB * 9.80665;

			sprintf(out_str, "accel x:%.3f m/s2 y:%.3f m/s2 z:%.3f m/s2", ax, ay, az);
			printk("%s\n", out_str);
		} else {
			printk("accel read failed: %d\n", ret);
		}

		gpio_pin_set_dt(&led_green, 0);   /* LED off until next cycle */

		k_sleep(K_MSEC(500));
	}
}
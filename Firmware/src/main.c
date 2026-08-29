#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <math.h>

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

#define LSM6DS3_ADDR       0x6A

#define REG_WHO_AM_I       0x0F
#define REG_CTRL1_XL       0x10   /* accel ODR/scale */
#define REG_CTRL2_G        0x11   /* gyro ODR/scale  */
#define REG_CTRL3_C        0x12   /* BDU, IF_INC      */
#define REG_OUTX_L_G       0x22   /* gyro: 6 bytes from here  */
#define REG_OUTX_L_XL      0x28   /* accel: 6 bytes from here */
#define REG_OUT_TEMP_L     0x20   /* temp: 2 bytes from here  */

/* ODR=104 Hz (0100), FS=+-2g (00), BW default (00) -> 0x40 */
#define CTRL1_XL_104HZ_2G  0x40
/* ODR=104 Hz (0100), FS=245 dps (00) -> 0x40 */
#define CTRL2_G_104HZ_245DPS 0x40
/* BDU=1 (bit6), IF_INC=1 (bit2, usually already default) */
#define CTRL3_C_BDU_IFINC  0x44

/* sensitivities at these full-scale settings */
#define ACCEL_SENS_G_PER_LSB   0.000061  /* g per LSB at +-2g   */
#define GYRO_SENS_DPS_PER_LSB  0.00875   /* dps per LSB at 245  */

#define TEMP_LSB_PER_DEGC      256.0     /* LSB per deg C, 0 LSB @ 25 C */
#define TEMP_OFFSET_DEGC       25.0

#define GRAVITY_MPS2           9.80665

/* --- state machine tuning --- */
#define IDLE_POLL_MS           1000  /* slow check rate while nothing is happening   */
#define MOVING_POLL_MS         100   /* fast read/print rate once motion is seen     */
#define IDLE_CONFIRM_SAMPLES   10    /* consecutive quiet samples (@ MOVING_POLL_MS)
                                       * before dropping back to IDLE, i.e. ~1 s      */
#define MOTION_THRESHOLD_MPS2  1.0   /* how far |accel| must deviate from 1 g to
                                       * count as "moving" -- tune this to taste      */

enum motion_state {
	STATE_IDLE,
	STATE_MOVING,
};

static int16_t to_s16(uint8_t lo, uint8_t hi)
{
	return (int16_t)((hi << 8) | lo);
}

/* Reads accel only, returns 0 on success and fills ax/ay/az in m/s^2. */
static int read_accel(double *ax, double *ay, double *az) {
	uint8_t buf[6];
	int ret = i2c_burst_read(i2c_dev, LSM6DS3_ADDR, REG_OUTX_L_XL, buf, 6);

	if (ret) {
		return ret;
	}

	*ax = to_s16(buf[0], buf[1]) * ACCEL_SENS_G_PER_LSB * GRAVITY_MPS2;
	*ay = to_s16(buf[2], buf[3]) * ACCEL_SENS_G_PER_LSB * GRAVITY_MPS2;
	*az = to_s16(buf[4], buf[5]) * ACCEL_SENS_G_PER_LSB * GRAVITY_MPS2;
	return 0;
}

static void print_gyro(void)
{
	uint8_t buf[6];
	char out_str[96];
	int ret = i2c_burst_read(i2c_dev, LSM6DS3_ADDR, REG_OUTX_L_G, buf, 6);

	if (ret == 0) {
		double gx = to_s16(buf[0], buf[1]) * GYRO_SENS_DPS_PER_LSB;
		double gy = to_s16(buf[2], buf[3]) * GYRO_SENS_DPS_PER_LSB;
		double gz = to_s16(buf[4], buf[5]) * GYRO_SENS_DPS_PER_LSB;

		sprintf(out_str, "gyro x:%.3f dps y:%.3f dps z:%.3f dps", gx, gy, gz);
		printk("%s\n", out_str);
	} else {
		printk("gyro read failed: %d\n", ret);
	}
}

static void print_temp(void)
{
	uint8_t tbuf[2];
	char out_str[64];
	int ret = i2c_burst_read(i2c_dev, LSM6DS3_ADDR, REG_OUT_TEMP_L, tbuf, 2);

	if (ret == 0) {
		double temp_c = TEMP_OFFSET_DEGC +
				(to_s16(tbuf[0], tbuf[1]) / TEMP_LSB_PER_DEGC);

		sprintf(out_str, "temp: %.2f C", temp_c);
		printk("%s\n", out_str);
	} else {
		printk("temp read failed: %d\n", ret);
	}
}

int main(void) {
	uint8_t who = 0;
	int ret;
	enum motion_state state = STATE_IDLE;
	int quiet_streak = 0;

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

	ret = i2c_reg_write_byte(i2c_dev, LSM6DS3_ADDR, REG_CTRL1_XL, CTRL1_XL_104HZ_2G);
	ret = i2c_reg_write_byte(i2c_dev, LSM6DS3_ADDR, REG_CTRL2_G, CTRL2_G_104HZ_245DPS);
	ret = i2c_reg_write_byte(i2c_dev, LSM6DS3_ADDR, REG_CTRL3_C, CTRL3_C_BDU_IFINC);

	printk("State machine armed, starting in IDLE (checking every %d ms)\n",
	       IDLE_POLL_MS);

	while (1) {
		double ax, ay, az;

		ret = read_accel(&ax, &ay, &az);
		if (ret) {
			printk("accel read failed: %d\n", ret);
			k_sleep(K_MSEC(IDLE_POLL_MS));
			continue;
		}

		double mag = sqrt(ax * ax + ay * ay + az * az);
		bool moving_now = fabs(mag - GRAVITY_MPS2) > MOTION_THRESHOLD_MPS2;

		if (state == STATE_IDLE) {
			if (moving_now) {
				printk("[state] IDLE -> MOVING (|accel|=%.3f m/s2)\n", mag);
				state = STATE_MOVING;
				quiet_streak = 0;
			} 
			else {
				/* Nothing happening -- just the slow check, no LED,
				 * no gyro/temp reads, no per-sample print. */
				k_sleep(K_MSEC(IDLE_POLL_MS));
				continue;
			}
		}

		/* STATE_MOVING: full read + print at the fast rate. */
		gpio_pin_set_dt(&led_green, 1);

		char out_str[96];

		sprintf(out_str, "accel x:%.3f m/s2 y:%.3f m/s2 z:%.3f m/s2", ax, ay, az);
		printk("%s\n", out_str);

		print_gyro();
		print_temp();

		gpio_pin_set_dt(&led_green, 0);

		if (moving_now) {
			quiet_streak = 0;
		} 
		else {
			quiet_streak++;
			if (quiet_streak >= IDLE_CONFIRM_SAMPLES) {
				printk("[state] MOVING -> IDLE (settled for %d samples)\n", quiet_streak);
				state = STATE_IDLE;
			}
		}

		k_sleep(K_MSEC(MOVING_POLL_MS));
	}
}
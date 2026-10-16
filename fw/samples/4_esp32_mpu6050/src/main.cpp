// Reads the MPU-6050 gyro through Zephyr's sensor API and prints body rates
// at 10 Hz. The first piece of real hardware the controller will need: on
// the bench it replaces the gyro the simulator puts in the sensor frame.

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

namespace {
const device *const kImu = DEVICE_DT_GET(DT_NODELABEL(mpu6050));

// printk has no %f on every target, so rates go out as mrad/s.
int milli(const sensor_value &v) {
	return static_cast<int>(sensor_value_to_milli(&v));
}
} // namespace

int main() {
	if (!device_is_ready(kImu)) {
		printk("mpu6050 not ready, check wiring and the 0x68 address\n");
		return 0;
	}
	printk("mpu6050 ready, gyro in mrad/s (sensor axes, not yet mapped to the body frame)\n");

	for (;;) {
		sensor_value gyro[3];
		if (sensor_sample_fetch(kImu) == 0 && sensor_channel_get(kImu, SENSOR_CHAN_GYRO_XYZ, gyro) == 0) {
			printk("gyro x %6d y %6d z %6d\n", milli(gyro[0]), milli(gyro[1]), milli(gyro[2]));
		} else {
			printk("read failed\n");
		}
		k_msleep(100);
	}
}

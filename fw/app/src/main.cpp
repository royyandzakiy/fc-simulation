// Flight controller firmware, simulator-in-the-loop build.
//
// Sensor frames arrive on the link UART, sticks ride inside them, and every
// frame is answered with one motor frame. The plant owns time: dt comes from
// the frame, so the run is lockstep and reproducible, the same as the host
// build fc_sitl_cpp with --input link.

#include "fc/core.hpp"
#include "fc/link.hpp"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fc, LOG_LEVEL_INF);

namespace {

const device *const kLink = DEVICE_DT_GET(DT_CHOSEN(fc_link_uart));

// Blocks until a byte arrives. The short sleep keeps native_sim off 100% CPU
// and costs at most 50 us per idle poll.
std::uint8_t read_byte() {
	unsigned char c;
	while (uart_poll_in(kLink, &c) != 0)
		k_usleep(50);
	return c;
}

void write_frame(const fc::link::MotorBytes &bytes) {
	for (const auto b : bytes)
		uart_poll_out(kLink, b);
}

} // namespace

int main() {
	if (!device_is_ready(kLink)) {
		LOG_ERR("link uart %s not ready", kLink->name);
		return 0;
	}

	fc::LinkSticks sticks;
	sticks.configure(CONFIG_FC_ARM_THRESHOLD_PERMILLE / 1000.0f, CONFIG_FC_ARM_HOLD_MS / 1000.0f,
					 CONFIG_FC_RC_TIMEOUT_MS / 1000.0f);
	fc::Controller controller;
	fc::link::SensorReader reader;

	LOG_INF("link on %s, arm >%d/1000 held %d ms, rc timeout %d ms", kLink->name,
			CONFIG_FC_ARM_THRESHOLD_PERMILLE, CONFIG_FC_ARM_HOLD_MS, CONFIG_FC_RC_TIMEOUT_MS);

	bool was_armed = false;
	bool was_lost = true;

	for (;;) {
		const auto res = reader.push(read_byte());
		if (res == fc::link::SensorReader::Result::BadCrc) {
			LOG_WRN("rx: bad crc, dropping");
			continue;
		}
		if (res != fc::link::SensorReader::Result::Frame)
			continue;
		const auto &s = reader.frame();

		const fc::Sticks rc = sticks.poll(s.rc, s.dt);
		const fc::link::MotorFrame out{
			.seq = s.seq,
			.m = controller.step(s.gyro, rc, s.dt),
			.rc = {rc.roll, rc.pitch, rc.yaw, rc.throttle},
			.armed = rc.armed,
		};
		write_frame(fc::link::encode_motor(out));

		// Reported after the reply, so logging never delays the frame.
		if (sticks.refused())
			LOG_WRN("arm refused, throttle %d%%, bring it down first", static_cast<int>(s.rc.rc[3] * 100.0f));
		if (sticks.rc_lost() != was_lost) {
			was_lost = sticks.rc_lost();
			if (was_lost)
				LOG_WRN("no valid sticks, failsafe");
			else
				LOG_INF("sticks valid");
		}
		if (rc.armed != was_armed) {
			was_armed = rc.armed;
			LOG_INF("%s", rc.armed ? "ARMED" : "disarmed");
		}
	}
}

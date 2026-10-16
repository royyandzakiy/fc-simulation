// Answers every sensor frame with a motor frame: same seq, motors at zero,
// sticks echoed. The framing and CRC of the real firmware with no flight
// logic behind it, so link problems show up before control problems can.

#include "fc/link.hpp"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(link_echo, LOG_LEVEL_INF);

namespace {
const device *const kLink = DEVICE_DT_GET(DT_CHOSEN(fc_link_uart));
} // namespace

int main() {
	if (!device_is_ready(kLink)) {
		LOG_ERR("link uart %s not ready", kLink->name);
		return 0;
	}
	LOG_INF("answering sensor frames on %s", kLink->name);

	fc::link::SensorReader reader;
	std::uint32_t frames = 0;
	std::uint32_t bad = 0;

	for (;;) {
		unsigned char c;
		if (uart_poll_in(kLink, &c) != 0) {
			k_usleep(50);
			continue;
		}

		const auto res = reader.push(c);
		if (res == fc::link::SensorReader::Result::BadCrc) {
			LOG_WRN("bad crc, dropped (%u so far)", ++bad);
			continue;
		}
		if (res != fc::link::SensorReader::Result::Frame)
			continue;

		const auto &s = reader.frame();
		const fc::link::MotorFrame out{.seq = s.seq, .m = {}, .rc = s.rc.rc, .armed = false};
		for (const auto b : fc::link::encode_motor(out))
			uart_poll_out(kLink, b);

		if (++frames % 1000 == 0)
			LOG_INF("%u frames answered, %u dropped", frames, bad);
	}
}

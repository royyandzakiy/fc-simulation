// Runs the flight core once under Zephyr and logs what it computes.
// Same numbers on native_sim and the ESP32 means the toolchain, the C++
// library and the core agree before any I/O is involved.

#include "fc/core.hpp"
#include "fc/link.hpp"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <cstdint>

LOG_MODULE_REGISTER(hello_core, LOG_LEVEL_INF);

namespace {

// LOG_* has no %f on every target, so floats go out as thousandths.
int milli(float v) {
	return static_cast<int>(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
}

} // namespace

int main() {
	static constexpr char kCheck[] = "123456789";
	const auto crc = fc::link::crc8(reinterpret_cast<const std::uint8_t *>(kCheck), sizeof(kCheck) - 1);
	LOG_INF("crc8(\"123456789\") = 0x%02X, expect 0xBC", crc);

	const auto m = fc::Mixer::apply(0.5f, {0.1f, 0.0f, 0.0f});
	LOG_INF("mixer thr 0.5 roll +0.1 -> m = %d %d %d %d (x1000), expect 400 400 600 600", milli(m[0]),
			milli(m[1]), milli(m[2]), milli(m[3]));

	// A frame encoded and decoded through an unaligned buffer: the case that
	// faults on Xtensa if anything reads a float through a packed struct.
	fc::link::SensorFrame in{};
	in.seq = 7;
	in.gyro = {0.1f, -0.2f, 0.3f};
	static std::uint8_t buf[fc::link::kSensorLen + 1];
	const auto bytes = fc::link::encode_sensor(in);
	for (std::size_t i = 0; i < bytes.size(); ++i)
		buf[i + 1] = bytes[i];
	fc::link::SensorFrame out{};
	const bool ok = fc::link::decode_sensor(buf + 1, out);
	LOG_INF("unaligned decode %s, seq %u gyro.y %d (x1000)", ok ? "ok" : "FAILED", out.seq, milli(out.gyro[1]));

	LOG_INF("done");
	return 0;
}

// The flight core under ztest. A subset of test/ (GoogleTest on the host),
// chosen for what can differ once Zephyr's toolchain and C++ library are
// underneath: CRC and byte layout, unaligned decode, float behaviour in the
// mixer and the time-based arming and failsafe logic.

#include "fc/core.hpp"
#include "fc/link.hpp"

#include <zephyr/ztest.h>

#include <array>
#include <cstdint>

namespace {

constexpr float kDt = 1.0f / 240.0f;

fc::link::SensorFrame sample_sensor() {
	fc::link::SensorFrame f{};
	f.seq = 7;
	f.dt = kDt;
	f.gyro = {0.1f, -0.2f, 0.3f};
	f.rc.rc = {0.0f, 0.5f, -0.5f, 0.25f};
	f.rc.arm = 1.0f;
	f.rc.valid = true;
	return f;
}

// protocol.build_sensor(7, 1/240, (0.1, -0.2, 0.3), (0.0, 0.5, -0.5, 0.25), 1.0)
constexpr std::array<std::uint8_t, fc::link::kSensorLen> kPythonSensor = {
	0xa5, 0x07, 0x00, 0x00, 0x00, 0x89, 0x88, 0x88, 0x3b, 0xcd, 0xcc, 0xcc, 0x3d, 0xcd, 0xcc,
	0x4c, 0xbe, 0x9a, 0x99, 0x99, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00,
	0x00, 0x00, 0xbf, 0x00, 0x00, 0x80, 0x3e, 0x00, 0x00, 0x80, 0x3f, 0x01, 0x83,
};

void run(fc::LinkSticks &ls, const fc::RcChannels &ch, float seconds, fc::Sticks &out) {
	for (float t = 0.0f; t < seconds; t += kDt)
		out = ls.poll(ch, kDt);
}

} // namespace

ZTEST_SUITE(fc_link, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(fc_link, test_crc8_check_value) {
	static constexpr std::uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	zassert_equal(fc::link::crc8(s, sizeof(s)), 0xBC);
}

ZTEST(fc_link, test_sensor_bytes_match_python) {
	const auto bytes = fc::link::encode_sensor(sample_sensor());
	zassert_mem_equal(bytes.data(), kPythonSensor.data(), bytes.size());
}

ZTEST(fc_link, test_unaligned_decode) {
	static std::uint8_t buf[fc::link::kSensorLen + 1];
	for (std::size_t i = 0; i < kPythonSensor.size(); ++i)
		buf[i + 1] = kPythonSensor[i];
	fc::link::SensorFrame f{};
	zassert_true(fc::link::decode_sensor(buf + 1, f));
	zassert_equal(f.seq, 7u);
	zassert_equal(f.gyro[1], -0.2f);
	zassert_true(f.rc.valid);
}

ZTEST(fc_link, test_reader_drops_bad_crc) {
	fc::link::SensorReader r;
	auto bad = kPythonSensor;
	bad[20] ^= 0xFF;
	auto last = fc::link::SensorReader::Result::Pending;
	for (auto b : bad)
		last = r.push(b);
	zassert_equal(last, fc::link::SensorReader::Result::BadCrc);
	for (auto b : kPythonSensor)
		last = r.push(b);
	zassert_equal(last, fc::link::SensorReader::Result::Frame);
}

ZTEST_SUITE(fc_core, nullptr, nullptr, nullptr, nullptr, nullptr);

ZTEST(fc_core, test_mixer_roll) {
	const auto m = fc::Mixer::apply(0.5f, {0.1f, 0.0f, 0.0f});
	zassert_within(m[0], 0.4f, 1e-6f);
	zassert_within(m[2], 0.6f, 1e-6f);
}

ZTEST(fc_core, test_mixer_airmode_shift) {
	const auto m = fc::Mixer::apply(0.0f, {0.1f, 0.0f, 0.0f});
	zassert_within(m[0], 0.0f, 1e-6f);
	zassert_within(m[2] - m[0], 0.2f, 1e-6f);
}

ZTEST(fc_core, test_controller_opposes_roll) {
	fc::Controller c;
	const auto m = c.step({1.0f, 0.0f, 0.0f}, fc::Sticks{.throttle = 0.5f, .armed = true}, kDt);
	zassert_true(m[2] < m[0]);
}

ZTEST(fc_core, test_arm_hold_and_failsafe) {
	fc::LinkSticks ls;
	ls.configure(0.5f, 0.2f, 0.5f);
	const fc::RcChannels low{.rc = {0, 0, 0, 0}, .arm = -1.0f, .valid = true};
	const fc::RcChannels high{.rc = {0, 0, 0, 0}, .arm = 1.0f, .valid = true};
	fc::Sticks s{};

	run(ls, low, 0.1f, s);
	run(ls, high, 0.1f, s);
	zassert_false(s.armed, "armed before the hold time");
	run(ls, high, 0.2f, s);
	zassert_true(s.armed);

	run(ls, fc::RcChannels{}, 0.6f, s);
	zassert_false(s.armed, "still armed after RC loss");
	zassert_true(ls.rc_lost());
}

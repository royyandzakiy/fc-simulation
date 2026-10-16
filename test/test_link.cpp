#include "fc/link.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

using namespace fc::link;

namespace {

std::vector<std::uint8_t> from_hex(std::string_view hex) {
	std::vector<std::uint8_t> out;
	for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
		out.push_back(static_cast<std::uint8_t>(std::stoi(std::string(hex.substr(i, 2)), nullptr, 16)));
	}
	return out;
}

SensorFrame sample_sensor() {
	SensorFrame f{};
	f.seq = 7;
	f.dt = 1.0f / 240.0f;
	f.gyro = {0.1f, -0.2f, 0.3f};
	f.rc.rc = {0.0f, 0.5f, -0.5f, 0.25f};
	f.rc.arm = 1.0f;
	f.rc.valid = true;
	return f;
}

// protocol.build_sensor(7, 1/240, (0.1, -0.2, 0.3), (0.0, 0.5, -0.5, 0.25), 1.0)
// from simulator/common/protocol.py. Pins the two implementations together.
constexpr std::string_view kPythonSensorHex =
	"a5070000008988883bcdcccc3dcdcc4cbe9a99993e000000000000003f000000bf0000803e0000803f0183";

} // namespace

TEST(Crc8, MatchesDvbS2CheckValue) {
	// CRC-8/DVB-S2 is poly 0xD5, init 0, no reflection: its check value is 0xBC.
	constexpr std::string_view s = "123456789";
	EXPECT_EQ(crc8(reinterpret_cast<const std::uint8_t *>(s.data()), s.size()), 0xBC);
}

TEST(Link, SensorBytesMatchPython) {
	const auto bytes = encode_sensor(sample_sensor());
	const auto want = from_hex(kPythonSensorHex);
	ASSERT_EQ(want.size(), kSensorLen);
	EXPECT_TRUE(std::equal(bytes.begin(), bytes.end(), want.begin()));
}

TEST(Link, SensorRoundTrip) {
	const SensorFrame in = sample_sensor();
	SensorFrame out{};
	ASSERT_TRUE(decode_sensor(encode_sensor(in).data(), out));
	EXPECT_EQ(out.seq, in.seq);
	EXPECT_EQ(out.dt, in.dt);
	EXPECT_EQ(out.gyro, in.gyro);
	EXPECT_EQ(out.rc.rc, in.rc.rc);
	EXPECT_EQ(out.rc.arm, in.rc.arm);
	EXPECT_TRUE(out.rc.valid);
}

TEST(Link, MotorRoundTrip) {
	const MotorFrame in{.seq = 42, .m = {0.1f, 0.2f, 0.3f, 0.4f}, .rc = {0.5f, -0.5f, 0.0f, 0.6f}, .armed = true};
	MotorFrame out{};
	ASSERT_TRUE(decode_motor(encode_motor(in).data(), out));
	EXPECT_EQ(out.seq, 42u);
	EXPECT_EQ(out.m, in.m);
	EXPECT_EQ(out.rc, in.rc);
	EXPECT_TRUE(out.armed);
}

TEST(Link, DecodeRejectsBadCrcAndSync) {
	auto bytes = encode_sensor(sample_sensor());
	SensorFrame out{};

	bytes[10] ^= 0x01;
	EXPECT_FALSE(decode_sensor(bytes.data(), out));

	bytes = encode_sensor(sample_sensor());
	bytes[0] = 0x00;
	EXPECT_FALSE(decode_sensor(bytes.data(), out));
}

TEST(Link, DecodesFromUnalignedBuffer) {
	// The frame starts one byte into the buffer, so every float is misaligned.
	std::array<std::uint8_t, kSensorLen + 1> buf{};
	const auto bytes = encode_sensor(sample_sensor());
	std::copy(bytes.begin(), bytes.end(), buf.begin() + 1);
	SensorFrame out{};
	ASSERT_TRUE(decode_sensor(buf.data() + 1, out));
	EXPECT_EQ(out.gyro[1], -0.2f);
}

TEST(SensorReader, ResyncsAfterGarbage) {
	SensorReader r;
	for (std::uint8_t b : {0x00, 0x13, 0x5A, 0xFF})
		EXPECT_EQ(r.push(b), SensorReader::Result::Pending);

	const auto bytes = encode_sensor(sample_sensor());
	for (std::size_t i = 0; i + 1 < bytes.size(); ++i)
		EXPECT_EQ(r.push(bytes[i]), SensorReader::Result::Pending);
	EXPECT_EQ(r.push(bytes.back()), SensorReader::Result::Frame);
	EXPECT_EQ(r.frame().seq, 7u);
}

TEST(SensorReader, DropsBadFrameThenRecovers) {
	SensorReader r;
	auto bad = encode_sensor(sample_sensor());
	bad[20] ^= 0xFF;
	SensorReader::Result last{};
	for (auto b : bad)
		last = r.push(b);
	EXPECT_EQ(last, SensorReader::Result::BadCrc);

	SensorFrame next = sample_sensor();
	next.seq = 8;
	for (auto b : encode_sensor(next))
		last = r.push(b);
	EXPECT_EQ(last, SensorReader::Result::Frame);
	EXPECT_EQ(r.frame().seq, 8u);
}

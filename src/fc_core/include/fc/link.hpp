// Simulator link: sensor frames in, motor frames out, CRC-8 framed.
// Mirrored in simulator/common/protocol.py; change both together.
//
//   sensor  43 B  0xA5  seq u32, dt f32, gyro[3] f32, rc[4] f32, arm f32, flags u8, crc u8
//   motor   39 B  0x5A  seq u32, m[4] f32, rc[4] f32, armed u8, crc u8
//
// Fields are copied in and out with memcpy at fixed offsets rather than
// through a packed struct. A reference into a packed struct carries no
// alignment information, and an unaligned float load faults on Xtensa and
// on Cortex-M with an FPU.

#pragma once

#include "fc/core.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace fc::link {

// Floats go on the wire in native byte order, so both ends must be little-endian.
static_assert(std::endian::native == std::endian::little, "link format is little-endian");

inline constexpr std::uint8_t kSyncSensor = 0xA5;
inline constexpr std::uint8_t kSyncMotor = 0x5A;
inline constexpr std::uint8_t kCrcPoly = 0xD5; // CRC-8, same as CRSF

// Set when rc[] and arm carry a real radio's values. Without it, "no radio"
// and "sticks centred, arm switch down" are the same bytes.
inline constexpr std::uint8_t kFlagSticksValid = 0x01;

inline constexpr std::size_t kSensorLen = 43;
inline constexpr std::size_t kMotorLen = 39;

using SensorBytes = std::array<std::uint8_t, kSensorLen>;
using MotorBytes = std::array<std::uint8_t, kMotorLen>;

// CRC-8, poly 0xD5, init 0, no reflection, no final xor.
constexpr std::uint8_t crc8(const std::uint8_t *p, std::size_t n) noexcept {
	std::uint8_t c = 0;
	while (n--) {
		c ^= *p++;
		for (int i = 0; i < 8; ++i) {
			c = (c & 0x80) ? static_cast<std::uint8_t>((c << 1) ^ kCrcPoly) : static_cast<std::uint8_t>(c << 1);
		}
	}
	return c;
}

struct SensorFrame {
	std::uint32_t seq{0};
	float dt{0.0f};				  // seconds since the last frame
	std::array<float, 3> gyro{};  // body rates, rad/s
	RcChannels rc{};			  // rc.valid is the kFlagSticksValid bit
};

struct MotorFrame {
	std::uint32_t seq{0};		 // echoed from the sensor frame
	std::array<float, 4> m{};	 // 0..1 each
	std::array<float, 4> rc{};	 // sticks after gating, for plotting
	bool armed{false};
};

namespace detail {

template <typename T> inline void put(std::uint8_t *buf, std::size_t off, const T &v) noexcept {
	std::memcpy(buf + off, &v, sizeof(T));
}

template <typename T> inline T get(const std::uint8_t *buf, std::size_t off) noexcept {
	T v;
	std::memcpy(&v, buf + off, sizeof(T));
	return v;
}

// Byte offsets. Each frame ends in its crc byte.
inline constexpr std::size_t kSeq = 1;
inline constexpr std::size_t kSensorDt = 5;
inline constexpr std::size_t kSensorGyro = 9;
inline constexpr std::size_t kSensorRc = 21;
inline constexpr std::size_t kSensorArm = 37;
inline constexpr std::size_t kSensorFlags = 41;
inline constexpr std::size_t kMotorM = 5;
inline constexpr std::size_t kMotorRc = 21;
inline constexpr std::size_t kMotorArmed = 37;

static_assert(kSensorFlags + 2 == kSensorLen);
static_assert(kMotorArmed + 2 == kMotorLen);

} // namespace detail

inline SensorBytes encode_sensor(const SensorFrame &f) noexcept {
	using namespace detail;
	SensorBytes b{};
	b[0] = kSyncSensor;
	put(b.data(), kSeq, f.seq);
	put(b.data(), kSensorDt, f.dt);
	for (std::size_t i = 0; i < 3; ++i)
		put(b.data(), kSensorGyro + 4 * i, f.gyro[i]);
	for (std::size_t i = 0; i < 4; ++i)
		put(b.data(), kSensorRc + 4 * i, f.rc.rc[i]);
	put(b.data(), kSensorArm, f.rc.arm);
	b[kSensorFlags] = f.rc.valid ? kFlagSticksValid : 0;
	b[kSensorLen - 1] = crc8(b.data(), kSensorLen - 1);
	return b;
}

// False on a bad sync byte or CRC; out is left untouched then.
inline bool decode_sensor(const std::uint8_t *b, SensorFrame &out) noexcept {
	using namespace detail;
	if (b[0] != kSyncSensor || b[kSensorLen - 1] != crc8(b, kSensorLen - 1))
		return false;
	out.seq = get<std::uint32_t>(b, kSeq);
	out.dt = get<float>(b, kSensorDt);
	for (std::size_t i = 0; i < 3; ++i)
		out.gyro[i] = get<float>(b, kSensorGyro + 4 * i);
	for (std::size_t i = 0; i < 4; ++i)
		out.rc.rc[i] = get<float>(b, kSensorRc + 4 * i);
	out.rc.arm = get<float>(b, kSensorArm);
	out.rc.valid = (b[kSensorFlags] & kFlagSticksValid) != 0;
	return true;
}

inline MotorBytes encode_motor(const MotorFrame &f) noexcept {
	using namespace detail;
	MotorBytes b{};
	b[0] = kSyncMotor;
	put(b.data(), kSeq, f.seq);
	for (std::size_t i = 0; i < 4; ++i)
		put(b.data(), kMotorM + 4 * i, f.m[i]);
	for (std::size_t i = 0; i < 4; ++i)
		put(b.data(), kMotorRc + 4 * i, f.rc[i]);
	b[kMotorArmed] = f.armed ? 1u : 0u;
	b[kMotorLen - 1] = crc8(b.data(), kMotorLen - 1);
	return b;
}

inline bool decode_motor(const std::uint8_t *b, MotorFrame &out) noexcept {
	using namespace detail;
	if (b[0] != kSyncMotor || b[kMotorLen - 1] != crc8(b, kMotorLen - 1))
		return false;
	out.seq = get<std::uint32_t>(b, kSeq);
	for (std::size_t i = 0; i < 4; ++i)
		out.m[i] = get<float>(b, kMotorM + 4 * i);
	for (std::size_t i = 0; i < 4; ++i)
		out.rc[i] = get<float>(b, kMotorRc + 4 * i);
	out.armed = b[kMotorArmed] != 0;
	return true;
}

// Assembles sensor frames from a byte stream, one byte at a time, so it
// works the same behind a pipe read, a UART poll or a UART interrupt.
// Waits for the sync byte, collects a full frame, then checks the CRC.
// A bad frame is dropped whole and the search starts again after it.
class SensorReader {
  public:
	enum class Result {
		Pending,
		Frame,
		BadCrc
	};

	Result push(std::uint8_t byte) noexcept {
		if (n_ == 0 && byte != kSyncSensor)
			return Result::Pending;
		buf_[n_++] = byte;
		if (n_ < kSensorLen)
			return Result::Pending;
		n_ = 0;
		return decode_sensor(buf_.data(), frame_) ? Result::Frame : Result::BadCrc;
	}

	const SensorFrame &frame() const noexcept {
		return frame_;
	}

  private:
	SensorBytes buf_{};
	std::size_t n_{0};
	SensorFrame frame_{};
};

} // namespace fc::link

// fc_core.hpp - flight controller core. No dependencies beyond the freestanding
// subset of the standard library: no heap, no exceptions, no RTTI, no iostream,
// no SDL, no fmt. This header compiles for Zephyr on Cortex-M33 unchanged.
//
// Everything platform-specific lives in fc_min.cpp, above this line.
//
// Frames are ENU body (x forward, y left, z up). Geometry matches
// gym-pybullet-drones CF2X (assets/cf2x.urdf, commit 7ebad1e).

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace fc {

// ===================================================================
// wire format
// ===================================================================

inline constexpr std::uint8_t kSyncSensor = 0xA5;
inline constexpr std::uint8_t kSyncMotor = 0x5A;
inline constexpr std::uint8_t kCrcPoly = 0xD5; // CRC-8, same as CRSF

struct [[gnu::packed]] SensorPacket {
	std::uint8_t sync; // 0xA5
	std::uint32_t seq;
	float dt;				   // seconds since last packet
	std::array<float, 3> gyro; // body rates, rad/s
	std::uint8_t crc;
};

struct [[gnu::packed]] MotorPacket {
	std::uint8_t sync;		 // 0x5A
	std::uint32_t seq;		 // echoed back
	std::array<float, 4> m;	 // 0..1 each
	std::array<float, 4> rc; // echoed sticks, for plotting
	std::uint8_t armed;
	std::uint8_t crc;
};

static_assert(sizeof(SensorPacket) == 22, "SensorPacket size");
static_assert(sizeof(MotorPacket) == 39, "MotorPacket size");

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

// crc covers every byte but the trailing crc field itself
template <typename Packet> inline std::uint8_t packet_crc(const Packet &p) noexcept {
	return crc8(reinterpret_cast<const std::uint8_t *>(&p), sizeof(Packet) - 1);
}

// ===================================================================
// stick input, source-agnostic: gamepad here, CRSF on hardware
// ===================================================================

struct Sticks {
	float roll{0.0f};	  // -1..1
	float pitch{0.0f};	  // -1..1
	float yaw{0.0f};	  // -1..1
	float throttle{0.0f}; //  0..1
	bool armed{false};
};

// ===================================================================
// rate PID
// ===================================================================

class RatePid {
  public:
	constexpr RatePid(float kp, float ki, float kd) noexcept : kp_{kp}, ki_{ki}, kd_{kd} {
	}

	float step(float err, float dt) noexcept {
		integ_ += err * dt;
		if (integ_ > kILimit)
			integ_ = kILimit;
		if (integ_ < -kILimit)
			integ_ = -kILimit;

		const float d = (dt > 1e-6f) ? (err - prev_err_) / dt : 0.0f;
		prev_err_ = err;

		return kp_ * err + ki_ * integ_ + kd_ * d;
	}

	void reset() noexcept {
		integ_ = 0.0f;
		prev_err_ = 0.0f;
	}

  private:
	static constexpr float kILimit = 0.3f;
	float kp_, ki_, kd_;
	float integ_{0.0f};
	float prev_err_{0.0f};
};

// ===================================================================
// mixer - quad X, CF2X layout
//
//   prop0 (+x,-y) front-right      prop3 (+x,+y) front-left
//   prop1 (-x,-y) rear-right       prop2 (-x,+y) rear-left
//
//   tau_x = sum(y_i * f_i),  tau_y = -sum(x_i * f_i)
//   yaw signs follow BaseAviary::_physics():  z_torque = -t0 +t1 -t2 +t3
// ===================================================================

class Mixer {
  public:
	static constexpr std::array<std::array<float, 3>, 4> kTable = {{
		{{-1.0f, -1.0f, -1.0f}}, // m0 front-right
		{{-1.0f, +1.0f, +1.0f}}, // m1 rear-right
		{{+1.0f, +1.0f, -1.0f}}, // m2 rear-left
		{{+1.0f, -1.0f, +1.0f}}, // m3 front-left
	}};

	static std::array<float, 4> apply(float thr, const std::array<float, 3> &pid) noexcept {
		std::array<float, 4> m{};
		for (std::size_t i = 0; i < 4; ++i) {
			m[i] = thr + kTable[i][0] * pid[0] + kTable[i][1] * pid[1] + kTable[i][2] * pid[2];
		}

		// airmode-lite: shift the set rather than clipping one motor, so
		// attitude authority survives at low and high throttle
		float lo = m[0], hi = m[0];
		for (std::size_t i = 1; i < 4; ++i) {
			if (m[i] < lo)
				lo = m[i];
			if (m[i] > hi)
				hi = m[i];
		}
		float shift = 0.0f;
		if (lo < 0.0f)
			shift = -lo;
		else if (hi > 1.0f)
			shift = 1.0f - hi;

		for (auto &v : m) {
			v += shift;
			if (v < 0.0f)
				v = 0.0f;
			if (v > 1.0f)
				v = 1.0f;
		}
		return m;
	}
};

// ===================================================================
// Controller - pure logic, zero I/O. Unit-testable, Zephyr-portable.
// ===================================================================

class Controller {
  public:
	static constexpr float kMaxRateRps = 6.0f; // full stick -> rad/s

	MotorPacket step(const SensorPacket &s, const Sticks &rc) noexcept {
		MotorPacket out{};
		out.sync = kSyncMotor;
		out.seq = s.seq;
		out.rc = {rc.roll, rc.pitch, rc.yaw, rc.throttle};
		out.armed = rc.armed ? 1u : 0u;

		if (!rc.armed) {
			reset();
			out.m = {0.0f, 0.0f, 0.0f, 0.0f};
		} else {
			const std::array<float, 3> sp = {rc.roll, rc.pitch, rc.yaw};
			std::array<float, 3> pid{};
			for (std::size_t a = 0; a < 3; ++a) {
				pid[a] = pid_[a].step(sp[a] * kMaxRateRps - s.gyro[a], s.dt);
			}
			out.m = Mixer::apply(rc.throttle, pid);
		}

		out.crc = packet_crc(out);
		return out;
	}

	void reset() noexcept {
		for (auto &p : pid_)
			p.reset();
	}

  private:
	std::array<RatePid, 3> pid_ = {{
		RatePid{0.10f, 0.20f, 0.0020f}, // roll
		RatePid{0.10f, 0.20f, 0.0020f}, // pitch
		RatePid{0.20f, 0.10f, 0.0000f}, // yaw
	}};
};

// ===================================================================
// arming policy - also pure logic, so it is testable and portable
// ===================================================================

class ArmingGate {
  public:
	static constexpr float kMaxArmThrottle = 0.15f;

	// hold_arm: the arm control is held. panic: a hard-disarm control.
	bool update(bool hold_arm, bool panic, float throttle) noexcept {
		if (panic || !hold_arm)
			armed_ = false;
		else if (!armed_ && throttle <= kMaxArmThrottle)
			armed_ = true;
		return armed_;
	}

	void force_disarm() noexcept {
		armed_ = false;
	}
	bool armed() const noexcept {
		return armed_;
	}

  private:
	bool armed_{false};
};

} // namespace fc

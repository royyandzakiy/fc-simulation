// fc_core.hpp - flight controller core. No dependencies beyond the freestanding
// subset of the standard library: no heap, no exceptions, no RTTI, no iostream,
// no SDL, no fmt. Compiler- and OS-agnostic: builds under clang-cl, MSVC, GCC,
// Clang, and arm-none-eabi for Zephyr on Cortex-M33, all unchanged.
//
// Everything platform-specific lives in platform.hpp and fc_min.cpp.
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

// #pragma pack rather than [[gnu::packed]]: MSVC ignores the attribute
// outright, which would silently reintroduce padding and desync the wire
// format from bridge.py. This one spelling is honoured by clang-cl, MSVC,
// GCC and Clang alike.
#pragma pack(push, 1)

struct SensorPacket {
	std::uint8_t sync; // 0xA5
	std::uint32_t seq;
	float dt;				   // seconds since last packet
	std::array<float, 3> gyro; // body rates, rad/s
	std::uint8_t crc;
};

struct MotorPacket {
	std::uint8_t sync;		 // 0x5A
	std::uint32_t seq;		 // echoed back
	std::array<float, 4> m;	 // 0..1 each
	std::array<float, 4> rc; // echoed sticks, for plotting
	std::uint8_t armed;
	std::uint8_t crc;
};

#pragma pack(pop)

// Fire at compile time if any toolchain reintroduces padding.
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

	// Setpoint and measurement separately, rather than a pre-computed error,
	// because the D term needs the measurement on its own. See below.
	float step(float setpoint, float meas, float dt) noexcept {
		const float err = setpoint - meas;

		integ_ += err * dt;
		if (integ_ > kILimit)
			integ_ = kILimit;
		if (integ_ < -kILimit)
			integ_ = -kILimit;

		// Derivative on the MEASUREMENT, not on the error. The two agree
		// while the setpoint is steady, but differentiating the error puts a
		// spike of (stick_step / dt) through D the instant a stick moves,
		// which slams all four motors to their rails for one cycle.
		// Betaflight does the same thing for the same reason.
		float d = 0.0f;
		if (have_prev_ && dt > 1e-6f) {
			d = -(meas - prev_meas_) / dt;
		}
		prev_meas_ = meas;
		have_prev_ = true;

		// Low-pass the derivative. Unfiltered it amplifies step-to-step gyro
		// noise until the loop oscillates at the sample rate - which reads as
		// a perfectly steady rate unless you look at consecutive samples.
		d_filt_ += kDLpf * (d - d_filt_);

		return kp_ * err + ki_ * integ_ + kd_ * d_filt_;
	}

	void reset() noexcept {
		integ_ = 0.0f;
		prev_meas_ = 0.0f;
		have_prev_ = false;
		d_filt_ = 0.0f;
	}

  private:
	static constexpr float kILimit = 0.3f;
	static constexpr float kDLpf = 0.08f; // fraction of the new sample kept
	float kp_, ki_, kd_;
	float integ_{0.0f};
	float prev_meas_{0.0f};
	bool have_prev_{false};
	float d_filt_{0.0f};
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
				pid[a] = pid_[a].step(sp[a] * kMaxRateRps, s.gyro[a], s.dt);
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

	// arm_btn: the arm control. panic: a hard-disarm control.
	//
	// The arm control TOGGLES on the press edge: press to arm, press again to
	// disarm. Not hold-to-arm - holding a shoulder button for a whole flight
	// is not flyable, and a tap would otherwise arm on the press and disarm
	// again on the release.
	//
	// Edge-triggered, not level: at 240 Hz a held button would otherwise flip
	// the state 240 times a second, and whether you ended up armed would
	// depend on how long you held it.
	bool update(bool arm_btn, bool panic, float throttle) noexcept {
		const bool pressed = arm_btn && !prev_btn_;
		prev_btn_ = arm_btn;
		refused_ = false;

		if (panic) {
			armed_ = false;
		} else if (pressed) {
			if (armed_) {
				armed_ = false;
			} else if (throttle <= kMaxArmThrottle) {
				armed_ = true;
			} else {
				// Refused: arming with the throttle already up would spin the
				// motors to that setting the instant it takes.
				refused_ = true;
			}
		}
		return armed_;
	}

	void force_disarm() noexcept {
		armed_ = false;
	}
	bool armed() const noexcept {
		return armed_;
	}
	// True for the one cycle an arm request was rejected, so the caller can
	// say so. Kept out of the gate itself: this class does no I/O.
	bool refused() const noexcept {
		return refused_;
	}

  private:
	bool armed_{false};
	bool prev_btn_{false};
	bool refused_{false};
};

} // namespace fc

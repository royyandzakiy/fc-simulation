// Flight logic: rate PID, mixer, arming and stick handling. No I/O, no heap,
// no exceptions and no wire types, so the same header builds for the host,
// Zephyr native_sim and the ESP32. The frame codec lives in fc/link.hpp.
//
// Frames are ENU body (x forward, y left, z up). Geometry matches
// gym-pybullet-drones CF2X (assets/cf2x.urdf, commit 7ebad1e).

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace fc {

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

// Raw channels as a receiver or the simulator delivers them, before any
// arming decision. arm is the unthresholded switch channel.
struct RcChannels {
	std::array<float, 4> rc{}; // roll, pitch, yaw in -1..1; throttle in 0..1
	float arm{-1.0f};		   // -1..1
	bool valid{false};		   // fresh data this cycle
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
		float d = 0.0f;
		if (have_prev_ && dt > 1e-6f) {
			d = -(meas - prev_meas_) / dt;
		}
		prev_meas_ = meas;
		have_prev_ = true;

		// Low-pass the derivative. Unfiltered it amplifies step-to-step gyro
		// noise until the loop oscillates at the sample rate.
		d_filt_ += kDLpf * (d - d_filt_);

		return kp_ * err + ki_ * integ_ + kd_ * d_filt_;
	}

	void reset() noexcept {
		integ_ = 0.0f;
		prev_meas_ = 0.0f;
		have_prev_ = false;
		d_filt_ = 0.0f;
	}

	static constexpr float kILimit = 0.3f;

  private:
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
// Controller - acro rate loop. Takes body rates, returns motor demands.
// ===================================================================

class Controller {
  public:
	static constexpr float kMaxRateRps = 6.0f; // full stick -> rad/s

	// gyro: body rates in rad/s. Returns four motor demands, 0..1 each.
	std::array<float, 4> step(const std::array<float, 3> &gyro, const Sticks &rc, float dt) noexcept {
		if (!rc.armed) {
			reset();
			return {0.0f, 0.0f, 0.0f, 0.0f};
		}

		const std::array<float, 3> sp = {rc.roll, rc.pitch, rc.yaw};
		std::array<float, 3> pid{};
		for (std::size_t a = 0; a < 3; ++a) {
			pid[a] = pid_[a].step(sp[a] * kMaxRateRps, gyro[a], dt);
		}
		return Mixer::apply(rc.throttle, pid);
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
// arming policy
// ===================================================================

class ArmingGate {
  public:
	static constexpr float kMaxArmThrottle = 0.15f;

	// arm_btn: the arm control. panic: a hard-disarm control.
	//
	// The arm control TOGGLES on the press edge: press to arm, press again to
	// disarm. Holding a button for a whole flight is not flyable, and a level
	// check at 240 Hz would flip the state every cycle the button is held.
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
				// Arming with the throttle up would spin the motors to that
				// setting the instant it takes.
				refused_ = true;
			}
		}
		return armed_;
	}

	// Level-triggered arming, for a physical switch rather than a button.
	//
	// Switch up is armed, switch down is disarmed. Toggling on the edge would
	// let the software and the switch get out of sync. The throttle gate
	// applies to the arming transition only, after arm_hold seconds up.
	void set_arm_hold(float seconds) noexcept {
		arm_hold_s_ = (seconds > 0.0f) ? seconds : 0.0f;
	}

	bool update_level(bool arm_sw, bool panic, float throttle, float dt) noexcept {
		refused_ = false;

		if (panic || !arm_sw) {
			// Disarm is immediate and never debounced: a delay here keeps the
			// motors running after the pilot has flipped the switch off.
			armed_ = false;
			seen_low_ = true;
			high_for_ = 0.0f;
			decided_ = false;
			return armed_;
		}

		// Require the switch to stay up before arming, so a bouncing channel
		// cannot flicker the motors on and off.
		high_for_ += dt;
		if (armed_ || decided_ || high_for_ < arm_hold_s_) {
			return armed_;
		}

		// Decide once per flip, so a refusal is reported once, not every cycle.
		decided_ = true;
		if (!seen_low_) {
			// The switch was already up when the link came alive. Arming here
			// would spin the motors the instant the aircraft was plugged in.
			refused_ = true;
		} else if (throttle <= kMaxArmThrottle) {
			armed_ = true;
		} else {
			refused_ = true;
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
	// say so. This class does no I/O.
	bool refused() const noexcept {
		return refused_;
	}

  private:
	bool armed_{false};
	bool prev_btn_{false};	 // update(), edge-triggered button
	float arm_hold_s_{0.2f}; // switch must stay up this long to arm
	float high_for_{0.0f};	 // how long it has been up
	bool decided_{false};	 // this flip has already been judged
	bool seen_low_{false};	 // switch observed down at least once since boot
	bool refused_{false};
};

// ===================================================================
// Schmitt trigger on an analogue arm channel, feeding the gate above.
//
// A plain "> threshold" compare chatters when the axis rests near the
// threshold. Two levels with a gap between them means noise smaller than
// the gap cannot cross both.
// ===================================================================

class ArmSwitch {
  public:
	// Gap either side of the threshold, in axis units. Wide enough to swallow
	// channel noise, far narrower than a real switch throw.
	static constexpr float kDefaultHysteresis = 0.15f;

	void configure(float threshold, float hold_s, float hysteresis = kDefaultHysteresis) noexcept {
		threshold_ = threshold;
		hyst_ = (hysteresis > 0.0f) ? hysteresis : 0.0f;
		gate_.set_arm_hold(hold_s);
	}

	bool update(float raw_axis, bool panic, float throttle, float dt) noexcept {
		if (raw_axis > threshold_ + hyst_) {
			high_ = true;
		} else if (raw_axis < threshold_ - hyst_) {
			high_ = false;
		} // in between: hold whatever it was

		return gate_.update_level(high_, panic, throttle, dt);
	}

	void force_disarm() noexcept {
		gate_.force_disarm();
	}
	bool armed() const noexcept {
		return gate_.armed();
	}
	bool refused() const noexcept {
		return gate_.refused();
	}

  private:
	ArmingGate gate_{};
	bool high_{false};
	float threshold_{0.5f};
	float hyst_{kDefaultHysteresis};
};

// ===================================================================
// RC-loss watchdog. kick() on every fresh RC frame; expired() once none
// has arrived for the timeout. Time comes from the caller's dt, so it
// runs the same in lockstep simulation and on a timer.
// ===================================================================

class RcWatchdog {
  public:
	void set_timeout(float seconds) noexcept {
		timeout_s_ = (seconds > 0.0f) ? seconds : 0.0f;
	}

	void kick() noexcept {
		since_ = 0.0f;
		ever_ = true;
	}

	// Starts expired: no RC since boot is the same as RC lost.
	bool update(float dt) noexcept {
		since_ += dt;
		return expired();
	}

	bool expired() const noexcept {
		return !ever_ || since_ > timeout_s_;
	}

  private:
	float timeout_s_{0.5f};
	float since_{0.0f};
	bool ever_{false};
};

// ===================================================================
// Sticks from raw channels: a receiver on hardware, the sensor frame in
// simulation. Owns the arm switch and the RC-loss failsafe, so every
// stick source that is not a local gamepad goes through the same code.
// ===================================================================

class LinkSticks {
  public:
	void configure(float arm_threshold, float arm_hold_s, float rc_timeout_s) noexcept {
		arm_.configure(arm_threshold, arm_hold_s);
		dog_.set_timeout(rc_timeout_s);
	}

	Sticks poll(const RcChannels &ch, float dt) noexcept {
		if (ch.valid) {
			last_ = ch;
			dog_.kick();
		}
		rc_lost_ = dog_.update(ch.valid ? 0.0f : dt);

		Sticks s{};
		if (rc_lost_) {
			// Failsafe: disarm and drop the sticks. Disarming needs the switch
			// seen down again before it re-arms, same as at boot.
			arm_.force_disarm();
			return s;
		}

		s.roll = clamp11(last_.rc[0]);
		s.pitch = clamp11(last_.rc[1]);
		s.yaw = clamp11(last_.rc[2]);
		s.throttle = clamp01(last_.rc[3]);

		s.armed = arm_.update(clamp11(last_.arm), false, s.throttle, dt);
		if (!s.armed)
			s.throttle = 0.0f;
		return s;
	}

	bool refused() const noexcept {
		return arm_.refused();
	}
	bool rc_lost() const noexcept {
		return rc_lost_;
	}

  private:
	static float clamp01(float v) noexcept {
		return (v < 0.0f) ? 0.0f : (v > 1.0f) ? 1.0f : v;
	}
	static float clamp11(float v) noexcept {
		return (v < -1.0f) ? -1.0f : (v > 1.0f) ? 1.0f : v;
	}

	ArmSwitch arm_{};
	RcWatchdog dog_{};
	RcChannels last_{};
	bool rc_lost_{true};
};

} // namespace fc

#include "fc/core.hpp"

#include <gtest/gtest.h>

#include <array>

using namespace fc;

namespace {
constexpr float kDt = 1.0f / 240.0f;

// Runs the switch for `seconds` at a fixed input and returns the last result.
bool hold(ArmSwitch &sw, float axis, float throttle, float seconds) {
	bool armed = false;
	for (float t = 0.0f; t < seconds; t += kDt)
		armed = sw.update(axis, false, throttle, kDt);
	return armed;
}
} // namespace

// ------------------------------------------------------------------- mixer

TEST(Mixer, ZeroCorrectionGivesThrottleOnAllMotors) {
	const auto m = Mixer::apply(0.5f, {0.0f, 0.0f, 0.0f});
	for (float v : m)
		EXPECT_FLOAT_EQ(v, 0.5f);
}

TEST(Mixer, PositiveRollRaisesLeftMotors) {
	const auto m = Mixer::apply(0.5f, {0.1f, 0.0f, 0.0f});
	EXPECT_GT(m[2], m[0]); // rear-left above front-right
	EXPECT_GT(m[3], m[1]); // front-left above rear-right
}

TEST(Mixer, AirmodeShiftKeepsDifferentialAtZeroThrottle) {
	const auto m = Mixer::apply(0.0f, {0.1f, 0.0f, 0.0f});
	EXPECT_FLOAT_EQ(m[0], 0.0f);
	EXPECT_FLOAT_EQ(m[2] - m[0], 0.2f);
}

TEST(Mixer, ClampsToUnitRange) {
	const auto m = Mixer::apply(1.0f, {2.0f, 2.0f, 2.0f});
	for (float v : m) {
		EXPECT_GE(v, 0.0f);
		EXPECT_LE(v, 1.0f);
	}
}

// --------------------------------------------------------------------- pid

TEST(RatePid, SetpointStepGivesNoDerivativeKick) {
	RatePid pid{0.0f, 0.0f, 1.0f};
	pid.step(0.0f, 0.0f, kDt);
	EXPECT_FLOAT_EQ(pid.step(10.0f, 0.0f, kDt), 0.0f);
}

TEST(RatePid, IntegralIsClamped) {
	RatePid pid{0.0f, 1.0f, 0.0f};
	float out = 0.0f;
	for (int i = 0; i < 10000; ++i)
		out = pid.step(1.0f, 0.0f, kDt);
	EXPECT_FLOAT_EQ(out, RatePid::kILimit);
}

TEST(Controller, DisarmedOutputsZero) {
	Controller c;
	const auto m = c.step({1.0f, 1.0f, 1.0f}, Sticks{.throttle = 0.8f, .armed = false}, kDt);
	for (float v : m)
		EXPECT_EQ(v, 0.0f);
}

TEST(Controller, ArmedAtRestOutputsThrottle) {
	Controller c;
	const auto m = c.step({0.0f, 0.0f, 0.0f}, Sticks{.throttle = 0.6f, .armed = true}, kDt);
	for (float v : m)
		EXPECT_FLOAT_EQ(v, 0.6f);
}

TEST(Controller, OpposesMeasuredRoll) {
	// Rolling positive with sticks centred: the left motors must come down.
	Controller c;
	const auto m = c.step({1.0f, 0.0f, 0.0f}, Sticks{.throttle = 0.5f, .armed = true}, kDt);
	EXPECT_LT(m[2], m[0]);
}

// ------------------------------------------------------------- arming gate

TEST(ArmingGate, TogglesOnPressEdge) {
	ArmingGate g;
	EXPECT_TRUE(g.update(true, false, 0.0f));
	EXPECT_TRUE(g.update(true, false, 0.0f)); // still held: no second toggle
	EXPECT_TRUE(g.update(false, false, 0.0f));
	EXPECT_FALSE(g.update(true, false, 0.0f));
}

TEST(ArmingGate, RefusesWithThrottleUp) {
	ArmingGate g;
	EXPECT_FALSE(g.update(true, false, 0.5f));
	EXPECT_TRUE(g.refused());
}

TEST(ArmingGate, PanicDisarms) {
	ArmingGate g;
	g.update(true, false, 0.0f);
	EXPECT_FALSE(g.update(false, true, 0.0f));
}

// -------------------------------------------------------------- arm switch

TEST(ArmSwitch, ArmsAfterHoldOnly) {
	ArmSwitch sw;
	sw.configure(0.5f, 0.2f);
	hold(sw, -1.0f, 0.0f, 0.1f);
	EXPECT_FALSE(hold(sw, 1.0f, 0.0f, 0.1f));
	EXPECT_TRUE(hold(sw, 1.0f, 0.0f, 0.2f));
}

TEST(ArmSwitch, DisarmIsImmediate) {
	ArmSwitch sw;
	sw.configure(0.5f, 0.2f);
	hold(sw, -1.0f, 0.0f, 0.1f);
	ASSERT_TRUE(hold(sw, 1.0f, 0.0f, 0.5f));
	EXPECT_FALSE(sw.update(-1.0f, false, 0.0f, kDt));
}

TEST(ArmSwitch, RefusesIfUpAtBoot) {
	ArmSwitch sw;
	sw.configure(0.5f, 0.2f);
	EXPECT_FALSE(hold(sw, 1.0f, 0.0f, 1.0f));
	hold(sw, -1.0f, 0.0f, 0.1f);
	EXPECT_TRUE(hold(sw, 1.0f, 0.0f, 0.5f));
}

TEST(ArmSwitch, HysteresisHoldsState) {
	ArmSwitch sw;
	sw.configure(0.5f, 0.0f, 0.15f);
	hold(sw, -1.0f, 0.0f, 0.1f);
	ASSERT_TRUE(hold(sw, 1.0f, 0.0f, 0.1f));
	EXPECT_TRUE(hold(sw, 0.4f, 0.0f, 0.1f)); // inside the band: stays armed
	EXPECT_FALSE(hold(sw, 0.3f, 0.0f, 0.1f));
}

// ---------------------------------------------------------- rc watchdog

TEST(RcWatchdog, StartsExpiredAndTimesOut) {
	RcWatchdog d;
	d.set_timeout(0.5f);
	EXPECT_TRUE(d.expired());
	d.kick();
	EXPECT_FALSE(d.update(0.4f));
	EXPECT_TRUE(d.update(0.2f));
}

// ------------------------------------------------------------ link sticks

TEST(LinkSticks, NeverArmsWithoutValidSticks) {
	LinkSticks ls;
	ls.configure(0.5f, 0.2f, 0.5f);
	RcChannels ch{.rc = {0, 0, 0, 0}, .arm = 1.0f, .valid = false};
	Sticks s{};
	for (int i = 0; i < 480; ++i)
		s = ls.poll(ch, kDt);
	EXPECT_FALSE(s.armed);
	EXPECT_TRUE(ls.rc_lost());
}

TEST(LinkSticks, FailsafeDisarmsAndNeedsSwitchCycle) {
	LinkSticks ls;
	ls.configure(0.5f, 0.2f, 0.5f);
	RcChannels low{.rc = {0, 0, 0, 0}, .arm = -1.0f, .valid = true};
	RcChannels high{.rc = {0, 0, 0, 0}, .arm = 1.0f, .valid = true};
	RcChannels none{};

	for (int i = 0; i < 24; ++i)
		ls.poll(low, kDt);
	Sticks s{};
	for (int i = 0; i < 120; ++i)
		s = ls.poll(high, kDt);
	ASSERT_TRUE(s.armed);

	// Short dropout under the timeout: keeps flying on the last sticks.
	for (int i = 0; i < 60; ++i)
		s = ls.poll(none, kDt);
	EXPECT_TRUE(s.armed);

	for (int i = 0; i < 120; ++i)
		s = ls.poll(none, kDt);
	EXPECT_FALSE(s.armed);
	EXPECT_TRUE(ls.rc_lost());

	// Link back with the switch still up: stays disarmed until it is cycled.
	for (int i = 0; i < 120; ++i)
		s = ls.poll(high, kDt);
	EXPECT_FALSE(s.armed);
	for (int i = 0; i < 24; ++i)
		ls.poll(low, kDt);
	for (int i = 0; i < 120; ++i)
		s = ls.poll(high, kDt);
	EXPECT_TRUE(s.armed);
}

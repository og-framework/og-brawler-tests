// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/InputMapping/StickRouting.h"
#include "OGBrawler/DAttackCamera.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/InputSequence/GameMotions.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "glm/trigonometric.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>

namespace stickRoutingTest
{
using dAttackMachineSimulation::MovementScheme;
using dInput::stickRouting::LogicalSticks;
using dInput::stickRouting::StickSources;
using dInput::stickRouting::routeSticks;

constexpr float kMoveDeadzone = 0.15f;

const StickSources kSources{ { 0.2f, -0.1f }, { 0.3f, 0.4f }, { -0.5f, 0.25f } };

bool bitIdentical(const glm::vec2& a, const glm::vec2& b)
{
	return std::memcmp(&a, &b, sizeof(glm::vec2)) == 0;
}

bool bitIdentical(const LogicalSticks& a, const LogicalSticks& b)
{
	return bitIdentical(a.move, b.move) && bitIdentical(a.aim, b.aim);
}

struct MovementSchemeRestorer
{
	MovementScheme saved = dAttackMachineSimulation::g_movementScheme.load();
	~MovementSchemeRestorer() { dAttackMachineSimulation::g_movementScheme = saved; }
};
TEST_CASE("StickRouting: isAimRelativeFamily is true for AimRelative and AimRelativeSwapped only",
	"[SimulatableBrawler][StickRouting]")
{
	using dAttackMachineSimulation::isAimRelativeFamily;
	STATIC_REQUIRE_FALSE(isAimRelativeFamily(MovementScheme::CameraRelative));
	STATIC_REQUIRE(isAimRelativeFamily(MovementScheme::AimRelative));
	STATIC_REQUIRE_FALSE(isAimRelativeFamily(MovementScheme::MoveRelativeAim));
	STATIC_REQUIRE(isAimRelativeFamily(MovementScheme::AimRelativeSwapped));
	STATIC_REQUIRE(static_cast<uint32_t>(MovementScheme::AimRelativeSwapped) == 3u);
}

TEST_CASE("StickRouting: row 1 - schemes 0/1/2 without legacy swap: move = clampUnit(keys + left), aim = right",
	"[SimulatableBrawler][StickRouting]")
{
	for (const MovementScheme scheme :
		{ MovementScheme::CameraRelative, MovementScheme::AimRelative, MovementScheme::MoveRelativeAim })
	{
		for (const bool feedsAim : { false, true })
		{
			const LogicalSticks out = routeSticks(kSources, scheme, false, feedsAim, kMoveDeadzone);
			CHECK(bitIdentical(out.move, kSources.moveKeys + kSources.leftStick));
			CHECK(bitIdentical(out.aim, kSources.rightStick));
		}
	}
}

TEST_CASE("StickRouting: row 2 - schemes 0/1/2 with legacy swap: move = right, aim = clampUnit(keys + left)",
	"[SimulatableBrawler][StickRouting]")
{
	for (const MovementScheme scheme :
		{ MovementScheme::CameraRelative, MovementScheme::AimRelative, MovementScheme::MoveRelativeAim })
	{
		for (const bool feedsAim : { false, true })
		{
			const LogicalSticks out = routeSticks(kSources, scheme, true, feedsAim, kMoveDeadzone);
			CHECK(bitIdentical(out.move, kSources.rightStick));
			CHECK(bitIdentical(out.aim, kSources.moveKeys + kSources.leftStick));
		}
	}
}

TEST_CASE("StickRouting: row 3 - scheme 3 with an active right stick: move = clampUnit(keys + right), aim = left",
	"[SimulatableBrawler][StickRouting]")
{
	for (const bool legacySwap : { false, true })
	{
		for (const bool feedsAim : { false, true })
		{
			const LogicalSticks out =
				routeSticks(kSources, MovementScheme::AimRelativeSwapped, legacySwap, feedsAim, kMoveDeadzone);
			CHECK(bitIdentical(out.move, kSources.moveKeys + kSources.rightStick));
			CHECK(bitIdentical(out.aim, kSources.leftStick));
		}
	}
}

TEST_CASE("StickRouting: row 3 - scheme 3 with feedsAim off keeps row 3 even with a neutral right stick",
	"[SimulatableBrawler][StickRouting]")
{
	StickSources sources = kSources;
	sources.rightStick = { 0.05f, -0.05f };
	for (const bool legacySwap : { false, true })
	{
		const LogicalSticks out =
			routeSticks(sources, MovementScheme::AimRelativeSwapped, legacySwap, false, kMoveDeadzone);
		CHECK(bitIdentical(out.move, sources.moveKeys + sources.rightStick));
		CHECK(bitIdentical(out.aim, sources.leftStick));
	}

	sources.rightStick = { 0.f, 0.f };
	const LogicalSticks zero = routeSticks(sources, MovementScheme::AimRelativeSwapped, false, false, kMoveDeadzone);
	CHECK(bitIdentical(zero.move, sources.moveKeys));
	CHECK(bitIdentical(zero.aim, sources.leftStick));
}

TEST_CASE("StickRouting: row 4 - scheme 3 with a neutral right stick and feedsAim on routes as row 1",
	"[SimulatableBrawler][StickRouting]")
{
	StickSources sources = kSources;
	for (const glm::vec2 neutral : { glm::vec2{ 0.f, 0.f }, glm::vec2{ 0.05f, -0.05f }, glm::vec2{ 0.f, -0.1f } })
	{
		sources.rightStick = neutral;
		for (const bool legacySwap : { false, true })
		{
			const LogicalSticks out =
				routeSticks(sources, MovementScheme::AimRelativeSwapped, legacySwap, true, kMoveDeadzone);
			CHECK(bitIdentical(out.move, sources.moveKeys + sources.leftStick));
			CHECK(bitIdentical(out.aim, sources.rightStick));
		}
	}
}

TEST_CASE("StickRouting: row 3 / row 4 boundary sits at length(rightStick) == moveDeadzone, inclusive on the neutral side",
	"[SimulatableBrawler][StickRouting]")
{
	StickSources sources = kSources;

	const auto routedAsRow4 = [&](const glm::vec2 right)
	{
		sources.rightStick = right;
		const LogicalSticks out = routeSticks(sources, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone);
		const bool row4 = bitIdentical(out.aim, sources.rightStick)
		               && bitIdentical(out.move, sources.moveKeys + sources.leftStick);
		const bool row3 = bitIdentical(out.aim, sources.leftStick)
		               && bitIdentical(out.move, sources.moveKeys + sources.rightStick);
		REQUIRE(row4 != row3);
		return row4;
	};

	CHECK(routedAsRow4({ kMoveDeadzone, 0.f }));
	CHECK(routedAsRow4({ 0.f, -kMoveDeadzone }));
	CHECK(routedAsRow4({ 0.149f, 0.f }));
	CHECK(routedAsRow4({ 0.f, 0.149f }));
	CHECK_FALSE(routedAsRow4({ 0.151f, 0.f }));
	CHECK_FALSE(routedAsRow4({ 0.f, -0.151f }));
	CHECK_FALSE(routedAsRow4({ 0.11f, 0.11f }));
}

TEST_CASE("StickRouting: scheme 3 with a neutral right stick is bit-identical to scheme 1",
	"[SimulatableBrawler][StickRouting]")
{
	const glm::vec2 lefts[] = { { 0.f, 0.f }, { 0.3f, 0.4f }, { -0.9f, -0.9f }, { 1.f, 0.f } };
	const glm::vec2 keys[] = { { 0.f, 0.f }, { 1.f, 0.f }, { 0.7071f, -0.7071f } };
	const glm::vec2 neutralRights[] = { { 0.f, 0.f }, { 0.1f, 0.f }, { kMoveDeadzone, 0.f } };
	for (const glm::vec2& left : lefts)
		for (const glm::vec2& k : keys)
			for (const glm::vec2& right : neutralRights)
			{
				const StickSources sources{ k, left, right };
				const LogicalSticks scheme1 =
					routeSticks(sources, MovementScheme::AimRelative, false, true, kMoveDeadzone);
				const LogicalSticks scheme3 =
					routeSticks(sources, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone);
				CHECK(bitIdentical(scheme1, scheme3));
			}
}

TEST_CASE("StickRouting: clampUnit clamps a summed length above 1 to 1 and leaves sub-unit input unchanged",
	"[SimulatableBrawler][StickRouting]")
{
	using dInput::stickRouting::clampUnit;

	CHECK(bitIdentical(clampUnit({ 2.f, 0.f }), glm::vec2{ 1.f, 0.f }));
	CHECK(bitIdentical(clampUnit({ 0.f, -3.f }), glm::vec2{ 0.f, -1.f }));
	CHECK(glm::length(clampUnit({ 1.f, 1.f })) == Catch::Approx(1.f).margin(1e-6));
	CHECK(bitIdentical(clampUnit({ 0.3f, -0.4f }), glm::vec2{ 0.3f, -0.4f }));
	CHECK(bitIdentical(clampUnit({ 1.f, 0.f }), glm::vec2{ 1.f, 0.f }));
	CHECK(bitIdentical(clampUnit({ 0.f, 0.f }), glm::vec2{ 0.f, 0.f }));

	const StickSources summed{ { 1.f, 0.f }, { 1.f, 0.f }, { 1.f, 0.f } };
	const LogicalSticks row1 = routeSticks(summed, MovementScheme::AimRelative, false, true, kMoveDeadzone);
	CHECK(bitIdentical(row1.move, glm::vec2{ 1.f, 0.f }));
	const LogicalSticks row2 = routeSticks(summed, MovementScheme::AimRelative, true, true, kMoveDeadzone);
	CHECK(bitIdentical(row2.aim, glm::vec2{ 1.f, 0.f }));
	const LogicalSticks row3 = routeSticks(summed, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone);
	CHECK(bitIdentical(row3.move, glm::vec2{ 1.f, 0.f }));
	CHECK(bitIdentical(row3.aim, glm::vec2{ 1.f, 0.f }));

	const StickSources diagonal{ { 0.f, 1.f }, { 1.f, 0.f }, { 0.f, 0.f } };
	const LogicalSticks row4 = routeSticks(diagonal, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone);
	CHECK(glm::length(row4.move) == Catch::Approx(1.f).margin(1e-6));
	CHECK(row4.move.x == Catch::Approx(row4.move.y).margin(1e-7));
}

TEST_CASE("StickRouting: SetVariable accepts AimRelativeSwapped and keeps the legacy numeral 3 as CameraRelative",
	"[SimulatableBrawler][StickRouting]")
{
	const MovementSchemeRestorer restore;

	dAttackMachineSimulation::g_movementScheme = MovementScheme::AimRelative;
	CHECK(dAttackMachineSimulation::SetVariable("MovementScheme", "AimRelativeSwapped"));
	CHECK(dAttackMachineSimulation::g_movementScheme.load() == MovementScheme::AimRelativeSwapped);

	CHECK(dAttackMachineSimulation::SetVariable("MovementScheme", "3"));
	CHECK(dAttackMachineSimulation::g_movementScheme.load() == MovementScheme::CameraRelative);
}

glm::vec3 cameraForwardFromDegrees(float pitchBelowHorizonDeg, float yawDeg)
{
	const float pitch = glm::radians(pitchBelowHorizonDeg);
	const float yaw = glm::radians(yawDeg);
	return { std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), -std::sin(pitch) };
}

TEST_CASE("StickRouting: cameraLookAim in AimRelative with BlockLook held is the flattened unit camera forward",
	"[SimulatableBrawler][StickRouting][CameraLookAim]")
{
	using dInput::stickRouting::cameraLookAim;

	struct Row { float pitchDeg; float yawDeg; float scale; };
	for (const Row row : { Row{ 60.f, 0.f, 1.f }, Row{ 30.f, 45.f, 1.f }, Row{ 80.f, -135.f, 1.f },
			Row{ 10.f, 170.f, 1.f }, Row{ 45.f, 90.f, 900.f }, Row{ 20.f, -60.f, 0.05f } })
	{
		CAPTURE(row.pitchDeg, row.yawDeg, row.scale);
		const glm::vec3 forward = row.scale * cameraForwardFromDegrees(row.pitchDeg, row.yawDeg);
		const std::optional<glm::vec3> aim = cameraLookAim(MovementScheme::AimRelative, true, forward);
		REQUIRE(aim.has_value());
		CHECK(aim->z == 0.f);
		CHECK(glm::length(*aim) == Catch::Approx(1.f).margin(1e-6));
		CHECK(aim->x == Catch::Approx(std::cos(glm::radians(row.yawDeg))).margin(1e-5));
		CHECK(aim->y == Catch::Approx(std::sin(glm::radians(row.yawDeg))).margin(1e-5));
	}
}

TEST_CASE("StickRouting: cameraLookAim applies to AimRelative with BlockLook held only, not to AimRelativeSwapped",
	"[SimulatableBrawler][StickRouting][CameraLookAim]")
{
	using dInput::stickRouting::cameraLookAim;

	const glm::vec3 forward = cameraForwardFromDegrees(60.f, 30.f);
	for (const MovementScheme scheme : { MovementScheme::CameraRelative, MovementScheme::AimRelative,
			MovementScheme::MoveRelativeAim, MovementScheme::AimRelativeSwapped })
	{
		for (const bool blockLookHeld : { false, true })
		{
			CAPTURE(static_cast<uint32_t>(scheme), blockLookHeld);
			const bool expected = blockLookHeld && scheme == MovementScheme::AimRelative;
			CHECK(cameraLookAim(scheme, blockLookHeld, forward).has_value() == expected);
		}
	}
}

TEST_CASE("StickRouting: cameraLookAim aims at the orbit camera's steepest pitch and falls back for a camera with no horizontal forward",
	"[SimulatableBrawler][StickRouting][CameraLookAim]")
{
	using dInput::stickRouting::cameraLookAim;
	using dInput::stickRouting::kMinCameraLookAimHorizontalLength;

	const glm::vec3 steepest = cameraForwardFromDegrees(dAttackCameraBehaviour::kHardPitchMaxDeg, 25.f);
	CHECK(glm::length(glm::vec2(steepest.x, steepest.y)) > 10.f * kMinCameraLookAimHorizontalLength);
	const std::optional<glm::vec3> aim = cameraLookAim(MovementScheme::AimRelative, true, steepest);
	REQUIRE(aim.has_value());
	CHECK(glm::length(*aim) == Catch::Approx(1.f).margin(1e-6));
	CHECK(aim->x == Catch::Approx(std::cos(glm::radians(25.f))).margin(1e-5));

	CHECK_FALSE(cameraLookAim(MovementScheme::AimRelative, true, glm::vec3(0.f, 0.f, -1.f)).has_value());
	CHECK_FALSE(cameraLookAim(MovementScheme::AimRelative, true, glm::vec3(0.f, 0.f, 0.f)).has_value());
	CHECK_FALSE(cameraLookAim(MovementScheme::AimRelative, true,
		glm::vec3(0.5f * kMinCameraLookAimHorizontalLength, 0.f, -1.f)).has_value());
}

TEST_CASE("StickRouting: with the camera-look aim, the projectile's back step is a move opposite to the camera forward",
	"[SimulatableBrawler][StickRouting][CameraLookAim]")
{
	using dInput::stickRouting::cameraLookAim;

	const glm::vec3 forward = cameraForwardFromDegrees(60.f, 40.f);
	const std::optional<glm::vec3> aim = cameraLookAim(MovementScheme::AimRelative, true, forward);
	REQUIRE(aim.has_value());

	const glm::vec3 oppositeCameraForward = -*aim;
	const glm::vec3 alongCameraForward = *aim;
	const glm::vec3 screenRight(aim->y, -aim->x, 0.f);
	const float epsilon = dAttackDirection::kMoveMagnitudeEpsilon;
	CHECK(inputSequence::stickMatchesStep(oppositeCameraForward, *aim, kBackShortcutStep, epsilon));
	CHECK_FALSE(inputSequence::stickMatchesStep(alongCameraForward, *aim, kBackShortcutStep, epsilon));
	CHECK_FALSE(inputSequence::stickMatchesStep(screenRight, *aim, kBackShortcutStep, epsilon));
}

constexpr float kAimDeadzone = 0.2f;

TEST_CASE("StickRouting: the latch tests use the shipped stick deadzones",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	CHECK(dAttackMachineSimulation::g_moveStickDeadzone.load() == kMoveDeadzone);
	CHECK(dAttackMachineSimulation::g_aimStickDeadzone.load() == kAimDeadzone);
}

TEST_CASE("StickRouting: a resting stick at or below its deadzone neither sets nor clears the gamepad latch",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::lastInputWasGamepadAfterStick;

	for (const float deadzone : { kMoveDeadzone, kAimDeadzone })
	{
		for (const glm::vec2 drift : { glm::vec2(0.01f, 0.f), glm::vec2(-0.02f, 0.03f), glm::vec2(0.05f, -0.05f),
				glm::vec2(0.f, -0.1f), glm::vec2(0.5f * deadzone, -0.5f * deadzone), glm::vec2(0.f, -deadzone),
				glm::vec2(-deadzone, 0.f) })
		{
			CAPTURE(deadzone, drift.x, drift.y);
			CHECK_FALSE(lastInputWasGamepadAfterStick(false, drift, deadzone));
			CHECK(lastInputWasGamepadAfterStick(true, drift, deadzone));
		}
	}
}

TEST_CASE("StickRouting: a stick past its deadzone sets the gamepad latch",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::lastInputWasGamepadAfterStick;

	for (const float deadzone : { kMoveDeadzone, kAimDeadzone })
	{
		for (const glm::vec2 stick : { glm::vec2(0.f, -1.f), glm::vec2(0.7f, 0.7f), glm::vec2(-1.f, 0.f),
				glm::vec2(1.01f * deadzone, 0.f), glm::vec2(0.f, -1.01f * deadzone), glm::vec2(0.8f * deadzone, 0.8f * deadzone) })
		{
			CAPTURE(deadzone, stick.x, stick.y);
			CHECK(lastInputWasGamepadAfterStick(false, stick, deadzone));
			CHECK(lastInputWasGamepadAfterStick(true, stick, deadzone));
		}
	}
}

TEST_CASE("StickRouting: a WASD move clears the gamepad latch, a D-pad move sets it, and a release leaves it",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::lastInputWasGamepadAfterMoveKeys;

	for (const glm::vec2 keys : { glm::vec2(0.f, -1.f), glm::vec2(1.f, 0.f), glm::vec2(-1.f, 1.f), glm::vec2(0.f, 1.f) })
	{
		CAPTURE(keys.x, keys.y);
		CHECK_FALSE(lastInputWasGamepadAfterMoveKeys(true, keys, false));
		CHECK_FALSE(lastInputWasGamepadAfterMoveKeys(false, keys, false));
		CHECK(lastInputWasGamepadAfterMoveKeys(false, keys, true));
		CHECK(lastInputWasGamepadAfterMoveKeys(true, keys, true));
	}
	for (const bool latch : { false, true })
	{
		CAPTURE(latch);
		CHECK(lastInputWasGamepadAfterMoveKeys(latch, glm::vec2(0.f, 0.f), false) == latch);
		CHECK(lastInputWasGamepadAfterMoveKeys(latch, glm::vec2(0.f, 0.f), true) == latch);
	}
}

TEST_CASE("StickRouting: the first cursor sample only sets the anchor",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::CursorLatch;
	using dInput::stickRouting::lastInputWasGamepadAfterCursor;

	for (const bool latch : { false, true })
	{
		CAPTURE(latch);
		const CursorLatch result = lastInputWasGamepadAfterCursor(latch, std::nullopt, glm::vec2(640.f, 360.f));
		CHECK(result.lastInputWasGamepad == latch);
		REQUIRE(result.anchor.has_value());
		CHECK(*result.anchor == glm::vec2(640.f, 360.f));
	}
}

TEST_CASE("StickRouting: the cursor clears the gamepad latch once it is kCursorLatchMinPixels from its anchor, and jitter does not",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::CursorLatch;
	using dInput::stickRouting::kCursorLatchMinPixels;
	using dInput::stickRouting::lastInputWasGamepadAfterCursor;

	STATIC_REQUIRE(kCursorLatchMinPixels == 8.f);
	const glm::vec2 start(640.f, 360.f);

	CursorLatch jitter{ true, start };
	for (int frame = 0; frame < 120; ++frame)
	{
		const float offset = (frame % 2 == 0) ? 3.f : -3.f;
		jitter = lastInputWasGamepadAfterCursor(jitter.lastInputWasGamepad, jitter.anchor, start + glm::vec2(offset, -offset));
		CAPTURE(frame);
		CHECK(jitter.lastInputWasGamepad);
		CHECK(jitter.anchor == std::optional<glm::vec2>(start));
	}

	CursorLatch slow{ true, start };
	for (int pixel = 1; pixel <= 10; ++pixel)
	{
		slow = lastInputWasGamepadAfterCursor(slow.lastInputWasGamepad, slow.anchor, start + glm::vec2(0.f, static_cast<float>(pixel)));
		CAPTURE(pixel);
		CHECK(slow.lastInputWasGamepad == (pixel < 8));
	}
	CHECK(slow.anchor == std::optional<glm::vec2>(start + glm::vec2(0.f, 10.f)));
}

TEST_CASE("StickRouting: while the latch says keyboard and mouse, the cursor anchor follows the cursor",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::CursorLatch;
	using dInput::stickRouting::lastInputWasGamepadAfterCursor;

	const CursorLatch moved = lastInputWasGamepadAfterCursor(false, glm::vec2(100.f, 100.f), glm::vec2(103.f, 98.f));
	CHECK_FALSE(moved.lastInputWasGamepad);
	CHECK(moved.anchor == std::optional<glm::vec2>(glm::vec2(103.f, 98.f)));

	const CursorLatch padTookOver = lastInputWasGamepadAfterCursor(true, moved.anchor, glm::vec2(106.f, 98.f));
	CHECK(padTookOver.lastInputWasGamepad);
	CHECK(padTookOver.anchor == moved.anchor);
}

TEST_CASE("StickRouting: WASD held beside a drifting pad stays keyboard and mouse, frame after frame",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::lastInputWasGamepadAfterMoveKeys;
	using dInput::stickRouting::lastInputWasGamepadAfterStick;

	const glm::vec2 leftDrift(0.03f, -0.02f);
	const glm::vec2 rightDrift(-0.02f, 0.04f);
	bool latch = true;
	for (int frame = 0; frame < 120; ++frame)
	{
		latch = lastInputWasGamepadAfterMoveKeys(latch, glm::vec2(0.f, -1.f), false);
		latch = lastInputWasGamepadAfterStick(latch, leftDrift, kMoveDeadzone);
		latch = lastInputWasGamepadAfterStick(latch, rightDrift, kAimDeadzone);
		CAPTURE(frame);
		CHECK_FALSE(latch);
	}
}

TEST_CASE("StickRouting: moving the mouse beside a drifting pad clears the gamepad latch, and the drift does not set it again",
	"[SimulatableBrawler][StickRouting][GamepadLatch]")
{
	using dInput::stickRouting::CursorLatch;
	using dInput::stickRouting::lastInputWasGamepadAfterCursor;
	using dInput::stickRouting::lastInputWasGamepadAfterStick;

	const glm::vec2 leftDrift(0.03f, -0.02f);
	const glm::vec2 rightDrift(-0.02f, 0.04f);
	CursorLatch latch{ true, glm::vec2(640.f, 360.f) };
	for (int frame = 1; frame <= 60; ++frame)
	{
		bool lastInputWasGamepad = lastInputWasGamepadAfterStick(latch.lastInputWasGamepad, leftDrift, kMoveDeadzone);
		lastInputWasGamepad = lastInputWasGamepadAfterStick(lastInputWasGamepad, rightDrift, kAimDeadzone);
		latch = lastInputWasGamepadAfterCursor(lastInputWasGamepad, latch.anchor,
			glm::vec2(640.f + 2.f * static_cast<float>(frame), 360.f));
		CAPTURE(frame);
		CHECK(latch.lastInputWasGamepad == (frame < 4));
	}
}

} // namespace stickRoutingTest

#endif // WITH_LOW_LEVEL_TESTS

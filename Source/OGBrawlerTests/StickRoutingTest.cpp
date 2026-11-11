// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/InputMapping/StickRouting.h"
#include "OGBrawler/DAttackCamera.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/InputSequence/GameMotions.h"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGSimulation/SimulationFrame.h"
#include "glm/trigonometric.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <utility>

namespace stickRoutingTest
{
using dAttackMachineSimulation::MovementScheme;
using dInput::stickRouting::LogicalSticks;
using dInput::stickRouting::StickSources;
using dInput::stickRouting::routeSticks;

constexpr float kMoveDeadzone = 0.15f;
constexpr float kBackStopMaxMagnitude = dAttackMachineSimulation::kDefaultBackStopMaxMagnitude;

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
			const LogicalSticks out = routeSticks(kSources, scheme, false, feedsAim, kMoveDeadzone, kBackStopMaxMagnitude);
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
			const LogicalSticks out = routeSticks(kSources, scheme, true, feedsAim, kMoveDeadzone, kBackStopMaxMagnitude);
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
				routeSticks(kSources, MovementScheme::AimRelativeSwapped, legacySwap, feedsAim, kMoveDeadzone, kBackStopMaxMagnitude);
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
			routeSticks(sources, MovementScheme::AimRelativeSwapped, legacySwap, false, kMoveDeadzone, kBackStopMaxMagnitude);
		CHECK(bitIdentical(out.move, sources.moveKeys + sources.rightStick));
		CHECK(bitIdentical(out.aim, sources.leftStick));
	}

	sources.rightStick = { 0.f, 0.f };
	const LogicalSticks zero = routeSticks(sources, MovementScheme::AimRelativeSwapped, false, false, kMoveDeadzone, kBackStopMaxMagnitude);
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
				routeSticks(sources, MovementScheme::AimRelativeSwapped, legacySwap, true, kMoveDeadzone, kBackStopMaxMagnitude);
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
		const LogicalSticks out = routeSticks(sources, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone, kBackStopMaxMagnitude);
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
					routeSticks(sources, MovementScheme::AimRelative, false, true, kMoveDeadzone, kBackStopMaxMagnitude);
				const LogicalSticks scheme3 =
					routeSticks(sources, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone, kBackStopMaxMagnitude);
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
	const LogicalSticks row1 = routeSticks(summed, MovementScheme::AimRelative, false, true, kMoveDeadzone, kBackStopMaxMagnitude);
	CHECK(bitIdentical(row1.move, glm::vec2{ 1.f, 0.f }));
	const LogicalSticks row2 = routeSticks(summed, MovementScheme::AimRelative, true, true, kMoveDeadzone, kBackStopMaxMagnitude);
	CHECK(bitIdentical(row2.aim, glm::vec2{ 1.f, 0.f }));
	const LogicalSticks row3 = routeSticks(summed, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone, kBackStopMaxMagnitude);
	CHECK(bitIdentical(row3.move, glm::vec2{ 1.f, 0.f }));
	CHECK(bitIdentical(row3.aim, glm::vec2{ 1.f, 0.f }));

	const StickSources diagonal{ { 0.f, 1.f }, { 1.f, 0.f }, { 0.f, 0.f } };
	const LogicalSticks row4 = routeSticks(diagonal, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone, kBackStopMaxMagnitude);
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
	return glm::vec3(std::cos(pitch) * std::sin(yaw), -std::sin(pitch), -(std::cos(pitch) * std::cos(yaw)));
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
		CHECK(simFrame::height(*aim) == 0.f);
		CHECK(glm::length(*aim) == Catch::Approx(1.f).margin(1e-6));
		CHECK(simFrame::forwardRight(*aim).x == Catch::Approx(std::cos(glm::radians(row.yawDeg))).margin(1e-5));
		CHECK(simFrame::forwardRight(*aim).y == Catch::Approx(std::sin(glm::radians(row.yawDeg))).margin(1e-5));
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
	CHECK(glm::length(simFrame::forwardRight(steepest)) > 10.f * kMinCameraLookAimHorizontalLength);
	const std::optional<glm::vec3> aim = cameraLookAim(MovementScheme::AimRelative, true, steepest);
	REQUIRE(aim.has_value());
	CHECK(glm::length(*aim) == Catch::Approx(1.f).margin(1e-6));
	CHECK(simFrame::forwardRight(*aim).x == Catch::Approx(std::cos(glm::radians(25.f))).margin(1e-5));

	CHECK_FALSE(cameraLookAim(MovementScheme::AimRelative, true, glm::vec3(0.f, -1.f, -0.f)).has_value());
	CHECK_FALSE(cameraLookAim(MovementScheme::AimRelative, true, glm::vec3(0.f, 0.f, 0.f)).has_value());
	CHECK_FALSE(cameraLookAim(MovementScheme::AimRelative, true,
		glm::vec3(0.f, -1.f, -(0.5f * kMinCameraLookAimHorizontalLength))).has_value());
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
	const glm::vec3 screenRight = simFrame::fromForwardRight(
		glm::vec2(simFrame::forwardRight(*aim).y, -simFrame::forwardRight(*aim).x), 0.f);
	const float epsilon = dAttackDirection::kMoveMagnitudeEpsilon;
	CHECK(inputSequence::stickMatchesStep(oppositeCameraForward, *aim, kBackShortcutStep, epsilon));
	CHECK_FALSE(inputSequence::stickMatchesStep(alongCameraForward, *aim, kBackShortcutStep, epsilon));
	CHECK_FALSE(inputSequence::stickMatchesStep(screenRight, *aim, kBackShortcutStep, epsilon));
}

const glm::vec2 kWStick(0.f, -1.f);
const glm::vec2 kDStick(1.f, 0.f);

void checkCameraRelative(float yawDeg, const glm::vec2& stick, const glm::vec3& expected)
{
	const glm::vec3 camForward = cameraForwardFromDegrees(50.f, yawDeg);
	const glm::vec3 moved =
		dInput::stickRouting::cameraRelativeDirection(camForward, simFrame::fromForwardRight(stick, 0.f));
	CAPTURE(yawDeg, stick.x, stick.y, moved.x, moved.y, moved.z, expected.x, expected.y, expected.z);
	CHECK(moved.x == Catch::Approx(expected.x).margin(1e-5));
	CHECK(moved.y == Catch::Approx(expected.y).margin(1e-5));
	CHECK(moved.z == Catch::Approx(expected.z).margin(1e-5));
}

TEST_CASE("StickRouting: cameraRelativeDirection moves W along the camera's flattened forward, at four camera yaws",
	"[SimulatableBrawler][StickRouting][CameraRelativeDirection]")
{
	for (const float yawDeg : { 0.f, 45.f, 180.f, -120.f })
	{
		const float yaw = glm::radians(yawDeg);
		checkCameraRelative(yawDeg, kWStick, glm::vec3(std::sin(yaw), 0.f, -std::cos(yaw)));
	}
}

TEST_CASE("StickRouting: cameraRelativeDirection moves D along the camera's right, at four camera yaws",
	"[SimulatableBrawler][StickRouting][CameraRelativeDirection]")
{
	for (const float yawDeg : { 0.f, 45.f, 180.f, -120.f })
	{
		const float yaw = glm::radians(yawDeg);
		checkCameraRelative(yawDeg, kDStick, glm::vec3(std::cos(yaw), 0.f, std::sin(yaw)));
	}
}

void checkCameraRelativeAtExactForward(const glm::vec3& camForward, const glm::vec2& stick, const glm::vec3& expected)
{
	const glm::vec3 moved =
		dInput::stickRouting::cameraRelativeDirection(camForward, simFrame::fromForwardRight(stick, 0.f));
	CAPTURE(camForward.x, camForward.y, camForward.z, stick.x, stick.y, moved.x, moved.y, moved.z, expected.x, expected.y,
		expected.z);
	REQUIRE(std::isfinite(moved.x));
	REQUIRE(std::isfinite(moved.y));
	REQUIRE(std::isfinite(moved.z));
	CHECK(moved.x == Catch::Approx(expected.x).margin(1e-5));
	CHECK(moved.y == Catch::Approx(expected.y).margin(1e-5));
	CHECK(moved.z == Catch::Approx(expected.z).margin(1e-5));
}

TEST_CASE("StickRouting: cameraRelativeDirection is finite and correct with the camera forward exactly along kLeft or kRight",
	"[SimulatableBrawler][StickRouting][CameraRelativeDirection]")
{
	for (const float cameraHeight : { 0.f, -1.2f })
	{
		const glm::vec3 alongRight = simFrame::withHeight(simFrame::kRight, cameraHeight);
		checkCameraRelativeAtExactForward(alongRight, kWStick, simFrame::kRight);
		checkCameraRelativeAtExactForward(alongRight, kDStick, -simFrame::kForward);

		const glm::vec3 alongLeft = simFrame::withHeight(simFrame::kLeft, cameraHeight);
		checkCameraRelativeAtExactForward(alongLeft, kWStick, simFrame::kLeft);
		checkCameraRelativeAtExactForward(alongLeft, kDStick, simFrame::kForward);
	}
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

glm::vec2 backStick(float magnitude, float degreesOffBack)
{
	const float radians = glm::radians(degreesOffBack);
	return glm::vec2(magnitude * std::sin(radians), magnitude * std::cos(radians));
}

const glm::vec2 kAimLeft(0.3f, 0.4f);

LogicalSticks routeRight(const glm::vec2& right, MovementScheme scheme, bool legacySwap, bool feedsAim,
	float backStopMaxMagnitude = kBackStopMaxMagnitude)
{
	return routeSticks(StickSources{ glm::vec2(0.f, 0.f), kAimLeft, right }, scheme, legacySwap, feedsAim, kMoveDeadzone,
		backStopMaxMagnitude);
}

TEST_CASE("StickRouting: the back-stop tests use the shipped g_backStopMaxMagnitude",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	CHECK(dAttackMachineSimulation::g_backStopMaxMagnitude.load() == kBackStopMaxMagnitude);
}

TEST_CASE("StickRouting: scheme 3 with the right stick moving stops a light pull within the back-shortcut window",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	REQUIRE(glm::radians(20.f) < kBackShortcutStep.angleTolerance);
	REQUIRE(glm::radians(25.f) > kBackShortcutStep.angleTolerance);
	for (const bool legacySwap : { false, true })
	{
		for (const bool feedsAim : { false, true })
		{
			for (const auto& [magnitude, degrees] :
				{ std::pair{ 0.3f, 0.f }, std::pair{ 0.3f, 20.f }, std::pair{ 0.3f, -20.f }, std::pair{ 0.4f, 0.f } })
			{
				CAPTURE(legacySwap, feedsAim, magnitude, degrees);
				const LogicalSticks out =
					routeRight(backStick(magnitude, degrees), MovementScheme::AimRelativeSwapped, legacySwap, feedsAim);
				CHECK(bitIdentical(out.move, glm::vec2(0.f, 0.f)));
				CHECK(bitIdentical(out.aim, kAimLeft));
			}
		}
	}
}

TEST_CASE("StickRouting: scheme 3 routes a pull outside the back-shortcut window, at or above the stop magnitude, or in the deadzone unchanged",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	const float overDeadzone = std::nextafter(kMoveDeadzone, 1.f);
	const glm::vec2 atStop(0.f, kBackStopMaxMagnitude);
	REQUIRE(glm::length(atStop) == kBackStopMaxMagnitude);
	for (const bool legacySwap : { false, true })
	{
		for (const bool feedsAim : { false, true })
		{
			for (const glm::vec2 right : { backStick(0.3f, 25.f), backStick(0.3f, -25.f), atStop, backStick(0.8f, 0.f),
					backStick(1.f, 0.f), backStick(0.3f, 90.f), backStick(0.3f, 180.f) })
			{
				CAPTURE(legacySwap, feedsAim, right.x, right.y);
				const LogicalSticks out = routeRight(right, MovementScheme::AimRelativeSwapped, legacySwap, feedsAim);
				CHECK(bitIdentical(out.move, right));
				CHECK(bitIdentical(out.aim, kAimLeft));
			}

			const glm::vec2 justOverDeadzone(0.f, overDeadzone);
			REQUIRE(glm::length(justOverDeadzone) > kMoveDeadzone);
			CHECK(bitIdentical(
				routeRight(justOverDeadzone, MovementScheme::AimRelativeSwapped, legacySwap, feedsAim).move,
				glm::vec2(0.f, 0.f)));
			const glm::vec2 justUnderStop(0.f, std::nextafter(kBackStopMaxMagnitude, 0.f));
			REQUIRE(glm::length(justUnderStop) < kBackStopMaxMagnitude);
			CHECK(bitIdentical(
				routeRight(justUnderStop, MovementScheme::AimRelativeSwapped, legacySwap, feedsAim).move,
				glm::vec2(0.f, 0.f)));
		}

		for (const glm::vec2 inDeadzone : { glm::vec2(0.f, kMoveDeadzone), glm::vec2(0.f, 0.1f), glm::vec2(0.f, 0.f) })
		{
			CAPTURE(legacySwap, inDeadzone.x, inDeadzone.y);
			const LogicalSticks mainBranch =
				routeRight(inDeadzone, MovementScheme::AimRelativeSwapped, legacySwap, false);
			CHECK(bitIdentical(mainBranch.move, inDeadzone));
			CHECK(bitIdentical(mainBranch.aim, kAimLeft));
			const LogicalSticks fallback = routeRight(inDeadzone, MovementScheme::AimRelativeSwapped, legacySwap, true);
			CHECK(bitIdentical(fallback.move, kAimLeft));
			CHECK(bitIdentical(fallback.aim, inDeadzone));
		}
	}
}

TEST_CASE("StickRouting: scheme 3's fallback branch routes a light back pull on the left stick unchanged",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	for (const float degrees : { 0.f, 20.f, -20.f })
	{
		const glm::vec2 lightBack = backStick(0.3f, degrees);
		const StickSources sources{ glm::vec2(0.f, 0.f), lightBack, glm::vec2(0.f, 0.f) };
		const LogicalSticks out = routeSticks(sources, MovementScheme::AimRelativeSwapped, false, true, kMoveDeadzone,
			kBackStopMaxMagnitude);
		CAPTURE(degrees);
		CHECK(bitIdentical(out.move, lightBack));
		CHECK(bitIdentical(out.aim, glm::vec2(0.f, 0.f)));
	}
}

TEST_CASE("StickRouting: scheme 3 keys held back route at full deflection, unchanged",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	for (const bool feedsAim : { false, true })
	{
		const StickSources keysBack{ glm::vec2(0.f, 1.f), glm::vec2(0.f, 0.f), glm::vec2(0.f, 0.f) };
		const LogicalSticks out = routeSticks(keysBack, MovementScheme::AimRelativeSwapped, false, feedsAim,
			kMoveDeadzone, kBackStopMaxMagnitude);
		CAPTURE(feedsAim);
		CHECK(bitIdentical(out.move, glm::vec2(0.f, 1.f)));
	}
}

TEST_CASE("StickRouting: a stop magnitude at or below the move deadzone disables the back stop",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	for (const float disabled : { 0.f, 0.1f, kMoveDeadzone })
	{
		for (const glm::vec2 right : { backStick(0.3f, 0.f), backStick(0.3f, 20.f),
				glm::vec2(0.f, std::nextafter(kMoveDeadzone, 1.f)) })
		{
			for (const bool feedsAim : { false, true })
			{
				CAPTURE(disabled, right.x, right.y, feedsAim);
				const LogicalSticks out = routeRight(right, MovementScheme::AimRelativeSwapped, false, feedsAim, disabled);
				CHECK(bitIdentical(out.move, right));
			}
		}
	}
}

TEST_CASE("StickRouting: schemes 0/1/2 route the stop-zone sticks bit for bit as before",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	const glm::vec2 keys(0.f, 0.f);
	for (const MovementScheme scheme :
		{ MovementScheme::CameraRelative, MovementScheme::AimRelative, MovementScheme::MoveRelativeAim })
	{
		for (const bool legacySwap : { false, true })
		{
			for (const bool feedsAim : { false, true })
			{
				for (const float degrees : { 0.f, 20.f, -20.f })
				{
					const glm::vec2 lightBack = backStick(0.3f, degrees);
					for (const StickSources& sources : { StickSources{ keys, lightBack, kAimLeft },
							 StickSources{ keys, kAimLeft, lightBack }, StickSources{ keys, lightBack, lightBack } })
					{
						CAPTURE(static_cast<uint32_t>(scheme), legacySwap, feedsAim, degrees);
						const LogicalSticks out =
							routeSticks(sources, scheme, legacySwap, feedsAim, kMoveDeadzone, kBackStopMaxMagnitude);
						const glm::vec2 keysAndLeft = dInput::stickRouting::clampUnit(sources.moveKeys + sources.leftStick);
						CHECK(bitIdentical(out.move, legacySwap ? sources.rightStick : keysAndLeft));
						CHECK(bitIdentical(out.aim, legacySwap ? keysAndLeft : sources.rightStick));
						for (const float other : { 0.f, 1.f })
							CHECK(bitIdentical(out,
								routeSticks(sources, scheme, legacySwap, feedsAim, kMoveDeadzone, other)));
					}
				}
			}
		}
	}
}

TEST_CASE("StickRouting: every stopped stick is the back-shortcut step through the aim-relative move, and +-25 degrees is not",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	using dInput::stickRouting::cameraRelativeDirection;
	using dInput::stickRouting::inBackStopZone;
	using inputSequence::stickMatchesStep;

	const float epsilon = dAttackDirection::kMoveMagnitudeEpsilon;
	const auto stickToWorld = [](const glm::vec2& stick, const glm::vec3& aim)
	{ return cameraRelativeDirection(aim, simFrame::fromForwardRight(stick, 0.f)); };

	int stoppedChecked = 0;
	int sweepChecked = 0;
	for (const float yawDeg : { 0.f, 37.f, 90.f, 180.f, -120.f })
	{
		const glm::vec3 aim = cameraForwardFromDegrees(0.f, yawDeg);
		for (const glm::vec2 stopped : { backStick(0.3f, 0.f), backStick(0.3f, 20.f), backStick(0.3f, -20.f),
				glm::vec2(0.f, std::nextafter(kMoveDeadzone, 1.f)), backStick(0.39f, 15.f), backStick(0.4f, 0.f) })
		{
			CAPTURE(yawDeg, stopped.x, stopped.y);
			REQUIRE(bitIdentical(
				routeRight(stopped, MovementScheme::AimRelativeSwapped, false, true).move, glm::vec2(0.f, 0.f)));
			CHECK(stickMatchesStep(stickToWorld(stopped, aim), aim, kBackShortcutStep, epsilon));
			++stoppedChecked;
		}
		for (const float degrees : { 25.f, -25.f })
		{
			CAPTURE(yawDeg, degrees);
			CHECK_FALSE(stickMatchesStep(stickToWorld(backStick(0.3f, degrees), aim), aim, kBackShortcutStep, epsilon));
		}
		for (int degrees = -180; degrees < 180; ++degrees)
		{
			if (std::abs(std::abs(degrees) - 22.5f) < 1.f)
				continue;
			const glm::vec2 stick = backStick(0.3f, static_cast<float>(degrees));
			CAPTURE(yawDeg, degrees);
			CHECK(inBackStopZone(stick, kMoveDeadzone, kBackStopMaxMagnitude)
				== stickMatchesStep(stickToWorld(stick, aim), aim, kBackShortcutStep, epsilon));
			++sweepChecked;
		}
	}
	CHECK(stoppedChecked == 5 * 6);
	CHECK(sweepChecked == 5 * 356);
}

TEST_CASE("StickRouting: SetVariable sets g_backStopMaxMagnitude, clamped to [0, 1]",
	"[SimulatableBrawler][StickRouting][BackStop]")
{
	const float saved = dAttackMachineSimulation::g_backStopMaxMagnitude.load();
	CHECK(dAttackMachineSimulation::SetVariable("BackStopMaxMagnitude", "0.3"));
	CHECK(dAttackMachineSimulation::g_backStopMaxMagnitude.load() == 0.3f);
	CHECK(dAttackMachineSimulation::SetVariable("BackStopMaxMagnitude", "5"));
	CHECK(dAttackMachineSimulation::g_backStopMaxMagnitude.load() == 1.f);
	CHECK(dAttackMachineSimulation::SetVariable("BackStopMaxMagnitude", "0"));
	CHECK(dAttackMachineSimulation::g_backStopMaxMagnitude.load() == 0.f);
	CHECK_FALSE(dAttackMachineSimulation::SetVariable("BackStopMaxMagnitude", "banana"));
	CHECK(dAttackMachineSimulation::g_backStopMaxMagnitude.load() == 0.f);
	CHECK(dAttackMachineSimulation::refusedVariableReason("BackStopMaxMagnitude") == nullptr);
	dAttackMachineSimulation::g_backStopMaxMagnitude = saved;
}

} // namespace stickRoutingTest

#endif // WITH_LOW_LEVEL_TESTS

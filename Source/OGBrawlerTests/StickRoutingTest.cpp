// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/InputMapping/StickRouting.h"

#include <cstdint>
#include <cstring>

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

} // namespace stickRoutingTest

#endif // WITH_LOW_LEVEL_TESTS

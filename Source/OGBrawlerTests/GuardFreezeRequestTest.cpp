// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/InputMapping/StickRouting.h"

#include <cmath>
#include <vector>

namespace guardFreezeRequestTest
{
using dAttackMachineSimulation::MovementScheme;
using dInput::stickRouting::StickSources;
using dInput::stickRouting::guardFreezeRequested;
using dInput::stickRouting::routeSticks;

constexpr float kMoveDeadzone = 0.15f;

const glm::vec2 kNone{ 0.f, 0.f };
const glm::vec2 kKeysUp{ 0.f, -1.f };
const glm::vec2 kKeysDiagonal{ 1.f, -1.f };
const glm::vec2 kLeftFull{ 0.f, -1.f };
const glm::vec2 kRightFull{ 1.f, 0.f };
const glm::vec2 kLeftDrift{ 0.04f, -0.06f };
const glm::vec2 kRightDrift{ -0.05f, 0.07f };

const MovementScheme kAllSchemes[] = { MovementScheme::CameraRelative, MovementScheme::AimRelative,
	MovementScheme::MoveRelativeAim, MovementScheme::AimRelativeSwapped };
const MovementScheme kNonSwappedSchemes[] = { MovementScheme::CameraRelative, MovementScheme::AimRelative,
	MovementScheme::MoveRelativeAim };

bool freeze(const StickSources& sources, MovementScheme scheme, bool legacySwap = false, bool feedsAim = true,
	bool holdGuard = true)
{
	return guardFreezeRequested(holdGuard, sources, scheme, legacySwap, feedsAim, kMoveDeadzone);
}

float actualMoveLength(const StickSources& sources, MovementScheme scheme, bool legacySwap, bool feedsAim)
{
	return glm::length(routeSticks(sources, scheme, legacySwap, feedsAim, kMoveDeadzone).move);
}

TEST_CASE("GuardFreeze: scheme 3 - left only roots, right only / both / keys move",
	"[SimulatableBrawler][GuardFreeze]")
{
	const MovementScheme s = MovementScheme::AimRelativeSwapped;
	for (const bool feedsAim : { true, false })
	{
		INFO("feedsAim " << feedsAim);
		CHECK(freeze({ kNone, kLeftFull, kNone }, s, false, feedsAim));
		CHECK_FALSE(freeze({ kNone, kNone, kRightFull }, s, false, feedsAim));
		CHECK_FALSE(freeze({ kNone, kLeftFull, kRightFull }, s, false, feedsAim));
		CHECK_FALSE(freeze({ kKeysUp, kNone, kNone }, s, false, feedsAim));
		CHECK_FALSE(freeze({ kKeysDiagonal, kNone, kNone }, s, false, feedsAim));
		CHECK(freeze({ kNone, kLeftFull, kRightDrift }, s, false, feedsAim));
		CHECK_FALSE(freeze({ kKeysUp, kLeftDrift, kRightDrift }, s, false, feedsAim));
		CHECK_FALSE(freeze({ kNone, kLeftDrift, kRightFull }, s, false, feedsAim));
	}
	CHECK(actualMoveLength({ kNone, kLeftFull, kNone }, s, false, true) == 1.f);
}

TEST_CASE("GuardFreeze: schemes 0/1/2 - left / keys move, right only roots (turn in place)",
	"[SimulatableBrawler][GuardFreeze]")
{
	for (const MovementScheme s : kNonSwappedSchemes)
	{
		for (const bool feedsAim : { true, false })
		{
			INFO("scheme " << static_cast<int>(s) << " feedsAim " << feedsAim);
			CHECK_FALSE(freeze({ kNone, kLeftFull, kNone }, s, false, feedsAim));
			CHECK(freeze({ kNone, kNone, kRightFull }, s, false, feedsAim));
			CHECK_FALSE(freeze({ kKeysUp, kNone, kNone }, s, false, feedsAim));
			CHECK_FALSE(freeze({ kKeysDiagonal, kNone, kNone }, s, false, feedsAim));
			CHECK_FALSE(freeze({ kNone, kLeftFull, kRightFull }, s, false, feedsAim));
			CHECK(freeze({ kNone, kLeftDrift, kRightFull }, s, false, feedsAim));
			CHECK_FALSE(freeze({ kKeysUp, kLeftDrift, kRightDrift }, s, false, feedsAim));
		}
	}
}

TEST_CASE("GuardFreeze: legacy swap - keys only roots, right stick moves",
	"[SimulatableBrawler][GuardFreeze]")
{
	for (const MovementScheme s : kNonSwappedSchemes)
	{
		for (const bool feedsAim : { true, false })
		{
			INFO("scheme " << static_cast<int>(s) << " feedsAim " << feedsAim);
			CHECK(freeze({ kKeysUp, kNone, kNone }, s, true, feedsAim));
			CHECK(freeze({ kKeysUp, kLeftFull, kRightDrift }, s, true, feedsAim));
			CHECK_FALSE(freeze({ kNone, kNone, kRightFull }, s, true, feedsAim));
			CHECK_FALSE(freeze({ kKeysUp, kLeftFull, kRightFull }, s, true, feedsAim));
		}
	}
	CHECK_FALSE(freeze({ kKeysUp, kNone, kNone }, MovementScheme::AimRelativeSwapped, true, true));
	CHECK(freeze({ kNone, kLeftFull, kNone }, MovementScheme::AimRelativeSwapped, true, true));
}

TEST_CASE("GuardFreeze: nothing pushed (or only drift) roots; guard released never freezes",
	"[SimulatableBrawler][GuardFreeze]")
{
	for (const MovementScheme s : kAllSchemes)
	{
		for (const bool legacySwap : { false, true })
		{
			for (const bool feedsAim : { false, true })
			{
				INFO("scheme " << static_cast<int>(s) << " swap " << legacySwap << " feedsAim " << feedsAim);
				CHECK(freeze({ kNone, kNone, kNone }, s, legacySwap, feedsAim));
				CHECK(freeze({ kNone, kLeftDrift, kRightDrift }, s, legacySwap, feedsAim));
				for (const StickSources& src : { StickSources{ kNone, kNone, kNone },
						 StickSources{ kKeysUp, kNone, kNone }, StickSources{ kNone, kLeftFull, kNone },
						 StickSources{ kNone, kNone, kRightFull }, StickSources{ kKeysUp, kLeftFull, kRightFull },
						 StickSources{ kNone, kLeftDrift, kRightDrift } })
					CHECK_FALSE(freeze(src, s, legacySwap, feedsAim, false));
			}
		}
	}
}

TEST_CASE("GuardFreeze: deadzone boundary - strictly below roots, at and over moves",
	"[SimulatableBrawler][GuardFreeze]")
{
	const float under = std::nextafter(kMoveDeadzone, 0.f);
	const float over = std::nextafter(kMoveDeadzone, 1.f);
	REQUIRE(glm::length(glm::vec2(under, 0.f)) < kMoveDeadzone);
	REQUIRE(glm::length(glm::vec2(kMoveDeadzone, 0.f)) == kMoveDeadzone);
	REQUIRE(glm::length(glm::vec2(over, 0.f)) > kMoveDeadzone);

	for (const MovementScheme s : kNonSwappedSchemes)
	{
		INFO("scheme " << static_cast<int>(s));
		CHECK(freeze({ kNone, { under, 0.f }, kNone }, s));
		CHECK_FALSE(freeze({ kNone, { kMoveDeadzone, 0.f }, kNone }, s));
		CHECK_FALSE(freeze({ kNone, { over, 0.f }, kNone }, s));
		CHECK(freeze({ kNone, kNone, { 0.f, under } }, s, true));
		CHECK_FALSE(freeze({ kNone, kNone, { 0.f, kMoveDeadzone } }, s, true));
		CHECK_FALSE(freeze({ kNone, kNone, { 0.f, over } }, s, true));
	}

	const MovementScheme s3 = MovementScheme::AimRelativeSwapped;
	CHECK(freeze({ kNone, kNone, { under, 0.f } }, s3, false, false));
	CHECK_FALSE(freeze({ kNone, kNone, { kMoveDeadzone, 0.f } }, s3, false, false));
	CHECK_FALSE(freeze({ kNone, kNone, { over, 0.f } }, s3, false, false));
	CHECK(freeze({ kNone, kNone, { under, 0.f } }, s3, false, true));
	CHECK(freeze({ kNone, kNone, { kMoveDeadzone, 0.f } }, s3, false, true));
	CHECK_FALSE(freeze({ kNone, kNone, { over, 0.f } }, s3, false, true));
}

TEST_CASE("GuardFreeze: counterexample (a) - scheme 3, right neutral, keys and left cancelling, roots",
	"[SimulatableBrawler][GuardFreeze]")
{
	const MovementScheme s3 = MovementScheme::AimRelativeSwapped;
	const StickSources cancelling{ kKeysUp, -kKeysUp, kNone };
	REQUIRE(glm::length(routeSticks(cancelling, s3, false, false, kMoveDeadzone).move) >= kMoveDeadzone);
	REQUIRE(actualMoveLength(cancelling, s3, false, true) == 0.f);
	CHECK(freeze(cancelling, s3, false, true));

	const StickSources cancellingWithDrift{ kKeysUp, -kKeysUp, kRightDrift };
	REQUIRE(glm::length(routeSticks(cancellingWithDrift, s3, false, false, kMoveDeadzone).move) >= kMoveDeadzone);
	REQUIRE(actualMoveLength(cancellingWithDrift, s3, false, true) < kMoveDeadzone);
	CHECK(freeze(cancellingWithDrift, s3, false, true));
}

TEST_CASE("GuardFreeze: counterexample (b) - scheme 3, right stick exactly at the deadzone, roots",
	"[SimulatableBrawler][GuardFreeze]")
{
	const MovementScheme s3 = MovementScheme::AimRelativeSwapped;
	const StickSources atDeadzone{ kNone, kNone, { 0.f, kMoveDeadzone } };
	REQUIRE(glm::length(routeSticks(atDeadzone, s3, false, false, kMoveDeadzone).move) == kMoveDeadzone);
	REQUIRE(actualMoveLength(atDeadzone, s3, false, true) == 0.f);
	CHECK(freeze(atDeadzone, s3, false, true));
}

TEST_CASE("GuardFreeze: invariant - guard held and not frozen implies the routed move reaches the deadzone",
	"[SimulatableBrawler][GuardFreeze]")
{
	const float under = std::nextafter(kMoveDeadzone, 0.f);
	const float over = std::nextafter(kMoveDeadzone, 1.f);
	const std::vector<glm::vec2> sticks = { kNone, kLeftDrift, kRightDrift, { 0.01f, 0.f }, { under, 0.f },
		{ kMoveDeadzone, 0.f }, { 0.f, -kMoveDeadzone }, { over, 0.f }, { 0.1f, 0.1f }, { 0.11f, -0.11f },
		{ 0.5f, 0.f }, { 0.f, 1.f }, { 0.f, -1.f }, { -1.f, 0.f }, { 1.f, 0.f }, { 0.6f, 0.8f },
		{ -0.6f, -0.8f } };
	std::vector<glm::vec2> keys;
	for (const float x : { -1.f, 0.f, 1.f })
		for (const float y : { -1.f, 0.f, 1.f })
			keys.push_back({ x, y });

	int checked = 0;
	int violations = 0;
	int notFrozen = 0;
	int frozenOnlyByActualMove = 0;
	int frozenOnlyByOwnMove = 0;
	for (const MovementScheme s : kAllSchemes)
		for (const bool legacySwap : { false, true })
			for (const bool feedsAim : { false, true })
				for (const glm::vec2& k : keys)
					for (const glm::vec2& l : sticks)
						for (const glm::vec2& r : sticks)
						{
							const StickSources src{ k, l, r };
							const bool frozen = freeze(src, s, legacySwap, feedsAim);
							const float actual = actualMoveLength(src, s, legacySwap, feedsAim);
							const float own = actualMoveLength(src, s, legacySwap, false);
							++checked;
							if (!frozen)
							{
								++notFrozen;
								if (actual < kMoveDeadzone)
								{
									++violations;
									FAIL_CHECK("not frozen with routed move " << actual << " scheme "
										<< static_cast<int>(s) << " swap " << legacySwap << " feedsAim "
										<< feedsAim << " keys (" << k.x << "," << k.y << ") L (" << l.x
										<< "," << l.y << ") R (" << r.x << "," << r.y << ")");
								}
							}
							else if (own >= kMoveDeadzone)
								++frozenOnlyByActualMove;
							else if (actual >= kMoveDeadzone)
								++frozenOnlyByOwnMove;
							if (freeze(src, s, legacySwap, feedsAim, false))
								FAIL_CHECK("guard released but frozen");
						}

	INFO("checked " << checked << " notFrozen " << notFrozen << " frozenOnlyByActualMove "
		<< frozenOnlyByActualMove << " frozenOnlyByOwnMove " << frozenOnlyByOwnMove);
	CHECK(checked == 4 * 2 * 2 * 9 * 17 * 17);
	CHECK(violations == 0);
	CHECK(notFrozen > 0);
	CHECK(frozenOnlyByActualMove > 0);
	CHECK(frozenOnlyByOwnMove > 0);
}

} // namespace guardFreezeRequestTest

#endif // WITH_LOW_LEVEL_TESTS

// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerInputPackaging.h"
#include "OGBrawler/InputSequence/InputSequence.h"

#include "MockInputSource.h"

// ---------------------------------------------------------------------------
// T12 / D5.4 — render-safe continuous-only input packaging.
//
// These cases exercise the PURE ASSEMBLY half of buildLatestVisualizationInput:
// simulatableBrawler::readContinuousInputFields + makeVisualizationPlayerInput,
// the exact two calls the UE-side UOGBrawlerInputCollectionComponent method is
// composed of. This test tree cannot link UE (OGBrawlerTests.Build.cs depends on
// Core/OGSimulation/OGBrawler only, and the CMake LLT links og_brawler +
// og_simulation + Catch2 + glm), which is precisely why the assembly lives in the
// engine-agnostic core and is templated on its source: MockInputSource stands in
// for the component with zero UE linkage.
//
// What is NOT covered here (and cannot be, from this tree): the live UE read
// itself — buildAimDirection()'s camera/mouse-aim resolution and Enhanced Input
// state. That is UE-side by design; see the seam note in BrawlerInputPackaging.h.
// ---------------------------------------------------------------------------

TEST_CASE("Visualization input carries the live continuous fields", "[InputPackaging][VisualizationInput]")
{
	MockInputSource src;
	src.aimDirection       = glm::vec3(0.6f, -0.8f, 0.f);
	src.moveStick          = glm::vec2(0.25f, -0.75f);
	src.moveDirectionWorld = glm::vec3(-0.5f, 0.5f, 0.f);

	const simulatableBrawler::PlayerInput viz =
		simulatableBrawler::makeVisualizationPlayerInput(
			simulatableBrawler::readContinuousInputFields(src));

	// Continuous fields land on the fields that carry them.
	REQUIRE(viz.aimDirection == src.aimDirection);

	REQUIRE(viz.moveStick == src.moveStick);
	REQUIRE(viz.moveDirectionWorld == src.moveDirectionWorld);
}

TEST_CASE("Visualization input leaves every discrete field neutral", "[InputPackaging][VisualizationInput]")
{
	MockInputSource src;
	src.aimDirection       = glm::vec3(1.f, 0.f, 0.f);
	src.moveStick          = glm::vec2(1.f, 1.f);
	src.moveDirectionWorld = glm::vec3(1.f, 0.f, 0.f);

	// The mock also holds discrete state, and it is deliberately set to the
	// "everything pressed" extreme — the visualization packer has no way to read
	// it, so the neutral result below is a structural guarantee, not a coincidence
	// of an idle mock.
	src.leftAttack  = true;
	src.rightAttack = true;

	const simulatableBrawler::PlayerInput viz =
		simulatableBrawler::makeVisualizationPlayerInput(
			simulatableBrawler::readContinuousInputFields(src));

	// The motion matcher was not run: triggeredActionId is untouched.
	REQUIRE(viz.triggeredActionId == inputSequence::kNoMatch);

	// Attack edges cannot render-echo.
	REQUIRE(viz.attackLeft  == false);
	REQUIRE(viz.attackRight == false);
}

TEST_CASE("Continuous read reflects a changed source on every call", "[InputPackaging][VisualizationInput]")
{
	// The render echo's whole purpose is freshness: two samples taken from the same
	// source at different moments must reflect the source's current value, with no
	// caching or tick quantization in between.
	MockInputSource src;
	src.aimDirection = glm::vec3(0.f, 1.f, 0.f);

	const simulatableBrawler::ContinuousInputFields first =
		simulatableBrawler::readContinuousInputFields(src);

	src.aimDirection = glm::vec3(0.f, -1.f, 0.f);

	const simulatableBrawler::ContinuousInputFields second =
		simulatableBrawler::readContinuousInputFields(src);

	REQUIRE(first.aimDirection  == glm::vec3(0.f, 1.f, 0.f));
	REQUIRE(second.aimDirection == glm::vec3(0.f, -1.f, 0.f));
	REQUIRE(src.readCount == 2);
}

TEST_CASE("Default continuous fields pack to the neutral player input", "[InputPackaging][VisualizationInput]")
{
	// Mirrors the component's !hasInputComponent() early-out, which returns
	// getZeroPlayerInput(). Pinning the two to agree means the cold path and the
	// live path cannot disagree about what "no input" looks like.
	//
	// [og-syncedInput-rework task 3] FIELD FOR FIELD. Until task 3 this case pinned a
	// quirk: getZeroPlayerInput() left the projectile sub-input's aim at (0,0,0) while
	// the packer gave it (0,0,1), and the case asserted that divergence. Task 3 put ONE
	// aim on the wire (simulatableBrawler::SyncedPlayerInput), so the quirk is gone and
	// the two now agree in every field.
	const simulatableBrawler::PlayerInput packed =
		simulatableBrawler::makeVisualizationPlayerInput(simulatableBrawler::ContinuousInputFields{});
	const simulatableBrawler::PlayerInput zero = simulatableBrawler::getZeroPlayerInput();

	REQUIRE(packed.aimDirection == zero.aimDirection);
	REQUIRE(packed.attackLeft == zero.attackLeft);
	REQUIRE(packed.attackRight == zero.attackRight);
	REQUIRE(packed.moveStick == zero.moveStick);
	REQUIRE(packed.moveDirectionWorld == zero.moveDirectionWorld);
	REQUIRE(packed.triggeredActionId == zero.triggeredActionId);
	REQUIRE(packed.flags == zero.flags);
}

#endif // WITH_LOW_LEVEL_TESTS

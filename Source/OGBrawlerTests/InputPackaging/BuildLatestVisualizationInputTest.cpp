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

	// Continuous fields land on every sub-input that carries them.
	REQUIRE(viz.get<dAttackRadialSimulation::PlayerInput>().aimDirection == src.aimDirection);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().aimDirection == src.aimDirection);
	REQUIRE(viz.get<dAttackGuardSimulation::PlayerInput>().aimDirection == src.aimDirection);
	REQUIRE(viz.get<brawlerProjectileSimulation::PlayerInput>().aimDirection == src.aimDirection);

	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().moveDirection == src.moveStick);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld == src.moveDirectionWorld);
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
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId == inputSequence::kNoMatch);

	// Attack edges cannot render-echo.
	REQUIRE(viz.get<dAttackRadialSimulation::PlayerInput>().attackLeft   == false);
	REQUIRE(viz.get<dAttackRadialSimulation::PlayerInput>().attackRight  == false);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().attackLeft  == false);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().attackRight == false);
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
	// ⚠ PRE-EXISTING QUIRK, pinned here rather than silently worked around:
	// getZeroPlayerInput() is NOT internally uniform. It hands aim (0,0,1) to the
	// radial, machine and guard sub-inputs but leaves the projectile sub-input
	// default-constructed, i.e. aim (0,0,0) — see SimulatableBrawlerTypes.h. The
	// LIVE path has never had that asymmetry: buildPlayerInput passes the same
	// aimDirection to all four sub-inputs, and makeSimPlayerInput /
	// makeVisualizationPlayerInput preserve exactly that. So the three uniform
	// slots agree and the projectile slot legitimately does not. Asserting both
	// halves keeps the discrepancy visible; if getZeroPlayerInput is ever made
	// uniform, the second block fails and points straight at this comment.
	const simulatableBrawler::PlayerInput packed =
		simulatableBrawler::makeVisualizationPlayerInput(simulatableBrawler::ContinuousInputFields{});
	const simulatableBrawler::PlayerInput zero = simulatableBrawler::getZeroPlayerInput();

	REQUIRE(packed.get<dAttackRadialSimulation::PlayerInput>().aimDirection
	        == zero.get<dAttackRadialSimulation::PlayerInput>().aimDirection);
	REQUIRE(packed.get<dAttackMachineSimulation::PlayerInput>().aimDirection
	        == zero.get<dAttackMachineSimulation::PlayerInput>().aimDirection);
	REQUIRE(packed.get<dAttackMachineSimulation::PlayerInput>().moveDirection
	        == zero.get<dAttackMachineSimulation::PlayerInput>().moveDirection);
	REQUIRE(packed.get<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld
	        == zero.get<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld);
	REQUIRE(packed.get<dAttackGuardSimulation::PlayerInput>().aimDirection
	        == zero.get<dAttackGuardSimulation::PlayerInput>().aimDirection);
	REQUIRE(packed.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
	        == zero.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId);

	// The documented divergence, asserted explicitly rather than omitted.
	REQUIRE(packed.get<brawlerProjectileSimulation::PlayerInput>().aimDirection
	        == glm::vec3(0.f, 0.f, 1.f));
	REQUIRE(zero.get<brawlerProjectileSimulation::PlayerInput>().aimDirection
	        == glm::vec3(0.f, 0.f, 0.f));
}

#endif // WITH_LOW_LEVEL_TESTS

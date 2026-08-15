// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerInputPackaging.h"
#include "OGBrawler/InputSequence/InputSequence.h"

#include "MockInputSource.h"

// ---------------------------------------------------------------------------
// T12 / D5.4 — the no-drift pin between the sim path and the render-echo path.
//
// This is deliberately NOT a byte-equality test. The two paths are SUPPOSED to
// differ on the discrete/motion-match fields: buildPlayerInput may set attack
// booleans and a triggeredActionId from the tick-stateful matcher;
// buildLatestVisualizationInput never does. Asserting whole-PlayerInput equality
// would encode a false claim and would fail the moment either path does its job.
//
// What IS pinned: given ONE continuous field state, both packers produce the
// SAME continuous fields. That is the property that would silently rot if
// someone re-read the accessors inline on one path instead of going through
// readContinuousInputFields.
//
// The mock reproduces the component's composition exactly:
//   sim path  = readContinuousInputFields(src) + makeSimPlayerInput(..., discrete)
//   viz path  = readContinuousInputFields(src) + makeVisualizationPlayerInput(...)
// ---------------------------------------------------------------------------

namespace
{

// Asserts the continuous fields agree across every sub-input that carries them.
void requireContinuousFieldsMatch(const simulatableBrawler::PlayerInput& sim,
                                  const simulatableBrawler::PlayerInput& viz)
{
	REQUIRE(viz.get<dAttackRadialSimulation::PlayerInput>().aimDirection
	        == sim.get<dAttackRadialSimulation::PlayerInput>().aimDirection);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().aimDirection
	        == sim.get<dAttackMachineSimulation::PlayerInput>().aimDirection);
	REQUIRE(viz.get<dAttackGuardSimulation::PlayerInput>().aimDirection
	        == sim.get<dAttackGuardSimulation::PlayerInput>().aimDirection);
	REQUIRE(viz.get<brawlerProjectileSimulation::PlayerInput>().aimDirection
	        == sim.get<brawlerProjectileSimulation::PlayerInput>().aimDirection);

	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().moveDirection
	        == sim.get<dAttackMachineSimulation::PlayerInput>().moveDirection);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld
	        == sim.get<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld);
}

} // namespace

TEST_CASE("Sim and visualization packers agree on every continuous field", "[InputPackaging][VisualizationInput]")
{
	MockInputSource src;
	src.aimDirection       = glm::vec3(-0.28f, 0.96f, 0.f);
	src.moveStick          = glm::vec2(-0.4f, 0.9f);
	src.moveDirectionWorld = glm::vec3(0.7f, 0.7f, 0.f);
	src.leftAttack         = true;
	src.rightAttack        = false;
	src.triggeredActionId  = inputSequence::kHadoukenActionId;

	// One continuous read, both packers — mirroring the component, where
	// buildPlayerInput and buildLatestVisualizationInput each call
	// readContinuousInputFields(*this) against the same live member state.
	const simulatableBrawler::ContinuousInputFields fields =
		simulatableBrawler::readContinuousInputFields(src);

	const simulatableBrawler::PlayerInput sim =
		simulatableBrawler::makeSimPlayerInput(fields, src.leftAttack, src.rightAttack,
		                                       src.triggeredActionId);
	const simulatableBrawler::PlayerInput viz =
		simulatableBrawler::makeVisualizationPlayerInput(fields);

	requireContinuousFieldsMatch(sim, viz);
}

TEST_CASE("Discrete fields are allowed to differ, and do", "[InputPackaging][VisualizationInput]")
{
	MockInputSource src;
	src.aimDirection       = glm::vec3(1.f, 0.f, 0.f);
	src.moveStick          = glm::vec2(0.3f, 0.3f);
	src.moveDirectionWorld = glm::vec3(1.f, 0.f, 0.f);
	src.leftAttack         = true;
	src.rightAttack        = true;
	src.triggeredActionId  = inputSequence::kHadoukenActionId;

	const simulatableBrawler::ContinuousInputFields fields =
		simulatableBrawler::readContinuousInputFields(src);

	const simulatableBrawler::PlayerInput sim =
		simulatableBrawler::makeSimPlayerInput(fields, src.leftAttack, src.rightAttack,
		                                       src.triggeredActionId);
	const simulatableBrawler::PlayerInput viz =
		simulatableBrawler::makeVisualizationPlayerInput(fields);

	// Continuous half still identical...
	requireContinuousFieldsMatch(sim, viz);

	// ...while the discrete half diverges, by design. This is the assertion that
	// makes the "byte equality would be wrong" claim concrete rather than a comment.
	REQUIRE(sim.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
	        == inputSequence::kHadoukenActionId);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
	        == inputSequence::kNoMatch);
	REQUIRE(sim.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
	        != viz.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId);

	REQUIRE(sim.get<dAttackRadialSimulation::PlayerInput>().attackLeft  == true);
	REQUIRE(viz.get<dAttackRadialSimulation::PlayerInput>().attackLeft  == false);
	REQUIRE(sim.get<dAttackMachineSimulation::PlayerInput>().attackRight == true);
	REQUIRE(viz.get<dAttackMachineSimulation::PlayerInput>().attackRight == false);
}

TEST_CASE("Agreement holds when the sim path carries no discrete input", "[InputPackaging][VisualizationInput]")
{
	// The idle case: with no buttons and no motion match, the two packers converge
	// on identical output. This is the boundary that shows the divergence above is
	// caused purely by the discrete arguments and not by two different assemblies.
	MockInputSource src;
	src.aimDirection       = glm::vec3(0.f, -1.f, 0.f);
	src.moveStick          = glm::vec2(0.f, 0.f);
	src.moveDirectionWorld = glm::vec3(0.f, 0.f, 0.f);

	const simulatableBrawler::ContinuousInputFields fields =
		simulatableBrawler::readContinuousInputFields(src);

	const simulatableBrawler::PlayerInput sim =
		simulatableBrawler::makeSimPlayerInput(fields, false, false, inputSequence::kNoMatch);
	const simulatableBrawler::PlayerInput viz =
		simulatableBrawler::makeVisualizationPlayerInput(fields);

	requireContinuousFieldsMatch(sim, viz);
	REQUIRE(sim.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
	        == viz.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId);
	REQUIRE(sim.get<dAttackRadialSimulation::PlayerInput>().attackLeft
	        == viz.get<dAttackRadialSimulation::PlayerInput>().attackLeft);
}

#endif // WITH_LOW_LEVEL_TESTS

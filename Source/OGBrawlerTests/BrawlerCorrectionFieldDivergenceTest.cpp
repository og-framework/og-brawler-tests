// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationComparison.h"

#include <string>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-field-defects / task 6 — THE THREE PLANTED FIELDS, ON THE REAL
// COMPOSITE.
//
// `Network/CorrectionFieldDivergenceTest.cpp` in og-simulation-tests covers the
// cost gate, the plumbing and the log line over mock slices. It CANNOT cover
// this: og-simulation does not know what a brawler is, and the fields the
// 2026-09-21 knockback analysis is waiting on are all in og-brawler.
//
// WHY THESE THREE AND NOT ANY THREE. Each one is a different arm of the walk,
// and each one is a field the analysis names:
//
//   * `dAttackMachineSimulation::State.m_currentState` — an ENUM, printed as
//     old/new rather than a delta. This is the field the bug report's D1 says
//     the two peers disagree about (the client predicts `Knockback`, the
//     authority produces `GuardFlinch`), and a magnitude would be meaningless
//     for it.
//   * `dAttackRadialSimulation::State.bodyState.angularVelocity` — a NESTED
//     glm::vec3 two levels down. This is D2's INFERRED culprit: the weapon keeps
//     its swing spin because the correction bypasses `deactivate()`. The whole
//     point of task 6 is that "inferred" becomes "measured", and the path has to
//     descend through `bodyState` to say so.
//   * `brawlerMovementSimulation::State.bodyState.position` — a nested
//     LinearBodyState, the SLIM body shape. It shares no descriptor with the
//     radial case: the radial body is a 52-byte PhysicsBodyState and this is the
//     24-byte one, so a walk that only worked for the full shape would pass the
//     case above and fail here.
//
// ⛔ THE BOOLEAN VERDICT IS ASSERTED IN EVERY CASE, ON THE SAME PAIR, BEFORE THE
// DESCRIPTION IS LOOKED AT. `isSimilarTo` is the production resim trigger; this
// feature is additive or it is a regression, and these REQUIREs are what say
// which. They also stop each case describing a divergence that does not exist:
// a description of an agreeing pair would be describing nothing.
//
// ⚠ THE PAIRS ARE HAND-BUILT AND NOT INTEGRATED. Nothing here runs a simulation
// tick — the claim under test is about the COMPARISON, not about how a state
// comes to differ. Driving it through integrate would couple these cases to
// every sub-simulation's behaviour and make a failure ambiguous.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	// Open the gate for the duration of a case. The walk's ENTRY POINT is called
	// directly here rather than through the cache, so the gate is not strictly
	// required — but leaving it closed in one file and open in another is how a
	// global knob acquires a case that only passes in a particular run order.
	class OpenGate
	{
	public:
		OpenGate()
		{
			correctionFieldDiff::setEnabledPredicate([]() { return true; });
			correctionFieldDiff::resetWalkCount();
		}
		~OpenGate()
		{
			correctionFieldDiff::clearEnabledPredicate();
			correctionFieldDiff::resetWalkCount();
		}
		OpenGate(const OpenGate&) = delete;
		OpenGate& operator=(const OpenGate&) = delete;
	};

	std::string pathOf(const FieldDivergence& divergence)
	{
		return std::string(divergence.path);
	}
} // namespace

// ---------------------------------------------------------------------------
// IDENTICAL PAIR — THE VACUITY CONTROL. It runs FIRST because every case below
// is a claim about a DIFFERENCE, and a walk that named a field here would name
// one anywhere.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.CorrectionFieldDivergence.IdenticalCompositesNameNothing",
          "[DAttack][DivergenceProbe]")
{
	OpenGate gate;

	simulatableBrawler::State predicted;
	simulatableBrawler::State authority;

	// The control: the fold agrees.
	REQUIRE(predicted.isSimilarTo(authority));

	FieldDivergence divergence;
	describeFirstDivergingField(predicted, authority, divergence);

	REQUIRE(divergence.evaluated);
	REQUIRE_FALSE(divergence.named());
	REQUIRE(divergence.kind == FieldDivergenceKind::None);

	// ...and it says so explicitly rather than printing a bare type name, which
	// would read like an answer.
	REQUIRE(pathOf(divergence) == "<none>");

	// The walk DID run — this zero is about what it found, not about the gate.
	REQUIRE(correctionFieldDiff::walkCount() == 1u);
}

// ---------------------------------------------------------------------------
// THE MACHINE ENUM — old/new, no magnitude.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.CorrectionFieldDivergence.MachineStateEnumIsNamedWithOldAndNew",
          "[DAttack][DivergenceProbe]")
{
	OpenGate gate;

	simulatableBrawler::State predicted;
	simulatableBrawler::State authority;

	// The client predicted a swing; the authority says the swing was guarded. The
	// two enumerators are read back from the composite below rather than re-typed,
	// so a renumbering of DAttackState cannot make this case quietly wrong.
	predicted.edit<dAttackMachineSimulation::State>().m_currentState = DAttackState::Attacking;
	authority.edit<dAttackMachineSimulation::State>().m_currentState = DAttackState::GuardFlinch;

	REQUIRE_FALSE(predicted.isSimilarTo(authority));    // the control

	FieldDivergence divergence;
	describeFirstDivergingField(predicted, authority, divergence);

	REQUIRE(pathOf(divergence) == "dAttackMachineSimulation::State.m_currentState");
	REQUIRE(divergence.kind == FieldDivergenceKind::Discrete);
	REQUIRE(divergence.oldValue == static_cast<long long>(DAttackState::Attacking));
	REQUIRE(divergence.newValue == static_cast<long long>(DAttackState::GuardFlinch));
	REQUIRE(divergence.oldValue != divergence.newValue);
}

// ---------------------------------------------------------------------------
// THE RADIAL WEAPON'S ANGULAR VELOCITY — the bug report's inferred culprit,
// two levels down, with its magnitude.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.CorrectionFieldDivergence.RadialBodyAngularVelocityIsNamedWithItsMagnitude",
          "[DAttack][DivergenceProbe]")
{
	OpenGate gate;

	simulatableBrawler::State predicted;
	simulatableBrawler::State authority;

	// The client's weapon kept its swing spin; the authority's is at rest. The
	// planted magnitude is on ONE axis and the other two are equal, which is what
	// makes `delta` readable as "the max component moved by this much".
	predicted.edit<dAttackRadialSimulation::State>().bodyState.angularVelocity =
		glm::vec3(0.f, 0.f, 12.25f);

	REQUIRE_FALSE(predicted.isSimilarTo(authority));    // the control

	FieldDivergence divergence;
	describeFirstDivergingField(predicted, authority, divergence);

	REQUIRE(pathOf(divergence) == "dAttackRadialSimulation::State.bodyState.angularVelocity");
	REQUIRE(divergence.kind == FieldDivergenceKind::Numeric);
	REQUIRE(divergence.delta == Catch::Approx(12.25f));

	// THE MAX COMPONENT, NOT A NORM — asserted rather than assumed, because the
	// two are equal for a single-axis plant and a norm would silently pass above.
	// Planting a SECOND axis of the same size leaves `delta` where it is; a norm
	// would report sqrt(2) times it.
	simulatableBrawler::State twoAxis = predicted;
	twoAxis.edit<dAttackRadialSimulation::State>().bodyState.angularVelocity =
		glm::vec3(0.f, 12.25f, 12.25f);

	FieldDivergence twoAxisDivergence;
	describeFirstDivergingField(twoAxis, authority, twoAxisDivergence);
	REQUIRE(twoAxisDivergence.delta == Catch::Approx(12.25f));
}

// ---------------------------------------------------------------------------
// THE MOVEMENT BODY'S POSITION — the SLIM 24-byte body shape, a different
// descriptor from the radial case above.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.CorrectionFieldDivergence.MovementLinearBodyStatePositionIsNamedWithItsMagnitude",
          "[DAttack][DivergenceProbe]")
{
	OpenGate gate;

	simulatableBrawler::State predicted;
	simulatableBrawler::State authority;

	// The knockback the client predicted and the authority never produced: the
	// target is 25 units downrange on one peer and at rest on the other.
	predicted.edit<brawlerMovementSimulation::State>().bodyState.position =
		glm::vec3(25.f, 0.f, 0.f);

	REQUIRE_FALSE(predicted.isSimilarTo(authority));    // the control

	FieldDivergence divergence;
	describeFirstDivergingField(predicted, authority, divergence);

	REQUIRE(pathOf(divergence) == "brawlerMovementSimulation::State.bodyState.position");
	REQUIRE(divergence.kind == FieldDivergenceKind::Numeric);
	REQUIRE(divergence.delta == Catch::Approx(25.f));
}

// ---------------------------------------------------------------------------
// THE PLANT IS WHAT IS NAMED, NOT THE COMPOSITE'S FIRST ELEMENT.
//
// ⛔ THIS IS THE CASE THAT CATCHES A WALK THAT ALWAYS ANSWERS THE SAME THING.
// The three cases above each plant into a DIFFERENT element of an eleven-element
// composite, and each names its own — but they do so in three separate runs. A
// walk that returned, say, the first element whose type it could parse would
// have to be caught by a case where the plant MOVES and the answer moves with
// it, on one pair, in one run.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.CorrectionFieldDivergence.TheNamedFieldFollowsThePlant",
          "[DAttack][DivergenceProbe]")
{
	OpenGate gate;

	const simulatableBrawler::State authority;

	const auto nameFor = [&authority](auto&& plant) {
		simulatableBrawler::State predicted;
		plant(predicted);
		REQUIRE_FALSE(predicted.isSimilarTo(authority));
		FieldDivergence divergence;
		describeFirstDivergingField(predicted, authority, divergence);
		return std::string(divergence.path);
	};

	const std::string machinePath = nameFor([](simulatableBrawler::State& s) {
		s.edit<dAttackMachineSimulation::State>().m_currentState = DAttackState::Attacking;
	});
	const std::string radialPath = nameFor([](simulatableBrawler::State& s) {
		s.edit<dAttackRadialSimulation::State>().attackTimer = 0.5f;
	});
	const std::string movementPath = nameFor([](simulatableBrawler::State& s) {
		s.edit<brawlerMovementSimulation::State>().bodyState.linearVelocity = glm::vec3(3.f, 0.f, 0.f);
	});

	REQUIRE(machinePath  == "dAttackMachineSimulation::State.m_currentState");
	REQUIRE(radialPath   == "dAttackRadialSimulation::State.attackTimer");
	REQUIRE(movementPath == "brawlerMovementSimulation::State.bodyState.linearVelocity");

	// Three plants, three different answers. Pairwise distinct is the property; a
	// constant-answer walk fails it whatever the constant is.
	REQUIRE(machinePath != radialPath);
	REQUIRE(radialPath != movementPath);
	REQUIRE(machinePath != movementPath);
}

#endif // WITH_LOW_LEVEL_TESTS

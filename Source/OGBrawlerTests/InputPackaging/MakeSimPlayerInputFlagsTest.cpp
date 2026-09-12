// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <type_traits>

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerInputPackaging.h"
#include "OGBrawler/InputSequence/InputSequence.h"

// ---------------------------------------------------------------------------
// [movement-sim task 52] THE INPUT WRITER'S OWN TESTS — findings F-3 and F-4 of task 14's
// review.
//
// F-3, verbatim: "NOTHING EXERCISES THE WRITER." Task 14 made holdGuard live end to end
// (makeSimPlayerInput -> flags bit 0 -> step 1's freeze gate) and task 12 wrote 33 movement
// cases, but no test anywhere CALLED the writer. The READER was covered — step 1's `frozen`
// gate is exercised by the movement suite, which seeds `PlayerInput::flags` by hand — and
// that is exactly why the gap survived review: a seeded-by-hand flags byte proves the gate
// reads the bit, and proves nothing at all about who writes it.
//
// So every case below reads the flags byte back OFF THE ASSEMBLED COMPOSITE
// (`packed.get<brawlerMovementSimulation::PlayerInput>().flags`) rather than off a helper's
// return value. That is the byte the ring serializes and the byte step 1 consumes; a writer
// that computed the right value and dropped it into the wrong slice would pass a helper test
// and fail these.
//
// F-4 is the call shape, and its fences are the last case in this file.
//
// ⛔ NOTE ON HOW THE EXPECTED VALUES ARE SPELLED. Every assertion names
// `kInputFlagHoldGuard`; not one of them writes `1`, `0x01` or `1u << 0`. Task 51 shipped the
// constant at bit 0 and task 14's recipe originally read `{ getHoldGuard() }`, which compiles
// and is correct ONLY because holdGuard happens to sit at bit 0 today. A test that pinned the
// numeral would keep passing while the writer and the reader disagreed about which bit they
// meant. Naming the constant makes these cases FOLLOW the bit if it ever moves — which it
// may, since bits 1-7 are already spoken for by wall-grab (20), jump (21), dash (31) and
// ski-tuck (48).
// ---------------------------------------------------------------------------

namespace
{

namespace movement = brawlerMovementSimulation;

// A non-neutral continuous pose. Deliberately NOT the default-constructed one: several cases
// below assert that nothing but the flag fields reaches the flags byte, and a neutral pose
// would let a writer that leaked `moveStick` into `flags` pass by accident.
simulatableBrawler::ContinuousInputFields livePose()
{
	simulatableBrawler::ContinuousInputFields fields;
	fields.aimDirection       = glm::vec3(0.f, -1.f, 0.f);
	fields.moveStick          = glm::vec2(-0.6f, 0.8f);
	fields.moveDirectionWorld = glm::vec3(0.8f, 0.6f, 0.f);
	return fields;
}

// The flags byte AS THE SIMULATION WILL SEE IT — off the composite slice, not off an
// intermediate. See the file header for why this indirection is the point of the test.
uint8_t packedFlags(const simulatableBrawler::PlayerInput& packed)
{
	return packed.get<movement::PlayerInput>().flags;
}

// Every bit that is NOT holdGuard. Written as a mask rather than as a list so it keeps
// covering the seven reserved bits as they are claimed, without an edit here.
constexpr uint8_t kEveryOtherBit = static_cast<uint8_t>(~movement::kInputFlagHoldGuard);

} // namespace

// ===========================================================================
// F-3 — the writer is exercised.
// ===========================================================================

TEST_CASE("InputWriter.PressedGuardRaisesExactlyTheHoldGuardBit", "[BrawlerMovement][InputWriter]")
{
	const simulatableBrawler::PlayerInput packed = simulatableBrawler::makeSimPlayerInput(
		livePose(), /*leftAttack*/ false, /*rightAttack*/ false, inputSequence::kNoMatch,
		simulatableBrawler::InputFlagFields{.holdGuard = true});

	const uint8_t flags = packedFlags(packed);

	// The bit is set...
	REQUIRE((flags & movement::kInputFlagHoldGuard) != 0u);
	// ...no other bit is...
	REQUIRE(static_cast<uint8_t>(flags & kEveryOtherBit) == 0u);
	// ...which together say the byte is exactly the constant, asserted directly as well so a
	// failure prints the byte rather than the result of a mask.
	REQUIRE(flags == movement::kInputFlagHoldGuard);

	// And it is DISTINGUISHABLE from the neutral input. Without this line a writer that
	// returned zero would still satisfy "no other bit is set", and `zero()` is precisely what
	// the pre-task-14 writer produced for every input.
	REQUIRE(flags != movement::PlayerInput::zero().flags);
}

TEST_CASE("InputWriter.ReleasedGuardPacksToTheNeutralFlagsByte", "[BrawlerMovement][InputWriter]")
{
	// Spelled out rather than omitted — omitting it does not compile any more, which is the
	// last case in this file.
	const simulatableBrawler::PlayerInput released = simulatableBrawler::makeSimPlayerInput(
		livePose(), /*leftAttack*/ false, /*rightAttack*/ false, inputSequence::kNoMatch,
		simulatableBrawler::InputFlagFields{.holdGuard = false});

	// The all-defaults form, which is what every caller that does not model a guard press now
	// writes. It must be indistinguishable from spelling `holdGuard = false` — that equivalence
	// is what makes `{}` a safe thing to ask seven call sites to write.
	const simulatableBrawler::PlayerInput defaulted = simulatableBrawler::makeSimPlayerInput(
		livePose(), /*leftAttack*/ false, /*rightAttack*/ false, inputSequence::kNoMatch,
		simulatableBrawler::InputFlagFields{});

	REQUIRE(packedFlags(released) == 0u);
	REQUIRE(packedFlags(defaulted) == 0u);
	REQUIRE(packedFlags(released) == packedFlags(defaulted));

	// ⭐ And the neutral is the sub-simulation's OWN zero(), not a local 0 this test invented.
	// brawlerMovementSimulation::PlayerInput::zero() is what SimulationComposite::zero() folds
	// in, so this is the assertion that keeps `{}` and "the neutral input" the same thing. If a
	// future flag's neutral value were ever non-zero, the packing rule at the type and this line
	// would disagree, loudly, here.
	REQUIRE(packedFlags(released) == movement::PlayerInput::zero().flags);
	REQUIRE(packedFlags(defaulted) == movement::PlayerInput::zero().flags);
}

TEST_CASE("InputWriter.NoOtherInputFieldReachesTheFlagsByte", "[BrawlerMovement][InputWriter]")
{
	// The flags byte is written from the flag fields and from NOTHING ELSE. The attack
	// booleans and triggeredActionId travel to their own sub-inputs and must not touch this
	// byte; a writer that OR-ed an attack in would be caught here and by nothing else in the
	// suite, because every other input test reads the sub-input it cares about.
	for (const bool leftAttack : {false, true})
	{
		for (const bool rightAttack : {false, true})
		{
			for (const uint32_t actionId : {inputSequence::kNoMatch, 4242u})
			{
				const simulatableBrawler::PlayerInput idle = simulatableBrawler::makeSimPlayerInput(
					livePose(), leftAttack, rightAttack, actionId,
					simulatableBrawler::InputFlagFields{});
				const simulatableBrawler::PlayerInput held = simulatableBrawler::makeSimPlayerInput(
					livePose(), leftAttack, rightAttack, actionId,
					simulatableBrawler::InputFlagFields{.holdGuard = true});

				INFO("leftAttack=" << leftAttack << " rightAttack=" << rightAttack
				                   << " triggeredActionId=" << actionId);
				REQUIRE(packedFlags(idle) == 0u);
				REQUIRE(packedFlags(held) == movement::kInputFlagHoldGuard);
			}
		}
	}
}

TEST_CASE("InputWriter.GuardPressChangesTheFlagsByteAndNothingElse", "[BrawlerMovement][InputWriter]")
{
	// The mirror of the case above: the flag fields reach the flags byte and NOTHING ELSE.
	// The realistic defect this catches is a transposition — handing `flagFields.holdGuard` to
	// one of the two attack booleans, which type-checks and which the flags-byte assertions
	// above would not notice, since the byte would still be right.
	const simulatableBrawler::ContinuousInputFields fields = livePose();

	const simulatableBrawler::PlayerInput idle = simulatableBrawler::makeSimPlayerInput(
		fields, /*leftAttack*/ true, /*rightAttack*/ false, inputSequence::kHadoukenActionId,
		simulatableBrawler::InputFlagFields{});
	const simulatableBrawler::PlayerInput held = simulatableBrawler::makeSimPlayerInput(
		fields, /*leftAttack*/ true, /*rightAttack*/ false, inputSequence::kHadoukenActionId,
		simulatableBrawler::InputFlagFields{.holdGuard = true});

	// The byte moved, so the two composites really are the pressed/released pair...
	REQUIRE(packedFlags(idle) != packedFlags(held));

	// ...and every other field a sub-input carries is untouched by the press.
	REQUIRE(held.get<dAttackRadialSimulation::PlayerInput>().aimDirection
	        == idle.get<dAttackRadialSimulation::PlayerInput>().aimDirection);
	REQUIRE(held.get<dAttackRadialSimulation::PlayerInput>().attackLeft
	        == idle.get<dAttackRadialSimulation::PlayerInput>().attackLeft);
	REQUIRE(held.get<dAttackMachineSimulation::PlayerInput>().aimDirection
	        == idle.get<dAttackMachineSimulation::PlayerInput>().aimDirection);
	REQUIRE(held.get<dAttackMachineSimulation::PlayerInput>().attackRight
	        == idle.get<dAttackMachineSimulation::PlayerInput>().attackRight);
	REQUIRE(held.get<dAttackMachineSimulation::PlayerInput>().moveDirection
	        == idle.get<dAttackMachineSimulation::PlayerInput>().moveDirection);
	REQUIRE(held.get<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld
	        == idle.get<dAttackMachineSimulation::PlayerInput>().moveDirectionWorld);
	REQUIRE(held.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId
	        == idle.get<dAttackMachineSimulation::PlayerInput>().triggeredActionId);
	REQUIRE(held.get<dAttackGuardSimulation::PlayerInput>().aimDirection
	        == idle.get<dAttackGuardSimulation::PlayerInput>().aimDirection);
	REQUIRE(held.get<brawlerProjectileSimulation::PlayerInput>().aimDirection
	        == idle.get<brawlerProjectileSimulation::PlayerInput>().aimDirection);

	// ⛔ And the press did NOT leak into the attack booleans it sits beside in the argument
	// list. `attackLeft` is true here on BOTH sides on purpose: a transposition that replaced
	// it with holdGuard would flip the released side to false, and the equality above would
	// catch it — but only if the value being compared is not the same by luck, so it is pinned
	// to its expected value directly too.
	REQUIRE(held.get<dAttackRadialSimulation::PlayerInput>().attackLeft  == true);
	REQUIRE(held.get<dAttackMachineSimulation::PlayerInput>().attackRight == false);
}

TEST_CASE("InputWriter.VisualizationPackerNeverRaisesAFlagBit", "[BrawlerMovement][InputWriter]")
{
	// The render-rate path takes continuous fields ONLY, so no flag bit can structurally reach
	// it — but "structurally" is a claim about the code as written, and this is the assertion
	// that keeps it true. holdGuard is a BUTTON, so a discrete input edge that render-echoed
	// would let the cosmetic path show a freeze the simulation never applied.
	REQUIRE(packedFlags(simulatableBrawler::makeVisualizationPlayerInput(livePose())) == 0u);
	REQUIRE(packedFlags(simulatableBrawler::makeVisualizationPlayerInput(
		simulatableBrawler::ContinuousInputFields{})) == 0u);
	REQUIRE(packedFlags(simulatableBrawler::makeVisualizationPlayerInput(livePose()))
	        == movement::PlayerInput::zero().flags);
}

// ===========================================================================
// F-4 — the call shape. These are COMPILE-TIME fences; they are the reason there is no
// runtime "a caller omitted the flag argument" case in this file. Omission is not a mistake
// this suite has to detect any more, because it is not a program.
// ===========================================================================

namespace
{

// True iff `makeSimPlayerInput` is callable with exactly this argument list. Written as a
// requires-expression rather than std::is_invocable_v because the function is a plain
// overload set, not an object, and taking its address would force the very signature the
// fences are trying to observe.
template <typename... Args>
concept PacksAPlayerInputFrom = requires(Args... args) {
	simulatableBrawler::makeSimPlayerInput(args...);
};

using Continuous = const simulatableBrawler::ContinuousInputFields&;
using Flags      = const simulatableBrawler::InputFlagFields&;

} // namespace

TEST_CASE("InputWriter.CallShapeRejectsOmittedAndPositionalFlagArguments", "[BrawlerMovement][InputWriter]")
{
	// ⭐ THE POSITIVE CONTROL, AND IT IS NOT OPTIONAL. Both fences below are negative, so a
	// concept that had gone vacuously false — a typo in the name, an argument list that stopped
	// matching for some unrelated reason — would satisfy them while proving nothing. This line
	// is what says the instrument still detects a call that DOES compile.
	STATIC_REQUIRE(PacksAPlayerInputFrom<Continuous, bool, bool, uint32_t, Flags>);
	STATIC_REQUIRE(PacksAPlayerInputFrom<Continuous, bool, bool, uint32_t,
	                                     simulatableBrawler::InputFlagFields>);

	// ⭐ F-4, THE FENCE THE TASK EXISTS FOR: OMITTING THE FLAG ARGUMENT IS NOT A PROGRAM.
	// Task 14's shape was `bool holdGuard = false`, so this call compiled and silently packed
	// a released guard. It was the right shape for one flag and the wrong shape for five: four
	// more trailing defaulted bools would have been four more callers who meant to send a
	// signal and sent zero instead. The default is gone; a caller must write `{}` and mean it.
	STATIC_REQUIRE_FALSE(PacksAPlayerInputFrom<Continuous, bool, bool, uint32_t>);

	// ⛔ AND A BARE `bool` IN THE FLAGS SLOT IS NOT A PROGRAM EITHER. This is the half that
	// stops the old shape being reintroduced one call at a time, and the half that makes flag
	// #2 safe: with five bools in a row, transposing jump and holdGuard type-checks and ships
	// a wrong bit. InputFlagFields is an aggregate with no converting constructor, so the only
	// way to fill this parameter is to NAME the fields.
	STATIC_REQUIRE_FALSE(PacksAPlayerInputFrom<Continuous, bool, bool, uint32_t, bool>);
	STATIC_REQUIRE_FALSE(std::is_convertible_v<bool, simulatableBrawler::InputFlagFields>);

	// The same guarantee stated at the type: nothing that is not an InputFlagFields converts
	// into one, so no numeric flags byte can be handed in positionally either.
	STATIC_REQUIRE_FALSE(std::is_convertible_v<uint8_t, simulatableBrawler::InputFlagFields>);
	STATIC_REQUIRE_FALSE(PacksAPlayerInputFrom<Continuous, bool, bool, uint32_t, uint8_t>);
}

#endif // WITH_LOW_LEVEL_TESTS

// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/BrawlerMovementSimulation.h"

#include <cstring>
#include <string>
#include <vector>

// ===========================================================================================
// [movement-sim task 16] A RETIRED NAME MUST FAIL LOUDLY — DEMONSTRATED, NOT ASSERTED.
//
// ⭐⭐ THE OBLIGATION, AND WHERE IT CAME FROM. User ruling #28 (movement-sim task 56) removed
// `maxSnapSpeed`: ruling #13's dead-beat velocity assignment was replaced by a spring-damper,
// so the VELOCITY clamp on the assignment had nothing left to clamp. Task 56 could not reject
// the retired name because there was no cvar/ini path in the tree at all — it left the
// breadcrumb at `hoverMaxAccel`'s declaration in BrawlerMovementSimulation.h and routed the
// obligation to task 16, which builds that path.
//
// ⛔ WHAT "LOUDLY" HAS TO MEAN HERE, AND WHY A RETURN CODE ALONE IS NOT IT. Before this task,
// `SetVariable("MaxSnapSpeed", "400")` returned `false` — and so did
// `SetVariable("MvoeSpeed", "400")`, and so does `SetVariable("qqq", "1")`. A retired constant
// and a typo were the SAME OBSERVABLE. That is the defect: an operator carrying a stale tuning
// script gets a silent no-op either way, and nothing tells them the constant is gone rather
// than misspelled. The fix is a table that DISCRIMINATES the two, and these cases measure the
// discrimination rather than the boolean.
//
// ⚠ THE UE HALF OF THE PATH IS NOT MEASURED HERE, and this file must not pretend otherwise:
// the `OGBrawler.*` console variables, their tombstones and the stale-ini sweep live in
// `Source/OGBrawlerUnreal/MovementSchemeCVar.cpp`, which this test module does not link
// (`OGBrawlerTests.Build.cs` depends on Core, OGSimulation and OGBrawler — not OGBrawlerUnreal).
// That half is demonstrated by a headless run in impl/impl_notes_seam_16.md. What IS measured
// here is the TABLE both halves consult, so a name going stale in one is impossible.
//
// Tagged `[BrawlerMovement]` — an already-whitelisted top-level tag in OgTagAliases.cpp, so
// these cases are visible to `[@og]` rather than only to a direct tag invocation.
// ===========================================================================================

namespace
{
    namespace tweak = dAttackMachineSimulation;

    // Every name this path still handles, spelled as `SetVariable`'s own `if` chain spells it.
    // ⛔ A REFUSED NAME MUST NEVER APPEAR HERE, and the fence case below is what enforces it:
    // the refused check runs FIRST inside `SetVariable`, so a live handler with the same
    // spelling would be dead code that silently resurrected a retired constant.
    const std::vector<std::string> kLiveNames = {
        "MovementScheme",
        "MoveStickDeadzone",
        "AimStickDeadzone",
        "SwapMoveAndAimSticks",
        "GamepadMoveStickFeedsAim",
        "MovementAndAimModeTest",   // deprecated-but-live: warns and still applies
    };
}

TEST_CASE("A retired tunable name is refused with a reason, not silently ignored",
          "[BrawlerMovement][RuntimeTweakables]")
{
    // (1) THE RETIRED NAME. `maxSnapSpeed` is the constant ruling #28 deleted.
    const char* reason = tweak::refusedVariableReason("MaxSnapSpeed");
    REQUIRE(reason != nullptr);
    // The reason is the OBSERVABLE, and it has to carry the two things a stale script's owner
    // needs: that the constant is gone, and what replaced it. An empty or generic string would
    // pass a `!= nullptr` check while leaving the defect exactly where it was.
    REQUIRE(std::strstr(reason, "RETIRED") != nullptr);
    REQUIRE(std::strstr(reason, "hoverMaxAccel") != nullptr);

    // (2) SETTING IT STILL FAILS — the value is not applied to anything.
    REQUIRE_FALSE(tweak::SetVariable("MaxSnapSpeed", "400"));

    // (3) ⭐ THE DISCRIMINATION, WHICH IS THE ACTUAL POINT. A near-miss typo returns the SAME
    // boolean and a DIFFERENT reason (none). If this case only checked the boolean it would
    // have passed against the pre-task tree, where both were an unexplained `false`.
    REQUIRE_FALSE(tweak::SetVariable("MaxSnapSpeedd", "400"));
    REQUIRE(tweak::refusedVariableReason("MaxSnapSpeedd") == nullptr);
    REQUIRE(tweak::refusedVariableReason("qqq") == nullptr);

    // (4) CASE-INSENSITIVE, because a stale ini's casing is not something to bet on: ruling
    // #28's own prose, task 56's breadcrumb and this table each spell it differently.
    REQUIRE(tweak::refusedVariableReason("maxsnapspeed") != nullptr);
    REQUIRE(tweak::refusedVariableReason("MAXSNAPSPEED") != nullptr);
    // ⚠ …but not a PREFIX of it, and not an extension. The comparison is whole-string.
    REQUIRE(tweak::refusedVariableReason("MaxSnap") == nullptr);
}

TEST_CASE("The walk speed's named-pipe name is refused now that it is a one-time cvar read",
          "[BrawlerMovement][RuntimeTweakables]")
{
    // The second kind of refusal: the CONSTANT still exists, but this PATH no longer owns it.
    // Task 16 deleted the sim-side walk-speed global and made the walk speed a one-time read of
    // `OGBrawler.MoveSpeed` into the movement sub-simulation's StaticData, so a mid-session set
    // over the named pipe cannot work by construction — and must say so instead of no-opping.
    const char* reason = tweak::refusedVariableReason("MoveSpeed");
    REQUIRE(reason != nullptr);
    REQUIRE(std::strstr(reason, "OGBrawler.MoveSpeed") != nullptr);
    REQUIRE_FALSE(tweak::SetVariable("MoveSpeed", "200"));

    // ⭐ AND IT IS NOT TOMBSTONED ON THE CVAR PATH — the two paths disagree about this one name
    // deliberately, and that flag is what stops the UE layer from tombstoning the live cvar.
    // Getting this backwards would register `OGBrawler.MoveSpeed` as retired and break the very
    // read this task exists to add, so it is worth a case of its own.
    bool found = false;
    for (const tweak::RefusedVariable* v = tweak::refusedVariablesBegin();
         v != tweak::refusedVariablesEnd(); ++v)
    {
        if (std::strcmp(v->name, "MoveSpeed") == 0)
        {
            found = true;
            REQUIRE_FALSE(v->refusedOnCVarPath);
        }
    }
    REQUIRE(found);
}

TEST_CASE("The hover gains are refused as authored-only, on the COUPLED reason",
          "[BrawlerMovement][RuntimeTweakables]")
{
    // ⭐⭐ THE SECOND ROUTED OBLIGATION, RECORDED WHERE SOMEBODY WOULD TRY TO VIOLATE IT.
    // The hover gains' valid region is TWO-DIMENSIONAL — stability is `W² + 4ζW < 4` with
    // `W = ω·dt`, and a monotone (non-ringing) response needs `ω < 1/dt` on top of that. A
    // per-variable range check on ω ALONE cannot express it: the obvious `hoverFrequency < 120`
    // (explicit Euler's bound, and this step is semi-implicit) ADMITS ω = 60 / ζ = 1, which
    // diverges 1, 1, 2, 3, 5, 8, 13.
    //
    // So these two are not exposed, and the refusal says WHY rather than just "unknown". The
    // coupled check itself is NOT duplicated here: it exists once, in
    // `brawlerMovementSimulation::StaticData`'s constructor, where both gains are in scope.
    for (const char* name : { "HoverFrequency", "HoverDampingRatio" })
    {
        const char* reason = tweak::refusedVariableReason(name);
        REQUIRE(reason != nullptr);
        REQUIRE(std::strstr(reason, "COUPLED") != nullptr);
        REQUIRE_FALSE(tweak::SetVariable(name, "46"));
    }
}

TEST_CASE("Every constant the refused table names is genuinely absent or genuinely authored-only",
          "[BrawlerMovement][RuntimeTweakables]")
{
    // ⭐⭐ THE FENCE THAT KEEPS THE TABLE HONEST OVER TIME, and it is the one this task most
    // needed: task 56 wrote its breadcrumb before ruling #29 (task 57) ADDED
    // `hoverPullDownAccel`, so a refused list copied from any document would have been one
    // constant out of date on the day it was written. A table of dead names rots silently — the
    // rot only shows up as a refusal of something that has quietly come back.
    //
    // The three AUTHORED-ONLY names must still EXIST on the shipped StaticData (otherwise their
    // reason text, which points a tuner at where the value lives, is a lie), and the RETIRED
    // one must not. `hoverMaxAccel` is the survivor `maxSnapSpeed`'s reason names, so it is
    // read here for the same purpose: to prove the redirection still lands somewhere.
    const simulatableBrawler::StaticData game;
    const brawlerMovementSimulation::StaticData& sd = game.m_movementStaticData;

    REQUIRE(sd.hoverFrequency     > 0.f);
    REQUIRE(sd.hoverDampingRatio  > 0.f);
    REQUIRE(sd.hoverMaxAccel      > 0.f);
    REQUIRE(sd.hoverPullDownAccel >= 0.f);   // ruling #29's one-sided knob; 0 is its shipped value

    // ⭐ AND THE GAINS THE TABLE DECLINES TO EXPOSE ARE INSIDE THE COUPLED REGION, measured on
    // the same recurrence the StaticData constructor checks. This is the assertion that would
    // have caught the `hoverFrequency < 120` mistake: ω = 60 / ζ = 1 passes that cap and fails
    // BOTH lines below.
    const float dt = brawlerMovementSimulation::kNominalSimStepSeconds;
    const float W  = sd.hoverFrequency * dt;
    REQUIRE(W * W + 4.f * sd.hoverDampingRatio * W < 4.f);   // stability
    REQUIRE(sd.hoverFrequency * dt < 1.f);                   // monotonicity: ω < 1/dt

    // No live handler may share a spelling with a refused name — the refused check runs first,
    // so such a handler would be unreachable and the constant silently un-retired.
    for (const std::string& live : kLiveNames)
        REQUIRE(tweak::refusedVariableReason(live) == nullptr);
}

TEST_CASE("The remaining named-pipe variables still work after the walk speed left the path",
          "[BrawlerMovement][RuntimeTweakables]")
{
    // Task 16's own acceptance criterion: "Named-pipe SetVariable path still compiles; remaining
    // variables work." Deleting a case out of a long `if` chain and inserting a new early-return
    // above it is exactly the edit that quietly eats a neighbouring branch, so the neighbours are
    // exercised rather than eyeballed.
    //
    // ⚠ These are process-global atomics shared with every other case in this binary. Each
    // assertion restores what it moved, in the same case, so nothing leaks into another file.
    const auto scheme0    = tweak::g_movementScheme.load();
    const auto moveDead0  = tweak::g_moveStickDeadzone.load();
    const auto aimDead0   = tweak::g_aimStickDeadzone.load();
    const auto swap0      = tweak::g_swapMoveAndAimSticks.load();
    const auto gamepad0   = tweak::g_gamepadMoveStickFeedsAim.load();

    REQUIRE(tweak::SetVariable("MovementScheme", "CameraRelative"));
    REQUIRE(tweak::g_movementScheme.load() == tweak::MovementScheme::CameraRelative);

    REQUIRE(tweak::SetVariable("MoveStickDeadzone", "0.25"));
    REQUIRE(tweak::g_moveStickDeadzone.load() == Catch::Approx(0.25f));
    // The clamp is part of the contract, not incidental.
    REQUIRE(tweak::SetVariable("AimStickDeadzone", "5"));
    REQUIRE(tweak::g_aimStickDeadzone.load() == Catch::Approx(1.f));

    REQUIRE(tweak::SetVariable("SwapMoveAndAimSticks", "1"));
    REQUIRE(tweak::g_swapMoveAndAimSticks.load());

    REQUIRE(tweak::SetVariable("GamepadMoveStickFeedsAim", "false"));
    REQUIRE_FALSE(tweak::g_gamepadMoveStickFeedsAim.load());

    // An unparseable value still fails, and still is not a refusal.
    REQUIRE_FALSE(tweak::SetVariable("MoveStickDeadzone", "banana"));
    REQUIRE(tweak::refusedVariableReason("MoveStickDeadzone") == nullptr);

    tweak::g_movementScheme          = scheme0;
    tweak::g_moveStickDeadzone       = moveDead0;
    tweak::g_aimStickDeadzone        = aimDead0;
    tweak::g_swapMoveAndAimSticks    = swap0;
    tweak::g_gamepadMoveStickFeedsAim = gamepad0;
}

TEST_CASE("The movement StaticData's five one-time parameters default to the authored literals",
          "[BrawlerMovement][RuntimeTweakables]")
{
    // ⭐⭐ THE PARAMETERISATION MUST BE INVISIBLE TO A DEFAULT-CONSTRUCTED OBJECT.
    // Task 16 turned five authored constants into defaulted constructor parameters so the UE
    // layer can feed them from the one-time cvar read. Every LLT rig and every test peer in both
    // suites default-constructs `simulatableBrawler::StaticData`, so if a default drifted from
    // the literal it replaced, the whole test tree would quietly become a measurement of the
    // parameter list instead of the shipped constants — green, and meaningless.
    const simulatableBrawler::StaticData defaulted;
    const brawlerMovementSimulation::StaticData& sd = defaulted.m_movementStaticData;

    REQUIRE(sd.model == brawlerMovementSimulation::MovementModel::ContinuousAccelBrake); // ruling #10
    REQUIRE(sd.maxWalkSpeed    == Catch::Approx(100.f));   // R-P1's one walk-speed literal
    REQUIRE(sd.stepPeriodTicks == 20u);                    // 1/3 s at the 60 Hz sim clock
    REQUIRE(sd.stepSpeed       == Catch::Approx(100.f));
    REQUIRE(sd.gravity         == Catch::Approx(-980.f));  // NEGATIVE, world −Z

    // And the values the task did NOT parameterise are still the authored ones beside them —
    // a positional constructor call is the classic place for an argument to shift by one.
    REQUIRE(sd.acceleration        == Catch::Approx(2048.f));
    REQUIRE(sd.brakingDeceleration == Catch::Approx(2000.f));
    REQUIRE(sd.maxSlopeAngleDeg    == Catch::Approx(45.f));
    REQUIRE(sd.terminalFallSpeed   == Catch::Approx(2000.f));
    REQUIRE(sd.rideHeight          == Catch::Approx(10.f));
    REQUIRE(sd.snapDistance        == Catch::Approx(40.f));

    // ⭐ AND THE PARAMETERS ACTUALLY REACH THE SUB-SIM — a defaulted-parameter refactor that
    // forwards nothing would pass every assertion above. This is the only place in either suite
    // that passes anything, mirroring the manager's single call site.
    const simulatableBrawler::StaticData tuned(
        brawlerMovementSimulation::MovementModel::Cadence, 200.f, 7u, 333.f, -500.f);
    REQUIRE(tuned.m_movementStaticData.model == brawlerMovementSimulation::MovementModel::Cadence);
    REQUIRE(tuned.m_movementStaticData.maxWalkSpeed    == Catch::Approx(200.f));
    REQUIRE(tuned.m_movementStaticData.stepPeriodTicks == 7u);
    REQUIRE(tuned.m_movementStaticData.stepSpeed       == Catch::Approx(333.f));
    REQUIRE(tuned.m_movementStaticData.gravity         == Catch::Approx(-500.f));
    // ⛔ …while everything NOT parameterised is untouched by the tuned call. This is the arm
    // that catches an argument shifted by one position in the forwarding list.
    REQUIRE(tuned.m_movementStaticData.acceleration        == Catch::Approx(2048.f));
    REQUIRE(tuned.m_movementStaticData.brakingDeceleration == Catch::Approx(2000.f));
    REQUIRE(tuned.m_movementStaticData.terminalFallSpeed   == Catch::Approx(2000.f));
    REQUIRE(tuned.m_movementStaticData.hoverFrequency      == Catch::Approx(sd.hoverFrequency));
    REQUIRE(tuned.m_movementStaticData.capsuleRadius       == Catch::Approx(sd.capsuleRadius));
    REQUIRE(tuned.m_movementStaticData.capsuleHalfHeight   == Catch::Approx(sd.capsuleHalfHeight));
}

#endif // WITH_LOW_LEVEL_TESTS

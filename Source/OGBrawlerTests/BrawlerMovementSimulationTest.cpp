// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================
// THE MOVEMENT SUB-SIMULATION'S LLTs  [movement-sim task 12, 2026-09-06]
//
// Subject: `brawlerMovementSimulation::integrate` -- steps 6', 0, 1, 2, 3, 4, 5 of
// architecture 3.3 as revision 6 and user ruling #14(c) settled them.
//
// ------------------------------- WHAT DRIVES WHAT ---------------------------
// These cases drive the SUB-SIMULATION DIRECTLY, not the whole `SimulatableBrawler`.
// That is a deliberate choice and it buys three things the whole-brawler rig cannot:
//   * an arbitrary `dAttackMachineSimulation::State` as the external dependency, which is
//     the only way to hold the machine in `HitFlinch` / `GuardFlinch` for a tick (the
//     machine sub-simulation would re-derive it every tick and walk straight back out);
//   * a per-case `brawlerMovementSimulation::StaticData` COPY, so `model`, `drivesBody`,
//     `gravity` and `acceleration` are fixture knobs rather than shipped constants.
//     ⚠ A COPY INHERITS PRODUCTION DEFAULTS: `drivesBody` shipped `false` when this file
//     was written and ships `true` since task 15, so `Rig()` PINS it to the passenger arm
//     and the driving-arm cases opt in explicitly. Read the ctor before adding a case;
//     [movement-sim task 15]
//   * a `machineInput` chosen per tick, which is where the movement stick actually lives.
// `SimulatableBrawlerTest.cpp` keeps the whole-brawler cases (`[MovementResim]`, task 50),
// and this file deliberately does NOT restate them -- see THE TASK 50 BOUNDARY below.
//
// ------------------------------ THE PRODUCTION DATA -------------------------
// Every fixture starts from `simulatableBrawler::StaticData`'s own
// `m_movementStaticData`, COPIED. The walk-speed / hover / gravity literals are spelled
// ONCE, at the construction site in `SimulatableBrawlerTypes.h` (R-P1); restating them
// here would create a second place to update and a case that passes against its own copy
// of a stale number. What this file spells is the ARITHMETIC those values imply, named:
//     accelPerTick() = acceleration * dt, brakePerTick(), gravityPerTick(), and -- since
//     [movement-sim task 56 / ruling #28] -- servoStepPerCm(), maxVerticalStep() and
//     discreteCriticalZeta(). `maxSnapStep()` retired with `maxSnapSpeed`.
//
// -------------------------- THE ENGINE STEP IS THE RIG'S ---------------------
// Nothing inside `integrate` advances the body. The generic
// `SimulationIntegrationExecutor::captureBodyStatesAll` pass does, from whatever the
// solver produced, and an engine-free rig has to stand in for it. `Rig::engineStep`
// integrates the command; `Rig::engineStepAndCapture` additionally injects a solver
// position and a solver VELOCITY, and lands both through the real
// `LinearBodyState::operator=(const PhysicsBodyState&)` capture bridge -- which is what
// makes `ContactImpulseNeverEntersVelocity` a measurement rather than an assertion about
// a field nobody wrote.
//
// ---------------------------- THE TASK 50 BOUNDARY ---------------------------
// `ReplayAfterAdoptionReproducesContactClamp` and `AgreeingAnchorKeepsPushOut` belong to
// TASK 50 and live in `SimulatableBrawlerTest.cpp` under `[MovementResim]`. They are about
// a CORRECTION: what survives `injectCorrectionState` + `prepareResimAll` +
// `firstResimStep`, and whether a replayed tick reproduces the live one.
// `WallPushOutZeroesIntoWallComponent` below is about step 6' ITSELF with no correction in
// sight -- the arithmetic that recovers the solver's contribution and the one authored
// contact rule that follows from it. Same wall, different subject; neither is redundant
// with the other, and neither is duplicated here.
//
// ------------------------------- WHAT IS NOT HERE -----------------------------
// The Backlog's case list carries five names whose CODE DOES NOT EXIST in the tree, and
// three of those are structurally unreachable rather than merely unwritten:
//   * `DashAssignsVelocityReplacingMomentum`, `DashHeldForWindowThenDecays` (task 31) and
//     `LaunchedDecelsAtLaunchDecel` (task 27). Step 1's `committed` is a hard
//     `const bool committed = false;` and `DAttackState` is {Attacking, Idle, GuardFlinch,
//     HitFlinch} -- there is no `Dashing` and no `Launched` enumerator to put the machine
//     in. A case written today would exercise the `else` arm under a name that promises
//     the committed one, which is the "passes for the wrong reason" shape this initiative
//     has been bitten by. They belong to the tasks that add the branches.
//   * `DrivesBodyTrueWritesVelocityOnly` was replaced by revision 5 and then by revision 6;
//     `WritesTransformAndVelocityEachTick` below IS its revision-6 successor.
//   * `WireFootprint` (`State`, `InitialConditions`) is ALREADY PINNED, at its CURRENT
//     values, by `SimulatableBrawlerTest.cpp`'s `DAttack.SimulatableBrawler.WireFootprint`
//     (`syncSize<brawlerMovementSimulation::State>() == 61u`,
//     `syncSize<...::InitialConditions>() == 16u`). The Backlog's `State == 37` predates
//     tasks 11 and 50. A second copy of a wire pin is a second place to forget.
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <variant>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/HitReaction.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationSerialization.h"
#include "OGSimulation/SpatialQueryResult.h"

#include "BrawlerSimulationMocks.h"

#include "glm/common.hpp"
#include "glm/geometric.hpp"
#include "glm/trigonometric.hpp"

namespace movementTests
{

namespace movement = brawlerMovementSimulation;
namespace machine  = dAttackMachineSimulation;

constexpr float                kDt       = 1.f / 60.f;
constexpr BodyId               kBodyId{ 1u };
constexpr QueryVolumeId        kVolumeId{ 91u };

// ---------------------------------------------------------------------------
// THE ANALYTIC GROUND.
//
// An infinite plane { p : dot(planeNormal, p) == planeOffset }, swept against the
// character capsule. Written FROM FIRST PRINCIPLES and deliberately NOT from the model
// under test -- that non-circularity is what made task 49's measurement admissible (its
// reviewer reproduced both arms independently to 5e-6 cm) and it is the whole reason
// `HoverOnRampDoesNotDrift` can be believed:
//   * while planeNormal.z > 0 the capsule's nearest point to the plane is on its LOWEST
//     axis endpoint, so the signed gap is dot(n, bottomAxisPoint) - planeOffset - radius;
//   * moving along the unit sweep direction `d` changes that gap at rate dot(n, d), so
//     contact happens at travel = gap / -dot(n, d).
// Neither line quotes the servo, the surface frame, or any candidate derivation of the
// slope coupling.
//
// It is a SEPARATE type from `brawlerTestMocks::MockSpatialQueryAdapter` on purpose: the
// shared mock answers from a script, which by construction cannot respond to the capsule
// MOVING -- and a probe origin that moves with the body is exactly the mechanism the ramp
// case exists to measure. Cases whose clearance is a controlled scalar use the scripted
// mock; cases whose clearance must follow the geometry use this one.
// ---------------------------------------------------------------------------
struct PlaneGroundQueryAdapter
{
    glm::vec3 planeNormal{ 0.f, 0.f, 1.f };
    float     planeOffset = 0.f;
    float     capsuleRadius     = 42.f;
    float     capsuleHalfHeight = 96.f;

    // Recorded per call so a case reads exactly the clearance the sim consumed.
    bool  sawProbe      = false;
    bool  lastBlocked   = false;
    float lastClearance = 0.f;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& pose, const glm::vec3& delta)
    {
        SweepHit hit{};
        if (volumeId != kVolumeId)
            return hit;

        const float length = glm::length(delta);
        if (length <= 0.f)
            return hit;
        const glm::vec3 dir = delta / length;

        const glm::vec3 centre(pose[3].x, pose[3].y, pose[3].z);
        const glm::vec3 bottom =
            centre - glm::vec3(0.f, 0.f, capsuleHalfHeight - capsuleRadius);

        const float gap  = glm::dot(planeNormal, bottom) - planeOffset - capsuleRadius;
        const float rate = -glm::dot(planeNormal, dir);   // closing speed per unit travel

        sawProbe = true;

        if (rate <= 0.f || gap / rate > length)
        {
            hit.blocked   = false;
            hit.fraction  = 1.f;
            lastBlocked   = false;
            lastClearance = length;
            return hit;
        }

        const float travel = gap / rate;
        hit.blocked     = true;
        hit.fraction    = travel > 0.f ? travel / length : 0.f;
        hit.normal      = planeNormal;
        hit.impactPoint = centre + dir * travel;
        lastBlocked     = true;
        lastClearance   = hit.fraction * length;
        return hit;
    }

    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<PlaneGroundQueryAdapter>);

// The capsule centre height that puts `clearance` cm of VERTICAL clearance between the
// capsule and the plane at x == y == 0. It is the mock's OWN gap equation inverted, so a
// seed is placed by the same geometry the probe reads back rather than by a second
// derivation that could disagree with it.
inline float seedZForClearance(const PlaneGroundQueryAdapter& ground, float clearance)
{
    const float axisHalf = ground.capsuleHalfHeight - ground.capsuleRadius;
    return axisHalf + clearance
         + (ground.planeOffset + ground.capsuleRadius) / ground.planeNormal.z;
}

// A scripted floor hit at a chosen clearance, for the cases whose ground is a controlled
// scalar rather than a geometry. `probeLength` is `rideHeight + snapDistance` -- the sim
// recovers `clearance` as `fraction * probeLength`, so the two must agree.
inline SweepHit floorHitAt(float clearance, float probeLength,
                           glm::vec3 normal = glm::vec3(0.f, 0.f, 1.f))
{
    SweepHit hit{};
    hit.blocked     = true;
    hit.fraction    = clearance / probeLength;
    hit.normal      = normal;
    hit.impactPoint = glm::vec3(0.f, 0.f, -clearance);
    return hit;
}

// A unit normal tilted `degrees` away from world up, in the XZ plane (so its `u` axis
// stays in XZ and `v` stays world +Y -- see `buildTangentFrame`).
inline glm::vec3 slopeNormal(float degrees)
{
    const float theta = glm::radians(degrees);
    return glm::vec3(-glm::sin(theta), 0.f, glm::cos(theta));
}

// The THREE CHANNELS of a velocity in the frame a surface normal induces: `(a, b, c)` such that
// `V == a*u + b*v + c*up`. [movement-sim task 57 / ruling #29]
//
// ⚠ WHY A CASE NEEDS THIS AT ALL, AND WHY IT IS NOT CIRCULAR. `state.velocity` is a WORLD
// vector, and on a slope its world `z` mixes the tangential channel's own vertical component with
// the servo's output -- so "the character never accelerates downward faster than gravity" is
// simply not a statement about `velocity.z`, it is a statement about `c`. This function spells the
// dual basis out again rather than calling the header's `decomposeVelocity`, so a case is never
// asking the subject to confirm its own answer; what it DOES borrow is `buildTangentFrame`, which
// is task 12's and is not under test here.
inline glm::vec3 channelsOf(const glm::vec3& velocity, const glm::vec3& n)
{
    glm::vec3 u(1.f, 0.f, 0.f), v(0.f, 1.f, 0.f);
    movement::buildTangentFrame(n, u, v);
    const float     b      = glm::dot(velocity, v);
    const glm::vec3 planar = velocity - b * v;
    const float     s      = glm::dot(u, movement::kWorldUp);
    const float     a =
        (glm::dot(planar, u) - glm::dot(planar, movement::kWorldUp) * s) / (1.f - s * s);
    const float     c = glm::dot(planar, movement::kWorldUp) - a * s;
    return glm::vec3(a, b, c);
}

// ---------------------------------------------------------------------------
// THE RIG.
// ---------------------------------------------------------------------------
template <typename QueryT>
struct Rig
{
    // Production values, from the ONE place that spells them.
    simulatableBrawler::StaticData game;
    // ...COPIED, so a case may tune a knob without touching the shipped aggregate.
    // ⚠ THE COPY CARRIES PRODUCTION *DEFAULTS* TOO -- see the `drivesBody` pin in the ctor.
    movement::StaticData sd;

    movement::InitialConditions ic;
    movement::State             state;
    movement::DerivedState      derived;
    machine::State              machineState;

    // ⭐ [movement-sim task 27] THE INBOUND-HIT SLICE, the second thing the movement sub-sim now
    // reads that the rig has to stand in for. In production `brawlerHitRouting::System::
    // postIntegrate` resolves it from the attacker's wire sequence id and the target's position;
    // here a case writes it directly, exactly as it writes `machineState`. It is a ONE-SHOT: the
    // routing pass clears the whole slice at the top of every tick, so `tick()` clears it after
    // handing it to `integrate` rather than leaving a hit that fires again next tick.
    brawlerInboundHit::DerivedState inbound;

    brawlerTestMocks::MockPhysicsAdapter phys{ 8u };
    QueryT                               query;
    movement::RuntimeBindings            bindings;

    Rig() : sd(game.m_movementStaticData)
    {
        // ⭐⭐ [movement-sim task 15] PIN THE PASSENGER ARM. `drivesBody` SHIPS `true` NOW, and
        // `sd` is a copy of the production aggregate, so without this line every case in this
        // file silently changes what it exercises the moment production flips.
        //
        // Why the passenger arm is the right rig default, and it is not merely inertia: this
        // rig has NO ENGINE. `Rig::engineStep()` is the only thing that ever moves
        // `state.bodyState.position`, and most cases here measure a per-tick quantity
        // (acceleration, braking, gravity, the hover servo) over consecutive `tick()` calls
        // with no engine step between them. Under `drivesBody = true` step 5 sets
        // `kFlagHasCommand`, which un-gates step 6', which then reads
        // `pushOut = position - (positionCmd + velocity*dt) = -velocity*dt` -- a push-out that
        // exists only because nothing integrated the body -- and the authored contact rule
        // zeroes the character's own direction of travel every tick. That is the rig standing
        // in for an engine badly, not a defect in `integrate`.
        //
        // MEASURED, not asserted: with this line removed, 6 of the 33 cases go RED
        // (`impl/task15/red_run_2026-09-06T1930.txt`) -- `ContinuousAccelerates` reads
        // 34.13334 where it requires 68.26667, i.e. exactly one tick of acceleration instead
        // of two.
        //
        // ⛔ THE CASES THAT MEASURE THE DRIVING ARM OPT IN, and they already did:
        // `WritesTransformAndVelocityEachTick`, `CapturedPushOutAdoptedIntoPosition`,
        // `WallPushOutZeroesIntoWallComponent`, `ContactImpulseNeverEntersVelocity`,
        // `DeterministicReplay` and the control arm of `DrivesBodyFalseSkipsAdapterWrites` each
        // set `drivesBody = true` on their own instance and drive the body with
        // `engineStep()` / `engineStepAndCapture()`. Those are the cases where the rig HAS an
        // engine, and they are unaffected by this pin.
        sd.drivesBody = false;

        bindings.ownBodyId      = kBodyId;
        bindings.parentBodyId   = kBodyId;
        bindings.queryVolumeIds = { kVolumeId };
    }

    // ⛔ [movement-sim task 27] `knockbackSpeed` LEFT `movement::StaticData`: it is per attack
    // now, authored in `simulatableBrawler::StaticData::m_hitReactions`. Read from there, from
    // the SAME production aggregate `sd` is a copy of, so R-P1 still holds — the literal is
    // spelled once, in `SimulatableBrawlerTypes.h`.
    float knockbackSpeed() const
    { return game.m_hitReactions[dAttackDirection::kRightSequenceId].knockbackSpeed; }

    // The closed form ruling #13 authored: v^2 / (2a). 500 cm at the shipped pair.
    float knockbackDistance() const
    { return knockbackSpeed() * knockbackSpeed() / (2.f * sd.launchDecel); }

    // The slide's duration, and the FLOOR under a knockback's lockout: v / a. 0.5 s.
    float knockbackSlideSeconds() const { return knockbackSpeed() / sd.launchDecel; }

    // Enter `HitFlinch` with a resolved reaction, exactly as the machine's veto does from the
    // routing system's slice. `directionXY` is the world XY unit direction away from the attacker.
    void hit(HitReactionKind kind, glm::vec2 directionXY, float lockoutDuration)
    {
        const bool knockback = kind == HitReactionKind::Knockback;
        inbound.wasHitThisTick = true;
        inbound.reactionKind   = kind;
        inbound.knockbackSpeed = knockback ? knockbackSpeed() : 0.f;
        inbound.hitDirectionXY = knockback ? directionXY : glm::vec2(0.f);
        inbound.flinchDuration = knockback
            ? glm::max(lockoutDuration, knockbackSlideSeconds())
            : lockoutDuration;

        machineState.m_currentState      = DAttackState::HitFlinch;
        machineState.m_timeInCurrentState = 0.f;
        machineState.m_hitReaction       = kind;
        machineState.m_flinchDuration    = inbound.flinchDuration;
    }

    float probeLength() const { return sd.rideHeight + sd.snapDistance; }
    float accelPerTick() const { return sd.acceleration * kDt; }
    float brakePerTick() const { return sd.brakingDeceleration * kDt; }
    float gravityPerTick() const { return sd.gravity * kDt; }

    // ---- THE SPRING-DAMPER SERVO'S ARITHMETIC, NAMED [movement-sim task 56 / ruling #28] ----
    // ⛔ `maxSnapStep()` is GONE with `maxSnapSpeed`: the dead-beat clamped a VELOCITY assigned
    // from position error, and there is no assignment left to clamp. Everything below is derived
    // from the SHIPPED gains, never restated — R-P1 as it has always applied in this file.

    // What one cm of clearance error commands in one tick, from rest: `a == k*e` exactly,
    // because the feed-forward cancels gravity. 35.266667 cm/s at the shipped k.
    float servoStepPerCm() const { return sd.hoverStiffness * kDt; }

    // ⭐ THE HONEST PER-TICK VELOCITY BOUND, and it is NOT `hoverMaxAccel * kDt`. The clamp is
    // applied to the servo term, which CONTAINS the `- gravity` feed-forward, so the total
    // acceleration is `gravity + clamp(servo, ±hoverMaxAccel)` and its magnitude reaches
    // `hoverMaxAccel + |gravity|` on the downward saturation. 516.333333 cm/s, against the
    // 500.000000 the tidier wrong form would have given — a 3.3 % difference that a case
    // asserting the wrong one would have failed on, in the right direction, for the wrong reason.
    float maxVerticalStep() const
    { return (sd.hoverMaxAccel + glm::abs(sd.gravity)) * kDt; }

    // ⭐⭐ THE DISCRETE CRITICAL DAMPING RATIO, `1 - ω·dt/2` — the ζ at which this recurrence's
    // two eigenvalues coincide and its response stops alternating sign. It is NOT 1: ζ = 1 is the
    // CONTINUOUS-time answer and rings here. 0.616667 at the shipped ω = 46.
    float discreteCriticalZeta() const
    { return 1.f - sd.hoverFrequency * kDt * 0.5f; }

    // The repeated eigenvalue AT that ζ: `1 - ω·dt`. Monotone decay additionally needs it
    // POSITIVE, i.e. `ω < 1/dt` — critical damping buys the fastest decay, not a monotone one.
    float discreteCriticalEigenvalue() const
    { return 1.f - sd.hoverFrequency * kDt; }

    movement::SupportState support() const
    {
        return static_cast<movement::SupportState>(
            (state.flags & movement::kFlagSupportMask) >> movement::kFlagSupportShift);
    }
    bool frozenBit() const { return (state.flags & movement::kFlagFrozen) != 0u; }
    bool hasCommandBit() const { return (state.flags & movement::kFlagHasCommand) != 0u; }

    // One movement tick. `stickWorld`'s LENGTH is the stick deflection
    // (`BrawlerInputPackaging.h`: `moveDirectionWorld` is the move stick rotated into
    // camera space, so the rotation preserves its length).
    void tick(std::uint32_t t,
              glm::vec3 stickWorld = glm::vec3(0.f),
              std::uint8_t inputFlags = 0u)
    {
        SimulationComposite<movement::InitialConditions,
                            movement::State,
                            machine::State> composite(ic, state, machineState);
        auto deps = makeDependencies<movement::Dependencies>(composite);

        movement::PlayerInput pi{};
        pi.flags = inputFlags;

        machine::PlayerInput mi = machine::PlayerInput::zero();
        mi.moveDirectionWorld = stickWorld;

        movement::IntegrationUtils<brawlerTestMocks::MockPhysicsAdapter, QueryT>
            utils{ kDt, t, phys, query };
        movement::AllInput<brawlerTestMocks::MockPhysicsAdapter, QueryT>
            allInput{ pi, utils };

        movement::integrate(kDt, allInput, mi, sd, deps, bindings, derived, inbound);
        inbound = brawlerInboundHit::DerivedState{};

        // The composite holds COPIES (it was constructed by value), so sync back.
        ic    = composite.get<movement::InitialConditions>();
        state = composite.get<movement::State>();
    }

    // THE UNCONTESTED ENGINE TICK: integrate the command, capture the result. This one
    // line is what `captureBodyStatesAll` does on a tick where the solver had nothing to
    // separate.
    void engineStep() { state.bodyState.position += state.velocity * kDt; }

    // THE CONTESTED ENGINE TICK. `solverPosition` is whatever the solver actually left
    // the body at (a caller clamps it against a wall, or adds a push-out);
    // `solverVelocity` is the velocity the solver has for the body, which production
    // captures too and which ruling #14(c) says must never be read. Both land through the
    // REAL narrowing bridge, so `State::bodyState.linearVelocity` genuinely carries it.
    void engineStepAndCapture(glm::vec3 solverPosition, glm::vec3 solverVelocity)
    {
        PhysicsBodyState captured{};
        captured.position       = solverPosition;
        captured.linearVelocity = solverVelocity;
        phys.capturedState      = captured;
        state.bodyState         = captured;   // LinearBodyState::operator=(const PhysicsBodyState&)
    }
};

using ScriptedRig = Rig<brawlerTestMocks::MockSpatialQueryAdapter>;
using PlaneRig    = Rig<PlaneGroundQueryAdapter>;

// Seat a scripted rig on flat ground at EXACTLY ride height, which is the steady state:
// `velocityUp` is then identically 0 and the whole of `state.velocity` is the surface
// frame's tangential term. Every model / freeze / cadence case below wants that, because
// it makes the tangential assertion exact rather than "exact apart from the servo".
inline void seatOnFlatGroundAtRideHeight(ScriptedRig& rig)
{
    rig.query.scriptedSweeps = { floorHitAt(rig.sd.rideHeight, rig.probeLength()) };
    rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);
}

// The full-deflection stick, as a world XY direction.
inline glm::vec3 stick(float x, float y) { return glm::vec3(x, y, 0.f); }

// A byte-addressable buffer of the shape `writeToSyncedBuffer` asks for. Poisoned rather
// than zeroed, so a field the codec never writes reads back as poison instead of as a
// legitimate zero.
struct ProbeBuffer
{
    std::vector<std::uint8_t> bytes;

    explicit ProbeBuffer(std::size_t size = 256u) : bytes(size, 0xCDu) {}

    template <typename T>
    void writeToBuffer(std::uint32_t off, const T& value)
    { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

    template <typename T>
    T readFromBuffer(std::uint32_t off) const
    { T v; std::memcpy(&v, bytes.data() + off, sizeof(T)); return v; }
};

} // namespace movementTests

// ===========================================================================
// STEP 0 -- THE TELEPORT SEED
// ===========================================================================

TEST_CASE("BrawlerMovement.TeleportSeedWritesTransformAndZeroVelocity", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    // `drivesBody` is FALSE here -- pinned by `Rig()`, no longer the shipped default (task 15
    // flipped production to `true`). That is the point of this case: step 5 writes nothing, so
    // the teleport's two adapter calls are the ONLY body writes in the tick and their count is
    // an assertion in its own right. The REQUIRE_FALSE below now asserts the RIG's pin, which
    // is what a fixture assertion should mean.
    REQUIRE_FALSE(rig.sd.drivesBody);

    rig.state.velocity         = glm::vec3(5.f, 6.f, 7.f);
    rig.state.committedStepDir = glm::vec2(0.25f, -0.75f);
    rig.state.stepStartTick    = 3u;
    rig.state.flags            = static_cast<std::uint8_t>(movement::kFlagHasCommand);
    rig.state.positionCmd      = glm::vec3(-1.f, -2.f, -3.f);

    rig.ic.teleportPending = 1u;
    rig.ic.teleportPos     = glm::vec3(10.f, 20.f, 30.f);

    rig.tick(/*tick*/ 7u);

    // 1. THE SEED LANDED IN STATE.
    REQUIRE(rig.state.bodyState.position == glm::vec3(10.f, 20.f, 30.f));
    REQUIRE(rig.state.committedStepDir == glm::vec2(0.f));
    REQUIRE(rig.state.stepStartTick == 7u);
    REQUIRE_FALSE(rig.hasCommandBit());

    // 2. THE EDGE IS COUNTER-FREE: step 0 consumed the request in the same tick.
    REQUIRE(rig.ic.teleportPending == 0u);

    // 3. THE TWO BODY WRITES, and EXACTLY those two. The transform is the teleport pose
    //    and the velocity is zero -- a respawn must not carry momentum across.
    REQUIRE(rig.phys.setTransformCalls.size() == 1u);
    REQUIRE(rig.phys.setLinearVelocityCalls.size() == 1u);
    REQUIRE(rig.phys.setTransformCalls[0].bodyId == kBodyId);
    REQUIRE(glm::vec3(rig.phys.setTransformCalls[0].transform[3])
            == glm::vec3(10.f, 20.f, 30.f));
    REQUIRE(rig.phys.setLinearVelocityCalls[0].value == glm::vec3(0.f));

    // 4. AND NOTHING WAS PUSHED THROUGH THE FORCE SEAM. Revision 5 would have; ruling
    //    #14(c) re-places the body instead.
    REQUIRE(rig.phys.addAccelerationCalls.empty());
    REQUIRE(rig.phys.addVelocityChangeCalls.empty());
}

// ===========================================================================
// STEP 2 -- ATTACHMENT
// ===========================================================================

TEST_CASE("BrawlerMovement.AttachSupportedWhenProbeNormalWalkable", "[BrawlerMovement]")
{
    using namespace movementTests;

    // The pair straddles the authored cap, so the case measures the THRESHOLD rather than
    // just "flat ground is floor". `maxSlopeAngleDeg` is 45 in the shipped data; the
    // arithmetic below is written against `rig.sd`, not against the number.
    const float cap = ScriptedRig{}.sd.maxSlopeAngleDeg;

    ScriptedRig flat;
    seatOnFlatGroundAtRideHeight(flat);
    flat.tick(1u);
    REQUIRE(flat.support() == movement::SupportState::Supported);
    REQUIRE(flat.derived.lastSupportState == static_cast<std::uint8_t>(movement::SupportState::Supported));
    REQUIRE(flat.derived.surfaceNormal == glm::vec3(0.f, 0.f, 1.f));

    ScriptedRig justUnder;
    justUnder.query.scriptedSweeps = {
        floorHitAt(justUnder.sd.rideHeight, justUnder.probeLength(), slopeNormal(cap - 1.f)) };
    justUnder.tick(1u);
    INFO("slope " << (cap - 1.f) << " deg vs cap " << cap << " deg");
    REQUIRE(justUnder.support() == movement::SupportState::Supported);
    // ...and the frame followed that normal rather than staying world up.
    REQUIRE(justUnder.derived.surfaceNormal.x == Catch::Approx(slopeNormal(cap - 1.f).x).margin(1e-5f));
}

TEST_CASE("BrawlerMovement.AttachUnsupportedWhenProbeMisses", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    rig.query.scriptedSweeps.clear();          // an empty script is a MISS on every call
    rig.state.bodyState.position = glm::vec3(3.f, -4.f, 500.f);

    rig.tick(1u);

    REQUIRE(rig.support() == movement::SupportState::Unsupported);
    // `impactPoint` is meaningless on an unblocked sweep (SpatialQueryResult.h's field-
    // validity rule), so the viz readout must be zeroed rather than carry the garbage.
    REQUIRE(rig.derived.lastProbePoint == glm::vec3(0.f));
    // The airborne frame falls back to the servo axis.
    REQUIRE(rig.derived.surfaceNormal == glm::vec3(0.f, 0.f, 1.f));

    // ⭐ THE PROBE ITSELF, read off the RECORDED CALL. Exactly one sweep per tick, on the
    // movement volume, from the body's own pose, straight down `rideHeight + snapDistance`.
    // This is the assertion that the attachment probe is aimed where step 2 claims -- and
    // it is the only place the recorded `(transform, delta)` pair is the subject.
    REQUIRE(rig.query.sweepCalls.size() == 1u);
    REQUIRE(rig.query.sweepCalls[0].volumeId == kVolumeId);
    REQUIRE(glm::vec3(rig.query.sweepCalls[0].transform[3]) == glm::vec3(3.f, -4.f, 500.f));
    REQUIRE(rig.query.sweepCalls[0].delta == glm::vec3(0.f, 0.f, -rig.probeLength()));

    // AND THE FALL BEGAN: an unattached capsule is on the authored gravity law.
    REQUIRE(rig.state.velocity.z == Catch::Approx(rig.gravityPerTick()).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.AttachUnsupportedWhenSlopeTooSteep", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    const float cap = rig.sd.maxSlopeAngleDeg;

    // BLOCKED, at ride height, with a walkable-looking clearance -- so the ONLY thing that
    // can make this Airborne is the normal test. Paired with `AttachFloorWhenProbeNormalWalkable`'s
    // `cap - 1` arm, this is a one-degree straddle of the authored cap.
    rig.query.scriptedSweeps = {
        floorHitAt(rig.sd.rideHeight, rig.probeLength(), slopeNormal(cap + 1.f)) };
    rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);

    rig.tick(1u);

    INFO("slope " << (cap + 1.f) << " deg vs cap " << cap
         << " deg; dot(n, up)=" << slopeNormal(cap + 1.f).z
         << " cosMaxSlope=" << rig.sd.cosMaxSlope);
    REQUIRE(rig.support() == movement::SupportState::Unsupported);
    // A too-steep face is not a frame either: the tangent frame falls back to world up,
    // so a character cannot walk on a wall by accident (task 20 is what changes this).
    REQUIRE(rig.derived.surfaceNormal == glm::vec3(0.f, 0.f, 1.f));
    // And the vertical term is gravity, not the hover servo -- there is no ground to hold.
    REQUIRE(rig.state.velocity.z == Catch::Approx(rig.gravityPerTick()).margin(1e-4f));
}

// ===========================================================================
// THE SURFACE FRAME
// ===========================================================================

TEST_CASE("BrawlerMovement.TangentFrameOnFlatGroundIsWorldXY", "[BrawlerMovement]")
{
    using namespace movementTests;

    // At ride height the servo commands nothing, so the whole of `state.velocity` is
    // `u * velocityUV.x + v * velocityUV.y`. Pushing +X and then +Y therefore READS OFF
    // `u` and `v` directly.
    ScriptedRig pushX;
    seatOnFlatGroundAtRideHeight(pushX);
    pushX.tick(1u, stick(1.f, 0.f));
    REQUIRE(pushX.state.velocity.z == 0.f);
    REQUIRE(pushX.state.velocity.x == Catch::Approx(pushX.accelPerTick()).margin(1e-4f));
    REQUIRE(pushX.state.velocity.y == 0.f);

    ScriptedRig pushY;
    seatOnFlatGroundAtRideHeight(pushY);
    pushY.tick(1u, stick(0.f, 1.f));
    REQUIRE(pushY.state.velocity.z == 0.f);
    REQUIRE(pushY.state.velocity.x == 0.f);
    REQUIRE(pushY.state.velocity.y == Catch::Approx(pushY.accelPerTick()).margin(1e-4f));

    // ⇒ u == world +X and v == world +Y, which is what makes "same stick, flat vs slope"
    //   a meaningful comparison instead of an arbitrary rotation.
}

TEST_CASE("BrawlerMovement.TangentFrameFollowsSlopeNormal", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr float kSlopeDeg = 30.f;
    const glm::vec3 n = slopeNormal(kSlopeDeg);

    ScriptedRig rig;
    rig.query.scriptedSweeps = { floorHitAt(rig.sd.rideHeight, rig.probeLength(), n) };
    rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);

    rig.tick(1u, stick(1.f, 0.f));

    // `buildTangentFrame` projects world +X into the plane and renormalises, so for a
    // normal tilted `theta` about +Y the frame is u = (cos theta, 0, sin theta),
    // v = (0, 1, 0). Derived here from the basis construction, NOT read back from it.
    const glm::vec3 expectedU(glm::cos(glm::radians(kSlopeDeg)), 0.f,
                              glm::sin(glm::radians(kSlopeDeg)));

    REQUIRE(rig.support() == movement::SupportState::Supported);
    REQUIRE(rig.derived.surfaceNormal == n);

    // At ride height the servo is silent, so the velocity IS the tangential term:
    // one tick of acceleration along `u`.
    const glm::vec3 expected = expectedU * rig.accelPerTick();
    INFO("velocity=(" << rig.state.velocity.x << ", " << rig.state.velocity.y << ", "
         << rig.state.velocity.z << ")  expected=(" << expected.x << ", " << expected.y
         << ", " << expected.z << ")");
    REQUIRE(rig.state.velocity.x == Catch::Approx(expected.x).margin(1e-4f));
    REQUIRE(rig.state.velocity.y == Catch::Approx(expected.y).margin(1e-4f));
    REQUIRE(rig.state.velocity.z == Catch::Approx(expected.z).margin(1e-4f));

    // ⭐ AND IT IS PERPENDICULAR TO THE NORMAL -- the character walks ALONG the plane.
    REQUIRE(glm::dot(rig.state.velocity, n) == Catch::Approx(0.f).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.ComposeIsSurfaceRelative", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr float kSlopeDeg = 30.f;

    ScriptedRig flat;
    seatOnFlatGroundAtRideHeight(flat);
    flat.tick(1u, stick(1.f, 0.f));

    ScriptedRig slope;
    slope.query.scriptedSweeps = {
        floorHitAt(slope.sd.rideHeight, slope.probeLength(), slopeNormal(kSlopeDeg)) };
    slope.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + slope.sd.rideHeight);
    slope.tick(1u, stick(1.f, 0.f));

    INFO("flat  v=(" << flat.state.velocity.x << ", " << flat.state.velocity.y << ", "
         << flat.state.velocity.z << ")\n"
         "slope v=(" << slope.state.velocity.x << ", " << slope.state.velocity.y << ", "
         << slope.state.velocity.z << ")");

    // 1. THE SAME STICK PRODUCED DIFFERENT WORLD VELOCITIES -- the frame is in the loop.
    REQUIRE_FALSE(flat.state.velocity == slope.state.velocity);
    REQUIRE(slope.state.velocity.z > 0.f);         // walking UP the ramp lifts you
    REQUIRE(flat.state.velocity.z == 0.f);

    // 2. AND THEY DIFFER BY EXACTLY THE FRAME, not by a speed loss: the tangential SPEED
    //    is identical. The stick's deflection is preserved into the surface plane, so a
    //    character does not walk more slowly up a ramp for want of a cosine.
    REQUIRE(glm::length(slope.state.velocity)
            == Catch::Approx(glm::length(flat.state.velocity)).margin(1e-4f));
}

// ===========================================================================
// STEP 3 -- ContinuousAccelBrake
// ===========================================================================

TEST_CASE("BrawlerMovement.ContinuousAccelerates", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    REQUIRE(rig.sd.model == movement::MovementModel::ContinuousAccelBrake);

    const float a = rig.accelPerTick();

    rig.tick(1u, stick(1.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(a).margin(1e-4f));
    rig.tick(2u, stick(1.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(2.f * a).margin(1e-4f));

    // The third tick would overshoot (3a = 102.4 > 100), and `moveTowards` is EXACT at
    // the endpoint rather than asymptotic, so the speed lands on the cap and stops.
    rig.tick(3u, stick(1.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.maxWalkSpeed).margin(1e-4f));
    for (std::uint32_t t = 4u; t <= 10u; ++t)
        rig.tick(t, stick(1.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.maxWalkSpeed).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.ContinuousBrakes", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.state.velocity = glm::vec3(rig.sd.maxWalkSpeed, 0.f, 0.f);

    const float b = rig.brakePerTick();

    rig.tick(1u);   // NO STICK -> the brake arm
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.maxWalkSpeed - b).margin(1e-4f));
    rig.tick(2u);
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.maxWalkSpeed - 2.f * b).margin(1e-4f));

    // ⭐ AND IT REACHES EXACTLY ZERO. `moveTowards` returns the target when the remaining
    // distance is within one step, so a braked character STOPS rather than approaching
    // zero forever and leaving a denormal crawling across the wire every tick.
    rig.tick(3u);
    REQUIRE(rig.state.velocity.x == 0.f);
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
}

TEST_CASE("BrawlerMovement.StickMagnitudeScalesSpeed", "[BrawlerMovement]")
{
    using namespace movementTests;

    // The stick arrives as a world XY direction whose LENGTH is the deflection, so a
    // quarter-deflected stick asks for a quarter of the walk speed -- not for the walk
    // speed in a slightly different direction.
    const float deflections[] = { 0.25f, 0.5f, 1.f };
    for (const float d : deflections)
    {
        ScriptedRig rig;
        seatOnFlatGroundAtRideHeight(rig);
        for (std::uint32_t t = 1u; t <= 10u; ++t)
            rig.tick(t, stick(d, 0.f));

        INFO("deflection " << d << " -> speed " << glm::length(rig.state.velocity));
        REQUIRE(rig.state.velocity.x
                == Catch::Approx(d * rig.sd.maxWalkSpeed).margin(1e-4f));
        REQUIRE(rig.state.velocity.y == 0.f);
    }
}

// ===========================================================================
// STEP 1 -- THE FREEZE GATE
// ===========================================================================

TEST_CASE("BrawlerMovement.FrozenByHoldGuard", "[BrawlerMovement]")
{
    using namespace movementTests;

    // SUBJECT: the INPUT flags byte's bit 0, and that step 1 is its reader. (What the
    // frozen VALUE is, and that it does not decay, is `FrozenIsExactThisTick` below.)
    ScriptedRig held;
    seatOnFlatGroundAtRideHeight(held);
    held.state.velocity = glm::vec3(held.sd.maxWalkSpeed, 0.f, 0.f);
    held.tick(1u, stick(1.f, 0.f), movement::kInputFlagHoldGuard);

    REQUIRE(held.frozenBit());
    REQUIRE(held.state.velocity == glm::vec3(0.f));

    // CONTROL 1 -- the same tick with the bit clear moves at full speed. Without this the
    // case would pass on a rig that could not move at all.
    ScriptedRig free;
    seatOnFlatGroundAtRideHeight(free);
    free.state.velocity = glm::vec3(free.sd.maxWalkSpeed, 0.f, 0.f);
    free.tick(1u, stick(1.f, 0.f), 0u);
    REQUIRE_FALSE(free.frozenBit());
    REQUIRE(free.state.velocity.x == Catch::Approx(free.sd.maxWalkSpeed).margin(1e-4f));

    // ⭐ CONTROL 2 -- A RESERVED BIT IS INERT. Bits 1-7 are spoken for by wall-grab (20),
    // jump (21), dash (31) and ski-tuck (48); this pins that `frozen` reads bit 0 and not
    // "any bit set", so the first of those four to land cannot silently freeze locomotion.
    ScriptedRig reserved;
    seatOnFlatGroundAtRideHeight(reserved);
    reserved.state.velocity = glm::vec3(reserved.sd.maxWalkSpeed, 0.f, 0.f);
    reserved.tick(1u, stick(1.f, 0.f), static_cast<std::uint8_t>(1u << 1));
    REQUIRE_FALSE(reserved.frozenBit());
    REQUIRE(reserved.state.velocity.x == Catch::Approx(reserved.sd.maxWalkSpeed).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.FrozenByHitFlinch", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.state.velocity = glm::vec3(rig.sd.maxWalkSpeed, 0.f, 0.f);
    rig.machineState.m_currentState = DAttackState::HitFlinch;

    rig.tick(1u, stick(1.f, 0.f));
    REQUIRE(rig.frozenBit());
    REQUIRE(rig.state.velocity == glm::vec3(0.f));

    // CONTROL -- being mid-ATTACK is not a flinch, and it must not FREEZE locomotion.
    // ⭐ [movement-sim task 84, 2026-09-20] THIS ARM USED TO READ `maxWalkSpeed`, i.e. the walk
    // law, and it cannot any more: `Attacking` now runs step 3's THIRD branch, `locked`, which
    // ramps the tangential channels linearly to a stop on the attack's last tick. The arm keeps
    // its subject -- a flinch freezes, an attack does not -- and states it against the law that
    // actually runs: with nine Attacking ticks left the first one keeps 8/9 of the walk speed,
    // which is a number neither the freeze nor a full stop could produce.
    ScriptedRig attacking;
    seatOnFlatGroundAtRideHeight(attacking);
    attacking.state.velocity = glm::vec3(attacking.sd.maxWalkSpeed, 0.f, 0.f);
    attacking.machineState.m_currentState  = DAttackState::Attacking;
    attacking.machineState.m_attackEndTick = 10u;      // R = 9 on tick 1
    attacking.tick(1u, stick(1.f, 0.f));
    REQUIRE_FALSE(attacking.frozenBit());
    REQUIRE(attacking.state.velocity.x
            == Catch::Approx(attacking.sd.maxWalkSpeed * 8.f / 9.f).margin(1e-3f));

    // ⚠ AND THE OTHER HALF, RECORDED RATHER THAN LEFT AS A SURPRISE: a DEFAULT-CONSTRUCTED
    // machine `State` held in `Attacking` carries `m_attackEndTick == 0`, which ⛔G-24's guard
    // saturates to `R = 1` and therefore to a full stop on the first tick. That is the designed
    // degrade for an end tick at or before the current one -- it is NOT a freeze, the frozen bit
    // stays clear -- and it is the reason the arm above has to set the field at all.
    ScriptedRig noEndTick;
    seatOnFlatGroundAtRideHeight(noEndTick);
    noEndTick.state.velocity = glm::vec3(noEndTick.sd.maxWalkSpeed, 0.f, 0.f);
    noEndTick.machineState.m_currentState = DAttackState::Attacking;
    REQUIRE(noEndTick.machineState.m_attackEndTick == 0u);
    noEndTick.tick(1u, stick(1.f, 0.f));
    REQUIRE_FALSE(noEndTick.frozenBit());
    REQUIRE(noEndTick.state.velocity == glm::vec3(0.f));
}

TEST_CASE("BrawlerMovement.FrozenByGuardFlinch", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.state.velocity = glm::vec3(rig.sd.maxWalkSpeed, 0.f, 0.f);
    rig.machineState.m_currentState = DAttackState::GuardFlinch;

    rig.tick(1u, stick(1.f, 0.f));
    REQUIRE(rig.frozenBit());
    REQUIRE(rig.state.velocity == glm::vec3(0.f));

    // CONTROL -- Idle is the default and must not freeze.
    ScriptedRig idle;
    seatOnFlatGroundAtRideHeight(idle);
    idle.state.velocity = glm::vec3(idle.sd.maxWalkSpeed, 0.f, 0.f);
    REQUIRE(idle.machineState.m_currentState == DAttackState::Idle);
    idle.tick(1u, stick(1.f, 0.f));
    REQUIRE_FALSE(idle.frozenBit());
    REQUIRE(idle.state.velocity.x == Catch::Approx(idle.sd.maxWalkSpeed).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.FrozenIsExactThisTick", "[BrawlerMovement]")
{
    using namespace movementTests;

    // SUBJECT: the genre's hitstop. A freeze zeroes the model's contribution FOR THAT
    // TICK. It is not a decay, and it does not restore the pre-freeze velocity afterwards.
    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.state.velocity = glm::vec3(rig.sd.maxWalkSpeed, 0.f, 0.f);

    rig.tick(1u, stick(1.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.maxWalkSpeed).margin(1e-4f));

    rig.tick(2u, stick(1.f, 0.f), movement::kInputFlagHoldGuard);
    // ⭐ EXACT, not `maxWalkSpeed - brakingDeceleration * dt` and not anything smaller.
    REQUIRE(rig.state.velocity == glm::vec3(0.f));

    rig.tick(3u, stick(1.f, 0.f));
    // ⭐ AND IT RESUMES FROM ZERO. If the freeze had merely been masked on the way out,
    // the model would re-read `maxWalkSpeed` here and this would be 100, not one tick of
    // acceleration.
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.accelPerTick()).margin(1e-4f));
    REQUIRE_FALSE(rig.frozenBit());
}

// ===========================================================================
// STEP 4 -- THE HOVER SERVO
// ===========================================================================

// ---------------------------------------------------------------------------
// ⭐⭐ THE GRAVITY FEED-FORWARD'S WHOLE POINT [movement-sim task 56 / ruling #28].
//
// Gravity is now added on EVERY tick, supported or not. So "holds ride height" is no longer the
// trivial consequence of a branch that switched gravity off -- it is an arithmetic identity that
// one term buys:
//     servo(e = 0, vUp = 0) = k*0 - c*0 - gravity = -gravity   =>   a = gravity + (-gravity) = 0
// WITHOUT the `- sd.gravity` feed-forward the spring would have to SAG until it held its own
// weight, `k*sag == |gravity|` => 980 / 2116 = 0.463138 cm. That is not a rounding error: it is
// 463x the scale every `clearance == rideHeight` pin in this file asserts to, and all of them
// would have had to be loosened to accept it.
//
// POISONED, NOT ASSUMED (impl notes §2): with `- sd.gravity` deleted from the servo term this
// case reads velocity.z == -16.333334 on the first tick instead of 0, and `HoverOnRampDoesNotDrift`
// settles 0.4631 cm low. Both arms were run.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.HoverHoldsRideHeight", "[BrawlerMovement]")
{
    using namespace movementTests;

    // 1. AT REST AT RIDE HEIGHT THE LAW COMMANDS EXACTLY ZERO. Not "small": the two terms are
    //    the same float with opposite signs, so they cancel bit-for-bit.
    ScriptedRig rest;
    seatOnFlatGroundAtRideHeight(rest);
    rest.tick(1u);
    INFO("at rest: clearance == rideHeight == " << rest.sd.rideHeight
         << "  gravity=" << rest.sd.gravity << "  k=" << rest.sd.hoverStiffness);
    REQUIRE(rest.support() == movement::SupportState::Supported);
    REQUIRE(rest.state.velocity.z == 0.f);
    REQUIRE(rest.state.velocity == glm::vec3(0.f));

    // 2. ⭐ A FALL ALREADY IN PROGRESS IS DAMPED, NOT ANNIHILATED -- and this row is the whole
    //    difference between ruling #28's law and the dead-beat it replaced. The dead-beat
    //    ASSIGNED `velocityUp` from clearance, so this seed read EXACTLY 0.000000 after one tick:
    //    600 cm/s of real momentum destroyed by a position servo. Here the only term acting is
    //    the damper (e == 0, and the feed-forward cancels gravity), so the closed form is
    //    `vUp * (1 - c*dt)` and nothing is discarded.
    ScriptedRig falling;
    seatOnFlatGroundAtRideHeight(falling);
    falling.state.velocity = glm::vec3(0.f, 0.f, -500.f);
    falling.tick(1u);

    const float damped = -500.f * (1.f - falling.sd.hoverDamping * kDt);
    INFO("fall in progress: -500 -> " << falling.state.velocity.z
         << "  (closed form vUp*(1 - c*dt) = " << damped
         << ", dead-beat would have assigned 0.000000)");
    REQUIRE(falling.state.velocity.z == Catch::Approx(damped).margin(1e-3f));
    REQUIRE(falling.state.velocity.z != 0.f);           // RED under the dead-beat
    REQUIRE(falling.state.velocity.z < 0.f);            // still falling, just slower
    REQUIRE(glm::abs(falling.state.velocity.z) < 500.f);

    // 3. ...AND IT CONVERGES TO THE SAME EXACT ZERO. The damping is real, not a stall.
    for (std::uint32_t t = 2u; t <= 9u; ++t)
        falling.tick(t);
    INFO("after 8 more ticks vz=" << falling.state.velocity.z);
    REQUIRE(glm::abs(falling.state.velocity.z) < 1e-3f);

    // 4. CONTROL -- the same seed with nothing under the capsule falls at the authored rate.
    //    That is what makes arm 2 a measurement: the two arms differ only in whether the probe
    //    found ground, and the SERVO TERM is the only thing that differs in the law.
    ScriptedRig unsupported;
    unsupported.query.scriptedSweeps.clear();
    unsupported.state.velocity = glm::vec3(0.f, 0.f, -500.f);
    unsupported.tick(1u);
    REQUIRE(unsupported.support() == movement::SupportState::Unsupported);
    REQUIRE(unsupported.state.velocity.z
            == Catch::Approx(-500.f + unsupported.gravityPerTick()).margin(1e-3f));
}

TEST_CASE("BrawlerMovement.HoverLiftsWhenLow", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    // 1 cm LOW, FROM REST. The whole law collapses to `a = k*e` on this tick: the feed-forward
    // cancels gravity exactly and the damper term is zero because `vUp` is. So the commanded
    // velocity is `k*e*dt` -- one multiplication, and `servoStepPerCm()` is that per cm.
    // ⚠ THE DEAD-BEAT WOULD HAVE COMMANDED `e / dt` = 60 cm/s here. 35.266667 vs 60 is the
    // visible content of ruling #28 at small errors: the spring does not close the gap in one
    // tick and is not trying to.
    rig.query.scriptedSweeps = { floorHitAt(rig.sd.rideHeight - 1.f, rig.probeLength()) };
    rig.tick(1u);

    const float expected = rig.servoStepPerCm();          // k * 1 cm * dt
    INFO("clearance error 1 cm -> commanded " << rig.state.velocity.z
         << " cm/s (k*e*dt = " << expected << ", retired dead-beat e/dt = " << (1.f / kDt)
         << ", one tick of gravity = " << rig.gravityPerTick() << ")");
    REQUIRE(rig.state.velocity.z > 0.f);
    REQUIRE(rig.state.velocity.z == Catch::Approx(expected).margin(1e-3f));
    REQUIRE(rig.state.velocity.z < rig.maxVerticalStep());   // this arm is UNSATURATED
    // ...and it is NOT the retired law's answer, which is what makes the value assertion above
    // a measurement of the new one rather than a transcription.
    REQUIRE(rig.state.velocity.z != Catch::Approx(1.f / kDt).margin(1e-3f));
}

// ⭐⭐ RENAMED AND INVERTED BY movement-sim TASK 57 / RULING #29. It was
// `HoverLowersWhenHigh`, and it asserted the OPPOSITE: that the spring pulls the character down
// at `k*e` when it is above ride height. That arm is gone. A case keeping the old name would have
// been a sentence the code contradicts, which is why this is a rename and not an edit.
// The one-sided law is the whole of the change: SUPPORTED MEANS HELD UP, NEVER PULLED DOWN.
TEST_CASE("BrawlerMovement.HoverDoesNotPullDownWhenHigh", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    rig.query.scriptedSweeps = { floorHitAt(rig.sd.rideHeight + 1.f, rig.probeLength()) };
    rig.tick(1u);

    // ONE cm HIGH. The task-56 spring commanded `k*e*dt` = -35.267 cm/s here; ruling #29 commands
    // ONE TICK OF GRAVITY and nothing else, because `hoverPullDownAccel` ships at 0.
    REQUIRE(rig.sd.hoverPullDownAccel == 0.f);
    REQUIRE(rig.support() == movement::SupportState::Supported);   // the servo IS on this tick...
    REQUIRE(rig.state.velocity.z < 0.f);
    REQUIRE(rig.state.velocity.z == Catch::Approx(rig.gravityPerTick()).margin(1e-3f));
    // ...and it is NOT the retired two-sided answer, which is what makes the row above a
    // measurement of the new arm rather than a coincidence of two small numbers.
    REQUIRE(rig.state.velocity.z != Catch::Approx(-rig.servoStepPerCm()).margin(1e-3f));

    // AND THE OTHER ARM IS UNTOUCHED: one cm LOW is still task 56's `k*e*dt`, to the digit. F1
    // changes exactly one side of `e == 0`, and this pair is the statement of that.
    ScriptedRig low;
    low.query.scriptedSweeps = { floorHitAt(low.sd.rideHeight - 1.f, low.probeLength()) };
    low.tick(1u);
    REQUIRE(low.state.velocity.z == Catch::Approx(low.servoStepPerCm()).margin(1e-3f));

    // THE KNOB RESTORES A BOUNDED PULL, and only a bounded one: at 980 the same one-cm-high tick
    // asks for `k*1` = 2116 and gets 980 -- `glm::max(k*e, -hoverPullDownAccel)` is the ceiling.
    ScriptedRig knob;
    knob.sd.hoverPullDownAccel = 980.f;
    knob.query.scriptedSweeps = { floorHitAt(knob.sd.rideHeight + 1.f, knob.probeLength()) };
    knob.tick(1u);
    INFO("1 cm high: default " << rig.state.velocity.z << ", knob 980 -> "
         << knob.state.velocity.z << ", task 56 would have said " << -rig.servoStepPerCm());
    REQUIRE(knob.sd.hoverStiffness * 1.f > knob.sd.hoverPullDownAccel);          // the ceiling binds
    REQUIRE(knob.state.velocity.z
            == Catch::Approx((knob.sd.gravity - knob.sd.hoverPullDownAccel) * kDt).margin(1e-3f));
}

TEST_CASE("BrawlerMovement.HoverServoAccelerationIsClamped", "[BrawlerMovement]")
{
    using namespace movementTests;

    // ⭐ WHAT IS CLAMPED CHANGED CATEGORY [movement-sim task 56 / ruling #28]. `maxSnapSpeed` was
    // a VELOCITY ceiling on a law that ASSIGNED velocity from position error; `hoverMaxAccel` is
    // an ACCELERATION ceiling on a law that integrates. The safety net underneath is unchanged:
    // a fall that outruns the servo bottoms out into contact and is caught by depenetration plus
    // step 6'.
    //
    // ⚠ AND THE PER-TICK VELOCITY BOUND IS ASYMMETRIC BY EXACTLY ONE `gravity`, which is the
    // whole reason `maxVerticalStep()` exists and `hoverMaxAccel * dt` is NOT used below. The
    // clamp is applied to the servo term, and the servo term CONTAINS the `- gravity`
    // feed-forward; the total is `gravity + clamp(servo, +/-hoverMaxAccel)`. Downward saturation
    // therefore reaches `-(hoverMaxAccel + |gravity|)*dt` = -516.333333, not -500.000000.

    ScriptedRig probe;
    const float bound = probe.maxVerticalStep();
    INFO("hoverMaxAccel=" << probe.sd.hoverMaxAccel << " gravity=" << probe.sd.gravity
         << " => |dv| bound per tick = " << bound
         << " (the tidier wrong form hoverMaxAccel*dt = " << (probe.sd.hoverMaxAccel * kDt) << ")");

    // ⭐⭐ ARMS 1 AND 2 RE-POINTED BY movement-sim TASK 57 / RULING #29. They used to reach the
    //    DOWNWARD clamp through the two-sided spring at clearance 24 and 45; that arm no longer
    //    exists, and at `hoverPullDownAccel == 0` those two clearances now command exactly one
    //    tick of gravity (`AboveRideHeightIsGravityOnly` owns that statement). The CLAMP itself is
    //    unchanged and still needs pinning, so both arms now drive it through the KNOB -- which
    //    also proves the knob feeds the same `hoverMaxAccel` clamp rather than a second one.

    // 1. JUST INSIDE THE CLAMP, DOWNWARD. At clearance 24 the one-sided spring asks for
    //    k*(10-24) = -29624; with the knob above that magnitude the pull is the SPRING's and is
    //    inside +/-30000, so the law runs UNSATURATED and the commanded velocity is the full
    //    `(gravity + servo)*dt`. ⚠ There is no `- gravity` feed-forward on this arm: it is the
    //    term F1 removed above ride height, and its absence is why the number differs from the
    //    task-56 one by exactly 980*dt.
    ScriptedRig inside;
    inside.sd.hoverPullDownAccel = 60000.f;
    inside.query.scriptedSweeps = { floorHitAt(24.f, inside.probeLength()) };
    inside.tick(1u);
    const float insideServo = glm::max(inside.sd.hoverStiffness * (inside.sd.rideHeight - 24.f),
                                       -inside.sd.hoverPullDownAccel);
    const float insideExpected = (inside.sd.gravity + insideServo) * kDt;
    INFO("clearance 24, knob 60000: servo=" << insideServo << " (|.| < "
         << inside.sd.hoverMaxAccel << ") -> commanded " << inside.state.velocity.z
         << " expected " << insideExpected);
    REQUIRE(glm::abs(insideServo) < inside.sd.hoverMaxAccel);        // the premise
    REQUIRE(insideServo == Catch::Approx(inside.sd.hoverStiffness * -14.f).margin(1e-2f));
    REQUIRE(inside.state.velocity.z == Catch::Approx(insideExpected).margin(1e-3f));
    REQUIRE(glm::abs(inside.state.velocity.z) < bound);              // this arm is UNSATURATED

    // 2. BEYOND IT, DOWNWARD. At clearance 45 -- deep in the snap band, a real step-down -- the
    //    spring asks for -74060 and the clamp holds the total to `gravity - hoverMaxAccel`. The
    //    commanded velocity lands exactly on the asymmetric bound, which is what pins the
    //    `+ |gravity|` half of it.
    ScriptedRig high;
    high.sd.hoverPullDownAccel = 60000.f;
    high.query.scriptedSweeps = { floorHitAt(45.f, high.probeLength()) };
    high.tick(1u);
    const float highServo = glm::max(high.sd.hoverStiffness * (high.sd.rideHeight - 45.f),
                                     -high.sd.hoverPullDownAccel);
    INFO("clearance 45, knob 60000: servo=" << highServo << " -> clamped -> commanded "
         << high.state.velocity.z);
    REQUIRE(glm::abs(highServo) > high.sd.hoverMaxAccel);            // the premise: it WOULD overshoot
    REQUIRE(high.state.velocity.z == Catch::Approx(-bound).margin(1e-3f));
    REQUIRE(high.support() == movement::SupportState::Supported);
    // ...and NOT the tidier wrong bound. This row is what a case written against
    // `hoverMaxAccel * kDt` would have failed on.
    REQUIRE(high.state.velocity.z != Catch::Approx(-high.sd.hoverMaxAccel * kDt).margin(1e-3f));
    // ...and at the SHIPPED knob the same clearance is one tick of gravity, which is the
    // difference F1 made and the reason both arms above had to be re-pointed.
    ScriptedRig shippedKnob;
    shippedKnob.query.scriptedSweeps = { floorHitAt(45.f, shippedKnob.probeLength()) };
    shippedKnob.tick(1u);
    REQUIRE(shippedKnob.sd.hoverPullDownAccel == 0.f);
    REQUIRE(shippedKnob.state.velocity.z
            == Catch::Approx(shippedKnob.gravityPerTick()).margin(1e-3f));

    // 3. BEYOND IT, UPWARD, AND IT IS THE *DAMPER* THAT SATURATES -- not the spring. At ride
    //    height the spring term is identically zero, so a fast fall is arrested purely by
    //    `-c*vUp`: at -600 cm/s that asks for +35204, and the clamp holds the total to
    //    `gravity + hoverMaxAccel`. There is no clearance error large enough to saturate upward
    //    on its own (the largest possible is `rideHeight` = 10 cm, worth 22140), which is why
    //    this arm is driven by velocity rather than by position.
    ScriptedRig fast;
    seatOnFlatGroundAtRideHeight(fast);
    fast.state.velocity = glm::vec3(0.f, 0.f, -600.f);
    fast.tick(1u);
    const float fastServo = -fast.sd.hoverDamping * -600.f - fast.sd.gravity;
    INFO("ride height, falling at -600: servo=" << fastServo << " -> clamped -> commanded "
         << fast.state.velocity.z);
    REQUIRE(fastServo > fast.sd.hoverMaxAccel);                      // the premise
    REQUIRE(fast.state.velocity.z
            == Catch::Approx(-600.f + (fast.sd.gravity + fast.sd.hoverMaxAccel) * kDt).margin(1e-3f));
    // The upward saturation is the OTHER side of the asymmetry: `hoverMaxAccel - |gravity|`.
    REQUIRE(fast.state.velocity.z - (-600.f)
            == Catch::Approx((fast.sd.hoverMaxAccel - glm::abs(fast.sd.gravity)) * kDt).margin(1e-3f));
    // ...and the largest clearance error the geometry admits cannot reach the clamp alone.
    REQUIRE(fast.sd.hoverStiffness * fast.sd.rideHeight - fast.sd.gravity
            < fast.sd.hoverMaxAccel);
}

TEST_CASE("BrawlerMovement.HoverAbsorbsSmallLedge", "[BrawlerMovement]")
{
    using namespace movementTests;

    // A 9 cm step UP. The ANALYTIC GROUND is the mock here, not a script: the whole claim
    // is that the servo CONVERGES, and convergence needs a ground that responds to the
    // capsule moving. A scripted clearance would be answering its own question.
    //
    // ⭐ RE-PINNED FROM TICK COUNTS TO A TRANSIENT [movement-sim task 56 / ruling #28]. The
    // dead-beat absorbed this ledge in exactly two ticks: one CLAMPED correction at
    // `maxSnapSpeed` and one unclamped remainder. Those two rows were a property of the clamp, not
    // of the hover, and they are gone with it. What is asserted now is the shape of the response,
    // which is the thing the gains are tuned for:
    //   * the first tick is `k*e*dt` and is NOT saturated -- 9 cm asks for 20024 cm/s^2 against a
    //     30000 ceiling, so a 9 cm ledge is inside the servo's linear range;
    //   * the approach is MONOTONE and never overshoots past ride height, which is the discrete
    //     critical damping ratio doing its job (`discreteCriticalZeta()`; at zeta = 1 this case
    //     rings past ride height and back);
    //   * it is within 0.05 cm after six ticks.
    PlaneRig rig;
    rig.state.bodyState.position =
        glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight));

    rig.tick(0u);
    REQUIRE(rig.query.lastClearance == Catch::Approx(rig.sd.rideHeight).margin(1e-3f));
    REQUIRE(rig.state.velocity.z == Catch::Approx(0.f).margin(1e-3f));
    rig.engineStep();

    // THE LEDGE. Clearance drops 10 -> 1.
    constexpr float kLedge = 9.f;
    rig.query.planeOffset += kLedge;

    std::vector<float> clearance;
    std::vector<float> commanded;
    for (std::uint32_t t = 1u; t <= 7u; ++t)
    {
        rig.tick(t);
        clearance.push_back(rig.query.lastClearance);
        commanded.push_back(rig.state.velocity.z);
        REQUIRE(rig.support() == movement::SupportState::Supported);   // never flinched
        rig.engineStep();
    }

    INFO("9 cm ledge: clearance=" << clearance[0] << ", " << clearance[1] << ", " << clearance[2]
         << ", " << clearance[3] << " ... " << clearance.back()
         << "  commanded=" << commanded[0] << ", " << commanded[1] << ", " << commanded[2]
         << " ... " << commanded.back()
         << "  (k*e*dt for e=9 is " << (9.f * rig.servoStepPerCm())
         << "; the retired dead-beat clamped this same ledge to maxSnapSpeed = 400)");

    // 1. THE FIRST TICK IS `k*e*dt`, EXACTLY, AND UNSATURATED.
    REQUIRE(clearance[0] == Catch::Approx(rig.sd.rideHeight - kLedge).margin(1e-3f));
    REQUIRE(commanded[0] == Catch::Approx(kLedge * rig.servoStepPerCm()).margin(1e-2f));
    REQUIRE(commanded[0] < rig.maxVerticalStep());
    REQUIRE(rig.sd.hoverStiffness * kLedge - rig.sd.gravity < rig.sd.hoverMaxAccel);  // the premise

    // 2. MONOTONE APPROACH FROM BELOW, AND NO OVERSHOOT PAST RIDE HEIGHT. This is the row that
    //    discriminates the shipped zeta from the continuous-time zeta = 1: at zeta = 1 clearance
    //    crosses ride height and comes back.
    for (std::size_t i = 1; i < clearance.size(); ++i)
    {
        INFO("tick " << (i + 1) << " clearance=" << clearance[i]
             << " previous=" << clearance[i - 1]);
        REQUIRE(clearance[i] > clearance[i - 1]);                       // strictly rising
        REQUIRE(clearance[i] <= rig.sd.rideHeight + 1e-3f);             // never past it
        REQUIRE(commanded[i] < commanded[i - 1]);                       // strictly decelerating
        REQUIRE(commanded[i] > 0.f);                                    // ...and never reverses
    }

    // 3. AND IT IS BACK AT RIDE HEIGHT. Six ticks for a 9 cm ledge, to 0.05 cm.
    REQUIRE(clearance.back() == Catch::Approx(rig.sd.rideHeight).margin(0.05f));
    REQUIRE(rig.sd.hoverDampingRatio >= rig.discreteCriticalZeta());    // why (2) holds
}

// ---------------------------------------------------------------------------
// ⭐⭐ THE RAMP. USER RULING #26 (a) / TASK 49.
//
// THE TARGET IS MEASURED, NOT INVENTED. Task 49 drove the header through
// `SimulatableBrawler::integrate` against this same analytic 30 deg plane, seeded 1 cm
// above ride height with a zero stick, and recorded the lateral travel over the whole
// transient on BOTH arms:
//     post-fix (correct along world up, shipped)   0.000000 cm
//     pre-fix  (correct along the surface normal)  0.433010 cm per cm of clearance error
// The pre-fix figure is `sin(theta) * cos(theta) * e` -- the probe is cast FROM the capsule,
// so a step along `n` also moves the probe origin sideways and the ground under it falls
// away; the servo gained `s / cos(theta)` of vertical instead of `s` and over-corrected,
// with error ratio `1 - 1/cos(theta)` = -0.1547 at 30 deg. Both arms are demonstrated, so
// THE CASE WAS KNOWN TO DISCRIMINATE BEFORE IT WAS WRITTEN. The margin below is anchored to
// the measured pre-fix number rather than chosen.
//
// ⭐⭐ WHAT TASK 56 / RULING #28 MOVED HERE, AND WHAT IT DID NOT. The AXIS property is
// untouched and is still the subject: the servo term is built from a SCALAR clearance error and
// composed onto `up` in step 5, so its lateral component is identically zero at every angle, and
// `driftX == 0.000000` is asserted exactly as before.
// ⛔ WHAT IS RETIRED IS CLOSURE. Task 49 measured this rig on the DEAD-BEAT, whose loop gain
// was 1 by construction, and this case pinned `clearance[1] == rideHeight` -- the whole error
// closed in one tick. Under ruling #28 closure is no longer 1 BY DESIGN: the spring-damper
// approaches over a transient. That row is replaced below by the transient's own closed form,
// `y1 = y0 * (1 - (omega*dt)^2)`, which is a stronger statement than the constant it replaces
// (it names the gain, not just the endpoint) and which the measured 0.412222 confirms.
// ⚠ The task-49 ratios above are a property of the AXES, not of the law, and are unchanged.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.HoverOnRampDoesNotDrift", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr float kSlopeDeg      = 30.f;
    // ⭐ SEEDED ONE cm *BELOW* RIDE HEIGHT SINCE movement-sim TASK 57 / RULING #29. It used to be
    // one cm ABOVE, and that is now the arm where the law is gravity and nothing else -- a seed
    // there would have measured a falling body, not the servo transient this case is about. The
    // AXIS property (no lateral component at any slope) is unchanged and is still the subject;
    // only the side of `e == 0` the transient is driven from moved, and the closed form below is
    // the same one with the sign flipped.
    constexpr float kSeedError     = 1.f;    // cm BELOW ride height
    constexpr int   kTicks         = 8;
    // Task 49 AC-1, MEASURED on a two-arm rig. Do not re-derive it here.
    constexpr float kPreFixLateralPerCmOfError = 0.433010f;

    PlaneRig rig;
    rig.query.planeNormal = slopeNormal(kSlopeDeg);
    rig.state.bodyState.position = glm::vec3(
        0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight - kSeedError));

    const float startX = rig.state.bodyState.position.x;

    std::vector<float>     clearance;
    std::vector<glm::vec3> commanded;
    for (int t = 0; t < kTicks; ++t)
    {
        rig.tick(static_cast<std::uint32_t>(t));
        clearance.push_back(rig.query.lastClearance);
        commanded.push_back(rig.state.velocity);
        rig.engineStep();
    }

    const float driftX = rig.state.bodyState.position.x - startX;

    INFO("30 deg ramp, uv = 0, seeded " << kSeedError << " cm high.\n"
         "  clearance[0]=" << clearance[0] << "  clearance[1]=" << clearance[1]
         << "  clearance[last]=" << clearance.back() << "\n"
         "  |commanded[0]|=" << glm::length(commanded[0])
         << "  commanded[0]=(" << commanded[0].x << ", " << commanded[0].y << ", "
         << commanded[0].z << ")\n"
         "  lateral drift=" << driftX << " cm   (measured pre-fix: "
         << kPreFixLateralPerCmOfError * kSeedError << " cm)");

    // ---- THE CONTROL ROWS. Without these a servo that did NOTHING would also not drift,
    //      and the case would be the strongest kind of vacuous: green because nothing ran.
    //      Both are deliberately COMPOSITION-INDEPENDENT -- a starting clearance and a
    //      MAGNITUDE -- so they stay GREEN under the pre-task-49 composition as well. What
    //      they fence is the RIG, and a control row that moved with the subject would fence
    //      nothing. (Measured on the two-arm control: both PASS in both arms.)
    REQUIRE(rig.support() == movement::SupportState::Supported);
    REQUIRE(clearance[0] == Catch::Approx(rig.sd.rideHeight - kSeedError).margin(1e-3f));
    // The servo ACTUATED. ⚠ The VALUE moved with ruling #28 (`k*e*dt`, not the dead-beat's
    // `e/dt`), but the row keeps the property that made it a control: it is a MAGNITUDE and a
    // starting clearance, so it reads the same under the pre-task-49 composition as under this
    // one, and it fences the rig rather than the subject.
    REQUIRE(glm::length(commanded[0])
            == Catch::Approx(kSeedError * rig.servoStepPerCm()).margin(1e-2f));

    // ---- THE ASSERTIONS. Measured and actuated on the SAME axis, so the correction has no
    //      horizontal component at any slope and there is nothing to drift.
    //
    // 1. THE COMMAND IS PURELY VERTICAL. Under the pre-fix composition this term is
    //    `n * velocityUp`, whose X is `-sin(theta) * velocityUp` = +17.6 cm/s here.
    REQUIRE(commanded[0].x == 0.f);
    REQUIRE(commanded[0].z == Catch::Approx(kSeedError * rig.servoStepPerCm()).margin(1e-2f));

    // 2. AND THE TRANSIENT IS THE SPRING'S, NOT A ONE-TICK CLOSURE. From rest the first tick is
    //    `v1 = -k*y0*dt` and position integrates with the NEW velocity (semi-implicit Euler), so
    //        y1 = y0 + v1*dt = y0 * (1 - (omega*dt)^2)
    //    = 0.412222 cm of the 1 cm seed left. ⚠ The retired dead-beat row asserted
    //    `clearance[1] == rideHeight` here; that was the CLAMP-FREE gain-1 law and is gone.
    //    Pre-task-49 the gain was 1/cos(theta), so this row still moves with the axis too.
    const float W = rig.sd.hoverFrequency * kDt;
    REQUIRE(clearance[1]
            == Catch::Approx(rig.sd.rideHeight - kSeedError * (1.f - W * W)).margin(1e-3f));
    //    ...and eight ticks is enough to be back, which is what the gains are chosen for.
    REQUIRE(clearance.back() == Catch::Approx(rig.sd.rideHeight).margin(1e-3f));

    // 3. AND THERE IS NO LATERAL TRAVEL AT ALL.
    REQUIRE(glm::abs(driftX) < kPreFixLateralPerCmOfError * kSeedError * 0.01f);
    // ...in fact EXACTLY zero: step 5 composes `up * velocityUp` with `up` = (0,0,1) and a
    // zero tangential term, so the X component is 0 identically rather than merely small.
    REQUIRE(driftX == 0.f);
    REQUIRE(rig.state.bodyState.position.y == 0.f);
}

// ===========================================================================
// STEP 4 -- THE AUTHORED GRAVITY LAW
// ===========================================================================

TEST_CASE("BrawlerMovement.UnsupportedIsPureGravity", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    rig.query.scriptedSweeps.clear();                    // nothing under the capsule
    rig.state.velocity = glm::vec3(0.f, 0.f, -100.f);

    // 1. THE EXISTING FALL SPEED IS KEPT AND ADDED TO -- there is no hold, no servo and no
    //    reset while airborne. Three ticks, each a pure accumulation of `gravity * dt`.
    float previous = rig.state.velocity.z;
    for (std::uint32_t t = 1u; t <= 3u; ++t)
    {
        rig.tick(t);
        INFO("tick " << t << " vz=" << rig.state.velocity.z
             << " delta=" << (rig.state.velocity.z - previous));
        REQUIRE(rig.support() == movement::SupportState::Unsupported);
        REQUIRE(rig.state.velocity.z - previous
                == Catch::Approx(rig.gravityPerTick()).margin(1e-3f));
        previous = rig.state.velocity.z;
    }
    REQUIRE(rig.state.velocity.z
            == Catch::Approx(-100.f + 3.f * rig.gravityPerTick()).margin(1e-3f));

    // 2. AND THE FALL IS BOUNDED. One tick short of terminal, the clamp lands the speed
    //    ON `-terminalFallSpeed` rather than past it, and holds it there.
    ScriptedRig terminal;
    terminal.query.scriptedSweeps.clear();
    terminal.state.velocity =
        glm::vec3(0.f, 0.f, -terminal.sd.terminalFallSpeed + 5.f);
    terminal.tick(1u);
    REQUIRE(terminal.state.velocity.z
            == Catch::Approx(-terminal.sd.terminalFallSpeed).margin(1e-4f));
    terminal.tick(2u);
    REQUIRE(terminal.state.velocity.z
            == Catch::Approx(-terminal.sd.terminalFallSpeed).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.AuthoredGravityWhileUnsupported", "[BrawlerMovement]")
{
    using namespace movementTests;

    // ⭐ THE PROPERTY IS AUTHORED-NESS, not the value: `gravity` is a PER-CHARACTER
    // `StaticData` knob (ruling #16 a), so two characters in the same world can fall at
    // different rates. A world constant could not produce the second arm.
    ScriptedRig heavy;
    heavy.query.scriptedSweeps.clear();
    heavy.tick(1u);
    REQUIRE(heavy.state.velocity.z == Catch::Approx(heavy.gravityPerTick()).margin(1e-4f));

    ScriptedRig floaty;
    floaty.query.scriptedSweeps.clear();
    floaty.sd.gravity = heavy.sd.gravity * 0.5f;
    floaty.tick(1u);
    REQUIRE(floaty.state.velocity.z == Catch::Approx(floaty.gravityPerTick()).margin(1e-4f));

    INFO("heavy vz=" << heavy.state.velocity.z << "  floaty vz=" << floaty.state.velocity.z);
    REQUIRE(floaty.state.velocity.z
            == Catch::Approx(heavy.state.velocity.z * 0.5f).margin(1e-4f));

    // ⛔ AND THE PAIRING THAT MAKES IT SOUND: the BODY's own gravity is OFF at the
    // descriptor. With the body re-placed from `State` every tick, engine gravity would
    // double-apply against this term AND perturb the very post-solve position step 6'
    // measures its push-out from.
    REQUIRE_FALSE(movement::PhysicsSetup::body.body.enableGravity);
}

// ===========================================================================
// STEP 2 + STEP 4 -- THE ONE VERTICAL LAW  [movement-sim task 56 / USER RULING #28, 2026-09-07]
//
// Step 4 used to be an `if` with two DIFFERENT KINDS of law in its arms: on `Floor` the vertical
// velocity was ASSIGNED from clearance error (dead-beat, clamped at `maxSnapSpeed`, gravity OFF);
// on `Airborne` it was INTEGRATED (gravity ON, retaining whatever `velocityUp` held). Crossing the
// boundary handed a POSITION-SERVO OUTPUT to a law that treats its input as MOMENTUM. Ruling #28
// replaced both with one: gravity every tick, a spring-damper servo term added while `Supported`.
//
// ⭐ WHAT THE CASES BELOW MEASURE, AND WHICH OF THEM DISCRIMINATE. Every one was run against a
// scratch restoration of the dead-beat (impl notes §2 records the RED output); the arms are named
// in each case rather than claimed in general:
//     RED under the dead-beat   HoverHoldsRideHeight arm 2 (-500 -> 0.000000 vs -24.666668)
//                               UpwardLaunchIsDampedNotOverwritten (2000 -> 0.000000 vs 1483.67)
//                               NoVelocityStepAtSupportBoundary (972.00 vs 483.67 cm/s step)
//                               HoverCatchesSnapBandDrop (six ticks of a -400 PLATEAU)
//                               GravityRunsWhileSupported (no such term to starve)
//     GREEN in BOTH arms        LedgeFallStartsFromRest -- and it says so, with both arms'
//                               measured numbers, because the AC's premise did not reproduce.
// ===========================================================================

namespace movementTests
{

// ---------------------------------------------------------------------------
// A LEDGE: the quarter-space { x <= edgeX, z <= 0 }, swept by the DESCRIBED probe.
//
// The first mock in this file that can express a character walking OFF something. The two ground
// mocks above cannot: `PlaneGroundQueryAdapter` is an infinite plane and `WorldSweepQueryAdapter`
// composes half-spaces, and a ledge is neither -- it is a solid with an EDGE, and the edge is the
// whole of the geometry that matters. Like `WorldSweepQueryAdapter` it takes the probe from the
// SHIPPED descriptor rather than restating it.
//
// Geometry, from first principles, quoting no part of the sub-simulation. The movement probe is
// always swept straight down, so this models that case only (and asserts it). For an upright
// capsule of radius r whose lower hemisphere centre is `q` (edge at the origin):
//   * q.x <= 0 -- the capsule is over the TOP FACE. It contacts at `q.z - r`, normal +Z.
//   * 0 < q.x < r -- the capsule clears the top face and contacts the EDGE LINE itself. The swept
//     sphere touches the edge when `(q.z - t)^2 + q.x^2 == r^2`, i.e. `t = q.z - sqrt(r^2 - q.x^2)`,
//     and the contact normal points from the edge to the sphere centre: `(q.x, 0, h) / r` with
//     `h = sqrt(r^2 - q.x^2)`.
//   * q.x >= r -- nothing under the capsule at all.
// ⭐ THE EDGE NORMAL TILTS, AND THAT IS WHAT ENDS SUPPORT. Its z component is `h / r`, so it
// passes `maxSlopeAngleDeg` = 45 deg when `q.x` reaches `r / sqrt(2)` = 28.99 cm -- the character
// detaches while the probe is still BLOCKED, on the normal test, not on running out of reach.
// That is what a real sweep against a real box does, and it is why this case needs a mock that
// answers with a normal rather than a scripted clearance.
//
// ⚠ THERE IS NO SOLVER HERE, deliberately: the subject is step 4's law, and `drivesBody` stays
// false so step 6' never runs. The body sinks about 0.5 cm past the floor plane over the traverse,
// which a real depenetration would have taken out; it does not touch what is asserted.
// ---------------------------------------------------------------------------
struct LedgeQueryAdapter
{
    float edgeX = 0.f;

    // THE PROBE, read from the shipped descriptor rather than restated.
    float     probeRadius     = 0.f;
    float     probeHalfHeight = 0.f;
    glm::vec3 probeOffset{ 0.f };

    // Recorded per call, so a case reads exactly what the sub-simulation consumed.
    bool      lastBlocked   = false;
    float     lastClearance = 0.f;
    glm::vec3 lastNormal{ 0.f };

    void configureFrom(const movement::StaticData& sd)
    {
        const std::vector<QueryVolumeDescriptor> volumes = movement::PhysicsSetup::queryVolumes(sd);
        REQUIRE(volumes.size() == 1u);
        const CapsuleGeometry* capsule = std::get_if<CapsuleGeometry>(&volumes[0].geometry);
        REQUIRE(capsule != nullptr);
        probeRadius     = capsule->radius;
        probeHalfHeight = capsule->halfHeight;
        probeOffset     = glm::vec3(volumes[0].offsetTransform[3]);
    }

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& pose, const glm::vec3& delta)
    {
        SweepHit hit{};
        if (volumeId != kVolumeId)
            return hit;

        const float length = glm::length(delta);
        REQUIRE(length > 0.f);
        REQUIRE(delta.x == 0.f);              // this mock models the straight-down sweep only
        REQUIRE(delta.y == 0.f);
        REQUIRE(delta.z < 0.f);

        const glm::vec3 centre   = glm::vec3(pose[3]) + probeOffset;
        const float     axisHalf = probeHalfHeight - probeRadius;
        const float     qx       = centre.x - edgeX;
        const float     qz       = centre.z - axisHalf;

        float     travel = 0.f;
        glm::vec3 normal(0.f, 0.f, 1.f);
        if (qx <= 0.f)
        {
            travel = qz - probeRadius;
        }
        else if (qx < probeRadius)
        {
            const float h = glm::sqrt(probeRadius * probeRadius - qx * qx);
            travel = qz - h;
            normal = glm::vec3(qx, 0.f, h) / probeRadius;
        }
        else
        {
            lastBlocked   = false;
            lastClearance = length;
            lastNormal    = glm::vec3(0.f);
            return hit;                        // past the edge entirely: nothing underneath
        }

        if (travel < 0.f || travel > length)
        {
            lastBlocked   = false;
            lastClearance = length;
            lastNormal    = glm::vec3(0.f);
            return hit;
        }

        hit.blocked     = true;
        hit.fraction    = travel / length;
        hit.normal      = normal;
        hit.impactPoint = centre + glm::vec3(0.f, 0.f, -travel);
        lastBlocked     = true;
        lastClearance   = travel;
        lastNormal      = normal;
        return hit;
    }

    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<LedgeQueryAdapter>);

using LedgeRig = Rig<LedgeQueryAdapter>;

} // namespace movementTests

// ---------------------------------------------------------------------------
// THE RENAME'S ONE STRUCTURAL CLAIM: `SupportState` occupies the SAME TWO BITS `SurfaceKind` did.
// That is what makes this task's "no wire pin moves" true, and it is asserted here rather than
// inferred from the composite size staying 321 -- a byte count cannot tell a re-layout from a
// rename, and `syncSize<State>() == 61u` would have been GREEN either way.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.SupportStateRoundTripsThroughTheSameTwoFlagBits", "[BrawlerMovement]")
{
    using namespace movementTests;

    REQUIRE(movement::kFlagSupportShift == 1u);
    REQUIRE(movement::kFlagSupportMask  == 0x06u);
    // ...and the mask does not touch either neighbour in the byte.
    REQUIRE((movement::kFlagSupportMask & movement::kFlagFrozen)     == 0u);
    REQUIRE((movement::kFlagSupportMask & movement::kFlagHasCommand) == 0u);

    // All three enumerators survive the pack/unpack, INCLUDING `SupportedSteep`, which has no
    // producer in v1. A value the wire cannot carry would be a defect discovered by task 48.
    for (const movement::SupportState value : { movement::SupportState::Unsupported,
                                                movement::SupportState::Supported,
                                                movement::SupportState::SupportedSteep })
    {
        const std::uint8_t bits = static_cast<std::uint8_t>(value);
        std::uint8_t flags = movement::kFlagFrozen | movement::kFlagHasCommand;
        flags = static_cast<std::uint8_t>(flags & ~movement::kFlagSupportMask);
        flags = static_cast<std::uint8_t>(
            flags | ((bits << movement::kFlagSupportShift) & movement::kFlagSupportMask));

        const std::uint8_t readBack =
            static_cast<std::uint8_t>((flags & movement::kFlagSupportMask)
                                      >> movement::kFlagSupportShift);
        INFO("value=" << int(bits) << " flags=0x" << std::hex << int(flags));
        REQUIRE(readBack == bits);
        REQUIRE((flags & movement::kFlagFrozen)     != 0u);   // neighbours untouched
        REQUIRE((flags & movement::kFlagHasCommand) != 0u);
    }
}

// ---------------------------------------------------------------------------
// ⭐⭐ THE DISCRETE STABILITY BOUND, PINNED -- because the alternative is finding it in PIE, and
// the obvious bound is the WRONG one.
//
// The design and the implementer brief both carried "explicit Euler is stable for omega*dt < 2",
// i.e. `hoverFrequency < 120` at 60 Hz. Two things are wrong with it, and both were MEASURED on the
// recurrence before this case was written:
//   1. The step is SEMI-IMPLICIT (symplectic) Euler -- step 4 produces the new velocity and the
//      engine integrates position with THAT (`x += v'*dt`), not with the old one.
//   2. The damping term is part of the bound, and a bare frequency cap cannot express a
//      two-parameter region.
// Jury's criterion on the 2x2 update matrix (trace `2 - a - b`, determinant `1 - a`, with
// `a = c*dt` and `b = (omega*dt)^2`) collapses to `b + 2a < 4`, i.e.
//
//        W^2 + 4*zeta*W < 4        W = hoverFrequency * dt
//
// ⚠ THE RETIRED BOUND WOULD HAVE ADMITTED DIVERGENT GAINS, and the rows below assert exactly
// that rather than merely asserting the shipped pair passes: omega 60 zeta 1 diverges
// (1, 1, 2, 3, 5, 8, 13, ...) and omega 119 zeta 1 reaches 1e6 in eight ticks. Both satisfy
// `omega < 120`. A case that only checked the shipped pair would have been GREEN under the wrong
// bound, which is the shape this initiative keeps being bitten by.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.HoverGainsAreStableAtTheSimStep", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;

    // The constant `StaticData`'s constructor checks against IS this rig's step. It cannot read
    // `dt` -- it runs once per session, before any tick -- so the two must be pinned equal here.
    REQUIRE(movement::kNominalSimStepSeconds == kDt);

    const auto stabilityMargin = [](float omega, float zeta)
    { const float W = omega * kDt; return 4.f - (W * W + 4.f * zeta * W); };

    INFO("shipped omega=" << rig.sd.hoverFrequency << " zeta=" << rig.sd.hoverDampingRatio
         << "  W^2+4zW=" << (4.f - stabilityMargin(rig.sd.hoverFrequency, rig.sd.hoverDampingRatio))
         << " (< 4)  discrete critical zeta=" << rig.discreteCriticalZeta()
         << "  eigenvalue=" << rig.discreteCriticalEigenvalue());

    // 1. THE SHIPPED PAIR IS INSIDE THE REGION.
    REQUIRE(stabilityMargin(rig.sd.hoverFrequency, rig.sd.hoverDampingRatio) > 0.f);

    // 2. ⭐ AND THE BOUND IS STRICTLY STRONGER THAN THE FREQUENCY CAP IT REPLACED. These two rows
    //    are the case's discriminator: they FAIL if someone re-derives `hoverFrequency < 120`.
    REQUIRE(60.f  < 120.f);                                  // the retired cap ADMITS it...
    REQUIRE(stabilityMargin(60.f,  1.f) <= 0.f);             // ...and this one REJECTS it
    REQUIRE(119.f < 120.f);
    REQUIRE(stabilityMargin(119.f, 1.f) <= 0.f);

    // 3. THE DERIVED GAINS ARE THE omega/zeta PAIR, not independently authored numbers. A tuner
    //    must not be able to reach a (k, c) that no (omega, zeta) produces -- that is exactly how
    //    the constructor's check would be bypassed.
    REQUIRE(rig.sd.hoverStiffness
            == Catch::Approx(rig.sd.hoverFrequency * rig.sd.hoverFrequency).margin(1e-3f));
    REQUIRE(rig.sd.hoverDamping
            == Catch::Approx(2.f * rig.sd.hoverDampingRatio * rig.sd.hoverFrequency).margin(1e-3f));

    // 4. ⭐⭐ STABLE IS NOT SMOOTH, AND THE MONOTONE REGION IS TIGHTER. At the discrete critical
    //    zeta `1 - W/2` the two eigenvalues coincide at `1 - W`; monotone decay needs THAT
    //    positive as well, i.e. `omega < 1/dt`. Critical damping buys the fastest decay, not a
    //    monotone one -- omega 70 (critical zeta 0.417) and omega 80 (0.333) are both critically
    //    damped and both alternate sign every tick.
    REQUIRE(rig.sd.hoverDampingRatio >= rig.discreteCriticalZeta());
    REQUIRE(rig.discreteCriticalEigenvalue() > 0.f);
    REQUIRE(rig.sd.hoverFrequency < 1.f / kDt);
    REQUIRE(rig.discreteCriticalZeta()
            == Catch::Approx(1.f - rig.sd.hoverFrequency * kDt * 0.5f).margin(1e-6f));
}

// ---------------------------------------------------------------------------
// ⭐⭐ AND THE MONOTONE CLAIM IS RUN, NOT ONLY DERIVED. Two arms, differing in ONE constant.
//
// This is the case that justifies shipping zeta = 0.62 rather than the 1 the design proposed.
// zeta = 1 is the CONTINUOUS-time critical damping ratio; for this recurrence at omega = 46 the
// critical value is `1 - omega*dt/2` = 0.6167, and at zeta = 1 the response ALTERNATES SIGN and
// takes ~13 ticks to lose 95 % instead of ~4.
// ⚠ The second arm has to set `hoverDamping` as well as `hoverDampingRatio`: `c` is DERIVED in
// the constructor, and a case that moved only the ratio would have measured nothing while looking
// like it had. That coupling is the point of deriving it, and this is the one place a test is
// allowed to reach past it.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.DiscreteCriticalDampingIsMonotone", "[BrawlerMovement]")
{
    using namespace movementTests;

    const auto errorTrace = [](float zeta)
    {
        PlaneRig rig;
        rig.sd.hoverDampingRatio = zeta;
        rig.sd.hoverDamping      = 2.f * zeta * rig.sd.hoverFrequency;   // the derived pair, kept true
        // ⭐ SEEDED ONE cm *BELOW* RIDE HEIGHT SINCE movement-sim TASK 57 / RULING #29 -- the
        // servo only exists on that side now, and a seed above it would trace gravity's parabola
        // instead of the recurrence this case is about. The magnitudes are unchanged: the
        // linearised recurrence is the same one, driven from the other side of `e == 0`.
        rig.state.bodyState.position =
            glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight - 1.f));

        std::vector<float> error;
        for (std::uint32_t t = 0u; t < 8u; ++t)
        {
            rig.tick(t);
            rig.engineStep();
            error.push_back(rig.query.lastClearance - rig.sd.rideHeight);
        }
        return error;
    };

    ScriptedRig reference;
    const std::vector<float> shipped  = errorTrace(reference.sd.hoverDampingRatio);
    const std::vector<float> continuous = errorTrace(1.f);

    const auto trace = [](const std::vector<float>& e)
    {
        std::string s;
        for (const float v : e) s += std::to_string(v) + " ";
        return s;
    };
    INFO("zeta=" << reference.sd.hoverDampingRatio << " (discrete critical "
         << reference.discreteCriticalZeta() << "): " << trace(shipped)
         << "\n  zeta=1 (continuous critical): " << trace(continuous));

    // THE SHIPPED ARM: strictly shrinking, every tick.
    int shippedGrowthTicks = 0;
    for (std::size_t i = 1; i < shipped.size(); ++i)
    {
        REQUIRE(glm::abs(shipped[i]) < glm::abs(shipped[i - 1]));
        if (glm::abs(shipped[i]) > glm::abs(shipped[i - 1])) ++shippedGrowthTicks;
    }
    REQUIRE(shippedGrowthTicks == 0);

    // ⚠ THE CONTROL ARM: zeta = 1 RINGS, and the observable is the error GROWING on alternate
    // ticks -- .412 .483 .161 .238 .057 .120 -- not a sign change. The measured trace does not
    // cross zero here: the negative eigenvalue (-0.7934 at zeta = 1) shows up as the magnitude
    // bouncing back up while the error stays on one side, which is what a player sees as a
    // wobble. MEASURED before this row was written; a sign-change test would have been GREEN in
    // both arms and this case would have proved nothing.
    // It is still STABLE -- both arms are inside `W^2 + 4zW < 4` -- which is exactly why the
    // stability bound alone cannot choose a zeta.
    int continuousGrowthTicks = 0;
    for (std::size_t i = 1; i < continuous.size(); ++i)
        if (glm::abs(continuous[i]) > glm::abs(continuous[i - 1])) ++continuousGrowthTicks;
    REQUIRE(continuousGrowthTicks > 0);
    // ...and it is slower to settle despite being the "more damped" number, which is the
    // counter-intuitive half and the reason this is a case and not a comment.
    REQUIRE(glm::abs(continuous.back()) > glm::abs(shipped.back()));
}

// ---------------------------------------------------------------------------
// ⭐⭐ GRAVITY IS ADDED ON EVERY TICK, INCLUDING WHILE SUPPORTED -- and this is the case that
// MEASURES it rather than reading the source.
//
// The trick is to starve the servo: with `hoverMaxAccel` below `|gravity|` the servo cannot hold
// the character up even at zero error, so a SUPPORTED character sinks at exactly
// `(gravity + hoverMaxAccel)*dt` per tick. There is no arrangement of the retired law that can
// produce that reading -- on `Floor` it assigned `velocityUp` from clearance and gravity was not
// in the branch at all, and `hoverMaxAccel` did not exist. RED by construction.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.GravityRunsWhileSupported", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig starved;
    seatOnFlatGroundAtRideHeight(starved);
    starved.sd.hoverMaxAccel = 500.f;                       // below |gravity| = 980
    REQUIRE(starved.sd.hoverMaxAccel < glm::abs(starved.sd.gravity));   // the premise

    const float sinkPerTick = (starved.sd.gravity + starved.sd.hoverMaxAccel) * kDt;
    for (std::uint32_t t = 1u; t <= 3u; ++t)
    {
        starved.tick(t);
        INFO("tick " << t << " vz=" << starved.state.velocity.z
             << " expected " << (sinkPerTick * float(t)));
        REQUIRE(starved.support() == movement::SupportState::Supported);   // it IS supported
        REQUIRE(starved.state.velocity.z
                == Catch::Approx(sinkPerTick * float(t)).margin(1e-3f));
        REQUIRE(starved.state.velocity.z < 0.f);                           // ...and sinking
    }

    // CONTROL -- the SAME rig with the shipped ceiling holds ride height exactly. So the sink is
    // the starvation, and the servo's authority is the only thing that changed between the arms.
    ScriptedRig healthy;
    seatOnFlatGroundAtRideHeight(healthy);
    healthy.tick(1u);
    REQUIRE(healthy.support() == movement::SupportState::Supported);
    REQUIRE(healthy.state.velocity.z == 0.f);
    // ...and what the servo is holding up against is exactly one gravity.
    REQUIRE(healthy.sd.hoverStiffness * 0.f - healthy.sd.gravity
            == Catch::Approx(-healthy.sd.gravity).margin(1e-4f));
}

// ---------------------------------------------------------------------------
// ⭐⭐ NO VELOCITY STEP AT THE SUPPORT BOUNDARY -- the property ruling #28 exists to buy, and the
// strongest RED arm in this file.
//
// The retired law ASSIGNED `velocityUp` while on `Floor`, so the tick a fall entered the band it
// threw away however much momentum the character had and replaced it with a clamped position
// correction. MEASURED on a 10 m fall: the dead-beat's worst per-tick `|delta velocity.up|` is
// 972.00 cm/s -- it destroyed 972 cm/s of real motion in one tick -- against 483.67 under this law,
// inside the 516.33 bound. Nothing here is ever assigned, so the bound is structural.
//
// ⚠ THE BOUND IS `(hoverMaxAccel + |gravity|)*dt`, NOT `hoverMaxAccel*dt`. The clamp bounds the
// SERVO TERM, and the servo term carries the `- gravity` feed-forward, so the total acceleration
// reaches `hoverMaxAccel + |gravity|` on downward saturation. The tidier form (500.000000) is
// exceeded by this very case at 516.33 on the saturating tick, which is how the difference was
// caught rather than argued.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.NoVelocityStepAtSupportBoundary", "[BrawlerMovement]")
{
    using namespace movementTests;

    PlaneRig rig;
    constexpr float kDropHeight = 1000.f;      // 10 m: enters the band at ~1372 cm/s
    rig.state.bodyState.position =
        glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight + kDropHeight));

    float        worstStep      = 0.f;
    int          worstTick      = -1;
    float        entrySpeed     = 0.f;
    float        stepAtTheFlip  = 0.f;
    int          boundaryFlips  = 0;
    bool         everSupported  = false;
    bool         everUnsupported = false;
    movement::SupportState previous = movement::SupportState::Unsupported;

    for (std::uint32_t t = 0u; t < 400u; ++t)
    {
        const float before = rig.state.velocity.z;
        rig.tick(t);
        const float step = glm::abs(rig.state.velocity.z - before);

        if (rig.support() == movement::SupportState::Supported)   everSupported = true;
        if (rig.support() == movement::SupportState::Unsupported) everUnsupported = true;
        if (t > 0u && rig.support() != previous)
        {
            ++boundaryFlips;
            if (rig.support() == movement::SupportState::Supported)
            {
                entrySpeed    = before;
                stepAtTheFlip = step;
            }
        }
        previous = rig.support();

        if (step > worstStep) { worstStep = step; worstTick = int(t); }
        rig.engineStep();
    }

    INFO("10 m fall: entered the band at " << entrySpeed << " cm/s, step at that flip "
         << stepAtTheFlip << ", worst step " << worstStep << " at tick " << worstTick
         << ", bound (hoverMaxAccel + |gravity|)*dt = " << rig.maxVerticalStep()
         << " (the retired dead-beat's worst step on this same fall: 972.00)");

    // ---- DISCRIMINATOR ROWS. A bounded step is worthless if nothing happened.
    REQUIRE(everUnsupported);                                   // it fell
    REQUIRE(everSupported);                                     // it landed
    REQUIRE(boundaryFlips >= 1);                                // it crossed
    REQUIRE(entrySpeed < -1000.f);                              // FAST when it crossed
    REQUIRE(worstStep > glm::abs(rig.gravityPerTick()) * 2.f);  // the servo really fired

    // ---- THE SUBJECT.
    REQUIRE(worstStep <= rig.maxVerticalStep() + 1e-2f);
    REQUIRE(stepAtTheFlip <= rig.maxVerticalStep() + 1e-2f);
    // ...and it settles where it should, so the bound was not bought by doing nothing.
    REQUIRE(rig.query.lastClearance == Catch::Approx(rig.sd.rideHeight).margin(1e-2f));
    REQUIRE(rig.support() == movement::SupportState::Supported);
}

// ---------------------------------------------------------------------------
// A DROP OF `snapDistance`, CAUGHT -- and caught WITHOUT the retired 400 cm/s constant descent.
//
// The floor falls away by exactly `snapDistance`, which puts clearance at 50.000000 == the probe's
// whole reach: the last reading that is still support at all, and the same edge task 9 q3 measured
// ruling #13's constants against.
//
// ⭐ THE DISCRIMINATOR IS THE SHAPE OF THE DESCENT, NOT THE CATCH. The dead-beat clamped at
// `maxSnapSpeed`, so it descended at EXACTLY -400.000000 for six consecutive ticks -- a plateau,
// which reads in game as a lift rather than a fall. The law accelerates into the drop, so no two
// consecutive ticks share a speed and the peak is not the first tick. Both rows below are RED
// against the dead-beat.
//
// ⭐⭐ RE-PINNED AGAIN BY movement-sim TASK 57 / RULING #29, and this is the case where the
// user-visible change lives. Under task 56 the two-sided spring SATURATED into this drop
// (-516.333 cm/s on the first tick) and had it caught in 8 ticks; the descent is now GRAVITY, so
// the same 40 cm takes ~18 ticks to fall and ~22 to settle, and the character DIPS about 1.6 cm
// below ride height on the way in because the whole fall now has to be absorbed by the 10 cm
// below ride height instead of by the 40 above it. That is the shape of a step-down under
// "supported means held up, never pulled down", and whether it feels right is the user's PIE
// call on `hoverPullDownAccel` -- `PullDownKnobRecapturesStep` measures both knob settings, and
// `FreeFallAcceleratesAtGravity` owns the "every tick is gravity" statement.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.HoverCatchesSnapBandDrop", "[BrawlerMovement]")
{
    using namespace movementTests;

    PlaneRig rig;
    rig.state.bodyState.position =
        glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight));
    rig.tick(0u);
    REQUIRE(rig.state.velocity.z == Catch::Approx(0.f).margin(1e-4f));
    rig.engineStep();

    rig.query.planeOffset -= rig.sd.snapDistance;          // THE DROP

    std::vector<float> clearance;
    std::vector<float> speed;
    int caughtAtTick = -1;
    for (std::uint32_t t = 1u; t <= 40u; ++t)
    {
        rig.tick(t);
        clearance.push_back(rig.query.lastClearance);
        speed.push_back(rig.state.velocity.z);
        rig.engineStep();
        if (caughtAtTick < 0
            && glm::abs(rig.query.lastClearance - rig.sd.rideHeight) < 0.05f
            && glm::abs(rig.state.velocity.z) < 25.f)
            caughtAtTick = int(t);
    }

    INFO("snap-band drop: clearance=" << clearance[0] << ", " << clearance[1] << ", "
         << clearance[2] << ", " << clearance[3] << ", " << clearance[4] << " ... "
         << clearance.back() << "\n  speed=" << speed[0] << ", " << speed[1] << ", " << speed[2]
         << ", " << speed[3] << ", " << speed[4] << " ... " << speed.back()
         << "\n  caught at tick " << caughtAtTick
         << " (the retired dead-beat held -400.000000 for six ticks)");

    // ---- PREMISE. The drop really put the character at the far edge of the band, still supported.
    REQUIRE(clearance[0]
            == Catch::Approx(rig.sd.rideHeight + rig.sd.snapDistance).margin(1e-3f));
    REQUIRE(rig.support() == movement::SupportState::Supported);

    // ---- 1. NO PLATEAU. Under the dead-beat ticks 0-5 all read -400.000000 exactly.
    for (std::size_t i = 1; i < 8; ++i)
    {
        INFO("tick " << (i + 1) << " speed=" << speed[i] << " previous=" << speed[i - 1]);
        REQUIRE(speed[i] != Catch::Approx(speed[i - 1]).margin(1e-3f));
    }

    // ---- 2. AND THE PEAK IS NOT THE FIRST TICK: the law ACCELERATES into the catch. The
    //         dead-beat reached its clamp immediately and held it, so its peak index is 0.
    std::size_t peak = 0;
    for (std::size_t i = 1; i < speed.size(); ++i)
        if (glm::abs(speed[i]) > glm::abs(speed[peak])) peak = i;
    REQUIRE(peak > 0);

    // ---- 3. CAUGHT, and settled at ride height. ⚠ THE "NEVER PAST RIDE HEIGHT" ROW IS GONE
    //         (task 57): the descent is gravity now, so the character arrives at ride height at
    //         ~280 cm/s and the servo has only the 10 cm below it to absorb that. The dip is
    //         REAL, is bounded, and is asserted as such rather than denied. It is also strictly
    //         smaller than what the 10 cm band could absorb at full saturation, which is the row
    //         that says the character did not bottom out into the geometry.
    const float arrivalSpeed =
        glm::sqrt(2.f * glm::abs(rig.sd.gravity) * rig.sd.snapDistance);
    // TWO REFERENCE DEPTHS, and the measured dip has to sit between them.
    //   * SATURATED: the least distance ANY law bounded by `hoverMaxAccel` could stop in. A spring
    //     is not saturated at small errors, so it always needs MORE than this -- which is why this
    //     is the LOWER bound and asserting it as an upper one is the mistake it looks like.
    //   * CRITICALLY DAMPED: `v0 / (omega * e)`, the peak of `v0*t*exp(-omega*t)`, which is what
    //     this servo actually is. 2.24 cm at the shipped gains against the measured 1.65.
    const float saturatedDip =
        arrivalSpeed * arrivalSpeed / (2.f * (rig.sd.hoverMaxAccel - glm::abs(rig.sd.gravity)));
    const float criticallyDampedDip =
        arrivalSpeed / (rig.sd.hoverFrequency * glm::exp(1.f));
    float minClearance = 1e9f;
    for (const float c : clearance) minClearance = glm::min(minClearance, c);
    INFO("arrival speed " << arrivalSpeed << " cm/s, dip " << (rig.sd.rideHeight - minClearance)
         << " cm; saturated floor " << saturatedDip
         << " cm, critically-damped peak " << criticallyDampedDip << " cm");
    REQUIRE(caughtAtTick > 0);
    REQUIRE(caughtAtTick <= 30);
    REQUIRE(minClearance > 0.f);                                       // did NOT bottom out
    REQUIRE(rig.sd.rideHeight - minClearance >= saturatedDip - 1e-3f);
    REQUIRE(rig.sd.rideHeight - minClearance <= criticallyDampedDip + 1e-3f);
    REQUIRE(clearance.back() == Catch::Approx(rig.sd.rideHeight).margin(1e-3f));
    REQUIRE(rig.sd.hoverDampingRatio >= rig.discreteCriticalZeta());   // why (3) holds
}

// ---------------------------------------------------------------------------
// A 2 m LANDING SETTLES WITHOUT BOUNCE -- no `Unsupported` re-entry after first contact, and the
// character never dips below ride height on the way in.
//
// ⭐ THE DIP ROW IS WHAT DISCRIMINATES THE SHIPPED zeta. At the discrete critical zeta the
// approach is monotone and the minimum clearance IS ride height; at the continuous-time zeta = 1
// the same landing dips to 9.9874 cm and rings back. Both are stable; only one is smooth.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.LandingSettlesWithoutBounce", "[BrawlerMovement]")
{
    using namespace movementTests;

    PlaneRig rig;
    rig.state.bodyState.position =
        glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight + 200.f));

    bool  contacted        = false;
    bool  wasSupported     = false;
    int   reEntries        = 0;
    float minClearance     = 1e9f;
    float impactSpeed      = 0.f;

    for (std::uint32_t t = 0u; t < 400u; ++t)
    {
        const float before = rig.state.velocity.z;
        rig.tick(t);
        const bool supported = rig.support() == movement::SupportState::Supported;
        if (supported && !contacted) { contacted = true; impactSpeed = before; }
        if (contacted && !supported && wasSupported) ++reEntries;
        if (contacted) minClearance = glm::min(minClearance, rig.query.lastClearance);
        wasSupported = supported;
        rig.engineStep();
    }

    // ⭐⭐ THE DIP IS REAL SINCE movement-sim TASK 57 / RULING #29, AND THAT IS THE POINT OF THE
    // CHANGE, NOT A REGRESSION. Under task 56 the two-sided spring braked the fall through the
    // 40 cm ABOVE ride height, so this landing never dipped at all. With the servo one-sided the
    // whole arrival has to be absorbed by the 10 cm BELOW ride height -- the suspension
    // compresses, which is what a landing looks like. It is bounded, it does not bounce, and it
    // settles exactly at ride height.
    // ⚠ THE NUMBER TO CARRY INTO PIE: the 10 cm below ride height can absorb at most
    // `sqrt(2*(hoverMaxAccel - |gravity|)*rideHeight)` = 761.8 cm/s at the shipped constants. A
    // fall arriving faster than that BOTTOMS OUT and the capsule meets the geometry, where
    // depenetration and step 6's landing rule take over. `terminalFallSpeed` is 2000, so a long
    // fall WILL do that -- which is one of the two constants ruling #29 leaves open.
    const float bottomOutSpeed = glm::sqrt(
        2.f * (rig.sd.hoverMaxAccel - glm::abs(rig.sd.gravity)) * rig.sd.rideHeight);

    INFO("2 m landing: impact speed " << impactSpeed << " cm/s, Unsupported re-entries after "
         << "first contact " << reEntries << ", minimum clearance " << minClearance
         << " (dip " << (rig.sd.rideHeight - minClearance) << " cm)"
         << ", settled at " << rig.query.lastClearance
         << "; the band below ride height saturates at " << bottomOutSpeed << " cm/s"
         << " (task 56 did not dip at all here; zeta = 1 dipped to 9.9874)");

    REQUIRE(contacted);                              // premise: it landed
    REQUIRE(impactSpeed < -500.f);                   // premise: hard enough to bounce
    REQUIRE(reEntries == 0);                         // no bounce back out of the band
    REQUIRE(minClearance > 0.f);                     // ...and it did NOT bottom out
    REQUIRE(minClearance < rig.sd.rideHeight - 1.f); // THE COMPRESSION IS REAL -- task 57
    REQUIRE(rig.query.lastClearance == Catch::Approx(rig.sd.rideHeight).margin(1e-3f));
    REQUIRE(rig.state.velocity.z == Catch::Approx(0.f).margin(1e-2f));
}

// ---------------------------------------------------------------------------
// ⭐⭐ AN UPWARD LAUNCH ON THE GROUND IS DAMPED, NOT OVERWRITTEN -- and it leaves the band on
// its own momentum.
//
// The retired law ASSIGNED `velocityUp` from clearance on every `Floor` tick, so an upward
// knockback applied while standing was destroyed on the very next tick: 2000 cm/s -> 0.000000, and
// the character never left the band at all. That is the same mechanism, in the other direction, as
// the fall momentum destroyed in `NoVelocityStepAtSupportBoundary`.
//
// ⚠ THIS IS A BONUS TO VERIFY, NOT A FEATURE BUILT HERE. There is no `Launched` enumerator in
// `DAttackState` yet (task 27), so the launch is seeded directly onto `state.velocity` -- which is
// exactly what that task's branch will do. What is measured is the VERTICAL LAW's response to an
// upward velocity while supported, and that is complete today.
// ⚠ It is also why step 2 needs a detach gate for jump (task 21): while the character is at or
// BELOW ride height the damper still takes up to 516.33 cm/s per tick off the ascent, and the
// probe still reads walkable ground. ⭐ TASK 57 / RULING #29 SHRANK THAT WINDOW BUT DID NOT CLOSE
// IT: once the ascent carries the character ABOVE ride height the law is gravity alone, so only
// the first tick or two of a launch are damped now. At the knockback speed that was already
// survivable; at a jump speed it is still `detachesFromSupport` that has to fix it.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.UpwardLaunchIsDampedNotOverwritten", "[BrawlerMovement]")
{
    using namespace movementTests;

    PlaneRig rig;
    rig.state.bodyState.position =
        glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight));
    rig.state.velocity = glm::vec3(0.f, 0.f, rig.knockbackSpeed());

    const float launch = rig.knockbackSpeed();
    rig.tick(0u);

    INFO("launched at " << launch << " cm/s while supported -> " << rig.state.velocity.z
         << " (the retired dead-beat assigned 0.000000 and the launch died on the spot); "
         << "one tick of servo authority = " << rig.maxVerticalStep());

    // 1. ⭐ IT SURVIVED THE TICK. RED under the dead-beat, which read exactly 0.000000 here.
    REQUIRE(rig.support() == movement::SupportState::Supported);   // still in the band
    REQUIRE(rig.state.velocity.z > 0.f);
    REQUIRE(rig.state.velocity.z != 0.f);

    // 2. ...DAMPED BY EXACTLY THE SERVO'S AUTHORITY, no more and no less. The damper term at
    //    2000 cm/s asks for far more than the ceiling, so this tick saturates.
    REQUIRE(rig.state.velocity.z
            == Catch::Approx(launch - rig.maxVerticalStep()).margin(1e-2f));
    REQUIRE(rig.sd.hoverDamping * launch - rig.sd.gravity > rig.sd.hoverMaxAccel);   // saturated

    // 3. AND IT LEAVES THE BAND ON ITS OWN MOMENTUM, still climbing.
    rig.engineStep();
    int leftAtTick = -1;
    for (std::uint32_t t = 1u; t <= 8u && leftAtTick < 0; ++t)
    {
        rig.tick(t);
        if (rig.support() == movement::SupportState::Unsupported) leftAtTick = int(t);
        rig.engineStep();
    }
    INFO("left the band at tick " << leftAtTick << " with vz=" << rig.state.velocity.z);
    REQUIRE(leftAtTick > 0);
    REQUIRE(rig.state.velocity.z > 0.f);
    // ...and once out, it is pure gravity: the servo term is gone, nothing was assigned.
    const float beforeBallistic = rig.state.velocity.z;
    rig.tick(9u);
    REQUIRE(rig.support() == movement::SupportState::Unsupported);
    REQUIRE(rig.state.velocity.z
            == Catch::Approx(beforeBallistic + rig.gravityPerTick()).margin(1e-3f));
}

// ---------------------------------------------------------------------------
// ⚠⚠ A LEDGE WALK-OFF -- AND THIS CASE IS GREEN IN BOTH ARMS ON PURPOSE. READ THIS BEFORE
// TREATING IT AS A REGRESSION GUARD FOR RULING #28.
//
// The acceptance criterion asked for it "shown RED against the dead-beat by ~ maxSnapSpeed". THAT
// PREMISE DID NOT REPRODUCE, and it was measured on both laws before this case was written:
//
//   arm                        steady hover, ground vanishes     first-principles ledge corner
//   dead-beat (retired)        vUp = -16.333334 (= gravity*dt)   detaches at -107.07 cm/s
//   ruling #28 (shipped)       vUp = -16.333334 (= gravity*dt)   detaches at -101.61 cm/s
//
// ⭐ WHY, stated plainly so nobody re-derives it: the dead-beat only commands `-maxSnapSpeed` when
// the clearance error EXCEEDS `maxSnapSpeed*dt` = 6.67 cm, i.e. when the floor recedes faster than
// 400 cm/s. A walk-off at `maxWalkSpeed` = 100 cm/s never does: the probe's clearance grows at the
// rate the capsule's lower hemisphere loses the ledge corner, which reaches ~107 cm/s at the angle
// where the corner normal passes `maxSlopeAngleDeg` and support ends. BOTH laws are TRACKING that
// recession, and both retain the tracking velocity -- the character genuinely has it.
// ⭐ WHAT RULING #28 CHANGED IS PROVENANCE, NOT MAGNITUDE: the retained velocity is now something
// the body was physically doing, rather than a one-tick position correction reinterpreted as
// momentum. The cases that measure THAT are `NoVelocityStepAtSupportBoundary` and
// `UpwardLaunchIsDampedNotOverwritten`.
//
// What this case is still worth: it pins that a walk-off FROM REST starts from rest exactly, that
// nothing anywhere in the descent equals an authored velocity constant, and that every ballistic
// tick after detachment is exactly one `gravity*dt`. It is the only ledge geometry in the file.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.LedgeFallStartsFromRest", "[BrawlerMovement]")
{
    using namespace movementTests;

    // ---- SECTION 1: the literal reading of the AC. Steady hover, then the ground is gone.
    //      EXACT in both arms, and worth pinning precisely because it is exact: a fall that begins
    //      from a settled hover begins at one tick of gravity and nothing else.
    ScriptedRig walkOff;
    seatOnFlatGroundAtRideHeight(walkOff);
    walkOff.tick(1u);
    REQUIRE(walkOff.support() == movement::SupportState::Supported);
    REQUIRE(walkOff.state.velocity.z == 0.f);                     // AT REST, exactly

    walkOff.query.scriptedSweeps.clear();                         // over the edge
    walkOff.tick(2u);
    REQUIRE(walkOff.support() == movement::SupportState::Unsupported);
    REQUIRE(walkOff.state.velocity.z
            == Catch::Approx(walkOff.gravityPerTick()).margin(1e-4f));
    walkOff.tick(3u);
    REQUIRE(walkOff.state.velocity.z
            == Catch::Approx(2.f * walkOff.gravityPerTick()).margin(1e-4f));

    // ---- SECTION 2: the real geometry. Walk off a ledge corner at full stick.
    LedgeRig rig;
    rig.query.configureFrom(rig.sd);
    rig.query.edgeX = 0.f;
    rig.state.bodyState.position =
        glm::vec3(-30.f, 0.f, rig.sd.capsuleHalfHeight + rig.sd.rideHeight);

    float detachSpeed   = 0.f;
    int   detachTick    = -1;
    float steadyHoverVz = 1e9f;
    float steadyClearance = 0.f;
    std::vector<float> ballisticDeltas;

    for (std::uint32_t t = 0u; t < 120u; ++t)
    {
        const float before = rig.state.velocity.z;
        rig.tick(t, stick(1.f, 0.f));

        if (t == 10u)                     // well before the edge: settled hover
        {
            steadyHoverVz   = rig.state.velocity.z;
            steadyClearance = rig.query.lastClearance;
        }
        if (detachTick < 0 && t > 0u
            && rig.support() == movement::SupportState::Unsupported)
        {
            detachTick  = int(t);
            detachSpeed = rig.state.velocity.z;
        }
        else if (detachTick > 0 && int(t) <= detachTick + 4)
        {
            ballisticDeltas.push_back(rig.state.velocity.z - before);
        }
        rig.engineStep();
    }

    INFO("ledge corner: settled hover vz=" << steadyHoverVz << " clearance=" << steadyClearance
         << "; detached at tick " << detachTick << " with vz=" << detachSpeed
         << " (measured on the retired dead-beat, same rig: -107.07)"
         << "; probe radius " << rig.query.probeRadius
         << ", support ends when the corner normal passes " << rig.sd.maxSlopeAngleDeg << " deg");

    // 1. PREMISE: it really was in a settled hover before the edge, and it really did detach.
    REQUIRE(steadyClearance == Catch::Approx(rig.sd.rideHeight).margin(1e-3f));
    REQUIRE(steadyHoverVz == Catch::Approx(0.f).margin(1e-3f));
    REQUIRE(detachTick > 10);

    // 2. THE RETAINED VELOCITY IS INSIDE THE LAW'S OWN AUTHORITY -- it is a tracked recession, not
    //    an assigned constant. (The AC's bound; GREEN in both arms, see the header.)
    REQUIRE(glm::abs(detachSpeed) <= rig.maxVerticalStep());
    REQUIRE(detachSpeed < 0.f);
    // ...and it is not any authored constant: not the walk speed, not terminal fall.
    REQUIRE(glm::abs(detachSpeed) != Catch::Approx(rig.sd.maxWalkSpeed).margin(1.f));
    REQUIRE(glm::abs(detachSpeed) < rig.sd.terminalFallSpeed);

    // 3. AND EVERY TICK AFTER DETACHMENT IS EXACTLY ONE `gravity*dt`. Nothing else is acting: the
    //    servo term is gone the moment support is, and no branch re-assigns anything.
    REQUIRE(ballisticDeltas.size() >= 3u);
    for (const float delta : ballisticDeltas)
        REQUIRE(delta == Catch::Approx(rig.gravityPerTick()).margin(1e-3f));
}

// ---------------------------------------------------------------------------
// ⭐⭐ THE DETACH GATE IS NO LONGER VACUOUS [movement-sim task 27].
//
// `DetachGateIsVacuousUntilJumpLands` STOOD HERE and its own header said to delete it the day the
// predicate was filled, because "a vacuity pin that outlives its vacuity is a false comfort". Task
// 27 fills it — and NOT with the enumerator the old fence anticipated. There is no `Launched`
// state by design (user ruling 2026-09-12); the arm is keyed on what actually matters,
// `committed && dot(velocity, up) > 0`, so it is FALSE for every XY knockback that ships today
// and TRUE the day a lift is authored, with no enumerator knowledge at all.
//
// ⛔ BOTH ARMS ARE DRIVEN HERE, which is what the old case could not do. Two of the three inputs
// are held while the third moves, so a predicate that ignored `committed`, or that ignored the
// sign of the vertical channel, fails a named arm rather than passing by luck.
// ⛔ STILL DOES NOT PROVE step 2 consults it — that wiring was verified under task 56 by poisoning
// the predicate to `return true` and watching four hover cases go red together; the run is in
// `impl/impl_notes_seam_56.md` §2. What IS new is that the poison is no longer the only way to
// reach the true arm.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.XYKnockbackDoesNotDetach", "[BrawlerMovement]")
{
    using namespace movementTests;

    movement::State state;

    // ARM 1 -- NOT COMMITTED. Nothing detaches, at any vertical speed, with any flags bit set.
    // This is the whole of the old vacuity case, kept: it is the shipped answer for walking,
    // braking, falling and flinching, and the arm every other case in this file runs under.
    for (const float vz : { 0.f, 5000.f, -5000.f })
    {
        state.velocity = glm::vec3(0.f, 0.f, vz);
        for (const std::uint8_t flags : { std::uint8_t(0u), std::uint8_t(0xFFu) })
        {
            state.flags = flags;
            INFO("uncommitted vz=" << vz << " flags=0x" << std::hex << int(flags));
            REQUIRE_FALSE(movement::detachesFromSupport(state, movement::kWorldUp, false));
        }
    }

    // ARM 2 -- COMMITTED AND PURELY HORIZONTAL: the shipped knockback. `hitDirectionXY` is an XY
    // unit vector and the assignment writes the tangential channels only, so the vertical channel
    // is whatever the hover servo left there. On flat ground in the steady state that is exactly
    // zero, and `> 0.f` is STRICT, so a 5 m shove stays SUPPORTED and keeps its hover.
    state.flags = 0u;
    state.velocity = glm::vec3(2000.f, -2000.f, 0.f);
    REQUIRE_FALSE(movement::detachesFromSupport(state, movement::kWorldUp, true));

    // ...and a committed body still SETTLING downward does not detach either. Detaching on a
    // negative vertical channel would drop a knockback out of support on any tick the servo is
    // pulling it back to ride height.
    state.velocity = glm::vec3(2000.f, 0.f, -1.f);
    REQUIRE_FALSE(movement::detachesFromSupport(state, movement::kWorldUp, true));

    // ARM 3 -- COMMITTED AND RISING: the arm the fence was written for, and the one an authored
    // lift (a launcher, a ski jump) reaches without adding a single enumerator.
    state.velocity = glm::vec3(0.f, 0.f, 1.f);
    REQUIRE(movement::detachesFromSupport(state, movement::kWorldUp, true));
    state.velocity = glm::vec3(2000.f, 0.f, 2000.f);
    REQUIRE(movement::detachesFromSupport(state, movement::kWorldUp, true));

    // ⭐ THE DISCRIMINATOR. The same rising velocity, uncommitted, does NOT detach — which is what
    // makes arm 3 a statement about `committed` and not merely about the sign of `vz`. Without
    // this line a predicate that read `dot(velocity, up) > 0` alone would pass every arm above,
    // and `UpwardLaunchIsDampedNotOverwritten` — which seeds exactly this velocity while
    // uncommitted — would have started detaching silently.
    REQUIRE_FALSE(movement::detachesFromSupport(state, movement::kWorldUp, false));
}

// ===========================================================================
// STEP 5 -- THE BODY WRITES
// ===========================================================================

TEST_CASE("BrawlerMovement.DrivesBodyFalseSkipsAdapterWrites", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    REQUIRE_FALSE(rig.sd.drivesBody);

    for (std::uint32_t t = 1u; t <= 3u; ++t)
    {
        rig.tick(t, stick(1.f, 0.f));
        rig.engineStep();
    }

    // 1. NOTHING REACHED THE BODY, because this rig pins the passenger arm. ⚠ PRODUCTION NO
    //    LONGER TAKES THIS ARM: task 15 flipped `drivesBody` to `true` and retired the
    //    CharacterMovementComponent, so what this case guards is the FLAG's contract -- that
    //    `false` skips step 5's two adapter writes -- and not a shipped configuration.
    REQUIRE(rig.phys.setTransformCalls.empty());
    REQUIRE(rig.phys.setLinearVelocityCalls.empty());

    // 2. AND THE COMMAND MARKER IS CLEAR, which is what keeps step 6' from reading a
    //    push-out out of a position nobody commanded.
    REQUIRE_FALSE(rig.hasCommandBit());
    REQUIRE(rig.derived.lastPushOut == glm::vec3(0.f));

    // 3. THE STATE WAS STILL COMPUTED IN FULL -- that is the whole point of the flag.
    REQUIRE(rig.state.velocity.x > 0.f);
    REQUIRE(rig.support() == movement::SupportState::Supported);

    // CONTROL -- the same three ticks with the flag flipped DO write, so the assertions
    // above measure the flag rather than an inert rig.
    ScriptedRig driving;
    seatOnFlatGroundAtRideHeight(driving);
    driving.sd.drivesBody = true;
    for (std::uint32_t t = 1u; t <= 3u; ++t)
    {
        driving.tick(t, stick(1.f, 0.f));
        driving.engineStep();
    }
    REQUIRE(driving.phys.setTransformCalls.size() == 3u);
    REQUIRE(driving.phys.setLinearVelocityCalls.size() == 3u);
}

TEST_CASE("BrawlerMovement.WritesTransformAndVelocityEachTick", "[BrawlerMovement]")
{
    using namespace movementTests;

    // Revision 6, ruling #14(c): the sim owns {position, velocity} and RE-PLACES the body
    // every tick. This case is the direct statement of that, and it replaces the Backlog's
    // `DrivesBodyTrueWritesVelocityOnly` (revision 4) and revision 5's
    // `DrivesBodyTrueAppliesAccelerationOnly`.
    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.sd.drivesBody = true;

    for (std::uint32_t t = 1u; t <= 3u; ++t)
    {
        rig.tick(t, stick(1.f, 0.f));

        // BOTH HALVES, EVERY TICK, with the values the sim just computed.
        REQUIRE(rig.phys.setTransformCalls.size() == static_cast<std::size_t>(t));
        REQUIRE(rig.phys.setLinearVelocityCalls.size() == static_cast<std::size_t>(t));
        REQUIRE(glm::vec3(rig.phys.setTransformCalls.back().transform[3])
                == rig.state.bodyState.position);
        REQUIRE(rig.phys.setLinearVelocityCalls.back().value == rig.state.velocity);
        REQUIRE(rig.phys.setTransformCalls.back().bodyId == kBodyId);
        REQUIRE(rig.hasCommandBit());

        rig.engineStep();
    }

    // ⛔ AND NOTHING WENT THROUGH THE FORCE SEAM. Revision 5 would have driven the body
    // with `addBodyAcceleration`; revision 6 retired that in favour of re-placement, and
    // this is the assertion that keeps the retirement honest.
    REQUIRE(rig.phys.addAccelerationCalls.empty());
    REQUIRE(rig.phys.addVelocityChangeCalls.empty());

    // The pose is a pure translation -- rotation is locked at the descriptor, which is
    // also what makes `LinearBodyState`'s fabricated identity rotation true.
    REQUIRE(movement::PhysicsSetup::body.body.lockRotation);
    const glm::mat4& pose = rig.phys.setTransformCalls.back().transform;
    REQUIRE(glm::vec3(pose[0]) == glm::vec3(1.f, 0.f, 0.f));
    REQUIRE(glm::vec3(pose[1]) == glm::vec3(0.f, 1.f, 0.f));
    REQUIRE(glm::vec3(pose[2]) == glm::vec3(0.f, 0.f, 1.f));
}

// ===========================================================================
// STEP 6' -- THE ENGINE'S POSITIONAL PUSH-OUT
// ===========================================================================

TEST_CASE("BrawlerMovement.CapturedPushOutAdoptedIntoPosition", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr glm::vec3 kPushOut(3.f, 0.f, 0.f);

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.sd.drivesBody = true;

    // Tick 1 issues a command with zero velocity, so the whole of the next capture that
    // is not the command is the solver.
    rig.tick(1u);
    REQUIRE(rig.hasCommandBit());
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
    const glm::vec3 commanded = rig.state.bodyState.position;

    // THE SOLVER SEPARATED THE CAPSULE FROM SOMETHING, 3 cm along +X.
    rig.engineStepAndCapture(commanded + rig.state.velocity * kDt + kPushOut,
                             /*solverVelocity*/ glm::vec3(0.f));

    rig.tick(2u);

    // 1. STEP 6' RECOVERED IT EXACTLY -- `captured - (positionCmd + velocity * dt)`.
    INFO("lastPushOut=(" << rig.derived.lastPushOut.x << ", " << rig.derived.lastPushOut.y
         << ", " << rig.derived.lastPushOut.z << ")");
    REQUIRE(rig.derived.lastPushOut.x == Catch::Approx(kPushOut.x).margin(1e-4f));
    REQUIRE(rig.derived.lastPushOut.y == Catch::Approx(0.f).margin(1e-4f));
    REQUIRE(glm::length(rig.derived.lastPushOut) > movement::kPushOutEps);

    // 2. ⭐ AND THE ADOPTION IS STRUCTURAL: the sim did NOT fight the solver back. The
    //    position it carries -- and the pose it hands the body on this very tick -- is the
    //    pushed-out one.
    REQUIRE(rig.state.bodyState.position.x == Catch::Approx(commanded.x + kPushOut.x).margin(1e-4f));
    REQUIRE(glm::vec3(rig.phys.setTransformCalls.back().transform[3])
            == rig.state.bodyState.position);

    // CONTROL -- an UNCONTESTED tick. `captured` is exactly `positionCmd + velocity * dt`,
    // so the push-out is zero and no contact rule fires. Without this arm the case would
    // pass on a rig that reported a push-out unconditionally.
    ScriptedRig free;
    seatOnFlatGroundAtRideHeight(free);
    free.sd.drivesBody = true;
    free.tick(1u);
    const glm::vec3 freeCommanded = free.state.bodyState.position;
    free.engineStepAndCapture(freeCommanded + free.state.velocity * kDt, glm::vec3(0.f));
    free.tick(2u);
    REQUIRE(free.derived.lastPushOut == glm::vec3(0.f));
}

TEST_CASE("BrawlerMovement.WallPushOutZeroesIntoWallComponent", "[BrawlerMovement]")
{
    using namespace movementTests;

    // ⚠ NOT TASK 50'S CASE. `ReplayAfterAdoptionReproducesContactClamp` is about what a
    // CORRECTION restores; this is about the clamp itself, on a live tick, with no
    // correction anywhere in the rig.
    //
    // THE FIXTURE TRICK: `acceleration = 0` turns `moveTowards(current, target, 0)` into
    // the identity, so the model becomes a PASS-THROUGH and the post-clamp velocity is
    // directly observable at the end of the tick. Without it the model re-accelerates on
    // the same tick and the clamp can only be inferred.
    constexpr float kWallX = 0.f;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.sd.drivesBody   = true;
    rig.sd.acceleration = 0.f;
    rig.state.bodyState.position = glm::vec3(kWallX, 0.f, 96.f + rig.sd.rideHeight);
    rig.state.velocity = glm::vec3(100.f, 50.f, 0.f);

    rig.tick(1u, stick(1.f, 0.f));
    REQUIRE(rig.state.velocity == glm::vec3(100.f, 50.f, 0.f));   // the pass-through premise
    REQUIRE(rig.hasCommandBit());

    // THE SOLVER: integrate the command, then refuse to let X past the wall. Written from
    // first principles and deliberately NOT from step 6's own arithmetic.
    glm::vec3 solved = rig.state.bodyState.position + rig.state.velocity * kDt;
    if (solved.x > kWallX)
        solved.x = kWallX;
    rig.engineStepAndCapture(solved, /*solverVelocity*/ glm::vec3(0.f));

    rig.tick(2u, stick(1.f, 0.f));

    INFO("pushOut=(" << rig.derived.lastPushOut.x << ", " << rig.derived.lastPushOut.y
         << ", " << rig.derived.lastPushOut.z << ")  velocity=("
         << rig.state.velocity.x << ", " << rig.state.velocity.y << ", "
         << rig.state.velocity.z << ")");

    REQUIRE(glm::length(rig.derived.lastPushOut) > movement::kPushOutEps);
    REQUIRE(rig.derived.lastPushOut.x < 0.f);                    // it pushed back along -X

    // ⭐ THE ONE AUTHORED CONTACT RULE: kill the component driving into the obstacle, KEEP
    // THE REST. The +Y half is untouched, which is the fighting-game corner slide.
    REQUIRE(rig.state.velocity.x == Catch::Approx(0.f).margin(1e-3f));
    REQUIRE(rig.state.velocity.y == Catch::Approx(50.f).margin(1e-3f));

    // CONTROL -- the same tick with nothing in the way. Both components survive, so the
    // zero above is the wall and not the fixture.
    ScriptedRig open;
    seatOnFlatGroundAtRideHeight(open);
    open.sd.drivesBody   = true;
    open.sd.acceleration = 0.f;
    open.state.bodyState.position = glm::vec3(kWallX, 0.f, 96.f + open.sd.rideHeight);
    open.state.velocity = glm::vec3(100.f, 50.f, 0.f);
    open.tick(1u, stick(1.f, 0.f));
    open.engineStepAndCapture(open.state.bodyState.position + open.state.velocity * kDt,
                              glm::vec3(0.f));
    open.tick(2u, stick(1.f, 0.f));
    REQUIRE(open.derived.lastPushOut == glm::vec3(0.f));
    REQUIRE(open.state.velocity.x == Catch::Approx(100.f).margin(1e-3f));
    REQUIRE(open.state.velocity.y == Catch::Approx(50.f).margin(1e-3f));
}

// ---------------------------------------------------------------------------
// ⭐⭐ REVISION 6'S CENTRAL CLAIM.
//
// "The captured `bodyState.linearVelocity` is NEVER READ -- no contact impulse can enter
// simulation state." The honest way to assert a NEGATIVE is invariance: run the SAME tick
// twice, differing ONLY in the captured velocity, and require the resulting `State` to be
// byte-identical over the real serializer. A single perturbed run asserting "velocity is
// what I expected" would pass on a rig where the field never reached the sim at all.
//
// The control arm perturbs `bodyState.POSITION` by the same magnitude through the same
// bridge and requires the bytes to DIFFER -- which is what proves the rig can see a change
// arriving through `bodyState`, and therefore that the invariance above is a property of
// the model rather than of the harness.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.ContactImpulseNeverEntersVelocity", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr glm::vec3 kSolverVelocityA(0.f, 0.f, 0.f);
    constexpr glm::vec3 kSolverVelocityB(-777.f, 999.f, 555.f);
    constexpr glm::vec3 kPushOut(3.f, 0.f, 0.f);

    auto runTwoTicks = [](glm::vec3 solverVelocity, glm::vec3 extraPosition)
    {
        ScriptedRig rig;
        seatOnFlatGroundAtRideHeight(rig);
        rig.sd.drivesBody = true;
        rig.state.velocity = glm::vec3(40.f, 0.f, 0.f);

        rig.tick(1u, stick(1.f, 0.f));
        rig.engineStepAndCapture(
            rig.state.bodyState.position + rig.state.velocity * kDt + kPushOut + extraPosition,
            solverVelocity);
        rig.tick(2u, stick(1.f, 0.f));

        ProbeBuffer buffer;
        writeToSyncedBuffer(rig.state, buffer, 0u);
        return buffer.bytes;
    };

    const std::vector<std::uint8_t> quiet =
        runTwoTicks(kSolverVelocityA, glm::vec3(0.f));
    const std::vector<std::uint8_t> perturbedVelocity =
        runTwoTicks(kSolverVelocityB, glm::vec3(0.f));
    const std::vector<std::uint8_t> perturbedPosition =
        runTwoTicks(kSolverVelocityA, glm::vec3(0.f, 0.f, 1.f));

    // ⚠⚠ THE ONE SPAN THAT IS EXPECTED TO MOVE, AND WHY IT IS EXCLUDED RATHER THAN
    // ASSERTED AWAY. `bodyState` is the FIRST entry of `SerializableFields<State>` and
    // `LinearBodyState` serializes { position, linearVelocity }, so bytes
    // [12, 24) ARE the captured velocity, verbatim, on the wire. It is CAPTURED and
    // CARRIED -- ruling #17(b) took that cost deliberately, and the header calls those 12
    // bytes dead weight. The claim under test is not that the field is absent; it is that
    // NOTHING ELSE MOVES WITH IT. So the comparison runs over every OTHER byte, and the
    // excluded span is asserted to move, which is what proves the perturbation arrived.
    constexpr std::size_t kCapturedVelocityBegin = sizeof(glm::vec3);
    constexpr std::size_t kCapturedVelocityEnd   = syncSize<LinearBodyState>();
    const std::size_t     kStateEnd              = syncSize<movement::State>();
    static_assert(kCapturedVelocityEnd - kCapturedVelocityBegin == sizeof(glm::vec3));

    auto sameOutsideCapturedVelocity = [&](const std::vector<std::uint8_t>& a,
                                           const std::vector<std::uint8_t>& b)
    {
        for (std::size_t i = 0; i < kStateEnd; ++i)
        {
            if (i >= kCapturedVelocityBegin && i < kCapturedVelocityEnd)
                continue;
            if (a[i] != b[i])
            {
                INFO("first differing byte at offset " << i);
                return false;
            }
        }
        return true;
    };

    // PREMISE 1 -- the perturbed velocity really did reach `State::bodyState` through the
    // shipped narrowing bridge, rather than being dropped on the way in.
    {
        ScriptedRig witness;
        seatOnFlatGroundAtRideHeight(witness);
        witness.engineStepAndCapture(witness.state.bodyState.position, kSolverVelocityB);
        REQUIRE(witness.state.bodyState.linearVelocity == kSolverVelocityB);
    }

    // PREMISE 2 -- and it reached THE WIRE, in the span excluded below. Without this the
    // exclusion could be hiding the only place the perturbation ever showed up.
    REQUIRE_FALSE(std::memcmp(perturbedVelocity.data() + kCapturedVelocityBegin,
                              quiet.data() + kCapturedVelocityBegin,
                              kCapturedVelocityEnd - kCapturedVelocityBegin) == 0);

    // ⭐ THE ASSERTION. Every byte of `State` that the SIMULATION owns -- `velocity`,
    // `committedStepDir`, `stepStartTick`, `flags`, `positionCmd` and the adopted
    // `bodyState.position` -- is INVARIANT under a wildly perturbed contact impulse.
    REQUIRE(sameOutsideCapturedVelocity(perturbedVelocity, quiet));

    // ⭐ THE CONTROL. A change arriving through the SAME member, on the SAME bridge, in
    // a field the sim DOES read, moves those same bytes. This is what makes the invariance
    // above a property of the model rather than of a harness that cannot see anything.
    REQUIRE_FALSE(sameOutsideCapturedVelocity(perturbedPosition, quiet));
}

// ===========================================================================
// STEP 3 -- THE CADENCE MODEL
// ===========================================================================

TEST_CASE("BrawlerMovement.CadenceCommitsOnlyAtBoundary", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.sd.model = movement::MovementModel::Cadence;
    REQUIRE(rig.state.stepStartTick == 0u);

    std::vector<std::uint32_t> commitTicks;
    for (std::uint32_t t = 0u; t <= 44u; ++t)
    {
        const std::uint32_t before = rig.state.stepStartTick;
        rig.tick(t, stick(1.f, 0.f));
        if (rig.state.stepStartTick != before)
            commitTicks.push_back(t);
    }

    INFO("stepPeriodTicks=" << rig.sd.stepPeriodTicks << ", commits at "
         << commitTicks.size() << " ticks");
    // The direction is COMMITTED at a period boundary and NOWHERE else, which is the
    // whole "steps, not a joystick" feel.
    REQUIRE(commitTicks == std::vector<std::uint32_t>{ 20u, 40u });
    REQUIRE(rig.state.committedStepDir == glm::vec2(1.f, 0.f));
}

TEST_CASE("BrawlerMovement.CadenceHoldsBetweenCommits", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.sd.model = movement::MovementModel::Cadence;

    for (std::uint32_t t = 0u; t < rig.sd.stepPeriodTicks; ++t)
        rig.tick(t, stick(1.f, 0.f));

    // The commit boundary.
    rig.tick(rig.sd.stepPeriodTicks, stick(1.f, 0.f));
    REQUIRE(rig.state.committedStepDir == glm::vec2(1.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.stepSpeed).margin(1e-4f));

    // ⭐ NOW FLICK THE STICK THE OTHER WAY FOR THE WHOLE STEP. A mid-step flick must not
    // steer the character; both the committed direction AND the world velocity are held.
    for (std::uint32_t t = rig.sd.stepPeriodTicks + 1u; t < 2u * rig.sd.stepPeriodTicks; ++t)
    {
        rig.tick(t, stick(-1.f, 0.f));
        INFO("tick " << t << " dir=(" << rig.state.committedStepDir.x << ", "
             << rig.state.committedStepDir.y << ") vx=" << rig.state.velocity.x);
        REQUIRE(rig.state.committedStepDir == glm::vec2(1.f, 0.f));
        REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.stepSpeed).margin(1e-4f));
        REQUIRE(rig.state.stepStartTick == rig.sd.stepPeriodTicks);
    }

    // ...and the NEXT boundary is where the flick finally lands.
    rig.tick(2u * rig.sd.stepPeriodTicks, stick(-1.f, 0.f));
    REQUIRE(rig.state.committedStepDir == glm::vec2(-1.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(-rig.sd.stepSpeed).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.CadenceFreezeEatsStepKeepsTimer", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.sd.model = movement::MovementModel::Cadence;

    for (std::uint32_t t = 0u; t < rig.sd.stepPeriodTicks; ++t)
        rig.tick(t, stick(1.f, 0.f));
    REQUIRE(rig.state.stepStartTick == 0u);
    REQUIRE(rig.state.committedStepDir == glm::vec2(0.f));

    // THE BOUNDARY TICK, FROZEN. Step 1 gates before step 3, so the model is not called:
    // the step is EATEN (no motion, no commit)...
    rig.tick(rig.sd.stepPeriodTicks, stick(1.f, 0.f), movement::kInputFlagHoldGuard);
    REQUIRE(rig.frozenBit());
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
    REQUIRE(rig.state.committedStepDir == glm::vec2(0.f));
    // ⭐ ...BUT THE TIMER IS NOT RESET. `stepStartTick` is only stamped by a commit, so a
    // freeze cannot push the cadence off the beat.
    REQUIRE(rig.state.stepStartTick == 0u);

    // ⇒ THE VERY NEXT LIVE TICK COMMITS, because the boundary was never consumed. A rig
    //   that had reset the timer would stand still for another full period here.
    rig.tick(rig.sd.stepPeriodTicks + 1u, stick(1.f, 0.f));
    REQUIRE(rig.state.committedStepDir == glm::vec2(1.f, 0.f));
    REQUIRE(rig.state.stepStartTick == rig.sd.stepPeriodTicks + 1u);
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.sd.stepSpeed).margin(1e-4f));
}

TEST_CASE("BrawlerMovement.CadenceNeutralAtCommitStands", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.sd.model = movement::MovementModel::Cadence;

    for (std::uint32_t t = 0u; t < rig.sd.stepPeriodTicks; ++t)
        rig.tick(t, stick(1.f, 0.f));

    // A NEUTRAL STICK AT THE BOUNDARY commits a STAND -- a zero direction, not "no commit".
    rig.tick(rig.sd.stepPeriodTicks, glm::vec3(0.f));
    REQUIRE(rig.state.committedStepDir == glm::vec2(0.f));
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
    // ⭐ AND THE TIMER RESTARTED. The stand CONSUMED the boundary; the cadence stays on the
    // beat instead of firing again on the next tick. This is the exact opposite of
    // `CadenceFreezeEatsStepKeepsTimer`, and the pair is what pins which one a stand is.
    REQUIRE(rig.state.stepStartTick == rig.sd.stepPeriodTicks);

    rig.tick(rig.sd.stepPeriodTicks + 1u, stick(1.f, 0.f));
    REQUIRE(rig.state.committedStepDir == glm::vec2(0.f));
    REQUIRE(rig.state.stepStartTick == rig.sd.stepPeriodTicks);
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
}

// ===========================================================================
// THE WIRE, AND DETERMINISM
// ===========================================================================

TEST_CASE("BrawlerMovement.ModelSwitchDoesNotChangeWireLayout", "[BrawlerMovement]")
{
    using namespace movementTests;

    // ⚠ THE ABSOLUTE SIZES ARE PINNED ELSEWHERE, ON PURPOSE.
    // `DAttack.SimulatableBrawler.WireFootprint` carries
    // `syncSize<brawlerMovementSimulation::State>() == 61u` and
    // `syncSize<...::InitialConditions>() == 16u`; restating either number here would
    // create a second place to forget. What is asserted below is the property that fence
    // cannot see: that the layout is the SAME under both movement models.
    auto runUnder = [](movement::MovementModel model)
    {
        ScriptedRig rig;
        seatOnFlatGroundAtRideHeight(rig);
        rig.sd.model = model;
        for (std::uint32_t t = 0u; t <= 25u; ++t)
        {
            rig.tick(t, stick(1.f, 0.f));
            rig.engineStep();
        }
        // `writeToSyncedBuffer` returns the number of BYTES WRITTEN; written from
        // offset 0, that is also where the slice ends.
        ProbeBuffer buffer;
        const std::uint32_t written = writeToSyncedBuffer(rig.state, buffer, 0u);
        return std::pair<std::uint32_t, std::vector<std::uint8_t>>{ written, buffer.bytes };
    };

    const auto continuous = runUnder(movement::MovementModel::ContinuousAccelBrake);
    const auto cadence    = runUnder(movement::MovementModel::Cadence);

    // 1. THE LAYOUT. Same end offset, and it is the slice's own `syncSize` -- so neither
    //    model wrote a byte more or fewer than the wire says the slice is.
    REQUIRE(continuous.first == cadence.first);
    REQUIRE(continuous.first == syncSize<movement::State>());

    // 2. NOTHING RAN PAST THE SLICE. The buffer is poisoned, so an over-write would show.
    for (std::size_t i = continuous.first; i < continuous.second.size(); ++i)
    {
        REQUIRE(continuous.second[i] == 0xCDu);
        REQUIRE(cadence.second[i] == 0xCDu);
    }

    // 3. THE PREMISE THAT MAKES (1) MEAN SOMETHING: the two models genuinely produced
    //    DIFFERENT state. If they had not, "the layout did not change" would be the empty
    //    observation that nothing changed at all.
    REQUIRE_FALSE(continuous.second == cadence.second);

    // 4. And specifically: `committedStepDir` / `stepStartTick` are Cadence's own slice and
    //    are zero-and-untouched under the continuous model, yet they occupy their wire
    //    bytes either way. That is what "the layout does not change with `sd.model`" means.
    ScriptedRig probe;
    probe.sd.model = movement::MovementModel::ContinuousAccelBrake;
    REQUIRE(probe.state.committedStepDir == glm::vec2(0.f));
}

TEST_CASE("BrawlerMovement.DeterministicReplay", "[BrawlerMovement]")
{
    using namespace movementTests;

    // A 120-tick script, run twice from identical seeds, compared on the SERIALIZED BYTES
    // of both wire types. Bytes rather than fields, because the wire is what a peer
    // actually replays from and a field-by-field comparison would silently skip anything
    // the serializer does not carry.
    constexpr int kTicks = 120;

    auto runScript = []()
    {
        PlaneRig rig;
        rig.sd.drivesBody = true;
        rig.state.bodyState.position =
            glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight + 3.f));

        for (int i = 0; i < kTicks; ++i)
        {
            const auto t = static_cast<std::uint32_t>(i);

            // A script with every branch in it: a turning stick, periodic freezes, a
            // ledge, a fall off the end of the world, and a respawn.
            const float phase = static_cast<float>(i) * 0.05f;
            const glm::vec3 stickWorld(glm::cos(phase), glm::sin(phase), 0.f);
            const std::uint8_t inputFlags =
                (i % 17 == 0) ? movement::kInputFlagHoldGuard : std::uint8_t{ 0u };

            if (i == 30) rig.query.planeOffset += 9.f;         // a ledge
            if (i == 55) rig.query.planeOffset -= 400.f;       // the floor drops away
            if (i == 80) { rig.ic.teleportPending = 1u; rig.ic.teleportPos = glm::vec3(5.f, -7.f, 130.f); }
            if (i == 90) rig.query.planeOffset += 400.f;       // and comes back

            rig.machineState.m_currentState = (i % 23 == 0)
                ? DAttackState::HitFlinch
                : DAttackState::Idle;

            rig.tick(t, stickWorld, inputFlags);
            rig.engineStep();
        }

        // ⚠ `writeToSyncedBuffer` returns the number of BYTES WRITTEN, not the end
        // offset -- it ends `return off - offset;`. Both slices are laid down
        // back to back, so the running offset is the accumulated count.
        ProbeBuffer buffer;
        std::uint32_t off = 0u;
        off += writeToSyncedBuffer(rig.state, buffer, off);
        off += writeToSyncedBuffer(rig.ic, buffer, off);
        return std::pair<std::uint32_t, std::vector<std::uint8_t>>{ off, buffer.bytes };
    };

    const auto first  = runScript();
    const auto second = runScript();

    REQUIRE(first.first == syncSize<movement::State>()
                         + syncSize<movement::InitialConditions>());

    // ANTI-VACUITY -- the script actually moved the state. A rig that never ran would also
    // replay bit-identically.
    ProbeBuffer seeded;
    {
        movement::State fresh;
        writeToSyncedBuffer(fresh, seeded, 0u);
    }
    REQUIRE_FALSE(std::memcmp(first.second.data(), seeded.bytes.data(),
                              syncSize<movement::State>()) == 0);

    INFO("compared " << first.first << " serialized bytes over " << kTicks << " ticks");
    REQUIRE(std::memcmp(first.second.data(), second.second.data(), first.first) == 0);
    REQUIRE(first.second == second.second);
}

// ===========================================================================
// TASK 54 -- THE WALL-PRESS LIMIT CYCLE, AND THE GROUND PROBE'S GEOMETRY
//
// The defect, MEASURED in task 54 Phase 1 (`impl/impl_notes_seam_54.md`) and observed by the
// user in PIE: driving into a static wall made the character sink into the floor and then
// launch, repeatedly -- 755 `[Movement.surface]` transitions in one session, every single
// `-> Airborne` line reporting `clearance=0.000`.
//
// THE MECHANISM IS IN STEP 2, NOT IN STEP 6' AND NOT IN THE SERVO. The attachment probe
// used to be a capsule the SAME SIZE as the body at the body's OWN pose. A body pressed
// against a wall therefore made the PROBE overlap that wall, and
// `ChaosSpatialQueryAdapter::sweep` sets `bFindInitialOverlaps = true` and then takes the
// MINIMUM-`Time` blocking hit -- so the wall arrived at `Time == 0` and BEAT the floor. Its
// normal is horizontal, `walkable` was false, and the character read `Airborne` with
// `clearance == 0` while standing on the floor.
//
// WHY THE EXISTING CASES COULD NOT SEE IT, and it is worth stating: every mock above
// answers with ONE solid. `MockSpatialQueryAdapter` answers from a script;
// `PlaneGroundQueryAdapter` models a single plane and hardcodes a BODY-SIZED probe. Neither
// can express "two solids, one probe", which is the whole of the defect.
// `WorldSweepQueryAdapter` below is the one that can, and it differs from both in the one way
// that matters: it takes the probe's geometry FROM THE SHIPPED DESCRIPTOR instead of
// restating it, so a change to `PhysicsSetup::queryVolumes` reaches these cases through
// exactly the channel it reaches production through.
// ===========================================================================

namespace movementTests
{

// ---------------------------------------------------------------------------
// A WORLD OF ANALYTIC HALF-SPACES, SWEPT BY THE DESCRIBED PROBE.
//
// Geometry, from first principles and quoting no part of the sub-simulation:
//   * a half-space is { p : dot(n, p) - offset >= 0 }; the solid is on the other side;
//   * an upright capsule of radius r and TOTAL half-height hh (centre to tip -- see the
//     convention note below) has cylinder-axis endpoints at c +/- (0, 0, hh - r), and its
//     signed gap to the plane is min over those two endpoints of dot(n, e) - offset - r;
//   * moving along a unit direction d changes that gap at rate dot(n, d), so contact happens
//     at travel = gap / -dot(n, d).
//
// THE HALF-HEIGHT CONVENTION IS *TOTAL* (centre to tip, hemisphere included), and it is
// established rather than assumed -- the offset arithmetic below is wrong under the other
// reading. `CapsuleGeometry` carries no comment of its own, so it is fixed by its two
// consumers, both of which hand the field straight to an engine API whose convention Epic
// documents:
//   * the PROBE path: `ChaosSpatialQueryAdapter::registerVolume` calls
//     `FCollisionShape::MakeCapsule(geo.radius, geo.halfHeight)`, and UE 5.6's
//     `CollisionShape.h` says of it: "Note: This is the full half-height (needs to include
//     the sphere radius)".
//   * the BODY path: `ChaosPhysicsFactory` calls `UCapsuleComponent::InitCapsuleSize(radius,
//     halfHeight)` and, on the adopt path, `checkf`s the descriptor against
//     `GetUnscaledCapsuleHalfHeight()`. UE 5.6's `CapsuleComponent.h` documents that field as
//     "Half-height, from center of capsule to the end of top or bottom hemisphere."
// Both APIs agree, which matters: they are different engine types and either could have
// differed. So the shipped `{42, 96}` capsule's bottom tip is at `centre.z - 96`.
//
// THE SELECTION RULE IS A TRANSCRIPTION, NOT A HYPOTHESIS. It is copied from
// `Source/OGSimulationUnreal/ChaosSpatialQueryAdapter.cpp::sweep`:
//     :543-544  `localParams.bFindInitialOverlaps = true;`
//     :562-563  "An initial overlap comes back with Time == 0, so it naturally wins."
//     :564-571  the minimum-`Time`-over-blocking-hits loop
//     :503      `worldMat = transform * vol.offsetTransform`  -- the probe's own offset
// A mock that decided the winner by its own rule would make every case below circular.
// ---------------------------------------------------------------------------
struct WorldSweepQueryAdapter
{
    struct Plane
    {
        glm::vec3   n{ 0.f, 0.f, 1.f };
        float       offset = 0.f;
        const char* name   = "";
    };

    // Every solid the SOLVER separates the body from. The probe searches the same list; the
    // `world`/`character` distinction is not the subject here (both a wall and a floor are
    // `world`, which is exactly why the probe cannot tell them apart by category).
    std::vector<Plane> solids;

    // THE PROBE, read from the shipped descriptor rather than restated.
    float     probeRadius     = 0.f;
    float     probeHalfHeight = 0.f;
    glm::vec3 probeOffset{ 0.f };
    // THE BODY, for the solver and for the independent clearance cross-check.
    float bodyRadius     = 0.f;
    float bodyHalfHeight = 0.f;

    // Recorded per call, so a case reads exactly what the sub-simulation consumed.
    bool        lastBlocked          = false;
    float       lastClearance        = 0.f;
    glm::vec3   lastNormal{ 0.f };
    bool        lastStartPenetrating = false;
    const char* lastHitName          = "(none)";

    void configureFrom(const movement::StaticData& sd)
    {
        const std::vector<QueryVolumeDescriptor> volumes = movement::PhysicsSetup::queryVolumes(sd);
        REQUIRE(volumes.size() == 1u);
        const CapsuleGeometry* capsule = std::get_if<CapsuleGeometry>(&volumes[0].geometry);
        REQUIRE(capsule != nullptr);
        probeRadius     = capsule->radius;
        probeHalfHeight = capsule->halfHeight;
        probeOffset     = glm::vec3(volumes[0].offsetTransform[3]);
        bodyRadius      = sd.capsuleRadius;
        bodyHalfHeight  = sd.capsuleHalfHeight;
    }

    static float capsuleGap(const Plane& plane, const glm::vec3& centre, float r, float hh)
    {
        const glm::vec3 axis(0.f, 0.f, hh - r);
        const float a = glm::dot(plane.n, centre + axis) - plane.offset;
        const float b = glm::dot(plane.n, centre - axis) - plane.offset;
        return glm::min(a, b) - r;
    }

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& pose, const glm::vec3& delta)
    {
        SweepHit hit{};
        if (volumeId != kVolumeId)
            return hit;

        const float length = glm::length(delta);
        if (length <= 0.f)
            return hit;
        const glm::vec3 dir = delta / length;

        // The adapter poses the volume at `transform * offsetTransform`, so the probe's
        // centre is the BODY's pose displaced by the descriptor's own offset.
        const glm::vec3 centre = glm::vec3(pose[3]) + probeOffset;

        int   best      = -1;
        float bestTime  = 0.f;
        float bestPen   = 0.f;
        bool  bestStart = false;

        for (std::size_t i = 0; i < solids.size(); ++i)
        {
            const float gap = capsuleGap(solids[i], centre, probeRadius, probeHalfHeight);

            float time  = 0.f;
            float pen   = 0.f;
            bool  start = false;

            if (gap <= 0.f)
            {
                // ALREADY OVERLAPPING AT FRACTION 0 -- `bFindInitialOverlaps` makes this a
                // blocking result at `Time == 0`, which is why it wins the minimum below.
                pen   = -gap;
                start = true;
            }
            else
            {
                const float rate = -glm::dot(solids[i].n, dir);
                if (rate <= 0.f)
                    continue;                        // parallel to it, or moving away
                const float travel = gap / rate;
                if (travel > length)
                    continue;                        // out of reach along this delta
                time = travel / length;
            }

            if (best < 0 || time < bestTime)
            {
                best      = static_cast<int>(i);
                bestTime  = time;
                bestPen   = pen;
                bestStart = start;
            }
        }

        if (best < 0)
        {
            lastBlocked          = false;
            lastClearance        = length;
            lastNormal           = glm::vec3(0.f);
            lastStartPenetrating = false;
            lastHitName          = "(miss)";
            return hit;                              // defaults: blocked == false, fraction == 1
        }

        hit.blocked          = true;
        hit.fraction         = bestTime;
        hit.normal           = solids[best].n;
        hit.impactPoint      = centre + dir * (bestTime * length);
        hit.startPenetrating = bestStart;
        hit.penetrationDepth = bestPen;

        lastBlocked          = true;
        lastClearance        = bestTime * length;
        lastNormal           = hit.normal;
        lastStartPenetrating = bestStart;
        lastHitName          = solids[best].name;
        return hit;
    }

    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}

    // THE SOLVER: separate the BODY from every half-space it penetrates. The correction is
    // the SUM of the per-contact push-outs, which is what a simultaneous constraint solve
    // produces and what step 6' reads back as `pushOut`.
    glm::vec3 depenetrate(glm::vec3 p) const
    {
        glm::vec3 correction(0.f);
        for (const Plane& solid : solids)
        {
            const float gap = capsuleGap(solid, p, bodyRadius, bodyHalfHeight);
            if (gap < 0.f)
                correction += solid.n * (-gap);
        }
        return p + correction;
    }

    // The clearance the BODY actually has along -Z, computed from the body's own geometry.
    // Independent of the probe descriptor, which is the whole point: it is what the probe's
    // answer is checked AGAINST.
    float trueBodyClearance(const glm::vec3& centre) const
    {
        float nearest = 1e9f;
        for (const Plane& solid : solids)
        {
            const float rate = solid.n.z;            // -dot(n, (0,0,-1))
            if (rate <= 0.f)
                continue;
            const float gap = capsuleGap(solid, centre, bodyRadius, bodyHalfHeight);
            nearest = glm::min(nearest, gap / rate);
        }
        return nearest;
    }
};

static_assert(SpatialQueryAdapter<WorldSweepQueryAdapter>);

using WorldRig = Rig<WorldSweepQueryAdapter>;

inline WorldSweepQueryAdapter::Plane analyticFloor()
{
    return WorldSweepQueryAdapter::Plane{ glm::vec3(0.f, 0.f, 1.f), 0.f, "floor" };
}
// Solid at x >= wallX:  dot(n, p) - offset == wallX - p.x.
inline WorldSweepQueryAdapter::Plane analyticWall(float wallX)
{
    return WorldSweepQueryAdapter::Plane{ glm::vec3(-1.f, 0.f, 0.f), -wallX, "wall" };
}

} // namespace movementTests

// ---------------------------------------------------------------------------
// THE DEFECT, STATED DIRECTLY: a character walking into a wall must stay on the floor.
//
// RED BEFORE THE FIX -- with the body-sized probe this case reported hundreds of ticks
// Airborne at `clearance == 0` and a `pos.z` excursion from 96 to 181 cm.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.WallPressKeepsTheCharacterOnTheFloor", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr float kWallX = 0.f;
    constexpr int   kTicks = 300;

    WorldRig rig;
    rig.sd.drivesBody = true;                  // the wall press needs the body driven and step 6' live
    rig.query.configureFrom(rig.sd);
    rig.query.solids = { analyticFloor(), analyticWall(kWallX) };

    // Hover steady state, 40 cm short of contact. The body's bottom tip is at
    // `centre.z - capsuleHalfHeight` under the TOTAL half-height convention established above.
    const float seatZ = rig.sd.capsuleHalfHeight + rig.sd.rideHeight;
    rig.state.bodyState.position = glm::vec3(kWallX - rig.sd.capsuleRadius - 40.f, 0.f, seatZ);
    rig.state.positionCmd        = rig.state.bodyState.position;

    const float contactX = kWallX - rig.sd.capsuleRadius;

    int   transitionsAfterContact  = 0;
    int   pressedTicks             = 0;
    int   floorTicksWhilePressed   = 0;
    float minZWhilePressed         =  1e9f;
    float maxZWhilePressed         = -1e9f;
    float minClearanceWhilePressed =  1e9f;
    float maxIntoWallVelocity      = 0.f;
    float maxPushOutMagnitude      = 0.f;
    bool  everTouchedTheWall       = false;

    std::uint8_t previousKind = static_cast<std::uint8_t>(rig.support());

    for (int t = 0; t < kTicks; ++t)
    {
        const glm::vec3 velocityBefore = rig.state.velocity;

        rig.tick(static_cast<std::uint32_t>(t), stick(1.f, 0.f));

        const std::uint8_t kind = static_cast<std::uint8_t>(rig.support());
        // "Pressed" == the solver has the body against the wall. Measured from POSITION, not
        // from the surface kind, so the window cannot be defined by the thing under test.
        const bool pressed = rig.state.bodyState.position.x >= contactX - 1e-3f;
        if (pressed)
        {
            everTouchedTheWall = true;
            ++pressedTicks;
            if (kind != previousKind)
                ++transitionsAfterContact;
            if (kind == static_cast<std::uint8_t>(movement::SupportState::Supported))
                ++floorTicksWhilePressed;
            minZWhilePressed         = glm::min(minZWhilePressed, rig.state.bodyState.position.z);
            maxZWhilePressed         = glm::max(maxZWhilePressed, rig.state.bodyState.position.z);
            minClearanceWhilePressed = glm::min(minClearanceWhilePressed, rig.query.lastClearance);
            maxIntoWallVelocity      = glm::max(maxIntoWallVelocity, velocityBefore.x);
            maxPushOutMagnitude      = glm::max(maxPushOutMagnitude,
                                                glm::length(rig.derived.lastPushOut));
        }
        previousKind = kind;

        // THE ENGINE STEP: integrate the command we just handed it, then separate.
        rig.state.bodyState.position =
            rig.query.depenetrate(rig.state.bodyState.position + rig.state.velocity * kDt);
    }

    INFO("wall press: pressedTicks=" << pressedTicks
         << "  transitionsAfterContact=" << transitionsAfterContact
         << "  floorTicksWhilePressed=" << floorTicksWhilePressed
         << "  pos.z in [" << minZWhilePressed << " .. " << maxZWhilePressed << "]"
         << "  minClearance=" << minClearanceWhilePressed
         << "  lastProbeHit=" << rig.query.lastHitName
         << "  maxPushOut=" << maxPushOutMagnitude);

    // ---- THE DISCRIMINATOR ROWS, FIRST. A transition count of zero is worthless unless the
    //      press actually happened, and "green because nothing ran" is the failure mode this
    //      initiative keeps finding. Every row here is GREEN in the RED run too.
    REQUIRE(everTouchedTheWall);                                  // the body reached the wall
    REQUIRE(pressedTicks > 200);                                  // and stayed there
    REQUIRE(rig.state.bodyState.position.x == Catch::Approx(contactX).margin(1e-3f));
    REQUIRE(maxIntoWallVelocity > 0.f);                           // it was DRIVING into it
    REQUIRE(maxPushOutMagnitude > movement::kPushOutEps);         // the solver pushed back
    REQUIRE(rig.hasCommandBit());                                 // step 6' was un-gated

    // ---- THE SUBJECT. The wall is lateral; nothing about touching it may change what the
    //      character is standing on, how high it hovers, or which surface it reports.
    REQUIRE(transitionsAfterContact == 0);
    REQUIRE(floorTicksWhilePressed == pressedTicks);
    REQUIRE(minClearanceWhilePressed == Catch::Approx(rig.sd.rideHeight).margin(1e-3f));
    REQUIRE(minZWhilePressed == Catch::Approx(seatZ).margin(1e-3f));
    REQUIRE(maxZWhilePressed == Catch::Approx(seatZ).margin(1e-3f));

    // ---- AND THE ONE AUTHORED CONTACT RULE STILL FIRES. The fix is to the PROBE, not to
    //      step 6', so the corner clamp must be exactly as it was.
    //
    //      ⚠ THE OBSERVABLE IS "DOES NOT ACCUMULATE", NOT "IS ZERO", and the difference is
    //      the tick order rather than a weaker assertion. Step 6' zeroes the into-wall
    //      component at the TOP of the tick and step 3's model then re-accelerates by
    //      `acceleration * dt` in the SAME tick, so the end-of-tick value is one tick of
    //      acceleration. `WallPushOutZeroesIntoWallComponent` reads the post-clamp velocity
    //      directly by setting `acceleration = 0`; this case cannot -- it needs a real walk to
    //      reach the wall at all. What a BROKEN clamp would look like here is the value
    //      climbing to `maxWalkSpeed` and staying there, which this row excludes.
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.accelPerTick()).margin(1e-3f));
    REQUIRE(rig.state.velocity.x < rig.sd.maxWalkSpeed);
    //      ...and 275 ticks of driving into it moved the body not one cm past contact.
    REQUIRE(maxIntoWallVelocity <= rig.sd.maxWalkSpeed);
}

// ---------------------------------------------------------------------------
// THE OFFSET'S OWN TEST, and it is the reason the descriptor carries an offset at all:
// shrinking the probe must not move what it measures.
//
// On FLAT GROUND the probe's answer must equal the BODY's true clearance EXACTLY -- the two
// are computed from different geometry (descriptor vs. `StaticData`) by different code paths,
// so agreement is a statement and not a tautology. GREEN before and after the fix: that is
// what makes it the guard rather than the subject.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.GroundProbeMeasuresTheBodysClearanceOnFlatGround", "[BrawlerMovement]")
{
    using namespace movementTests;

    for (const float seededClearance : { 3.f, 10.f, 25.f })
    {
        WorldRig rig;
        rig.query.configureFrom(rig.sd);
        rig.query.solids = { analyticFloor() };

        rig.state.bodyState.position =
            glm::vec3(0.f, 0.f, rig.sd.capsuleHalfHeight + seededClearance);
        rig.tick(1u);

        const float trueClearance = rig.query.trueBodyClearance(rig.state.bodyState.position);

        INFO("seeded=" << seededClearance << "  probe=" << rig.query.lastClearance
             << "  body=" << trueClearance
             << "  probeRadius=" << rig.query.probeRadius
             << "  probeHalfHeight=" << rig.query.probeHalfHeight
             << "  probeOffset.z=" << rig.query.probeOffset.z);

        REQUIRE(rig.query.lastBlocked);
        REQUIRE(trueClearance == Catch::Approx(seededClearance).margin(1e-4f));
        REQUIRE(rig.query.lastClearance == Catch::Approx(trueClearance).margin(1e-4f));
    }
}

// ---------------------------------------------------------------------------
// AND WHAT IT COSTS ON A SLOPE, PINNED AS A CLOSED FORM RATHER THAN LEFT IMPLICIT.
//
// A probe whose bottom TIP is pinned to the body's but whose radius is smaller by `s` does
// NOT read the same clearance on a tilted plane. On flat ground the tip IS the contact point,
// so nothing moves; on a slope the contact point sits on the hemisphere, and a smaller
// hemisphere with the same tip is slightly further from the plane. Derivation:
//
//   gap(probe) - gap(body) = n.z * offset.z + (bodyRadius - probeRadius)
// and with the shipped `offset.z == -(bodyRadius - probeRadius) == -s` that is `s * (1 - n.z)`.
// Clearance is `gap / n.z`, so
//
//   clearanceBias = s * (1/cos(theta) - 1) = s * (sec(theta) - 1)
//
// which is 0 on flat ground, `0.155 * s` cm at 30 deg and `0.414 * s` cm at the shipped 45 deg
// `maxSlopeAngleDeg` cap. The character therefore settles that much LOWER on a slope. It is
// second-order against the `rideHeight * cos(theta)` perpendicular gap the file already
// documents (7.07 cm at 45 deg), it is one-way, and it is the price of option 1.
// It is NOT removable by a different offset: zeroing it needs `offset.z = -s / n.z`, which
// is slope-dependent and a static descriptor cannot express it.
//
// GREEN both before and after the fix -- before, `s` is 0 and the closed form predicts 0 -- so
// this is a LAW, not a snapshot of one build.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.GroundProbeSlopeBiasIsTheShrinkTerm", "[BrawlerMovement]")
{
    using namespace movementTests;

    for (const float slopeDeg : { 0.f, 30.f, 45.f })
    {
        WorldRig rig;
        rig.query.configureFrom(rig.sd);
        const glm::vec3 n = slopeNormal(slopeDeg);
        rig.query.solids = { WorldSweepQueryAdapter::Plane{ n, 0.f, "ramp" } };

        // `s` is read from the SHIPPED descriptor, never restated here.
        const float s = rig.sd.capsuleRadius - rig.query.probeRadius;
        REQUIRE(s >= 0.f);

        // Seat the BODY at a known true clearance by inverting its OWN gap equation at
        // x == y == 0, for a plane through the origin:
        //     gap       = n.z * (z - axisHalf) - r
        //     clearance = gap / n.z            (the sweep is straight down, rate == n.z)
        //  => z         = axisHalf + clearance + r / n.z
        // the same inversion `seedZForClearance` above performs. The premise row below
        // is what fences it: a mis-seeded ramp reads 2.16 cm instead of 10 and would have
        // made the bias comparison meaningless without ever failing on the bias itself.
        constexpr float kSeeded = 10.f;
        const float axisHalf = rig.sd.capsuleHalfHeight - rig.sd.capsuleRadius;
        rig.state.bodyState.position = glm::vec3(
            0.f, 0.f, axisHalf + kSeeded + rig.sd.capsuleRadius / n.z);

        rig.tick(1u);

        const float trueClearance = rig.query.trueBodyClearance(rig.state.bodyState.position);
        const float predictedBias = s * (1.f / n.z - 1.f);
        const float measuredBias  = rig.query.lastClearance - trueClearance;

        INFO("slope=" << slopeDeg << " deg  s=" << s
             << "  probeRadius=" << rig.query.probeRadius
             << "  probeHalfHeight=" << rig.query.probeHalfHeight
             << "  probeOffset.z=" << rig.query.probeOffset.z
             << "  trueClearance=" << trueClearance
             << "  probeClearance=" << rig.query.lastClearance
             << "  measuredBias=" << measuredBias
             << "  predictedBias=" << predictedBias);

        REQUIRE(rig.query.lastBlocked);
        REQUIRE(trueClearance == Catch::Approx(kSeeded).margin(1e-3f));
        REQUIRE(measuredBias == Catch::Approx(predictedBias).margin(1e-3f));
        // ...and on FLAT ground the bias is identically zero, whatever `s` is.
        if (slopeDeg == 0.f)
            REQUIRE(measuredBias == Catch::Approx(0.f).margin(1e-4f));
    }
}

// ===========================================================================
// LEDGE FALL AND LANDING  [movement-sim task 57 / USER RULING #29, 2026-09-07]
//
// Three defects, from the user's own PIE run of task 56 -- *"walking off a ledge I see my
// character gain momentum and velocity too quickly ... when I hit the ground, if there is a
// slope where I landed my brawler goes flying at very high speeds."*
//
//   R1 THE DOWNWARD PULL WAS BOUNDED BY ACCELERATION, NOT BY SPEED. Above ride height the
//      task-56 spring pulled toward ride height at `k*e`, clamped only by `hoverMaxAccel`
//      = 30 g. A clearance that jumps into the band therefore costs -516.333 cm/s in ONE tick
//      against gravity's -16.333: 31x too fast. FIXED by the ONE-SIDED servo (F1): at or below
//      ride height task 56's full servo, unchanged; above it a bounded spring pull capped at
//      `hoverPullDownAccel`, whose shipped value is 0 -- gravity and nothing else.
//   R2 `(u, v, up)` IS NOT AN ORTHONORMAL BASIS ON A SLOPE, and velocity was decomposed with
//      dot products. `dot(u, up) = sin(theta)`, so a fall leaked into the tangential channel
//      and a walk leaked into the vertical one. FIXED by the DUAL-BASIS decomposition (F2),
//      which reduces to today's dots at `s == 0` -- that is why no flat-ground case moves.
//   R3 STEP 6' KILLED THE COMPONENT ALONG THE CONTACT NORMAL FOR EVERY CONTACT, which turns a
//      vertical landing on a slope into a tangential slide. FIXED by the FLOOR-CONTACT RULE
//      (F3): a contact whose normal is walkable absorbs the VERTICAL component; everything
//      else keeps the into-contact kill.
//
// THE RED NUMBERS BELOW ARE MEASURED ON THE TASK-56 TREE, NOT DERIVED. Where the design
// note's arithmetic did not reproduce, the case says so and carries both numbers -- see
// `LedgeExitIsFreeFall`, which is the one that did not.
// ===========================================================================

// ---------------------------------------------------------------------------
// R1, STATED DIRECTLY AND WITHOUT ANY GEOMETRY: above ride height, at the shipped
// `hoverPullDownAccel == 0`, the vertical acceleration IS gravity. Not approximately.
//
// The observable is `state.velocity.z` after one tick from a known `vUp`, which is
// `vUp + accelUp*dt` -- so an exact equality here is an exact equality on `accelUp`.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.AboveRideHeightIsGravityOnly", "[BrawlerMovement]")
{
    using namespace movementTests;

    // Every clearance strictly inside the band above ride height, plus the far edge of the
    // probe's reach. All of them classify `Supported`, which is the premise that makes this a
    // statement about the SERVO rather than about the support test.
    for (const float clearance : { 10.5f, 12.f, 15.f, 24.f, 35.f, 45.f, 50.f })
    {
        for (const float startVz : { 0.f, -300.f, 300.f })
        {
            ScriptedRig rig;
            rig.query.scriptedSweeps = { floorHitAt(clearance, rig.probeLength()) };
            rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + clearance);
            rig.state.velocity = glm::vec3(0.f, 0.f, startVz);
            rig.tick(1u);

            const float e = rig.sd.rideHeight - clearance;
            // What the TWO-SIDED task-56 servo would have commanded on this same tick. Spelled
            // from the shipped gains, so it is the law's own number and not a transcription.
            const float twoSided = startVz
                + (rig.sd.gravity
                   + glm::clamp(rig.sd.hoverStiffness * e
                                - rig.sd.hoverDamping * startVz
                                - rig.sd.gravity,
                                -rig.sd.hoverMaxAccel, rig.sd.hoverMaxAccel)) * kDt;

            INFO("clearance=" << clearance << " (e=" << e << ")  startVz=" << startVz
                 << "  ->  vz=" << rig.state.velocity.z
                 << "   gravity only would be " << (startVz + rig.gravityPerTick())
                 << "; the task-56 two-sided servo would have been " << twoSided);

            REQUIRE(rig.sd.hoverPullDownAccel == 0.f);                    // the shipped knob
            REQUIRE(e < 0.f);                                             // ABOVE ride height
            REQUIRE(rig.support() == movement::SupportState::Supported);  // and still supported
            REQUIRE(rig.state.velocity.z
                  == Catch::Approx(startVz + rig.gravityPerTick()).margin(1e-3f));
        }
    }

    // ---- AND IT IS NOT VACUOUS: the same rig one cm BELOW ride height is task 56's full servo,
    //      untouched. F1 changes exactly one arm.
    ScriptedRig low;
    low.query.scriptedSweeps = { floorHitAt(low.sd.rideHeight - 1.f, low.probeLength()) };
    low.tick(1u);
    REQUIRE(low.state.velocity.z == Catch::Approx(low.servoStepPerCm()).margin(1e-3f));
    REQUIRE(low.state.velocity.z != Catch::Approx(low.gravityPerTick()).margin(1e-3f));

    // ---- AND THE KNOB IS THE ONLY WAY BACK TO A DOWNWARD PULL. At 980 the same 45 cm clearance
    //      commands 2 g, at 60000 it saturates the SAME `hoverMaxAccel` clamp task 56 used.
    for (const float knob : { 980.f, 60000.f })
    {
        ScriptedRig k;
        k.sd.hoverPullDownAccel = knob;
        k.query.scriptedSweeps = { floorHitAt(45.f, k.probeLength()) };
        k.tick(1u);
        const float pull = glm::max(k.sd.hoverStiffness * (k.sd.rideHeight - 45.f), -knob);
        const float expected =
            (k.sd.gravity + glm::clamp(pull, -k.sd.hoverMaxAccel, k.sd.hoverMaxAccel)) * kDt;
        INFO("knob=" << knob << " -> vz=" << k.state.velocity.z << " expected " << expected);
        REQUIRE(k.state.velocity.z == Catch::Approx(expected).margin(1e-3f));
    }
}

// ---------------------------------------------------------------------------
// R1'S CONSEQUENCE: A FALL THAT BEGINS INSIDE THE BAND IS A FALL, not a launch.
//
// The floor drops away by exactly `snapDistance`, which is the largest drop that is still
// support at all (`clearance == rideHeight + snapDistance == probeLength`, task 9 q3's edge).
// Under task 56 the FIRST tick of that drop cost -516.333 cm/s -- the whole of R1 in one
// number, and 31x one tick of gravity. Under ruling #29 every tick of the descent is gravity.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.FreeFallAcceleratesAtGravity", "[BrawlerMovement]")
{
    using namespace movementTests;

    PlaneRig rig;
    rig.state.bodyState.position =
        glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight));
    rig.tick(0u);
    REQUIRE(rig.state.velocity.z == Catch::Approx(0.f).margin(1e-4f));      // settled
    rig.engineStep();

    rig.query.planeOffset -= rig.sd.snapDistance;                          // THE DROP

    std::vector<float> deltas;
    std::vector<float> clearance;
    int   backAtRideHeight = -1;
    float firstDelta       = 0.f;
    for (std::uint32_t t = 1u; t <= 40u; ++t)
    {
        const float before = rig.state.velocity.z;
        rig.tick(t);
        if (rig.query.lastClearance > rig.sd.rideHeight)          // still ABOVE ride height
            deltas.push_back(rig.state.velocity.z - before);
        if (t == 1u)
            firstDelta = rig.state.velocity.z - before;
        clearance.push_back(rig.query.lastClearance);
        rig.engineStep();
        if (backAtRideHeight < 0
            && glm::abs(rig.query.lastClearance - rig.sd.rideHeight) < 0.05f
            && glm::abs(rig.state.velocity.z) < 25.f)
            backAtRideHeight = int(t);
    }

    // The free-fall time for `snapDistance` at the authored gravity, in ticks: sqrt(2h/|g|)/dt.
    const float freeFallTicks =
        glm::sqrt(2.f * rig.sd.snapDistance / glm::abs(rig.sd.gravity)) / kDt;

    INFO("snapDistance drop: first delta=" << firstDelta
         << " (gravity*dt = " << rig.gravityPerTick()
         << ", the task-56 saturated pull was -(hoverMaxAccel+|gravity|)*dt = "
         << -rig.maxVerticalStep() << ")"
         << "\n  above-ride-height deltas counted: " << deltas.size()
         << "\n  clearance=" << clearance[0] << ", " << clearance[1] << ", " << clearance[2]
         << ", " << clearance[3] << " ... " << clearance.back()
         << "\n  back at ride height at tick " << backAtRideHeight
         << " (free-fall arithmetic says " << freeFallTicks << ")");

    // ---- PREMISE. The drop really put the character at the far edge of the band, still supported.
    REQUIRE(clearance[0] == Catch::Approx(rig.sd.rideHeight + rig.sd.snapDistance).margin(1e-3f));
    REQUIRE(rig.support() == movement::SupportState::Supported);
    REQUIRE(deltas.size() >= 10u);                          // it really spent the fall up there

    // ---- THE SUBJECT. EVERY tick above ride height is one tick of gravity, starting with the
    //      first -- the tick R1 turned into a 31x launch.
    REQUIRE(firstDelta == Catch::Approx(rig.gravityPerTick()).margin(1e-3f));
    for (std::size_t i = 0; i < deltas.size(); ++i)
    {
        INFO("above-ride-height tick " << i << " delta=" << deltas[i]);
        REQUIRE(deltas[i] == Catch::Approx(rig.gravityPerTick()).margin(1e-3f));
    }

    // ---- AND IT IS CAUGHT WHEN IT ARRIVES, at free-fall time rather than at servo time.
    REQUIRE(backAtRideHeight > 0);
    REQUIRE(float(backAtRideHeight) > freeFallTicks * 0.9f);
    REQUIRE(float(backAtRideHeight) < freeFallTicks * 1.6f);
    // ...and it never rises above where the drop put it on the way back.
    for (std::size_t i = 0; i < clearance.size(); ++i)
    {
        INFO("tick " << (i + 1) << " clearance=" << clearance[i]);
        REQUIRE(clearance[i] <= rig.sd.rideHeight + rig.sd.snapDistance + 1e-3f);
    }
}

// ---------------------------------------------------------------------------
// THE LEDGE ITSELF -- and THE ONE CASE WHOSE DESIGNED RED PREMISE DID NOT REPRODUCE AT THE
// SHIPPED WALK SPEED. Both measurements are in the case, because the difference is the finding.
//
// `impl/design_ledge_fall_and_landing.md` §3 predicted a walk-off exiting at ~1500 cm/s under
// task 56 and ~280 under ruling #29. MEASURED on this rig (`LedgeQueryAdapter`, the swept-capsule
// corner, task 56's own mock), at the shipped `maxWalkSpeed = 100`:
//
//     traverse speed     task 56 exit vz     ruling #29 exit vz     worst downward dv/tick (task 56)
//         100                -89.73               -94.28                -16.333  (== gravity*dt)
//         300               -249.97               -76.09                -47.511  (2.9x gravity)
//         600               -470.67               -49.36               -162.256  (9.9x gravity)
//
// WHY: at 100 cm/s the corner's virtual floor recedes no faster than the character was already
// falling, so the task-56 spring TRACKED it instead of saturating, and the exit speed is the free
// fall the traverse bought either way. The launch needs the floor to outrun the servo, which
// starts at ~300 cm/s of traverse. => THE SUBJECT OF THIS CASE IS THE INVARIANT, NOT THE EXIT
// NUMBER: with `hoverPullDownAccel == 0` no tick above ride height may accelerate the character
// downward faster than gravity. That is the user's complaint, stated as a law, and it is RED at
// 300 and 600 and green at 100 in BOTH arms -- which is exactly what the table says.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.LedgeExitIsFreeFall", "[BrawlerMovement]")
{
    using namespace movementTests;

    for (const float traverseSpeed : { 100.f, 300.f, 600.f })
    {
        LedgeRig rig;
        rig.query.configureFrom(rig.sd);
        rig.query.edgeX      = 0.f;
        rig.sd.maxWalkSpeed  = traverseSpeed;      // a FIXTURE knob; the shipped walk is 100
        rig.state.bodyState.position =
            glm::vec3(-60.f, 0.f, rig.sd.capsuleHalfHeight + rig.sd.rideHeight);

        float worstDownStep = 0.f;
        int   worstTick     = -1;
        float exitSpeed     = 0.f;
        int   exitTick      = -1;
        float steadyClearance = 0.f;
        float steadyVz        = 1e9f;
        std::vector<float> ballistic;

        for (std::uint32_t t = 0u; t < 400u; ++t)
        {
            const glm::vec3 velocityBefore = rig.state.velocity;
            const bool      supportedBefore =
                rig.support() == movement::SupportState::Supported;
            rig.tick(t, stick(1.f, 0.f));

            if (t == 5u) { steadyClearance = rig.query.lastClearance; steadyVz = rig.state.velocity.z; }
            // ⚠ MEASURED ON THE VERTICAL CHANNEL, NOT ON `velocity.z`, and gated on being ABOVE
            // RIDE HEIGHT. Two corrections, both of which this row needed:
            //   * on a tilted corner normal `velocity.z` carries the TANGENTIAL channel's own
            //     vertical component, so its per-tick change is not what step 4 did. `channelsOf`
            //     against the frame the tick actually used (`derived.surfaceNormal`) is.
            //   * the one-sided law only promises "gravity and nothing else" ABOVE ride height. At
            //     or below it task 56's full servo runs and its damper legitimately pushes DOWN
            //     harder than gravity when the character is rising. Ungated, this row would be
            //     asserting something F1 never claimed.
            if (supportedBefore && t > 0u && rig.query.lastClearance > rig.sd.rideHeight)
            {
                const glm::vec3 n = rig.derived.surfaceNormal;
                const float step = channelsOf(rig.state.velocity, n).z
                                 - channelsOf(velocityBefore, n).z;
                if (step < worstDownStep) { worstDownStep = step; worstTick = int(t); }
            }
            if (exitTick < 0 && t > 5u
                && rig.support() == movement::SupportState::Unsupported)
            {
                exitTick  = int(t);
                exitSpeed = rig.state.velocity.z;
            }
            else if (exitTick > 0 && int(t) <= exitTick + 4)
                ballistic.push_back(rig.state.velocity.z - velocityBefore.z);
            rig.engineStep();
        }

        INFO("traverse " << traverseSpeed << " cm/s: settled clearance=" << steadyClearance
             << " vz=" << steadyVz
             << "; exit at tick " << exitTick << " with vz=" << exitSpeed
             << "; worst downward step " << worstDownStep << " at tick " << worstTick
             << " (one tick of gravity is " << rig.gravityPerTick()
             << "; the task-56 servo's bound was " << -rig.maxVerticalStep() << ")");

        // ---- PREMISES: it really hovered, and it really left. Green in both arms.
        REQUIRE(steadyClearance == Catch::Approx(rig.sd.rideHeight).margin(0.5f));
        REQUIRE(exitTick > 5);
        REQUIRE(exitSpeed < 0.f);

        // ---- THE SUBJECT. Nothing pulls the character down harder than gravity, ever.
        REQUIRE(worstDownStep >= rig.gravityPerTick() - 1e-2f);
        // ...so the exit speed cannot exceed what free fall over the same traverse would give.
        REQUIRE(glm::abs(exitSpeed) < glm::abs(rig.gravityPerTick()) * float(exitTick));

        // ---- AND THE FALL AFTER IT IS BALLISTIC, exactly one gravity per tick.
        REQUIRE(ballistic.size() >= 3u);
        for (const float d : ballistic)
            REQUIRE(d == Catch::Approx(rig.gravityPerTick()).margin(1e-3f));
    }
}

// ---------------------------------------------------------------------------
// THE KNOB WORKS, AND THE DEFAULT IS NOT REQUIRED TO. Both arms of `hoverPullDownAccel` on the
// SAME 40 cm step-down, so the case is its own control: at 0 the step is a gravity fall (~17
// ticks), at 980 it is 2 g (~12), and neither overshoots past ride height.
// THIS IS THE ONLY CASE THAT MAY SHIP A NON-ZERO `hoverPullDownAccel`. The shipped default is
// 0 and is the user's (ruling #29); this case proves the knob is wired, not that it should move.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.PullDownKnobRecapturesStep", "[BrawlerMovement]")
{
    using namespace movementTests;

    int   caughtAt[2]  = { -1, -1 };   // clearance back at ride height AND settled (|vz| < 25)
    int   descentAt[2] = { -1, -1 };   // clearance first back DOWN to ride height
    float dip[2]       = { 0.f, 0.f };
    int arm = 0;
    for (const float knob : { 0.f, 980.f })
    {
        PlaneRig rig;
        rig.sd.hoverPullDownAccel = knob;
        rig.state.bodyState.position =
            glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight));
        rig.tick(0u);
        rig.engineStep();
        rig.query.planeOffset -= rig.sd.snapDistance;

        float minClearance = 1e9f;
        for (std::uint32_t t = 1u; t <= 60u; ++t)
        {
            rig.tick(t);
            rig.engineStep();
            minClearance = glm::min(minClearance, rig.query.lastClearance);
            if (descentAt[arm] < 0 && rig.query.lastClearance <= rig.sd.rideHeight)
                descentAt[arm] = int(t);
            if (caughtAt[arm] < 0
                && glm::abs(rig.query.lastClearance - rig.sd.rideHeight) < 0.05f
                && glm::abs(rig.state.velocity.z) < 25.f)
                caughtAt[arm] = int(t);
        }
        dip[arm] = rig.sd.rideHeight - minClearance;

        // The free-fall time for the drop at the TOTAL downward acceleration this knob buys:
        // `sqrt(2h/a)/dt` with `a = |gravity| + knob`. 17.14 ticks at 0, 12.13 at 980 -- and the
        // second is the design note's own "40 cm in 12 ticks".
        const float predictedDescent =
            glm::sqrt(2.f * rig.sd.snapDistance / (glm::abs(rig.sd.gravity) + knob)) / kDt;

        INFO("hoverPullDownAccel=" << knob << ": clearance back at ride height on tick "
             << descentAt[arm] << " (free-fall arithmetic " << predictedDescent
             << "), settled on tick " << caughtAt[arm]
             << ", dip below ride height " << dip[arm] << " cm");
        REQUIRE(caughtAt[arm] > 0);
        REQUIRE(descentAt[arm] > 0);
        // THE DESCENT IS THE KNOB'S OWN ARITHMETIC, to within a tick of discretisation.
        REQUIRE(float(descentAt[arm]) == Catch::Approx(predictedDescent).margin(1.5f));
        // AND IT DOES NOT BOTTOM OUT: the dip stays inside the 10 cm below ride height.
        REQUIRE(dip[arm] < rig.sd.rideHeight);
        ++arm;
    }

    INFO("descent ticks: hoverPullDownAccel 0 -> " << descentAt[0] << ", 980 -> " << descentAt[1]
         << ";  settle ticks: " << caughtAt[0] << " -> " << caughtAt[1]
         << ";  dips: " << dip[0] << " -> " << dip[1] << " cm");
    // ⭐ THE KNOB'S WHOLE CONTENT: 980 is strictly faster on both clocks...
    REQUIRE(descentAt[1] < descentAt[0]);
    REQUIRE(caughtAt[1]  < caughtAt[0]);
    // ...and it recaptures the step inside the AC's 13 ticks while the default does not.
    // ⚠ THE AC'S "13 TICKS" IS THE *DESCENT*, NOT THE SETTLE. Measured, the settle takes 18
    // ticks at 980 and 22 at 0, because the servo also has to absorb the arrival speed the fall
    // bought (280 cm/s at 0, 397 at 980). The design note's 12 is `sqrt(2*40/1960)/dt`, which is
    // the row above; recorded here so the two clocks are not confused again.
    REQUIRE(descentAt[1] <= 13);
    REQUIRE(descentAt[0] >  13);
    // ...and the faster arm pays for it with a deeper dip, which is the trade the knob IS.
    REQUIRE(dip[1] > dip[0]);
}

// ---------------------------------------------------------------------------
// R2'S MIRROR, WHICH NOBODY HAD WALKED: the walk's OWN vertical component was read back as `vUp`
// and DAMPED. Walking at 600 cm/s along a 30 deg plane genuinely rises at 300 cm/s, and task 56's
// servo saw that as something to fight: `-hoverDamping * 300` = -17 112 cm/s^2 of correction.
//
// Under the dual basis the two channels are complementary: a velocity that is PURELY along the
// slope has vertical channel ZERO, and the servo sees nothing to do.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.SlopeWalkDoesNotExciteTheServo", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr float kSlopeDeg = 30.f;
    constexpr float kWalk     = 600.f;

    // BOTH DIRECTIONS, because the defect is SIGNED: task 56 read the walk's own vertical
    // component as `vUp` and damped it, so the character settled HIGH walking uphill and LOW
    // walking downhill. A single direction would have looked like a constant offset.
    for (const float stickX : { 1.f, -1.f })
    {
        PlaneRig rig;
        rig.sd.maxWalkSpeed   = kWalk;                  // a FIXTURE knob; the shipped walk is 100
        rig.query.planeNormal = slopeNormal(kSlopeDeg);
        rig.state.bodyState.position =
            glm::vec3(0.f, 0.f, seedZForClearance(rig.query, rig.sd.rideHeight));

        // The tangent frame this rig produces, from the header's own `buildTangentFrame`, so the
        // expected numbers are the frame's and not a second derivation.
        glm::vec3 u(1.f, 0.f, 0.f), v(0.f, 1.f, 0.f);
        movement::buildTangentFrame(rig.query.planeNormal, u, v);
        const float s = glm::dot(u, movement::kWorldUp);

        float minClearance = 1e9f, maxClearance = -1e9f, worstVertical = 0.f;
        float tailSpeedAlongU = 0.f, tailVz = 0.f;
        for (std::uint32_t t = 0u; t < 200u; ++t)
        {
            rig.tick(t, stick(stickX, 0.f));
            rig.engineStep();
            if (t >= 60u)                                // after the acceleration transient
            {
                minClearance = glm::min(minClearance, rig.query.lastClearance);
                maxClearance = glm::max(maxClearance, rig.query.lastClearance);
                // THE CHANNELS, recomputed here exactly as step 3 and step 4 read them.
                const glm::vec3 V  = rig.state.velocity;
                const glm::vec3 ch = channelsOf(V, rig.query.planeNormal);
                worstVertical   = glm::max(worstVertical, glm::abs(ch.z));
                tailSpeedAlongU = ch.x;
                tailVz          = V.z;
            }
        }

        // The one-tick ripple the one-sided law admits at `e == 0`: a tick that lands a float hair
        // above ride height gets gravity alone and falls `|gravity|*dt^2` before the servo takes
        // it back. `design_ledge_fall_and_landing.md` section 1 predicted exactly this bound, and
        // it is the whole price of "gravity and nothing else above ride height".
        const float rippleBound = glm::abs(rig.sd.gravity) * kDt * kDt;
        // What task 56's servo settled at: the plain dot read `kWalk*s` as `vUp`, and the steady
        // state that leaves is `e = -hoverDamping*kWalk*s*... ` -- measured, 0.4197 cm, sign
        // following the walk. Spelled as the measurement rather than re-derived.
        const float taskFiftySixOffset = 0.4197f;

        INFO((stickX > 0.f ? "UPHILL" : "DOWNHILL") << " 30 deg walk at " << kWalk
             << ": s=dot(u,up)=" << s
             << "  clearance in [" << minClearance << " .. " << maxClearance << "]"
             << "  worst |vertical channel|=" << worstVertical
             << "  tangential=" << tailSpeedAlongU << "  world vz=" << tailVz
             << "  (task 56 read the walk's own " << (stickX * kWalk * s)
             << " cm/s as vUp and damped it with " << (-rig.sd.hoverDamping * stickX * kWalk * s)
             << " cm/s^2, settling " << taskFiftySixOffset
             << " cm off ride height; one-tick ripple bound " << rippleBound << " cm)");

        // ---- PREMISES. It really is on a slope, really walking along it, really supported, and
        //      really climbing/descending -- the vertical motion the servo must NOT see is real.
        REQUIRE(s == Catch::Approx(glm::sin(glm::radians(kSlopeDeg))).margin(1e-5f));
        REQUIRE(rig.support() == movement::SupportState::Supported);
        REQUIRE(tailSpeedAlongU == Catch::Approx(stickX * kWalk).margin(1.f));
        // ⚠ The margin is the RIPPLE, not slop: `tailVz` is sampled on whatever phase of the
        // one-tick limit cycle the last tick landed on, so it moves by up to `|gravity|*dt`.
        REQUIRE(glm::abs(tailVz)
                == Catch::Approx(kWalk * s).margin(glm::abs(rig.gravityPerTick()) + 1.f));

        // ---- A GUARD, NOT THE SUBJECT -- and it is bounded by the RIPPLE, not by zero, which is
        //      the honest reading and was not the one this row started with. Task 56 settled with
        //      a steady velocity purely along `u`, so recomputing the vertical channel from the
        //      RESULT returned 0.0054 there; under the one-sided law the character oscillates in
        //      and out of `e == 0` and the vertical channel swings by exactly one tick of gravity.
        //      What actually differs between the two laws is WHERE the character settles, and
        //      that is the pair of rows below.
        REQUIRE(worstVertical <= glm::abs(rig.gravityPerTick()) + 1e-2f);

        // ---- THE SUBJECT. The character rides at RIDE HEIGHT in both directions. The lower
        //      bound is the one-tick ripple; the upper bound is exact, because with the servo
        //      one-sided nothing can hold the character ABOVE ride height at all.
        REQUIRE(minClearance >= rig.sd.rideHeight - rippleBound - 1e-3f);
        REQUIRE(maxClearance <= rig.sd.rideHeight + 1e-3f);
    }
}

// ---------------------------------------------------------------------------
// R2 AND R3 TOGETHER, AS THE USER SAW THEM: a straight-down 2 m fall onto a 30 deg slope must
// land, not fly. This rig has a SOLVER (`WorldSweepQueryAdapter::depenetrate`) and `drivesBody`
// on, so step 6' is live and both the decomposition and the contact rule are in the path.
//
// THE PRE-FIX BEHAVIOUR IS NOT A SLIDE, IT IS A DIVERGENCE. The design note predicted ~1000
// cm/s down-slope; MEASURED on this rig the task-56 tree reaches 1.5e5 cm/s and keeps going,
// because the leak is a positive feedback loop: the fall leaks into the tangential channel, the
// tangential term has its own vertical component, and that comes back as more "fall".
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.VerticalFallOntoSlopeDoesNotSlide", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr float kSlopeDeg = 30.f;
    constexpr float kDrop     = 200.f;

    WorldRig rig;
    rig.sd.drivesBody = true;                     // step 6' must be live: R3 is one of the subjects
    rig.query.configureFrom(rig.sd);
    const glm::vec3 n = slopeNormal(kSlopeDeg);
    rig.query.solids = { WorldSweepQueryAdapter::Plane{ n, 0.f, "slope" } };

    const float axisHalf = rig.sd.capsuleHalfHeight - rig.sd.capsuleRadius;
    rig.state.bodyState.position = glm::vec3(
        0.f, 0.f, axisHalf + rig.sd.rideHeight + kDrop + rig.sd.capsuleRadius / n.z);
    rig.state.positionCmd = rig.state.bodyState.position;

    const float startX = rig.state.bodyState.position.x;

    int   firstSupported  = -1;
    float impactSpeed     = 0.f;
    float worstTangential = 0.f;
    for (std::uint32_t t = 0u; t < 120u; ++t)
    {
        const float before = rig.state.velocity.z;
        rig.tick(t, stick(0.f, 0.f));
        if (firstSupported < 0 && rig.support() == movement::SupportState::Supported)
        {
            firstSupported = int(t);
            impactSpeed    = before;
        }
        if (firstSupported >= 0 && int(t) <= firstSupported + 30)
        {
            // The speed ALONG the slope: the horizontal component is the whole of it here,
            // because the plane tilts in XZ only.
            worstTangential = glm::max(worstTangential,
                                       glm::length(glm::vec2(rig.state.velocity.x,
                                                             rig.state.velocity.y)));
        }
        rig.state.bodyState.position =
            rig.query.depenetrate(rig.state.bodyState.position + rig.state.velocity * kDt);
    }

    const float driftX = rig.state.bodyState.position.x - startX;

    INFO("2 m fall onto " << kSlopeDeg << " deg: first supported at tick " << firstSupported
         << " at " << impactSpeed << " cm/s"
         << "; worst horizontal speed in the 30 ticks after contact = " << worstTangential
         << "; drift " << driftX << " cm; settled clearance " << rig.query.lastClearance
         << "  (the design note predicted ~1000 cm/s of slide; the task-56 tree DIVERGES)");

    // ---- PREMISES. It really fell, really arrived fast, and really landed.
    REQUIRE(firstSupported > 0);
    REQUIRE(impactSpeed < -400.f);
    REQUIRE(rig.support() == movement::SupportState::Supported);

    // ---- WHICH FIX THIS CASE ACTUALLY MEASURES, MEASURED RATHER THAN ASSUMED. Poisoned back to
    //      plain dot products (F2 off) it goes RED at 1102.20 cm/s of slide; poisoned back to the
    //      unconditional into-contact kill (F3 off) it stays GREEN, because under ruling #29 the
    //      servo catches this 2 m fall inside the band and the solver never has to separate
    //      anything -- `lastPushOut` is zero for the whole run. So THIS case is F2's, and
    //      `DiagonalLandingKeepsHorizontalSpeed` is F3's. R3 is reachable only by a fall the
    //      10 cm below ride height cannot absorb (761.8 cm/s at the shipped constants).
    //
    // ---- THE SUBJECT. A vertical fall lands dead: no tangential speed, no drift, and it is
    //      sitting at ride height when it is over.
    REQUIRE(worstTangential < 50.f);
    REQUIRE(glm::abs(driftX) < 1.f);
    REQUIRE(rig.query.lastClearance == Catch::Approx(rig.sd.rideHeight).margin(0.5f));
}

// ---------------------------------------------------------------------------
// F3'S TWO ARMS, ON THE SAME TICK SHAPE, WITH THE CONTACT NORMAL AS THE ONLY DIFFERENCE.
//
// The fixture is `WallPushOutZeroesIntoWallComponent`'s, with one addition: the character is
// AIRBORNE (`scriptedSweeps` empty), so step 4 is pure gravity and the whole end-of-tick velocity
// is predictable to the last digit. `acceleration = 0` makes step 3's model a pass-through, so the
// post-step-6' velocity is readable at the end of the tick instead of only inferable, and the
// solver position is authored from first principles rather than from step 6's own arithmetic.
// `u`/`v`/`up` are the world axes while unsupported, so F2 is the identity here -- which is what
// makes this case measure F3 ALONE.
//
// THE END-OF-TICK ARITHMETIC, spelled once: step 6' removes one authored component from the
// velocity, step 3 passes the tangential part through unchanged, and step 4 adds ONE tick of
// gravity to the vertical part. So a LANDING leaves `(vx, vy, gravity*dt)` and an into-contact
// kill leaves `(kill.x, kill.y, kill.z + gravity*dt)`.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.DiagonalLandingKeepsHorizontalSpeed", "[BrawlerMovement]")
{
    using namespace movementTests;

    // A landing is a contact whose normal is WALKABLE. 44 deg is inside the shipped 45 deg cap.
    const glm::vec3 landingNormal = slopeNormal(44.f);

    // ---- 1. THE DIAGONAL LANDING. 300 cm/s of walk plus 800 cm/s of fall, onto a 44 deg face.
    ScriptedRig rig;
    rig.query.scriptedSweeps.clear();                    // AIRBORNE: step 4 is gravity alone
    rig.sd.drivesBody   = true;
    rig.sd.acceleration = 0.f;
    rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);
    rig.state.velocity  = glm::vec3(300.f, 0.f, -800.f);
    rig.tick(1u, stick(1.f, 0.f));
    REQUIRE(rig.support() == movement::SupportState::Unsupported);
    REQUIRE(rig.state.velocity.x == Catch::Approx(300.f).margin(1e-3f));   // pass-through premise
    REQUIRE(rig.hasCommandBit());

    const glm::vec3 arriving = rig.state.velocity;
    // THE SOLVER: it separated the body along the face normal by 2 cm.
    rig.engineStepAndCapture(rig.state.bodyState.position + arriving * kDt + landingNormal * 2.f,
                             glm::vec3(0.f));
    rig.tick(2u, stick(1.f, 0.f));

    // What the task-56 rule would have produced on this same push-out: kill along the normal,
    // then the same one tick of gravity.
    const glm::vec3 intoContactKill =
        arriving - glm::dot(arriving, landingNormal) * landingNormal;

    INFO("diagonal landing on 44 deg: arriving=(" << arriving.x << ", "
         << arriving.y << ", " << arriving.z << ")  ->  ("
         << rig.state.velocity.x << ", " << rig.state.velocity.y << ", "
         << rig.state.velocity.z << ")"
         << "\n  the task-56 into-contact kill would have given (" << intoContactKill.x
         << ", " << intoContactKill.y << ", "
         << (intoContactKill.z + rig.gravityPerTick())
         << ") -- the horizontal REVERSES");

    REQUIRE(glm::length(rig.derived.lastPushOut) > movement::kPushOutEps);
    REQUIRE(glm::dot(glm::normalize(rig.derived.lastPushOut), movement::kWorldUp)
            >= rig.sd.cosMaxSlope);                       // the premise: it IS a landing
    // ...and the two arms really do disagree on this push-out, which is what makes the rows below
    // a measurement of F3's branch rather than of the fixture.
    REQUIRE(intoContactKill.x < 0.f);
    REQUIRE(rig.state.velocity.x == Catch::Approx(300.f).margin(1e-3f));
    REQUIRE(rig.state.velocity.y == Catch::Approx(0.f).margin(1e-3f));
    REQUIRE(rig.state.velocity.z
          == Catch::Approx(rig.gravityPerTick()).margin(1e-3f));

    // ---- 2. THE STRAIGHT-DOWN LANDING, which is the design note's own 1000 cm/s number.
    ScriptedRig plumb;
    plumb.query.scriptedSweeps.clear();
    plumb.sd.drivesBody   = true;
    plumb.sd.acceleration = 0.f;
    const glm::vec3 thirty = slopeNormal(30.f);
    plumb.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + plumb.sd.rideHeight);
    plumb.state.velocity  = glm::vec3(0.f, 0.f, -2000.f);
    plumb.tick(1u, stick(0.f, 0.f));
    const glm::vec3 falling = plumb.state.velocity;
    plumb.engineStepAndCapture(plumb.state.bodyState.position + falling * kDt + thirty * 2.f,
                               glm::vec3(0.f));
    plumb.tick(2u, stick(0.f, 0.f));
    const glm::vec3 deflected = falling - glm::dot(falling, thirty) * thirty;
    INFO("plumb 2000 cm/s fall onto 30 deg: -> (" << plumb.state.velocity.x << ", "
         << plumb.state.velocity.y << ", " << plumb.state.velocity.z
         << ")  the task-56 rule deflected it to (" << deflected.x << ", " << deflected.y
         << ", " << deflected.z << "), |horizontal| = " << glm::abs(deflected.x)
         << ", down-slope = " << (glm::abs(deflected.x) / glm::cos(glm::radians(30.f))));
    // THE DESIGN NOTE'S OWN NUMBER, confirmed on the shipped constants: 866.025 cm/s of horizontal
    // is 1000.000 cm/s along a 30 deg face. That is what the character used to be given.
    REQUIRE(glm::abs(deflected.x) == Catch::Approx(866.025f).margin(1.f));
    REQUIRE(plumb.state.velocity.x == Catch::Approx(0.f).margin(1e-3f));
    REQUIRE(plumb.state.velocity.y == Catch::Approx(0.f).margin(1e-3f));
    REQUIRE(plumb.state.velocity.z
          == Catch::Approx(plumb.gravityPerTick()).margin(1e-3f));
}

// ---------------------------------------------------------------------------
// ...AND THE OTHER ARM IS UNTOUCHED. A wall, a steep face past the walkable cap, and another
// brawler all keep task 54's into-contact kill exactly as it was: F3 is a BRANCH, not a
// replacement. Same fixture, same tick shape; only the normal moves.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.WallContactStillKillsIntoWallComponent", "[BrawlerMovement]")
{
    using namespace movementTests;

    struct Arm { glm::vec3 normal; const char* what; };
    const Arm arms[] = {
        { glm::vec3(-1.f, 0.f, 0.f), "a vertical wall" },
        { slopeNormal(46.f),         "a face one degree past the walkable cap" },
        { glm::normalize(glm::vec3(-1.f, 0.f, 0.25f)), "another brawler riding up" },
    };

    for (const Arm& arm : arms)
    {
        ScriptedRig rig;
        rig.query.scriptedSweeps.clear();                // AIRBORNE, exactly as the landing case
        rig.sd.drivesBody   = true;
        rig.sd.acceleration = 0.f;
        rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);
        rig.state.velocity  = glm::vec3(100.f, 50.f, -300.f);
        rig.tick(1u, stick(1.f, 0.f));
        const glm::vec3 arriving = rig.state.velocity;
        rig.engineStepAndCapture(rig.state.bodyState.position + arriving * kDt + arm.normal * 2.f,
                                 glm::vec3(0.f));
        rig.tick(2u, stick(1.f, 0.f));

        // The into-contact kill, then step 4's one tick of gravity on the vertical.
        const glm::vec3 killed  = arriving - glm::dot(arriving, arm.normal) * arm.normal;
        const glm::vec3 expected(killed.x, killed.y, killed.z + rig.gravityPerTick());
        // What the LANDING arm would have produced instead, so the case names both.
        const glm::vec3 landingWouldGive(arriving.x, arriving.y, rig.gravityPerTick());

        INFO(arm.what << ": n=(" << arm.normal.x << ", " << arm.normal.y << ", "
             << arm.normal.z << ")  dot(n, up)=" << glm::dot(arm.normal, movement::kWorldUp)
             << " < cosMaxSlope=" << rig.sd.cosMaxSlope
             << "  ->  (" << rig.state.velocity.x << ", " << rig.state.velocity.y << ", "
             << rig.state.velocity.z << "); into-contact kill + gravity says (" << expected.x
             << ", " << expected.y << ", " << expected.z
             << "); the landing arm would have said (" << landingWouldGive.x << ", "
             << landingWouldGive.y << ", " << landingWouldGive.z << ")");

        // The premise that puts this arm on the WALL side of F3's branch...
        REQUIRE(glm::dot(arm.normal, movement::kWorldUp) < rig.sd.cosMaxSlope);
        REQUIRE(glm::length(rig.derived.lastPushOut) > movement::kPushOutEps);
        // ...and the premise that makes the rows below discriminate: the two arms disagree here.
        REQUIRE(glm::length(expected - landingWouldGive) > 1.f);

        REQUIRE(rig.state.velocity.x == Catch::Approx(expected.x).margin(1e-3f));
        REQUIRE(rig.state.velocity.y == Catch::Approx(expected.y).margin(1e-3f));
        REQUIRE(rig.state.velocity.z == Catch::Approx(expected.z).margin(1e-3f));
    }
}

// ===========================================================================
// HIT REACTIONS -- STEP 1's `committed` ARM AND STEP 3's KNOCKBACK  [movement-sim task 27]
//
// SUBJECT: the user's 2026-09-03 requirement -- "a significant impulse, thrown 5 metres, then
// quickly to a stop" -- as revision 6 / ruling #14(c) settled it: a velocity ASSIGNMENT on the hit
// tick and the Smash decay afterwards, never an impulse.
//
// ⭐ THERE IS NO `Launched` STATE. `HitFlinch` carries the lockout and the HIT carries the
// reaction: `m_hitReaction` says freeze (Stun) or assign-and-decay (Knockback) and
// `m_flinchDuration` says for how long, both resolved ONCE by `brawlerHitRouting::System` from the
// attacking sequence's `HitReactionSpec` and both on the wire. This rig therefore drives the
// reaction the way it drives everything else: `Rig::hit()` writes the machine state and the
// inbound slice together, which is exactly the pair the routing pass and the machine veto produce.
//
// ⛔ WHAT THIS FILE MEASURES AND WHAT IT DOES NOT. The lockout's LENGTH and its retrigger are
// the machine's, and they are pinned in `IntegrateThreeAttackSelectionTest.cpp` where the machine
// actually runs; the reaction TABLE and the resolved dwell are the routing system's, pinned in
// `BrawlerHitRoutingTest.cpp`. Everything below is about what the BODY does while the machine
// holds it, which is this file's subject and the one the rig can drive without a second sub-sim.
// ===========================================================================

// ---------------------------------------------------------------------------
// ⭐⭐ THE OWED CASE. Routed to task 27 from task 12 on 2026-09-06 under this exact name,
// because task 12 had no state to drive it: step 1's `committed` was a hard
// `const bool committed = false;` and there was no reaction on the wire to test. Both exist now.
//
// ⭐ THE CLOSED FORM IS ASSERTED, NOT RESTATED. `knockbackDistance()` is computed from the two
// SHIPPED constants -- `m_hitReactions[right].knockbackSpeed` and `StaticData::launchDecel` -- and
// the assertion is that v^2/(2a) equals FIVE METRES. Re-typing 500 would have made this case pass
// against its own copy of the number.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.LaunchedDecelsAtLaunchDecel", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);

    // 1. THE USER'S NUMBER, AS AN IDENTITY OVER THE SHIPPED CONSTANTS. 2000^2 / 8000 = 500 cm.
    INFO("knockbackSpeed=" << rig.knockbackSpeed() << " launchDecel=" << rig.sd.launchDecel
         << " -> v^2/(2a)=" << rig.knockbackDistance() << " cm over "
         << rig.knockbackSlideSeconds() << " s");
    REQUIRE(rig.knockbackDistance() == Catch::Approx(500.f).margin(1e-3f));
    REQUIRE(rig.knockbackSlideSeconds() == Catch::Approx(0.5f).margin(1e-6f));

    // 2. THE ASSIGNMENT, on the hit tick and keyed on the hit -- not on the machine's timer.
    const glm::vec3 startPosition = rig.state.bodyState.position;
    rig.hit(HitReactionKind::Knockback, glm::vec2(1.f, 0.f), 0.f);
    rig.tick(1u, stick(0.f, 0.f));
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.knockbackSpeed()).margin(1e-3f));
    REQUIRE(rig.state.velocity.y == Catch::Approx(0.f).margin(1e-4f));
    // Steady hover at ride height: the vertical channel is identically zero, so the whole of the
    // velocity is the knockback. That is what makes the rows below exact rather than approximate.
    REQUIRE(rig.state.velocity.z == Catch::Approx(0.f).margin(1e-4f));
    rig.engineStep();

    // 3. THE DECAY IS `launchDecel`, EVERY TICK, AND IT REACHES EXACTLY ZERO. `moveTowards`
    //    returns the TARGET once the remaining distance is inside one step, so the last tick lands
    //    on 0.000000 rather than overshooting into a backwards crawl.
    const float perTick = rig.sd.launchDecel * kDt;
    int zeroAtTick = -1;
    float travelled = rig.state.velocity.x * kDt;   // the hit tick's own displacement
    for (std::uint32_t t = 2u; t <= 40u; ++t)
    {
        const float before = rig.state.velocity.x;
        rig.tick(t, stick(0.f, 0.f));
        const float after = rig.state.velocity.x;
        if (zeroAtTick < 0)
        {
            INFO("tick " << t << ": " << before << " -> " << after << " (step " << perTick << ")");
            REQUIRE(before - after == Catch::Approx(glm::min(perTick, before)).margin(1e-3f));
            if (after == 0.f) zeroAtTick = int(t);
        }
        travelled += rig.state.velocity.x * kDt;
        rig.engineStep();
    }

    // EXACTLY zero -- and at tick 32, which is THIRTY-ONE decay ticks after the hit tick and not
    // the thirty the real arithmetic names. ⛔ THIS IS THE FLOAT NOTE THE DESIGN ASKED TO BE
    // PINNED RATHER THAN HAND-TUNED, and it is MEASURED: `4000 * (1/60)` is 66.666664, not
    // 66.666667, so thirty steps remove 1999.99992 and leave 8e-5 cm/s behind -- one more tick,
    // which `moveTowards` then takes all the way to the endpoint. A 30-or-31-step exit was
    // anticipated (both are "lockout >= slide"); the row pins WHICH, so a future retune that
    // changes it has to be looked at rather than absorbed.
    INFO("reached zero at tick " << zeroAtTick << "; travelled " << travelled << " cm");
    REQUIRE(zeroAtTick == 32);
    REQUIRE(rig.state.velocity.x == 0.f);

    // 4. ...AND IT STAYS AT ZERO: the decay does not push the character backwards once it arrives,
    //    and nothing re-reads `knockbackSpeed` after the hit tick.
    rig.tick(41u, stick(0.f, 0.f));
    REQUIRE(rig.state.velocity.x == 0.f);

    // 5. THE DISTANCE, MEASURED, AGAINST THE CLOSED FORM PLUS ITS OWN DISCRETISATION TERM.
    //    Summing v*dt over the N+1 ticks of a linear ramp gives dt*(N+1)*v0/2, which is
    //    v0^2/(2a) + v0*dt/2 exactly -- half a tick of the launch speed, 16.667 cm at the shipped
    //    pair. ⛔ Asserting the bare 500 here would have been wrong by that term and would have
    //    had to be absorbed into a loose margin; naming it keeps the row exact and keeps the
    //    closed form in row 1 where it belongs.
    const float discreteExpectation =
        rig.knockbackDistance() + rig.knockbackSpeed() * kDt * 0.5f;
    INFO("travelled " << travelled << " cm; closed form " << rig.knockbackDistance()
         << " + half a tick of launch speed " << (rig.knockbackSpeed() * kDt * 0.5f)
         << " = " << discreteExpectation);
    REQUIRE(travelled == Catch::Approx(discreteExpectation).margin(1e-2f));
    REQUIRE(rig.state.bodyState.position.x - startPosition.x
            == Catch::Approx(discreteExpectation).margin(1e-2f));
}

// ---------------------------------------------------------------------------
// The direction is the HIT's, and a second hit REPLACES it. [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.KnockbackHitLaunches", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);

    const glm::vec2 first = glm::normalize(glm::vec2(1.f, -1.f));
    rig.hit(HitReactionKind::Knockback, first, 0.f);
    rig.tick(1u, stick(-1.f, 0.f));   // the stick is IGNORED: the arm is committed, not modelled

    INFO("velocity=(" << rig.state.velocity.x << ", " << rig.state.velocity.y << ", "
         << rig.state.velocity.z << ") for direction (" << first.x << ", " << first.y << ")");
    REQUIRE(rig.state.velocity.x == Catch::Approx(first.x * rig.knockbackSpeed()).margin(1e-2f));
    REQUIRE(rig.state.velocity.y == Catch::Approx(first.y * rig.knockbackSpeed()).margin(1e-2f));
    // ⭐ THE SPEED IS THE SPEED, not its projection: `u` and `v` are orthonormal, so mapping the
    // world XY direction onto them component-wise preserves the magnitude exactly.
    REQUIRE(glm::length(glm::vec2(rig.state.velocity.x, rig.state.velocity.y))
            == Catch::Approx(rig.knockbackSpeed()).margin(1e-2f));

    // One tick of decay, so the re-launch below is measurably a REPLACEMENT and not an addition.
    rig.tick(2u, stick(0.f, 0.f));
    const float decayed = glm::length(glm::vec2(rig.state.velocity.x, rig.state.velocity.y));
    REQUIRE(decayed == Catch::Approx(rig.knockbackSpeed() - rig.sd.launchDecel * kDt).margin(1e-2f));

    // ⛔ RE-LAUNCH REPLACES. Revision 5's "consecutive hits SUM" is superseded by revision 6's
    // assignment (architect review 2026-09-12 section 2.4): the second hit's direction and speed
    // are the whole of the answer, and the decayed remainder of the first is discarded. A sum
    // would read ~3933 cm/s here, and in a direction neither hit asked for.
    const glm::vec2 second(0.f, 1.f);
    rig.hit(HitReactionKind::Knockback, second, 0.f);
    rig.tick(3u, stick(0.f, 0.f));
    INFO("re-launched to (" << rig.state.velocity.x << ", " << rig.state.velocity.y << ")");
    REQUIRE(rig.state.velocity.x == Catch::Approx(0.f).margin(1e-2f));
    REQUIRE(rig.state.velocity.y == Catch::Approx(rig.knockbackSpeed()).margin(1e-2f));
    REQUIRE(glm::length(glm::vec2(rig.state.velocity.x, rig.state.velocity.y))
            == Catch::Approx(rig.knockbackSpeed()).margin(1e-2f));
}

// ---------------------------------------------------------------------------
// The forward/overhead hit and the projectile STUN: the pre-task-27 `HitFlinch` behaviour,
// exactly, and now reachable only through `m_hitReaction == Stun`. [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.StunHitFreezesWithoutKnockback", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.state.velocity = glm::vec3(rig.sd.maxWalkSpeed, 0.f, 0.f);

    const glm::vec3 startPosition = rig.state.bodyState.position;
    rig.hit(HitReactionKind::Stun, glm::vec2(0.f), 0.3f);

    for (std::uint32_t t = 1u; t <= 18u; ++t)
    {
        rig.tick(t, stick(1.f, 0.f));
        INFO("stun dwell tick " << t);
        REQUIRE(rig.frozenBit());
        REQUIRE(rig.state.velocity == glm::vec3(0.f));
        rig.engineStep();
    }
    // NO DISPLACEMENT AT ALL -- a stun is the hitstop freeze, not a slow slide.
    REQUIRE(rig.state.bodyState.position == startPosition);

    // ⭐ THE DISCRIMINATOR, and it is the whole of this task on the movement side: the SAME
    // machine state with the OTHER reaction byte is not frozen at all. Without this arm the case
    // would pass on a `machineFreezesMovement` that still froze on plain `HitFlinch`.
    ScriptedRig knocked;
    seatOnFlatGroundAtRideHeight(knocked);
    knocked.hit(HitReactionKind::Knockback, glm::vec2(1.f, 0.f), 0.f);
    knocked.tick(1u, stick(0.f, 0.f));
    REQUIRE(knocked.machineState.m_currentState == rig.machineState.m_currentState);
    REQUIRE_FALSE(knocked.frozenBit());
    REQUIRE(knocked.state.velocity.x == Catch::Approx(knocked.knockbackSpeed()).margin(1e-2f));
}

// ---------------------------------------------------------------------------
// THE CROSS-KIND POLICY, section 3 of `impl/design_hit_reactions.md`, and it is the USER'S RULED
// DEFAULT: a stun landing mid-slide KILLS the momentum. The fighting-game convention (a hit resets
// the reaction) rather than an accident of the branch order. [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.StunDuringSlideKillsMomentum", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.hit(HitReactionKind::Knockback, glm::vec2(1.f, 0.f), 0.f);
    rig.tick(1u, stick(0.f, 0.f));

    // Mid-slide, with real momentum left: the premise that makes the kill measurable.
    for (std::uint32_t t = 2u; t <= 10u; ++t)
        rig.tick(t, stick(0.f, 0.f));
    const float midSlide = rig.state.velocity.x;
    INFO("mid-slide speed " << midSlide << " cm/s when the stun lands");
    REQUIRE(midSlide > 0.5f * rig.knockbackSpeed());

    rig.hit(HitReactionKind::Stun, glm::vec2(0.f), 0.3f);
    rig.tick(11u, stick(0.f, 0.f));

    // ⛔ DEAD, not decayed: the stun freezes, and a freeze is exact this tick (see
    // `FrozenIsExactThisTick`). `midSlide - launchDecel*dt` would be the decay's answer.
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
    REQUIRE(midSlide - rig.sd.launchDecel * kDt > 1.f);   // ...and the decay would NOT have been 0
    rig.tick(12u, stick(0.f, 0.f));
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
}

// ---------------------------------------------------------------------------
// ⛔ `committed` IS TESTED BEFORE `frozen`, and this is the case that says so. Holding guard
// during a knockback must not freeze the slide: guard is blocked by the shape gate in
// `DAttackGuardSimulation` (`machine != Idle`), never by freezing the body. Swap the two branches
// in step 3 and this case reads 0 where it requires 2000. [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.HoldGuardDoesNotFreezeASlide", "[BrawlerMovement]")
{
    using namespace movementTests;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    rig.hit(HitReactionKind::Knockback, glm::vec2(1.f, 0.f), 0.f);

    rig.tick(1u, stick(0.f, 0.f), movement::kInputFlagHoldGuard);
    INFO("holdGuard held through the launch tick: v.x=" << rig.state.velocity.x);
    REQUIRE(rig.state.velocity.x == Catch::Approx(rig.knockbackSpeed()).margin(1e-2f));

    rig.tick(2u, stick(0.f, 0.f), movement::kInputFlagHoldGuard);
    REQUIRE(rig.state.velocity.x
            == Catch::Approx(rig.knockbackSpeed() - rig.sd.launchDecel * kDt).margin(1e-2f));

    // CONTROL -- the same input with no knockback freezes, so the bit really is live in this
    // fixture and the rows above are a statement about the ORDER, not about a dead input path.
    ScriptedRig walking;
    seatOnFlatGroundAtRideHeight(walking);
    walking.state.velocity = glm::vec3(walking.sd.maxWalkSpeed, 0.f, 0.f);
    walking.tick(1u, stick(1.f, 0.f), movement::kInputFlagHoldGuard);
    REQUIRE(walking.state.velocity == glm::vec3(0.f));
}

// ---------------------------------------------------------------------------
// ⭐⭐ A SHOVE ON A SLOPE STAYS ON THE SLOPE. The hit direction is a world XY unit vector and
// it is mapped into the tangent frame COMPONENT-WISE (`d.x -> u`, `d.y -> v`), which is exact
// because `buildTangentFrame` makes `u`'s horizontal projection X-aligned and `v` horizontal
// whenever the normal is walkable. The consequences, both measured here: the launched velocity is
// PERPENDICULAR to the surface normal, and its horizontal speed is `knockbackSpeed * cos(theta)`
// -- the character covers 5 m ALONG THE FACE, not 5 m of ground.
//
// ⛔ THE PLAUSIBLE WRONG EDIT this case exists to catch is
// `glm::vec2(dot(dWorld, u), dot(dWorld, v))` -- the projection. It is the reflex spelling, it is
// dimensionally innocent, and on a 30 degree face it launches at 1732 cm/s instead of 2000, losing
// 13.4 % of the user's 5 m. See `docs/BrawlerMovementSimulation-rationale.md` D-06.
// [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.SlopeKnockbackStaysOnSlope", "[BrawlerMovement]")
{
    using namespace movementTests;

    const float theta = 30.f;
    const glm::vec3 n = slopeNormal(theta);

    ScriptedRig rig;
    rig.query.scriptedSweeps = { floorHitAt(rig.sd.rideHeight, rig.probeLength(), n) };
    rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);

    rig.hit(HitReactionKind::Knockback, glm::vec2(1.f, 0.f), 0.f);
    rig.tick(1u, stick(0.f, 0.f));

    const glm::vec3 velocity = rig.state.velocity;
    const float horizontal = glm::length(glm::vec2(velocity.x, velocity.y));
    INFO("30 deg face: v=(" << velocity.x << ", " << velocity.y << ", " << velocity.z
         << ")  |v|=" << glm::length(velocity) << "  horizontal=" << horizontal
         << "  expected horizontal=" << (rig.knockbackSpeed() * glm::cos(glm::radians(theta)))
         << "  dot(v, n)=" << glm::dot(velocity, n));

    REQUIRE(rig.support() == movement::SupportState::Supported);
    // 1. ON the face: no component into or out of it.
    REQUIRE(glm::dot(velocity, n) == Catch::Approx(0.f).margin(1e-2f));
    // 2. AT the authored speed, along the face -- this is the row the projection spelling fails.
    REQUIRE(glm::length(velocity) == Catch::Approx(rig.knockbackSpeed()).margin(1e-2f));
    // 3. ...so the HORIZONTAL speed is knockbackSpeed * cos(theta), and the case names the number
    //    the wrong spelling would have produced as the full speed.
    REQUIRE(horizontal
            == Catch::Approx(rig.knockbackSpeed() * glm::cos(glm::radians(theta))).margin(1e-2f));
    REQUIRE(velocity.z
            == Catch::Approx(rig.knockbackSpeed() * glm::sin(glm::radians(theta))).margin(1e-2f));
    // ...and the two are genuinely different, so row 2 discriminates.
    REQUIRE(glm::abs(glm::length(velocity) - horizontal) > 100.f);

    // 4. THE CHANNELS, read back through the file's own independent dual basis: all of the speed
    //    is in `u`, none in `v`, and the vertical channel is untouched by the assignment.
    const glm::vec3 channels = channelsOf(velocity, n);
    REQUIRE(channels.x == Catch::Approx(rig.knockbackSpeed()).margin(1e-2f));
    REQUIRE(channels.y == Catch::Approx(0.f).margin(1e-2f));
    REQUIRE(channels.z == Catch::Approx(0.f).margin(1e-2f));

    // ⚠ ⚠ MEASURED AND ROUTED, NOT ASSERTED AS DESIRABLE [movement-sim task 27]. The detach
    // arm the architect ruled is `committed && dot(velocity, up) > 0` -- WORLD z. On flat ground a
    // knockback's world z is exactly 0 and the arm is false, which is what section 2.5(b) of
    // `impl/review_task27_knockback_design.md` means by "false for every XY hit". ON A SLOPE IT IS
    // NOT: `u.z == sin(theta)`, so an up-slope shove carries `knockbackSpeed * sin(theta)` of
    // world z -- 1000 cm/s here -- and the NEXT tick's step 2 therefore detaches. The frame does
    // not move with it (`n` is keyed on `walkable`, not on `support`), so the slide stays on the
    // face; what is lost is the hover hold for the rest of the slide. The number is printed and
    // the state is recorded here rather than hidden, and the finding is routed to the lead in
    // `impl/impl_notes_seam_27.md`.
    rig.tick(2u, stick(0.f, 0.f));
    INFO("tick 2 on the face: support=" << int(rig.support())
         << " (0 Unsupported, 1 Supported)  v.z=" << rig.state.velocity.z
         << "  -- world z carries the TANGENTIAL channel on a slope; the vertical CHANNEL is "
         << channelsOf(rig.state.velocity, n).z);
    REQUIRE(glm::dot(velocity, movement::kWorldUp) > 0.f);
    REQUIRE(rig.support() == movement::SupportState::Unsupported);
}

// ---------------------------------------------------------------------------
// A KNOCKBACK INTO A WALL STOPS AT THE WALL, and the tangential remainder keeps decaying.
// Task 54's contact rule is untouched by task 27: step 6' kills the into-wall component of
// whatever the body is carrying, and the decay in step 3 then works on what is left.
// [movement-sim task 27]
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.KnockbackIntoWallStopsAtWall", "[BrawlerMovement]")
{
    using namespace movementTests;

    // The wall faces -X; the shove is +X into it, with a +Y component that must SURVIVE.
    const glm::vec3 wallNormal(-1.f, 0.f, 0.f);

    ScriptedRig rig;
    rig.query.scriptedSweeps.clear();          // airborne, exactly as the task-54 wall cases
    rig.sd.drivesBody = true;
    rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);

    const glm::vec2 direction = glm::normalize(glm::vec2(1.f, 1.f));
    rig.hit(HitReactionKind::Knockback, direction, 0.f);
    rig.tick(1u, stick(0.f, 0.f));

    const glm::vec3 arriving = rig.state.velocity;
    REQUIRE(arriving.x == Catch::Approx(direction.x * rig.knockbackSpeed()).margin(1e-2f));
    REQUIRE(arriving.y == Catch::Approx(direction.y * rig.knockbackSpeed()).margin(1e-2f));

    // The solver stops the body at the wall and pushes it out along the normal.
    rig.engineStepAndCapture(rig.state.bodyState.position + arriving * kDt + wallNormal * 2.f,
                             glm::vec3(0.f));
    rig.tick(2u, stick(0.f, 0.f));

    INFO("into the wall at (" << arriving.x << ", " << arriving.y << ") -> ("
         << rig.state.velocity.x << ", " << rig.state.velocity.y << "); pushOut.x="
         << rig.derived.lastPushOut.x);

    // PREMISE: step 6' really ran and really found the wall.
    REQUIRE(glm::length(rig.derived.lastPushOut) > movement::kPushOutEps);

    // 1. THE INTO-WALL COMPONENT IS GONE. Killed by step 6' before step 3 decays anything, so the
    //    character does not keep pressing 1414 cm/s of nothing into the surface.
    REQUIRE(rig.state.velocity.x == Catch::Approx(0.f).margin(1e-2f));

    // 2. ...AND THE TANGENTIAL REMAINDER IS STILL DECAYING AT `launchDecel`, not zeroed with it.
    //    `moveTowards` works on the 2D pair, so the step it takes is along the REMAINING
    //    direction: with x killed, the whole of one tick's decay lands on y.
    const float remaining = direction.y * rig.knockbackSpeed();
    REQUIRE(rig.state.velocity.y
            == Catch::Approx(remaining - rig.sd.launchDecel * kDt).margin(1e-2f));
    REQUIRE(rig.state.velocity.y > 0.f);
}

// ===========================================================================
// STEP 3 -- THE ATTACK MOVEMENT LOCK  [movement-sim task 84, 2026-09-20]
//
// SUBJECT: the THIRD branch of step 3's dispatch, `locked`, between `committed` and `frozen`.
// While the machine is `Attacking`, the two tangent-plane channels are multiplied by
// `(R-1)/R` where `R` is the number of Attacking ticks left INCLUDING this one, taken from
// the machine's wire `m_attackEndTick` (the first tick the machine will be `Idle`). The stick
// is never read; the vertical channel, gravity and the hover servo run unchanged.
//
// THE INDEXING, spelled once so every case below can be read against it. Seat the rig at
// `tick0` with `m_attackEndTick = tick0 + N`. On the m-th Attacking tick (m = 1..N,
// t = tick0 + m - 1) the branch sees `R = N - m + 1`, so the speed AFTER that tick is
//     v_m = v_(m-1) * (R-1)/R = v0 * (N - m) / N
// by telescoping -- a linear ramp, the same displacement a constant deceleration `v0/(N*dt)`
// would give, and EXACTLY zero on m = N because the factor is `0/1` and the multiply is exact.
// ===========================================================================

// ---------------------------------------------------------------------------
// ⭐⭐ THE USER'S SENTENCE, MEASURED: "comes to a full stop exactly the tick where the attack
// ends". The ramp is checked to the digit on every tick of the swing and the last one is
// asserted with `==`, not `Approx` -- the ratio form reaches zero by MULTIPLICATION, so there
// is no epsilon to spend. The control arm at the end proves the fixture can still move.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.AttackSlideStopsExactlyOnTheLastAttackingTick", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr std::uint32_t kTick0 = 1u;
    constexpr std::uint32_t kN     = 10u;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    const float v0 = rig.sd.maxWalkSpeed;
    rig.state.velocity = glm::vec3(v0, 0.f, 0.f);
    rig.machineState.m_currentState  = DAttackState::Attacking;
    rig.machineState.m_attackEndTick = kTick0 + kN;

    for (std::uint32_t m = 1u; m <= kN; ++m)
    {
        // The stick is held HARD ACROSS the direction of travel on every tick. If step 3 read it
        // at all, `velocity.y` would grow and `velocity.x` would not follow the ramp.
        rig.tick(kTick0 + m - 1u, stick(0.f, 1.f));

        const float expected = v0 * float(kN - m) / float(kN);
        INFO("attacking tick " << m << " of " << kN << " (t=" << (kTick0 + m - 1u)
             << ", R=" << (kN - m + 1u) << "): v=(" << rig.state.velocity.x << ", "
             << rig.state.velocity.y << ", " << rig.state.velocity.z << ")  expected v.x="
             << expected);
        REQUIRE(rig.state.velocity.x == Catch::Approx(expected).margin(1e-3f));
        REQUIRE(rig.state.velocity.y == Catch::Approx(0.f).margin(1e-4f));
    }

    // ⭐ EXACT. `float(R-1)/float(R)` is `0.f/1.f` on the last tick, and `currentUV * 0.f` is a
    // pair of signed zeroes; `-0.f == 0.f` is true, so this reads bitwise as well as numerically.
    REQUIRE(rig.state.velocity == glm::vec3(0.f));

    // CONTROL -- the machine leaves Attacking and the walk law resumes from rest. Without this the
    // case would pass on a rig that had simply been frozen.
    rig.machineState.m_currentState = DAttackState::Idle;
    rig.tick(kTick0 + kN, stick(0.f, 1.f));
    REQUIRE(rig.state.velocity.y == Catch::Approx(rig.accelPerTick()).margin(1e-4f));
    REQUIRE_FALSE(rig.frozenBit());
}

// ---------------------------------------------------------------------------
// ⭐ THE DIRECTION IS LOCKED BECAUSE NOTHING CAN TURN IT. The branch multiplies `currentUV` by a
// SCALAR, so the tangent-plane direction is preserved on every tick to rounding, whatever the
// stick says. Here the stick is held exactly REVERSED, which is the input that would show a
// projection or a `moveTowards`-to-the-stick spelling immediately.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.AttackSlideDirectionIsLockedAgainstTheStick", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr std::uint32_t kTick0 = 1u;
    constexpr std::uint32_t kN     = 12u;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);

    // OBLIQUE, so neither channel is zero and a component-swap would be visible too.
    const glm::vec2 direction = glm::normalize(glm::vec2(1.f, 2.f));
    const float     v0        = rig.sd.maxWalkSpeed;
    rig.state.velocity = glm::vec3(direction.x * v0, direction.y * v0, 0.f);
    rig.machineState.m_currentState  = DAttackState::Attacking;
    rig.machineState.m_attackEndTick = kTick0 + kN;

    for (std::uint32_t m = 1u; m < kN; ++m)      // ...up to but NOT including the zero tick
    {
        rig.tick(kTick0 + m - 1u, stick(-direction.x, -direction.y));

        const glm::vec2 planar(rig.state.velocity.x, rig.state.velocity.y);
        const float     speed = glm::length(planar);
        INFO("tick " << m << ": v=(" << planar.x << ", " << planar.y << ")  |v|=" << speed
             << "  expected |v|=" << (v0 * float(kN - m) / float(kN)));
        REQUIRE(speed > 0.f);
        REQUIRE(glm::dot(planar * (1.f / speed), direction) == Catch::Approx(1.f).margin(1e-5f));
        REQUIRE(speed == Catch::Approx(v0 * float(kN - m) / float(kN)).margin(1e-3f));
    }

    rig.tick(kTick0 + kN - 1u, stick(-direction.x, -direction.y));
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
}

// ---------------------------------------------------------------------------
// ⭐⭐ A SLIDE UP A SLOPE KEEPS ITS HOVER HOLD, and this is the case the whole branch placement
// exists for. `locked` is NOT a tenant of the `committed` arm: `committed` also feeds
// `detachesFromSupport(state, up, committed)` -- `committed && dot(velocity, up) > 0` -- and a
// tangential slide up a face carries world z, so folding this law into `committed` would detach
// the character and fly it off the slope. That is exactly the behaviour ⛔G-07 records and
// `SlopeKnockbackStaysOnSlope` measures; the control arm below reproduces it on the SAME fixture.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.AttackSlideDoesNotDetachOnASlope", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr std::uint32_t kTick0 = 1u;
    constexpr std::uint32_t kN     = 8u;

    const float     theta = 30.f;
    const glm::vec3 n     = slopeNormal(theta);

    ScriptedRig rig;
    rig.query.scriptedSweeps = { floorHitAt(rig.sd.rideHeight, rig.probeLength(), n) };
    rig.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + rig.sd.rideHeight);

    // UP the face: the `u` axis of the surface frame, whose world z is `sin(theta)` > 0.
    glm::vec3 u(1.f, 0.f, 0.f), v(0.f, 1.f, 0.f);
    movement::buildTangentFrame(n, u, v);
    const float v0 = rig.sd.maxWalkSpeed;
    rig.state.velocity = u * v0;
    REQUIRE(glm::dot(rig.state.velocity, movement::kWorldUp) > 0.f);   // PREMISE: it really is up-slope

    rig.machineState.m_currentState  = DAttackState::Attacking;
    rig.machineState.m_attackEndTick = kTick0 + kN;

    for (std::uint32_t m = 1u; m <= kN; ++m)
    {
        rig.tick(kTick0 + m - 1u, stick(0.f, 0.f));

        const glm::vec3 channels = channelsOf(rig.state.velocity, n);
        INFO("up-slope tick " << m << ": support=" << int(rig.support())
             << " (0 Unsupported, 1 Supported)  channels=(" << channels.x << ", " << channels.y
             << ", " << channels.z << ")  expected u-channel="
             << (v0 * float(kN - m) / float(kN)) << "  v.z=" << rig.state.velocity.z);

        // 1. THE HOVER HOLD SURVIVES THE WHOLE SLIDE -- the row the `committed` arm would fail.
        REQUIRE(rig.support() == movement::SupportState::Supported);
        // 2. ...and the ramp is the same ramp, read through the file's own dual basis.
        REQUIRE(channels.x == Catch::Approx(v0 * float(kN - m) / float(kN)).margin(1e-2f));
        REQUIRE(channels.y == Catch::Approx(0.f).margin(1e-2f));
        // 3. ON the face: the slide follows the surface, it does not lift off it.
        REQUIRE(glm::dot(rig.state.velocity, n) == Catch::Approx(0.f).margin(1e-2f));
    }
    REQUIRE(rig.state.velocity == glm::vec3(0.f));

    // ⛔ CONTROL -- the SAME fixture, the SAME up-slope velocity, under a KNOCKBACK, detaches on
    // the second tick. This is G-07's recorded finding reproduced here so the row above is a
    // statement about the BRANCH and not about a fixture that could never detach.
    ScriptedRig launched;
    launched.query.scriptedSweeps = { floorHitAt(launched.sd.rideHeight, launched.probeLength(), n) };
    launched.state.bodyState.position = glm::vec3(0.f, 0.f, 96.f + launched.sd.rideHeight);
    launched.hit(HitReactionKind::Knockback, glm::vec2(1.f, 0.f), 0.f);
    launched.tick(1u, stick(0.f, 0.f));
    REQUIRE(launched.support() == movement::SupportState::Supported);
    launched.tick(2u, stick(0.f, 0.f));
    INFO("control (knockback on the same face): support=" << int(launched.support())
         << "  v.z=" << launched.state.velocity.z);
    REQUIRE(launched.support() == movement::SupportState::Unsupported);
}

// ---------------------------------------------------------------------------
// ⛔G-24's CASE. `m_attackEndTick` is WIRE STATE: a remote proxy can adopt a correction whose end
// tick is at or before the tick it is replaying, and an unsigned `m_attackEndTick - tick` would
// then wrap to ~2^32, making the factor `1 - 2^-32` -- a slide that never ends, silently, with
// nothing logged and nothing crashed. The guarded form saturates at `R = 1`, whose factor is 0,
// so the correct degrade is "stop now".
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.AttackSlideSaturatesWhenTheEndTickIsInThePast", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr std::uint32_t kTick = 10u;

    // ARM 1 -- the end tick is five ticks in the PAST. Unguarded this reads ~v0.
    ScriptedRig past;
    seatOnFlatGroundAtRideHeight(past);
    past.state.velocity = glm::vec3(past.sd.maxWalkSpeed, 0.f, 0.f);
    past.machineState.m_currentState  = DAttackState::Attacking;
    past.machineState.m_attackEndTick = kTick - 5u;
    // The stick is held IN the direction of travel, so neither the walk law nor a wrapped factor
    // could have produced the zero this case requires.
    past.tick(kTick, stick(1.f, 0.f));
    INFO("endTick=" << past.machineState.m_attackEndTick << " tick=" << kTick
         << " -> v=(" << past.state.velocity.x << ", " << past.state.velocity.y << ", "
         << past.state.velocity.z << ")");
    REQUIRE(past.state.velocity == glm::vec3(0.f));

    // ARM 2 -- the EXACT boundary. `m_attackEndTick == tick` is the first value the strict `>`
    // rejects, and it is the one an off-by-one in the write sites would produce.
    ScriptedRig boundary;
    seatOnFlatGroundAtRideHeight(boundary);
    boundary.state.velocity = glm::vec3(boundary.sd.maxWalkSpeed, 0.f, 0.f);
    boundary.machineState.m_currentState  = DAttackState::Attacking;
    boundary.machineState.m_attackEndTick = kTick;
    boundary.tick(kTick, stick(1.f, 0.f));
    REQUIRE(boundary.state.velocity == glm::vec3(0.f));

    // CONTROL -- one tick further out and the SAME fixture is a half-speed slide, so the two arms
    // above are a statement about the guard and not about a rig that always reads zero.
    ScriptedRig live;
    seatOnFlatGroundAtRideHeight(live);
    live.state.velocity = glm::vec3(live.sd.maxWalkSpeed, 0.f, 0.f);
    live.machineState.m_currentState  = DAttackState::Attacking;
    live.machineState.m_attackEndTick = kTick + 2u;
    live.tick(kTick, stick(1.f, 0.f));
    REQUIRE(live.state.velocity.x == Catch::Approx(0.5f * live.sd.maxWalkSpeed).margin(1e-3f));
}

// ---------------------------------------------------------------------------
// ⭐ RECOMPUTED EVERY TICK, NEVER CAPTURED AT ENTRY. The ramp is a pure function of
// `(velocity, tick, m_attackEndTick)` -- all three on the wire -- so when the end tick MOVES
// mid-slide (the chain, and task 31's dash-cancel-into-attack) the ramp re-targets and the stop
// is still exact at the NEW end. A form that captured `v0` or a deceleration on the entry tick
// would ramp to the old end and sit there, and it could not have been a seventh movement `State`
// member anyway -- `StateHasExactlySixMembers` forbids it.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.AttackSlideRetargetsWhenTheEndTickMoves", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr std::uint32_t kTick0     = 1u;
    constexpr std::uint32_t kN         = 10u;
    constexpr std::uint32_t kMovedEnd  = 20u;   // rewritten before tick 5

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    const float v0 = rig.sd.maxWalkSpeed;
    rig.state.velocity = glm::vec3(v0, 0.f, 0.f);
    rig.machineState.m_currentState  = DAttackState::Attacking;
    rig.machineState.m_attackEndTick = kTick0 + kN;

    for (std::uint32_t m = 1u; m <= 4u; ++m)
        rig.tick(kTick0 + m - 1u, stick(0.f, 0.f));

    const float midRamp = rig.state.velocity.x;
    INFO("mid-ramp after 4 of " << kN << " ticks: v.x=" << midRamp);
    REQUIRE(midRamp == Catch::Approx(v0 * 6.f / float(kN)).margin(1e-3f));

    rig.machineState.m_attackEndTick = kMovedEnd;
    for (std::uint32_t t = 5u; t <= kMovedEnd - 1u; ++t)
    {
        rig.tick(t, stick(0.f, 0.f));
        // From tick 5 the factors telescope to `(kMovedEnd - 1 - t) / (kMovedEnd - 5)`.
        const float expected = midRamp * float(kMovedEnd - 1u - t) / float(kMovedEnd - 5u);
        INFO("retargeted tick " << t << " (R=" << (kMovedEnd - t) << "): v.x="
             << rig.state.velocity.x << " expected " << expected);
        REQUIRE(rig.state.velocity.x == Catch::Approx(expected).margin(1e-3f));
    }

    // ...and it is still EXACTLY zero, at the NEW end.
    REQUIRE(rig.state.velocity == glm::vec3(0.f));
}

// ---------------------------------------------------------------------------
// ⛔ `locked` IS TESTED BEFORE `frozen`, for the reason ⛔G-23 already records for `committed`:
// guarding during an attack is blocked by `DAttackGuardSimulation`'s shape gate
// (`m_currentState != Idle`), which covers the whole swing, so freezing the body on the
// `holdGuard` bit would be a second, wrong mechanism. Swap the two branches in step 3 and the
// first row here reads 0 where it requires 90. The mirror of `HoldGuardDoesNotFreezeASlide`.
// ---------------------------------------------------------------------------
TEST_CASE("BrawlerMovement.HoldGuardDoesNotFreezeAnAttackSlide", "[BrawlerMovement]")
{
    using namespace movementTests;

    constexpr std::uint32_t kTick0 = 1u;
    constexpr std::uint32_t kN     = 10u;

    ScriptedRig rig;
    seatOnFlatGroundAtRideHeight(rig);
    const float v0 = rig.sd.maxWalkSpeed;
    rig.state.velocity = glm::vec3(v0, 0.f, 0.f);
    rig.machineState.m_currentState  = DAttackState::Attacking;
    rig.machineState.m_attackEndTick = kTick0 + kN;

    rig.tick(kTick0, stick(0.f, 0.f), movement::kInputFlagHoldGuard);
    INFO("holdGuard held through the first attacking tick: v.x=" << rig.state.velocity.x);
    REQUIRE(rig.state.velocity.x == Catch::Approx(v0 * 9.f / float(kN)).margin(1e-3f));

    rig.tick(kTick0 + 1u, stick(0.f, 0.f), movement::kInputFlagHoldGuard);
    REQUIRE(rig.state.velocity.x == Catch::Approx(v0 * 8.f / float(kN)).margin(1e-3f));

    // ⚠ AND THE FROZEN BIT STILL READS FROZEN -- G-23's second paragraph, now true of a guard-held
    // SLIDE as well as a guard-held knockback. Its only reader is the visualizer; recorded, not
    // treated as the answer to "did the model run".
    REQUIRE(rig.frozenBit());

    // CONTROL -- the same input with the machine Idle freezes, so the bit really is live in this
    // fixture and the rows above are a statement about the ORDER.
    ScriptedRig walking;
    seatOnFlatGroundAtRideHeight(walking);
    walking.state.velocity = glm::vec3(walking.sd.maxWalkSpeed, 0.f, 0.f);
    walking.tick(1u, stick(1.f, 0.f), movement::kInputFlagHoldGuard);
    REQUIRE(walking.state.velocity == glm::vec3(0.f));
}

#endif // WITH_LOW_LEVEL_TESTS

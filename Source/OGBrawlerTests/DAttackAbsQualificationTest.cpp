// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ⭐ THE TWO SIM HEADERS ARE INCLUDED FIRST, DELIBERATELY, AND MUST STAY FIRST.
//
// [movement-sim task 32] This file asks WHICH OVERLOAD the `abs(float)` calls inside
// og-brawler bind to. In `DAttackRadialSimulation.h` and `DAttackMachineSimulation.h`
// those calls sit inside function TEMPLATES, on operands (`float`, `glm::vec3`) that do
// not depend on the template parameter — so the name is NON-DEPENDENT and is looked up
// at the POINT OF DEFINITION, i.e. against whatever overloads this TU's preprocessor has
// pulled in by the time it reaches the header text, and against nothing that comes after.
// Putting them first therefore measures the MINIMAL include set — the configuration most
// likely to see only `::abs(int)`. If they bind the float overload HERE they bind it in
// every richer TU in this tree, because a later include can only ADD overloads. Move
// these below catch2 and the probe silently starts measuring catch2's include closure.
// ⚠ [movement-sim task 17, from the task-32 review's F-2] "MINIMAL" IS RELATIVE TO THIS TU'S
// OWN INCLUDE LIST, NOT ABSOLUTE. UBT force-includes `SharedPCH.Core….h` ahead of line 1, and
// the reviewer compiled a TU carrying nothing but the probe assertion on this same response
// file: it PASSES, so the PCH alone already supplies the float overload. Ordering the sim
// header first is still the right shape — it is the leanest set this TU can choose, and a
// later include can only ADD overloads — but it does not measure a bare `::abs(int)` world,
// and nothing downstream may be written as if it did.
//
// (This is the technique task 29 established in DAttackGuardSimulationTest.cpp. Ordering
// DAttackRadialSimulation.h before DAttackMachineSimulation.h costs nothing and changes
// nothing: the machine header includes the radial header itself, above its own `abs`
// site, so the machine site's overload set is the same either way.)
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"

// ⚠ DAttackCamera's three sites are in a .cpp, NOT a header — they are compiled once,
// inside the OGBrawler module, with an include set no test TU can influence. The camera
// cases below therefore pin BEHAVIOUR through the exported `dAttackCameraBehaviour::
// integrate` symbol rather than lookup context. That is strictly the more valuable half
// anyway: it is what a Godot/Jolt port would break.
#include "OGBrawler/DAttackCamera.h"

#include "catch_amalgamated.hpp"
#include "OGBrawler/DAttackCircle.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/DPID.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <vector>

// ---------------------------------------------------------------------------
// [movement-sim task 32] EVERY UNQUALIFIED FLOAT `abs(` IN og-brawler, PINNED.
//
// ⚠ THIS IS PORTABILITY HARDENING, NOT A BUG FIX. Every probe below was GREEN on
// the pre-fix tree: on this toolchain (MSVC 14.38) all five sites were ALREADY
// binding the float overload, exactly as task 29 measured for the guard site. No
// behaviour changed when they became `glm::abs`. Nothing here fixes a live defect.
//
// The exposure is OTHER toolchains. og-brawler targets a Godot port and a Jolt
// adapter, where the only `abs` visible at a header's point of definition may be
// C's `::abs(int)`. Under that overload a `float` argument is TRUNCATED on the way
// in and the result promoted back on the way out. `glm::abs` cannot resolve to an
// integer overload for a float argument, which is the whole reason for the change.
//
// A GREEN PROBE ALONE PROVES NOTHING, so each case below states the value the
// integer overload would produce, and each was run against a faithful mimic of that
// overload spliced into the real site — see impl/impl_notes_seam_32.md §3 for the
// RED output and the discriminating margins.
// ---------------------------------------------------------------------------

namespace dattackabstests
{

static constexpr float kDt  = 1.f / 60.f;
static constexpr float kEps = 1e-4f;

// ---------------------------------------------------------------------------
// Mocks. Deliberately local to this TU rather than shared with the task-29 guard
// file or the integrate3 file — both are held by other lanes, and a shared header
// would couple this file's fate to theirs.
// ---------------------------------------------------------------------------

struct MockPhysicsAdapter
{
    std::vector<glm::mat4> transforms;

    explicit MockPhysicsAdapter(std::size_t bodyCount)
        : transforms(bodyCount, glm::mat4(1.f))
    {}

    glm::mat4 getBodyTransform(BodyId id) const       { return transforms[id.value]; }
    void setBodyTransform(BodyId id, const glm::mat4& t) { transforms[id.value] = t; }

    void setBodyLinearVelocity(BodyId, const glm::vec3&)  {}
    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&)    {}
    void addBodyVelocityChange(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

struct MockSpatialQueryAdapter
{
    SpatialQueryReport report;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return report; }
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

// ===========================================================================
// RADIAL RIG — one collisionCheck tick, reduced to "did the swing register a hit".
//
// `collisionCheck` lives in an ANONYMOUS namespace inside the header, so it cannot
// be called directly; the rig drives it through the public `integrate` and reads
// the DerivedState it fills.
//
// The fixture is chosen so that everything except `lengthAlongRotationAxis` is held
// constant between the two overloads:
//   * rotationAxis == defaultUp() (+Z) and initialAimAngle == 0 about +Z, so the
//     initial rotation is the identity and
//         glm::dot(hitDirection, worldSequenceRotationAxis) == hitDirection.z
//     exactly — the site under test reduces to `abs(hitDirection.z)`.
//   * the sequence's ONE segment spans [0, 7) rad, i.e. wider than a full turn, so
//     every direction resolves to segment 0 / Damaging. The segment lookup therefore
//     cannot become a second, uncontrolled discriminator: whatever the overload does
//     to the projected direction, both arms match the current segment.
//   * the surviving gate is `lengthAlongRotationAxis < halfThickness`, which is a
//     BOOLEAN. The probe reads 1 hit vs 0 hits — the widest margin available.
//
// ⚠ `DerivedState`'s default constructor seeds attackHits/guardHits with FOUR
// default-constructed entries, and collisionCheck's first line early-returns at
// `size() >= 4`. The rig clears both, which is what the live sim's deactivate()
// path does before the first damaging tick. Without the clear every case here would
// pass vacuously (0 hits under BOTH overloads) — see the positive control below.
// ===========================================================================

struct RadialTickResult
{
    std::size_t attackHits = 0;
    std::size_t guardHits  = 0;
};

static RadialTickResult radialTick(float hitZ, float halfThickness)
{
    using namespace dAttackRadialSimulation;

    std::vector<DAttackRadialSequence> sequences;
    sequences.emplace_back(
        std::vector<DAttackRadialSequencePoint>{
            { 0.0f, 0.0f, DAttackRadialSequenceState::Damaging },
            { 0.2f, 7.0f, DAttackRadialSequenceState::Idle } },
        0.1f,
        DAttackRadialSequence::defaultUp());

    // DAttackCircle takes THICKNESS; the sim gates on getHalfThickness().
    DAttackCircle circle(8u, 50.f, 100.f, halfThickness * 2.f, false, 1.f);
    StaticData   staticData(sequences, circle);

    InitialConditions ic{};
    ic.initialAimAngle        = 0.f;
    ic.initialAimRotationAxis = glm::vec3(0.f, 0.f, 1.f);
    ic.activeAttackSequence   = 0u;
    ic.activeRootBodyId       = 0u;

    State st{};
    st.attackTimer      = 0.f;
    // Equal to ic.activeAttackSequence on purpose: that is what makes integrate SKIP
    // setInitialConditions, which would otherwise overwrite the body transform the
    // fixture depends on.
    st.currenSequenceId = 0u;

    SimulationComposite<InitialConditions, State> composite(ic, st);
    auto deps = makeDependencies<Dependencies>(composite);

    MockPhysicsAdapter physics{ 2 };            // 0 = weapon (own), 1 = capsule (parent)
    MockSpatialQueryAdapter query{};
    SpatialQueryHit hit{};
    hit.objectPosition   = glm::vec3(75.f, 0.f, hitZ);
    hit.bodyId           = BodyId{ 5u };
    hit.rootBodyId       = BodyId{ 5u };
    hit.objectCategories = CollisionCategories::single(collisionCategory::body);
    query.report.hits.push_back(hit);

    PlayerInput pi{};
    pi.aimDirection = glm::vec3(1.f, 0.f, 0.f);

    IntegrationUtils<MockPhysicsAdapter, MockSpatialQueryAdapter> utils{ kDt, physics, query };
    AllInput<MockPhysicsAdapter, MockSpatialQueryAdapter> allInput{ pi, utils };

    RuntimeBindings bindings{};
    bindings.ownBodyId        = BodyId{ 0u };
    bindings.parentBodyId     = BodyId{ 1u };
    bindings.attachmentOffset = glm::vec3(0.f);      // root ends the tick at the origin
    bindings.shapeIds         = {};
    bindings.queryVolumeIds   = { QueryVolumeId{ 1u } };

    DerivedState derived{};
    derived.editAttackHits().clear();
    derived.editGuardHits().clear();

    integrate(kDt, allInput, staticData, deps, bindings, derived);

    return RadialTickResult{ derived.getAttackHits().size(), derived.getGuardHits().size() };
}

// ===========================================================================
// CAMERA RIG — one dAttackCameraBehaviour::integrate tick.
//
// Everything the three camera sites decide is readable from public getters on
// DAttackCameraState, so no transform round-trip is needed to observe them:
//   * :42 and :45 both land in editPitchPIDState().setAdjustment(...)
//   * :55 lands in setCameraBoomLength(...)
//
// The PID is pure-proportional (p=1, i=0, d=0), so its adjustment is exactly the
// error `targetPitch - currentPitch` and every number below is closed form.
// ===========================================================================

struct CameraTickResult
{
    float adjustment = 0.f;
    float boomLength = 0.f;
};

static CameraTickResult cameraTick(const glm::vec2& mouseAxis, float startPitch, float targetPitch)
{
    const DPIDSettings pidSettings(1.f, 0.f, 0.f);
    const DAttackCameraInput input(glm::vec3(0.f), mouseAxis, /*blockLook*/ true,
                                   targetPitch, pidSettings);

    DAttackCameraState state;
    state.setCameraBoomTransform(glm::mat4_cast(glm::quat(glm::vec3(0.f, startPitch, 0.f))));

    dAttackCameraBehaviour::integrate(kDt, input, state);

    return CameraTickResult{ state.getPitchPIDState().getAdjustment(),
                             state.getCameraBoomLength() };
}

// ===========================================================================
// MACHINE RIG — one dAttackMachineSimulation::integrate tick from Idle with the
// left attack held, which is the shortest path into the anonymous-namespace
// setRadialSimulationInitialConditions() where the :167 site lives. The site's
// only observable is the rotation axis it writes into the radial InitialConditions.
// ===========================================================================

static dAttackRadialSimulation::InitialConditions machineTick(const glm::vec3& aim)
{
    dAttackMachineSimulation::PlayerInput pi{};
    pi.aimDirection = aim;
    pi.attackLeft   = true;

    MockPhysicsAdapter physics{ 2 };
    const std::vector<DAttackRadialSequence> sequences;   // never indexed on this path
    const brawlerProjectileSimulation::StaticData projectileStaticData(1.f, 1.f, 1.f, 1.f, 1.f);

    dAttackMachineSimulation::IntegrationUtils<MockPhysicsAdapter>
        utils{ kDt, sequences, physics, projectileStaticData };
    dAttackMachineSimulation::AllInput<MockPhysicsAdapter> allInput{ pi, utils };

    const dAttackRadialSimulation::State attackState{};
    dAttackRadialSimulation::InitialConditions ic{};
    dAttackMachineSimulation::State machineState{};       // Idle by default

    dAttackMachineSimulation::integrate(kDt, allInput, attackState, ic, machineState);

    REQUIRE(machineState.m_currentState == DAttackState::Attacking);
    return ic;
}

} // namespace dattackabstests

// ===========================================================================
// ⭐ 1. DAttackRadialSimulation.h:460 — `abs(glm::dot(hitDirection, worldSequenceRotationAxis))`
//
// THE HIGHEST-RISK SITE. `hitDirection` is a raw world-space delta and the axis is a
// unit vector, so the dot product is a SIGNED DISTANCE along the rotation axis, in
// centimetres — NOT a cosine. (The backlog's census describes it as "a dot product in
// (-1,1)"; that is true only for hits within a centimetre of the swing plane. The
// exposure is broader than the census says and does not depend on sub-unit values:
// an integer overload throws away the FRACTION at every magnitude, which is why the
// second SECTION below uses a 5.7 cm offset.)
//
// The value gates `lengthAlongRotationAxis < halfThickness` — whether a struck body
// counts as inside the swing disc at all. Under `::abs(int)` the whole expression is
//     (float) abs( (int) glm::dot(...) )
// so the gate is compared against a truncated distance and hits that the swing plainly
// misses vertically start landing.
//
// Margin: the observable is a HIT COUNT, 0 vs 1 — categorical, not a tolerance.
// ===========================================================================

TEST_CASE("DAttackAbs.RadialAxisDistanceKeepsItsFraction", "[DAttack][AbsQualification]")
{
    using namespace dattackabstests;

    SECTION("POSITIVE CONTROL — a hit inside the disc registers, so 0 is never vacuous")
    {
        // 0.3 cm off the swing plane, half-thickness 0.5 cm: inside under either
        // overload. This arm does NOT discriminate and is not meant to; it exists so
        // that the "0 hits" in the arms below cannot be read as a rig that never
        // records anything (which is exactly what an un-cleared DerivedState gives).
        const RadialTickResult r = radialTick(/*hitZ*/ 0.3f, /*halfThickness*/ 0.5f);
        INFO("attackHits = " << r.attackHits << " (expected 1 under BOTH overloads)");
        REQUIRE(r.attackHits == 1u);
        REQUIRE(r.guardHits  == 0u);
    }

    SECTION("0.6 cm above the swing plane, half-thickness 0.5 cm — OUT of the disc")
    {
        // float overload: |0.6| = 0.6, and 0.6 < 0.5 is false -> no hit.
        // int overload:   abs((int)0.6) = 0, and 0 < 0.5 is TRUE -> a phantom hit on a
        //                 body the swing passes cleanly under.
        const RadialTickResult r = radialTick(/*hitZ*/ 0.6f, /*halfThickness*/ 0.5f);
        INFO("attackHits = " << r.attackHits << " (float overload: 0, int overload: 1)");
        REQUIRE(r.attackHits == 0u);
    }

    SECTION("0.6 cm BELOW the plane — the same, and it pins the sign handling too")
    {
        // A truncating overload collapses -0.6 to 0 just as it collapses +0.6, so this
        // arm fails under `::abs(int)` for the same reason. It is here because a
        // hand-rolled `v < 0 ? -v : v` on the WRONG type is the realistic way this
        // regresses, and only a negative operand distinguishes that from a no-op.
        const RadialTickResult r = radialTick(/*hitZ*/ -0.6f, /*halfThickness*/ 0.5f);
        INFO("attackHits = " << r.attackHits << " (float overload: 0, int overload: 1)");
        REQUIRE(r.attackHits == 0u);
    }

    SECTION("5.7 cm above the plane, half-thickness 5.5 cm — truncation at REAL scale")
    {
        // ⭐ The arm that shows the exposure is not confined to sub-unit values, which
        // is where the backlog's census stops. float: 5.7 < 5.5 is false -> no hit.
        // int: abs((int)5.7) = 5, and 5 < 5.5 is TRUE -> a phantom hit. Every swing in
        // the game works at this scale; the census's "(-1,1)" framing understates it.
        const RadialTickResult r = radialTick(/*hitZ*/ 5.7f, /*halfThickness*/ 5.5f);
        INFO("attackHits = " << r.attackHits << " (float overload: 0, int overload: 1)");
        REQUIRE(r.attackHits == 0u);
    }
}

// ===========================================================================
// ⭐ 2. DAttackCamera.cpp:45 — `abs(normalizedCameraAxis.x) - abs(normalizedCameraAxis.y)`
//
// THE OTHER HIGHEST-RISK SITE, and the one with no float-side escape hatch:
// `normalizedCameraAxis` is a NORMALIZED glm::vec2, so BOTH components are in [-1,1]
// by construction and an integer overload truncates BOTH to 0. The scale factor
// becomes 0 - 0 = 0 and the pitch PID adjustment is multiplied to nothing on EVERY
// frame — the camera's pitch control silently stops working, with no NaN, no assert
// and no discontinuity to notice.
//
// Fixture: mouse axis (0.8, 0.6) — already unit length, so cameraAxis == its own
// normalization and |x| > |y|, which is what routes past the :42 branch into this one.
// Start pitch 0.6, target 1.0, p=1 -> the PID adjustment entering the line is 0.4.
//   float overload: 0.4 * (0.8 - 0.6) = 0.08
//   int overload:   0.4 * (0   - 0  ) = 0
// Margin 0.08 against a 1e-4 tolerance — 800x.
//
// ⚠ Independent of :42 by construction: |x| < |y| is false under the float overload
// and `0 < 0` is false under the integer one, so BOTH send this fixture down the else
// branch. This case measures :45 alone.
// ===========================================================================

TEST_CASE("DAttackAbs.CameraPitchAdjustmentScalesByAxisMagnitudes", "[DAttack][AbsQualification]")
{
    using namespace dattackabstests;

    const CameraTickResult r = cameraTick(glm::vec2(0.8f, 0.6f), /*startPitch*/ 0.6f,
                                          /*targetPitch*/ 1.f);
    INFO("adjustment = " << r.adjustment << " (float overload: 0.08, int overload: 0)");
    REQUIRE(r.adjustment == Catch::Approx(0.08f).margin(kEps));

    // Stated as an assertion rather than a comment: the whole hazard is that the int
    // overload makes this identically zero, and zero is also what a lot of other
    // breakage looks like.
    REQUIRE(r.adjustment != Catch::Approx(0.f).margin(kEps));
}

// ===========================================================================
// 3. DAttackCamera.cpp:42 — `abs(cameraAxis.x) < abs(cameraAxis.y)`
//
// The branch that decides whether a mostly-VERTICAL stick/mouse gesture suppresses the
// pitch adjustment entirely. `cameraAxis` is normalized one line earlier in every
// reachable path, so both magnitudes are in [0,1] and an integer overload compares
// `0 < 0` — always false. The suppression branch becomes UNREACHABLE and a vertical
// gesture starts scaling the adjustment by a negative factor instead of zeroing it.
//
// Fixture: mouse axis (0.6, 0.8) — |x| < |y|.
//   float overload: takes the suppression branch -> adjustment = 0
//   int overload:   falls through to :45 -> 0.4 * (0.6 - 0.8) = -0.08
// Margin 0.08, and a SIGN change on top of it.
// ===========================================================================

TEST_CASE("DAttackAbs.CameraAxisDominanceComparesMagnitudes", "[DAttack][AbsQualification]")
{
    using namespace dattackabstests;

    const CameraTickResult r = cameraTick(glm::vec2(0.6f, 0.8f), /*startPitch*/ 0.6f,
                                          /*targetPitch*/ 1.f);
    INFO("adjustment = " << r.adjustment << " (float overload: 0, int overload: -0.08)");
    REQUIRE(r.adjustment == Catch::Approx(0.f).margin(kEps));

    // The mirrored fixture, so "0" above cannot be an artefact of a rig that never
    // writes an adjustment at all. Same tick, |x| > |y|, non-zero result.
    const CameraTickResult dominantX = cameraTick(glm::vec2(0.8f, 0.6f), 0.6f, 1.f);
    REQUIRE(dominantX.adjustment == Catch::Approx(0.08f).margin(kEps));
}

// ===========================================================================
// 4. DAttackCamera.cpp:55 — `abs(clampedPitch)`
//
// `clampedPitch` is `std::clamp(currentPitch, 0.f, targetPitch)` — already non-negative,
// so the `abs` is defensive rather than load-bearing. It is still the same portability
// hazard: an integer overload truncates it, and for the sub-1-radian pitches this camera
// actually uses that means distanceFactor collapses to 1 and the boom snaps to its
// minimum length on every frame.
//
// Fixture: currentPitch 0.6 rad, targetPitch 1.0 rad.
//   float overload: distanceFactor = 1 - 0.6/1.0 = 0.4 -> boom = 900 - 500*0.4 = 700
//   int overload:   distanceFactor = 1 -   0/1.0 = 1.0 -> boom = 900 - 500*1.0 = 400
// Margin 300 world units.
//
// This assertion also transitively pins the euler round-trip the fixture depends on: a
// boom length of 700 is only reachable if the seeded transform really reads back as a
// 0.6 rad pitch.
// ===========================================================================

TEST_CASE("DAttackAbs.CameraBoomLengthTracksFractionalPitch", "[DAttack][AbsQualification]")
{
    using namespace dattackabstests;

    const CameraTickResult r = cameraTick(glm::vec2(0.8f, 0.6f), /*startPitch*/ 0.6f,
                                          /*targetPitch*/ 1.f);
    INFO("boomLength = " << r.boomLength << " (float overload: 700, int overload: 400)");
    REQUIRE(r.boomLength == Catch::Approx(700.f).margin(1e-2f));
}

// ===========================================================================
// 5. DAttackMachineSimulation.h:167 — `abs(abs(aimDot) - 1.f) < 0.0001f`
//
// BYTE-IDENTICAL to the guard site task 29 fixed, and it feeds the same near-pole
// epsilon band: inside the band the initial aim rotation axis is forced to +Z instead
// of being derived from `cross(defaultForward, aim)`, which keeps `normalize` away from
// the zero vector. An integer overload collapses the expression to `|aimDot| == 1`
// EXACTLY and the band disappears.
//
// Fixture: aim 0.01 rad off +X with NEGATIVE y, so |aimDot| = cos(0.01) = 0.99995 and
// ||aimDot| - 1| = 5e-5 — inside the 1e-4 band with a factor of two to spare.
//   float overload: band live  -> axis = defaultUp        -> axis.z = +1
//   int overload:   band dead  -> axis = normalize(cross(+X, aim)) = -Z -> axis.z = -1
// Margin 2.0 against 1e-4.
//
// ⚠ NON-DISCRIMINATING BY CONSTRUCTION, so nobody adds one thinking it strengthens the
// case: a POSITIVE-y near-pole aim gives +Z under BOTH overloads, because the cross
// product there already points at defaultUp. Only a negative-y aim separates them.
// ===========================================================================

TEST_CASE("DAttackAbs.MachineNearPoleAimTakesTheEpsilonBandBranch", "[DAttack][AbsQualification]")
{
    using namespace dattackabstests;

    const float theta = 0.01f;
    const float c = std::cos(theta);
    const float s = std::sin(theta);

    SECTION("0.01 rad off +X with NEGATIVE y — inside the band")
    {
        const auto ic = machineTick(glm::vec3(c, -s, 0.f));
        INFO("axis.z = " << ic.initialAimRotationAxis.z
             << " (float overload: +1, int overload: -1)");
        REQUIRE(ic.initialAimRotationAxis.z == Catch::Approx(1.f).margin(kEps));
        REQUIRE(ic.initialAimAngle == Catch::Approx(theta).margin(1e-3f));
    }

    SECTION("POSITIVE CONTROL — a far-field aim derives a real axis under both overloads")
    {
        // 90 degrees off +X: ||aimDot| - 1| = 1, far outside the band either way, so the
        // axis comes from the cross product. -Y aims give -Z; the arm exists to show the
        // rig reads a genuine axis rather than always reporting +Z.
        const auto right = machineTick(glm::vec3(0.f, -1.f, 0.f));
        REQUIRE(right.initialAimRotationAxis.z == Catch::Approx(-1.f).margin(kEps));

        const auto left = machineTick(glm::vec3(0.f, 1.f, 0.f));
        REQUIRE(left.initialAimRotationAxis.z == Catch::Approx(1.f).margin(kEps));
    }
}

#endif // WITH_LOW_LEVEL_TESTS

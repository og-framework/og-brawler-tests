// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/BrawlerHitDetectionSystem.h"
#include "OGBrawler/DAttackCircle.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <vector>

// ---------------------------------------------------------------------------
// [movement-sim task 33] THE SWING-PLANE PROJECTION MUST BE SYMMETRIC ABOUT THE PLANE.
//
// ⚠ UNLIKE task 32, WHICH WAS DELIBERATELY BEHAVIOUR-NEUTRAL, THIS IS A REAL BUG FIX
// WITH A GAMEPLAY-VISIBLE EFFECT. Every case below was RED on the pre-fix header, with
// the asymmetric values quoted in each arm.
//
// `DAttackRadialSimulation.h` projects a hit onto the swing plane with
//     hitDirectionOnRotationPlane = hitDirection - lengthAlongRotationAxis * axis
// and `lengthAlongRotationAxis` is `glm::abs(dot(hitDirection, axis))`. A projection
// needs the SIGNED dot. For a hit ABOVE the plane the two agree, so the line looked
// correct forever; for a hit BELOW it the axial component is DOUBLED AWAY from the
// plane instead of removed — (75, 0, -35) became (75, 0, -70), not (75, 0, 0).
//
// `hitDistance` is therefore inflated for below-plane hits only, and it gates BOTH ends
// of the annulus (`> innerRadius && < outerRadius`), so the strike zone below the plane
// was pushed bodily INWARD: the swing lost outward reach below the plane and gained
// phantom hits inside the inner hole. Both halves are pinned below.
//
// ⚠ THE HALF-THICKNESS GATE'S USE OF THE UNSIGNED VALUE IS CORRECT AND IS NOT TOUCHED —
// a distance FROM a plane has no sign. `SwingPlaneHalfThicknessGateStaysUnsigned` is the
// arm that says so; it was GREEN before the fix and is GREEN after, which is the point.
// ---------------------------------------------------------------------------

namespace dattackradialswingplanetests
{

static constexpr float kDt = 1.f / 60.f;

// ---------------------------------------------------------------------------
// Mocks — deliberately local to this TU. Task 32's file carries an equivalent pair and
// task 29's another; sharing a header would couple this file's fate to lanes that are
// still open.
// ---------------------------------------------------------------------------

struct MockPhysicsAdapter
{
    std::vector<glm::mat4> transforms;

    explicit MockPhysicsAdapter(std::size_t bodyCount)
        : transforms(bodyCount, glm::mat4(1.f))
    {}

    glm::mat4 getBodyTransform(BodyId id) const          { return transforms[id.value]; }
    void setBodyTransform(BodyId id, const glm::mat4& t)  { transforms[id.value] = t; }

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
// RIG — one detection tick, reduced to "did the swing register a hit".
//
// [og-netcode-v2-field-defects task 9] Detection is no longer inside the radial (it was its
// anonymous-namespace collisionCheck). The rig runs the radial's public `integrate`, then
// `brawlerHitDetection::detectRadialHits` on the state integrate left -- the production
// tick's two steps, in that order -- and reads the DerivedState the detector fills. Three fixture
// choices hold everything except the projection constant (they are the same choices
// task 32's radial rig makes, for the same reasons):
//
//   * the sequence rotation axis is defaultUp() (+Z) and initialAimAngle is 0 about +Z,
//     so the initial rotation is the identity and
//         glm::dot(hitDirection, worldSequenceRotationAxis) == hitDirection.z
//     exactly. The site under test reduces to arithmetic on `hitZ`.
//   * the ONE segment spans [0, 7) rad — wider than a full turn — so every direction
//     resolves to segment 0 / Damaging. The segment lookup cannot become a second,
//     uncontrolled discriminator. (It would otherwise be one: the projection's z-skew
//     leaves `glm::normalize(hitDirectionOnRotationPlane)` with a non-unit XY part, and
//     `glm::orientedAngle` on a non-unit operand reports a bogus angle.)
//   * `rootTranslation` is the origin — `integrate`'s attachment block sets the own body
//     to parentPosition + attachmentOffset, both zero — so `hitDirection` IS the hit's
//     world position, and the numbers below are the ones in the comments.
//
// ⚠ `DerivedState()` used to seed its hit vectors with FOUR default entries (until
// movement-sim task 34) and the detector's cap check early-returned on them. The per-swing
// ledger is the synced `State::hitTargets` since og-netcode-v2-field-defects task 27 and
// starts empty here; the positive controls still prove a 0 is the geometry. One pass
// registers into `hitsThisTick`, and that count is what `radialBodyHits` returns.
// ===========================================================================

static std::size_t radialBodyHits(float planarX,
                                    float hitZ,
                                    float halfThickness,
                                    float innerRadius,
                                    float outerRadius)
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
    DAttackCircle circle(8u, innerRadius, outerRadius, halfThickness * 2.f, false, 1.f);
    StaticData   staticData(sequences, circle);

    InitialConditions ic{};
    ic.initialAimAngle        = 0.f;
    ic.initialAimRotationAxis = glm::vec3(0.f, 0.f, 1.f);
    ic.activeAttackSequence   = 0u;

    State st{};
    st.attackTimer      = 0.f;
    // Equal to activeAttackSequence on purpose: that makes integrate SKIP
    // setInitialConditions, which would otherwise rewrite the body transform.
    st.currenSequenceId = 0u;

    SimulationComposite<InitialConditions, State> composite(ic, st);
    auto deps = makeDependencies<Dependencies>(composite);

    MockPhysicsAdapter physics{ 2 };            // 0 = weapon (own), 1 = capsule (parent)
    MockSpatialQueryAdapter query{};
    SpatialQueryHit hit{};
    hit.objectPosition   = glm::vec3(planarX, 0.f, hitZ);
    hit.bodyId           = BodyId{ 5u };
    hit.rootBodyId       = BodyId{ 5u };
    hit.objectCategories = CollisionCategories::single(collisionCategory::body);
    query.report.hits.push_back(hit);

    PlayerInput pi{};
    pi.aimDirection = glm::vec3(1.f, 0.f, 0.f);

    IntegrationUtils<MockPhysicsAdapter> utils{ kDt, physics };
    AllInput<MockPhysicsAdapter> allInput{ pi, utils };

    RuntimeBindings bindings{};
    bindings.ownBodyId        = BodyId{ 0u };
    bindings.parentBodyId     = BodyId{ 1u };
    bindings.attachmentOffset = glm::vec3(0.f);      // root ends the tick at the origin
    bindings.shapeIds         = {};
    bindings.queryVolumeIds   = { QueryVolumeId{ 1u } };

    DerivedState derived{};

    integrate(kDt, allInput, staticData, deps, bindings, derived);
    // [og-netcode-v2-field-defects task 9] The production tick's second step: detection runs
    // AFTER the radial's integrate, on the state it left, exactly as
    // brawlerHitDetection::System::preIntegrate does for every character on the next tick (task 20).
    brawlerHitDetection::detectRadialHits(kDt, staticData, composite.get<InitialConditions>(),
        composite.get<State>(), bindings, derived, physics, query,
        [](BodyId root) { return root == BodyId{ 5u } ? SimCharacterId{ 5u } : SimCharacterId::None; });

    return derived.getHitsThisTick().size();
}

// ---------------------------------------------------------------------------
// `hitDistance` is a local inside an anonymous-namespace function, so it cannot be read
// directly. It IS measurable, though, because it is compared against a value the fixture
// owns: the outer gate is `hitDistance < getOuterRadius()`, so the SMALLEST outer radius
// that still admits the hit IS `hitDistance`. Bisecting on the outer radius therefore
// reads the quantity the acceptance criterion is about, rather than a proxy for it.
//
// The inner radius is held at 0.5 cm so the `> innerRadius` half of the gate is
// satisfied for every distance considered here (all are >= 45 cm) and the bisection
// measures the OUTER threshold alone. 40 iterations over [0.5, 512] resolve to ~5e-13.
// ---------------------------------------------------------------------------
static float measureHitDistance(float planarX, float hitZ, float halfThickness)
{
    float lo = 0.5f;      // no hit at this outer radius (distance is always >= 45)
    float hi = 512.f;     // hit at this outer radius
    for (int i = 0; i < 40; ++i)
    {
        const float mid = 0.5f * (lo + hi);
        if (radialBodyHits(planarX, hitZ, halfThickness, /*innerRadius*/ 0.5f, mid) == 1u)
            hi = mid;
        else
            lo = mid;
    }
    return hi;
}

} // namespace dattackradialswingplanetests

// ===========================================================================
// ⭐ 1. THE ACCEPTANCE CRITERION, MEASURED DIRECTLY.
//
// Two hits that are exact mirror images through the swing plane — (75, 0, +35) and
// (75, 0, -35), both comfortably inside a 40 cm half-thickness — must be the same
// distance from the swing AXIS, because reflecting a point through the plane cannot
// change how far it is from the axis lying in that plane. Both are 75 cm out.
//
// PRE-FIX (measured, 2026-09-04):
//     above =  75.0000   below = 102.5914     <- sqrt(75^2 + 70^2); the axial component
//                                                was doubled away instead of removed
// POST-FIX:
//     above =  75.0000   below =  75.0000
//
// The equality assertion is the one the acceptance criterion asks for; the two absolute
// assertions are what stop a future regression from satisfying it by making BOTH sides
// equally wrong.
// ===========================================================================

TEST_CASE("DAttackRadial.MirroredHitsAreTheSameDistanceFromTheSwingAxis",
          "[DAttack][HitDetection][RadialSwingPlane]")
{
    using namespace dattackradialswingplanetests;

    const float above = measureHitDistance(/*planarX*/ 75.f, /*hitZ*/ +35.f, /*halfThickness*/ 40.f);
    const float below = measureHitDistance(/*planarX*/ 75.f, /*hitZ*/ -35.f, /*halfThickness*/ 40.f);

    INFO("above = " << above << ", below = " << below
         << "  (pre-fix: 75.0 vs 102.5914 — the below-plane hit reads 36.8% further out)");

    REQUIRE(above == Catch::Approx(below).margin(1e-3f));
    REQUIRE(above == Catch::Approx(75.f).margin(1e-3f));
    REQUIRE(below == Catch::Approx(75.f).margin(1e-3f));
}

// ===========================================================================
// ⭐ 2. THE SAME DEFECT AS THE GAMEPLAY OBSERVABLE — a hit count, 0 vs 1.
//
// The bisection above is the precise statement; this case is the one a designer would
// recognise. Annulus 50..100 cm, half-thickness 40 cm, so a hit 35 cm off the plane is
// well inside the disc and the ONLY thing deciding it is the radius gate.
//
//   OUTWARD REACH — (75, 0, ±35), true distance 75, inside 50..100:
//       above: 1 hit before and after.
//       below: pre-fix distance 102.59 > 100 -> 0 hits. The swing MISSED a body it
//              plainly overlapped. Post-fix: 1 hit.
//
//   INWARD PHANTOM — (45, 0, ±20), true distance 45, INSIDE the 50 cm inner hole:
//       above: 0 hits before and after (correctly ignored).
//       below: pre-fix distance sqrt(45^2 + 40^2) = 60.21, which lands inside the
//              annulus -> 1 PHANTOM hit on a body the swing passes around. Post-fix: 0.
//
// The two arms fail in OPPOSITE directions, which is why both are here: a "fix" that
// merely widened or narrowed the annulus would flip one of them.
// ===========================================================================

TEST_CASE("DAttackRadial.SwingReachIsSymmetricAboveAndBelowThePlane",
          "[DAttack][HitDetection][RadialSwingPlane]")
{
    using namespace dattackradialswingplanetests;

    SECTION("POSITIVE CONTROL — an in-plane hit inside the annulus registers")
    {
        // Not discriminating, and not meant to be: it exists so that a 0 below cannot be
        // read as a rig that never records anything (which is what an un-cleared
        // DerivedState gives — see the header note).
        REQUIRE(radialBodyHits(75.f, 0.f, 40.f, 50.f, 100.f) == 1u);
    }

    SECTION("OUTWARD REACH — 35 cm ABOVE the plane, 75 cm out: hit")
    {
        // Green before AND after. Paired with the arm below, it localises the defect to
        // the below-plane case rather than to the fixture.
        REQUIRE(radialBodyHits(75.f, +35.f, 40.f, 50.f, 100.f) == 1u);
    }

    SECTION("OUTWARD REACH — 35 cm BELOW the plane, 75 cm out: hit (was a MISS)")
    {
        INFO("pre-fix: 0 hits — hitDistance read 102.59 against a 100 cm outer radius");
        REQUIRE(radialBodyHits(75.f, -35.f, 40.f, 50.f, 100.f) == 1u);
    }

    SECTION("INWARD HOLE — 20 cm ABOVE the plane, 45 cm out: no hit")
    {
        REQUIRE(radialBodyHits(45.f, +20.f, 40.f, 50.f, 100.f) == 0u);
    }

    SECTION("INWARD HOLE — 20 cm BELOW the plane, 45 cm out: no hit (was a PHANTOM hit)")
    {
        INFO("pre-fix: 1 hit — hitDistance read 60.21, inside the 50..100 annulus");
        REQUIRE(radialBodyHits(45.f, -20.f, 40.f, 50.f, 100.f) == 0u);
    }
}

// ===========================================================================
// 3. ⛔ THE HALF-THICKNESS GATE STILL USES THE UNSIGNED DISTANCE — AND MUST.
//
// `lengthAlongRotationAxis` feeds TWO consumers. The projection needs the signed dot;
// the gate `lengthAlongRotationAxis < getHalfThickness()` needs the UNSIGNED one,
// because a distance FROM a plane has no sign. Task 33 introduced a separate signed
// value for the projection and left the gate's operand alone.
//
// This case is the proof, and it is the one arm here that was GREEN BEFORE THE FIX AND
// IS GREEN AFTER — deliberately. Its four arms are a mirrored pair straddling a 5.5 cm
// half-thickness, so they are only all satisfied if the gate compares MAGNITUDES:
//   * had the gate been switched to the signed value, every hit BELOW the plane would
//     pass it (a negative is less than any positive half-thickness) and the -5.7 arm
//     would go red;
//   * had the abs been dropped from both, -5.3 and -5.7 would both register.
// The annulus is 50..100 with the hit 75 cm out, so the radius gate is satisfied on
// every arm and the thickness gate is the sole discriminator.
// ===========================================================================

TEST_CASE("DAttackRadial.SwingPlaneHalfThicknessGateStaysUnsigned",
          "[DAttack][HitDetection][RadialSwingPlane]")
{
    using namespace dattackradialswingplanetests;

    // 5.3 cm from the plane, half-thickness 5.5 — inside the disc on both sides.
    REQUIRE(radialBodyHits(75.f, +5.3f, 5.5f, 50.f, 100.f) == 1u);
    REQUIRE(radialBodyHits(75.f, -5.3f, 5.5f, 50.f, 100.f) == 1u);

    // 5.7 cm from the plane — outside the disc on both sides. The negative arm is the
    // one that goes red if the gate is ever handed the signed value.
    REQUIRE(radialBodyHits(75.f, +5.7f, 5.5f, 50.f, 100.f) == 0u);
    REQUIRE(radialBodyHits(75.f, -5.7f, 5.5f, 50.f, 100.f) == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS

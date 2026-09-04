// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ⭐ THE GUARD HEADER IS INCLUDED FIRST, DELIBERATELY, AND IT MUST STAY FIRST.
//
// The `abs` pin below (F-G4) asks WHICH OVERLOAD an unqualified `abs(float)`
// inside DAttackGuardSimulation.h binds to. `aimDot` is a plain `const float` in a
// template that does not depend on it, so the name is NON-DEPENDENT and is looked
// up at the POINT OF DEFINITION — i.e. against whatever overloads are in scope
// where this TU's preprocessor reaches the header text, and against nothing that
// comes after. Putting the header first therefore measures the MINIMAL include
// set: the configuration most likely to see only `::abs(int)`. If it binds the
// float overload HERE, it binds it in every richer TU in this tree too, because a
// later include can only ADD overloads. Move this include below catch2 and the
// probe silently starts measuring catch2's include closure instead of the sim's.
#include "OGBrawler/DAttackGuardSimulation.h"

#include "catch_amalgamated.hpp"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/DAttackCircle.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationSerialization.h"
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
// [movement-sim task 29] THE GUARD SUB-SIMULATION'S INTEGRATE, PINNED.
//
// The guard body is a 40 cm query-only sphere hung under the capsule at
// (0,0,30). Its pose is not integrated: the POSITION is snapped to the parent
// every tick and the ROTATION is rebuilt from this tick's aim input. Both are
// closed-form functions of state that is already on the wire, which is why task
// 29 takes `State::bodyState` OFF the wire (the projectile pattern).
//
// Consumers read the guard's FACING through the physics adapter, not through
// State: BrawlerProjectileSimulation.h reads `getBodyTransform(hit.bodyId)[0]`
// as the guard forward when a projectile overlaps the guard shape, and
// DAttackRadialSimulation.h does the same for the radial block test. So the
// thing worth pinning is the TRANSFORM THIS INTEGRATE WRITES — column 0 is the
// guard's forward, column 3 its translation — and that is what every case here
// asserts.
// ---------------------------------------------------------------------------

namespace dattackguardtests
{

// ---------------------------------------------------------------------------
// Mock physics adapter — records every transform read and write, per body, so
// the F-G3 "one write, one parent read, no own read" claim is COUNTED rather
// than asserted in prose. BodyId.value indexes `bodies`.
// ---------------------------------------------------------------------------
struct MockPhysicsAdapter
{
    std::vector<glm::mat4>    transforms;
    mutable std::vector<int>  getCounts;
    std::vector<int>          setCounts;

    explicit MockPhysicsAdapter(std::size_t bodyCount)
        : transforms(bodyCount, glm::mat4(1.f))
        , getCounts(bodyCount, 0)
        , setCounts(bodyCount, 0)
    {}

    glm::mat4 getBodyTransform(BodyId id) const
    {
        ++getCounts[id.value];
        return transforms[id.value];
    }
    void setBodyTransform(BodyId id, const glm::mat4& t)
    {
        ++setCounts[id.value];
        transforms[id.value] = t;
    }

    void setBodyLinearVelocity(BodyId, const glm::vec3&)  {}
    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    // Present so this mock satisfies PhysicsBodyAdapter both WITH and WITHOUT
    // task 3b's force seam in the concept: extra members never break a concept.
    void addBodyAcceleration(BodyId, const glm::vec3&)    {}
    void addBodyVelocityChange(BodyId, const glm::vec3&)  {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return PhysicsBodyState{}; }
};

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

// ---------------------------------------------------------------------------
// Mock spatial query adapter — records shape enable/disable so the F-G5
// "unconditional every tick" toggle is observable.
// ---------------------------------------------------------------------------
struct MockSpatialQueryAdapter
{
    std::vector<ShapeId> enabled;
    std::vector<ShapeId> disabled;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return SpatialQueryReport{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId id)  { enabled.push_back(id); }
    void disableShape(ShapeId id) { disabled.push_back(id); }
};

static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr float    kDt              = 1.f / 60.f;
static constexpr uint32_t kOwnBodyValue    = 0u;
static constexpr uint32_t kParentBodyValue = 1u;
static const     glm::vec3 kAttachOffset   { 0.f, 0.f, 30.f };
static const     glm::vec3 kParentPos      { 100.f, -250.f, 7.f };

// Catch2's float comparison, at the tolerance the discriminating cases need. The
// abs pin separates +/- 0.0099998, so 1e-4 is two orders of margin below the gap.
static constexpr float kEps = 1e-4f;

static dAttackGuardSimulation::RuntimeBindings makeBindings()
{
    dAttackGuardSimulation::RuntimeBindings b{};
    b.ownBodyId        = BodyId{ kOwnBodyValue };
    b.parentBodyId     = BodyId{ kParentBodyValue };
    b.attachmentOffset = kAttachOffset;
    b.shapeIds         = { ShapeId{ 7u } };
    b.queryVolumeIds   = {};
    return b;
}

// One integrate tick. Returns the transform the guard body ends the tick with,
// and (through the out-params) the guard State the composite ends with.
struct TickResult
{
    glm::mat4                        bodyTransform{ 1.f };
    dAttackGuardSimulation::State    state{};
    int                              ownGets    = 0;
    int                              parentGets = 0;
    int                              ownSets    = 0;
    std::vector<ShapeId>             enabled;
    std::vector<ShapeId>             disabled;
};

static TickResult tick(const glm::vec3& aim,
                       DAttackState machineState = DAttackState::Idle,
                       const glm::mat4& startingOwnTransform = glm::mat4(1.f))
{
    using namespace dAttackGuardSimulation;

    DAttackCircle circle{ 8u, 50.f, 100.f, 10.f, false, 1.f };
    StaticData    staticData{ circle };
    DerivedState  derived{};

    dAttackMachineSimulation::State machine{};
    machine.m_currentState = machineState;

    SimulationComposite<InitialConditions, State, dAttackMachineSimulation::State>
        composite(InitialConditions{}, State{}, machine);

    auto deps = makeDependencies<Dependencies>(composite);

    MockPhysicsAdapter physics{ 2 };
    physics.transforms[kParentBodyValue] = glm::translate(glm::mat4(1.f), kParentPos);
    physics.transforms[kOwnBodyValue]    = startingOwnTransform;
    // Reset the counters so the seeding above is not counted as sim traffic.
    physics.getCounts.assign(2, 0);
    physics.setCounts.assign(2, 0);

    MockSpatialQueryAdapter query{};

    PlayerInput pi{};
    pi.aimDirection = aim;

    IntegrationUtils<MockPhysicsAdapter, MockSpatialQueryAdapter> utils{ kDt, physics, query };
    AllInput<MockPhysicsAdapter, MockSpatialQueryAdapter> allInput{ pi, utils };

    const auto bindings = makeBindings();
    integrate(kDt, allInput, staticData, deps, bindings, derived);

    TickResult r;
    r.bodyTransform = physics.transforms[kOwnBodyValue];
    r.state         = composite.get<State>();
    r.ownGets       = physics.getCounts[kOwnBodyValue];
    r.parentGets    = physics.getCounts[kParentBodyValue];
    r.ownSets       = physics.setCounts[kOwnBodyValue];
    r.enabled       = query.enabled;
    r.disabled      = query.disabled;
    return r;
}

static glm::vec3 forwardOf(const TickResult& r) { return glm::vec3(r.bodyTransform[0]); }
static glm::vec3 originOf (const TickResult& r) { return glm::vec3(r.bodyTransform[3]); }

} // namespace dattackguardtests

// ===========================================================================
// 1. FACING — the four cardinal aims.
//
// The guard's local forward is column 0 of the transform integrate writes. For a
// planar aim the rotation is `angle = acos(dot(aim, +X))` about
// `normalize(cross(+X, aim))`, so a correct implementation reproduces the aim
// EXACTLY in column 0 for every direction in the XY plane. Asserting all four
// cardinals rather than one is what makes a sign error in the axis (the failure
// F-G4 was originally reported as) visible: +Y and -Y differ only in the sign of
// the cross product, and a build that dropped it would return +Y for both.
// ===========================================================================

TEST_CASE("DAttackGuard.FacesTheFourCardinalAims", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    struct Row { const char* name; glm::vec3 aim; };
    const Row rows[] = {
        { "+X (forward, the aimEqualsForward degenerate branch)", glm::vec3( 1.f,  0.f, 0.f) },
        { "+Y (left)",                                           glm::vec3( 0.f,  1.f, 0.f) },
        { "-X (backward, the ANTIPARALLEL degenerate branch)",    glm::vec3(-1.f,  0.f, 0.f) },
        { "-Y (right)",                                          glm::vec3( 0.f, -1.f, 0.f) },
    };

    for (const Row& row : rows)
    {
        INFO("aim = " << row.name);
        const TickResult r = tick(row.aim);
        const glm::vec3 fwd = forwardOf(r);

        REQUIRE(fwd.x == Catch::Approx(row.aim.x).margin(kEps));
        REQUIRE(fwd.y == Catch::Approx(row.aim.y).margin(kEps));
        REQUIRE(fwd.z == Catch::Approx(0.f).margin(kEps));

        // ...and the body sat where the attachment says, in the same tick.
        const glm::vec3 origin = originOf(r);
        REQUIRE(origin.x == Catch::Approx(kParentPos.x + kAttachOffset.x).margin(kEps));
        REQUIRE(origin.y == Catch::Approx(kParentPos.y + kAttachOffset.y).margin(kEps));
        REQUIRE(origin.z == Catch::Approx(kParentPos.z + kAttachOffset.z).margin(kEps));
    }
}

// ===========================================================================
// 2. THE DEGENERATE BRANCH, AND WHY IT EXISTS.
//
// `cross(+X, aim)` is the ZERO VECTOR when the aim is parallel or antiparallel
// to +X, and `normalize(vec3(0))` is a division by zero — NaN into the body
// transform, and from there into every overlap the guard takes part in. The
// `aimEqualsForward` test is the guard against exactly that, and `acos` returning
// an UNSIGNED angle is why testing |aimDot| (rather than aimDot) is correct:
// at the antiparallel pole the angle is pi and any axis in the XY plane's normal
// gives the same result, so `defaultUp` is a legitimate substitute.
// ===========================================================================

TEST_CASE("DAttackGuard.DegenerateAimsNeverProduceNaN", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    for (const glm::vec3 aim : { glm::vec3(1.f, 0.f, 0.f), glm::vec3(-1.f, 0.f, 0.f) })
    {
        INFO("aim = (" << aim.x << ", " << aim.y << ", " << aim.z << ")");
        const TickResult r = tick(aim);
        const glm::vec3 fwd = forwardOf(r);
        REQUIRE(std::isfinite(fwd.x));
        REQUIRE(std::isfinite(fwd.y));
        REQUIRE(std::isfinite(fwd.z));
        REQUIRE(fwd.x == Catch::Approx(aim.x).margin(kEps));
    }
}

TEST_CASE("DAttackGuard.ZeroAimFallsBackToDefaultForward", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    // A value-initialised aim would reach normalize(vec3(0)). The length guard
    // substitutes +X instead — the same reason guard PlayerInput::zero() is
    // (0,0,1) and not PlayerInput{}.
    const TickResult r = tick(glm::vec3(0.f));
    const glm::vec3 fwd = forwardOf(r);
    REQUIRE(std::isfinite(fwd.x));
    REQUIRE(fwd.x == Catch::Approx(1.f).margin(kEps));
    REQUIRE(fwd.y == Catch::Approx(0.f).margin(kEps));
}

// ===========================================================================
// ⭐ 3. F-G4 — WHICH `abs` OVERLOAD BINDS. The pin, and why this shape.
//
// `DAttackGuardSimulation.h` tests `abs(abs(aimDot) - 1.f) < 0.0001f` on a
// `float`. If the only `abs` in scope at the header's point of definition were
// `::abs(int)`, the expression would evaluate as
//
//     abs( (int)( (float)abs((int)aimDot) - 1.f ) )
//
// which for any |aimDot| strictly inside (0,1) truncates to 0, gives -1.f, and
// comes back as 1 — i.e. `aimEqualsForward` would collapse to `|aimDot| == 1`
// EXACTLY, and the epsilon band would vanish.
//
// ⭐ THE ONLY OBSERVABLE DIFFERENCE IS INSIDE THAT BAND, so that is where this
// case aims. |aimDot| > 0.9999 means an aim within ~0.81 degrees of +/-X:
//   * float overload -> the band is live -> axis = defaultUp (+Z), and the sign
//     of cross(+X, aim) is NOT consulted, so the tiny residual angle comes back
//     with acos's UNSIGNED sign: forward.y is POSITIVE for a negative-y aim.
//   * int overload   -> the band is dead -> axis = normalize(cross(+X, aim)) =
//     -Z for a negative-y aim, and forward.y comes back NEGATIVE.
// The two answers are +0.0099998 and -0.0099998: a 0.02 gap, 200x the tolerance.
//
// ⚠ SO THIS CASE PINS A MIRROR, ON PURPOSE, AND IT IS NOT A BUG. Inside the band
// the aim is at most 0.81 degrees off the pole, the guard is a SPHERE, and the
// band exists to keep `normalize(cross(...))` away from the zero vector (case 2).
// Trading <=0.81 degrees of facing for a NaN-free normalize is the whole point of
// the branch. What must not drift is WHICH branch a near-pole aim takes, because
// that is a function of overload resolution and therefore of the toolchain.
//
// ⚠ NON-DISCRIMINATING BY CONSTRUCTION, stated so nobody adds one and thinks it
// strengthens the case: a POSITIVE-y near-pole aim gives forward.y = +0.0099998
// under BOTH overloads (the cross product is +Z there, which is defaultUp). Only
// the negative-y arms below can tell the two apart.
// ===========================================================================

TEST_CASE("DAttackGuard.NearPoleAimTakesTheEpsilonBandBranch", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    // theta = 0.01 rad -> |aimDot| = cos(0.01) = 0.99995, i.e. ||aimDot| - 1| =
    // 5e-5, inside the header's 1e-4 band with a factor of two to spare.
    const float theta = 0.01f;
    const float c = std::cos(theta);
    const float s = std::sin(theta);

    SECTION("0.01 rad off +X, with NEGATIVE y")
    {
        const TickResult r = tick(glm::vec3(c, -s, 0.f));
        const glm::vec3 fwd = forwardOf(r);
        INFO("forward.y = " << fwd.y << " (float overload: +" << s
             << ", int overload: -" << s << ")");
        REQUIRE(fwd.y == Catch::Approx(+s).margin(kEps));
    }

    SECTION("0.01 rad off -X, with NEGATIVE y")
    {
        const TickResult r = tick(glm::vec3(-c, -s, 0.f));
        const glm::vec3 fwd = forwardOf(r);
        INFO("forward.y = " << fwd.y << " (float overload: +" << s
             << ", int overload: -" << s << ")");
        REQUIRE(fwd.y == Catch::Approx(+s).margin(kEps));
    }
}

// ===========================================================================
// 4. F-G3 — ONE WRITE, ONE PARENT READ, NO OWN READ.
//
// Before task 29 the tick did an attachment write (parent read + own read + own
// write) and then an aim write (a SECOND own read + a second own write) that
// re-used the translation the first write had just put there. Two adapter
// round-trips per character per tick that compose into one transform.
//
// This is COUNTED, not inspected: a re-introduced `getBodyTransform(ownBodyId)`
// is a physics-thread proxy lookup that nothing else in the suite would notice.
// ===========================================================================

TEST_CASE("DAttackGuard.WritesTheBodyTransformExactlyOnce", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    const TickResult idle = tick(glm::vec3(0.f, 1.f, 0.f), DAttackState::Idle);
    INFO("idle: ownGets=" << idle.ownGets << " parentGets=" << idle.parentGets
         << " ownSets=" << idle.ownSets);
    REQUIRE(idle.ownSets    == 1);
    REQUIRE(idle.ownGets    == 0);
    REQUIRE(idle.parentGets == 1);

    // The non-Idle tick too. The shapes are disabled on that tick, so nothing can
    // QUERY the guard — but the body must still follow the parent, because the
    // first Idle tick after an attack reads this pose through the adapter.
    const TickResult attacking = tick(glm::vec3(0.f, 1.f, 0.f), DAttackState::Attacking);
    INFO("attacking: ownGets=" << attacking.ownGets << " parentGets=" << attacking.parentGets
         << " ownSets=" << attacking.ownSets);
    REQUIRE(attacking.ownSets    == 1);
    REQUIRE(attacking.ownGets    == 0);
    REQUIRE(attacking.parentGets == 1);
}

TEST_CASE("DAttackGuard.BodyFollowsTheParentInEveryMachineState", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    // The TRANSLATION is the half of the pose that must not change with the
    // machine state, and the half a consumer reads. Pinned for every enumerator
    // so a restructured integrate cannot quietly drop the follow on one branch.
    for (const DAttackState ms : { DAttackState::Idle, DAttackState::Attacking,
                                   DAttackState::GuardFlinch, DAttackState::HitFlinch })
    {
        INFO("machine state ordinal = " << static_cast<int>(ms));
        const TickResult r = tick(glm::vec3(1.f, 0.f, 0.f), ms);
        const glm::vec3 origin = originOf(r);
        REQUIRE(origin.x == Catch::Approx(kParentPos.x + kAttachOffset.x).margin(kEps));
        REQUIRE(origin.y == Catch::Approx(kParentPos.y + kAttachOffset.y).margin(kEps));
        REQUIRE(origin.z == Catch::Approx(kParentPos.z + kAttachOffset.z).margin(kEps));
    }
}

// ===========================================================================
// 5. F-G5 — the shape toggle is UNCONDITIONAL, and that is the design.
//
// Making it edge-triggered would need a "last written" bit that must survive
// resim; off-wire derived state resets on nothing, so a replay that changes the
// machine state would desync the toggle. The write is idempotent
// (SetQueryEnabled on the PT), so it is done every tick instead.
// ===========================================================================

TEST_CASE("DAttackGuard.ShapesToggleEveryTickFromTheMachineState", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    const TickResult idle = tick(glm::vec3(1.f, 0.f, 0.f), DAttackState::Idle);
    REQUIRE(idle.enabled.size()  == 1u);
    REQUIRE(idle.disabled.empty());

    for (const DAttackState ms : { DAttackState::Attacking, DAttackState::GuardFlinch,
                                   DAttackState::HitFlinch })
    {
        INFO("machine state ordinal = " << static_cast<int>(ms));
        const TickResult r = tick(glm::vec3(1.f, 0.f, 0.f), ms);
        REQUIRE(r.disabled.size() == 1u);
        REQUIRE(r.enabled.empty());
    }
}

// ===========================================================================
// 6. F-G1/F-G2/F-G7 — the guard State is OFF THE WIRE, and the transient agrees
//    with what was written.
//
// Every field of the guard's pose is a closed-form function of state that is
// already replicated (the capsule's bodyState and this tick's aim), so it
// carries no information the wire does not already have. `SerializableFields`
// is empty; the member survives as a LOCAL transient for the capture loop (which
// needs a PhysicsBodyState lvalue via PhysicsDeclaration::bodyStateOf) and for
// the legacy-arcs visualization.
// ===========================================================================

TEST_CASE("DAttackGuard.StateIsEntirelyOffTheWire", "[DAttack][DAttackGuardSimulation]")
{
    REQUIRE(syncSize<dAttackGuardSimulation::State>() == 0u);
    REQUIRE(syncSize<dAttackGuardSimulation::InitialConditions>() == 0u);
    static_assert(syncSize<dAttackGuardSimulation::State>() == 0u,
        "The guard sub-simulation put a field back on the wire. Its whole pose is "
        "derivable from the capsule's bodyState plus this tick's aim input, which is "
        "why task 29 took it off; anything that genuinely needs replicating belongs "
        "in a sub-simulation that OWNS it.");
}

TEST_CASE("DAttackGuard.TransientBodyStateMatchesTheWrittenTransform", "[DAttack][DAttackGuardSimulation]")
{
    using namespace dattackguardtests;

    // integrate keeps the transient in step with the transform it just wrote, so
    // the visualization and the post-solve capture cannot disagree WITHIN a tick.
    const TickResult r = tick(glm::vec3(0.f, -1.f, 0.f));

    const glm::vec3 origin = originOf(r);
    REQUIRE(r.state.bodyState.position.x == Catch::Approx(origin.x).margin(kEps));
    REQUIRE(r.state.bodyState.position.y == Catch::Approx(origin.y).margin(kEps));
    REQUIRE(r.state.bodyState.position.z == Catch::Approx(origin.z).margin(kEps));

    const glm::vec3 fwdFromTransform = forwardOf(r);
    const glm::vec3 fwdFromTransient = glm::mat3_cast(r.state.bodyState.rotation) * glm::vec3(1.f, 0.f, 0.f);
    REQUIRE(fwdFromTransient.x == Catch::Approx(fwdFromTransform.x).margin(kEps));
    REQUIRE(fwdFromTransient.y == Catch::Approx(fwdFromTransform.y).margin(kEps));
    REQUIRE(fwdFromTransient.z == Catch::Approx(fwdFromTransform.z).margin(kEps));
}

#endif // WITH_LOW_LEVEL_TESTS

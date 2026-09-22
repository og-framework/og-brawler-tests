// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/DAttackCircle.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/DAttackSequenceId.h"
#include "OGBrawler/OGBrawlerLog.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <glm/geometric.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/trigonometric.hpp>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// [og-netcode-v2-field-defects task 5] THE `[Radial.setInitialConditions]` LINE MUST
// CARRY THE AIM, AND A DEGENERATE AIM MUST REACH THE LOG AS `nan`.
//
// This is a diagnostic instrument, not a feature. The remote-swing-pose bug report asks
// the user to tell two candidate causes of a world-aligned remote swing apart from ONE
// PIE session:
//   (A) innocent -- a second PIE window with no aim input falls back to camera-forward,
//                   so the aim is legitimately cardinal (0 / +-1.5708).
//   (B) real     -- a press resolved against the NEUTRAL input gives aim (0,0,1), whose
//                   XY projection is the zero vector, and the initial conditions are NaN.
// The discriminator IS the printed value, so a "defensive" isfinite / normalize / clamp
// guard on the logged path would destroy the evidence the instrument exists to capture.
// The `...KeepsANonFiniteAimAsNan` case below is the fence against a future one.
//
// SOURCE OF THE LOGGED VALUES (the other half of the acceptance criterion): the line
// READS `InitialConditions::initialAimAngle` / `::initialAimRotationAxis` -- the same two
// fields `setInitialConditions` hands to `glm::rotate` a few lines later to build the
// swing's own initial rotation. Nothing is recomputed, so the log cannot disagree with
// what the swing does.
// ---------------------------------------------------------------------------

namespace dattackradialaimlogtests
{

static constexpr float kDt = 1.f / 60.f;

// ---------------------------------------------------------------------------
// Mocks -- deliberately local to this TU, matching the convention of the sibling radial
// test files (task 32's, task 33's and task 34's each carry their own pair).
// ---------------------------------------------------------------------------

struct MockPhysicsAdapter
{
    std::vector<glm::mat4> transforms;

    explicit MockPhysicsAdapter(std::size_t bodyCount)
        : transforms(bodyCount, glm::mat4(1.f))
    {}

    glm::mat4 getBodyTransform(BodyId id) const           { return transforms[id.value]; }
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
// RIG -- one radial integrate that TAKES the setInitialConditions branch, with the
// OGBLOG_G sink captured.
//
//   * `setInitialConditions` sits in an ANONYMOUS namespace inside the header, so the rig
//     drives it through the public `integrate`. The branch is
//     `state.currenSequenceId != initialConditions.activeAttackSequence`, so the rig puts
//     the state at InvalidAttackSequenceId (idle) and the initial conditions at sequence
//     0 -- the Idle -> Attacking edge, which is exactly the edge the instrument fires on.
//   * `ogblog::g_sink` is a process-global; it is saved and restored around the run so
//     this TU cannot leak a dangling sink into any other case in the suite.
//   * The query report is left EMPTY. These cases are about a formatted string, not about
//     hits, and an empty report keeps collisionCheck (which runs later in the same tick)
//     from becoming a second source of failure.
// ===========================================================================

static std::string captureSetInitialConditionsLine(float aimAngle, const glm::vec3& aimAxis)
{
    using namespace dAttackRadialSimulation;

    std::vector<DAttackRadialSequence> sequences;
    sequences.emplace_back(
        std::vector<DAttackRadialSequencePoint>{
            { 0.0f, 0.0f, DAttackRadialSequenceState::Damaging },
            { 0.2f, 7.0f, DAttackRadialSequenceState::Idle } },
        0.1f,
        DAttackRadialSequence::defaultUp());

    DAttackCircle circle(8u, 90.f, 300.f, 70.f, false, 1.f);
    StaticData    staticData(sequences, circle);

    InitialConditions ic{};
    ic.initialAimAngle        = aimAngle;
    ic.initialAimRotationAxis = aimAxis;
    ic.activeAttackSequence   = 0u;
    ic.activeRootBodyId       = 0u;

    State st{};
    st.attackTimer      = 0.f;
    st.currenSequenceId = InvalidAttackSequenceId;   // idle -> the edge fires

    SimulationComposite<InitialConditions, State> composite(ic, st);
    auto deps = makeDependencies<Dependencies>(composite);

    MockPhysicsAdapter      physics{ 2 };            // 0 = weapon (own), 1 = capsule (parent)
    MockSpatialQueryAdapter query{};

    PlayerInput pi{};
    pi.aimDirection = glm::vec3(1.f, 0.f, 0.f);

    IntegrationUtils<MockPhysicsAdapter, MockSpatialQueryAdapter> utils{ kDt, physics, query };
    AllInput<MockPhysicsAdapter, MockSpatialQueryAdapter> allInput{ pi, utils };

    RuntimeBindings bindings{};
    bindings.ownBodyId        = BodyId{ 0u };
    bindings.parentBodyId     = BodyId{ 1u };
    bindings.attachmentOffset = glm::vec3(0.f);
    bindings.shapeIds         = {};
    bindings.queryVolumeIds   = { QueryVolumeId{ 1u } };

    DerivedState derived{};

    std::vector<std::string> lines;
    auto previousSink = ::ogblog::g_sink;
    ::ogblog::setGlobal([&lines](const char* msg) { lines.emplace_back(msg); });

    integrate(kDt, allInput, staticData, deps, bindings, derived);

    ::ogblog::setGlobal(std::move(previousSink));

    for (const std::string& line : lines)
    {
        if (line.rfind("[Radial.setInitialConditions]", 0) == 0)
            return line;
    }
    return std::string{};
}

} // namespace dattackradialaimlogtests

// ===========================================================================
// 1. THE FIELDS ARE THERE, AND THEY ARE THE VALUES THE SWING USES.
//
// The format the bug report's runbook asks for is
//     seq=%u aimAngle=%.4f axis=(%.3f,%.3f,%.3f)
// A 45-degree aim about +Z is pi/4 = 0.785398 -> "0.7854" at four decimals, and the axis
// renders at three. Asserting the rendered SUBSTRINGS rather than a parsed float is
// deliberate: the acceptance criterion is about what a human reads in a PIE log.
// ===========================================================================

TEST_CASE("DAttackRadial.SetInitialConditionsLogCarriesTheAimAngleAndAxis",
          "[DAttack][RadialAimLog]")
{
    using namespace dattackradialaimlogtests;

    const std::string line =
        captureSetInitialConditionsLine(glm::pi<float>() * 0.25f, glm::vec3(0.f, 0.f, 1.f));

    INFO("line: " << line);
    REQUIRE_FALSE(line.empty());                                   // the edge really fired
    CHECK(line.find("seq=0") != std::string::npos);
    CHECK(line.find("aimAngle=0.7854") != std::string::npos);
    CHECK(line.find("axis=(0.000,0.000,1.000)") != std::string::npos);
}

// ===========================================================================
// 2. A NON-FINITE AIM MUST PRINT AS `nan` -- THE WHOLE POINT OF THE INSTRUMENT.
//
// The non-finite angle is not hand-made: it is produced by replaying the upstream
// expression from DAttackMachineSimulation.h::setRadialSimulationInitialConditions
//     normalize(vec3(aim.x, aim.y, 0))  ->  acos(dot(that, (1,0,0)))
// with the aim a neutral-resolved press leaves behind, (0,0,1) -- whose XY projection is
// the zero vector. That replay is a PREMISE (REQUIRE, not CHECK): the day glm or the
// toolchain stops producing a non-finite angle there, cause (B) has changed shape and
// this case must go RED and say so, rather than quietly testing a synthetic NaN.
//
// The sources are `volatile` on purpose. This target compiles /fp:fast (read the per-TU
// `.obj.rsp`, do not infer it), under which MSVC is licensed to assume no NaN exists and
// may fold a constant-sourced comparison away.
//
// ⭐ ONLY THE ANGLE CARRIES THE SIGNAL, AND THAT IS A MEASUREMENT. Upstream picks the aim
// axis through `glm::abs(glm::abs(aimDot) - 1.f) < 0.0001f`, a comparison against NaN --
// precisely the shape /fp:fast is allowed to rewrite. Measured here, 2026-09-19: that
// predicate is TRUE for a NaN aimDot on this toolchain, so a degenerate aim takes the
// `defaultUp` branch and its AXIS renders as a clean cardinal (0.000,0.000,1.000) -- it is
// indistinguishable from the innocent cause (A). The `aimAngle` field is the ONLY place
// the nan surfaces, which is why the runbook must read that field first.
// ===========================================================================

TEST_CASE("DAttackRadial.SetInitialConditionsLogKeepsANonFiniteAimAsNan",
          "[DAttack][RadialAimLog]")
{
    using namespace dattackradialaimlogtests;

    const glm::vec3 defaultForward(1.f, 0.f, 0.f);
    const glm::vec3 defaultUp(0.f, 0.f, 1.f);

    volatile float neutralAimX = 0.f;
    volatile float neutralAimY = 0.f;
    const glm::vec3 degenerateAim =
        glm::normalize(glm::vec3(neutralAimX, neutralAimY, 0.f));
    const float aimDot          = glm::dot(degenerateAim, defaultForward);
    const float degenerateAngle = glm::acos(aimDot);

    REQUIRE(std::isnan(degenerateAngle));   // premise: cause (B) still produces a NaN aim

    // The upstream axis branch, replayed. MEASURED on this toolchain, 2026-09-19:
    // aimEqualsForward is TRUE for a NaN aimDot. IEEE-754 says every comparison against
    // NaN is false, so this pins a /fp:fast FACT, not a spelling -- and it goes RED the
    // day the flag changes, which is exactly the day the axis field below changes shape.
    const bool aimEqualsForward = glm::abs(glm::abs(aimDot) - 1.f) < 0.0001f;
    INFO("aimEqualsForward: " << aimEqualsForward);
    CHECK(aimEqualsForward);
    const glm::vec3 degenerateAxis = aimEqualsForward
        ? defaultUp
        : glm::normalize(glm::cross(defaultForward, degenerateAim));

    const std::string line = captureSetInitialConditionsLine(degenerateAngle, degenerateAxis);

    INFO("line: " << line);
    REQUIRE_FALSE(line.empty());
    CHECK(line.find("seq=0") != std::string::npos);

    // ⭐ WHAT THE RUNBOOK'S READER MUST NOT BE FOOLED BY. Because the branch above is
    // taken, the AXIS of a degenerate aim renders as a clean cardinal (0,0,1) -- it looks
    // exactly like the innocent cause (A). Only the ANGLE carries the nan.
    CHECK(line.find("axis=(0.000,0.000,1.000)") != std::string::npos);

    // The angle field itself must carry the nan. MSVC renders the indefinite NaN that
    // `0 * inf` produces as "-nan(ind)"; a quiet NaN as "nan". Both begin "nan" after the
    // optional sign, and both are what the runbook's reader is looking for.
    const bool angleIsNan = line.find("aimAngle=nan")  != std::string::npos
                         || line.find("aimAngle=-nan") != std::string::npos;
    CHECK(angleIsNan);

    // A guard, clamp or normalize on the logged path would land here instead.
    CHECK(line.find("aimAngle=0.0000") == std::string::npos);
}

#endif // WITH_LOW_LEVEL_TESTS

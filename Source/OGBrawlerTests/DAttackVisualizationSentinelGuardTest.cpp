// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/DAttackRadialVisualization.h"
#include "OGBrawler/DAttackAimVisualization.h"
// [Task 36] kHadoukenSequenceSentinel relocated here; this TU references it directly
// (transitively visible via the viz headers, but made explicit).
#include "OGBrawler/DAttackSequenceId.h"

// ---------------------------------------------------------------------------
// T24 regression: viz consumers must guard kHadoukenSequenceSentinel.
//
// When a Hadouken fires, dAttackMachineSimulation::integrate3 writes
// kHadoukenSequenceSentinel (= UINT_MAX - 1) into the radial IC's
// activeAttackSequence for the 1+ ticks the projectile owns the attack. The
// radial sim early-returns on the sentinel, but the visualization layer used to
// index staticData.getAttackSequences()[activeAttackSequence] unconditionally
// (or guarded only against InvalidAttackSequenceId, which the sentinel passes),
// dereferencing ~4 billion entries out of bounds → EXCEPTION_ACCESS_VIOLATION in
// PIE (DAttackRadialVisualization.h:195 → getAttackSegment).
//
// The fix routes every sequence-table lookup through
// dAttackRadialSimulation::isRealAttackSequence(id). This test drives the viz
// functions with the sentinel set and an EMPTY sequence table: pre-fix this would
// index OOB; post-fix the guard skips the lookup entirely, so no segment mesh is
// emitted and nothing crashes.
// ---------------------------------------------------------------------------

namespace vizsentineltests
{

// Counters shared across copies of the functor (the viz takes the renderer by
// value and copies it internally), so the functor holds a pointer to these.
struct DrawCounts
{
    int line = 0;
    int point = 0;
    int sphere = 0;
    int circleArc = 0;
    int mesh = 0;   // a "segment draw" — drawSegmentSolid() emits exactly one mesh
};

struct MockRenderer
{
    DrawCounts* counts = nullptr;

    void drawLine(const glm::vec3&, const glm::vec3&, unsigned int, float) { ++counts->line; }
    void drawPoint(const glm::vec3&) { ++counts->point; }
    void drawSphere(const glm::vec3&, float, unsigned int, float) { ++counts->sphere; }
    void drawCircleArc(const glm::vec3&, const glm::vec3&, float, float, unsigned int, float) { ++counts->circleArc; }
    void drawMesh(const std::vector<glm::vec3>&, const std::vector<unsigned int>&, unsigned int) { ++counts->mesh; }
};

struct MockLogger
{
    void logVec3(const char*, const glm::vec3&) {}
    void logInt(const char*, int) {}
};

// Build a radial StaticData with an intentionally EMPTY sequence table; the
// caller keeps the backing vector + circle alive (StaticData stores references).
struct RadialFixture
{
    std::vector<DAttackRadialSequence> sequences;            // empty on purpose
    DAttackCircle circle{ 8u, 50.f, 100.f, 10.f, false, 1.f };
    dAttackRadialSimulation::StaticData staticData{ sequences, circle };

    dAttackRadialSimulation::State state;
    dAttackRadialSimulation::DerivedState derived;
    dAttackRadialSimulation::InitialConditions ic;

    RadialFixture()
    {
        // The crash trigger: IC carries the Hadouken sentinel, not a real index.
        ic.activeAttackSequence = kHadoukenSequenceSentinel;
        state.bodyState.position = glm::vec3(0.f);
        state.bodyState.rotation = glm::quat(1.f, 0.f, 0.f, 0.f);
    }
};

} // namespace vizsentineltests

// ---------------------------------------------------------------------------
// dAttackRadialVisualization::visualize must not index the (empty) sequence
// table when the sentinel is active, and must emit no segment mesh.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.VizSentinelGuard.RadialVisualizeNoCrash", "[DAttack]")
{
    using namespace vizsentineltests;

    RadialFixture fx;
    DrawCounts counts;
    MockRenderer renderer{ &counts };
    MockLogger logger;

    dAttackRadialVisualization::State vizState;
    dAttackRadialVisualization::Input<MockRenderer, MockLogger> input(
        1.f / 60.f, glm::vec3(1.f, 0.f, 0.f), renderer, logger);

    dAttackRadialVisualization::visualize(
        input, fx.state, fx.ic, fx.derived, fx.staticData, vizState);

    // Guard held: no attack-segment mesh emitted for the sentinel tick.
    REQUIRE(counts.mesh == 0);
}

// ---------------------------------------------------------------------------
// dAttackRadialVisualization::visualize2 — same guarantee (this is the function
// that crashed in PIE at DAttackRadialVisualization.h:195).
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.VizSentinelGuard.RadialVisualize2NoCrash", "[DAttack]")
{
    using namespace vizsentineltests;

    RadialFixture fx;
    DrawCounts counts;
    MockRenderer renderer{ &counts };
    MockLogger logger;

    dAttackRadialVisualization::State vizState;
    dAttackRadialVisualization::Input<MockRenderer, MockLogger> input(
        1.f / 60.f, glm::vec3(1.f, 0.f, 0.f), renderer, logger);

    dAttackRadialVisualization::visualize2(
        input, fx.state, fx.ic, fx.derived, fx.staticData, vizState);

    REQUIRE(counts.mesh == 0);
}

// ---------------------------------------------------------------------------
// dAttackAimVisualization::visualize — the aim viz binds (but does not use) the
// sequence; the bare index would still read OOB. Guard must skip it.
// ---------------------------------------------------------------------------
TEST_CASE("DAttack.VizSentinelGuard.AimVisualizeNoCrash", "[DAttack]")
{
    using namespace vizsentineltests;

    RadialFixture fx;
    DrawCounts counts;
    MockRenderer renderer{ &counts };
    MockLogger logger;

    dAttackAimVisualization::State vizState;
    dAttackAimVisualization::Input<MockRenderer, MockLogger> input(
        1.f / 60.f,
        glm::vec3(1.f, 0.f, 0.f),   // aimDirection (non-degenerate)
        renderer,
        logger,
        glm::vec2(1.f, 0.f),        // moveDirection
        glm::vec3(1.f, 0.f, 0.f));  // moveDirectionWorld

    dAttackAimVisualization::visualize(
        input, fx.state, fx.ic, fx.derived, fx.staticData, vizState);

    // The aim viz never emits a sequence mesh; the assertion that matters is that
    // we reached this line at all (no OOB segfault) with the sentinel active.
    REQUIRE(counts.mesh == 0);
}

#endif // WITH_LOW_LEVEL_TESTS

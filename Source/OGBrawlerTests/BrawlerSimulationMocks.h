#pragma once
// SPDX-License-Identifier: BUSL-1.1

// ============================================================================
// SHARED LLT MOCK ADAPTERS for the brawler sub-simulations.
//
// [movement-sim task 12, 2026-09-06] HOISTED OUT OF `BrawlerProjectileSimulationTest.cpp`,
// and the reason is structural rather than tidiness: task 12's movement cases live in
// their own translation unit, and a mock defined in another `.cpp` cannot be reached from
// one. The Backlog offered "extend the projectile mocks in place" as the alternative; that
// would have grown recording machinery no test could call. One definition, extended once,
// used by both TUs.
//
// SCOPE, STATED SO A LATER READER DOES NOT MISTAKE THIS FOR HALF A SWEEP: the other six
// brawler test files still carry their own local mock pairs (`FMockPhysicsBodyAdapter` in
// `SimulatableBrawlerTest.cpp`, `MockSpatialQueryAdapter` in the four DAttack files, ...).
// Folding those in is a mechanical sweep across six green files with no test to show for
// it; it was deliberately NOT taken here. This header is what the two TUs that share a
// need actually share.
//
// `sweep`, `setBodyTransform`, `setBodyLinearVelocity`, `addBodyAcceleration` and
// `addBodyVelocityChange` are NON-CONST here. That is not an oversight and it costs no
// concept churn: `SpatialQueryAdapter` and `PhysicsBodyAdapter` both bind a NON-CONST
// `T adapter` for exactly these members (they bind a separate `const T cadapter` for
// `getBodyTransform` / `getBodyInertiaTensor` / `captureBodyState`, which therefore stay
// const below). Recording a call is a write, so the write has to be allowed.
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <vector>

#include "OGSimulation/BodyId.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"

namespace brawlerTestMocks
{

// ---------------------------------------------------------------------------
// Mock physics adapter.
//
// Holds a per-body transform / linear velocity (BodyId.value indexes `bodies`, the shape
// the projectile pool tests were written against) AND records every mutating call in
// issue order, which is what a movement case needs: "step 5 wrote the body exactly once
// per tick, with these arguments" is a statement about the CALLS, not about the last
// value left behind.
//
// `capturedState` is the settable `captureBodyState` return. Nothing in the brawler
// sub-simulations calls `captureBodyState` -- the generic
// `SimulationIntegrationExecutor::captureBodyStatesAll` pass does, outside `integrate`.
// A test that wants the captured pose to reach a sub-simulation must therefore stand in
// for that pass itself (assign into `State::bodyState`); setting this field alone proves
// nothing. See `BrawlerMovementSimulationTest.cpp`'s `engineStepAndCapture`.
// ---------------------------------------------------------------------------
struct MockPhysicsAdapter
{
    struct BodyRecord
    {
        glm::mat4 transform{ 1.f };
        glm::vec3 linearVelocity{ 0.f };
    };

    struct TransformCall { BodyId bodyId; glm::mat4 transform{ 1.f }; };
    struct VectorCall    { BodyId bodyId; glm::vec3 value{ 0.f }; };

    std::vector<BodyRecord> bodies;

    // Recorded, in issue order.
    std::vector<TransformCall> setTransformCalls;
    std::vector<VectorCall>    setLinearVelocityCalls;
    std::vector<VectorCall>    addAccelerationCalls;
    std::vector<VectorCall>    addVelocityChangeCalls;

    // The settable `captureBodyState` return -- see the caveat in the block above.
    PhysicsBodyState capturedState{};

    explicit MockPhysicsAdapter(std::size_t bodyCount = 8u)
        : bodies(bodyCount)
    {}

    glm::mat4 getBodyTransform(BodyId id) const { return bodies[id.value].transform; }

    void setBodyTransform(BodyId id, const glm::mat4& t)
    {
        bodies[id.value].transform = t;
        setTransformCalls.push_back(TransformCall{ id, t });
    }

    void setBodyLinearVelocity(BodyId id, const glm::vec3& v)
    {
        bodies[id.value].linearVelocity = v;
        setLinearVelocityCalls.push_back(VectorCall{ id, v });
    }

    // Task 3b force seam. Recorded but unused by the shipped movement model: ruling
    // #14(c) re-places the body from `State` every tick instead of asking the solver to
    // integrate a force. A movement case asserting these vectors stay EMPTY is the
    // cheapest available statement that revision 5's force model did not come back.
    void addBodyAcceleration(BodyId id, const glm::vec3& a)
    {
        addAccelerationCalls.push_back(VectorCall{ id, a });
    }

    void addBodyVelocityChange(BodyId id, const glm::vec3& dv)
    {
        addVelocityChangeCalls.push_back(VectorCall{ id, dv });
    }

    void addBodyTorque(BodyId, const glm::vec3&)          {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    glm::vec3 getBodyInertiaTensor(BodyId) const          { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const       { return capturedState; }

    void clearRecordedCalls()
    {
        setTransformCalls.clear();
        setLinearVelocityCalls.clear();
        addAccelerationCalls.clear();
        addVelocityChangeCalls.clear();
    }
};

static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

// ---------------------------------------------------------------------------
// Mock spatial query adapter.
//
// `overlap` returns a configurable report (the projectile tests' original use).
// `sweep` answers from `scriptedSweeps`, consumed in order by `sweepCursor`, and RECORDS
// every call's `(volumeId, transform, delta)`.
//
// THE LAST SCRIPTED ENTRY IS HELD once the script runs out, so a steady-state case
// scripts ONE hit and ticks as long as it likes; an EMPTY script is a miss on every call.
// Both are deliberate: the alternative (running off the end) turns a script that is one
// tick short into a silent surface-kind flip halfway through a case.
//
// GATE ON THE VOLUME when the subject is driven through the whole `SimulatableBrawler`:
// four sub-simulations reach the query adapter in one `integrate`, and an ungated script
// hands the movement probe's ground to the guard and projectile volumes too. Set
// `filterVolume` + `scriptedVolume`; every other volume then reads as a miss.
// ---------------------------------------------------------------------------
struct MockSpatialQueryAdapter
{
    struct SweepCall
    {
        QueryVolumeId volumeId;
        glm::mat4     transform{ 1.f };
        glm::vec3     delta{ 0.f };
    };

    SpatialQueryReport nextReport{};

    std::vector<SweepHit> scriptedSweeps;
    std::size_t           sweepCursor = 0;

    bool          filterVolume = false;
    QueryVolumeId scriptedVolume{};

    std::vector<SweepCall> sweepCalls;

    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return nextReport; }

    SweepHit sweep(QueryVolumeId volumeId, const glm::mat4& transform, const glm::vec3& delta)
    {
        sweepCalls.push_back(SweepCall{ volumeId, transform, delta });

        if (filterVolume && volumeId != scriptedVolume)
            return SweepHit{};
        if (scriptedSweeps.empty())
            return SweepHit{};

        const std::size_t index = (sweepCursor < scriptedSweeps.size())
            ? sweepCursor
            : scriptedSweeps.size() - 1u;
        ++sweepCursor;
        return scriptedSweeps[index];
    }

    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId)  {}
    void disableShape(ShapeId) {}
};

static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

} // namespace brawlerTestMocks

// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerGroundFrame.h"
#include "OGBrawler/BrawlerInboundHit.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SpatialQueryResult.h"

#include "BrawlerSimulationMocks.h"

#include "glm/geometric.hpp"
#include "glm/trigonometric.hpp"

namespace groundSampleTests
{
namespace movement = brawlerMovementSimulation;
namespace machine  = dAttackMachineSimulation;

constexpr float         kDt = 1.f / 60.f;
constexpr BodyId        kBodyId{ 1u };
constexpr QueryVolumeId kVolumeId{ 91u };

inline glm::vec3 tiltedNormal(float degrees)
{
    const float theta = glm::radians(degrees);
    return glm::vec3(0.f, glm::cos(theta), glm::sin(theta));
}

inline SweepHit floorHit(float fraction, const glm::vec3& normal, const glm::vec3& surfaceNormal)
{
    SweepHit hit{};
    hit.blocked       = true;
    hit.fraction      = fraction;
    hit.normal        = normal;
    hit.impactPoint   = glm::vec3(0.25f, 0.5f, -0.75f);
    hit.surfaceNormal = surfaceNormal;
    return hit;
}

struct Rig
{
    simulatableBrawler::StaticData game;
    movement::StaticData           sd;

    movement::InitialConditions     ic;
    movement::State                 state;
    movement::DerivedState          derived;
    machine::State                  machineState;
    brawlerInboundHit::DerivedState inbound;

    brawlerTestMocks::MockPhysicsAdapter     phys{ 8u };
    brawlerTestMocks::MockSpatialQueryAdapter query;
    movement::RuntimeBindings                bindings;

    Rig() : sd(game.m_movementStaticData)
    {
        sd.drivesBody = false;
        bindings.ownBodyId      = kBodyId;
        bindings.parentBodyId   = kBodyId;
        bindings.queryVolumeIds = { kVolumeId };
        state.bodyState.position = glm::vec3(-0.04f, 5.f, -0.03f);
    }

    float probeLength() const { return sd.rideHeight + sd.snapDistance; }

    void tick(std::uint32_t t)
    {
        SimulationComposite<movement::InitialConditions, movement::State, machine::State>
            composite(ic, state, machineState);
        auto deps = makeDependencies<movement::Dependencies>(composite);
        const movement::PlayerInputView pi{ .flags = 0u, .moveDirectionWorld = glm::vec3(0.f) };
        movement::IntegrationUtils<brawlerTestMocks::MockPhysicsAdapter, brawlerTestMocks::MockSpatialQueryAdapter>
            utils{ kDt, t, phys, query };
        movement::AllInput<brawlerTestMocks::MockPhysicsAdapter, brawlerTestMocks::MockSpatialQueryAdapter>
            allInput{ pi, utils };
        movement::integrate(kDt, allInput, sd, deps, bindings, derived, inbound);
        ic    = composite.get<movement::InitialConditions>();
        state = composite.get<movement::State>();
    }

    template <bool Aligned>
    brawlerGround::GroundSample sample()
    {
        return movement::sampleAttackGround<Aligned>(state, sd, bindings, query);
    }
};

inline bool sameCall(const brawlerTestMocks::MockSpatialQueryAdapter::SweepCall& a,
                     const brawlerTestMocks::MockSpatialQueryAdapter::SweepCall& b)
{
    return a.volumeId == b.volumeId && a.transform == b.transform && a.delta == b.delta;
}

} // namespace groundSampleTests

TEST_CASE("BrawlerGroundSample.ProbeGroundIssuesTheMovementSweep", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    Rig rig;
    const SweepHit scripted = floorHit(0.5f, tiltedNormal(10.f), glm::vec3(0.f));
    rig.query.scriptedSweeps = { scripted };

    const movement::GroundProbe probe = movement::probeGround(rig.state, rig.sd, rig.bindings, rig.query);

    REQUIRE(rig.query.sweepCalls.size() == 1u);
    const auto& call = rig.query.sweepCalls[0];
    REQUIRE(call.volumeId == kVolumeId);
    glm::mat4 expectedPose(1.f);
    expectedPose[3] = glm::vec4(glm::vec3(-0.04f, 5.f, -0.03f), 1.f);
    REQUIRE(sameCall(call, { kVolumeId, expectedPose, -movement::kWorldUp * rig.probeLength() }));
    REQUIRE(call.delta == simFrame::withHeight(glm::vec3(0.f), -rig.probeLength()));

    REQUIRE(probe.hit.blocked);
    REQUIRE(simFrameDetail::sameBits(probe.hit.normal, scripted.normal));
    REQUIRE(probe.clearance == scripted.fraction * rig.probeLength());
    REQUIRE(probe.walkable);

    Rig moved;
    moved.query.scriptedSweeps = { scripted };
    moved.tick(1u);
    REQUIRE(moved.query.sweepCalls.size() == 1u);
    REQUIRE(sameCall(moved.query.sweepCalls[0], call));
}

TEST_CASE("BrawlerGroundSample.ProbeGroundMissAndSteepFace", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    Rig miss;
    const movement::GroundProbe missed = movement::probeGround(miss.state, miss.sd, miss.bindings, miss.query);
    REQUIRE(miss.query.sweepCalls.size() == 1u);
    REQUIRE_FALSE(missed.hit.blocked);
    REQUIRE(missed.clearance == miss.probeLength());
    REQUIRE_FALSE(missed.walkable);

    Rig steep;
    steep.query.scriptedSweeps = { floorHit(0.5f, tiltedNormal(steep.sd.maxSlopeAngleDeg + 5.f), glm::vec3(0.f)) };
    const movement::GroundProbe tooSteep = movement::probeGround(steep.state, steep.sd, steep.bindings, steep.query);
    REQUIRE(tooSteep.hit.blocked);
    REQUIRE_FALSE(tooSteep.walkable);

    Rig unbound;
    unbound.bindings.queryVolumeIds.clear();
    const movement::GroundProbe none = movement::probeGround(unbound.state, unbound.sd, unbound.bindings, unbound.query);
    REQUIRE(unbound.query.sweepCalls.empty());
    REQUIRE_FALSE(none.hit.blocked);
    REQUIRE(none.clearance == unbound.probeLength());
}

TEST_CASE("BrawlerGroundSample.MovementIntegrateSweepsOncePerTick", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    Rig rig;
    rig.query.scriptedSweeps = { floorHit(rig.sd.rideHeight / rig.probeLength(), glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f)) };

    constexpr std::uint32_t kTicks = 12u;
    for (std::uint32_t t = 1u; t <= kTicks; ++t)
    {
        const glm::vec3 positionBefore = rig.state.bodyState.position;
        rig.tick(t);
        REQUIRE(rig.query.sweepCalls.size() == t);
        REQUIRE(rig.query.sweepCalls.back().volumeId == kVolumeId);
        REQUIRE(glm::vec3(rig.query.sweepCalls.back().transform[3]) == positionBefore);
        REQUIRE(rig.query.sweepCalls.back().delta == simFrame::withHeight(glm::vec3(0.f), -rig.probeLength()));
    }
}

TEST_CASE("BrawlerGroundSample.KnobOffMakesNoQueryAndReturnsUp", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    Rig rig;
    rig.query.scriptedSweeps = { floorHit(0.5f, tiltedNormal(20.f), tiltedNormal(30.f)) };

    const brawlerGround::GroundSample sample = rig.sample<false>();

    REQUIRE(rig.query.sweepCalls.empty());
    REQUIRE(simFrameDetail::sameBits(sample.normal, simFrame::kUp));

    Rig shipped;
    shipped.query.scriptedSweeps = rig.query.scriptedSweeps;
    const brawlerGround::GroundSample shippedSample =
        movement::sampleAttackGround(shipped.state, shipped.sd, shipped.bindings, shipped.query);
    REQUIRE(shipped.query.sweepCalls.size() == (brawlerGround::kGroundAlignedAttacks ? 1u : 0u));
    if constexpr (!brawlerGround::kGroundAlignedAttacks)
    {
        REQUIRE(simFrameDetail::sameBits(shippedSample.normal, simFrame::kUp));
    }
}

TEST_CASE("BrawlerGroundSample.AlignedPrefersTheExactFaceNormal", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    Rig rig;
    const glm::vec3 face = tiltedNormal(30.f);
    const glm::vec3 gjk  = tiltedNormal(29.f);
    rig.query.scriptedSweeps = { floorHit(0.5f, gjk, face) };

    const brawlerGround::GroundSample sample = rig.sample<true>();

    INFO("sample=(" << sample.normal.x << ", " << sample.normal.y << ", " << sample.normal.z << ")");
    REQUIRE(rig.query.sweepCalls.size() == 1u);
    REQUIRE(simFrameDetail::sameBits(sample.normal, face));

    Rig probed;
    probed.query.scriptedSweeps = rig.query.scriptedSweeps;
    (void)movement::probeGround(probed.state, probed.sd, probed.bindings, probed.query);
    REQUIRE(sameCall(rig.query.sweepCalls[0], probed.query.sweepCalls[0]));
}

TEST_CASE("BrawlerGroundSample.AlignedFallsBackToTheSweepNormal", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    Rig rig;
    const glm::vec3 gjk = tiltedNormal(20.f);
    rig.query.scriptedSweeps = { floorHit(0.5f, gjk, glm::vec3(0.f)) };

    const brawlerGround::GroundSample sample = rig.sample<true>();

    REQUIRE(rig.query.sweepCalls.size() == 1u);
    REQUIRE(simFrameDetail::sameBits(sample.normal, gjk));
}

TEST_CASE("BrawlerGroundSample.AlignedReturnsUpOffWalkableGround", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    SECTION("a face steeper than the walkable cap")
    {
        Rig rig;
        const glm::vec3 steep = tiltedNormal(50.f);
        REQUIRE(glm::dot(steep, movement::kWorldUp) < rig.sd.cosMaxSlope);
        rig.query.scriptedSweeps = { floorHit(0.5f, steep, steep) };
        REQUIRE(simFrameDetail::sameBits(rig.sample<true>().normal, simFrame::kUp));
        REQUIRE(rig.query.sweepCalls.size() == 1u);
    }
    SECTION("a steep exact face under a walkable sweep normal")
    {
        Rig rig;
        rig.query.scriptedSweeps = { floorHit(0.5f, tiltedNormal(20.f), tiltedNormal(50.f)) };
        REQUIRE(simFrameDetail::sameBits(rig.sample<true>().normal, simFrame::kUp));
    }
    SECTION("a miss")
    {
        Rig rig;
        REQUIRE(simFrameDetail::sameBits(rig.sample<true>().normal, simFrame::kUp));
        REQUIRE(rig.query.sweepCalls.size() == 1u);
    }
    SECTION("an unblocked hit that carries normals")
    {
        Rig rig;
        SweepHit unblocked = floorHit(0.5f, tiltedNormal(20.f), tiltedNormal(30.f));
        unblocked.blocked = false;
        rig.query.scriptedSweeps = { unblocked };
        REQUIRE(simFrameDetail::sameBits(rig.sample<true>().normal, simFrame::kUp));
    }
    SECTION("a vertical exact face under a walkable sweep normal")
    {
        Rig rig;
        rig.query.scriptedSweeps = { floorHit(0.5f, tiltedNormal(20.f), simFrame::kForward) };
        REQUIRE(simFrameDetail::sameBits(rig.sample<true>().normal, simFrame::kUp));
    }
}

#endif // WITH_LOW_LEVEL_TESTS

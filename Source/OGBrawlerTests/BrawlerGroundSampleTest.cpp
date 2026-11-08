// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <cstdio>
#include <variant>
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

    brawlerGround::GroundSample sample()
    {
        return movement::sampleAttackGround(state, sd, bindings, query);
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

TEST_CASE("BrawlerGroundSample.AlignedPrefersTheExactFaceNormal", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    Rig rig;
    const glm::vec3 face = tiltedNormal(30.f);
    const glm::vec3 gjk  = tiltedNormal(29.f);
    rig.query.scriptedSweeps = { floorHit(0.5f, gjk, face) };

    const brawlerGround::GroundSample sample = rig.sample();

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

    const brawlerGround::GroundSample sample = rig.sample();

    REQUIRE(rig.query.sweepCalls.size() == 1u);
    REQUIRE(simFrameDetail::sameBits(sample.normal, gjk));
}

TEST_CASE("BrawlerGroundSample.DeclaresTheGroundProbeAndTheCentreProbe", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    const simulatableBrawler::StaticData shipped;
    const movement::StaticData& sd = shipped.m_movementStaticData;

    const std::vector<QueryVolumeDescriptor> two = movement::PhysicsDeclaration::queryVolumes(sd);
    REQUIRE(two.size() == 2u);
    REQUIRE(movement::PhysicsSetup::queryVolumes(sd).size() == 2u);

    const CapsuleGeometry* probe = std::get_if<CapsuleGeometry>(&two[0].geometry);
    REQUIRE(probe != nullptr);
    REQUIRE(probe->radius == sd.capsuleRadius - movement::kProbeShrink);
    REQUIRE(probe->halfHeight == sd.capsuleHalfHeight - movement::kProbeShrink);
    const glm::vec3 probeOffset(two[0].offsetTransform[3]);
    REQUIRE(simFrameDetail::sameBits(probeOffset, simFrame::withHeight(glm::vec3(0.f), -movement::kProbeShrink)));

    const SphereGeometry* centre = std::get_if<SphereGeometry>(&two[1].geometry);
    REQUIRE(centre != nullptr);
    REQUIRE(centre->radius == movement::kCenterProbeRadius);
    const glm::vec3 centreOffset(two[1].offsetTransform[3]);
    REQUIRE(simFrameDetail::sameBits(centreOffset,
        simFrame::withHeight(glm::vec3(0.f), -(sd.capsuleHalfHeight - movement::kCenterProbeRadius))));
    REQUIRE(simFrame::height(centreOffset) - centre->radius == -sd.capsuleHalfHeight);
    REQUIRE(glm::mat3(two[1].offsetTransform) == glm::mat3(1.f));
    REQUIRE(two[1].searchCategories.bits == two[0].searchCategories.bits);
    REQUIRE(two[1].traceCategory == two[0].traceCategory);
}

TEST_CASE("BrawlerGroundSample.V2SampleIsTheEstimatesFaceNormal", "[BrawlerMovement]")
{
    using namespace groundSampleTests;

    constexpr QueryVolumeId kCentreId{ 94u };
    const Rig base;
    const movement::StaticData& sd = base.sd;
    std::vector<SweepHit> hits = {
        floorHit(0.5f, tiltedNormal(10.f), glm::vec3(0.f)),
        floorHit(0.5f, tiltedNormal(20.f), tiltedNormal(30.f)),
        floorHit(0.5f, tiltedNormal(29.f), tiltedNormal(30.f)),
        floorHit(0.5f, tiltedNormal(20.f), tiltedNormal(50.f)),
        floorHit(0.5f, tiltedNormal(20.f), simFrame::kForward),
        floorHit(0.5f, tiltedNormal(sd.maxSlopeAngleDeg + 5.f), glm::vec3(0.f)),
        floorHit(0.5f, tiltedNormal(sd.maxSlopeAngleDeg + 5.f), tiltedNormal(30.f)),
        SweepHit{},
    };
    for (int gjk = -90; gjk <= 90; gjk += 5)
        for (int face = -105; face <= 90; face += 15)
            for (const bool blocked : { true, false })
            {
                SweepHit hit = floorHit(0.5f, tiltedNormal(static_cast<float>(gjk)),
                    face < -90 ? glm::vec3(0.f) : tiltedNormal(static_cast<float>(face)));
                hit.blocked = blocked;
                hits.push_back(hit);
            }
    const SweepHit centres[] = {
        SweepHit{},
        floorHit(0.4f, tiltedNormal(30.f), glm::vec3(0.f)),
        floorHit(0.4f, tiltedNormal(30.f), tiltedNormal(30.f)),
        floorHit(0.4f, tiltedNormal(20.f), simFrame::kForward),
        floorHit(0.4f, tiltedNormal(60.f), tiltedNormal(60.f)),
    };

    int estimates = 0;
    int centreSweeps = 0;
    int kinds[3] = { 0, 0, 0 };
    for (const SweepHit& hit : hits)
        for (const SweepHit& centre : centres)
            {
                Rig sampled;
                Rig estimated;
                for (Rig* rig : { &sampled, &estimated })
                {
                    rig->bindings.queryVolumeIds = { kVolumeId, kCentreId };
                    rig->query.scriptedSweeps = { hit, centre };
                }
                const brawlerGround::GroundSample sample = sampled.sample();
                const movement::GroundSupport g = movement::estimateSupport(estimated.state, estimated.sd, estimated.bindings, estimated.query);
                INFO("hit blocked " << hit.blocked << " normal=(" << hit.normal.x << ", " << hit.normal.y << ", "
                     << hit.normal.z << ") surfaceNormal=(" << hit.surfaceNormal.x << ", " << hit.surfaceNormal.y << ", "
                     << hit.surfaceNormal.z << ") centre blocked " << centre.blocked);
                REQUIRE(simFrameDetail::sameBits(sample.normal, g.faceNormal));
                REQUIRE(sampled.query.sweepCalls.size() == estimated.query.sweepCalls.size());
                for (std::size_t i = 0u; i < sampled.query.sweepCalls.size(); ++i)
                    REQUIRE(sameCall(sampled.query.sweepCalls[i], estimated.query.sweepCalls[i]));
                REQUIRE(sampled.query.sweepCalls[0].volumeId == kVolumeId);
                const bool provided = glm::dot(hit.surfaceNormal, hit.surfaceNormal) > 0.f;
                const bool edge = hit.blocked && provided && glm::dot(hit.surfaceNormal, hit.normal) < movement::kEdgeContactCos;
                REQUIRE(sampled.query.sweepCalls.size() == (edge ? 2u : 1u));
                if (edge)
                {
                    const auto& second = sampled.query.sweepCalls[1];
                    REQUIRE(second.volumeId == kCentreId);
                    REQUIRE(second.transform == sampled.query.sweepCalls[0].transform);
                    REQUIRE(second.delta == sampled.query.sweepCalls[0].delta);
                    ++centreSweeps;
                }
                REQUIRE(simFrameDetail::sameBits(g.frameNormal, g.faceNormal));
                REQUIRE(g.eligible == (g.kind != movement::SupportKind::None));
                if (g.kind != movement::SupportKind::Face)
                    REQUIRE(simFrameDetail::sameBits(g.faceNormal, simFrame::kUp));
                ++kinds[static_cast<int>(g.kind)];
                ++estimates;
            }

    std::printf("[BrawlerGroundSample.v2] estimates=%d centreSweeps=%d none=%d face=%d lip=%d\n", estimates, centreSweeps,
        kinds[0], kinds[1], kinds[2]);
    REQUIRE(estimates == int(hits.size() * 5u));
    REQUIRE(centreSweeps > 0);
    REQUIRE(kinds[0] > 0);
    REQUIRE(kinds[1] > 0);
    REQUIRE(kinds[2] > 0);
}

#endif // WITH_LOW_LEVEL_TESTS

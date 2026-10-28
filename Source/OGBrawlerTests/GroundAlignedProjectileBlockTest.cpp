// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "OGBrawler/BrawlerProjectileHitDetection.h"
#include "OGBrawler/BrawlerProjectileSimulation.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <tuple>

#include "catch_amalgamated.hpp"

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "BrawlerSimulationMocks.h"
#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SimulationSerialization.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SurfaceFrame.h"

static_assert(!Serializable<brawlerProjectileSimulation::SlotDetection>);
static_assert(!Serializable<brawlerProjectileSimulation::DerivedState>);
static_assert(std::tuple_size_v<decltype(SerializableFields<brawlerProjectileSimulation::ProjectileSlot>::get())> == 5);
static_assert(std::tuple_size_v<decltype(SerializableFields<brawlerProjectileSimulation::State>::get())> == 1);
static_assert(std::tuple_size_v<decltype(SerializableFields<brawlerProjectileSimulation::InitialConditions>::get())> == 3);

namespace groundAlignedProjectileBlockTests
{
    using brawlerTestMocks::MockPhysicsAdapter;
    using brawlerTestMocks::MockSpatialQueryAdapter;

    constexpr float kDt = 1.f / 60.f;
    constexpr std::uint32_t kParentBody = brawlerProjectileSimulation::kMaxProjectilePoolSize;
    constexpr std::uint32_t kGuardBody = brawlerProjectileSimulation::kMaxProjectilePoolSize + 1;
    constexpr std::uint32_t kTargetRoot = 77u;
    constexpr std::uint32_t kSpawnTick = 100u;

    const glm::vec3 kThirtyDegreeNormal(0.f, std::cos(glm::radians(30.f)), std::sin(glm::radians(30.f)));

    glm::mat4 slopeTilt()
    {
        return glm::mat4(surfaceFrame::tiltMatrix(kThirtyDegreeNormal));
    }

    glm::vec3 rotatePoint(const glm::mat4& m, const glm::vec3& p)
    {
        return glm::vec3(m * glm::vec4(p, 1.f));
    }

    glm::vec3 rotateDirection(const glm::mat4& m, const glm::vec3& v)
    {
        return glm::vec3(m * glm::vec4(v, 0.f));
    }

    glm::vec3 headingDirection(float heading)
    {
        return rotateDirection(glm::rotate(glm::mat4(1.f), heading, simFrame::kUp), simFrame::kForward);
    }

    glm::mat4 guardPose(float heading, bool slope, const glm::vec3& root)
    {
        glm::mat4 guard = glm::rotate(glm::mat4(1.f), heading, simFrame::kUp);
        if (slope)
            guard = slopeTilt() * guard;
        guard[3] = glm::vec4(root, 1.f);
        return guard;
    }

    const brawlerProjectileSimulation::StaticData& projectileStaticData()
    {
        static const simulatableBrawler::StaticData staticData;
        return staticData.m_projectileStaticData;
    }

    struct Todays
    {
        bool blocks = false;
        float angle = 0.f;
    };

    Todays todaysProjectileGuardBlocks(const glm::mat4& guardTransform, const glm::vec3& spawnDir,
                                       float guardMiddleSectionHalfAngle)
    {
        glm::vec3 guardForward = simFrame::forwardColumn(guardTransform);
        const float fwdLen = glm::length(guardForward);
        if (fwdLen > 0.0001f)
            guardForward /= fwdLen;

        const glm::vec3 incoming = -spawnDir;
        const float d     = glm::clamp(glm::dot(guardForward, incoming), -1.f, 1.f);
        const float angle = glm::acos(d);
        return Todays{ angle <= guardMiddleSectionHalfAngle, angle };
    }

    glm::vec3 todaysBlockMarker(const glm::vec3& spawnPos, const glm::vec3& spawnDir, const glm::vec3& charPos,
                                const glm::vec3& objectPosition, float innerCircleRadius, bool& foundIntersection)
    {
        glm::vec3 blockPos = objectPosition;
        foundIntersection = false;
        const glm::vec2 rayO = simFrame::planar(spawnPos);
        const glm::vec2 rayD = simFrame::planar(spawnDir);
        const glm::vec2 circC = simFrame::planar(charPos);
        const glm::vec2 fromC = rayO - circC;
        const float aQ = glm::dot(rayD, rayD);
        const float bQ = 2.f * glm::dot(fromC, rayD);
        const float cQ = glm::dot(fromC, fromC) - innerCircleRadius * innerCircleRadius;
        const float disc = bQ * bQ - 4.f * aQ * cQ;
        if (aQ > 1e-8f && disc >= 0.f)
        {
            const float sq = std::sqrt(disc);
            float t = (-bQ - sq) / (2.f * aQ);
            if (t < 0.f)
                t = (-bQ + sq) / (2.f * aQ);
            if (t >= 0.f)
            {
                blockPos = spawnPos + t * spawnDir;
                foundIntersection = true;
            }
        }
        return blockPos;
    }

    bool sameBits(const glm::vec3& a, const glm::vec3& b)
    {
        return std::bit_cast<std::uint32_t>(a.x) == std::bit_cast<std::uint32_t>(b.x)
            && std::bit_cast<std::uint32_t>(a.y) == std::bit_cast<std::uint32_t>(b.y)
            && std::bit_cast<std::uint32_t>(a.z) == std::bit_cast<std::uint32_t>(b.z);
    }

    std::array<brawlerProjectileSimulation::RuntimeBindings, brawlerProjectileSimulation::kMaxProjectilePoolSize>
    makeBindings()
    {
        std::array<brawlerProjectileSimulation::RuntimeBindings, brawlerProjectileSimulation::kMaxProjectilePoolSize> b{};
        for (std::uint32_t i = 0; i < brawlerProjectileSimulation::kMaxProjectilePoolSize; ++i)
        {
            b[i].ownBodyId = BodyId{ i };
            b[i].parentBodyId = BodyId{ kParentBody };
            b[i].queryVolumeIds = { QueryVolumeId{ i } };
        }
        return b;
    }

    struct Shot
    {
        glm::vec3 spawnPos{ 0.f };
        glm::vec3 spawnDir{ 0.f };
        glm::mat4 guard{ 1.f };
        glm::vec3 objectPosition{ 0.f };
        bool guardHit = true;
    };

    struct Outcome
    {
        brawlerProjectileSimulation::SlotOutcome outcome = brawlerProjectileSimulation::SlotOutcome::None;
        glm::vec3 guardUp{ 0.f };
        bool hasBlock = false;
        glm::vec3 marker{ 0.f };
    };

    Outcome run(const Shot& shot)
    {
        using namespace brawlerProjectileSimulation;
        const StaticData& sd = projectileStaticData();

        State state;
        state.slots[0].spawnTick = kSpawnTick;
        state.slots[0].spawnPos = shot.spawnPos;
        state.slots[0].spawnDir = shot.spawnDir;
        state.slots[0].endTick = 0u;

        MockPhysicsAdapter physics{ kMaxProjectilePoolSize + 2 };
        physics.bodies[kGuardBody].transform = shot.guard;

        MockSpatialQueryAdapter query;
        SpatialQueryHit hit{};
        hit.rootBodyId = BodyId{ kTargetRoot };
        hit.objectPosition = shot.objectPosition;
        hit.bodyId = shot.guardHit ? BodyId{ kGuardBody } : BodyId{ kTargetRoot };
        hit.objectCategories = CollisionCategories::single(shot.guardHit ? collisionCategory::guard : collisionCategory::body);
        query.nextReport.hits.push_back(hit);

        auto bindings = makeBindings();
        DerivedState derived;
        brawlerHitDetection::detectProjectileHits(kDt, kSpawnTick, sd, state, bindings, derived, physics, query);

        Outcome out;
        out.outcome = derived.detectedThisTick[0].outcome;
        out.guardUp = derived.detectedThisTick[0].guardUp;

        InitialConditions ic;
        SimulationComposite<InitialConditions, State> composite(ic, state);
        auto deps = makeDependencies<Dependencies>(composite);
        PlayerInputView pi{};
        IntegrationUtils<MockPhysicsAdapter> utils{ kDt, kSpawnTick, physics };
        AllInput<MockPhysicsAdapter> allInput{ pi, utils };
        integrate(kDt, allInput, sd, deps, bindings, derived);

        out.hasBlock = derived.blocks.size() == 1u;
        if (out.hasBlock)
            out.marker = derived.blocks.front().position;
        return out;
    }

    float inPlaneDistance(const glm::vec3& p, const glm::vec3& centre, const glm::vec3& up)
    {
        glm::vec3 d = p - centre;
        d -= up * glm::dot(d, up);
        return glm::length(d);
    }
}

using namespace groundAlignedProjectileBlockTests;

TEST_CASE("GroundAlignedProjectileBlock.ConjugationIdentityOnTheThirtyDegreePlane", "[BrawlerProjectile]")
{
    using brawlerProjectileSimulation::SlotOutcome;
    std::mt19937 rng(0x6a8u);
    std::uniform_real_distribution<float> unit(-1.f, 1.f);
    std::uniform_real_distribution<float> heading(-glm::pi<float>(), glm::pi<float>());
    std::bernoulli_distribution nearFacing(0.5);
    const float halfAngle = projectileStaticData().guardMiddleSectionHalfAngle;
    const glm::mat4 tilt = slopeTilt();
    const float margin = 1.e-4f;

    int accepted = 0;
    int generated = 0;
    int mismatches = 0;
    int flatVersusToday = 0;
    int slopeBranchMissed = 0;
    int guardUpNotRecorded = 0;
    int blocked = 0;
    int notBlocked = 0;
    int markerMissing = 0;
    float worstMarkerConjugation = 0.f;
    while (accepted < 2000 && generated < 200000)
    {
        ++generated;
        const float guardHeading = heading(rng);
        const glm::vec3 guardRoot(4.f * unit(rng), 1.5f * unit(rng), 4.f * unit(rng));
        const glm::mat4 guard = guardPose(guardHeading, false, guardRoot);
        const float shotHeading = nearFacing(rng) ? guardHeading + glm::pi<float>() + 0.6f * unit(rng) : heading(rng);
        const glm::vec3 spawnDir = headingDirection(shotHeading);
        const glm::vec3 across = glm::cross(simFrame::kUp, spawnDir);
        const glm::vec3 spawnPos = guardRoot - spawnDir * (2.75f + 1.25f * unit(rng)) + across * (0.5f * unit(rng))
            + simFrame::withHeight(glm::vec3(0.f), 0.5f + 0.3f * unit(rng));
        const glm::vec3 objectPosition = guardRoot + glm::vec3(0.3f * unit(rng), 0.3f * unit(rng), 0.3f * unit(rng));

        const Todays today = todaysProjectileGuardBlocks(guard, spawnDir, halfAngle);
        if (std::abs(today.angle - halfAngle) <= margin)
            continue;
        ++accepted;

        Shot flatShot{ spawnPos, spawnDir, guard, objectPosition, true };
        const Outcome flat = run(flatShot);
        const bool flatBlocks = flat.outcome == SlotOutcome::BlockedByGuard;
        if (flatBlocks != today.blocks)
            ++flatVersusToday;

        Shot slopeShot{ rotatePoint(tilt, spawnPos), rotateDirection(tilt, spawnDir), tilt * guard,
                        rotatePoint(tilt, objectPosition), true };
        const glm::vec3 slopeUp = surfaceFrame::surfaceUpOf(slopeShot.guard);
        if (surfaceFrame::isFlat(slopeUp))
            ++slopeBranchMissed;
        const Outcome onSlope = run(slopeShot);
        if (!sameBits(onSlope.guardUp, slopeUp))
            ++guardUpNotRecorded;
        if (onSlope.outcome != flat.outcome)
            ++mismatches;
        (flatBlocks ? blocked : notBlocked) += 1;
        if (flatBlocks)
        {
            if (!flat.hasBlock || !onSlope.hasBlock)
                ++markerMissing;
            else
                worstMarkerConjugation = std::max(worstMarkerConjugation,
                    glm::length(onSlope.marker - rotatePoint(tilt, flat.marker)));
        }
    }

    std::printf("[GroundAlignedProjectileBlock.identity] accepted=%d generated=%d mismatches=%d flatVersusToday=%d "
                "slopeBranchMissed=%d guardUpNotRecorded=%d blocked=%d notBlocked=%d markerMissing=%d "
                "worstMarkerConjugation=%.3g m\n",
                accepted, generated, mismatches, flatVersusToday, slopeBranchMissed, guardUpNotRecorded,
                blocked, notBlocked, markerMissing, worstMarkerConjugation);
    REQUIRE(accepted == 2000);
    CHECK(mismatches == 0);
    CHECK(flatVersusToday == 0);
    CHECK(slopeBranchMissed == 0);
    CHECK(guardUpNotRecorded == 0);
    CHECK(blocked > 0);
    CHECK(notBlocked > 0);
    CHECK(markerMissing == 0);
    CHECK(worstMarkerConjugation <= 1.e-4f);
}

TEST_CASE("GroundAlignedProjectileBlock.AFlatGroundShotIntoAGuardOnTheSlopeHeadOnIsBlocked", "[BrawlerProjectile]")
{
    using brawlerProjectileSimulation::SlotOutcome;
    const float halfAngle = projectileStaticData().guardMiddleSectionHalfAngle;
    const float r = projectileStaticData().innerCircleRadius;
    const glm::vec3 guardRoot(0.f);
    const glm::mat4 guard = guardPose(glm::pi<float>(), true, guardRoot);
    const glm::vec3 guardUp = surfaceFrame::surfaceUpOf(guard);
    const glm::vec3 downSlope = surfaceFrame::tiltToSurface(kThirtyDegreeNormal, -simFrame::kForward);
    const glm::vec3 spawnDir = simFrame::kForward;
    const glm::vec3 spawnPos = simFrame::withHeight(glm::vec3(0.f), 0.5f) - spawnDir * 3.f;

    const Todays today = todaysProjectileGuardBlocks(guard, spawnDir, halfAngle);
    const Outcome slope = run(Shot{ spawnPos, spawnDir, guard, guardRoot, true });
    const float markerInPlane = inPlaneDistance(slope.marker, guardRoot, guardUp);
    const float markerOffRay = glm::length(glm::cross(slope.marker - spawnPos, spawnDir));
    const float markerTowardsShooter = glm::dot(slope.marker - guardRoot, -spawnDir);

    const Outcome alongNormal = run(Shot{ guardRoot + guardUp * 2.f, -guardUp, guard, guardRoot, true });
    glm::vec3 residual = guardUp;
    residual -= guardUp * glm::dot(residual, guardUp);

    std::printf("[GroundAlignedProjectileBlock.crossPlane] todaysAngle=%.7f rad (%.4f deg) todaysBlocks=%d "
                "slopeOutcome=%d forwardDotDownSlope=%.7f markerInPlane=%.7f m (r=%.3f) markerOffRay=%.3g m "
                "markerTowardsShooter=%.4f m alongNormalOutcome=%d alongNormalResidual=%.3g\n",
                today.angle, glm::degrees(today.angle), today.blocks ? 1 : 0, static_cast<int>(slope.outcome),
                glm::dot(simFrame::forwardColumn(guard), downSlope), markerInPlane, r, markerOffRay,
                markerTowardsShooter, static_cast<int>(alongNormal.outcome), glm::length(residual));
    CHECK(std::abs(today.angle - glm::radians(30.f)) <= 1.e-5f);
    CHECK_FALSE(today.blocks);
    CHECK(slope.outcome == SlotOutcome::BlockedByGuard);
    CHECK(sameBits(slope.guardUp, guardUp));
    REQUIRE(slope.hasBlock);
    CHECK(std::abs(markerInPlane - r) <= 1.e-4f);
    CHECK(markerOffRay <= 1.e-4f);
    CHECK(markerTowardsShooter > 0.f);
    CHECK(alongNormal.outcome == SlotOutcome::Hit);
}

TEST_CASE("GroundAlignedProjectileBlock.SlopeMarkerLiesOnTheInnerCircleInTheGuardPlane", "[BrawlerProjectile]")
{
    using brawlerProjectileSimulation::SlotOutcome;
    const float r = projectileStaticData().innerCircleRadius;
    const float heightOffset = 0.5f;
    const glm::mat4 tilt = slopeTilt();
    const glm::vec3 n = kThirtyDegreeNormal;

    int checked = 0;
    int notBlocked = 0;
    float worstOffCircle = 0.f;
    float worstOffLaunchPlane = 0.f;
    float worstOffRay = 0.f;
    float leastTowardsShooter = 1.e9f;
    const glm::vec3 roots[2] = { glm::vec3(0.f), rotatePoint(tilt, glm::vec3(1.3f, 0.f, -2.1f)) };
    for (const glm::vec3& guardRoot : roots)
    {
        for (int h = 0; h < 8; ++h)
        {
            const float localHeading = glm::two_pi<float>() * static_cast<float>(h) / 8.f + 0.1f;
            const glm::vec3 localDir = headingDirection(localHeading);
            const glm::vec3 spawnDir = rotateDirection(tilt, localDir);
            const glm::vec3 across = rotateDirection(tilt, glm::cross(simFrame::kUp, localDir));
            const glm::mat4 guard = guardPose(localHeading + glm::pi<float>(), true, guardRoot);
            for (int lateral = -2; lateral <= 2; ++lateral)
            {
                for (int distance = 0; distance < 3; ++distance)
                {
                    const glm::vec3 spawnPos = guardRoot - spawnDir * (1.5f + static_cast<float>(distance))
                        + across * (0.3f * r * static_cast<float>(lateral)) + n * heightOffset;
                    const Outcome slope = run(Shot{ spawnPos, spawnDir, guard, guardRoot, true });
                    if (slope.outcome != SlotOutcome::BlockedByGuard || !slope.hasBlock)
                    {
                        ++notBlocked;
                        continue;
                    }
                    ++checked;
                    const glm::vec3 guardUp = surfaceFrame::surfaceUpOf(guard);
                    worstOffCircle = std::max(worstOffCircle, std::abs(inPlaneDistance(slope.marker, guardRoot, guardUp) - r));
                    worstOffLaunchPlane = std::max(worstOffLaunchPlane,
                        std::abs(glm::dot(slope.marker - guardRoot, guardUp) - heightOffset));
                    worstOffRay = std::max(worstOffRay, glm::length(glm::cross(slope.marker - spawnPos, spawnDir)));
                    leastTowardsShooter = std::min(leastTowardsShooter, glm::dot(slope.marker - guardRoot, -spawnDir));
                }
            }
        }
    }

    std::printf("[GroundAlignedProjectileBlock.marker] checked=%d notBlocked=%d worstOffInnerCircle=%.3g m "
                "worstOffLaunchPlane=%.3g m worstOffRay=%.3g m leastTowardsShooter=%.4f m\n",
                checked, notBlocked, worstOffCircle, worstOffLaunchPlane, worstOffRay, leastTowardsShooter);
    CHECK(checked == 2 * 8 * 5 * 3);
    CHECK(notBlocked == 0);
    CHECK(worstOffCircle <= 1.e-4f);
    CHECK(worstOffLaunchPlane <= 1.e-4f);
    CHECK(worstOffRay <= 1.e-4f);
    CHECK(leastTowardsShooter > 0.f);
}

TEST_CASE("GroundAlignedProjectileBlock.FlatGuardReproducesTodaysOutcomeAndMarkerBits", "[BrawlerProjectile]")
{
    using brawlerProjectileSimulation::SlotOutcome;
    const float r = projectileStaticData().innerCircleRadius;
    const float halfAngle = projectileStaticData().guardMiddleSectionHalfAngle;
    const glm::vec3 roots[2] = { glm::vec3(0.f), glm::vec3(2.7f, 0.31f, -1.9f) };
    const float pitches[5] = { 0.f, 0.2f, -0.15f, 0.3f, -0.3f };

    int compared = 0;
    int missedTheCircle = 0;
    int mismatches = 0;
    int outcomeMismatches = 0;
    int outOfTheCone = 0;
    for (const glm::vec3& guardRoot : roots)
    {
        for (int h = 0; h < 6; ++h)
        {
            const float shotHeading = glm::two_pi<float>() * static_cast<float>(h) / 6.f + 0.37f;
            const glm::mat4 guard = guardPose(shotHeading + glm::pi<float>(), false, guardRoot);
            for (const float pitch : pitches)
            {
                const glm::vec3 flatDir = headingDirection(shotHeading);
                const glm::vec3 spawnDir = glm::normalize(flatDir + simFrame::withHeight(glm::vec3(0.f), pitch));
                const glm::vec3 across = glm::cross(simFrame::kUp, flatDir);
                for (int lateral = -3; lateral <= 3; ++lateral)
                {
                    for (int distance = 0; distance < 3; ++distance)
                    {
                        const glm::vec3 spawnPos = guardRoot - flatDir * (1.2f + 1.1f * static_cast<float>(distance))
                            + across * (0.4f * r * static_cast<float>(lateral)) + simFrame::withHeight(glm::vec3(0.f), 0.5f);
                        const glm::vec3 objectPosition = guardRoot + glm::vec3(0.05f, 0.4f, -0.03f);
                        const Todays today = todaysProjectileGuardBlocks(guard, spawnDir, halfAngle);
                        const Outcome flat = run(Shot{ spawnPos, spawnDir, guard, objectPosition, true });
                        if ((flat.outcome == SlotOutcome::BlockedByGuard) != today.blocks)
                            ++outcomeMismatches;
                        if (!today.blocks)
                            ++outOfTheCone;
                        if (!flat.hasBlock)
                            continue;
                        bool found = false;
                        const glm::vec3 expected = todaysBlockMarker(spawnPos, spawnDir, guardRoot, objectPosition, r, found);
                        ++compared;
                        if (!found)
                            ++missedTheCircle;
                        if (!sameBits(flat.marker, expected))
                            ++mismatches;
                    }
                }
            }
        }
    }

    std::printf("[GroundAlignedProjectileBlock.flatMarker] compared=%d missedTheCircle=%d mismatches=%d "
                "outcomeMismatches=%d outOfTheCone=%d\n", compared, missedTheCircle, mismatches,
                outcomeMismatches, outOfTheCone);
    CHECK(compared > 0);
    CHECK(missedTheCircle > 0);
    CHECK(outOfTheCone > 0);
    CHECK(mismatches == 0);
    CHECK(outcomeMismatches == 0);
}

TEST_CASE("GroundAlignedProjectileBlock.GuardUpIsKUpOffTheGuard", "[BrawlerProjectile]")
{
    using brawlerProjectileSimulation::SlotOutcome;
    const glm::mat4 slopeGuard = guardPose(0.4f, true, glm::vec3(0.f));
    const Outcome bodyHit = run(Shot{ glm::vec3(0.f, 0.5f, 2.f), simFrame::kForward, slopeGuard, glm::vec3(0.f), false });
    CHECK(bodyHit.outcome == SlotOutcome::Hit);
    CHECK(sameBits(bodyHit.guardUp, simFrame::kUp));
    CHECK(sameBits(brawlerProjectileSimulation::SlotDetection{}.guardUp, simFrame::kUp));
}

#endif

// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "OGBrawler/BrawlerHitDetectionSystem.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "catch_amalgamated.hpp"

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGBrawler/DAttackCircle.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/SimCharacterId.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SurfaceFrame.h"

namespace groundAlignedMeleeBlockTests
{
    constexpr float kDt = 1.f / 60.f;
    constexpr std::uint32_t kWeaponBody = 30u;
    constexpr std::uint32_t kTargetRoot = 31u;
    constexpr std::uint32_t kTargetGuardBody = 32u;
    constexpr std::uint32_t kRadialVolume = 7u;
    constexpr SimCharacterId kTargetId = static_cast<SimCharacterId>(2u);

    const glm::vec3 kThirtyDegreeNormal(0.f, std::cos(glm::radians(30.f)), std::sin(glm::radians(30.f)));

    glm::mat4 slopeTilt()
    {
        return glm::mat4(surfaceFrame::tiltMatrix(kThirtyDegreeNormal));
    }

    glm::vec3 rotatePoint(const glm::mat4& m, const glm::vec3& p)
    {
        return glm::vec3(m * glm::vec4(p, 1.f));
    }

    struct Todays
    {
        bool blocks = false;
        float td = 0.f;
        float side = 0.f;
    };

    Todays todaysWouldGuardBlock(unsigned int activeAttackSequence, const glm::vec3& attackerRoot,
                                 const glm::mat4& guardTransform, const glm::vec3& guardOverlapPosition)
    {
        Todays out;
        const glm::vec3 guardForward = simFrame::forwardColumn(guardTransform);
        glm::vec3 collidingPosition = guardOverlapPosition;
        collidingPosition = simFrame::withHeight(collidingPosition, simFrame::height(attackerRoot));
        const glm::vec3 collisionDirection = attackerRoot - collidingPosition;
        const glm::vec3 normalizedCollisionDirection = glm::normalize(collisionDirection);
        const float shieldAngle = 0.25f;
        const float outerShieldAngle = glm::pi<float>() * 0.5;
        const float td = std::acos(glm::dot(normalizedCollisionDirection, guardForward));
        out.td = td;
        out.side = simFrame::turnComponent(glm::cross(normalizedCollisionDirection, guardForward));
        if (td < outerShieldAngle)
        {
            const glm::vec3 guardAxis = glm::cross(normalizedCollisionDirection, guardForward);
            if (td < shieldAngle)
            {
                if (activeAttackSequence == 4)
                    out.blocks = true;
            }
            else
            {
                if (simFrame::turnComponent(guardAxis) < 0.f && (activeAttackSequence == 0 || activeAttackSequence == 2))
                    out.blocks = true;
                if (simFrame::turnComponent(guardAxis) > 0.f && (activeAttackSequence == 1 || activeAttackSequence == 3))
                    out.blocks = true;
            }
        }
        return out;
    }

    glm::vec3 todaysIndicator(const DAttackCircle& circle, const glm::vec3& currentDirection,
                              const glm::vec3& rootTranslation, const glm::vec3& objectPosition, bool& missedTheCircle)
    {
        const glm::vec2 weaponDirXY = glm::normalize(simFrame::planar(currentDirection));
        const glm::vec2 attackerXY = simFrame::planar(rootTranslation);
        const glm::vec2 opponentXY = simFrame::planar(objectPosition);
        const glm::vec2 v = attackerXY - opponentXY;
        const float innerR = circle.getInnerRadius();
        const float outerR = circle.getOuterRadius();
        const float b = glm::dot(v, weaponDirXY);
        const float c = glm::dot(v, v) - innerR * innerR;
        const float discriminant = b * b - c;
        float t;
        missedTheCircle = !(discriminant >= 0.f);
        if (discriminant >= 0.f)
            t = -b - std::sqrt(discriminant);
        else
            t = glm::dot(opponentXY - attackerXY, weaponDirXY);
        t = glm::clamp(t, 0.f, outerR);
        return simFrame::compose(glm::vec2(attackerXY.x + t * weaponDirXY.x, attackerXY.y + t * weaponDirXY.y),
            simFrame::height(objectPosition));
    }

    bool sameBits(const glm::vec3& a, const glm::vec3& b)
    {
        return std::bit_cast<std::uint32_t>(a.x) == std::bit_cast<std::uint32_t>(b.x)
            && std::bit_cast<std::uint32_t>(a.y) == std::bit_cast<std::uint32_t>(b.y)
            && std::bit_cast<std::uint32_t>(a.z) == std::bit_cast<std::uint32_t>(b.z);
    }

    struct Physics
    {
        glm::mat4 weapon{ 1.f };
        glm::mat4 guard{ 1.f };

        glm::mat4 getBodyTransform(BodyId id) const
        {
            if (id == BodyId{ kTargetGuardBody })
                return guard;
            if (id == BodyId{ kWeaponBody })
                return weapon;
            return glm::mat4(1.f);
        }
    };

    struct Query
    {
        glm::vec3 objectPosition{ 0.f };

        SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const
        {
            SpatialQueryReport report;
            SpatialQueryHit body{};
            body.objectPosition = objectPosition;
            body.bodyId = BodyId{ kTargetRoot };
            body.rootBodyId = BodyId{ kTargetRoot };
            body.objectCategories = CollisionCategories::single(collisionCategory::body);
            report.hits.push_back(body);
            SpatialQueryHit guard{};
            guard.objectPosition = objectPosition;
            guard.bodyId = BodyId{ kTargetGuardBody };
            guard.rootBodyId = BodyId{ kTargetRoot };
            guard.objectCategories = CollisionCategories::single(collisionCategory::guard);
            report.hits.push_back(guard);
            return report;
        }

        void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    };

    const simulatableBrawler::StaticData& brawlerStaticData()
    {
        static const simulatableBrawler::StaticData staticData;
        return staticData;
    }

    struct Swing
    {
        dAttackRadialSimulation::InitialConditions ic;
        glm::mat4 initialRotation{ 1.f };
        glm::vec3 worldSequenceAxis{ 0.f };
        glm::vec3 swingUp{ 0.f };
        glm::vec3 currentDirection{ 0.f };
        glm::mat4 weapon{ 1.f };
        bool found = false;
    };

    Swing makeSwing(unsigned int sequence, float localHeading, bool slope, const glm::vec3& root)
    {
        const auto& radialStaticData = brawlerStaticData().m_attackSimulationStaticData;
        const auto& attackSequence = radialStaticData.getAttackSequences()[sequence];
        constexpr glm::vec3 up = simFrame::kUp;
        Swing swing;
        swing.ic.activeAttackSequence = sequence;
        const glm::mat4 aim = glm::rotate(glm::mat4(1.f), localHeading, up);
        if (slope)
        {
            const surfaceFrame::AngleAxis composite = surfaceFrame::toAngleAxis(glm::mat3(slopeTilt() * aim));
            swing.ic.initialAimAngle = composite.angle;
            swing.ic.initialAimRotationAxis = composite.axis;
        }
        else
        {
            swing.ic.initialAimAngle = localHeading;
            swing.ic.initialAimRotationAxis = up;
        }
        swing.initialRotation = glm::rotate(glm::mat4(1.f), swing.ic.initialAimAngle, swing.ic.initialAimRotationAxis);
        swing.worldSequenceAxis = glm::vec3(swing.initialRotation * glm::vec4(attackSequence.getRotationAxis(), 0.f));
        swing.swingUp = glm::mat3(swing.initialRotation) * up;
        for (int step = 0; step < 720 && !swing.found; ++step)
        {
            const float phi = glm::two_pi<float>() * static_cast<float>(step) / 720.f;
            const glm::mat4 rotation = swing.initialRotation * glm::rotate(glm::mat4(1.f), phi, attackSequence.getRotationAxis());
            const glm::vec3 direction = glm::vec3(rotation * glm::vec4(DAttackRadialSequence::defaultForward(), 0.f));
            if (std::abs(glm::dot(direction, swing.swingUp)) >= 0.5f)
                continue;
            const auto segment = dAttackRadialSimulation::getAttackSegment(swing.ic, radialStaticData, direction);
            if (segment.state != DAttackRadialSequenceState::Damaging)
                continue;
            swing.found = true;
            swing.currentDirection = direction;
            swing.weapon = rotation;
            swing.weapon[3] = glm::vec4(root, 1.f);
        }
        return swing;
    }

    bool findBlockingGuard(unsigned int sequence, bool slope, const glm::vec3& root, const glm::vec3& objectPosition,
                           glm::mat4& guardOut)
    {
        constexpr glm::vec3 up = simFrame::kUp;
        for (int step = 0; step < 360; ++step)
        {
            const float psi = glm::two_pi<float>() * static_cast<float>(step) / 360.f;
            glm::mat4 guard = glm::rotate(glm::mat4(1.f), psi, up);
            if (slope)
                guard = slopeTilt() * guard;
            guard[3] = glm::vec4(objectPosition, 1.f);
            if (dAttackRadialSimulation::wouldGuardBlock(sequence, root, guard, objectPosition))
            {
                guardOut = guard;
                return true;
            }
        }
        return false;
    }

    struct Detection
    {
        bool ran = false;
        bool blocked = false;
        std::size_t guardHits = 0;
        glm::vec3 marker{ 0.f };
    };

    Detection detect(const Swing& swing, const glm::mat4& guard, const glm::vec3& objectPosition)
    {
        const auto& radialStaticData = brawlerStaticData().m_attackSimulationStaticData;
        dAttackRadialSimulation::State state;
        state.attackTimer = 0.2f;
        state.currenSequenceId = swing.ic.activeAttackSequence;
        dAttackRadialSimulation::RuntimeBindings bindings;
        bindings.ownBodyId = BodyId{ kWeaponBody };
        bindings.queryVolumeIds = { QueryVolumeId{ kRadialVolume } };
        dAttackRadialSimulation::DerivedState derived;
        Physics physics;
        physics.weapon = swing.weapon;
        physics.guard = guard;
        Query query;
        query.objectPosition = objectPosition;
        auto resolve = [](BodyId id) { return id == BodyId{ kTargetRoot } ? kTargetId : SimCharacterId::None; };
        brawlerHitDetection::detectRadialHits(kDt, radialStaticData, swing.ic, state, bindings, derived, physics, query, resolve);
        Detection out;
        out.ran = true;
        out.blocked = derived.getGuardBlockedThisTick();
        out.guardHits = derived.getGuardHits().size();
        if (!derived.getGuardHits().empty())
            out.marker = derived.getGuardHits().front().position;
        return out;
    }
}

using namespace groundAlignedMeleeBlockTests;

TEST_CASE("GroundAlignedMeleeBlock.ConjugationIdentityOnTheThirtyDegreePlane", "[DAttack]")
{
    std::mt19937 rng(0x6a7u);
    std::uniform_real_distribution<float> unit(-1.f, 1.f);
    std::uniform_real_distribution<float> heading(-glm::pi<float>(), glm::pi<float>());
    std::uniform_int_distribution<unsigned int> sequencePick(0u, 4u);
    constexpr glm::vec3 up = simFrame::kUp;
    const glm::mat4 tilt = slopeTilt();
    const float shieldAngle = 0.25f;
    const float outerShieldAngle = glm::half_pi<float>();
    const float margin = 1.e-4f;

    int accepted = 0;
    int generated = 0;
    int mismatches = 0;
    int flatVersusToday = 0;
    int slopeBranchMissed = 0;
    int blocked[5] = { 0, 0, 0, 0, 0 };
    int notBlocked[5] = { 0, 0, 0, 0, 0 };
    while (accepted < 2000 && generated < 200000)
    {
        ++generated;
        const glm::vec3 guardPosition(4.f * unit(rng), 1.5f * unit(rng), 4.f * unit(rng));
        glm::mat4 guard = glm::rotate(glm::mat4(1.f), heading(rng), up);
        guard[3] = glm::vec4(guardPosition, 1.f);
        const glm::vec3 overlap = guardPosition + glm::vec3(0.4f * unit(rng), 0.4f * unit(rng), 0.4f * unit(rng));
        const float attackerHeading = heading(rng);
        const float attackerDistance = 1.75f + 1.25f * unit(rng);
        const glm::vec3 attackerRoot = overlap
            + simFrame::withHeight(glm::vec3(0.f), 0.6f * unit(rng))
            + glm::vec3(glm::rotate(glm::mat4(1.f), attackerHeading, up) * glm::vec4(simFrame::kForward, 0.f)) * attackerDistance;
        const unsigned int sequence = sequencePick(rng);

        const Todays today = todaysWouldGuardBlock(sequence, attackerRoot, guard, overlap);
        if (std::abs(today.td - shieldAngle) <= margin || std::abs(today.td - outerShieldAngle) <= margin
            || std::abs(today.side) <= margin)
            continue;
        ++accepted;

        const bool flat = dAttackRadialSimulation::wouldGuardBlock(sequence, attackerRoot, guard, overlap);
        if (flat != today.blocks)
            ++flatVersusToday;

        const glm::mat4 slopeGuard = tilt * guard;
        if (surfaceFrame::isFlat(surfaceFrame::surfaceUpOf(slopeGuard)))
            ++slopeBranchMissed;
        const bool onSlope = dAttackRadialSimulation::wouldGuardBlock(
            sequence, rotatePoint(tilt, attackerRoot), slopeGuard, rotatePoint(tilt, overlap));
        if (onSlope != flat)
            ++mismatches;
        (flat ? blocked : notBlocked)[sequence] += 1;
    }

    std::printf("[GroundAlignedMeleeBlock.identity] accepted=%d generated=%d mismatches=%d flatVersusToday=%d "
                "slopeBranchMissed=%d blocked=(%d,%d,%d,%d,%d) notBlocked=(%d,%d,%d,%d,%d)\n",
                accepted, generated, mismatches, flatVersusToday, slopeBranchMissed,
                blocked[0], blocked[1], blocked[2], blocked[3], blocked[4],
                notBlocked[0], notBlocked[1], notBlocked[2], notBlocked[3], notBlocked[4]);
    REQUIRE(accepted == 2000);
    CHECK(mismatches == 0);
    CHECK(flatVersusToday == 0);
    CHECK(slopeBranchMissed == 0);
    for (int sequence = 0; sequence < 5; ++sequence)
    {
        CHECK(blocked[sequence] > 0);
        CHECK(notBlocked[sequence] > 0);
    }
}

TEST_CASE("GroundAlignedMeleeBlock.OverheadFromStraightDownSlopeBlocksAGuardFacingDownSlope", "[DAttack]")
{
    constexpr glm::vec3 up = simFrame::kUp;
    const glm::mat4 tilt = slopeTilt();
    const glm::vec3 downSlope = surfaceFrame::tiltToSurface(kThirtyDegreeNormal, -simFrame::kForward);
    const glm::vec3 pivot = simFrame::withHeight(glm::vec3(0.f), 0.3f);
    const glm::vec3 overlap = pivot;
    const glm::vec3 attackerRoot = downSlope * 2.f + pivot;
    glm::mat4 guard = tilt * glm::rotate(glm::mat4(1.f), glm::pi<float>(), up);
    guard[3] = glm::vec4(simFrame::withHeight(glm::vec3(0.f), 0.3f), 1.f);

    const Todays today = todaysWouldGuardBlock(4u, attackerRoot, guard, overlap);
    std::printf("[GroundAlignedMeleeBlock.overhead] todaysTd=%.7f rad (%.4f deg) todaysBlocks=%d slopeBlocks=%d "
                "forwardDotDownSlope=%.7f\n",
                today.td, glm::degrees(today.td), today.blocks ? 1 : 0,
                dAttackRadialSimulation::wouldGuardBlock(4u, attackerRoot, guard, overlap) ? 1 : 0,
                glm::dot(simFrame::forwardColumn(guard), downSlope));
    CHECK(glm::dot(simFrame::forwardColumn(guard), downSlope) > 0.99999f);
    CHECK(std::abs(today.td - glm::radians(30.f)) < 1.e-4f);
    CHECK_FALSE(today.blocks);
    CHECK(dAttackRadialSimulation::wouldGuardBlock(4u, attackerRoot, guard, overlap));
    for (unsigned int sequence = 0; sequence < 4; ++sequence)
        CHECK_FALSE(dAttackRadialSimulation::wouldGuardBlock(sequence, attackerRoot, guard, overlap));

    const glm::vec3 straightAbove = overlap + kThirtyDegreeNormal * 2.f;
    CHECK_FALSE(dAttackRadialSimulation::wouldGuardBlock(4u, straightAbove, guard, overlap));
}

TEST_CASE("GroundAlignedMeleeBlock.SlopeMarkerLiesInTheSwingPlaneOnTheInnerCircle", "[DAttack]")
{
    const DAttackCircle& circle = brawlerStaticData().m_attackCircle;
    const glm::vec3 root(1.f, 2.f, -3.f);
    const float headings[] = { 0.4f, 1.3f, 2.6f, -0.7f, -2.2f };
    int checked = 0;
    float worstPlane = 0.f;
    float worstCircle = 0.f;
    float worstAlongWeapon = 1.f;
    for (unsigned int sequence = 0; sequence < 5; ++sequence)
    {
        for (const float localHeading : headings)
        {
            const Swing swing = makeSwing(sequence, localHeading, true, root);
            REQUIRE(swing.found);
            REQUIRE_FALSE(surfaceFrame::isFlat(swing.swingUp));
            const glm::vec3 objectPosition = root + swing.currentDirection * 1.5f;
            glm::mat4 guard(1.f);
            REQUIRE(findBlockingGuard(sequence, true, root, objectPosition, guard));
            const Detection detection = detect(swing, guard, objectPosition);
            REQUIRE(detection.blocked);
            REQUIRE(detection.guardHits == 1u);
            const glm::vec3 fromRoot = detection.marker - root;
            const glm::vec3 fromOpponent = detection.marker - objectPosition;
            const glm::vec3 inPlane = fromOpponent - swing.swingUp * glm::dot(fromOpponent, swing.swingUp);
            worstPlane = std::max(worstPlane, std::abs(glm::dot(fromRoot, swing.worldSequenceAxis)));
            worstCircle = std::max(worstCircle, std::abs(glm::length(inPlane) - circle.getInnerRadius()));
            worstAlongWeapon = std::min(worstAlongWeapon, glm::dot(glm::normalize(fromRoot), swing.currentDirection));
            ++checked;
        }
    }
    std::printf("[GroundAlignedMeleeBlock.marker] checked=%d worstOffSwingPlane=%.3g m worstOffInnerCircle=%.3g m "
                "worstCosToWeapon=%.7f\n",
                checked, worstPlane, worstCircle, worstAlongWeapon);
    CHECK(checked == 25);
    CHECK(worstPlane <= 1.e-4f);
    CHECK(worstCircle <= 1.e-4f);
    CHECK(worstAlongWeapon > 0.9999f);
}

TEST_CASE("GroundAlignedMeleeBlock.FlatMarkerReproducesTodaysIndicator", "[DAttack]")
{
    const DAttackCircle& circle = brawlerStaticData().m_attackCircle;
    const glm::vec3 roots[] = { glm::vec3(1.f, 2.f, -3.f), glm::vec3(-7.25f, 0.3f, 11.5f) };
    const float headings[] = { 0.4f, 1.3f, 2.6f, -0.7f, -2.2f, 0.f };
    const float distances[] = { 1.5f, 2.4f, 1.f };
    const float axialOffsets[] = { 0.f, 0.2f, -0.3f };
    const float lateralOffsets[] = { 0.f, 1.f, -1.f };
    int compared = 0;
    int mismatches = 0;
    int missedTheCircle = 0;
    for (const glm::vec3& root : roots)
    {
        for (unsigned int sequence = 0; sequence < 5; ++sequence)
        {
            for (const float localHeading : headings)
            {
                const Swing swing = makeSwing(sequence, localHeading, false, root);
                REQUIRE(swing.found);
                REQUIRE(surfaceFrame::isFlat(swing.swingUp));
                for (const float distance : distances)
                {
                    for (const float axial : axialOffsets)
                    for (const float lateral : lateralOffsets)
                    {
                        const glm::vec3 across = glm::cross(swing.worldSequenceAxis, swing.currentDirection);
                        const glm::vec3 objectPosition = root + swing.currentDirection * distance
                            + swing.worldSequenceAxis * axial + across * lateral;
                        glm::mat4 guard(1.f);
                        if (!findBlockingGuard(sequence, false, root, objectPosition, guard))
                            continue;
                        const Detection detection = detect(swing, guard, objectPosition);
                        if (detection.guardHits != 1u)
                            continue;
                        bool missed = false;
                        const glm::vec3 expected = todaysIndicator(circle, swing.currentDirection, root, objectPosition, missed);
                        if (!sameBits(detection.marker, expected))
                            ++mismatches;
                        ++compared;
                        missedTheCircle += missed ? 1 : 0;
                    }
                }
            }
        }
    }
    std::printf("[GroundAlignedMeleeBlock.flatMarker] compared=%d missedTheCircle=%d mismatches=%d\n",
                compared, missedTheCircle, mismatches);
    CHECK(compared >= 300);
    CHECK(missedTheCircle > 0);
    CHECK(mismatches == 0);
}

#endif

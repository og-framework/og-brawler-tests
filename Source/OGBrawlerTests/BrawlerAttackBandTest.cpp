// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "OGBrawler/BrawlerHitDetectionSystem.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <variant>
#include <vector>

#include "catch_amalgamated.hpp"

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"

#include "OGBrawler/CollisionCategoryConstants.h"
#include "OGBrawler/DAttackCircle.h"
#include "OGBrawler/DAttackDirectionClassifier.h"
#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/DAttackRadialSimulation.h"
#include "OGBrawler/SimCharacterId.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SurfaceFrame.h"

namespace brawlerAttackBandTests
{
    constexpr float kDt = 1.f / 60.f;
    constexpr std::uint32_t kWeaponBody = 40u;
    constexpr std::uint32_t kTargetRoot = 41u;
    constexpr std::uint32_t kTargetGuardBody = 42u;
    constexpr std::uint32_t kRadialVolume = 9u;
    constexpr SimCharacterId kTargetId = static_cast<SimCharacterId>(3u);
    constexpr unsigned int kOverheadSequence = 4u;

    const simulatableBrawler::StaticData& brawlerStaticData()
    {
        static const simulatableBrawler::StaticData staticData;
        return staticData;
    }

    const dAttackRadialSimulation::StaticData& radialStaticData()
    {
        return brawlerStaticData().m_attackSimulationStaticData;
    }

    const DAttackCircle& attackCircle()
    {
        return radialStaticData().getAttackCircle();
    }

    brawlerHitDetection::TargetCapsule shippedCapsule()
    {
        const auto& movement = brawlerStaticData().m_movementStaticData;
        return brawlerHitDetection::TargetCapsule{ movement.capsuleRadius, movement.capsuleHalfHeight };
    }

    float pivotOffset()
    {
        return simFrame::height(dAttackRadialSimulation::PhysicsDeclaration::attachmentOffset(radialStaticData()));
    }

    glm::mat3 thirtyDegreeTilt()
    {
        const glm::vec3 normal(0.f, std::cos(glm::radians(30.f)), std::sin(glm::radians(30.f)));
        return surfaceFrame::tiltMatrix(normal);
    }

    bool pointTestAdmits(const glm::vec3& pivot, const glm::vec3& axis, const glm::vec3& centre)
    {
        const glm::vec3 hitDirection = centre - pivot;
        const float signedDistanceAlongRotationAxis = glm::dot(hitDirection, axis);
        const float lengthAlongRotationAxis = glm::abs(signedDistanceAlongRotationAxis);
        const glm::vec3 hitDirectionOnRotationPlane = hitDirection - signedDistanceAlongRotationAxis * axis;
        const float hitDistance = glm::length(hitDirectionOnRotationPlane);
        return hitDistance > attackCircle().getInnerRadius() && hitDistance < attackCircle().getOuterRadius()
            && lengthAlongRotationAxis < attackCircle().getHalfThickness();
    }

    float sampledDistance(const glm::vec3& pivot, const glm::vec3& axis, const glm::vec3& centre,
                          const brawlerHitDetection::TargetCapsule& capsule, int kReferenceSamples)
    {
        const double h = double(capsule.halfHeight) - double(capsule.radius);
        double best = 1.e30;
        for (int k = 0; k < kReferenceSamples; ++k)
        {
            const double s = h * (2.0 * double(k) / double(kReferenceSamples - 1) - 1.0);
            const double qx = double(centre.x) - double(pivot.x) + s * double(simFrame::kUp.x);
            const double qy = double(centre.y) - double(pivot.y) + s * double(simFrame::kUp.y);
            const double qz = double(centre.z) - double(pivot.z) + s * double(simFrame::kUp.z);
            const double z = qx * double(axis.x) + qy * double(axis.y) + qz * double(axis.z);
            const double px = qx - z * double(axis.x);
            const double py = qy - z * double(axis.y);
            const double pz = qz - z * double(axis.z);
            const double rho = std::sqrt(px * px + py * py + pz * pz);
            const double dRho = std::max(std::max(double(attackCircle().getInnerRadius()) - rho,
                                                  rho - double(attackCircle().getOuterRadius())), 0.0);
            const double dZ = std::max(std::abs(z) - double(attackCircle().getHalfThickness()), 0.0);
            best = std::min(best, std::sqrt(dRho * dRho + dZ * dZ));
        }
        return float(best);
    }

    float referenceDistance(const glm::vec3& pivot, const glm::vec3& axis, const glm::vec3& centre,
                            const brawlerHitDetection::TargetCapsule& capsule)
    {
        const float coarse = sampledDistance(pivot, axis, centre, capsule, 1001);
        if (coarse > capsule.radius + 0.01f)
            return coarse;
        return sampledDistance(pivot, axis, centre, capsule, 10001);
    }

    struct BandAxis
    {
        const char* name;
        glm::vec3 axis;
    };

    std::vector<BandAxis> bandAxes()
    {
        const glm::vec3 side = DAttackRadialSequence::turnAxis();
        const glm::vec3 overhead = radialStaticData().getAttackSequences()[kOverheadSequence].getRotationAxis();
        const glm::mat3 tilt = thirtyDegreeTilt();
        return { { "sideFlat", side }, { "overheadFlat", overhead },
                 { "sideTilted30", glm::normalize(tilt * side) }, { "overheadTilted30", glm::normalize(tilt * overhead) } };
    }

    template <typename Visit>
    void forEachGridCentre(const glm::vec3& attackerRoot, Visit&& visit)
    {
        constexpr int kHeadings = 8;
        for (int heading = 0; heading < kHeadings; ++heading)
        {
            const float angle = glm::two_pi<float>() * static_cast<float>(heading) / static_cast<float>(kHeadings) + 0.1f;
            const glm::vec3 across = simFrame::kForward * std::cos(angle) + simFrame::kRight * std::sin(angle);
            for (int r = 0; r <= 80; ++r)
            {
                const float rho = 0.05f * static_cast<float>(r);
                for (int z = -40; z <= 40; ++z)
                {
                    const float height = 0.05f * static_cast<float>(z);
                    visit(attackerRoot + across * rho + simFrame::kUp * height, rho, height);
                }
            }
        }
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
        bool guardUp = false;

        SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const
        {
            SpatialQueryReport report;
            SpatialQueryHit body{};
            body.objectPosition = objectPosition;
            body.bodyId = BodyId{ kTargetRoot };
            body.rootBodyId = BodyId{ kTargetRoot };
            body.objectCategories = CollisionCategories::single(collisionCategory::body);
            report.hits.push_back(body);
            if (guardUp)
            {
                SpatialQueryHit guard{};
                guard.objectPosition = objectPosition;
                guard.bodyId = BodyId{ kTargetGuardBody };
                guard.rootBodyId = BodyId{ kTargetRoot };
                guard.objectCategories = CollisionCategories::single(collisionCategory::guard);
                report.hits.push_back(guard);
            }
            return report;
        }

        void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    };

    struct Swing
    {
        dAttackRadialSimulation::InitialConditions ic;
        glm::vec3 worldSequenceAxis{ 0.f };
        glm::vec3 currentDirection{ 0.f };
        glm::mat4 weapon{ 1.f };
        bool found = false;
    };

    Swing flatSwing(unsigned int sequence, float heading, const glm::vec3& pivot)
    {
        const auto& attackSequence = radialStaticData().getAttackSequences()[sequence];
        constexpr glm::vec3 up = simFrame::kUp;
        Swing swing;
        swing.ic.activeAttackSequence = sequence;
        swing.ic.initialAimAngle = heading;
        swing.ic.initialAimRotationAxis = up;
        const glm::mat4 initialRotation = glm::rotate(glm::mat4(1.f), heading, up);
        swing.worldSequenceAxis = glm::vec3(initialRotation * glm::vec4(attackSequence.getRotationAxis(), 0.f));
        for (int step = 0; step < 720 && !swing.found; ++step)
        {
            const float phi = glm::two_pi<float>() * static_cast<float>(step) / 720.f;
            const glm::mat4 rotation = initialRotation * glm::rotate(glm::mat4(1.f), phi, attackSequence.getRotationAxis());
            const glm::vec3 direction = glm::vec3(rotation * glm::vec4(DAttackRadialSequence::defaultForward(), 0.f));
            if (std::abs(simFrame::height(direction)) > 0.5f && sequence != kOverheadSequence)
                continue;
            const auto segment = dAttackRadialSimulation::getAttackSegment(swing.ic, radialStaticData(), direction);
            if (segment.state != DAttackRadialSequenceState::Damaging)
                continue;
            swing.found = true;
            swing.currentDirection = direction;
            swing.weapon = rotation;
            swing.weapon[3] = glm::vec4(pivot, 1.f);
        }
        return swing;
    }

    struct Detection
    {
        std::size_t hits = 0;
        std::size_t guardHits = 0;
        bool blocked = false;
        bool finite = true;
    };

    bool isFinite(const glm::vec3& v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    Detection detect(const Swing& swing, const glm::vec3& objectPosition, bool guardUp = false,
                     const glm::mat4& guard = glm::mat4(1.f))
    {
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
        query.guardUp = guardUp;
        auto resolve = [](BodyId id) { return id == BodyId{ kTargetRoot } ? kTargetId : SimCharacterId::None; };
        brawlerHitDetection::detectRadialHits(kDt, radialStaticData(), shippedCapsule(), swing.ic, state, bindings, derived,
            physics, query, resolve);
        Detection out;
        out.hits = derived.getHitsThisTick().size();
        out.guardHits = derived.getGuardHits().size();
        out.blocked = derived.getGuardBlockedThisTick();
        for (const auto& hit : derived.getHitsThisTick())
            out.finite = out.finite && isFinite(hit.position) && isFinite(hit.swingTangent);
        for (const auto& hit : derived.getGuardHits())
            out.finite = out.finite && isFinite(hit.position);
        return out;
    }

    const glm::vec3 kAttackerRoot(1.25f, 2.f, -3.5f);
}

using namespace brawlerAttackBandTests;

TEST_CASE("BrawlerAttackBand: the shipped values the cases below are written against", "[DAttack]")
{
    CHECK(attackCircle().getInnerRadius() == 0.9f);
    CHECK(attackCircle().getOuterRadius() == 3.f);
    CHECK(attackCircle().getHalfThickness() == 0.35f);
    CHECK(shippedCapsule().radius == 0.42f);
    CHECK(shippedCapsule().halfHeight == 0.96f);
    CHECK(std::abs(pivotOffset() - 0.3f) < 1.e-6f);
}

TEST_CASE("BrawlerAttackBand: a flat side swing reaches a target from 1.01 m below to 1.61 m above the attacker's root", "[DAttack]")
{
    const glm::vec3 pivot = kAttackerRoot + simFrame::kUp * pivotOffset();
    constexpr glm::vec3 up = simFrame::kUp;
    struct Probe
    {
        float relativeHeight;
        bool hits;
    };
    const Probe probes[] = { { -1.02f, false }, { -1.f, true }, { -0.5f, true }, { 0.f, true }, { 0.5f, true },
                             { 1.f, true }, { 1.6f, true }, { 1.62f, false } };
    int checked = 0;
    for (const unsigned int sequence : { dAttackDirection::kLeftSequenceId, dAttackDirection::kRightSequenceId })
        for (const float heading : { 0.f, 0.9f, -2.3f })
        {
            const Swing swing = flatSwing(sequence, heading, pivot);
            REQUIRE(swing.found);
            REQUIRE(std::abs(glm::dot(swing.worldSequenceAxis, up)) > 0.99999f);
            for (const Probe& probe : probes)
            {
                const glm::vec3 centre = simFrame::withHeight(pivot + swing.currentDirection * 2.f,
                    simFrame::height(kAttackerRoot) + probe.relativeHeight);
                const Detection detection = detect(swing, centre);
                INFO("sequence " << sequence << " heading " << heading << " relative height " << probe.relativeHeight);
                CHECK(detection.hits == (probe.hits ? 1u : 0u));
                CHECK(detection.finite);
                ++checked;
            }
        }
    CHECK(checked == 48);
}

TEST_CASE("BrawlerAttackBand: at the attacker's height the reach is 0.48 m to 3.42 m on the swing plane", "[DAttack]")
{
    const glm::vec3 pivot = kAttackerRoot + simFrame::kUp * pivotOffset();
    struct Probe
    {
        float distance;
        bool hits;
    };
    const Probe probes[] = { { 0.47f, false }, { 0.49f, true }, { 1.5f, true }, { 3.41f, true }, { 3.43f, false } };
    for (const unsigned int sequence : { dAttackDirection::kLeftSequenceId, dAttackDirection::kRightSequenceId })
    {
        const Swing swing = flatSwing(sequence, 0.4f, pivot);
        REQUIRE(swing.found);
        for (const Probe& probe : probes)
        {
            const glm::vec3 centre = simFrame::withHeight(pivot + swing.currentDirection * probe.distance,
                simFrame::height(kAttackerRoot));
            INFO("sequence " << sequence << " distance " << probe.distance);
            CHECK(detect(swing, centre).hits == (probe.hits ? 1u : 0u));
        }
    }
}

TEST_CASE("BrawlerAttackBand: every centre the point test admits is admitted", "[DAttack]")
{
    const glm::vec3 pivot = kAttackerRoot + simFrame::kUp * pivotOffset();
    const brawlerHitDetection::TargetCapsule capsule = shippedCapsule();
    for (const BandAxis& band : bandAxes())
    {
        int pointAdmits = 0;
        int capsuleAdmits = 0;
        int lost = 0;
        forEachGridCentre(kAttackerRoot, [&](const glm::vec3& centre, float, float) {
            const bool point = pointTestAdmits(pivot, band.axis, centre);
            const bool overlaps = brawlerHitDetection::capsuleOverlapsAttackBand(pivot, band.axis, attackCircle(), centre, capsule);
            pointAdmits += point ? 1 : 0;
            capsuleAdmits += overlaps ? 1 : 0;
            lost += point && !overlaps ? 1 : 0;
        });
        std::printf("[BrawlerAttackBand.superset] axis=%s centres=%d pointAdmits=%d capsuleAdmits=%d lost=%d\n", band.name,
            8 * 81 * 81, pointAdmits, capsuleAdmits, lost);
        INFO("axis " << band.name);
        CHECK(pointAdmits > 0);
        CHECK(capsuleAdmits > pointAdmits);
        CHECK(lost == 0);
    }
}

TEST_CASE("BrawlerAttackBand: against a 10 001-sample reference there is no false hit and every miss is within the sample spacing", "[DAttack]")
{
    const glm::vec3 pivot = kAttackerRoot + simFrame::kUp * pivotOffset();
    const brawlerHitDetection::TargetCapsule capsule = shippedCapsule();
    const float h = capsule.halfHeight - capsule.radius;
    const float spacingBound = h / static_cast<float>(brawlerHitDetection::kCapsuleBandSamples - 1);
    CHECK(spacingBound < 0.034f);
    for (const BandAxis& band : bandAxes())
    {
        int overlaps = 0;
        int falseHits = 0;
        int falseMisses = 0;
        int missesTooDeep = 0;
        float deepestMiss = 0.f;
        forEachGridCentre(kAttackerRoot, [&](const glm::vec3& centre, float, float) {
            const float reference = referenceDistance(pivot, band.axis, centre, capsule);
            const bool hits = brawlerHitDetection::capsuleOverlapsAttackBand(pivot, band.axis, attackCircle(), centre, capsule);
            const bool referenceHits = reference < capsule.radius;
            overlaps += referenceHits ? 1 : 0;
            falseHits += hits && !referenceHits ? 1 : 0;
            if (!hits && referenceHits)
            {
                ++falseMisses;
                deepestMiss = std::max(deepestMiss, capsule.radius - reference);
                missesTooDeep += reference <= capsule.radius - spacingBound ? 1 : 0;
            }
        });
        std::printf("[BrawlerAttackBand.reference] axis=%s centres=%d referenceOverlaps=%d falseHits=%d falseMisses=%d "
                    "deepestMissM=%.6f boundM=%.6f\n",
            band.name, 8 * 81 * 81, overlaps, falseHits, falseMisses, double(deepestMiss), double(spacingBound));
        INFO("axis " << band.name);
        CHECK(overlaps > 0);
        CHECK(falseHits == 0);
        CHECK(missesTooDeep == 0);
        if (std::string(band.name) == "sideFlat")
            CHECK(falseMisses == 0);
    }
}

TEST_CASE("BrawlerAttackBand: the 6 m query reports every target whose capsule the band can reach", "[DAttack]")
{
    const auto volumes = dAttackRadialSimulation::PhysicsSetup::queryVolumes(radialStaticData());
    REQUIRE(!volumes.empty());
    REQUIRE(std::holds_alternative<SphereGeometry>(volumes.back().geometry));
    const float queryRadius = std::get<SphereGeometry>(volumes.back().geometry).radius;
    const auto& shapes = dAttackRadialSimulation::PhysicsSetup::body.shapes;
    REQUIRE(shapes.size() == 1u);
    REQUIRE(std::holds_alternative<SphereGeometry>(shapes.front().geometry));
    const float weaponRadius = std::get<SphereGeometry>(shapes.front().geometry).radius;
    const brawlerHitDetection::TargetCapsule capsule = shippedCapsule();
    const float h = capsule.halfHeight - capsule.radius;
    const float outer = attackCircle().getOuterRadius();
    const float halfThickness = attackCircle().getHalfThickness();
    const float farthestReach = std::sqrt(outer * outer + halfThickness * halfThickness) + capsule.radius + h + pivotOffset();
    std::printf("[BrawlerAttackBand.broadphase] queryRadiusM=%.4f weaponRadiusM=%.4f reportedWithinM=%.4f farthestWeaponCentreM=%.4f\n",
        double(queryRadius), double(weaponRadius), double(queryRadius + weaponRadius), double(farthestReach));
    CHECK(queryRadius == 2.f * outer);
    CHECK(queryRadius + weaponRadius >= farthestReach);
}

TEST_CASE("BrawlerAttackBand: a centre on the swing axis is no hit and makes no NaN", "[DAttack]")
{
    const glm::vec3 pivot = kAttackerRoot + simFrame::kUp * pivotOffset();
    const brawlerHitDetection::TargetCapsule capsule = shippedCapsule();

    const Swing side = flatSwing(dAttackDirection::kLeftSequenceId, 0.4f, pivot);
    REQUIRE(side.found);
    const glm::vec3 onSideAxis = pivot + side.worldSequenceAxis * 0.2f;
    CHECK_FALSE(brawlerHitDetection::capsuleOverlapsAttackBand(pivot, side.worldSequenceAxis, attackCircle(), onSideAxis, capsule));
    const Detection sideDetection = detect(side, onSideAxis);
    CHECK(sideDetection.hits == 0u);
    CHECK(sideDetection.finite);

    const Swing overhead = flatSwing(kOverheadSequence, 0.f, pivot);
    REQUIRE(overhead.found);
    REQUIRE(glm::dot(overhead.worldSequenceAxis, simFrame::kUp) == 0.f);
    int checked = 0;
    for (const float along : { 0.f, 0.25f, -0.25f, 0.5f, -0.5f })
    {
        const glm::vec3 onOverheadAxis = pivot + overhead.worldSequenceAxis * along;
        const glm::vec3 offPlane = onOverheadAxis - pivot - glm::dot(onOverheadAxis - pivot, overhead.worldSequenceAxis) * overhead.worldSequenceAxis;
        INFO("along " << along);
        REQUIRE(glm::dot(offPlane, offPlane) <= 0.f);
        CHECK(brawlerHitDetection::capsuleOverlapsAttackBand(pivot, overhead.worldSequenceAxis, attackCircle(), onOverheadAxis, capsule));
        for (const bool guardUp : { false, true })
        {
            const Detection detection = detect(overhead, onOverheadAxis, guardUp);
            INFO("guardUp " << guardUp);
            CHECK(detection.hits == 0u);
            CHECK(detection.guardHits == 0u);
            CHECK_FALSE(detection.blocked);
            CHECK(detection.finite);
            ++checked;
        }
    }
    CHECK(checked == 10);
}

#endif

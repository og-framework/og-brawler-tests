// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "OGBrawler/DAttackGuardSimulation.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "catch_amalgamated.hpp"

#include "glm/geometric.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"

#include "OGBrawler/BrawlerGroundFrame.h"
#include "OGBrawler/DAttackCircle.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/SpatialQueryResult.h"
#include "OGSimulation/SurfaceFrame.h"

namespace guardGroundFrameTests
{
    struct MockPhysicsAdapter
    {
        std::vector<glm::mat4> transforms;

        explicit MockPhysicsAdapter(std::size_t bodyCount)
            : transforms(bodyCount, glm::mat4(1.f))
        {
        }

        glm::mat4 getBodyTransform(BodyId id) const { return transforms[id.value]; }
        void setBodyTransform(BodyId id, const glm::mat4& t) { transforms[id.value] = t; }
        void setBodyLinearVelocity(BodyId, const glm::vec3&) {}
        void addBodyTorque(BodyId, const glm::vec3&) {}
        void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
        void addBodyAcceleration(BodyId, const glm::vec3&) {}
        void addBodyVelocityChange(BodyId, const glm::vec3&) {}
        glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
        PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
    };

    static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);

    struct MockSpatialQueryAdapter
    {
        SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return SpatialQueryReport{}; }
        SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
        void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
        void enableShape(ShapeId) {}
        void disableShape(ShapeId) {}
    };

    static_assert(SpatialQueryAdapter<MockSpatialQueryAdapter>);

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kOwnBodyValue = 0u;
    constexpr uint32_t kParentBodyValue = 1u;
    constexpr glm::vec3 kAttachOffset = glm::vec3(0.f, 0.3f, -0.f);
    constexpr glm::vec3 kParentPos = glm::vec3(-2.5f, 0.07f, -1.f);

    const glm::vec3 kThirtyDegreeNormal(0.f, std::cos(glm::radians(30.f)), std::sin(glm::radians(30.f)));

    brawlerGround::GroundSample slopeSample()
    {
        brawlerGround::GroundSample sample;
        sample.normal = kThirtyDegreeNormal;
        return sample;
    }

    struct GuardTick
    {
        glm::mat4 transform{1.f};
        dAttackGuardSimulation::State state{};
    };

    GuardTick tick(const glm::vec3& aim, const brawlerGround::GroundSample& ground)
    {
        using namespace dAttackGuardSimulation;

        DAttackCircle circle{8u, 0.5f, 1.f, 0.1f, false, 1.f};
        StaticData staticData{circle};
        DerivedState derived{};

        dAttackMachineSimulation::State machine{};
        machine.m_currentState = DAttackState::Idle;

        SimulationComposite<InitialConditions, State, dAttackMachineSimulation::State> composite(
            InitialConditions{}, State{}, machine);
        auto deps = makeDependencies<Dependencies>(composite);

        MockPhysicsAdapter physics{2};
        physics.transforms[kParentBodyValue] = glm::translate(glm::mat4(1.f), kParentPos);
        MockSpatialQueryAdapter query{};

        PlayerInputView playerInput{};
        playerInput.aimDirection = aim;
        IntegrationUtils<MockPhysicsAdapter, MockSpatialQueryAdapter> utils{kDt, physics, query};
        AllInput<MockPhysicsAdapter, MockSpatialQueryAdapter> allInput{playerInput, utils};

        RuntimeBindings bindings{};
        bindings.ownBodyId = BodyId{kOwnBodyValue};
        bindings.parentBodyId = BodyId{kParentBodyValue};
        bindings.attachmentOffset = kAttachOffset;
        bindings.shapeIds = {ShapeId{7u}};

        integrate(kDt, allInput, staticData, deps, bindings, derived, ground);

        return GuardTick{physics.transforms[kOwnBodyValue], composite.get<State>()};
    }

    glm::mat4 todaysGuardTransform(const glm::vec3& aim)
    {
        const glm::vec3 rootTranslation = glm::vec3(glm::translate(glm::mat4(1.f), kParentPos)[3]) + kAttachOffset;
        const glm::vec3 rawAim = simFrame::flatten(aim);
        const float aimLen = glm::length(rawAim);
        constexpr glm::vec3 defaultForward = simFrame::kForward;
        constexpr glm::vec3 aimFallbackAxis = simFrame::withHeight(glm::vec3(0.f), -1.f);
        const glm::vec3 aimDirection = (aimLen > 0.0001f) ? (rawAim / aimLen) : defaultForward;
        const float aimDot = glm::clamp(glm::dot(aimDirection, defaultForward), -1.f, 1.f);
        const float aimAngle = glm::acos(aimDot);
        const bool aimEqualsForward = glm::abs(glm::abs(aimDot) - 1.f) < 0.0001f;
        const glm::vec3 aimRotationAxis =
            aimEqualsForward ? aimFallbackAxis : glm::normalize(glm::cross(defaultForward, aimDirection));
        glm::mat4 transform = glm::rotate(glm::mat4(1.f), aimAngle, aimRotationAxis);
        transform[3] = glm::vec4(rootTranslation, 1.f);
        return transform;
    }

    bool sameBits(const glm::mat4& a, const glm::mat4& b)
    {
        for (glm::length_t column = 0; column < 4; ++column)
        {
            const glm::vec4& ca = a[column];
            const glm::vec4& cb = b[column];
            if (!(simFrameDetail::sameBits(ca.x, cb.x) && simFrameDetail::sameBits(ca.y, cb.y)
                    && simFrameDetail::sameBits(ca.z, cb.z) && simFrameDetail::sameBits(ca.w, cb.w)))
            {
                return false;
            }
        }
        return true;
    }

    float maxAbsDifference(const glm::vec3& a, const glm::vec3& b)
    {
        return std::max({std::abs(a.x - b.x), std::abs(a.y - b.y), std::abs(a.z - b.z)});
    }

    float maxAbsDifference(const glm::mat3& a, const glm::mat3& b)
    {
        float worst = 0.f;
        for (glm::length_t column = 0; column < 3; ++column)
        {
            worst = std::max(worst, maxAbsDifference(a[column], b[column]));
        }
        return worst;
    }

    glm::vec3 projectOntoPlane(const glm::vec3& v, const glm::vec3& n)
    {
        return glm::normalize(v - glm::dot(v, n) * n);
    }

    float angleBetween(const glm::vec3& a, const glm::vec3& b)
    {
        return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
    }

    glm::vec3 localHeading(float angleAboutUp)
    {
        return std::cos(angleAboutUp) * simFrame::kForward - std::sin(angleAboutUp) * simFrame::kRight;
    }

    std::vector<glm::vec3> sweepAims()
    {
        std::vector<glm::vec3> aims;
        for (int degree = 0; degree < 360; ++degree)
        {
            const float heading = glm::radians(static_cast<float>(degree));
            const float height = -0.35f + 0.25f * std::sin(3.f * heading);
            aims.push_back(glm::normalize(std::cos(heading) * simFrame::kForward + std::sin(heading) * simFrame::kLeft
                + height * simFrame::kUp));
        }
        return aims;
    }

    std::vector<glm::vec3> flatAims()
    {
        std::vector<glm::vec3> aims = sweepAims();
        aims.push_back(simFrame::kForward);
        aims.push_back(-simFrame::kForward);
        aims.push_back(simFrame::kRight);
        aims.push_back(simFrame::kLeft);
        aims.push_back(glm::vec3(0.f));
        aims.push_back(simFrame::kUp);
        aims.push_back(glm::normalize(localHeading(0.005f) - 0.4f * simFrame::kUp));
        aims.push_back(glm::normalize(localHeading(-0.005f) - 0.4f * simFrame::kUp));
        aims.push_back(glm::normalize(localHeading(glm::pi<float>() - 0.005f) - 0.4f * simFrame::kUp));
        aims.push_back(glm::normalize(localHeading(-(glm::pi<float>() - 0.005f)) - 0.4f * simFrame::kUp));
        return aims;
    }
} // namespace guardGroundFrameTests

TEST_CASE("DAttackGuardGroundFrame.GuardPoseLiesInTheGroundPlane", "[DAttack]")
{
    using namespace guardGroundFrameTests;
    const glm::vec3 n = kThirtyDegreeNormal;

    float worstUp = 0.f;
    float worstForwardOffPlane = 0.f;
    float worstHeading = 0.f;
    int worstHeadingAim = -1;
    int finite = 0;
    const std::vector<glm::vec3> aims = sweepAims();
    for (std::size_t i = 0; i < aims.size(); ++i)
    {
        const GuardTick r = tick(aims[i], slopeSample());
        const glm::vec3 up = surfaceFrame::surfaceUpOf(r.transform);
        const glm::vec3 forward = simFrame::forwardColumn(r.transform);
        finite += (std::isfinite(forward.x) && std::isfinite(forward.y) && std::isfinite(forward.z)) ? 1 : 0;
        worstUp = std::max(worstUp, maxAbsDifference(up, n));
        worstForwardOffPlane = std::max(worstForwardOffPlane, std::abs(glm::dot(forward, n)));
        const float heading = angleBetween(forward, projectOntoPlane(aims[i], n));
        if (!(heading <= worstHeading))
        {
            worstHeading = heading;
            worstHeadingAim = static_cast<int>(i);
        }
    }

    INFO("worst |surfaceUpOf - n| = " << worstUp << ", worst |forwardColumn . n| = " << worstForwardOffPlane
        << ", worst heading error = " << worstHeading << " rad at aim " << worstHeadingAim);
    CHECK(finite == 360);
    CHECK(worstUp <= 1.e-6f);
    CHECK(worstForwardOffPlane <= 1.e-6f);
    CHECK(worstHeading <= 1.e-5f);
}

TEST_CASE("DAttackGuardGroundFrame.SlopePoseKeepsTheTranslationAndTheTransient", "[DAttack]")
{
    using namespace guardGroundFrameTests;
    const glm::vec3 rootTranslation = kParentPos + kAttachOffset;

    float worstRotation = 0.f;
    int translationMismatches = 0;
    int positionMismatches = 0;
    const std::vector<glm::vec3> aims = sweepAims();
    for (const glm::vec3& aim : aims)
    {
        const GuardTick r = tick(aim, slopeSample());
        const glm::vec4 column3 = r.transform[3];
        translationMismatches += simFrameDetail::sameBits(glm::vec3(column3), rootTranslation) && column3.w == 1.f ? 0 : 1;
        positionMismatches += simFrameDetail::sameBits(r.state.bodyState.position, rootTranslation) ? 0 : 1;
        worstRotation = std::max(worstRotation,
            maxAbsDifference(glm::mat3_cast(r.state.bodyState.rotation), glm::mat3(r.transform)));
        CHECK(r.transform[0].w == 0.f);
        CHECK(r.transform[1].w == 0.f);
        CHECK(r.transform[2].w == 0.f);
    }

    INFO("worst |mat3_cast(bodyState.rotation) - transform| = " << worstRotation);
    CHECK(translationMismatches == 0);
    CHECK(positionMismatches == 0);
    CHECK(worstRotation <= 1.e-6f);
}

TEST_CASE("DAttackGuardGroundFrame.FlatSampleReproducesTodaysGuardPose", "[DAttack]")
{
    using namespace guardGroundFrameTests;

    int transformMismatches = 0;
    int rotationMismatches = 0;
    const std::vector<glm::vec3> aims = flatAims();
    for (const glm::vec3& aim : aims)
    {
        const GuardTick r = tick(aim, brawlerGround::GroundSample{});
        const glm::mat4 today = todaysGuardTransform(aim);
        if (!sameBits(r.transform, today))
        {
            ++transformMismatches;
            UNSCOPED_INFO("aim (" << aim.x << ", " << aim.y << ", " << aim.z << "): forward column ("
                << r.transform[2].x << ", " << r.transform[2].y << ", " << r.transform[2].z << ") vs ("
                << today[2].x << ", " << today[2].y << ", " << today[2].z << ")");
        }
        const glm::quat todayRotation = glm::quat_cast(today);
        const glm::quat rotation = r.state.bodyState.rotation;
        if (!(simFrameDetail::sameBits(rotation.x, todayRotation.x) && simFrameDetail::sameBits(rotation.y, todayRotation.y)
                && simFrameDetail::sameBits(rotation.z, todayRotation.z) && simFrameDetail::sameBits(rotation.w, todayRotation.w)))
        {
            ++rotationMismatches;
        }
    }

    INFO(aims.size() << " aims");
    CHECK(transformMismatches == 0);
    CHECK(rotationMismatches == 0);
}

TEST_CASE("DAttackGuardGroundFrame.NearPoleLocalHeadingTakesTheBandBranch", "[DAttack]")
{
    using namespace guardGroundFrameTests;
    const glm::vec3 n = kThirtyDegreeNormal;
    const glm::mat3 tilt = surfaceFrame::tiltMatrix(n);
    constexpr glm::vec3 fallbackAxis = simFrame::withHeight(glm::vec3(0.f), -1.f);

    const float localAngles[] = {0.01f, -0.01f, glm::pi<float>() - 0.01f, -(glm::pi<float>() - 0.01f)};
    for (const float localAngle : localAngles)
    {
        const glm::vec3 aim = glm::normalize(tilt * localHeading(localAngle) + 0.3f * n);
        const glm::vec3 local = glm::normalize(simFrame::flatten(surfaceFrame::tiltFromSurface(n, aim)));
        const float localDot = glm::dot(local, simFrame::kForward);
        const float aimAngle = glm::acos(glm::clamp(localDot, -1.f, 1.f));
        const glm::mat3 expected = tilt * glm::mat3(glm::rotate(glm::mat4(1.f), aimAngle, fallbackAxis));
        const glm::mat3 crossAxisComposite = tilt
            * glm::mat3(glm::rotate(glm::mat4(1.f), aimAngle, glm::normalize(glm::cross(simFrame::kForward, local))));

        const GuardTick r = tick(aim, slopeSample());
        const glm::mat3 pose(r.transform);
        INFO("local angle " << localAngle << ": |local . kForward| - 1 = " << (std::abs(localDot) - 1.f)
            << ", |pose - G A(fallback)| = " << maxAbsDifference(pose, expected)
            << ", |pose - G A(cross axis)| = " << maxAbsDifference(pose, crossAxisComposite));
        REQUIRE(std::abs(std::abs(localDot) - 1.f) < 0.0001f);
        CHECK(maxAbsDifference(pose, expected) <= 1.e-6f);
        if (std::sin(localAngle) > 0.f)
        {
            CHECK(maxAbsDifference(pose, crossAxisComposite) > 1.e-3f);
        }
        CHECK(maxAbsDifference(surfaceFrame::surfaceUpOf(r.transform), n) <= 1.e-6f);
    }

    for (const glm::vec3& heading : {simFrame::kForward, -simFrame::kForward})
    {
        const GuardTick r = tick(glm::normalize(tilt * heading + 0.3f * n), slopeSample());
        const glm::vec3 forward = simFrame::forwardColumn(r.transform);
        INFO("exact local heading (" << heading.x << ", " << heading.y << ", " << heading.z << ")");
        CHECK(std::isfinite(forward.x));
        CHECK(std::isfinite(forward.y));
        CHECK(std::isfinite(forward.z));
        CHECK(maxAbsDifference(forward, tilt * heading) <= 1.e-5f);
        CHECK(maxAbsDifference(surfaceFrame::surfaceUpOf(r.transform), n) <= 1.e-6f);
    }
}

#endif

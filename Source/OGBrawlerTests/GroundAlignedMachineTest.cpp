// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cmath>
#include <vector>

#include "catch_amalgamated.hpp"

#include "glm/geometric.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"

#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SurfaceFrame.h"

#include "GroundAlignedMachineRig.h"

namespace groundAlignedMachineTests
{
    using namespace groundAlignedMachineRig;

    const glm::vec3 kThirtyDegreeNormal(0.f, std::cos(glm::radians(30.f)), std::sin(glm::radians(30.f)));

    brawlerGround::GroundSample slopeSample()
    {
        brawlerGround::GroundSample sample;
        sample.normal = kThirtyDegreeNormal;
        return sample;
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

    glm::vec3 projectOntoPlane(const glm::vec3& v, const glm::vec3& n)
    {
        return glm::normalize(v - glm::dot(v, n) * n);
    }

    float angleBetween(const glm::vec3& a, const glm::vec3& b)
    {
        return std::atan2(glm::length(glm::cross(a, b)), glm::dot(a, b));
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

    glm::vec3 localHeading(float angleAboutUp)
    {
        return std::cos(angleAboutUp) * simFrame::kForward - std::sin(angleAboutUp) * simFrame::kRight;
    }

    struct TodaysSwingIC
    {
        float angle;
        glm::vec3 axis;
    };

    TodaysSwingIC todaysSwingIC(const glm::vec3& aim)
    {
        const glm::vec3 aimDirection = glm::normalize(simFrame::flatten(aim));
        constexpr glm::vec3 defaultForward = simFrame::kForward;
        constexpr glm::vec3 aimFallbackAxis = simFrame::withHeight(glm::vec3(0.f), -1.f);
        const float aimDot = glm::dot(aimDirection, defaultForward);
        const float aimAngle = glm::acos(aimDot);
        const bool aimEqualsForward = glm::abs(glm::abs(aimDot) - 1.f) < 0.0001f;
        const glm::vec3 aimRotationAxis =
            aimEqualsForward ? aimFallbackAxis : glm::normalize(glm::cross(defaultForward, aimDirection));
        return TodaysSwingIC{aimAngle, aimRotationAxis};
    }

    struct TodaysLaunch
    {
        glm::vec3 spawnPos;
        glm::vec3 spawnDir;
    };

    TodaysLaunch todaysLaunch(const glm::vec3& aim, const glm::vec3& capsulePosition)
    {
        const simulatableBrawler::StaticData staticData;
        const brawlerProjectileSimulation::StaticData& projSD = staticData.m_projectileStaticData;
        const glm::vec3 parentPosition = glm::vec3(glm::translate(glm::mat4(1.f), capsulePosition)[3]);
        constexpr glm::vec3 aimFallback = simFrame::kForward;
        const glm::vec3 aimXYraw = simFrame::flatten(aim);
        const glm::vec3 aimXY = (glm::length(aimXYraw) < 0.0001f) ? aimFallback : glm::normalize(aimXYraw);
        return TodaysLaunch{
            parentPosition + aimXY * projSD.spawnForwardOffset + simFrame::withHeight(glm::vec3(0.f), projSD.spawnHeightOffset),
            aimXY};
    }

    std::vector<glm::vec3> flatAims()
    {
        std::vector<glm::vec3> aims = sweepAims();
        aims.push_back(simFrame::kForward);
        aims.push_back(-simFrame::kForward);
        aims.push_back(glm::normalize(localHeading(0.005f) - 0.4f * simFrame::kUp));
        aims.push_back(glm::normalize(localHeading(glm::pi<float>() - 0.005f) - 0.4f * simFrame::kUp));
        aims.push_back(simFrame::kRight);
        aims.push_back(simFrame::kLeft);
        return aims;
    }
} // namespace groundAlignedMachineTests

TEST_CASE("GroundAlignedMachine.SwingFrameLiesInTheGroundPlane", "[DAttack]")
{
    using namespace groundAlignedMachineTests;
    const glm::vec3 n = kThirtyDegreeNormal;

    float worstUp = 0.f;
    float worstForwardOffPlane = 0.f;
    float worstHeading = 0.f;
    int worstHeadingDegree = -1;
    int attacking = 0;
    int finite = 0;
    const std::vector<glm::vec3> aims = sweepAims();
    for (std::size_t i = 0; i < aims.size(); ++i)
    {
        const MachineTickResult r = swingStart(aims[i], slopeSample());
        attacking += r.machine.m_currentState == DAttackState::Attacking ? 1 : 0;
        finite += std::isfinite(r.radialIC.initialAimAngle) ? 1 : 0;
        const glm::mat3 frame = swingFrame(r.radialIC);
        const glm::vec3 up = frame * simFrame::kUp;
        const glm::vec3 forward = frame * simFrame::kForward;
        worstUp = std::max(worstUp, maxAbsDifference(up, n));
        worstForwardOffPlane = std::max(worstForwardOffPlane, std::abs(glm::dot(forward, n)));
        const float heading = angleBetween(forward, projectOntoPlane(aims[i], n));
        if (!(heading <= worstHeading))
        {
            worstHeading = heading;
            worstHeadingDegree = static_cast<int>(i);
        }
    }

    INFO("worst |R kUp - n| = " << worstUp << ", worst |R kForward . n| = " << worstForwardOffPlane
        << ", worst heading error = " << worstHeading << " rad at aim " << worstHeadingDegree);
    CHECK(attacking == 360);
    CHECK(finite == 360);
    CHECK(worstUp <= 1.e-6f);
    CHECK(worstForwardOffPlane <= 1.e-6f);
    CHECK(worstHeading <= 1.e-5f);
}

TEST_CASE("GroundAlignedMachine.NearPoleLocalHeadingComposesTheFallbackAxis", "[DAttack]")
{
    using namespace groundAlignedMachineTests;
    const glm::vec3 n = kThirtyDegreeNormal;
    const glm::mat3 tilt = surfaceFrame::tiltMatrix(n);
    constexpr glm::vec3 fallbackAxis = simFrame::withHeight(glm::vec3(0.f), -1.f);

    const float localAngles[] = {0.01f, -0.01f, glm::pi<float>() - 0.01f, -(glm::pi<float>() - 0.01f)};
    for (const float localAngle : localAngles)
    {
        const glm::vec3 aim = glm::normalize(tilt * localHeading(localAngle) + 0.3f * n);
        const glm::vec3 local = glm::normalize(simFrame::flatten(surfaceFrame::tiltFromSurface(n, aim)));
        const float localDot = glm::dot(local, simFrame::kForward);
        const float aimAngle = glm::acos(localDot);
        const glm::mat3 expected = tilt * glm::mat3(glm::rotate(glm::mat4(1.f), aimAngle, fallbackAxis));
        const glm::mat3 crossAxisComposite = tilt
            * glm::mat3(glm::rotate(glm::mat4(1.f), aimAngle, glm::normalize(glm::cross(simFrame::kForward, local))));

        const MachineTickResult r = swingStart(aim, slopeSample());
        const glm::mat3 frame = swingFrame(r.radialIC);
        INFO("local angle " << localAngle << ": |local . kForward| - 1 = " << (std::abs(localDot) - 1.f)
            << ", |frame - G A(fallback)| = " << maxAbsDifference(frame, expected)
            << ", |frame - G A(cross axis)| = " << maxAbsDifference(frame, crossAxisComposite));
        REQUIRE(std::abs(std::abs(localDot) - 1.f) < 0.0001f);
        CHECK(r.machine.m_currentState == DAttackState::Attacking);
        CHECK(maxAbsDifference(frame, expected) <= 1.e-6f);
        if (std::sin(localAngle) > 0.f)
        {
            CHECK(maxAbsDifference(frame, crossAxisComposite) > 1.e-3f);
        }
        CHECK(maxAbsDifference(frame * simFrame::kUp, n) <= 1.e-6f);
    }
}

TEST_CASE("GroundAlignedMachine.HadoukenLaunchesAlongTheGroundPlane", "[DAttack]")
{
    using namespace groundAlignedMachineTests;
    const glm::vec3 n = kThirtyDegreeNormal;
    const glm::vec3 capsule(0.25f, 1.f, -0.5f);
    const simulatableBrawler::StaticData staticData;
    const float spawnHeightOffset = staticData.m_projectileStaticData.spawnHeightOffset;

    float worstOffPlane = 0.f;
    float worstLength = 0.f;
    float worstClearance = 0.f;
    float worstHeading = 0.f;
    int pending = 0;
    const std::vector<glm::vec3> aims = sweepAims();
    for (const glm::vec3& aim : aims)
    {
        const MachineTickResult r = hadoukenLaunch(aim, capsule, slopeSample());
        pending += r.projectileIC.spawnRequestPending == 1u ? 1 : 0;
        const glm::vec3 dir = r.projectileIC.spawnDir;
        worstOffPlane = std::max(worstOffPlane, std::abs(glm::dot(dir, n)));
        worstLength = std::max(worstLength, std::abs(glm::length(dir) - 1.f));
        worstClearance = std::max(worstClearance, std::abs(glm::dot(r.projectileIC.spawnPos - capsule, n) - spawnHeightOffset));
        const float heading = angleBetween(dir, projectOntoPlane(aim, n));
        worstHeading = heading <= worstHeading ? worstHeading : heading;
    }

    INFO("worst |spawnDir . n| = " << worstOffPlane << ", worst ||spawnDir| - 1| = " << worstLength
        << ", worst |(spawnPos - capsule) . n - h| = " << worstClearance << ", worst heading error = " << worstHeading);
    CHECK(pending == 360);
    CHECK(worstOffPlane <= 1.e-6f);
    CHECK(worstLength <= 1.e-6f);
    CHECK(worstClearance <= 1.e-6f);
    CHECK(worstHeading <= 1.e-5f);
}

TEST_CASE("GroundAlignedMachine.FlatSampleReproducesTodaysInitialConditions", "[DAttack]")
{
    using namespace groundAlignedMachineTests;
    const glm::vec3 capsule(0.25f, 1.f, -0.5f);

    int swingMismatches = 0;
    int launchMismatches = 0;
    const std::vector<glm::vec3> aims = flatAims();
    for (const glm::vec3& aim : aims)
    {
        const MachineTickResult swing = swingStart(aim, brawlerGround::GroundSample{});
        const TodaysSwingIC today = todaysSwingIC(aim);
        if (!(simFrameDetail::sameBits(swing.radialIC.initialAimAngle, today.angle)
                && simFrameDetail::sameBits(swing.radialIC.initialAimRotationAxis, today.axis)))
        {
            ++swingMismatches;
            UNSCOPED_INFO("swing aim (" << aim.x << ", " << aim.y << ", " << aim.z << "): angle " << swing.radialIC.initialAimAngle
                << " vs " << today.angle << ", axis (" << swing.radialIC.initialAimRotationAxis.x << ", "
                << swing.radialIC.initialAimRotationAxis.y << ", " << swing.radialIC.initialAimRotationAxis.z << ") vs ("
                << today.axis.x << ", " << today.axis.y << ", " << today.axis.z << ")");
        }

        const MachineTickResult launch = hadoukenLaunch(aim, capsule, brawlerGround::GroundSample{});
        const TodaysLaunch todayLaunch = todaysLaunch(aim, capsule);
        if (!(simFrameDetail::sameBits(launch.projectileIC.spawnPos, todayLaunch.spawnPos)
                && simFrameDetail::sameBits(launch.projectileIC.spawnDir, todayLaunch.spawnDir)))
        {
            ++launchMismatches;
            UNSCOPED_INFO("launch aim (" << aim.x << ", " << aim.y << ", " << aim.z << ") differs");
        }
    }

    INFO(aims.size() << " aims");
    CHECK(swingMismatches == 0);
    CHECK(launchMismatches == 0);
}

#endif

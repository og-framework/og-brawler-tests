// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackCameraIsoSeedTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"
#include "OGSimulation/DPID.h"

#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/trigonometric.hpp"

#include <cmath>
#include <vector>

#if __has_include("Math/Rotator.h")
#include "CoreMinimal.h"
#define OG_ISO_SEED_TEST_HAS_UNREAL_MATH 1
#else
#define OG_ISO_SEED_TEST_HAS_UNREAL_MATH 0
#endif

namespace dAttackCameraIsoSeedTest
{
constexpr float kDt = 1.f / 60.f;
constexpr float kIsoPitchDegrees = -60.f;
constexpr float kLegacyTargetPitch = 0.8f;
constexpr float kFullBoomLength = 900.f;
constexpr int kTicks = 600;

DPIDSettings productionPitchPidSettings()
{
    return DPIDSettings(0.03f, 0.01f, 0.01f);
}

float boomPitch(const DAttackCameraState& state)
{
    return glm::eulerAngles(glm::quat_cast(state.getCameraBoomTransform())).y;
}

float boomYaw(const DAttackCameraState& state)
{
    return glm::eulerAngles(glm::quat_cast(state.getCameraBoomTransform())).z;
}

DAttackCameraState seededState(float pitch, float yaw)
{
    DAttackCameraState state;
    state.setCameraBoomTransform(glm::mat4_cast(glm::quat(glm::vec3(0.f, pitch, yaw))));
    state.setCameraBoomLength(kFullBoomLength);
    return state;
}

struct LookRun
{
    std::vector<float> pitches;
    std::vector<float> lengths;
};

LookRun runHorizontalLook(DAttackCameraState& state, float targetPitch, int ticks)
{
    const DAttackCameraInput input(glm::vec3(0.f), glm::vec2(1.f, 0.f), /*blockLook*/ true,
                                   targetPitch, productionPitchPidSettings());
    LookRun run;
    for (int tick = 0; tick < ticks; ++tick)
    {
        dAttackCameraBehaviour::integrate(kDt, input, state);
        run.pitches.push_back(boomPitch(state));
        run.lengths.push_back(state.getCameraBoomLength());
    }
    return run;
}

void requireHoldsTargetAtFullLength(DAttackCameraState& state, float targetPitch)
{
    const float yawBefore = boomYaw(state);
    const LookRun run = runHorizontalLook(state, targetPitch, kTicks);

    float worstPitchError = 0.f;
    float shortestLength = kFullBoomLength;
    for (int tick = 0; tick < kTicks; ++tick)
    {
        worstPitchError = std::fmax(worstPitchError, std::fabs(run.pitches[tick] - targetPitch));
        shortestLength = std::fmin(shortestLength, run.lengths[tick]);
    }
    INFO("target " << targetPitch << " worst pitch error " << worstPitchError
                   << " shortest length " << shortestLength);
    REQUIRE(worstPitchError < 1e-3f);
    REQUIRE(shortestLength == Catch::Approx(kFullBoomLength).margin(0.5f));
    REQUIRE(std::fabs(boomYaw(state) - yawBefore) > 0.01f);
}
} // namespace dAttackCameraIsoSeedTest

using namespace dAttackCameraIsoSeedTest;

TEST_CASE("DAttackCamera.IsoTargetPitchStaysInsideTheOpenQuarterTurn", "[DAttack][CameraIsoSeed]")
{
    const float target = dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(kIsoPitchDegrees);
    INFO("iso pitch -60 deg -> target " << target);
    REQUIRE(target == Catch::Approx(glm::pi<float>() / 3.f).margin(1e-6f));
    REQUIRE(dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(-45.f)
            == Catch::Approx(glm::pi<float>() / 4.f).margin(1e-6f));

    const float inputs[] = { 30.f, 0.f, -0.5f, -1.f, -60.f, -89.f, -90.f, -120.f, -180.f, 400.f, -400.f };
    for (const float uePitch : inputs)
    {
        const float clamped = dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(uePitch);
        INFO("ue pitch " << uePitch << " -> target " << clamped);
        REQUIRE(clamped > 0.f);
        REQUIRE(clamped < glm::half_pi<float>());
    }
    REQUIRE(dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(30.f)
            == Catch::Approx(glm::radians(dAttackCameraBehaviour::kMinTargetPitchDegrees)));
    REQUIRE(dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(-90.f)
            == Catch::Approx(glm::radians(dAttackCameraBehaviour::kMaxTargetPitchDegrees)));
}

TEST_CASE("DAttackCamera.IsoSeedHoldsPitchAndFullLengthUnderHorizontalLook", "[DAttack][CameraIsoSeed]")
{
    const float target = dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(kIsoPitchDegrees);
    const float yaws[] = { 0.f, glm::quarter_pi<float>(), -2.4f, 3.1f };
    for (const float yaw : yaws)
    {
        INFO("seed yaw " << yaw);
        DAttackCameraState state = seededState(target, yaw);
        REQUIRE(boomPitch(state) == Catch::Approx(target).margin(1e-5f));
        requireHoldsTargetAtFullLength(state, target);
    }
}

TEST_CASE("DAttackCamera.OffTargetSeedConvergesToTheIsoPitchNotTheLegacyTarget", "[DAttack][CameraIsoSeed]")
{
    const float target = dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(kIsoPitchDegrees);
    const float startPitches[] = { 0.3f, 1.3f };
    for (const float start : startPitches)
    {
        DAttackCameraState state = seededState(start, 0.f);
        const LookRun run = runHorizontalLook(state, target, kTicks);
        const float finalPitch = run.pitches.back();
        INFO("start " << start << " pitch after 1 s " << run.pitches[59] << " after 5 s " << run.pitches[299]
                      << " final " << finalPitch << " target " << target);
        REQUIRE(finalPitch == Catch::Approx(target).margin(0.01f));
        REQUIRE(std::fabs(finalPitch - kLegacyTargetPitch) > 0.2f);
        REQUIRE(run.lengths.back() == Catch::Approx(kFullBoomLength).margin(5.f));
    }
}

#if OG_ISO_SEED_TEST_HAS_UNREAL_MATH
TEST_CASE("DAttackCamera.UnrealIsoRotatorSeedReadsAsTheTargetPitch", "[DAttack][CameraIsoSeed]")
{
    const float target = dAttackCameraBehaviour::targetPitchFromUEPitchDegrees(kIsoPitchDegrees);
    const double actorYaws[] = { 0.0, 90.0, -37.0, 180.0 };
    const double isoYaws[] = { 0.0, 45.0 };
    for (const double actorYaw : actorYaws)
    {
        for (const double isoYaw : isoYaws)
        {
            const FQuat relative = FRotator(0.0, actorYaw, 0.0).Quaternion().Inverse()
                                 * FRotator(kIsoPitchDegrees, isoYaw, 0.0).Quaternion();
            const glm::quat rawCopy(static_cast<float>(relative.W), static_cast<float>(relative.X),
                                    static_cast<float>(relative.Y), static_cast<float>(relative.Z));
            DAttackCameraState state;
            state.setCameraBoomTransform(glm::mat4_cast(rawCopy));
            state.setCameraBoomLength(kFullBoomLength);

            const glm::vec3 euler = glm::eulerAngles(glm::quat_cast(state.getCameraBoomTransform()));
            INFO("actor yaw " << actorYaw << " iso yaw " << isoYaw << " euler (" << euler.x << ", "
                              << euler.y << ", " << euler.z << ") target " << target);
            REQUIRE(euler.y == Catch::Approx(target).margin(1e-4f));
            REQUIRE(euler.x == Catch::Approx(0.f).margin(1e-4f));
            requireHoldsTargetAtFullLength(state, target);
        }
    }
}
#endif

#endif // WITH_LOW_LEVEL_TESTS

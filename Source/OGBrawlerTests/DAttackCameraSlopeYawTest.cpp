// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackCameraSlopeYawTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"

#include "glm/geometric.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <limits>
#include <vector>

namespace dAttackCameraSlopeYawTest
{
using dAttackCameraBehaviour::OrbitCameraInput;
using dAttackCameraBehaviour::OrbitCameraSettings;
using dAttackCameraBehaviour::OrbitCameraState;
using dAttackCameraBehaviour::WorldView;

constexpr double kPi = 3.14159265358979323846;
constexpr float kDts[] = { 1.f / 30.f, 1.f / 60.f, 1.f / 144.f, 1.f / 240.f };
constexpr float kIsoTargetDeg = 60.f;
constexpr float kStartPitchDeg = 40.f;
constexpr float kStartOffsetsDeg[] = { 10.1f, -47.3f, 100.7f, -179.1f, 133.3f, -20.2f, 64.9f, -91.9f };
constexpr int kHoldFrames = 600;
constexpr int kMaxFrames = 2000;

std::uint32_t bitsOf(float value)
{
    std::uint32_t bits = 0u;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

glm::vec3 upTiltedToward(double degrees, double awayFromYawDeg)
{
    const double radians = degrees * kPi / 180.0;
    const double away = awayFromYawDeg * kPi / 180.0;
    return glm::vec3(static_cast<float>(-std::sin(radians) * std::cos(away)),
        static_cast<float>(-std::sin(radians) * std::sin(away)), static_cast<float>(std::cos(radians)));
}

double signedYawGapDeg(double to, double from)
{
    return std::remainder(to - from, 360.0);
}

double upSlopeYawDegInDouble(const glm::vec3& up)
{
    return std::atan2(-static_cast<double>(up.y), -static_cast<double>(up.x)) * 180.0 / kPi;
}

OrbitCameraState stateAt(float yawDeg, float pitchDeg, const glm::vec3& frameUp)
{
    OrbitCameraState state;
    state.yawDeg = yawDeg;
    state.pitchDeg = pitchDeg;
    state.frameUp = frameUp;
    return state;
}

OrbitCameraSettings twentyOneDSettings()
{
    OrbitCameraSettings settings;
    settings.slopeYawEaseSeconds = 0.f;
    settings.slopeYawAccelDegPerSec2 = 0.f;
    return settings;
}

OrbitCameraSettings twentyOneCSettings()
{
    OrbitCameraSettings settings = twentyOneDSettings();
    settings.slopeYawDwellSeconds = 0.f;
    return settings;
}

OrbitCameraInput lookOff(const glm::vec3& targetUp)
{
    OrbitCameraInput input;
    input.lookHeld = false;
    input.mouseUnits = glm::vec2(40.f, -30.f);
    input.stick = glm::vec2(1.f, 1.f);
    input.targetPitchDeg = kIsoTargetDeg;
    input.targetUp = targetUp;
    return input;
}

OrbitCameraState run(float dt, int frames, const OrbitCameraInput& input, const OrbitCameraSettings& settings,
    OrbitCameraState state)
{
    for (int frame = 0; frame < frames; ++frame)
        state = dAttackCameraBehaviour::integrate(dt, input, settings, state);
    return state;
}

int framesByTheFormula(double gapDeg, float degPerSec, float dt)
{
    return static_cast<int>(std::ceil(std::fabs(gapDeg) / (static_cast<double>(degPerSec) * static_cast<double>(dt))));
}

struct Arrival
{
    int frame;
    int wrongWaySteps;
    double worstStepOverTheRateDeg;
    OrbitCameraState state;
};

Arrival turnUntilArrived(float dt, const OrbitCameraInput& input, const OrbitCameraSettings& settings,
    OrbitCameraState state, float targetYawDeg, double startGapDeg)
{
    Arrival arrival{ -1, 0, 0.0, state };
    const double maxStepDeg = static_cast<double>(settings.slopeYawDegPerSec) * dt;
    for (int frame = 1; frame <= kMaxFrames; ++frame)
    {
        const OrbitCameraState next = dAttackCameraBehaviour::integrate(dt, input, settings, state);
        const double stepDeg = signedYawGapDeg(next.yawDeg, state.yawDeg);
        if (stepDeg != 0.0 && (stepDeg > 0.0) != (startGapDeg > 0.0))
            ++arrival.wrongWaySteps;
        arrival.worstStepOverTheRateDeg = std::max(arrival.worstStepOverTheRateDeg, std::fabs(stepDeg) - maxStepDeg);
        state = next;
        if (bitsOf(state.yawDeg) == bitsOf(targetYawDeg))
        {
            arrival.frame = frame;
            break;
        }
    }
    arrival.state = state;
    return arrival;
}

struct Face
{
    const char* name;
    glm::vec3 up;
};

const Face kThirtyDegreeFaces[] = {
    { "30 deg rising along Unreal +X", upTiltedToward(30.0, 0.0) },
    { "30 deg rising along Unreal +Y", upTiltedToward(30.0, 90.0) },
    { "30 deg rising along Unreal -X (the up-slope heading on the yaw seam)", upTiltedToward(30.0, 180.0) },
    { "G3 capture face normal (0, 0.866025448, 0.49999997)", glm::vec3(-0.49999997f, 0.f, 0.866025448f) },
    { "G3 capture face normal (-1.49011612e-08, 0.866025388, 0.5)", glm::vec3(-0.5f, -1.49011612e-08f, 0.866025388f) },
};
} // namespace dAttackCameraSlopeYawTest

using namespace dAttackCameraSlopeYawTest;

TEST_CASE("DAttackCamera.SlopeYawLookModeOffTurnsToFaceUpTheSlope", "[DAttack][CameraSlopeYaw]")
{
    const OrbitCameraSettings noDwell = twentyOneCSettings();
    REQUIRE(noDwell.slopeYawDegPerSec == 90.f);
    REQUIRE(noDwell.slopeYawDwellSeconds == 0.f);
    REQUIRE(noDwell.slopeYawEaseSeconds == 0.f);
    REQUIRE(noDwell.slopeYawAccelDegPerSec2 == 0.f);
    REQUIRE(dAttackCameraBehaviour::kSlopeYawMinSlopeDeg == 15.f);

    for (const Face& face : kThirtyDegreeFaces)
    {
        const glm::vec3 targetUp = dAttackCameraBehaviour::cameraTargetUp(face.up, true);
        REQUIRE(!(targetUp == dAttackCameraBehaviour::kCameraUp));
        const float targetYawDeg = dAttackCameraBehaviour::wrapYawDeg(dAttackCameraBehaviour::upSlopeYawDeg(targetUp));
        const double expectedYawDeg = upSlopeYawDegInDouble(targetUp);
        INFO(face.name << ": up-slope yaw " << targetYawDeg << " (double " << expectedYawDeg << ")");
        CHECK(std::fabs(signedYawGapDeg(targetYawDeg, expectedYawDeg)) < 1e-4);

        for (const float dt : kDts)
        {
            for (const float offsetDeg : kStartOffsetsDeg)
            {
                const float startYawDeg = dAttackCameraBehaviour::wrapYawDeg(targetYawDeg + offsetDeg);
                const double startGapDeg = signedYawGapDeg(targetYawDeg, startYawDeg);
                const OrbitCameraState start = stateAt(startYawDeg, kStartPitchDeg, targetUp);
                const OrbitCameraInput off = lookOff(targetUp);
                const Arrival arrival = turnUntilArrived(dt, off, noDwell, start, targetYawDeg, startGapDeg);
                const int expectedFrames = framesByTheFormula(startGapDeg, noDwell.slopeYawDegPerSec, dt);
                INFO("fps " << 1.f / dt << " start yaw " << startYawDeg << " (gap " << startGapDeg << " deg): arrived after "
                    << arrival.frame << " frames, " << expectedFrames << " by the formula, wrong-way steps "
                    << arrival.wrongWaySteps << ", worst step over the rate " << arrival.worstStepOverTheRateDeg << " deg");
                CHECK(arrival.frame == expectedFrames);
                CHECK(arrival.wrongWaySteps == 0);
                CHECK(arrival.worstStepOverTheRateDeg < 1e-4);

                OrbitCameraState held = arrival.state;
                int yawChanges = 0;
                for (int frame = 0; frame < kHoldFrames; ++frame)
                {
                    held = dAttackCameraBehaviour::integrate(dt, off, noDwell, held);
                    if (bitsOf(held.yawDeg) != bitsOf(targetYawDeg))
                        ++yawChanges;
                }
                CHECK(yawChanges == 0);
                CHECK(bitsOf(held.pitchDeg) == bitsOf(kIsoTargetDeg));
                CHECK(bitsOf(held.frameUp.z) == bitsOf(1.f));

                const WorldView view = dAttackCameraBehaviour::worldView(held, noDwell);
                CHECK(bitsOf(view.yawDeg) == bitsOf(targetYawDeg));
                CHECK(bitsOf(view.pitchDeg) == bitsOf(kIsoTargetDeg));
                const double yawRad = static_cast<double>(view.yawDeg) * kPi / 180.0;
                const double pitchRad = static_cast<double>(view.pitchDeg) * kPi / 180.0;
                const double viewX = std::cos(pitchRad) * std::cos(yawRad);
                const double viewY = std::cos(pitchRad) * std::sin(yawRad);
                const double viewZ = -std::sin(pitchRad);
                const double riseDeg = std::atan(-(targetUp.x * std::cos(yawRad) + targetUp.y * std::sin(yawRad))
                                           / targetUp.z) * 180.0 / kPi;
                const double gapToMinusNormal = std::max({ std::fabs(viewX + targetUp.x), std::fabs(viewY + targetUp.y),
                    std::fabs(viewZ + targetUp.z) });
                INFO("view (" << viewX << ", " << viewY << ", " << viewZ << "), rise along the view " << riseDeg
                    << " deg, largest component gap to -n " << gapToMinusNormal);
                CHECK(riseDeg == Catch::Approx(30.0).margin(1e-3));
                CHECK(gapToMinusNormal <= 1e-4);
            }
        }
    }
}

TEST_CASE("DAttackCamera.SlopeYawTurnsFromFifteenDegreesUpToTheFollowedCutOff", "[DAttack][CameraSlopeYaw]")
{
    const OrbitCameraSettings noDwell = twentyOneCSettings();
    constexpr float kStartYawDeg = 33.f;
    constexpr float dt = 1.f / 60.f;

    struct Ground
    {
        const char* name;
        glm::vec3 up;
        bool supported;
        bool turns;
    };
    const Ground grounds[] = {
        { "15 deg face", upTiltedToward(15.0, 0.0), true, true },
        { "14.9995 deg face (inside the threshold's tolerance)", upTiltedToward(14.9995, 0.0), true, true },
        { "20 deg face", upTiltedToward(20.0, 0.0), true, true },
        { "30.09 deg face", upTiltedToward(30.09, 0.0), true, true },
        { "14.998 deg face (outside the threshold's tolerance)", upTiltedToward(14.998, 0.0), true, false },
        { "14.9 deg face", upTiltedToward(14.9, 0.0), true, false },
        { "12 deg face", upTiltedToward(12.0, 0.0), true, false },
        { "flat ground", dAttackCameraBehaviour::kCameraUp, true, false },
        { "flat ground, 1-ULP-short up", glm::vec3(0.f, 0.f, 0.99999994f), true, false },
        { "30 deg face, unsupported", upTiltedToward(30.0, 0.0), false, false },
        { "30.11 deg face", upTiltedToward(30.11, 0.0), true, false },
        { "45 deg face", upTiltedToward(45.0, 0.0), true, false },
        { "60 deg face", upTiltedToward(60.0, 0.0), true, false },
    };
    for (const Ground& ground : grounds)
    {
        const glm::vec3 targetUp = dAttackCameraBehaviour::cameraTargetUp(ground.up, ground.supported);
        INFO(ground.name << ": target up slope " << std::setprecision(9)
                         << std::acos(targetUp.z) * dAttackCameraBehaviour::kSightRadiansToDegrees << " deg");
        const OrbitCameraInput off = lookOff(targetUp);
        const OrbitCameraState start = stateAt(kStartYawDeg, kStartPitchDeg, targetUp);
        if (ground.turns)
        {
            const OrbitCameraState once = dAttackCameraBehaviour::integrate(dt, off, noDwell, start);
            CHECK(signedYawGapDeg(once.yawDeg, kStartYawDeg) == Catch::Approx(-1.5).margin(1e-4));
            const OrbitCameraState settled = run(dt, kHoldFrames, off, noDwell, start);
            CHECK(bitsOf(settled.yawDeg) == bitsOf(dAttackCameraBehaviour::wrapYawDeg(
                                               dAttackCameraBehaviour::upSlopeYawDeg(targetUp))));
            CHECK(std::fabs(settled.yawDeg) < 1e-4f);
            continue;
        }
        OrbitCameraState state = start;
        int yawChanges = 0;
        for (int frame = 0; frame < kHoldFrames; ++frame)
        {
            state = dAttackCameraBehaviour::integrate(dt, off, noDwell, state);
            if (bitsOf(state.yawDeg) != bitsOf(kStartYawDeg))
                ++yawChanges;
        }
        CHECK(yawChanges == 0);
        CHECK(bitsOf(state.pitchDeg) == bitsOf(kIsoTargetDeg));
    }
}

TEST_CASE("DAttackCamera.SlopeYawHeadingStaysWhenTheSlopeIsLeft", "[DAttack][CameraSlopeYaw]")
{
    const OrbitCameraSettings noDwell = twentyOneCSettings();
    const glm::vec3 slopeUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 0.0), true);
    const glm::vec3 leftFor[] = {
        dAttackCameraBehaviour::kCameraUp,
        dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 90.0), false),
        dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(12.0, 90.0), true),
    };
    for (const float dt : kDts)
    {
        const OrbitCameraState onSlope = run(dt, kHoldFrames, lookOff(slopeUp), noDwell, stateAt(-120.f, 25.f, slopeUp));
        const float arrivedYawDeg = onSlope.yawDeg;
        REQUIRE(bitsOf(arrivedYawDeg) == bitsOf(dAttackCameraBehaviour::wrapYawDeg(
                                             dAttackCameraBehaviour::upSlopeYawDeg(slopeUp))));
        for (const glm::vec3& up : leftFor)
        {
            INFO("fps " << 1.f / dt << " target up (" << up.x << ", " << up.y << ", " << up.z << ")");
            const OrbitCameraInput off = lookOff(up);
            OrbitCameraState state = onSlope;
            int yawChanges = 0;
            for (int frame = 0; frame < kHoldFrames; ++frame)
            {
                state = dAttackCameraBehaviour::integrate(dt, off, noDwell, state);
                if (bitsOf(state.yawDeg) != bitsOf(arrivedYawDeg))
                    ++yawChanges;
            }
            CHECK(yawChanges == 0);
            CHECK(bitsOf(state.pitchDeg) == bitsOf(kIsoTargetDeg));
        }
    }
}

TEST_CASE("DAttackCamera.SlopeYawLookModeOnTurnsOnlyWithThePlayer", "[DAttack][CameraSlopeYaw]")
{
    const OrbitCameraSettings noDwell = twentyOneCSettings();
    const glm::vec3 slopeUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 0.0), true);
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        OrbitCameraInput still;
        still.lookHeld = true;
        still.targetPitchDeg = kIsoTargetDeg;
        still.targetUp = slopeUp;
        const OrbitCameraState start = stateAt(33.f, 45.f, slopeUp);
        const OrbitCameraState held = run(dt, kHoldFrames, still, noDwell, start);
        CHECK(bitsOf(held.yawDeg) == bitsOf(start.yawDeg));

        OrbitCameraInput turning = still;
        turning.mouseUnits = glm::vec2(0.2f, 0.f);
        OrbitCameraInput turningOnFlat = turning;
        turningOnFlat.targetUp = dAttackCameraBehaviour::kCameraUp;
        OrbitCameraState onSlope = start;
        OrbitCameraState onFlat = start;
        int yawMismatches = 0;
        for (int frame = 0; frame < kHoldFrames; ++frame)
        {
            onSlope = dAttackCameraBehaviour::integrate(dt, turning, noDwell, onSlope);
            onFlat = dAttackCameraBehaviour::integrate(dt, turningOnFlat, noDwell, onFlat);
            if (bitsOf(onSlope.yawDeg) != bitsOf(onFlat.yawDeg))
                ++yawMismatches;
        }
        CHECK(yawMismatches == 0);
        CHECK(bitsOf(onSlope.yawDeg) != bitsOf(start.yawDeg));
    }
}

TEST_CASE("DAttackCamera.SlopeYawRateIsTheSettingAndZeroHolds", "[DAttack][CameraSlopeYaw]")
{
    const glm::vec3 slopeUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 0.0), true);
    const float targetYawDeg = dAttackCameraBehaviour::wrapYawDeg(dAttackCameraBehaviour::upSlopeYawDeg(slopeUp));
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        OrbitCameraSettings slower = twentyOneCSettings();
        slower.slopeYawDegPerSec = 45.f;
        const float startYawDeg = 100.7f;
        const double startGapDeg = signedYawGapDeg(targetYawDeg, startYawDeg);
        const Arrival arrival = turnUntilArrived(dt, lookOff(slopeUp), slower, stateAt(startYawDeg, kStartPitchDeg, slopeUp),
            targetYawDeg, startGapDeg);
        CHECK(arrival.frame == framesByTheFormula(startGapDeg, 45.f, dt));
        CHECK(arrival.wrongWaySteps == 0);

        for (const float held : { 0.f, -30.f })
        {
            OrbitCameraSettings holding = twentyOneCSettings();
            holding.slopeYawDegPerSec = held;
            const OrbitCameraState start = stateAt(startYawDeg, kStartPitchDeg, slopeUp);
            OrbitCameraState state = start;
            int yawChanges = 0;
            for (int frame = 0; frame < kHoldFrames; ++frame)
            {
                state = dAttackCameraBehaviour::integrate(dt, lookOff(slopeUp), holding, state);
                if (bitsOf(state.yawDeg) != bitsOf(startYawDeg))
                    ++yawChanges;
            }
            INFO("slopeYawDegPerSec " << held);
            CHECK(yawChanges == 0);
            CHECK(bitsOf(state.pitchDeg) == bitsOf(kIsoTargetDeg));
            CHECK(bitsOf(state.frameUp.z) == bitsOf(1.f));
        }
    }
}

namespace dAttackCameraSlopeYawTest
{
constexpr int kFps[] = { 30, 60, 144, 240 };

float dtAt(int fps)
{
    return 1.f / static_cast<float>(fps);
}

int dwellFrameAt(int fps)
{
    return (fps + 4) / 5;
}

std::vector<OrbitCameraState> runTargets(float dt, const std::vector<glm::vec3>& targets, const OrbitCameraSettings& settings,
    OrbitCameraState state)
{
    std::vector<OrbitCameraState> states;
    states.reserve(targets.size());
    for (const glm::vec3& targetUp : targets)
    {
        state = dAttackCameraBehaviour::integrate(dt, lookOff(targetUp), settings, state);
        states.push_back(state);
    }
    return states;
}

std::vector<glm::vec3> alternating(const glm::vec3& first, const glm::vec3& second, int runLength, int frames)
{
    std::vector<glm::vec3> targets;
    for (int frame = 0; frame < frames; ++frame)
        targets.push_back(((frame / runLength) % 2 == 0) ? first : second);
    return targets;
}

int yawChangesFrom(const std::vector<OrbitCameraState>& states, float yawDeg)
{
    int changes = 0;
    for (const OrbitCameraState& state : states)
        if (bitsOf(state.yawDeg) != bitsOf(yawDeg))
            ++changes;
    return changes;
}

OrbitCameraState twentyOneCLookOffStep(float dt, const glm::vec3& targetUp, const OrbitCameraSettings& settings,
    const OrbitCameraState& state)
{
    OrbitCameraState next = state;
    next.holdoffSeconds = std::max(0.f, state.holdoffSeconds - dt);
    const float returnStepDeg = std::max(0.f, settings.returnDegPerSec) * dt;
    next.frameUp = dAttackCameraBehaviour::rotateUpToward(state.frameUp, dAttackCameraBehaviour::kCameraUp, returnStepDeg);
    next.pitchDeg = dAttackCameraBehaviour::clampPitchDeg(dAttackCameraBehaviour::stepToward(state.pitchDeg,
        dAttackCameraBehaviour::clampPitchDeg(kIsoTargetDeg, settings), returnStepDeg), settings);
    next.yawDeg = dAttackCameraBehaviour::slopeYawStep(state.yawDeg, targetUp,
        std::max(0.f, settings.slopeYawDegPerSec) * dt);
    return next;
}

bool sameCameraBits(const OrbitCameraState& a, const OrbitCameraState& b)
{
    return bitsOf(a.yawDeg) == bitsOf(b.yawDeg) && bitsOf(a.pitchDeg) == bitsOf(b.pitchDeg)
        && bitsOf(a.holdoffSeconds) == bitsOf(b.holdoffSeconds) && bitsOf(a.frameUp.x) == bitsOf(b.frameUp.x)
        && bitsOf(a.frameUp.y) == bitsOf(b.frameUp.y) && bitsOf(a.frameUp.z) == bitsOf(b.frameUp.z);
}

const glm::vec3 kSlopeUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 0.0), true);
const glm::vec3 kEdgeUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 40.0), true);
} // namespace dAttackCameraSlopeYawTest

TEST_CASE("DAttackCamera.SlopeDwellFlickeringTargetNeverTurns", "[DAttack][CameraSlopeYaw][CameraSlopeDwell]")
{
    const OrbitCameraSettings twentyOneD = twentyOneDSettings();
    REQUIRE(twentyOneD.slopeYawDwellSeconds == 0.2f);
    REQUIRE(twentyOneD.slopeYawEaseSeconds == 0.f);
    REQUIRE(twentyOneD.slopeYawAccelDegPerSec2 == 0.f);
    REQUIRE(dAttackCameraBehaviour::kSlopeYawDwellToleranceDeg == 10.f);
    const glm::vec3 flatUp = dAttackCameraBehaviour::cameraTargetUp(dAttackCameraBehaviour::kCameraUp, true);
    const glm::vec3 unsupportedUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 0.0), false);
    REQUIRE(flatUp == dAttackCameraBehaviour::kCameraUp);
    REQUIRE(unsupportedUp == dAttackCameraBehaviour::kCameraUp);

    struct Flicker
    {
        const char* name;
        glm::vec3 first;
        glm::vec3 second;
        int runLength;
    };
    const Flicker flickers[] = {
        { "slope/flat every tick", kSlopeUp, flatUp, 1 },
        { "flat/slope every tick", flatUp, kSlopeUp, 1 },
        { "slope/flat in 2-tick runs", kSlopeUp, flatUp, 2 },
        { "flat/slope in 2-tick runs", flatUp, kSlopeUp, 2 },
        { "slope/airborne every tick", kSlopeUp, unsupportedUp, 1 },
    };
    OrbitCameraSettings noDwell = twentyOneD;
    noDwell.slopeYawDwellSeconds = 0.f;
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        for (const Flicker& flicker : flickers)
        {
            const OrbitCameraState start = stateAt(100.7f, kStartPitchDeg, dAttackCameraBehaviour::kCameraUp);
            const std::vector<glm::vec3> targets = alternating(flicker.first, flicker.second, flicker.runLength, fps);
            const std::vector<OrbitCameraState> gated = runTargets(dt, targets, twentyOneD, start);
            const std::vector<OrbitCameraState> ungated = runTargets(dt, targets, noDwell, start);
            int pitchOrFrameMismatches = 0;
            for (std::size_t frame = 0; frame < gated.size(); ++frame)
            {
                OrbitCameraState sameYaw = ungated[frame];
                sameYaw.yawDeg = gated[frame].yawDeg;
                if (!sameCameraBits(gated[frame], sameYaw))
                    ++pitchOrFrameMismatches;
            }
            const int yawChanges = yawChangesFrom(gated, start.yawDeg);
            const int ungatedYawChanges = yawChangesFrom(ungated, start.yawDeg);
            INFO("fps " << fps << ", " << flicker.name << " for 1 s: gated yaw changes " << yawChanges
                        << ", with the dwell at 0 " << ungatedYawChanges << ", pitch/frame-up mismatches "
                        << pitchOrFrameMismatches << ", last dwell " << gated.back().slopeDwellSeconds << " s");
            CHECK(yawChanges == 0);
            CHECK(ungatedYawChanges > 0);
            CHECK(pitchOrFrameMismatches == 0);
            CHECK(gated.back().slopeDwellSeconds <= 2.f * dt);
        }
    }
}

TEST_CASE("DAttackCamera.SlopeDwellEdgeFlickerHoldsUntilTheSlopeIsSteady", "[DAttack][CameraSlopeYaw][CameraSlopeDwell]")
{
    const OrbitCameraSettings twentyOneD = twentyOneDSettings();
    const float slopeYawDeg = dAttackCameraBehaviour::wrapYawDeg(dAttackCameraBehaviour::upSlopeYawDeg(kSlopeUp));
    const float edgeYawDeg = dAttackCameraBehaviour::wrapYawDeg(dAttackCameraBehaviour::upSlopeYawDeg(kEdgeUp));
    REQUIRE(std::fabs(signedYawGapDeg(edgeYawDeg, slopeYawDeg) - 40.0) < 1e-3);
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        const int dwellFrame = dwellFrameAt(fps);
        const OrbitCameraState start = stateAt(dAttackCameraBehaviour::wrapYawDeg(slopeYawDeg + 170.f), kStartPitchDeg,
            kSlopeUp);
        const std::vector<glm::vec3> steadyTargets(static_cast<std::size_t>(fps), kSlopeUp);
        const std::vector<OrbitCameraState> steady = runTargets(dt, steadyTargets, twentyOneD, start);
        const OrbitCameraState onSlope = steady.back();
        const double gapLeftDeg = signedYawGapDeg(slopeYawDeg, onSlope.yawDeg);
        INFO("fps " << fps << ": after 1 s on the slope the yaw is " << onSlope.yawDeg << ", " << gapLeftDeg
                    << " deg from the up-slope heading");
        REQUIRE(bitsOf(onSlope.yawDeg) != bitsOf(steady[steady.size() - 2].yawDeg));
        REQUIRE(std::fabs(gapLeftDeg) > 10.0);

        const std::vector<OrbitCameraState> flicker = runTargets(dt, alternating(kEdgeUp, kSlopeUp, 1, fps + 1), twentyOneD,
            onSlope);
        const int flickerYawChanges = yawChangesFrom(flicker, onSlope.yawDeg);

        const std::vector<OrbitCameraState> resumed = runTargets(dt, steadyTargets, twentyOneD, flicker.back());
        const std::vector<OrbitCameraState> beforeTheDwell(resumed.begin(), resumed.begin() + (dwellFrame - 1));
        const int waitingYawChanges = yawChangesFrom(beforeTheDwell, onSlope.yawDeg);
        const float firstStepDeg = dAttackCameraBehaviour::slopeYawStep(onSlope.yawDeg, kSlopeUp,
            twentyOneD.slopeYawDegPerSec * dt);
        INFO("edge flicker yaw changes " << flickerYawChanges << ", yaw changes in the first " << dwellFrame - 1
                                         << " steady frames " << waitingYawChanges << ", frame " << dwellFrame << " yaw "
                                         << resumed[dwellFrame - 1].yawDeg << " (21c's step " << firstStepDeg << ")");
        CHECK(flickerYawChanges == 0);
        CHECK(waitingYawChanges == 0);
        CHECK(bitsOf(resumed[dwellFrame - 1].yawDeg) == bitsOf(firstStepDeg));
        CHECK(bitsOf(firstStepDeg) != bitsOf(onSlope.yawDeg));
    }
}

TEST_CASE("DAttackCamera.SlopeDwellToleranceIsTenDegrees", "[DAttack][CameraSlopeYaw][CameraSlopeDwell]")
{
    const OrbitCameraSettings twentyOneD = twentyOneDSettings();
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        const int dwellFrame = dwellFrameAt(fps);
        for (const double offsetDeg : { 9.9, -9.9, 10.1, -10.1 })
        {
            const bool continues = std::fabs(offsetDeg) < 10.0;
            const glm::vec3 nearUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, offsetDeg), true);
            const OrbitCameraState start = stateAt(120.f, kStartPitchDeg, kSlopeUp);
            const std::vector<OrbitCameraState> states = runTargets(dt, alternating(kSlopeUp, nearUp, 1, fps), twentyOneD,
                start);
            int firstTurn = 0;
            for (std::size_t frame = 0; frame < states.size() && firstTurn == 0; ++frame)
                if (bitsOf(states[frame].yawDeg) != bitsOf(start.yawDeg))
                    firstTurn = static_cast<int>(frame) + 1;
            INFO("fps " << fps << ", heading alternating with one " << offsetDeg << " deg off: first turning frame "
                        << firstTurn << " (dwell frame " << dwellFrame << ")");
            CHECK(firstTurn == (continues ? dwellFrame : 0));
        }
    }
}

TEST_CASE("DAttackCamera.SlopeDwellSteadySlopeTurnsAsTwentyOneCAfterTheDwell", "[DAttack][CameraSlopeYaw][CameraSlopeDwell]")
{
    const OrbitCameraSettings twentyOneD = twentyOneDSettings();
    OrbitCameraSettings noDwell = twentyOneD;
    noDwell.slopeYawDwellSeconds = 0.f;
    const int expectedDwellFrames[] = { 6, 12, 29, 48 };
    for (std::size_t rate = 0; rate < std::size(kFps); ++rate)
    {
        const int fps = kFps[rate];
        const float dt = dtAt(fps);
        const int dwellFrame = dwellFrameAt(fps);
        REQUIRE(dwellFrame == expectedDwellFrames[rate]);

        for (const Face& face : kThirtyDegreeFaces)
        {
            const glm::vec3 targetUp = dAttackCameraBehaviour::cameraTargetUp(face.up, true);
            const float targetYawDeg = dAttackCameraBehaviour::wrapYawDeg(dAttackCameraBehaviour::upSlopeYawDeg(targetUp));
            for (const float offsetDeg : kStartOffsetsDeg)
            {
                const float startYawDeg = dAttackCameraBehaviour::wrapYawDeg(targetYawDeg + offsetDeg);
                const double startGapDeg = signedYawGapDeg(targetYawDeg, startYawDeg);
                const int turnFrames = framesByTheFormula(startGapDeg, twentyOneD.slopeYawDegPerSec, dt);
                const int frames = dwellFrame - 1 + turnFrames + kHoldFrames;
                const OrbitCameraState start = stateAt(startYawDeg, kStartPitchDeg, targetUp);
                const std::vector<glm::vec3> targets(static_cast<std::size_t>(frames), targetUp);
                const std::vector<OrbitCameraState> gated = runTargets(dt, targets, twentyOneD, start);
                const std::vector<OrbitCameraState> ungated = runTargets(dt, targets, noDwell, start);

                int waitingYawChanges = 0;
                int shiftedYawMismatches = 0;
                int pitchOrFrameMismatches = 0;
                int arrivalFrame = -1;
                for (int frame = 1; frame <= frames; ++frame)
                {
                    const OrbitCameraState& state = gated[frame - 1];
                    if (frame < dwellFrame)
                        waitingYawChanges += (bitsOf(state.yawDeg) != bitsOf(startYawDeg)) ? 1 : 0;
                    else
                        shiftedYawMismatches +=
                            (bitsOf(state.yawDeg) != bitsOf(ungated[frame - dwellFrame].yawDeg)) ? 1 : 0;
                    OrbitCameraState sameYaw = ungated[frame - 1];
                    sameYaw.yawDeg = state.yawDeg;
                    pitchOrFrameMismatches += sameCameraBits(state, sameYaw) ? 0 : 1;
                    if (arrivalFrame < 0 && bitsOf(state.yawDeg) == bitsOf(targetYawDeg))
                        arrivalFrame = frame;
                }
                const bool dwellBoundary =
                    !(gated[dwellFrame - 2].slopeDwellSeconds + dAttackCameraBehaviour::kSlopeYawDwellEpsilonSeconds
                        >= twentyOneD.slopeYawDwellSeconds)
                    && gated[dwellFrame - 1].slopeDwellSeconds + dAttackCameraBehaviour::kSlopeYawDwellEpsilonSeconds
                        >= twentyOneD.slopeYawDwellSeconds;
                INFO(face.name << ", start yaw " << startYawDeg << " (gap " << startGapDeg << " deg): yaw changes in the first "
                               << dwellFrame - 1 << " frames " << waitingYawChanges << ", shifted-yaw mismatches "
                               << shiftedYawMismatches << ", pitch/frame-up mismatches " << pitchOrFrameMismatches
                               << ", arrived on frame " << arrivalFrame << ", " << dwellFrame - 1 + turnFrames
                               << " by the formula, dwell on frames " << dwellFrame - 1 << "/" << dwellFrame << " "
                               << std::setprecision(9) << gated[dwellFrame - 2].slopeDwellSeconds << "/"
                               << gated[dwellFrame - 1].slopeDwellSeconds);
                CHECK(waitingYawChanges == 0);
                CHECK(shiftedYawMismatches == 0);
                CHECK(pitchOrFrameMismatches == 0);
                CHECK(arrivalFrame == dwellFrame - 1 + turnFrames);
                CHECK(dwellBoundary);
                if (fps == 240)
                    CHECK(gated[dwellFrame - 1].slopeDwellSeconds < twentyOneD.slopeYawDwellSeconds);
            }
        }
    }
}

TEST_CASE("DAttackCamera.SlopeDwellLookModeOnResetsTheDwell", "[DAttack][CameraSlopeYaw][CameraSlopeDwell]")
{
    const OrbitCameraSettings twentyOneD = twentyOneDSettings();
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        const int dwellFrame = dwellFrameAt(fps);
        INFO("fps " << fps);
        const OrbitCameraState start = stateAt(120.f, kStartPitchDeg, kSlopeUp);
        for (const int lookOffFrames : { dwellFrame - 1, dwellFrame + 30 })
        {
            const std::vector<OrbitCameraState> before = runTargets(dt,
                std::vector<glm::vec3>(static_cast<std::size_t>(lookOffFrames), kSlopeUp), twentyOneD, start);
            REQUIRE(before.back().slopeDwellSeconds > 0.f);

            for (const glm::vec2 mouseUnits : { glm::vec2(0.f), glm::vec2(0.2f, -0.1f) })
            {
                OrbitCameraInput look;
                look.lookHeld = true;
                look.mouseUnits = mouseUnits;
                look.targetPitchDeg = kIsoTargetDeg;
                look.targetUp = kSlopeUp;
                OrbitCameraState withoutDwell = before.back();
                withoutDwell.slopeDwellSeconds = 0.f;
                withoutDwell.slopeDwellYawDeg = 0.f;
                const OrbitCameraState looked = dAttackCameraBehaviour::integrate(dt, look, twentyOneD, before.back());
                const OrbitCameraState lookedFresh = dAttackCameraBehaviour::integrate(dt, look, twentyOneD, withoutDwell);
                INFO("after " << lookOffFrames << " look-off frames, mouse (" << mouseUnits.x << ", " << mouseUnits.y
                              << "): dwell after the look frame " << looked.slopeDwellSeconds);
                CHECK(sameCameraBits(looked, lookedFresh));
                CHECK(bitsOf(looked.slopeDwellSeconds) == bitsOf(0.f));

                const std::vector<OrbitCameraState> after =
                    runTargets(dt, std::vector<glm::vec3>(static_cast<std::size_t>(dwellFrame), kSlopeUp), twentyOneD, looked);
                const std::vector<OrbitCameraState> waiting(after.begin(), after.end() - 1);
                CHECK(yawChangesFrom(waiting, looked.yawDeg) == 0);
                CHECK(bitsOf(after.back().yawDeg) != bitsOf(looked.yawDeg));
            }
        }
    }
}

TEST_CASE("DAttackCamera.SlopeDwellZeroIsTwentyOneC", "[DAttack][CameraSlopeYaw][CameraSlopeDwell]")
{
    const glm::vec3 flatUp = dAttackCameraBehaviour::kCameraUp;
    for (const float dwellSeconds : { 0.f, -1.f })
    {
        OrbitCameraSettings settings = twentyOneDSettings();
        settings.slopeYawDwellSeconds = dwellSeconds;
        for (const int fps : kFps)
        {
            const float dt = dtAt(fps);
            const std::vector<std::vector<glm::vec3>> sequences = {
                std::vector<glm::vec3>(static_cast<std::size_t>(2 * fps), kSlopeUp),
                alternating(kSlopeUp, flatUp, 1, 2 * fps),
                alternating(kSlopeUp, flatUp, 2, 2 * fps),
                alternating(kEdgeUp, kSlopeUp, 1, 2 * fps),
                std::vector<glm::vec3>(static_cast<std::size_t>(2 * fps), flatUp),
            };
            for (std::size_t sequence = 0; sequence < sequences.size(); ++sequence)
            {
                for (const float offsetDeg : kStartOffsetsDeg)
                {
                    OrbitCameraState shipped = stateAt(offsetDeg, kStartPitchDeg, kSlopeUp);
                    OrbitCameraState reference = shipped;
                    int mismatches = 0;
                    for (const glm::vec3& targetUp : sequences[sequence])
                    {
                        shipped = dAttackCameraBehaviour::integrate(dt, lookOff(targetUp), settings, shipped);
                        reference = twentyOneCLookOffStep(dt, targetUp, settings, reference);
                        mismatches += sameCameraBits(shipped, reference) ? 0 : 1;
                    }
                    INFO("dwell " << dwellSeconds << " s, fps " << fps << ", sequence " << sequence << ", start yaw "
                                  << offsetDeg << ": frames differing from 21c's look-off step " << mismatches);
                    CHECK(mismatches == 0);
                }
            }
        }
    }
}

namespace dAttackCameraSlopeYawTest
{
constexpr float kEaseStartOffsetsDeg[] = { 5.f, -5.f, 30.f, -91.9f, 180.f, -179.1f };
constexpr double kRateToleranceDegPerSec = 0.01;
constexpr double kStepToleranceDeg = 1e-4;
constexpr double kRateSumToleranceDegPerSec = 1e-5;

float goalOf(const glm::vec3& targetUp)
{
    return dAttackCameraBehaviour::wrapYawDeg(dAttackCameraBehaviour::upSlopeYawDeg(targetUp));
}

struct EaseRun
{
    int firstTurnFrame = -1;
    int arrivalFrame = -1;
    int wrongWaySteps = 0;
    int overshoots = 0;
    int growingGaps = 0;
    double firstStepRateDegPerSec = 0.0;
    float firstStateRateDegPerSec = 0.f;
    float firstDistanceDeg = 0.f;
    float worstStateRateDegPerSec = 0.f;
    double worstMeasuredRateDegPerSec = 0.0;
    float worstRateRiseDegPerSec = 0.f;
    double worstStepVsRateDeg = 0.0;
    int heldYawChanges = 0;
    int heldNonZeroRates = 0;
    OrbitCameraState arrived;
};

EaseRun easeOnASteadySlope(int fps, const glm::vec3& targetUp, float startYawDeg, const OrbitCameraSettings& settings)
{
    const float dt = dtAt(fps);
    const float goalDeg = goalOf(targetUp);
    const OrbitCameraInput off = lookOff(targetUp);
    EaseRun run;
    OrbitCameraState state = stateAt(startYawDeg, kStartPitchDeg, targetUp);
    double direction = 0.0;
    double previousDistanceDeg = std::fabs(signedYawGapDeg(goalDeg, startYawDeg));
    for (int frame = 1; frame <= kMaxFrames && run.arrivalFrame < 0; ++frame)
    {
        const OrbitCameraState next = dAttackCameraBehaviour::integrate(dt, off, settings, state);
        const double stepDeg = signedYawGapDeg(next.yawDeg, state.yawDeg);
        const double gapDeg = signedYawGapDeg(goalDeg, next.yawDeg);
        const bool arrived = bitsOf(next.yawDeg) == bitsOf(goalDeg);
        run.worstStateRateDegPerSec = std::max(run.worstStateRateDegPerSec, next.slopeYawRateDegPerSec);
        run.worstRateRiseDegPerSec =
            std::max(run.worstRateRiseDegPerSec, next.slopeYawRateDegPerSec - state.slopeYawRateDegPerSec);
        if (stepDeg != 0.0)
        {
            if (run.firstTurnFrame < 0)
            {
                run.firstTurnFrame = frame;
                direction = (stepDeg > 0.0) ? 1.0 : -1.0;
                run.firstStepRateDegPerSec = std::fabs(stepDeg) / dt;
                run.firstStateRateDegPerSec = next.slopeYawRateDegPerSec;
                run.firstDistanceDeg =
                    glm::abs(dAttackCameraBehaviour::wrapYawDeg(goalDeg - state.yawDeg));
            }
            else if ((stepDeg > 0.0) != (direction > 0.0))
                ++run.wrongWaySteps;
            run.worstMeasuredRateDegPerSec = std::max(run.worstMeasuredRateDegPerSec, std::fabs(stepDeg) / dt);
            if (!arrived)
                run.worstStepVsRateDeg = std::max(run.worstStepVsRateDeg,
                    std::fabs(std::fabs(stepDeg) - static_cast<double>(next.slopeYawRateDegPerSec) * dt));
        }
        if (gapDeg != 0.0 && direction != 0.0 && (gapDeg > 0.0) != (direction > 0.0))
            ++run.overshoots;
        if (std::fabs(gapDeg) > previousDistanceDeg)
            ++run.growingGaps;
        previousDistanceDeg = std::fabs(gapDeg);
        state = next;
        if (arrived)
            run.arrivalFrame = frame;
    }
    run.arrived = state;
    for (int frame = 0; frame < kHoldFrames; ++frame)
    {
        state = dAttackCameraBehaviour::integrate(dt, off, settings, state);
        run.heldYawChanges += (bitsOf(state.yawDeg) != bitsOf(goalDeg)) ? 1 : 0;
        run.heldNonZeroRates += (bitsOf(state.slopeYawRateDegPerSec) != bitsOf(0.f)) ? 1 : 0;
    }
    return run;
}

double turnSeconds(const EaseRun& run, int fps)
{
    return static_cast<double>(run.arrivalFrame - run.firstTurnFrame + 1) / static_cast<double>(fps);
}

int countNonZeroRates(const std::vector<OrbitCameraState>& states)
{
    int count = 0;
    for (const OrbitCameraState& state : states)
        count += (bitsOf(state.slopeYawRateDegPerSec) != bitsOf(0.f)) ? 1 : 0;
    return count;
}
} // namespace dAttackCameraSlopeYawTest

TEST_CASE("DAttackCamera.SlopeEaseTurnsTowardTheDwellsAnchor", "[DAttack][CameraSlopeYaw][CameraSlopeEase]")
{
    const OrbitCameraSettings defaults;
    REQUIRE(defaults.slopeYawDegPerSec == 90.f);
    REQUIRE(defaults.slopeYawDwellSeconds == 0.2f);
    REQUIRE(defaults.slopeYawEaseSeconds == 0.3f);
    REQUIRE(defaults.slopeYawAccelDegPerSec2 == 360.f);
    REQUIRE(dAttackCameraBehaviour::kSlopeYawArriveDeg == 0.01f);
    const float goalDeg = goalOf(kSlopeUp);
    REQUIRE(std::fabs(goalDeg) < 1e-4f);
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        const OrbitCameraState start = stateAt(dAttackCameraBehaviour::wrapYawDeg(goalDeg + 20.f), kStartPitchDeg, kSlopeUp);
        const std::vector<glm::vec3> steadyTargets(static_cast<std::size_t>(6 * fps), kSlopeUp);
        const std::vector<OrbitCameraState> steady = runTargets(dt, steadyTargets, defaults, start);
        REQUIRE(bitsOf(steady.back().yawDeg) == bitsOf(goalDeg));
        REQUIRE(bitsOf(steady.back().slopeYawRateDegPerSec) == bitsOf(0.f));

        for (const double offsetDeg : { 5.0, -5.0, 9.9, -9.9, 10.1, -10.1 })
        {
            const bool continues = std::fabs(offsetDeg) < 10.0;
            const glm::vec3 nearUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, offsetDeg), true);
            const float nearGoalDeg = goalOf(nearUp);
            REQUIRE(std::fabs(signedYawGapDeg(nearGoalDeg, goalDeg) - offsetDeg) < 1e-3);

            const std::vector<OrbitCameraState> flicker =
                runTargets(dt, alternating(nearUp, kSlopeUp, 1, fps + 1), defaults, steady.back());
            const int yawChanges = yawChangesFrom(flicker, goalDeg);
            const int nonZeroRates = countNonZeroRates(flicker);
            INFO("fps " << fps << ", up-slope heading alternating 0 / " << offsetDeg << " deg every tick for 1 s after "
                        << "arriving: yaw changes " << yawChanges << ", non-zero rates " << nonZeroRates << ", last dwell "
                        << flicker.back().slopeDwellSeconds << " s on heading " << flicker.back().slopeDwellYawDeg);
            CHECK(yawChanges == 0);
            CHECK(nonZeroRates == 0);
            if (continues)
            {
                CHECK(flicker.back().slopeDwellSeconds > 6.f);
                CHECK(bitsOf(flicker.back().slopeDwellYawDeg) == bitsOf(goalDeg));

                const std::vector<OrbitCameraState> turningFlicker =
                    runTargets(dt, alternating(kSlopeUp, nearUp, 1, 6 * fps), defaults, start);
                int turningMismatches = 0;
                for (std::size_t frame = 0; frame < steady.size(); ++frame)
                    turningMismatches += (bitsOf(turningFlicker[frame].yawDeg) != bitsOf(steady[frame].yawDeg)
                                             || bitsOf(turningFlicker[frame].slopeYawRateDegPerSec)
                                                 != bitsOf(steady[frame].slopeYawRateDegPerSec))
                        ? 1
                        : 0;
                INFO("the same alternation from 20 deg off, while turning: frames differing from the steady slope's "
                     << turningMismatches);
                CHECK(turningMismatches == 0);
            }
            else
            {
                CHECK(bitsOf(flicker.back().slopeDwellSeconds) == bitsOf(dt));
                CHECK(bitsOf(flicker.back().slopeDwellYawDeg) == bitsOf(nearGoalDeg));
            }
        }
    }
}

TEST_CASE("DAttackCamera.SlopeEaseFromFiveDegreesStartsGently", "[DAttack][CameraSlopeYaw][CameraSlopeEase]")
{
    const OrbitCameraSettings defaults;
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        for (const Face& face : kThirtyDegreeFaces)
        {
            const glm::vec3 targetUp = dAttackCameraBehaviour::cameraTargetUp(face.up, true);
            for (const float offsetDeg : { 5.f, -5.f })
            {
                const float startYawDeg = dAttackCameraBehaviour::wrapYawDeg(goalOf(targetUp) + offsetDeg);
                const EaseRun run = easeOnASteadySlope(fps, targetUp, startYawDeg, defaults);
                const double boundDegPerSec = std::min({ 90.0, static_cast<double>(run.firstDistanceDeg) / 0.3,
                    360.0 * static_cast<double>(dt) });
                INFO(face.name << ", fps " << fps << ", start " << offsetDeg << " deg off: first turning frame "
                               << run.firstTurnFrame << " (dwell frame " << dwellFrameAt(fps) << "), first step "
                               << run.firstStepRateDegPerSec << " deg/s (state " << run.firstStateRateDegPerSec
                               << "), bound min(90, " << run.firstDistanceDeg << " / 0.3, 360 * dt) = " << boundDegPerSec);
                CHECK(run.firstTurnFrame == dwellFrameAt(fps));
                CHECK(run.firstStepRateDegPerSec <= boundDegPerSec + kRateToleranceDegPerSec);
                CHECK(static_cast<double>(run.firstStateRateDegPerSec) <= boundDegPerSec * (1.0 + 1e-6));
                CHECK(run.firstStateRateDegPerSec > 0.f);
            }
        }
    }
}

TEST_CASE("DAttackCamera.SlopeEaseRampsCapsAndArrivesExactlyAtEveryFrameRate", "[DAttack][CameraSlopeYaw][CameraSlopeEase]")
{
    const OrbitCameraSettings defaults;
    for (const Face& face : kThirtyDegreeFaces)
    {
        const glm::vec3 targetUp = dAttackCameraBehaviour::cameraTargetUp(face.up, true);
        const float goalDeg = goalOf(targetUp);
        for (const float offsetDeg : kEaseStartOffsetsDeg)
        {
            const float startYawDeg = dAttackCameraBehaviour::wrapYawDeg(goalDeg + offsetDeg);
            double seconds[std::size(kFps)] = {};
            for (std::size_t rate = 0; rate < std::size(kFps); ++rate)
            {
                const int fps = kFps[rate];
                const float dt = dtAt(fps);
                const EaseRun run = easeOnASteadySlope(fps, targetUp, startYawDeg, defaults);
                seconds[rate] = turnSeconds(run, fps);
                INFO(face.name << ", fps " << fps << ", start " << offsetDeg << " deg off: turns on frame "
                               << run.firstTurnFrame << ", arrives on frame " << run.arrivalFrame << " ("
                               << seconds[rate] << " s of turning), wrong-way steps " << run.wrongWaySteps
                               << ", overshoots " << run.overshoots << ", growing gaps " << run.growingGaps
                               << ", top rate " << run.worstStateRateDegPerSec << " (measured "
                               << run.worstMeasuredRateDegPerSec << "), top rise " << run.worstRateRiseDegPerSec
                               << " deg/s per frame (360 * dt = " << 360.f * dt << "), worst step vs rate * dt "
                               << run.worstStepVsRateDeg << " deg, held yaw changes " << run.heldYawChanges
                               << ", held non-zero rates " << run.heldNonZeroRates);
                REQUIRE(run.arrivalFrame > 0);
                CHECK(run.firstTurnFrame == dwellFrameAt(fps));
                CHECK(run.wrongWaySteps == 0);
                CHECK(run.overshoots == 0);
                CHECK(run.growingGaps == 0);
                CHECK(run.worstStateRateDegPerSec <= 90.f);
                CHECK(run.worstMeasuredRateDegPerSec <= 90.0 + kRateToleranceDegPerSec);
                CHECK(static_cast<double>(run.worstRateRiseDegPerSec)
                    <= 360.0 * static_cast<double>(dt) + kRateSumToleranceDegPerSec);
                CHECK(run.worstStepVsRateDeg <= kStepToleranceDeg);
                CHECK(bitsOf(run.arrived.yawDeg) == bitsOf(goalDeg));
                CHECK(bitsOf(run.arrived.slopeYawRateDegPerSec) == bitsOf(0.f));
                CHECK(run.heldYawChanges == 0);
                CHECK(run.heldNonZeroRates == 0);
                if (std::fabs(offsetDeg) >= 90.f)
                    CHECK(run.worstStateRateDegPerSec == 90.f);
            }
            int inconsistentPairs = 0;
            double worstExcessSeconds = -1.0;
            for (std::size_t a = 0; a < std::size(kFps); ++a)
                for (std::size_t b = a + 1; b < std::size(kFps); ++b)
                {
                    const double allowed = 1.0 / static_cast<double>(std::min(kFps[a], kFps[b]))
                        + 0.02 * std::max(seconds[a], seconds[b]);
                    const double excess = std::fabs(seconds[a] - seconds[b]) - allowed;
                    worstExcessSeconds = std::max(worstExcessSeconds, excess);
                    inconsistentPairs += (excess > 0.0) ? 1 : 0;
                }
            INFO(face.name << ", start " << offsetDeg << " deg off: turning seconds at 30/60/144/240 fps " << seconds[0]
                           << " / " << seconds[1] << " / " << seconds[2] << " / " << seconds[3]
                           << ", worst pair excess over one frame + 2 % " << worstExcessSeconds << " s");
            CHECK(inconsistentPairs == 0);
        }
    }
}

TEST_CASE("DAttackCamera.SlopeEaseRateIsZeroWheneverTheTurnIsGated", "[DAttack][CameraSlopeYaw][CameraSlopeEase]")
{
    const OrbitCameraSettings defaults;
    const glm::vec3 unsupportedUp = dAttackCameraBehaviour::cameraTargetUp(upTiltedToward(30.0, 0.0), false);
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        const int dwellFrame = dwellFrameAt(fps);
        const OrbitCameraState start = stateAt(dAttackCameraBehaviour::wrapYawDeg(goalOf(kSlopeUp) + 120.f), kStartPitchDeg,
            kSlopeUp);
        const OrbitCameraState turning = runTargets(dt,
            std::vector<glm::vec3>(static_cast<std::size_t>(dwellFrame + fps / 2), kSlopeUp), defaults, start).back();
        REQUIRE(turning.slopeYawRateDegPerSec > 0.f);

        struct Gate
        {
            const char* name;
            OrbitCameraInput input;
            OrbitCameraSettings settings;
            bool restartsTheDwell;
        };
        OrbitCameraInput look;
        look.lookHeld = true;
        look.targetPitchDeg = kIsoTargetDeg;
        look.targetUp = kSlopeUp;
        OrbitCameraSettings held = defaults;
        held.slopeYawDegPerSec = 0.f;
        const Gate gates[] = {
            { "flat ground", lookOff(dAttackCameraBehaviour::kCameraUp), defaults, true },
            { "in the air", lookOff(unsupportedUp), defaults, true },
            { "a heading 40 deg off (the dwell restarts)", lookOff(kEdgeUp), defaults, true },
            { "look mode on, still", look, defaults, true },
            { "a turn rate of 0 (the dwell runs on)", lookOff(kSlopeUp), held, false },
        };
        for (const Gate& gate : gates)
        {
            const OrbitCameraState gated = dAttackCameraBehaviour::integrate(dt, gate.input, gate.settings, turning);
            INFO("fps " << fps << ", " << gate.name << ": rate " << gated.slopeYawRateDegPerSec << " (was "
                        << turning.slopeYawRateDegPerSec << "), yaw " << gated.yawDeg << " (was " << turning.yawDeg << ")");
            CHECK(bitsOf(gated.slopeYawRateDegPerSec) == bitsOf(0.f));
            CHECK(bitsOf(gated.yawDeg) == bitsOf(turning.yawDeg));

            const int resumeFrame = gate.restartsTheDwell ? dwellFrame : 1;
            const std::vector<OrbitCameraState> resumed =
                runTargets(dt, std::vector<glm::vec3>(static_cast<std::size_t>(resumeFrame), kSlopeUp), defaults, gated);
            CHECK(yawChangesFrom(std::vector<OrbitCameraState>(resumed.begin(), resumed.end() - 1), turning.yawDeg) == 0);
            CHECK(bitsOf(resumed.back().yawDeg) != bitsOf(turning.yawDeg));
            CHECK(static_cast<double>(resumed.back().slopeYawRateDegPerSec)
                <= 360.0 * static_cast<double>(dt) * (1.0 + 1e-6));
        }
    }
}

TEST_CASE("DAttackCamera.SlopeEaseNegativeOrNaNSettingsAreZero", "[DAttack][CameraSlopeYaw][CameraSlopeEase]")
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    struct Pair
    {
        const char* name;
        float easeSeconds;
        float accelDegPerSec2;
        float referenceEaseSeconds;
        float referenceAccelDegPerSec2;
    };
    const Pair pairs[] = {
        { "ease -1", -1.f, 360.f, 0.f, 360.f },
        { "ease NaN", nan, 360.f, 0.f, 360.f },
        { "accel -1", 0.3f, -1.f, 0.3f, 0.f },
        { "accel NaN", 0.3f, nan, 0.3f, 0.f },
        { "both -1", -1.f, -1.f, 0.f, 0.f },
        { "both NaN", nan, nan, 0.f, 0.f },
    };
    for (const int fps : kFps)
    {
        const float dt = dtAt(fps);
        for (const Pair& pair : pairs)
        {
            OrbitCameraSettings settings;
            settings.slopeYawEaseSeconds = pair.easeSeconds;
            settings.slopeYawAccelDegPerSec2 = pair.accelDegPerSec2;
            OrbitCameraSettings reference;
            reference.slopeYawEaseSeconds = pair.referenceEaseSeconds;
            reference.slopeYawAccelDegPerSec2 = pair.referenceAccelDegPerSec2;
            for (const float offsetDeg : kEaseStartOffsetsDeg)
            {
                const OrbitCameraState start =
                    stateAt(dAttackCameraBehaviour::wrapYawDeg(goalOf(kSlopeUp) + offsetDeg), kStartPitchDeg, kSlopeUp);
                const std::vector<glm::vec3> targets(static_cast<std::size_t>(5 * fps), kSlopeUp);
                const std::vector<OrbitCameraState> shipped = runTargets(dt, targets, settings, start);
                const std::vector<OrbitCameraState> expected = runTargets(dt, targets, reference, start);
                int mismatches = 0;
                for (std::size_t frame = 0; frame < shipped.size(); ++frame)
                    mismatches += (bitsOf(shipped[frame].yawDeg) != bitsOf(expected[frame].yawDeg)
                                      || bitsOf(shipped[frame].slopeYawRateDegPerSec)
                                          != bitsOf(expected[frame].slopeYawRateDegPerSec))
                        ? 1
                        : 0;
                INFO("fps " << fps << ", " << pair.name << ", start " << offsetDeg << " deg off: frames differing from "
                            << "the zero setting " << mismatches << ", final yaw " << shipped.back().yawDeg);
                CHECK(mismatches == 0);
                CHECK(bitsOf(shipped.back().yawDeg) == bitsOf(goalOf(kSlopeUp)));
            }
        }
    }
}

#endif // WITH_LOW_LEVEL_TESTS

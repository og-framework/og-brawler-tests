// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackOrbitCameraTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackCamera.h"
#include "OGBrawler/InputMapping/GameInputMapping.h"

#include "glm/geometric.hpp"
#include "glm/vec2.hpp"

#include <cmath>
#include <string>
#include <vector>

#if __has_include("Math/Rotator.h")
#include "CoreMinimal.h"
#define OG_ORBIT_CAMERA_TEST_HAS_UNREAL_MATH 1
#else
#define OG_ORBIT_CAMERA_TEST_HAS_UNREAL_MATH 0
#endif

namespace dAttackOrbitCameraTest
{
using dAttackCameraBehaviour::OrbitCameraInput;
using dAttackCameraBehaviour::OrbitCameraSettings;
using dAttackCameraBehaviour::OrbitCameraState;

constexpr float kDts[] = { 1.f / 30.f, 1.f / 60.f, 1.f / 100.f, 1.f / 144.f, 1.f / 240.f };
constexpr float kReferenceDt = 1.f / 60.f;
constexpr float kFrameRateToleranceDeg = 0.1f;
constexpr float kIsoTargetDeg = 60.f;
constexpr float kMouse2DSensitivity = 0.07f;
constexpr float kTriedHoldoffSeconds = 0.6f;
constexpr float kTriedDominanceRatio = 2.f;

OrbitCameraSettings holdoffAndDominanceOn()
{
    OrbitCameraSettings settings;
    settings.verticalHoldoffSeconds = kTriedHoldoffSeconds;
    settings.dominanceRatio = kTriedDominanceRatio;
    return settings;
}

struct Phase
{
    float seconds;
    bool lookHeld;
    glm::vec2 mouseUnitsPerSecond;
    glm::vec2 stick;
};

struct PathRun
{
    OrbitCameraState final;
    std::vector<OrbitCameraState> frames;
    std::vector<float> elapsed;
};

PathRun runPath(float dt, const std::vector<Phase>& phases, OrbitCameraState start,
                float targetDeg = kIsoTargetDeg, const OrbitCameraSettings& settings = {})
{
    PathRun run;
    run.final = start;
    float elapsed = 0.f;
    for (const Phase& phase : phases)
    {
        const long frames = std::lround(phase.seconds / dt);
        for (long frame = 0; frame < frames; ++frame)
        {
            const OrbitCameraInput input{ phase.lookHeld, phase.mouseUnitsPerSecond * dt, phase.stick, targetDeg };
            run.final = dAttackCameraBehaviour::integrate(dt, input, settings, run.final);
            elapsed += dt;
            run.frames.push_back(run.final);
            run.elapsed.push_back(elapsed);
        }
    }
    return run;
}

float yawDifferenceDeg(float a, float b)
{
    return std::fabs(std::remainder(a - b, 360.f));
}

OrbitCameraState at(float yawDeg, float pitchDeg)
{
    return OrbitCameraState{ yawDeg, pitchDeg, 0.f };
}

OrbitCameraState oneFrame(float dt, const OrbitCameraInput& input, OrbitCameraState start,
                          const OrbitCameraSettings& settings = {})
{
    return dAttackCameraBehaviour::integrate(dt, input, settings, start);
}

dInput::KeyModifier lookMouseModifier()
{
    const dInput::MappingContext ctx = dInput::gameMapping::buildDefaultContext();
    std::vector<dInput::KeyModifier> modifiers;
    for (const auto& mapping : ctx.actionMappings)
        if (mapping.action == &dInput::gameMapping::Look)
            for (const auto& binding : mapping.bindings)
                if (binding.key == dInput::KeyId::Mouse_XY)
                    modifiers.push_back(binding.modifier);
    REQUIRE(modifiers.size() == 1);
    return modifiers.front();
}

glm::vec2 lookActionFromMouseCounts(float countsRight, float countsUp)
{
    REQUIRE(lookMouseModifier() == dInput::KeyModifier::Negate);
    return -kMouse2DSensitivity * glm::vec2(countsRight, countsUp);
}

struct NamedPath
{
    std::string name;
    OrbitCameraState start;
    std::vector<Phase> phases;
};

std::vector<NamedPath> frameRatePaths()
{
    const glm::vec2 none(0.f);
    return {
        { "stick full right 2 s from 30", at(0.f, 30.f), { { 2.f, true, none, { 1.f, 0.f } } } },
        { "stick half tilt right 2 s from 20", at(0.f, 20.f), { { 2.f, true, none, { 0.5f, 0.f } } } },
        { "stick full left 2 s from 75", at(10.f, 75.f), { { 2.f, true, none, { -1.f, 0.f } } } },
        { "stick up 0.5 s then right 1.5 s (a hold-off expires mid-frame)", at(0.f, 60.f),
          { { 0.5f, true, none, { 0.f, 1.f } }, { 1.5f, true, none, { 1.f, 0.f } } } },
        { "stick 30 deg off horizontal 1 s from 30", at(0.f, 30.f),
          { { 1.f, true, none, { std::cos(0.5f), std::sin(0.5f) } } } },
        { "stick 45 deg diagonal 0.5 s from 30", at(0.f, 30.f),
          { { 0.5f, true, none, { std::sqrt(0.5f), std::sqrt(0.5f) } } } },
        { "mouse diagonal 0.5 s from 40", at(0.f, 40.f), { { 0.5f, true, { -40.f, 16.f }, none } } },
        { "mouse flick 140 deg in 0.5 s (rate cap binds)", at(0.f, 30.f), { { 0.5f, true, { -112.f, 0.f }, none } } },
        { "mouse diagonal 0.5 s then pan 1 s", at(0.f, 40.f),
          { { 0.5f, true, { -40.f, 16.f }, none }, { 1.f, true, { -60.f, 0.f }, none } } },
        { "mouse slow pan with 4 deg/s vertical drift 2 s", at(0.f, 25.f), { { 2.f, true, { -24.f, 1.6f }, none } } },
        { "mouse and stick summed 1 s", at(0.f, 70.f), { { 1.f, true, { 20.f, 0.f }, { 0.6f, 0.f } } } },
        { "drag, release 0.5 s, pan 1 s", at(0.f, 60.f),
          { { 0.5f, true, none, { 0.f, -1.f } }, { 0.5f, false, none, none }, { 1.f, true, none, { 1.f, 0.f } } } },
    };
}
} // namespace dAttackOrbitCameraTest

using namespace dAttackOrbitCameraTest;

TEST_CASE("DAttackOrbitCamera.SamePathSameAnglesAtEveryFrameRate", "[DAttack][OrbitCamera]")
{
    struct NamedSettings
    {
        const char* name;
        OrbitCameraSettings settings;
    };
    const NamedSettings settingsSets[] = { { "defaults", OrbitCameraSettings{} },
                                           { "hold-off 0.6 s, dominance 2", holdoffAndDominanceOn() } };
    for (const NamedSettings& set : settingsSets)
    {
        for (const NamedPath& path : frameRatePaths())
        {
            const OrbitCameraState reference =
                runPath(kReferenceDt, path.phases, path.start, kIsoTargetDeg, set.settings).final;
            for (const float dt : kDts)
            {
                const OrbitCameraState result = runPath(dt, path.phases, path.start, kIsoTargetDeg, set.settings).final;
                INFO(set.name << " | " << path.name << " fps " << 1.f / dt << " yaw " << result.yawDeg << " pitch "
                              << result.pitchDeg << " | 60 fps yaw " << reference.yawDeg << " pitch "
                              << reference.pitchDeg);
                CHECK(yawDifferenceDeg(result.yawDeg, reference.yawDeg) <= kFrameRateToleranceDeg);
                CHECK(std::fabs(result.pitchDeg - reference.pitchDeg) <= kFrameRateToleranceDeg);
            }
        }
    }
}

TEST_CASE("DAttackOrbitCamera.PullFollowsTheClosedFormBelowTheCap", "[DAttack][OrbitCamera]")
{
    const OrbitCameraSettings settings;
    const float eFold = dAttackCameraBehaviour::kPullYawDegreesPerEFold;
    const float diagonal = std::sqrt(0.5f);
    const float halfResponseTilt = settings.stickDeadzone
                                   + (1.f - settings.stickDeadzone) * std::pow(0.5f, 1.f / settings.stickExponent);
    const float halfResponse = dAttackCameraBehaviour::stickCurve(halfResponseTilt, settings);
    REQUIRE(halfResponse == Catch::Approx(0.5f).margin(1e-5f));

    struct ClosedForm
    {
        const char* name;
        float startDeg;
        Phase phase;
        float yawDegPerSec;
        float pitchDegPerSec;
    };
    const ClosedForm forms[] = {
        { "half-response stick right", 30.f, { 2.f, true, glm::vec2(0.f), { halfResponseTilt, 0.f } },
          settings.stickYawDegPerSec * halfResponse, 0.f },
        { "full stick 45 deg up-right", 30.f, { 2.f, true, glm::vec2(0.f), { diagonal, diagonal } },
          settings.stickYawDegPerSec * diagonal, -settings.stickPitchDegPerSec * diagonal },
        { "mouse 3:5 diagonal right-down", 30.f, { 1.f, true, { -12.f, 7.2f }, glm::vec2(0.f) },
          12.f * dAttackCameraBehaviour::kMouseDegreesPerUnit, 7.2f * dAttackCameraBehaviour::kMouseDegreesPerUnit },
    };
    for (const ClosedForm& form : forms)
    {
        const float rate = form.yawDegPerSec / eFold;
        const float rest = kIsoTargetDeg + form.pitchDegPerSec / rate;
        REQUIRE(rate * std::fabs(kIsoTargetDeg - form.startDeg) < settings.pullMaxDegPerSec);
        for (const float dt : kDts)
        {
            const PathRun run = runPath(dt, { form.phase }, at(0.f, form.startDeg));
            float worstError = 0.f;
            for (std::size_t frame = 0; frame < run.frames.size(); ++frame)
            {
                const float closedForm = rest + (form.startDeg - rest) * std::exp(-rate * run.elapsed[frame]);
                worstError = std::fmax(worstError, std::fabs(run.frames[frame].pitchDeg - closedForm));
            }
            INFO(form.name << " fps " << 1.f / dt << " rest " << rest << " final " << run.final.pitchDeg
                           << " worst error against the closed form " << worstError);
            CHECK(worstError < 0.01f);
        }
    }
}

TEST_CASE("DAttackOrbitCamera.AFortyFiveDegreeTurnClosesMostOfTheGap", "[DAttack][OrbitCamera]")
{
    const OrbitCameraSettings settings;
    const float halfResponseTilt = settings.stickDeadzone
                                   + (1.f - settings.stickDeadzone) * std::pow(0.5f, 1.f / settings.stickExponent);
    const float startDeg = 30.f;
    const Phase turns[] = {
        { 0.5f, true, { -36.f, 0.f }, glm::vec2(0.f) },
        { 0.5f, true, { 36.f, 0.f }, glm::vec2(0.f) },
        { 0.5f, true, glm::vec2(0.f), { halfResponseTilt, 0.f } },
    };
    for (const float dt : kDts)
    {
        for (const Phase& turn : turns)
        {
            const PathRun run = runPath(dt, { turn }, at(0.f, startDeg));
            const float closed = (run.final.pitchDeg - startDeg) / (kIsoTargetDeg - startDeg);
            INFO("fps " << 1.f / dt << " mouse " << turn.mouseUnitsPerSecond.x << " stick " << turn.stick.x << " yaw "
                        << run.final.yawDeg << " pitch " << run.final.pitchDeg << " closed " << closed);
            CHECK(std::fabs(run.final.yawDeg) == Catch::Approx(45.f).margin(0.01f));
            CHECK(closed >= 0.85f);
            CHECK(run.final.pitchDeg <= kIsoTargetDeg);
        }
    }
}

TEST_CASE("DAttackOrbitCamera.MouseIsADisplacement", "[DAttack][OrbitCamera]")
{
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const OrbitCameraState single = oneFrame(dt, { true, lookActionFromMouseCounts(1.f, 0.f), {}, kIsoTargetDeg },
                                                 at(0.f, kIsoTargetDeg));
        CHECK(single.yawDeg == Catch::Approx(dAttackCameraBehaviour::kMouseDegreesPerUnit * kMouse2DSensitivity).margin(1e-5f));
        CHECK(single.pitchDeg == Catch::Approx(kIsoTargetDeg).margin(1e-5f));

        const OrbitCameraState unitStep = oneFrame(dt, { true, { 0.07f, 0.f }, {}, kIsoTargetDeg }, at(0.f, kIsoTargetDeg));
        CHECK(unitStep.yawDeg == Catch::Approx(-0.175f).margin(1e-5f));

        const float rates[] = { 10.f, 20.f };
        float yaw[2] = {};
        float pitch[2] = {};
        for (int i = 0; i < 2; ++i)
        {
            const OrbitCameraState slowDrag = runPath(dt, { { 0.5f, true, { -rates[i], 0.f }, glm::vec2(0.f) } },
                                                      at(0.f, kIsoTargetDeg)).final;
            const OrbitCameraState vertical = runPath(dt, { { 0.5f, true, { 0.f, rates[i] * 0.2f }, glm::vec2(0.f) } },
                                                      at(0.f, 40.f)).final;
            yaw[i] = slowDrag.yawDeg;
            pitch[i] = vertical.pitchDeg - 40.f;
        }
        CHECK(yaw[0] == Catch::Approx(12.5f).margin(1e-3f));
        CHECK(yaw[1] == Catch::Approx(2.f * yaw[0]).margin(1e-3f));
        CHECK(pitch[0] == Catch::Approx(2.5f).margin(1e-3f));
        CHECK(pitch[1] == Catch::Approx(2.f * pitch[0]).margin(1e-3f));
    }
}

TEST_CASE("DAttackOrbitCamera.StickDeadzoneAndCurve", "[DAttack][OrbitCamera]")
{
    const OrbitCameraSettings settings;
    CHECK(dAttackCameraBehaviour::stickCurve(0.14f, settings) == 0.f);
    CHECK(dAttackCameraBehaviour::stickCurve(0.15f, settings) == 0.f);
    CHECK(dAttackCameraBehaviour::stickCurve(0.1501f, settings) < 1e-5f);
    CHECK(dAttackCameraBehaviour::stickCurve(0.575f, settings) == Catch::Approx(std::pow(0.5f, 1.5f)).margin(1e-6f));
    CHECK(dAttackCameraBehaviour::stickCurve(1.f, settings) == Catch::Approx(1.f).margin(1e-6f));
    CHECK(dAttackCameraBehaviour::stickCurve(1.5f, settings) == Catch::Approx(1.f).margin(1e-6f));

    float previous = 0.f;
    float largestStep = 0.f;
    for (int i = 0; i <= 1000; ++i)
    {
        const float response = dAttackCameraBehaviour::stickCurve(0.15f + 0.00085f * static_cast<float>(i), settings);
        CHECK(response >= previous);
        largestStep = std::fmax(largestStep, response - previous);
        previous = response;
    }
    CHECK(largestStep < 0.003f);

    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const glm::vec2 insideDeadzone[] = { { 0.14f, 0.f }, { 0.f, 0.14f }, { -0.1f, 0.1f } };
        for (const glm::vec2& stick : insideDeadzone)
        {
            const OrbitCameraState still = runPath(dt, { { 1.f, true, glm::vec2(0.f), stick } }, at(5.f, 45.f)).final;
            CHECK(still.yawDeg == 5.f);
            CHECK(still.pitchDeg == 45.f);
        }

        const OrbitCameraState radial = runPath(dt, { { 1.f, true, glm::vec2(0.f), { 0.12f, 0.12f } } }, at(0.f, 45.f)).final;
        CHECK(radial.yawDeg > 0.f);

        const OrbitCameraState fullRight = runPath(dt, { { 0.5f, true, glm::vec2(0.f), { 1.f, 0.f } } }, at(0.f, 45.f)).final;
        CHECK(fullRight.yawDeg == Catch::Approx(90.f).margin(1e-2f));
    }
}

TEST_CASE("DAttackOrbitCamera.PitchClampHoldsUnderTenSecondsOfVerticalInput", "[DAttack][OrbitCamera]")
{
    struct Push
    {
        glm::vec2 mouseUnitsPerSecond;
        glm::vec2 stick;
        float expectedDeg;
    };
    const Push pushes[] = {
        { glm::vec2(0.f), { 0.f, -1.f }, dAttackCameraBehaviour::kPitchMaxDeg },
        { glm::vec2(0.f), { 0.f, 1.f }, dAttackCameraBehaviour::kPitchMinDeg },
        { { 0.f, 400.f }, glm::vec2(0.f), dAttackCameraBehaviour::kPitchMaxDeg },
        { { 0.f, -400.f }, glm::vec2(0.f), dAttackCameraBehaviour::kPitchMinDeg },
        { { -30.f, 400.f }, { 1.f, -1.f }, dAttackCameraBehaviour::kPitchMaxDeg },
    };
    for (const float dt : kDts)
    {
        for (const Push& push : pushes)
        {
            const PathRun run = runPath(dt, { { 10.f, true, push.mouseUnitsPerSecond, push.stick } }, at(0.f, kIsoTargetDeg));
            float lowest = 90.f;
            float highest = 0.f;
            for (const OrbitCameraState& frame : run.frames)
            {
                lowest = std::fmin(lowest, frame.pitchDeg);
                highest = std::fmax(highest, frame.pitchDeg);
            }
            INFO("fps " << 1.f / dt << " expected " << push.expectedDeg << " range [" << lowest << ", " << highest << "]");
            CHECK(lowest >= dAttackCameraBehaviour::kPitchMinDeg);
            CHECK(highest <= dAttackCameraBehaviour::kPitchMaxDeg);
            CHECK(run.final.pitchDeg == push.expectedDeg);
            CHECK(std::fabs(run.final.yawDeg) <= 180.f);
        }
    }

    OrbitCameraSettings outOfRange;
    outOfRange.pitchMinDeg = -20.f;
    outOfRange.pitchMaxDeg = 120.f;
    CHECK(dAttackCameraBehaviour::clampPitchDeg(-5.f, outOfRange) == dAttackCameraBehaviour::kHardPitchMinDeg);
    CHECK(dAttackCameraBehaviour::clampPitchDeg(95.f, outOfRange) == dAttackCameraBehaviour::kHardPitchMaxDeg);
    OrbitCameraSettings inverted;
    inverted.pitchMinDeg = 70.f;
    inverted.pitchMaxDeg = 20.f;
    CHECK(dAttackCameraBehaviour::clampPitchDeg(10.f, inverted) == 70.f);
    CHECK(dAttackCameraBehaviour::clampPitchDeg(85.f, inverted) == 70.f);
}

TEST_CASE("DAttackOrbitCamera.PullConvergesWithoutOvershootWithinTheRateCap", "[DAttack][OrbitCamera]")
{
    const float capDegPerSec = dAttackCameraBehaviour::kPullMaxDegPerSec;
    struct Approach
    {
        const char* name;
        float startDeg;
        Phase phase;
    };
    const Approach approaches[] = {
        { "stick right from 30", 30.f, { 4.f, true, glm::vec2(0.f), { 1.f, 0.f } } },
        { "stick left from 30", 30.f, { 4.f, true, glm::vec2(0.f), { -1.f, 0.f } } },
        { "stick right from 80", 80.f, { 4.f, true, glm::vec2(0.f), { 1.f, 0.f } } },
        { "mouse flick from 30", 30.f, { 1.f, true, { -400.f, 0.f }, glm::vec2(0.f) } },
        { "mouse flick left from 75", 75.f, { 1.f, true, { 400.f, 0.f }, glm::vec2(0.f) } },
    };
    for (const float dt : kDts)
    {
        float rightFinal = 0.f;
        float leftFinal = 0.f;
        for (const Approach& approach : approaches)
        {
            const PathRun run = runPath(dt, { approach.phase }, at(0.f, approach.startDeg));
            const float direction = approach.startDeg < kIsoTargetDeg ? 1.f : -1.f;
            float previous = approach.startDeg;
            float largestStepRate = 0.f;
            bool monotone = true;
            bool overshot = false;
            for (const OrbitCameraState& frame : run.frames)
            {
                const float step = (frame.pitchDeg - previous) * direction;
                monotone = monotone && step >= 0.f;
                overshot = overshot || (frame.pitchDeg - kIsoTargetDeg) * direction > 1e-4f;
                largestStepRate = std::fmax(largestStepRate, step / dt);
                previous = frame.pitchDeg;
            }
            INFO(approach.name << " fps " << 1.f / dt << " final " << run.final.pitchDeg << " largest pull rate "
                               << largestStepRate << " deg/s");
            CHECK(monotone);
            CHECK_FALSE(overshot);
            CHECK(largestStepRate <= capDegPerSec * 1.001f);
            if (approach.phase.mouseUnitsPerSecond.x != 0.f)
                CHECK(largestStepRate == Catch::Approx(capDegPerSec).epsilon(0.001f));
            if (approach.phase.seconds >= 4.f)
                CHECK(run.final.pitchDeg == Catch::Approx(kIsoTargetDeg).margin(0.05f));
            if (std::string(approach.name) == "stick right from 30")
                rightFinal = run.frames[static_cast<std::size_t>(std::lround(0.5f / dt)) - 1].pitchDeg;
            if (std::string(approach.name) == "stick left from 30")
                leftFinal = run.frames[static_cast<std::size_t>(std::lround(0.5f / dt)) - 1].pitchDeg;
        }
        CHECK(leftFinal == Catch::Approx(rightFinal).margin(1e-4f));
        CHECK(rightFinal > 45.f);
    }
}

TEST_CASE("DAttackOrbitCamera.NoPullDuringVerticalInputOrTheHoldoffWhenSwitchedOn", "[DAttack][OrbitCamera]")
{
    const OrbitCameraSettings on = holdoffAndDominanceOn();
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);

        const PathRun diagonal = runPath(dt, { { 1.f, true, { -12.f, 7.2f }, glm::vec2(0.f) } }, at(0.f, 30.f),
                                         kIsoTargetDeg, on);
        CHECK(diagonal.final.pitchDeg == Catch::Approx(30.f + 18.f).margin(1e-3f));
        CHECK(diagonal.final.holdoffSeconds == 0.f);

        const std::vector<Phase> dragThenPan{ { 0.5f, true, glm::vec2(0.f), { 0.f, 1.f } },
                                              { 1.f, true, glm::vec2(0.f), { 1.f, 0.f } } };
        const PathRun run = runPath(dt, dragThenPan, at(0.f, kIsoTargetDeg), kIsoTargetDeg, on);
        const std::size_t dragFrames = static_cast<std::size_t>(std::lround(0.5f / dt));
        const float afterDrag = run.frames[dragFrames - 1].pitchDeg;
        CHECK(afterDrag == dAttackCameraBehaviour::kPitchMinDeg);
        CHECK(run.frames[dragFrames - 1].holdoffSeconds == kTriedHoldoffSeconds);
        bool heldDuringHoldoff = true;
        for (std::size_t frame = dragFrames; frame < run.frames.size(); ++frame)
        {
            const float panSeconds = run.elapsed[frame] - 0.5f;
            if (panSeconds <= kTriedHoldoffSeconds - 1e-4f)
                heldDuringHoldoff = heldDuringHoldoff && std::fabs(run.frames[frame].pitchDeg - afterDrag) < 1e-4f;
        }
        CHECK(heldDuringHoldoff);
        CHECK(run.final.pitchDeg > afterDrag + 5.f);

        const std::vector<Phase> dragReleasePan{ { 0.5f, true, glm::vec2(0.f), { 0.f, 1.f } },
                                                 { 1.f, false, glm::vec2(0.f), glm::vec2(0.f) },
                                                 { 0.5f, true, glm::vec2(0.f), { 1.f, 0.f } } };
        const PathRun released = runPath(dt, dragReleasePan, at(0.f, kIsoTargetDeg), kIsoTargetDeg, on);
        const std::size_t firstPan = static_cast<std::size_t>(std::lround(1.5f / dt));
        CHECK(released.frames[firstPan - 1].holdoffSeconds == 0.f);
        CHECK(released.frames[firstPan].pitchDeg > dAttackCameraBehaviour::kPitchMinDeg);

        const PathRun tremor = runPath(dt, { { 1.f, true, { -40.f, 1.6f }, glm::vec2(0.f) } }, at(0.f, 30.f),
                                       kIsoTargetDeg, on);
        CHECK(tremor.final.holdoffSeconds == 0.f);
        CHECK(tremor.final.pitchDeg > 30.f + 4.f + 5.f);
    }
}

TEST_CASE("DAttackOrbitCamera.NoPullWithoutLookInput", "[DAttack][OrbitCamera]")
{
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const PathRun idle = runPath(dt, { { 5.f, true, glm::vec2(0.f), glm::vec2(0.f) } }, at(33.f, 25.f));
        CHECK(idle.final.yawDeg == 33.f);
        CHECK(idle.final.pitchDeg == 25.f);

        const PathRun stickUp = runPath(dt, { { 0.5f, true, glm::vec2(0.f), { 0.f, 1.f } } }, at(33.f, 70.f));
        CHECK(stickUp.final.yawDeg == 33.f);
        CHECK(stickUp.final.pitchDeg == Catch::Approx(70.f - 0.5f * dAttackCameraBehaviour::kStickPitchDegPerSec).margin(1e-3f));

        const PathRun mouseDown = runPath(dt, { { 0.5f, true, { 0.f, 8.f }, glm::vec2(0.f) } }, at(33.f, 25.f));
        CHECK(mouseDown.final.yawDeg == 33.f);
        CHECK(mouseDown.final.pitchDeg == Catch::Approx(25.f + 4.f * dAttackCameraBehaviour::kMouseDegreesPerUnit).margin(1e-3f));
    }
}

TEST_CASE("DAttackOrbitCamera.PullActsDuringDiagonalsAndRightAfterVerticalInput", "[DAttack][OrbitCamera]")
{
    const OrbitCameraSettings defaults;
    CHECK(defaults.verticalHoldoffSeconds == 0.f);
    CHECK(defaults.dominanceRatio == 0.f);

    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);

        const PathRun mouseDiagonal = runPath(dt, { { 1.f, true, { -12.f, 7.2f }, glm::vec2(0.f) } }, at(0.f, 30.f));
        CHECK(mouseDiagonal.final.pitchDeg > 30.f + 18.f + 10.f);
        CHECK(mouseDiagonal.final.holdoffSeconds == 0.f);

        const float diagonal = std::sqrt(0.5f);
        const PathRun stickDiagonal =
            runPath(dt, { { 0.5f, true, glm::vec2(0.f), { diagonal, diagonal } } }, at(0.f, kIsoTargetDeg));
        const float playerOnly = kIsoTargetDeg - 0.5f * dAttackCameraBehaviour::kStickPitchDegPerSec * diagonal;
        CHECK(stickDiagonal.final.pitchDeg > playerOnly + 10.f);
        CHECK(stickDiagonal.final.pitchDeg < kIsoTargetDeg);
        CHECK(stickDiagonal.final.holdoffSeconds == 0.f);

        const std::vector<Phase> dragThenPan{ { 0.5f, true, glm::vec2(0.f), { 0.f, 1.f } },
                                              { 0.5f, true, glm::vec2(0.f), { 1.f, 0.f } } };
        const PathRun run = runPath(dt, dragThenPan, at(0.f, kIsoTargetDeg));
        const std::size_t dragFrames = static_cast<std::size_t>(std::lround(0.5f / dt));
        CHECK(run.frames[dragFrames - 1].pitchDeg == dAttackCameraBehaviour::kPitchMinDeg);
        CHECK(run.frames[dragFrames - 1].holdoffSeconds == 0.f);
        CHECK(run.frames[dragFrames].pitchDeg > dAttackCameraBehaviour::kPitchMinDeg + 0.5f);
        CHECK(run.final.pitchDeg > kIsoTargetDeg - 1.f);
    }
}

TEST_CASE("DAttackOrbitCamera.HostPullSettingsAreSanitized", "[DAttack][OrbitCamera]")
{
    const Phase panRight{ 1.f, true, glm::vec2(0.f), { 1.f, 0.f } };
    const Phase diagonalUp{ 0.5f, true, glm::vec2(0.f), { std::sqrt(0.5f), std::sqrt(0.5f) } };
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const float eFolds[] = { 0.f, -5.f, 0.001f };
        for (const float eFold : eFolds)
        {
            OrbitCameraSettings snap;
            snap.pullYawDegreesPerEFold = eFold;
            const PathRun run = runPath(dt, { panRight }, at(0.f, 30.f), kIsoTargetDeg, snap);
            INFO("e-fold " << eFold << " final " << run.final.pitchDeg);
            bool sane = true;
            float previous = 30.f;
            for (const OrbitCameraState& frame : run.frames)
            {
                sane = sane && std::isfinite(frame.pitchDeg) && frame.pitchDeg >= previous && frame.pitchDeg <= kIsoTargetDeg;
                previous = frame.pitchDeg;
            }
            CHECK(sane);
            CHECK(run.final.pitchDeg == Catch::Approx(kIsoTargetDeg).margin(1e-3f));
        }

        const float caps[] = { 0.f, -10.f };
        for (const float cap : caps)
        {
            OrbitCameraSettings noPull;
            noPull.pullMaxDegPerSec = cap;
            const PathRun run = runPath(dt, { panRight }, at(0.f, 30.f), kIsoTargetDeg, noPull);
            INFO("cap " << cap);
            CHECK(run.final.pitchDeg == 30.f);
        }

        const OrbitCameraState reference = runPath(dt, { diagonalUp }, at(0.f, 30.f)).final;
        OrbitCameraSettings negative;
        negative.verticalHoldoffSeconds = -1.f;
        negative.dominanceRatio = -3.f;
        const OrbitCameraState result = runPath(dt, { diagonalUp }, at(0.f, 30.f), kIsoTargetDeg, negative).final;
        CHECK(result.pitchDeg == reference.pitchDeg);
        CHECK(result.holdoffSeconds == 0.f);
    }
}

TEST_CASE("DAttackOrbitCamera.NothingMovesWithoutLookHeld", "[DAttack][OrbitCamera]")
{
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const OrbitCameraState start{ -120.f, 25.f, 0.6f };
        const PathRun run = runPath(dt, { { 2.f, false, { 500.f, -300.f }, { 1.f, 1.f } } }, start);
        for (const OrbitCameraState& frame : run.frames)
        {
            CHECK(frame.yawDeg == start.yawDeg);
            CHECK(frame.pitchDeg == start.pitchDeg);
        }
        CHECK(run.final.holdoffSeconds == 0.f);
    }
}

TEST_CASE("DAttackOrbitCamera.SignPins", "[DAttack][OrbitCamera]")
{
    CHECK(lookMouseModifier() == dInput::KeyModifier::Negate);

    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const OrbitCameraState start = at(0.f, 45.f);
        const OrbitCameraState mouseRight = oneFrame(dt, { true, lookActionFromMouseCounts(10.f, 0.f), {}, 45.f }, start);
        const OrbitCameraState mouseUp = oneFrame(dt, { true, lookActionFromMouseCounts(0.f, 10.f), {}, 45.f }, start);
        const OrbitCameraState stickRight = oneFrame(dt, { true, {}, { 1.f, 0.f }, 45.f }, start);
        const OrbitCameraState stickUp = oneFrame(dt, { true, {}, { 0.f, 1.f }, 45.f }, start);
        CHECK(mouseRight.yawDeg > 0.f);
        CHECK(mouseUp.pitchDeg < 45.f);
        CHECK(mouseUp.yawDeg == 0.f);
        CHECK(stickRight.yawDeg > 0.f);
        CHECK(stickUp.pitchDeg < 45.f);

        OrbitCameraSettings inverted;
        inverted.invertMouseY = true;
        inverted.invertStickY = true;
        const OrbitCameraState invertedMouseUp =
            oneFrame(dt, { true, lookActionFromMouseCounts(10.f, 10.f), {}, 45.f }, start, inverted);
        const OrbitCameraState invertedStickUp = oneFrame(dt, { true, {}, { 0.5f, 1.f }, 45.f }, start, inverted);
        CHECK(invertedMouseUp.pitchDeg > 45.f);
        CHECK(invertedMouseUp.yawDeg > 0.f);
        CHECK(invertedStickUp.pitchDeg > 45.f);
        CHECK(invertedStickUp.yawDeg > 0.f);
    }

#if OG_ORBIT_CAMERA_TEST_HAS_UNREAL_MATH
    const OrbitCameraState turnedRight{ 90.f, 60.f, 0.f };
    const FVector forward = FRotator(-turnedRight.pitchDeg, turnedRight.yawDeg, 0.f).Vector();
    INFO("boom forward " << forward.X << ", " << forward.Y << ", " << forward.Z);
    CHECK(forward.Y == Catch::Approx(0.5).margin(1e-5));
    CHECK(forward.X == Catch::Approx(0.0).margin(1e-5));
    CHECK(forward.Z == Catch::Approx(-std::sqrt(3.0) / 2.0).margin(1e-5));
    CHECK(forward.GetSafeNormal2D().Equals(FVector::RightVector, 1e-5));
#endif
}

TEST_CASE("DAttackOrbitCamera.BoomLengthKeepsTheOldCurve", "[DAttack][OrbitCamera]")
{
    const OrbitCameraSettings settings;
    const float targets[] = { 10.f, 35.f, 60.f, 80.f };
    for (const float target : targets)
    {
        INFO("target " << target);
        CHECK(dAttackCameraBehaviour::boomLength(target, target, settings) == dAttackCameraBehaviour::kBoomLengthAtTargetPitch);
        CHECK(dAttackCameraBehaviour::boomLength(85.f, target, settings) == dAttackCameraBehaviour::kBoomLengthAtTargetPitch);
        CHECK(dAttackCameraBehaviour::boomLength(0.f, target, settings) == dAttackCameraBehaviour::kBoomLengthAtHorizon);
        CHECK(dAttackCameraBehaviour::boomLength(target * 0.5f, target, settings) == Catch::Approx(650.f).margin(1e-3f));
    }
    CHECK(dAttackCameraBehaviour::kBoomLengthAtTargetPitch == 900.f);
    CHECK(dAttackCameraBehaviour::kBoomLengthAtHorizon == 400.f);
    CHECK(dAttackCameraBehaviour::boomLength(10.f, 0.f, settings) == dAttackCameraBehaviour::kBoomLengthAtTargetPitch);
    CHECK(dAttackCameraBehaviour::boomLength(5.f, -30.f, settings) == Catch::Approx(650.f).margin(1e-3f));
}

TEST_CASE("DAttackOrbitCamera.YawStaysWrapped", "[DAttack][OrbitCamera]")
{
    for (const float dt : kDts)
    {
        INFO("fps " << 1.f / dt);
        const PathRun run = runPath(dt, { { 10.f, true, glm::vec2(0.f), { 1.f, 0.f } } }, at(170.f, kIsoTargetDeg));
        float widest = 0.f;
        for (const OrbitCameraState& frame : run.frames)
            widest = std::fmax(widest, std::fabs(frame.yawDeg));
        CHECK(widest <= 180.f);
        CHECK(yawDifferenceDeg(run.final.yawDeg, 170.f) <= 0.1f);
    }
}

#endif // WITH_LOW_LEVEL_TESTS

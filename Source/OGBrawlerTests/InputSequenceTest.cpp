// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/InputSequence/InputSequence.h"
#include "OGBrawler/DAttackMachineSimulation.h"
#include <map>

// ---------------------------------------------------------------------------
// Convention (see InputSequence.h):
//   angle 0      = aim direction (Forward).
//   angle +pi/2  = right-of-aim  (numpad Down, clockwise from forward).
//   angle +pi    = opposite of aim (numpad Back).
//   angle -pi/2  = left-of-aim   (numpad Up).
//   angles wrap at +-pi.
//
// The matcher reads moveDirectionWorld (world-frame XY) from history entries
// because aim is in world coords. Tests treat the "stick" arg as already
// world-frame (a stand-in for moveDirectionWorld) for simplicity.
// ---------------------------------------------------------------------------

namespace inputseqtests
{

static dAttackMachineSimulation::PlayerInput makeInput(glm::vec2 stick, glm::vec3 aim)
{
    dAttackMachineSimulation::PlayerInput pi;
    pi.moveDirection      = stick;
    pi.moveDirectionWorld = glm::vec3(stick.x, stick.y, 0.f);
    pi.aimDirection       = aim;
    return pi;
}

// Hadouken motion (aim-relative): stick angle sweeps Back -> DownBack -> Down,
// finishing with attackLeft rising edge.
static inputSequence::MotionCommand makeHadouken(uint8_t maxGap = 8,
                                                 uint8_t windowAfterFinal = 6,
                                                 float   tolerance = inputSequence::pi / 8.f)
{
    using namespace inputSequence;
    return MotionCommand{
        {
            MotionStep{ angle::Back,     tolerance, maxGap },
            MotionStep{ angle::DownBack, tolerance, maxGap },
            MotionStep{ angle::Down,     tolerance, maxGap },
        },
        0b01,
        windowAfterFinal,
        kHadoukenActionId
    };
}

static constexpr float kDeadzone = 0.2f;

} // namespace inputseqtests

// ---------------------------------------------------------------------------
// aimRelativeAngle — axis-aligned aim (1,0,0): all eight cardinal/diagonal
// directions map to their expected angle values.
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.AimRelativeAngle.AxisAlignedAim", "[InputSequence]")
{
    using namespace inputSequence;

    const glm::vec3 aim(1.f, 0.f, 0.f);
    constexpr float eps = 1e-4f;

    auto angleOf = [&](glm::vec2 stick) {
        auto a = aimRelativeAngle(stick, aim, inputseqtests::kDeadzone);
        REQUIRE(a.has_value());
        return *a;
    };

    REQUIRE(std::abs(angleOf(glm::vec2( 1.f,  0.f)) - angle::Forward    ) < eps);
    REQUIRE(std::abs(angleOf(glm::vec2( 1.f, -1.f)) - angle::DownForward) < eps);
    REQUIRE(std::abs(angleOf(glm::vec2( 0.f, -1.f)) - angle::Down       ) < eps);
    REQUIRE(std::abs(angleOf(glm::vec2(-1.f, -1.f)) - angle::DownBack   ) < eps);

    // Back wraps: any of +pi or -pi is valid; angularDistance treats them equal.
    REQUIRE(angularDistance(angleOf(glm::vec2(-1.f, 0.f)), angle::Back) < eps);

    REQUIRE(std::abs(angleOf(glm::vec2(-1.f,  1.f)) - angle::UpBack     ) < eps);
    REQUIRE(std::abs(angleOf(glm::vec2( 0.f,  1.f)) - angle::Up         ) < eps);
    REQUIRE(std::abs(angleOf(glm::vec2( 1.f,  1.f)) - angle::UpForward  ) < eps);
}

// ---------------------------------------------------------------------------
// aimRelativeAngle — rotated aim (0.707, 0.707, 0): the aim-relative frame
// rotates with the reference, so a stick aligned with the diagonal returns 0.
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.AimRelativeAngle.RotatedAim", "[InputSequence]")
{
    using namespace inputSequence;

    const glm::vec3 aim(0.707107f, 0.707107f, 0.f);
    constexpr float eps = 1e-3f;

    auto angleOf = [&](glm::vec2 stick) {
        auto a = aimRelativeAngle(stick, aim, inputseqtests::kDeadzone);
        REQUIRE(a.has_value());
        return *a;
    };

    REQUIRE(std::abs(angleOf(glm::vec2( 0.707107f,  0.707107f)) - angle::Forward) < eps);
    REQUIRE(std::abs(angleOf(glm::vec2( 0.707107f, -0.707107f)) - angle::Down   ) < eps);
    REQUIRE(angularDistance(angleOf(glm::vec2(-0.707107f, -0.707107f)), angle::Back) < eps);
    REQUIRE(std::abs(angleOf(glm::vec2(-0.707107f,  0.707107f)) - angle::Up     ) < eps);
}

// ---------------------------------------------------------------------------
// aimRelativeAngle — nullopt cases: below deadzone and degenerate aim.
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.AimRelativeAngle.NulloptCases", "[InputSequence]")
{
    using namespace inputSequence;

    const glm::vec3 validAim(1.f, 0.f, 0.f);
    const glm::vec3 zeroAim(0.f, 0.f, 0.f);

    REQUIRE_FALSE(aimRelativeAngle(glm::vec2(0.1f, 0.f), validAim, 0.5f ).has_value());
    REQUIRE_FALSE(aimRelativeAngle(glm::vec2(0.f,  0.f), validAim, 0.01f).has_value());
    REQUIRE_FALSE(aimRelativeAngle(glm::vec2(1.f,  0.f), zeroAim, inputseqtests::kDeadzone).has_value());
    REQUIRE_FALSE(aimRelativeAngle(glm::vec2(0.f,  1.f), zeroAim, inputseqtests::kDeadzone).has_value());
}

// ---------------------------------------------------------------------------
// angularDistance — wrap correctly across +-pi.
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.AngularDistance.Wraps", "[InputSequence]")
{
    using namespace inputSequence;
    constexpr float eps = 1e-5f;

    REQUIRE(std::abs(angularDistance(0.f, 0.f) - 0.f) < eps);
    REQUIRE(std::abs(angularDistance(0.f, pi / 2.f) - pi / 2.f) < eps);
    REQUIRE(std::abs(angularDistance(pi, -pi) - 0.f) < eps);                  // wrap
    REQUIRE(std::abs(angularDistance(3.f * pi / 4.f, -3.f * pi / 4.f) - pi / 2.f) < eps); // shortest arc
}

// ---------------------------------------------------------------------------
// matchSequence — full Hadouken sequence (Back -> DownBack -> Down) returns
// kHadoukenActionId.
//
// currentTick=10, windowAfterFinalStep=6, maxGap=8:
//   tick 9 -> Down     (in [3, 9])
//   tick 7 -> DownBack (in [0, 8])
//   tick 5 -> Back     (in [-2, 6])
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.MatchSequence.HadoukenMatches", "[InputSequence]")
{
    using namespace inputSequence;
    using namespace inputseqtests;

    const glm::vec3 aim(1.f, 0.f, 0.f);
    const uint32_t currentTick = 10;

    std::map<uint32_t, dAttackMachineSimulation::PlayerInput> history;
    history[5] = makeInput(glm::vec2(-1.f,  0.f), aim); // Back
    history[7] = makeInput(glm::vec2(-1.f, -1.f), aim); // DownBack
    history[9] = makeInput(glm::vec2( 0.f, -1.f), aim); // Down

    auto accessor = [&](uint32_t tick) -> const dAttackMachineSimulation::PlayerInput*
    {
        auto it = history.find(tick);
        return (it != history.end()) ? &it->second : nullptr;
    };

    const std::vector<MotionCommand> defs{ makeHadouken() };

    const uint32_t result = matchSequence(
        accessor,
        currentTick,
        glm::vec2(0.f, -1.f),
        glm::vec3(aim),
        0b01,
        0b01,
        kDeadzone,
        defs);

    REQUIRE(result == kHadoukenActionId);
}

// ---------------------------------------------------------------------------
// matchSequence — maxGapFrames violation: the gap between DownBack (tick 5)
// and Down (tick 9) is 4 > maxGap=2.
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.MatchSequence.MaxGapFramesViolation", "[InputSequence]")
{
    using namespace inputSequence;
    using namespace inputseqtests;

    const glm::vec3 aim(1.f, 0.f, 0.f);
    const uint32_t currentTick = 10;

    std::map<uint32_t, dAttackMachineSimulation::PlayerInput> history;
    history[3] = makeInput(glm::vec2(-1.f,  0.f), aim); // Back
    history[5] = makeInput(glm::vec2(-1.f, -1.f), aim); // DownBack (gap=4 from Down at 9)
    history[9] = makeInput(glm::vec2( 0.f, -1.f), aim); // Down

    auto accessor = [&](uint32_t tick) -> const dAttackMachineSimulation::PlayerInput*
    {
        auto it = history.find(tick);
        return (it != history.end()) ? &it->second : nullptr;
    };

    const std::vector<MotionCommand> defs{ makeHadouken(/*maxGap=*/2) };

    const uint32_t result = matchSequence(
        accessor,
        currentTick,
        glm::vec2(0.f, -1.f),
        glm::vec3(aim),
        0b01,
        0b01,
        kDeadzone,
        defs);

    REQUIRE(result == kNoMatch);
}

// ---------------------------------------------------------------------------
// matchSequence — angle tolerance: with a tight tolerance (pi/16 = 11.25°),
// a stick slightly off the target angle does NOT match.
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.MatchSequence.TightToleranceRejects", "[InputSequence]")
{
    using namespace inputSequence;
    using namespace inputseqtests;

    const glm::vec3 aim(1.f, 0.f, 0.f);

    // Stick at ~30° from aim — well outside a pi/16 tolerance around Forward (0).
    auto inputAt30Deg = makeInput(glm::vec2(0.866f, -0.5f), aim);

    auto accessor = [&](uint32_t) -> const dAttackMachineSimulation::PlayerInput* { return &inputAt30Deg; };

    // Single-step motion: target Forward, tolerance pi/16.
    MotionCommand cmd{
        { MotionStep{ angle::Forward, pi / 16.f, 8 } },
        0b01,
        6,
        kHadoukenActionId
    };

    const uint32_t result = matchSequence(
        accessor, 10u, glm::vec2(0.866f, -0.5f), aim,
        0b01, 0b01, kDeadzone, { cmd });

    REQUIRE(result == kNoMatch);
}

// ---------------------------------------------------------------------------
// matchSequence — kNoMatch when accessor always returns nullptr (cold cache).
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.MatchSequence.ColdCacheReturnsNoMatch", "[InputSequence]")
{
    using namespace inputSequence;
    using namespace inputseqtests;

    const glm::vec3 aim(1.f, 0.f, 0.f);

    auto nullAccessor = [](uint32_t) -> const dAttackMachineSimulation::PlayerInput* { return nullptr; };

    const std::vector<MotionCommand> defs{ makeHadouken() };

    const uint32_t result = matchSequence(
        nullAccessor, 10u, glm::vec2(0.f, -1.f), aim,
        0b01, 0b01, kDeadzone, defs);

    REQUIRE(result == kNoMatch);
}

// ---------------------------------------------------------------------------
// matchSequence — kNoMatch when buffer entries are all below deadzone
// (aimRelativeAngle returns nullopt -> step never matches).
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.MatchSequence.NeverMatchingBufferReturnsNoMatch", "[InputSequence]")
{
    using namespace inputSequence;
    using namespace inputseqtests;

    const glm::vec3 aim(1.f, 0.f, 0.f);

    dAttackMachineSimulation::PlayerInput neutral = makeInput(glm::vec2(0.f, 0.f), aim);
    auto accessor = [&](uint32_t) -> const dAttackMachineSimulation::PlayerInput* { return &neutral; };

    const std::vector<MotionCommand> defs{ makeHadouken() };

    const uint32_t result = matchSequence(
        accessor, 10u, glm::vec2(0.f, 0.f), aim,
        0b01, 0b01, kDeadzone, defs);

    REQUIRE(result == kNoMatch);
}

// ---------------------------------------------------------------------------
// matchSequence — tie-break: longer step-count wins.
//
// Two motions both ending with a Down step:
//   short: [Down]             (1 step, actionId=2)
//   long:  [DownBack, Down]   (2 steps, actionId=3)
// Buffer satisfies both. The 2-step command must win.
// ---------------------------------------------------------------------------
TEST_CASE("InputSequence.MatchSequence.TieBreakLongerWins", "[InputSequence]")
{
    using namespace inputSequence;
    using namespace inputseqtests;

    const glm::vec3 aim(1.f, 0.f, 0.f);

    std::map<uint32_t, dAttackMachineSimulation::PlayerInput> history;
    history[7] = makeInput(glm::vec2(-1.f, -1.f), aim); // DownBack
    history[9] = makeInput(glm::vec2( 0.f, -1.f), aim); // Down

    auto accessor = [&](uint32_t tick) -> const dAttackMachineSimulation::PlayerInput*
    {
        auto it = history.find(tick);
        return (it != history.end()) ? &it->second : nullptr;
    };

    MotionCommand shortMotion{
        { MotionStep{ angle::Down, pi / 8.f, 8 } },
        0b01,
        6,
        2u
    };
    MotionCommand longMotion{
        {
            MotionStep{ angle::DownBack, pi / 8.f, 8 },
            MotionStep{ angle::Down,     pi / 8.f, 8 },
        },
        0b01,
        6,
        3u
    };

    const std::vector<MotionCommand> defs{ shortMotion, longMotion };

    const uint32_t result = matchSequence(
        accessor, 10u, glm::vec2(0.f, -1.f), aim,
        0b01, 0b01, kDeadzone, defs);

    REQUIRE(result == 3u);
}

#endif // WITH_LOW_LEVEL_TESTS

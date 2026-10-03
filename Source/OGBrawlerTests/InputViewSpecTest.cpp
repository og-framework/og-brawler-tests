// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <type_traits>

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerInputPackaging.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGBrawler/InputSequence/InputSequence.h"

namespace inputViewSpecTest
{
namespace machine    = dAttackMachineSimulation;
namespace guard      = dAttackGuardSimulation;
namespace movement   = brawlerMovementSimulation;
namespace radial     = dAttackRadialSimulation;
namespace projectile = brawlerProjectileSimulation;
namespace ringout    = brawlerRingout;

struct Buttons
{
    bool left  = false;
    bool right = false;
};

const glm::vec3 kAims[] = {
    glm::vec3(0.f, 0.f, 1.f),
    glm::vec3(1.f, 0.f, 0.f),
    glm::vec3(0.6f, 0.8f, 0.f),
    glm::vec3(-0.28f, 0.96f, 0.f),
};

const Buttons kButtons[] = {
    Buttons{ false, false },
    Buttons{ false, true },
    Buttons{ true, false },
    Buttons{ true, true },
};

const glm::vec2 kSticks[] = {
    glm::vec2(0.f, 0.f),
    glm::vec2(0.3f, -0.7f),
    glm::vec2(1.f, 0.f),
};

const glm::vec3 kWorlds[] = {
    glm::vec3(0.f, 0.f, 0.f),
    glm::vec3(0.6f, -0.8f, 0.f),
    glm::vec3(1.f, 0.f, 0.f),
};

const std::uint32_t kTriggers[] = {
    inputSequence::kNoMatch,
    inputSequence::kHadoukenActionId,
};

const bool kHoldGuards[] = { false, true };

simulatableBrawler::ContinuousInputFields fieldsOf(const glm::vec3& aim, const glm::vec2& stick, const glm::vec3& world)
{
    return simulatableBrawler::ContinuousInputFields{
        .aimDirection       = aim,
        .moveStick          = stick,
        .moveDirectionWorld = world };
}

TEST_CASE("SimulatableBrawler.InputViewSpec.EveryViewFieldIsItsPackerArgument",
          "[SimulatableBrawler][InputViewSpec]")
{
    std::uint32_t rows = 0u;
    for (const glm::vec3& aim : kAims)
    for (const Buttons& buttons : kButtons)
    for (const glm::vec2& stick : kSticks)
    for (const glm::vec3& world : kWorlds)
    for (const std::uint32_t trigger : kTriggers)
    for (const bool holdGuard : kHoldGuards)
    {
        const auto packed = simulatableBrawler::makeSimPlayerInput(
            fieldsOf(aim, stick, world), buttons.left, buttons.right, trigger,
            simulatableBrawler::InputFlagFields{ .holdGuard = holdGuard });

        INFO("row " << rows << " aim=(" << aim.x << "," << aim.y << "," << aim.z << ")"
             << " L=" << buttons.left << " R=" << buttons.right
             << " stick=(" << stick.x << "," << stick.y << ")"
             << " world=(" << world.x << "," << world.y << "," << world.z << ")"
             << " trigger=" << trigger << " holdGuard=" << holdGuard);

        const machine::PlayerInputView machineView = machine::PlayerInputView::from(packed);
        CHECK(machineView.aimDirection == aim);
        CHECK(machineView.attackLeft == buttons.left);
        CHECK(machineView.attackRight == buttons.right);
        CHECK(machineView.moveDirection == stick);
        CHECK(machineView.moveDirectionWorld == world);
        CHECK(machineView.triggeredActionId == trigger);

        CHECK(guard::PlayerInputView::from(packed).aimDirection == aim);

        const movement::PlayerInputView movementView = movement::PlayerInputView::from(packed);
        CHECK(movementView.flags == (holdGuard ? movement::kInputFlagHoldGuard : std::uint8_t{ 0u }));
        CHECK(movementView.moveDirectionWorld == world);

        ++rows;
    }
    REQUIRE(rows == 576u);
}

TEST_CASE("SimulatableBrawler.InputViewSpec.NeutralInputViews",
          "[SimulatableBrawler][InputViewSpec]")
{
    const auto zero = simulatableBrawler::getZeroPlayerInput();

    const machine::PlayerInputView machineView = machine::PlayerInputView::from(zero);
    REQUIRE(machineView.aimDirection == glm::vec3(0.f, 0.f, 1.f));
    REQUIRE(machineView.attackLeft == false);
    REQUIRE(machineView.attackRight == false);
    REQUIRE(machineView.moveDirection == glm::vec2(0.f, 0.f));
    REQUIRE(machineView.moveDirectionWorld == glm::vec3(0.f, 0.f, 0.f));
    REQUIRE(machineView.triggeredActionId == 0u);

    REQUIRE(guard::PlayerInputView::from(zero).aimDirection == glm::vec3(0.f, 0.f, 1.f));

    const movement::PlayerInputView movementView = movement::PlayerInputView::from(zero);
    REQUIRE(movementView.flags == 0u);
    REQUIRE(movementView.moveDirectionWorld == glm::vec3(0.f, 0.f, 0.f));
}

TEST_CASE("SimulatableBrawler.InputViewSpec.RenderRateInputViews",
          "[SimulatableBrawler][InputViewSpec]")
{
    std::uint32_t rows = 0u;
    for (const glm::vec3& aim : kAims)
    for (const glm::vec2& stick : kSticks)
    for (const glm::vec3& world : kWorlds)
    {
        const auto echoed = simulatableBrawler::makeVisualizationPlayerInput(fieldsOf(aim, stick, world));

        INFO("row " << rows << " aim=(" << aim.x << "," << aim.y << "," << aim.z << ")"
             << " stick=(" << stick.x << "," << stick.y << ")"
             << " world=(" << world.x << "," << world.y << "," << world.z << ")");

        const machine::PlayerInputView machineView = machine::PlayerInputView::from(echoed);
        CHECK(machineView.aimDirection == aim);
        CHECK(machineView.attackLeft == false);
        CHECK(machineView.attackRight == false);
        CHECK(machineView.moveDirection == stick);
        CHECK(machineView.moveDirectionWorld == world);
        CHECK(machineView.triggeredActionId == inputSequence::kNoMatch);

        CHECK(guard::PlayerInputView::from(echoed).aimDirection == aim);

        const movement::PlayerInputView movementView = movement::PlayerInputView::from(echoed);
        CHECK(movementView.flags == 0u);
        CHECK(movementView.moveDirectionWorld == world);

        ++rows;
    }
    REQUIRE(rows == 36u);
}

TEST_CASE("SimulatableBrawler.InputViewSpec.SimsThatReadNoInputHaveEmptyViews",
          "[SimulatableBrawler][InputViewSpec]")
{
    STATIC_REQUIRE(std::is_empty_v<radial::PlayerInputView>);
    STATIC_REQUIRE(std::is_empty_v<projectile::PlayerInputView>);
    STATIC_REQUIRE(std::is_empty_v<ringout::PlayerInputView>);
}

} // namespace inputViewSpecTest

#endif // WITH_LOW_LEVEL_TESTS

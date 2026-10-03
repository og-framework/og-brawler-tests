// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackMachineSimulationRuntimeTweakables.h"
#include "OGBrawler/InputMapping/StickRouting.h"

#include "glm/geometric.hpp"

#include <iterator>

namespace fullLengthMoveTest
{
using dAttackMachineSimulation::MovementScheme;
using dInput::stickRouting::StickSources;
using dInput::stickRouting::routeSticks;

constexpr float kMoveDeadzone = 0.15f;

enum class MoveStick
{
	Left,
	Right
};

struct RoutingCase
{
	MovementScheme scheme;
	bool legacySwap;
	bool feedsAim;
	MoveStick moveStick;
};

const RoutingCase kCases[] = {
	{ MovementScheme::CameraRelative, false, true, MoveStick::Left },
	{ MovementScheme::AimRelative, false, true, MoveStick::Left },
	{ MovementScheme::MoveRelativeAim, false, true, MoveStick::Left },
	{ MovementScheme::CameraRelative, false, false, MoveStick::Left },
	{ MovementScheme::AimRelative, false, false, MoveStick::Left },
	{ MovementScheme::MoveRelativeAim, false, false, MoveStick::Left },
	{ MovementScheme::CameraRelative, true, true, MoveStick::Right },
	{ MovementScheme::AimRelative, true, true, MoveStick::Right },
	{ MovementScheme::MoveRelativeAim, true, false, MoveStick::Right },
	{ MovementScheme::AimRelativeSwapped, false, true, MoveStick::Right },
	{ MovementScheme::AimRelativeSwapped, false, false, MoveStick::Right },
	{ MovementScheme::AimRelativeSwapped, true, true, MoveStick::Right },
	{ MovementScheme::AimRelativeSwapped, false, true, MoveStick::Left },
};

const glm::vec2 kDirections[] = {
	{ 0.f, -1.f }, { 1.f, 0.f }, { -1.f, 0.f }, { 0.f, 1.f }, { 0.6f, -0.8f }, { -0.8f, 0.6f }, { -0.28f, -0.96f },
};

StickSources onMoveStick(const RoutingCase& routingCase, const glm::vec2& stick)
{
	if (routingCase.moveStick == MoveStick::Left)
		return StickSources{ { 0.f, 0.f }, stick, { 0.f, 0.f } };
	return StickSources{ { 0.f, 0.f }, { 0.f, 0.f }, stick };
}

glm::vec2 routedMove(const RoutingCase& routingCase, const glm::vec2& stick)
{
	return routeSticks(onMoveStick(routingCase, stick), routingCase.scheme, routingCase.legacySwap,
		routingCase.feedsAim, kMoveDeadzone)
		.move;
}

TEST_CASE("FullLengthMove: a half and a full stick route to the same direction above the deadzone",
	"[SimulatableBrawler][FullLengthMove]")
{
	int combosChecked = 0;
	for (const RoutingCase& routingCase : kCases)
	{
		for (const glm::vec2& direction : kDirections)
		{
			INFO("scheme " << static_cast<int>(routingCase.scheme) << " swap " << routingCase.legacySwap
				<< " feedsAim " << routingCase.feedsAim << " direction (" << direction.x << ", " << direction.y << ")");
			const glm::vec2 fullMove = routedMove(routingCase, direction);
			const glm::vec2 halfMove = routedMove(routingCase, 0.5f * direction);
			const glm::vec2 subDeadzoneMove = routedMove(routingCase, 0.1f * direction);

			REQUIRE(glm::length(fullMove) >= kMoveDeadzone);
			REQUIRE(glm::length(halfMove) >= kMoveDeadzone);
			const glm::vec2 fullDirection = glm::normalize(fullMove);
			const glm::vec2 halfDirection = glm::normalize(halfMove);
			CHECK(halfDirection.x == Catch::Approx(fullDirection.x).margin(1e-6f));
			CHECK(halfDirection.y == Catch::Approx(fullDirection.y).margin(1e-6f));
			CHECK(fullDirection.x == Catch::Approx(direction.x).margin(1e-6f));
			CHECK(fullDirection.y == Catch::Approx(direction.y).margin(1e-6f));
			CHECK(glm::length(subDeadzoneMove) < kMoveDeadzone);
			++combosChecked;
		}
	}
	CHECK(combosChecked == static_cast<int>(std::size(kCases) * std::size(kDirections)));
}
} // namespace fullLengthMoveTest

#endif // WITH_LOW_LEVEL_TESTS

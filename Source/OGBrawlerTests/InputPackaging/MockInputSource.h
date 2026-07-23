#pragma once
// SPDX-License-Identifier: BUSL-1.1

#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

// Test double for UOGBrawlerInputCollectionComponent's read surface.
//
// simulatableBrawler::readContinuousInputFields is templated on its source precisely
// so this type can substitute for the UE component: it exposes the same three
// accessors and nothing else. If the component's continuous-read surface ever grows
// a field, readContinuousInputFields must grow with it, and this mock stops compiling
// — which is the intended failure mode.
//
// The discrete members (leftAttack / rightAttack / triggeredActionId) mirror what the
// real component also holds. readContinuousInputFields deliberately cannot see them;
// the sim-path tests pass them explicitly to makeSimPlayerInput, exactly as
// buildPlayerInput does.
struct MockInputSource
{
	glm::vec3 aimDirection       = glm::vec3(0.f, 0.f, 1.f);
	glm::vec2 moveStick          = glm::vec2(0.f, 0.f);
	glm::vec3 moveDirectionWorld = glm::vec3(0.f, 0.f, 0.f);

	bool     leftAttack        = false;
	bool     rightAttack       = false;
	uint32_t triggeredActionId = 0;

	// Counts calls through the continuous-read surface, so a test can prove a
	// re-sample actually re-read the source rather than returning a cached value.
	mutable int readCount = 0;

	glm::vec3 buildAimDirection() const      { ++readCount; return aimDirection; }
	glm::vec2 getMoveStick() const           { return moveStick; }
	glm::vec3 buildMoveDirectionWorld() const { return moveDirectionWorld; }
};

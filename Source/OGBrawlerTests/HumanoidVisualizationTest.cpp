// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGBrawler/CharacterVisualizationData.h"
#include <glm/gtc/quaternion.hpp>

TEST_CASE("CharacterViz.buildDefault.PositiveRadiiOnAllParts", "[CharacterViz]")
{
    HumanoidVisualization viz = humanoidVisualizationDefaults::buildDefault();

    REQUIRE(viz.head.radii.x  > 0.f);
    REQUIRE(viz.head.radii.y  > 0.f);
    REQUIRE(viz.head.radii.z  > 0.f);

    REQUIRE(viz.torso.radii.x > 0.f);
    REQUIRE(viz.torso.radii.y > 0.f);
    REQUIRE(viz.torso.radii.z > 0.f);

    REQUIRE(viz.legs.radii.x  > 0.f);
    REQUIRE(viz.legs.radii.y  > 0.f);
    REQUIRE(viz.legs.radii.z  > 0.f);
}

TEST_CASE("CharacterViz.buildDefault.HeadAboveTorsoAboveLegsAlongZ", "[CharacterViz]")
{
    HumanoidVisualization viz = humanoidVisualizationDefaults::buildDefault();

    REQUIRE(viz.head.localOffset.z  > viz.torso.localOffset.z);
    REQUIRE(viz.torso.localOffset.z > viz.legs.localOffset.z);
}

TEST_CASE("CharacterViz.buildDefault.AllRotationsUnitLength", "[CharacterViz]")
{
    HumanoidVisualization viz = humanoidVisualizationDefaults::buildDefault();

    REQUIRE(glm::length(viz.head.localRotation)  == Catch::Approx(1.f));
    REQUIRE(glm::length(viz.torso.localRotation) == Catch::Approx(1.f));
    REQUIRE(glm::length(viz.legs.localRotation)  == Catch::Approx(1.f));
}

#endif // WITH_LOW_LEVEL_TESTS

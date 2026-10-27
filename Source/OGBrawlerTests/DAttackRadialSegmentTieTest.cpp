// SPDX-License-Identifier: BUSL-1.1
// docs/DAttackRadialSegmentTieTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGBrawler/DAttackRadialSequence.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"

#include "glm/vec2.hpp"

#include <bit>
#include <cstdint>
#include <initializer_list>

namespace dAttackRadialSegmentTieTest
{
void requireParallelDirectionIsSegmentZero(unsigned int sequenceId, bool expectDescending)
{
    const simulatableBrawler::StaticData staticData;
    REQUIRE(sequenceId < staticData.m_attackSequences.size());
    const DAttackRadialSequence& sequence = staticData.m_attackSequences[sequenceId];

    const DAttackSegment firstSegment = sequence.getAttackSegment(0u);
    REQUIRE((firstSegment.endAngle < firstSegment.startAngle) == expectDescending);

    const glm::vec2 initialDirection = sequence.getDirection(0u);
    const glm::vec2 parallelDirection = 2.f * initialDirection;
    const DAttackSegment segment = sequence.getAttackSegment(parallelDirection);
    INFO("sequence " << sequenceId << " initialAngle=" << sequence.getInitialAngle()
         << " parallelDirection=(" << parallelDirection.x << ", " << parallelDirection.y << ") bits=(0x"
         << std::hex << std::bit_cast<std::uint32_t>(parallelDirection.x) << ", 0x"
         << std::bit_cast<std::uint32_t>(parallelDirection.y) << std::dec << ")"
         << " segment.index=" << segment.index << " noSegment=" << sequence.getAttackPointCount());
    CHECK(segment.index == 0u);
    CHECK(segment.state == sequence.getDAttackRadialSequenceState(0u));
    CHECK(segment.startAngle == firstSegment.startAngle);
    CHECK(segment.endAngle == firstSegment.endAngle);
}
} // namespace dAttackRadialSegmentTieTest

TEST_CASE("DAttack.RadialSegmentTie.ParallelDirectionIsSegmentZeroOnDescendingSequences", "[DAttack][RadialSegmentTie]")
{
    for (const unsigned int sequenceId : { 0u, 2u, 4u })
    {
        dAttackRadialSegmentTieTest::requireParallelDirectionIsSegmentZero(sequenceId, true);
    }
}

TEST_CASE("DAttack.RadialSegmentTie.ParallelDirectionIsSegmentZeroOnAscendingSequences", "[DAttack][RadialSegmentTie]")
{
    for (const unsigned int sequenceId : { 1u, 3u })
    {
        dAttackRadialSegmentTieTest::requireParallelDirectionIsSegmentZero(sequenceId, false);
    }
}

#endif

// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <set>
#include <type_traits>

#include "catch_amalgamated.hpp"

#include "OGBrawler/SimCharacterId.h"

TEST_CASE("SimCharacterId.TheTypeIsOneByteAndDistinctFromTheStorageKey", "[SimulatableBrawler][SimCharacterId]")
{
    STATIC_REQUIRE(sizeof(SimCharacterId) == 1u);
    STATIC_REQUIRE(std::is_same_v<std::underlying_type_t<SimCharacterId>, std::uint8_t>);
    STATIC_REQUIRE_FALSE(std::is_convertible_v<unsigned int, SimCharacterId>);
    STATIC_REQUIRE_FALSE(std::is_convertible_v<SimCharacterId, unsigned int>);
    STATIC_REQUIRE(toStorageKey(SimCharacterId::None) == 0u);
    STATIC_REQUIRE(SimCharacterIdAllocator::kLastAssignable == 255u);
}

TEST_CASE("SimCharacterId.TheAllocatorCannotBeResetOrDuplicated", "[SimulatableBrawler][SimCharacterId]")
{
    STATIC_REQUIRE_FALSE(std::is_copy_assignable_v<SimCharacterIdAllocator>);
    STATIC_REQUIRE_FALSE(std::is_move_assignable_v<SimCharacterIdAllocator>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<SimCharacterIdAllocator>);
    STATIC_REQUIRE_FALSE(std::is_move_constructible_v<SimCharacterIdAllocator>);
}

TEST_CASE("SimCharacterId.AssignsOneThrough255InOrderAndRefusesThe256th", "[SimulatableBrawler][SimCharacterId]")
{
    SimCharacterIdAllocator allocator;
    REQUIRE(allocator.assignedCount() == 0u);
    REQUIRE_FALSE(allocator.isExhausted());

    for (unsigned int expected = 1u; expected <= 255u; ++expected)
    {
        const SimCharacterId id = allocator.allocate();
        REQUIRE(id != SimCharacterId::None);
        REQUIRE(toStorageKey(id) == expected);
    }
    REQUIRE(allocator.assignedCount() == 255u);
    REQUIRE(allocator.isExhausted());

    REQUIRE(allocator.allocate() == SimCharacterId::None);
    REQUIRE(allocator.allocate() == SimCharacterId::None);
    REQUIRE(allocator.assignedCount() == 255u);
}

TEST_CASE("SimCharacterId.NoIdIsReusedAcrossRegisterAndUnregister", "[SimulatableBrawler][SimCharacterId]")
{
    SimCharacterIdAllocator allocator;
    std::set<unsigned int> registered;
    std::set<unsigned int> everIssued;

    for (int round = 0; round < 100; ++round)
    {
        const SimCharacterId joined = allocator.allocate();
        REQUIRE(joined != SimCharacterId::None);
        REQUIRE(everIssued.insert(toStorageKey(joined)).second);
        registered.insert(toStorageKey(joined));

        if (round % 3 == 2)
            registered.erase(registered.begin());
    }

    registered.clear();
    const SimCharacterId afterEveryoneLeft = allocator.allocate();
    REQUIRE(toStorageKey(afterEveryoneLeft) == 101u);
    REQUIRE(everIssued.count(toStorageKey(afterEveryoneLeft)) == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS

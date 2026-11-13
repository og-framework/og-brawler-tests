// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGBrawler/BrawlerRingoutRespawnSystem.h"
#include "OGBrawler/BrawlerRingoutSimulation.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationSerialization.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StorageView.h"
#include "OGSimulation/SystemRoleAffinity.h"
#include "OGSimulation/SystemsExecutor.h"

// [og-input-latency task 2d, 2026-11-11] brawlerRingout::RespawnSlotSystem: the authority moves a
// respawn off an occupied spawn point. Rationale: BrawlerRingoutRespawnSystem-rationale.md.

namespace ringoutRespawnSlotTests
{

namespace ringout = brawlerRingout;
namespace movement = brawlerMovementSimulation;

constexpr unsigned int kRigCharacters = 4u;

struct FRespawnRig
{
    using Exec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>,
                                           simulatableBrawler::StaticData,
                                           ringout::RespawnSlotSystem>;

    simulatableBrawler::StaticData              staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    Exec                                        exec;

    // Character i holds slot i and stands on spawn point i, settled 0.45 m below it.
    explicit FRespawnRig(bool ascendingIds = true)
    {
        for (unsigned int i = 0u; i < kRigCharacters; ++i)
        {
            const unsigned int id = ascendingIds ? i : kRigCharacters - 1u - i;
            this->storage.add<SimulatableBrawler>(id, SimulatableBrawler(this->staticData));
            this->exec.notifyCharacterRegistered(id, this->storage, this->staticData, /*isAuthority*/ true);
        }
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
        {
            slotOf(id) = id;
            standAt(id, point(id));
        }
    }

    const glm::vec3& point(uint32_t slot) const { return this->staticData.m_ringoutStaticData.spawnPoints[slot]; }

    SimulatableBrawler& brawler(unsigned int id) { return this->storage.get<SimulatableBrawler>(id); }

    uint32_t& slotOf(unsigned int id)
    {
        return brawler(id).editAllState().editState().edit<ringout::InitialConditions>().spawnSlot;
    }

    void standAt(unsigned int id, const glm::vec3& where)
    {
        auto& state = brawler(id).editAllState().editState();
        state.edit<ringout::State>().flags = 0u;
        state.edit<movement::State>().bodyState.position = where + simFrame::withHeight(glm::vec3(0.f), -0.45f);
        state.edit<movement::InitialConditions>().teleportPending = 0u;
    }

    void poseDueToRespawn(unsigned int id, uint32_t respawnAtTick)
    {
        auto& state = brawler(id).editAllState().editState();
        auto& ringoutState = state.edit<ringout::State>();
        ringoutState.flags = static_cast<uint8_t>(ringoutState.flags | ringout::kFlagDead);
        ringoutState.respawnAtTick = respawnAtTick;
        state.edit<movement::State>().bodyState.position = simFrame::withHeight(glm::vec3(0.f), -40.f);
        brawler(id).editAllState().editDerivedState().edit<ringout::DerivedState>().diedThisTick = false;
    }

    void poseDeathEdge(unsigned int id)
    {
        poseDueToRespawn(id, 1000u);
        brawler(id).editAllState().editDerivedState().edit<ringout::DerivedState>().diedThisTick = true;
    }

    void pre(uint32_t tick, bool isAuthority = true)
    {
        const SimulationTimeStep s(tick, false, StepKind::Normal);
        this->exec.firePreIntegrate(s, this->storage, this->staticData, isAuthority);
    }

    void post(uint32_t tick, bool isAuthority = true)
    {
        const SimulationTimeStep s(tick, false, StepKind::Normal);
        this->exec.firePostIntegrate(s, this->storage, this->staticData, isAuthority);
    }
};

TEST_CASE("RingoutRespawnSlot.SatisfiesTheSystemConceptAndIsAuthorityOnly", "[BrawlerRingout]")
{
    STATIC_REQUIRE(SimulationSystem<ringout::RespawnSlotSystem, simulatableBrawler::StaticData>);
    STATIC_REQUIRE(ringout::RespawnSlotSystem::kRoleAffinity == SystemRoleAffinity::AuthorityOnly);
    STATIC_REQUIRE_FALSE(Serializable<ringout::RespawnSlotSystem>);
}

TEST_CASE("RingoutRespawnSlot.ADueRespawnOntoAnOccupiedPointMovesToTheFreeSlot", "[BrawlerRingout]")
{
    FRespawnRig rig;
    rig.poseDueToRespawn(0u, 100u);
    rig.standAt(3u, rig.point(0u));

    rig.pre(99u);
    REQUIRE(rig.slotOf(0u) == 0u);

    // Slots 1 and 2 hold bodies, slot 0 holds id 3, so slot 3 is the lowest free one.
    rig.pre(100u);
    REQUIRE(rig.slotOf(0u) == 3u);
    REQUIRE(rig.slotOf(1u) == 1u);
    REQUIRE(rig.slotOf(2u) == 2u);
    REQUIRE(rig.slotOf(3u) == 3u);
}

TEST_CASE("RingoutRespawnSlot.ADueRespawnOntoAFreePointKeepsItsSlot", "[BrawlerRingout]")
{
    FRespawnRig rig;
    rig.poseDueToRespawn(2u, 100u);
    rig.pre(100u);
    REQUIRE(rig.slotOf(2u) == 2u);
}

TEST_CASE("RingoutRespawnSlot.TheDeathTickAlreadyMovesTheSlot", "[BrawlerRingout]")
{
    FRespawnRig rig;
    rig.poseDeathEdge(1u);
    rig.standAt(0u, rig.point(1u));

    rig.post(50u);
    REQUIRE(rig.slotOf(1u) == 0u);
}

TEST_CASE("RingoutRespawnSlot.APendingTeleportOccupiesItsTarget", "[BrawlerRingout]")
{
    FRespawnRig rig;
    rig.poseDueToRespawn(0u, 100u);
    rig.standAt(3u, rig.point(5u));
    auto& teleport = rig.brawler(3u).editAllState().editState().edit<movement::InitialConditions>();
    teleport.teleportPending = 1u;
    teleport.teleportPos = rig.point(0u);

    rig.pre(100u);
    REQUIRE(rig.slotOf(0u) == 3u);
}

TEST_CASE("RingoutRespawnSlot.ADeadCharactersSlotIsReserved", "[BrawlerRingout]")
{
    FRespawnRig rig;
    rig.poseDueToRespawn(0u, 100u);
    rig.standAt(1u, rig.point(0u));
    rig.standAt(2u, rig.point(1u));
    rig.poseDueToRespawn(3u, 200u);
    rig.slotOf(3u) = 2u;

    // Slots 0 and 1 hold bodies. Slot 2 holds none, but dead id 3 will respawn there, so the
    // lowest free slot is 3. Without the reservation it would be 2.
    rig.pre(100u);
    REQUIRE(rig.slotOf(0u) == 3u);
}

TEST_CASE("RingoutRespawnSlot.TwoSimultaneousRespawnsDoNotShareAFreeSlot", "[BrawlerRingout]")
{
    for (const bool ascending : { true, false })
    {
        INFO("ids added in " << (ascending ? "ascending" : "descending") << " order");
        FRespawnRig rig(ascending);
        rig.poseDueToRespawn(0u, 100u);
        rig.poseDueToRespawn(1u, 100u);
        rig.standAt(2u, rig.point(0u));
        rig.standAt(3u, rig.point(1u));

        // Id 0 chooses first: slots 0 and 1 hold bodies, so slot 2. Id 1 then sees slot 2
        // reserved by id 0, so slot 3.
        rig.pre(100u);
        REQUIRE(rig.slotOf(0u) == 2u);
        REQUIRE(rig.slotOf(1u) == 3u);
    }
}

TEST_CASE("RingoutRespawnSlot.ANonAuthorityRoleChoosesNothing", "[BrawlerRingout]")
{
    FRespawnRig rig;
    rig.poseDueToRespawn(0u, 100u);
    rig.standAt(3u, rig.point(0u));
    rig.poseDeathEdge(1u);
    rig.standAt(2u, rig.point(1u));

    rig.pre(100u, /*isAuthority*/ false);
    rig.post(100u, /*isAuthority*/ false);
    REQUIRE(rig.slotOf(0u) == 0u);
    REQUIRE(rig.slotOf(1u) == 1u);

    rig.pre(100u, /*isAuthority*/ true);
    REQUIRE(rig.slotOf(0u) != 0u);
}

} // namespace ringoutRespawnSlotTests

#endif // WITH_LOW_LEVEL_TESTS

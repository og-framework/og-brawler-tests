// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include <cmath>
#include <cstdint>
// [ringout task 7] The resim rig at the bottom of this file needs a byte buffer it can poison
// and compare (`<vector>` / `<cstring>`) and a place to name a divergence offset (`<sstream>`
// is NOT used - Catch's INFO streams directly).
#include <cstring>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGBrawler/BrawlerRingoutSimulation.h"
#include "OGBrawler/BrawlerMovementSimulation.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationDependencies.h"
#include "OGSimulation/SimulationSerialization.h"
#include "OGBrawler/DAttackMachineSimulation.h"
// [ringout task 7] THE FULL COMPOSITE AND THE REAL WIRE CODEC. Everything above this line
// drives `brawlerRingout::integrate` through a LOCAL four-type composite; the resim cases
// cannot, because the property under test is what a CORRECTION does to the whole
// `simulatableBrawler::State` - and that struct, its codec and `SimulatableBrawler::integrate`
// are the three things a hand-built rig would have to fake.
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/SpatialQueryResult.h"

// ---------------------------------------------------------------------------
// brawlerRingout — the kill-plane / respawn sub-simulation [ringout task 1].
//
// THE SUB-SIM IS NOT WIRED INTO THE COMPOSITE YET (that is task 2), so every case below
// drives it through a LOCAL `SimulationComposite` holding exactly the four types its
// `Dependencies` names. That is the same rig `BrawlerMovementSimulationTest.cpp` uses, and
// it is deliberately the REAL `makeDependencies` path rather than hand-built references:
// the owned/external split and the const-ness of the movement State read are then checked
// by the compiler on every case rather than asserted in prose.
//
// ⛔ THE COMPOSITE HOLDS COPIES. `SimulationComposite`'s constructor takes its arguments by
// value, so `Rig::tick` has to copy the results back out afterwards. A case that forgets
// that reads the PRE-tick value and passes for the wrong reason.
//
// TIME. There is no delta anywhere in this file, because the sub-sim is handed none: the
// respawn countdown is `tick >= respawnAtTick` against absolute sim ticks, and
// `IntegrationUtils` carries nothing but the tick.
// ---------------------------------------------------------------------------

namespace ringouttests
{

namespace ringout  = brawlerRingout;
namespace movement = brawlerMovementSimulation;

struct Rig
{
    ringout::StaticData        sd{};
    ringout::InitialConditions ic{};
    ringout::State             state{};
    ringout::DerivedState      derived{};

    // The two movement slices this sub-sim reaches through `ExternalDeps`: the State it READS
    // (const) and the InitialConditions it WRITES.
    movement::InitialConditions movementIc{};
    movement::State             movementState{};

    // Park the body at a world Z. Nothing else about the movement state matters to ring-out —
    // it reads exactly one float.
    void setBodyZ(float z) { movementState.bodyState.position.z = z; }

    float aboveThePlane() const { return sd.killPlaneZ + 100.f; }
    float belowThePlane() const { return sd.killPlaneZ - 100.f; }

    bool dead() const { return ringout::isDead(state); }

    // One ring-out step at the given absolute sim tick.
    void tick(std::uint32_t t)
    {
        SimulationComposite<ringout::InitialConditions,
                            ringout::State,
                            movement::InitialConditions,
                            movement::State> composite(ic, state, movementIc, movementState);
        auto deps = makeDependencies<ringout::Dependencies>(composite);

        ringout::PlayerInput     pi    = ringout::PlayerInput::zero();
        ringout::IntegrationUtils utils{ t };
        ringout::AllInput         allInput{ pi, utils };

        ringout::integrate(allInput, sd, deps, derived);

        ic            = composite.get<ringout::InitialConditions>();
        state         = composite.get<ringout::State>();
        movementIc    = composite.get<movement::InitialConditions>();
        movementState = composite.get<movement::State>();
    }

    // What the movement sub-sim would consume LATER IN THE SAME TICK. Ring-out integrates
    // BEFORE movement, so a seed written on tick T is spent on tick T.
    void consumeTeleportAsMovementWould()
    {
        if (movementIc.teleportPending != 0u)
        {
            movementState.bodyState.position = movementIc.teleportPos;
            movementState.velocity           = glm::vec3(0.f);
            movementIc.teleportPending       = 0u;
        }
    }
};

// ===========================================================================
// ⛔⛔ THE EXECUTION-ORDER EDGE, MACHINE-CHECKED — ringout BEFORE movement.
//
// `BrawlerRingoutSimulation.h` argues in prose that ring-out must integrate before the
// movement sub-sim, and that the dependency validator agrees because the HARD edge (the
// non-const `InitialConditions&` write) beats the SOFT edge (the const `State&` read) when
// both point at the same sub-sim. Prose is not a check. This is the check, and it is here
// rather than in `SimulatableBrawlerTypes.h` because task 1 does not touch that file.
//
// THE STUB. `brawlerMovementSimulation::Dependencies` reads `const machine::State&`, so a
// two-element order would report that as an UNOWNED external ref and both arms below would
// fail for a reason that has nothing to do with ring-out. `MachineStateOwner` stands in for
// whichever sub-sim owns that type in the real `ExecutionOrder`; it closes the graph and
// nothing else.
// ===========================================================================

namespace machine = dAttackMachineSimulation;

struct MachineStateOwner
{
    using Owned      = OwnedDeps<machine::State>;
    using External   = ExternalDeps<>;
    using InputType  = machine::PlayerInput;
    Owned    owned;
    External external;
};

using RingoutBeforeMovement = std::tuple<MachineStateOwner,
                                         ringout::Dependencies,
                                         movement::Dependencies>;
using MovementBeforeRingout = std::tuple<MachineStateOwner,
                                         movement::Dependencies,
                                         ringout::Dependencies>;

inline constexpr auto kGoodOrderViolation = compositeDetail::findFirstViolation<RingoutBeforeMovement>();
inline constexpr auto kBadOrderViolation  = compositeDetail::findFirstViolation<MovementBeforeRingout>();

static_assert(kGoodOrderViolation.kind == compositeDetail::ViolationKind::None,
    "brawlerRingout::Dependencies - ringout BEFORE movement must validate. Ring-out declares "
    "BOTH a const read of brawlerMovementSimulation::State and a non-const write of its "
    "InitialConditions; the read alone would demand movement run first. This asserts the "
    "library resolves that pair the way the header claims it does.");

// THE POISON ARM. Without it the assertion above is satisfied by a validator that accepts
// everything.
static_assert(kBadOrderViolation.kind == compositeDetail::ViolationKind::ExecutionOrderViolation,
    "brawlerRingout::Dependencies - movement BEFORE ringout must FAIL to validate. If this "
    "ever passes, the teleport seed ring-out writes is no longer guaranteed to be consumed in "
    "the same tick, and a respawn re-enters the kill plane on the tick after it clears.");

// ===========================================================================
// The wire slices and the dead bit, re-quoted at the test that reads them.
// ===========================================================================

TEST_CASE("Ringout.ExecutionOrder.RingoutIntegratesBeforeMovement", "[BrawlerRingout]")
{
    // The two static_asserts above, re-quoted as a runnable case so the property is visible
    // in the suite rather than only in a build that did not break.
    REQUIRE(kGoodOrderViolation.kind == compositeDetail::ViolationKind::None);
    REQUIRE(kBadOrderViolation.kind == compositeDetail::ViolationKind::ExecutionOrderViolation);

    // The violating pair is named, so a future break says WHICH two sims disagreed: index 2
    // (ring-out, the writer) wanted to run before index 1 (movement, the owner).
    REQUIRE(kBadOrderViolation.simA == 2u);
    REQUIRE(kBadOrderViolation.simB == 1u);
}

TEST_CASE("Ringout.Types.WireSlicesAndTheDeadBit", "[BrawlerRingout]")
{
    // The two slices task 2 adds to `simulatableBrawler::State`. Their SUM is the +9 B that
    // task 2 has to find on the composite total; pinning them here says WHICH of the two moved
    // when the total moves.
    REQUIRE(syncSize<ringout::InitialConditions>() == 4u);
    REQUIRE(syncSize<ringout::State>() == 5u);

    // ⛔ THE INPUT COSTS NOTHING, and it must stay that way. An input byte is multiplied
    // across every entry of every relayed input ring — roughly ten times what a state byte
    // costs. Ring-out carries no per-tick player signal: death is positional and respawn is a
    // tick countdown.
    REQUIRE(syncSize<ringout::PlayerInput>() == 0u);

    // Bit 0, on the wire and in the checksum.
    REQUIRE(ringout::kFlagDead == 1u);

    // Sized against `ASimulationManagerUImpl::kPreDietCharacterCap`.
    REQUIRE(ringout::kMaxSpawnPoints == 4u);

    // A fresh character is alive, with no countdown armed.
    const ringout::State fresh{};
    REQUIRE_FALSE(ringout::isDead(fresh));
    REQUIRE(fresh.respawnAtTick == 0u);

    // The authored defaults are DECLARED ONCE, in StaticData's constructor, and this is the
    // one place they are re-read. 120 ticks is 2.0 s at the 60 Hz fixed step.
    const ringout::StaticData sd{};
    REQUIRE(sd.respawnDelayTicks == 120u);
    REQUIRE(sd.spawnPoints.size() == ringout::kMaxSpawnPoints);
}

// ===========================================================================
// (a) Above the plane, nothing happens — however long you wait.
// ===========================================================================

TEST_CASE("Ringout.AboveThePlaneDoesNotDie", "[BrawlerRingout]")
{
    Rig rig;
    rig.setBodyZ(rig.aboveThePlane());

    for (std::uint32_t t = 1u; t <= 10u; ++t)
    {
        rig.tick(t);
        INFO("tick " << t);
        REQUIRE_FALSE(rig.dead());
        REQUIRE_FALSE(rig.derived.diedThisTick);
        REQUIRE(rig.state.respawnAtTick == 0u);
        // Ring-out wrote nothing to the movement sub-sim.
        REQUIRE(rig.movementIc.teleportPending == 0u);
    }

    // THE BOUNDARY, and it is STRICT. `position.z < killPlaneZ` — standing exactly on the
    // plane is alive. A `<=` here would kill a character resting at the authored height on a
    // level whose floor happens to sit there.
    rig.setBodyZ(rig.sd.killPlaneZ);
    rig.tick(11u);
    REQUIRE_FALSE(rig.dead());
    REQUIRE_FALSE(rig.derived.diedThisTick);

    // DECOY CONTROL for the boundary above: one float lower and it does die, so the
    // assertion is measuring the comparison rather than a sub-sim that never kills anybody.
    rig.setBodyZ(std::nextafterf(rig.sd.killPlaneZ, -1000.f));
    rig.tick(12u);
    REQUIRE(rig.dead());
    REQUIRE(rig.derived.diedThisTick);
}

// ===========================================================================
// (b) Crossing the plane kills EXACTLY ONCE and arms the countdown.
// ===========================================================================

TEST_CASE("Ringout.CrossingThePlaneKillsOnceAndArmsTheCountdown", "[BrawlerRingout]")
{
    Rig rig;
    rig.setBodyZ(rig.aboveThePlane());
    rig.tick(100u);
    REQUIRE_FALSE(rig.dead());

    // THE CROSSING.
    rig.setBodyZ(rig.belowThePlane());
    rig.tick(101u);

    REQUIRE(rig.dead());
    REQUIRE(rig.derived.diedThisTick);
    // The countdown is stored as the ABSOLUTE tick to respawn at, derived from the tick the
    // death was detected on plus the authored delay.
    REQUIRE(rig.state.respawnAtTick == 101u + rig.sd.respawnDelayTicks);

    // ⭐ THE EDGE IS ONE TICK WIDE. `diedThisTick` is what the authority-side score system
    // reads; if it stayed true while the character lay dead, the same death would score again
    // on every tick until the respawn.
    rig.tick(102u);
    REQUIRE(rig.dead());
    REQUIRE_FALSE(rig.derived.diedThisTick);
    REQUIRE(rig.state.respawnAtTick == 101u + rig.sd.respawnDelayTicks);
}

// ===========================================================================
// (c) At `respawnAtTick`: the dead bit clears and the movement teleport seed is written
//     with THIS character's authored spawn point.
// ===========================================================================

TEST_CASE("Ringout.RespawnWritesTheTeleportSeedAndClearsDead", "[BrawlerRingout]")
{
    Rig rig;
    // A slot that is neither 0 nor the last entry, so an off-by-one in either direction shows.
    rig.ic.spawnSlot = 2u;

    rig.setBodyZ(rig.belowThePlane());
    rig.tick(100u);
    REQUIRE(rig.dead());

    const std::uint32_t respawnAt = rig.state.respawnAtTick;
    REQUIRE(respawnAt == 100u + rig.sd.respawnDelayTicks);

    // THE TICK BEFORE. Still dead, still no seed — the comparison is `>=`, so the boundary is
    // worth one assertion of its own.
    rig.tick(respawnAt - 1u);
    REQUIRE(rig.dead());
    REQUIRE(rig.movementIc.teleportPending == 0u);

    // THE RESPAWN TICK.
    rig.tick(respawnAt);

    REQUIRE_FALSE(rig.dead());
    REQUIRE(rig.movementIc.teleportPending == 1u);
    REQUIRE(rig.movementIc.teleportPos == rig.sd.spawnPoints[2]);

    // ⭐ AND IT DID NOT IMMEDIATELY RE-DIE. The position ring-out can see on the respawn tick
    // is STILL the below-the-plane position — the movement sub-sim has not consumed the seed
    // yet, because it integrates after ring-out within the same tick. The `else if` between
    // the two arms is what makes this hold; a plain `if` would re-arm the countdown on the
    // very tick it expired.
    REQUIRE_FALSE(rig.derived.diedThisTick);
    REQUIRE(rig.movementState.bodyState.position.z < rig.sd.killPlaneZ);

    // Now let the movement sub-sim do what it does later in this same tick, and step on. The
    // character is on its spawn point and stays alive.
    rig.consumeTeleportAsMovementWould();
    REQUIRE(rig.movementState.bodyState.position == rig.sd.spawnPoints[2]);

    rig.tick(respawnAt + 1u);
    REQUIRE_FALSE(rig.dead());
    REQUIRE_FALSE(rig.derived.diedThisTick);
    REQUIRE(rig.movementIc.teleportPending == 0u);
}

// ===========================================================================
// (d) A dead character that keeps falling does not re-die and does not re-arm.
// ===========================================================================

TEST_CASE("Ringout.DeadAndStillFallingDoesNotReDieOrReArm", "[BrawlerRingout]")
{
    Rig rig;
    rig.setBodyZ(rig.belowThePlane());
    rig.tick(50u);
    REQUIRE(rig.dead());
    REQUIRE(rig.derived.diedThisTick);

    const std::uint32_t armedAt = rig.state.respawnAtTick;

    // Keep falling, well past where a second detection would land.
    float z = rig.belowThePlane();
    for (std::uint32_t t = 51u; t <= 60u; ++t)
    {
        z -= 250.f;
        rig.setBodyZ(z);
        rig.tick(t);

        INFO("tick " << t << " z=" << z);
        REQUIRE(rig.dead());
        REQUIRE_FALSE(rig.derived.diedThisTick);
        // ⛔ THE COUNTDOWN DID NOT MOVE. A re-arm here is the defect that makes the respawn
        // never arrive: each tick of the fall would push the deadline one tick further out.
        REQUIRE(rig.state.respawnAtTick == armedAt);
        REQUIRE(rig.movementIc.teleportPending == 0u);
    }
}

// ===========================================================================
// The defensive index — a slot past the end of the authored table.
// ===========================================================================

TEST_CASE("Ringout.SpawnSlotOutOfRangeWritesNoTeleportSeed", "[BrawlerRingout]")
{
    Rig rig;
    // The authority assigns this; a value past the end is an assignment bug, and it must not
    // also be an out-of-bounds read. This case exists BECAUSE the guard is a runtime branch
    // rather than an `OG_CHECK` — an assert would take the process down here and the recovery
    // arm would be code no test could reach.
    rig.ic.spawnSlot = ringout::kMaxSpawnPoints;

    rig.setBodyZ(rig.belowThePlane());
    rig.tick(10u);
    REQUIRE(rig.dead());

    rig.tick(rig.state.respawnAtTick);

    // The dead bit still clears — the character is not left permanently dead by a bad slot.
    REQUIRE_FALSE(rig.dead());
    // But nothing was written to the movement sub-sim, and nothing was read out of range.
    REQUIRE(rig.movementIc.teleportPending == 0u);
    REQUIRE(rig.movementIc.teleportPos == glm::vec3(0.f));

    // DECOY CONTROL: the LAST VALID slot, on the same path, does write the seed. Without this
    // the case above is satisfied by a sub-sim that never writes a teleport seed at all.
    Rig ok;
    ok.ic.spawnSlot = ringout::kMaxSpawnPoints - 1u;
    ok.setBodyZ(ok.belowThePlane());
    ok.tick(10u);
    ok.tick(ok.state.respawnAtTick);
    REQUIRE(ok.movementIc.teleportPending == 1u);
    REQUIRE(ok.movementIc.teleportPos
            == ok.sd.spawnPoints[ringout::kMaxSpawnPoints - 1u]);
}

// ===========================================================================
// A PENDING TELEPORT SUPPRESSES DETECTION — the belt-and-braces term in the death arm.
// ===========================================================================

TEST_CASE("Ringout.APendingTeleportSuppressesDeathDetection", "[BrawlerRingout]")
{
    Rig rig;
    rig.setBodyZ(rig.belowThePlane());

    // A teleport is pending, so the position ring-out can see is about to be REPLACED. This
    // is the shape `SimulationManagerUImpl` leaves a character in at registration: it seeds
    // the movement teleport from the capsule pose before the movement sub-sim has integrated
    // once.
    rig.movementIc.teleportPending = 1u;
    rig.movementIc.teleportPos     = glm::vec3(0.f, 0.f, 500.f);

    rig.tick(1u);
    REQUIRE_FALSE(rig.dead());
    REQUIRE_FALSE(rig.derived.diedThisTick);
    // ⛔ AND RING-OUT DID NOT CLEAR IT. Consuming the seed is the movement sub-sim's job, and
    // exactly one consumer may clear a counter-free edge.
    REQUIRE(rig.movementIc.teleportPending == 1u);

    // DECOY CONTROL: the SAME position with no teleport pending does kill. Without this the
    // assertion above is satisfied by a kill plane that never triggers.
    rig.movementIc.teleportPending = 0u;
    rig.tick(2u);
    REQUIRE(rig.dead());
    REQUIRE(rig.derived.diedThisTick);
}


// ===========================================================================
// ⛔⛔ THE AUTHORITY'S SPAWN-SLOT TABLE (task 3).
//
// These cases exist because `SimulationManagerUImpl.cpp` — the UE module file that CALLS the
// allocator — is unreachable from this target. The task's own acceptance criteria demand a
// DEMONSTRATION of two properties, so the logic lives in the engine-free header and the UE
// layer holds nothing but the two calls. What follows is that demonstration.
//
// Character ids stand in for `UObject::GetUniqueID()`; the allocator never interprets them.
// ===========================================================================

TEST_CASE("Ringout.SpawnSlots.TwoRemoteClientsDoNotBothGetZero", "[BrawlerRingout]")
{
    // ⛔ THIS IS THE CASE THE OBVIOUS ANSWER GETS WRONG, so it states the wrong answer first.
    // `UEConnectionHandle::GetPlayerSlotForActor` is PER-WIRE: it returns which LOCAL PLAYER
    // on ONE connection owns the actor — 0 for the primary pawn, 1..N only for couch-co-op
    // siblings on the SAME machine. Two players on two different machines are each the
    // primary pawn on their own wire, so it answers 0 for BOTH of them. Indexing the authored
    // spawn table by that number drops them on the same point.
    //
    // These two constants ARE that wrong answer, written down so the case says what it rules
    // out rather than merely passing.
    constexpr std::uint32_t perWireSlotOfRemoteClientA = 0u;
    constexpr std::uint32_t perWireSlotOfRemoteClientB = 0u;
    STATIC_REQUIRE(perWireSlotOfRemoteClientA == perWireSlotOfRemoteClientB);

    // The authority's table, on a server with two remote clients connected.
    ringout::SpawnSlotAllocator allocator;
    constexpr std::uint32_t idA = 7001u;
    constexpr std::uint32_t idB = 7002u;

    const std::uint32_t slotA = allocator.acquire(idA);
    const std::uint32_t slotB = allocator.acquire(idB);

    // The property, stated directly: two concurrently-live characters never share an index.
    REQUIRE(slotA != slotB);

    // And stated against the wrong answer: B did NOT get the 0 the per-wire call would hand it.
    REQUIRE(slotB != perWireSlotOfRemoteClientB);

    // Both are indexable — neither fell into the defensive out-of-range arm.
    REQUIRE(slotA < ringout::kMaxSpawnPoints);
    REQUIRE(slotB < ringout::kMaxSpawnPoints);

    // LOWEST FREE INDEX, so arrival order and nothing else decides: first in takes 0.
    REQUIRE(slotA == 0u);
    REQUIRE(slotB == 1u);

    // A third and fourth client fill the table in the same order.
    REQUIRE(allocator.acquire(7003u) == 2u);
    REQUIRE(allocator.acquire(7004u) == 3u);

    // The table reads back what it handed out.
    REQUIRE(allocator.slotOf(idA) == 0u);
    REQUIRE(allocator.slotOf(idB) == 1u);
    // An id that never registered holds nothing.
    REQUIRE(allocator.slotOf(9999u) == ringout::SpawnSlotAllocator::kNoFreeSlot);
}

TEST_CASE("Ringout.SpawnSlots.ReleaseFreesTheIndexAndALaterJoinReusesIt", "[BrawlerRingout]")
{
    ringout::SpawnSlotAllocator allocator;

    REQUIRE(allocator.acquire(1u) == 0u);
    REQUIRE(allocator.acquire(2u) == 1u);
    REQUIRE(allocator.acquire(3u) == 2u);
    REQUIRE(allocator.acquire(4u) == 3u);

    // The character holding index 1 leaves.
    allocator.release(2u);
    REQUIRE(allocator.slotOf(2u) == ringout::SpawnSlotAllocator::kNoFreeSlot);

    // ⭐ THE RELEASED INDEX — not the next one up, and not the out-of-range value. A later
    // join reuses 1, which is what stops a long churning session from exhausting the table.
    REQUIRE(allocator.acquire(5u) == 1u);

    // ⛔ DECOY CONTROL. Without releasing first, the table is full and the NEXT join gets the
    // out-of-range value. Both assertions above are satisfied by an allocator that never
    // tracks occupancy at all; this one is not.
    REQUIRE(allocator.acquire(6u) == ringout::SpawnSlotAllocator::kNoFreeSlot);

    // The holders that never left kept their own indices throughout.
    REQUIRE(allocator.slotOf(1u) == 0u);
    REQUIRE(allocator.slotOf(3u) == 2u);
    REQUIRE(allocator.slotOf(4u) == 3u);

    // Releasing an id that holds nothing is a no-op, which is what makes the UE unregister
    // path safe to run ungated on the client role where nothing ever acquired.
    allocator.release(9999u);
    allocator.release(6u);
    REQUIRE(allocator.slotOf(1u) == 0u);
    REQUIRE(allocator.acquire(8u) == ringout::SpawnSlotAllocator::kNoFreeSlot);
}

TEST_CASE("Ringout.SpawnSlots.AcquireIsIdempotentAndDoesNotConsumeASecondEntry", "[BrawlerRingout]")
{
    ringout::SpawnSlotAllocator allocator;

    REQUIRE(allocator.acquire(42u) == 0u);
    // Registration is RETRIED until the physics bodies resolve. A second acquire for the same
    // character must return the same index and consume nothing.
    REQUIRE(allocator.acquire(42u) == 0u);
    REQUIRE(allocator.acquire(42u) == 0u);

    // The proof that nothing was consumed: three more distinct characters still fit, and the
    // fifth does not.
    REQUIRE(allocator.acquire(43u) == 1u);
    REQUIRE(allocator.acquire(44u) == 2u);
    REQUIRE(allocator.acquire(45u) == 3u);
    REQUIRE(allocator.acquire(46u) == ringout::SpawnSlotAllocator::kNoFreeSlot);
}

TEST_CASE("Ringout.SpawnSlots.AFullTableYieldsTheOutOfRangeValueNotSlotZero", "[BrawlerRingout]")
{
    // `kNoFreeSlot` IS `kMaxSpawnPoints` on purpose: written straight into `spawnSlot` it
    // lands on `integrate`'s defensive branch, which logs and writes no teleport seed. The
    // alternative — falling back to 0 — would drop the over-capacity character on top of
    // whoever legitimately holds 0, silently.
    STATIC_REQUIRE(ringout::SpawnSlotAllocator::kNoFreeSlot == ringout::kMaxSpawnPoints);

    ringout::SpawnSlotAllocator allocator;
    for (std::uint32_t i = 0u; i < ringout::kMaxSpawnPoints; ++i)
        REQUIRE(allocator.acquire(100u + i) == i);

    const std::uint32_t overflow = allocator.acquire(999u);
    REQUIRE(overflow == ringout::SpawnSlotAllocator::kNoFreeSlot);
    REQUIRE_FALSE(overflow < ringout::kMaxSpawnPoints);
    // ⛔ THE POINT: it is NOT 0. A fallback to 0 is the failure this value exists to avoid.
    REQUIRE(overflow != 0u);

    // An overflowed character was never entered in the table, so it did not displace anyone.
    REQUIRE(allocator.slotOf(999u) == ringout::SpawnSlotAllocator::kNoFreeSlot);
    REQUIRE(allocator.slotOf(100u) == 0u);
}

TEST_CASE("Ringout.SpawnSlots.TwoCharactersRespawnToDifferentAuthoredPoints", "[BrawlerRingout]")
{
    // ⭐ THE END-TO-END ARM. The four cases above test the allocator; this one spends its
    // answer on the real `integrate` and proves the number actually reaches the respawn —
    // that the seeded `spawnSlot` selects this character's OWN authored point.
    ringout::SpawnSlotAllocator allocator;

    Rig a;
    Rig b;
    a.ic.spawnSlot = allocator.acquire(7001u);
    b.ic.spawnSlot = allocator.acquire(7002u);

    for (Rig* rig : { &a, &b })
    {
        rig->setBodyZ(rig->belowThePlane());
        rig->tick(10u);
        REQUIRE(rig->dead());
        rig->tick(rig->state.respawnAtTick);
        REQUIRE_FALSE(rig->dead());
        REQUIRE(rig->movementIc.teleportPending == 1u);
    }

    REQUIRE(a.movementIc.teleportPos == a.sd.spawnPoints[0]);
    REQUIRE(b.movementIc.teleportPos == b.sd.spawnPoints[1]);

    // ⛔ THE ASSERTION THE WHOLE TASK IS FOR. Under `GetPlayerSlotForActor` both of these are
    // `spawnPoints[0]` and this line is where that defect would surface.
    REQUIRE(a.movementIc.teleportPos != b.movementIc.teleportPos);
}

// ============================================================================================
// ⭐⭐ [ringout task 9, 2026-09-13] THE SPAWN TABLE COMES FROM THE LEVEL'S PLAYER STARTS.
//
// The authored table is placeholder authoring at (±200, ±200, Z=200); the platform this mode
// ships on is elsewhere, so every respawn dropped the fighter off the map (the user's own PIE
// complaint). User ruling 2026-09-13: respawn slot N is PLAYER START N.
//
// ⛔ THE UE HALF OF THAT — the `TActorIterator<APlayerStart>` walk and the one-time write into
// `m_staticData.m_ringoutStaticData` in `BeginPlay` — lives in `SimulationManagerUImpl.cpp`,
// which this target CANNOT REACH. That is exactly why the POLICY is a pure function in
// `BrawlerRingoutSimulation.h` and these cases exist: the three properties that can go wrong
// silently (order-dependence, an empty level, a short level) are asserted here rather than
// reviewed there. See current_state's task-6b finding: no mechanical gate in this tree guards
// any claim made inside a `Source/OGBrawlerUnreal` file.
//
// ⛔⛔ AND THE ONE THAT MATTERS MOST IS THE FIRST. `TActorIterator` hands actors back in
// UNSPECIFIED order. This table is indexed by the wire-carried `spawnSlot`, so a server and a
// client disagreeing about which point is slot 0 means the client predicts every respawn to
// the wrong place — a gameplay-visible desync that breaks no fence, moves no wire byte and
// fails to compile nowhere.
// ============================================================================================

namespace spawnpoints
{

using Placements = std::vector<ringout::LevelSpawnPoint>;

// ⛔ POSITIONS DELIBERATELY ANTI-CORRELATED WITH NAMES. `Alpha` is the furthest +X, `Delta`
// the furthest −X, so a comparator that quietly sorted by position instead of by name produces
// the EXACT REVERSE of the right answer rather than something that happens to coincide with it.
// A fixture whose two candidate orderings agree cannot tell them apart.
inline ringout::LevelSpawnPoint alpha() { return { "Alpha", glm::vec3( 900.f,  10.f, 50.f) }; }
inline ringout::LevelSpawnPoint bravo() { return { "Bravo", glm::vec3( 300.f,  20.f, 51.f) }; }
inline ringout::LevelSpawnPoint chas()  { return { "Chas",  glm::vec3(-300.f,  30.f, 52.f) }; }
inline ringout::LevelSpawnPoint delta() { return { "Delta", glm::vec3(-900.f,  40.f, 53.f) }; }

// The authored table, taken from the shipped defaults rather than restated — the whole point of
// `StaticData`'s defaulted constructor is that the literals live in exactly one place.
inline std::array<glm::vec3, ringout::kMaxSpawnPoints> authored()
{
    return ringout::StaticData{}.spawnPoints;
}

// ⛔ THE NEGATIVE CONTROL, AND IT IS NOT SHIPPED CODE. This is what the UE layer would produce
// if it trusted `TActorIterator`'s order — the defect the sort exists to prevent — written out
// by hand so a case can assert that two arrival orders ARE distinguishable. Without it, "the
// two peers agreed" is a claim that would also hold if the fixture handed both peers the same
// list.
inline std::array<glm::vec3, ringout::kMaxSpawnPoints> arrivalOrderTable(
    const Placements& placements)
{
    std::array<glm::vec3, ringout::kMaxSpawnPoints> t = authored();
    const size_t n = placements.size() < static_cast<size_t>(ringout::kMaxSpawnPoints)
        ? placements.size()
        : static_cast<size_t>(ringout::kMaxSpawnPoints);
    for (size_t i = 0u; i < n; ++i)
        t[i] = placements[i].position;
    return t;
}

inline bool anySlotIsTheWorldOrigin(const std::array<glm::vec3, ringout::kMaxSpawnPoints>& t)
{
    for (const glm::vec3& p : t)
    {
        if (p == glm::vec3(0.f, 0.f, 0.f))
            return true;
    }
    return false;
}

} // namespace spawnpoints


// ============================================================================================
// ⭐⭐ CASE 1 — THE ACCEPTANCE CRITERION ITSELF: THE SORT KEY IS PEER-IDENTICAL, DEMONSTRATED.
//
// Two peers, the same four player starts, TWO DIFFERENT ARRIVAL ORDERS — which is precisely
// what `TActorIterator` is allowed to do and what One File Per Actor makes likely. The shipped
// merge must produce the same table for both.
// ============================================================================================

TEST_CASE("Ringout.SpawnPoints.ArrivalOrderDoesNotChangeTheResult", "[BrawlerRingout]")
{
    using namespace spawnpoints;

    // "The server walked its level's actor array"; "the client walked its own".
    const Placements serverOrder{ alpha(), bravo(), chas(), delta() };
    const Placements clientOrder{ delta(), bravo(), alpha(), chas() };

    // ⭐ THE CONTROL FIRST, BECAUSE IT IS WHAT MAKES THE ASSERTION BELOW MEAN ANYTHING.
    // The two orders really are distinguishable: a merge that trusted arrival order would hand
    // these two peers different tables. If this line ever passes the case has become vacuous.
    REQUIRE(arrivalOrderTable(serverOrder) != arrivalOrderTable(clientOrder));

    const auto serverTable = ringout::spawnPointsFromLevelPlacements(serverOrder, authored());
    const auto clientTable = ringout::spawnPointsFromLevelPlacements(clientOrder, authored());

    // ⛔ THE ASSERTION THE WHOLE TASK IS FOR. Broken (delete the `std::sort`, or key it on
    // anything process-local such as an `FName` table index), these two disagree and every
    // respawn on the client snaps.
    REQUIRE(serverTable == clientTable);

    // And the answer is the one the NAME order dictates, not the one either arrival order did.
    REQUIRE(serverTable[0] == alpha().position);
    REQUIRE(serverTable[1] == bravo().position);
    REQUIRE(serverTable[2] == chas().position);
    REQUIRE(serverTable[3] == delta().position);

    // A third order, to make the point that this is a property of the function and not a lucky
    // pairing: every permutation of the same set lands on the same table.
    const Placements thirdOrder{ chas(), delta(), bravo(), alpha() };
    REQUIRE(ringout::spawnPointsFromLevelPlacements(thirdOrder, authored()) == serverTable);
}


// ============================================================================================
// CASE 2 — THE KEY IS THE NAME, NOT THE POSITION AND NOT THE ARRIVAL INDEX.
//
// The fixture's positions run the OPPOSITE way to its names on purpose (see the ⛔ at their
// declaration), so a comparator that drifted onto `position.x` produces the exact reverse.
// ============================================================================================

TEST_CASE("Ringout.SpawnPoints.TheOrderingKeyIsTheActorName", "[BrawlerRingout]")
{
    using namespace spawnpoints;

    const auto table = ringout::spawnPointsFromLevelPlacements(
        Placements{ delta(), chas(), bravo(), alpha() }, authored());

    // Ascending by NAME.
    REQUIRE(table[0] == alpha().position);
    REQUIRE(table[3] == delta().position);

    // ⛔ AND DESCENDING BY X, which is what a position-keyed comparator would have produced.
    REQUIRE(table[0].x > table[1].x);
    REQUIRE(table[1].x > table[2].x);
    REQUIRE(table[2].x > table[3].x);

    // The comparator itself, exercised directly in both directions — a strict order, not just
    // one that happens to give the right answer for this input.
    REQUIRE(ringout::levelSpawnPointOrderBefore(alpha(), delta()));
    REQUIRE_FALSE(ringout::levelSpawnPointOrderBefore(delta(), alpha()));
    REQUIRE_FALSE(ringout::levelSpawnPointOrderBefore(alpha(), alpha()));

    // ⚠ BYTE-LEXICOGRAPHIC, NOT NUMERIC, and the header says so out loud because a level
    // designer will meet it: `PlayerStart_10` sorts BEFORE `PlayerStart_2`. This is pinned
    // rather than merely documented so that a future "let me make this natural-order" edit is a
    // deliberate change to a stated behaviour instead of a silent reshuffle of everyone's slots.
    const ringout::LevelSpawnPoint ten{ "PlayerStart_10", glm::vec3(1.f, 1.f, 1.f) };
    const ringout::LevelSpawnPoint two{ "PlayerStart_2",  glm::vec3(2.f, 2.f, 2.f) };
    REQUIRE(ringout::levelSpawnPointOrderBefore(ten, two));
}


// ============================================================================================
// CASE 3 — TRAP 3, FIRST HALF: ZERO PLAYER STARTS.
//
// An undressed level falls back to the authored table ENTIRELY. ⛔ Not to zeroes: a fighter
// respawning at the world origin is the same class of bug as one respawning off the platform.
// ============================================================================================

TEST_CASE("Ringout.SpawnPoints.ZeroPlayerStartsKeepsTheWholeAuthoredTable", "[BrawlerRingout]")
{
    using namespace spawnpoints;

    const auto table = ringout::spawnPointsFromLevelPlacements(Placements{}, authored());

    REQUIRE(table == authored());
    REQUIRE_FALSE(anySlotIsTheWorldOrigin(table));

    // The authored table itself has no origin entry either — the fallback is only a safe
    // fallback while that holds, so it is asserted rather than assumed.
    REQUIRE_FALSE(anySlotIsTheWorldOrigin(authored()));
}


// ============================================================================================
// CASE 4 — TRAP 3, SECOND HALF: FEWER PLAYER STARTS THAN SLOTS.
//
// Fill what the level gives, leave the rest authored. Every count from 0 to kMaxSpawnPoints is
// walked, so the boundary is covered rather than sampled.
// ============================================================================================

TEST_CASE("Ringout.SpawnPoints.FewerThanTheTableFillsWhatItCanAndLeavesTheRestAuthored",
          "[BrawlerRingout]")
{
    using namespace spawnpoints;

    const Placements all{ alpha(), bravo(), chas(), delta() };

    for (size_t count = 0u; count <= static_cast<size_t>(ringout::kMaxSpawnPoints); ++count)
    {
        INFO("player starts present: " << count);

        // ⛔ REVERSED ON THE WAY IN, so "fills the low slots" is a statement about the SORTED
        // order and not about the order the fixture happened to build the list in.
        Placements present;
        for (size_t i = count; i > 0u; --i)
            present.push_back(all[i - 1u]);

        const auto table = ringout::spawnPointsFromLevelPlacements(present, authored());

        for (size_t slot = 0u; slot < count; ++slot)
            REQUIRE(table[slot] == all[slot].position);

        for (size_t slot = count; slot < static_cast<size_t>(ringout::kMaxSpawnPoints); ++slot)
            REQUIRE(table[slot] == authored()[slot]);

        // ⛔ THE (0,0,0) FENCE, AT EVERY COUNT. This is the assertion that a `merged{}` instead
        // of `merged = authoredFallback` would trip.
        REQUIRE_FALSE(anySlotIsTheWorldOrigin(table));
    }
}


// ============================================================================================
// CASE 5 — MORE PLAYER STARTS THAN SLOTS: the first `kMaxSpawnPoints` IN KEY ORDER win.
//
// Which four is a level-design question, so the only thing worth pinning is that the answer is
// decided by the key and not by arrival — a level with six starts must not hand two peers
// different fours.
// ============================================================================================

TEST_CASE("Ringout.SpawnPoints.MoreThanTheTableKeepsTheFirstFourInKeyOrder", "[BrawlerRingout]")
{
    using namespace spawnpoints;

    const ringout::LevelSpawnPoint echo{ "Echo", glm::vec3(-1200.f, 50.f, 54.f) };
    const ringout::LevelSpawnPoint fox { "Fox",  glm::vec3(-1500.f, 60.f, 55.f) };

    const Placements serverOrder{ alpha(), bravo(), chas(), delta(), echo, fox };
    const Placements clientOrder{ fox, echo, delta(), chas(), bravo(), alpha() };

    const auto serverTable = ringout::spawnPointsFromLevelPlacements(serverOrder, authored());
    const auto clientTable = ringout::spawnPointsFromLevelPlacements(clientOrder, authored());

    REQUIRE(serverTable == clientTable);
    REQUIRE(serverTable[0] == alpha().position);
    REQUIRE(serverTable[1] == bravo().position);
    REQUIRE(serverTable[2] == chas().position);
    REQUIRE(serverTable[3] == delta().position);
    REQUIRE_FALSE(anySlotIsTheWorldOrigin(serverTable));
}


// ============================================================================================
// CASE 6 — THE TIE-BREAK IS A TOTALITY GUARD, AND IT IS REACHABLE.
//
// Two placed actors can share a name across two sublevel outers. With a NAME-ONLY comparator
// the tied pair keeps its arrival order (MSVC's `std::sort` falls back to insertion sort, which
// is stable, at N <= 32 — current_state, task 6's toolchain findings), so two peers walking the
// same level in different orders get different tables. The position tie-break closes that.
// ============================================================================================

TEST_CASE("Ringout.SpawnPoints.DuplicateNamesStillOrderDeterministically", "[BrawlerRingout]")
{
    using namespace spawnpoints;

    const ringout::LevelSpawnPoint dupLow { "Dup", glm::vec3(-700.f, 0.f, 60.f) };
    const ringout::LevelSpawnPoint dupHigh{ "Dup", glm::vec3( 700.f, 0.f, 60.f) };

    const Placements serverOrder{ dupLow, dupHigh };
    const Placements clientOrder{ dupHigh, dupLow };

    REQUIRE(arrivalOrderTable(serverOrder) != arrivalOrderTable(clientOrder));

    REQUIRE(ringout::spawnPointsFromLevelPlacements(serverOrder, authored())
         == ringout::spawnPointsFromLevelPlacements(clientOrder, authored()));

    // The comparator is a STRICT order on the tied pair — exactly one direction is true.
    REQUIRE(ringout::levelSpawnPointOrderBefore(dupLow, dupHigh)
         != ringout::levelSpawnPointOrderBefore(dupHigh, dupLow));

    // ⛔ AND IT IS A BIT-PATTERN COMPARE, NOT A FLOAT COMPARE. `-700.f` has the sign bit set, so
    // it sorts ABOVE `+700.f` — non-numeric and deliberately so (a float comparator is not a
    // strict weak ordering in the presence of a NaN, which makes `std::sort` undefined rather
    // than merely wrong). This line pins the spelling, because the "obvious fix" to it
    // reintroduces the hazard the header argues against.
    REQUIRE(ringout::levelSpawnPointOrderBefore(dupHigh, dupLow));
    REQUIRE(ringout::spawnOrderBits(-700.f) > ringout::spawnOrderBits(700.f));
}


// ============================================================================================
// ⭐ CASE 7 — THE END-TO-END ARM: a level-seeded table is what the respawn actually teleports
// to. Everything above tests the merge; this one spends its answer on the real `integrate`,
// the same way task 3's `TwoCharactersRespawnToDifferentAuthoredPoints` spends the allocator's.
//
// It is the arm that would still be green if the merge were perfect and nothing ever called it.
// ============================================================================================

TEST_CASE("Ringout.SpawnPoints.ALevelSeededTableIsWhereTheRespawnLands", "[BrawlerRingout]")
{
    using namespace spawnpoints;

    const auto levelTable = ringout::spawnPointsFromLevelPlacements(
        Placements{ delta(), bravo(), alpha(), chas() }, authored());

    // The seeded table must actually differ from the authored one, or this case would pass on
    // a manager that never read the level at all.
    REQUIRE(levelTable != authored());

    ringout::SpawnSlotAllocator allocator;
    Rig a;
    Rig b;

    // What `ASimulationManagerUImpl::BeginPlay` does once, before any integrate: overwrite the
    // authored points in `StaticData` with the level's.
    a.sd.spawnPoints = levelTable;
    b.sd.spawnPoints = levelTable;

    a.ic.spawnSlot = allocator.acquire(9001u);
    b.ic.spawnSlot = allocator.acquire(9002u);

    for (Rig* rig : { &a, &b })
    {
        rig->setBodyZ(rig->belowThePlane());
        rig->tick(10u);
        REQUIRE(rig->dead());
        rig->tick(rig->state.respawnAtTick);
        REQUIRE_FALSE(rig->dead());
        REQUIRE(rig->movementIc.teleportPending == 1u);
    }

    // ⛔ THE USER'S ORIGINAL COMPLAINT, IN ASSERTION FORM: the respawn goes to the LEVEL's
    // point, not to the placeholder (±200, ±200, 200) that dropped fighters off the platform.
    REQUIRE(a.movementIc.teleportPos == alpha().position);
    REQUIRE(b.movementIc.teleportPos == bravo().position);
    REQUIRE(a.movementIc.teleportPos != authored()[0]);
    REQUIRE(b.movementIc.teleportPos != authored()[1]);
    REQUIRE(a.movementIc.teleportPos != b.movementIc.teleportPos);
}


// ============================================================================================
// CASE 8 — ⛔ THE WIRE DID NOT MOVE. Task 9 changes AUTHORED DATA, not state.
//
// `StaticData` is not serialized and never was; the wire carries the INDEX (`spawnSlot`, 4 B),
// which is exactly why a per-level table is free. Re-quoted here as an independent witness, the
// same way `Ringout.Resim.TheResimTestsCostTheCompositeNothing` is.
// ============================================================================================

// The trait is restated rather than shared with the resim block further down this file, for the
// same reason that block restated it: an independent witness to the number is worth more than a
// second reference to one derivation of it.
template <typename T> struct FRingoutSpawnPointsCompositeWireSize;
template <typename... Ts> struct FRingoutSpawnPointsCompositeWireSize<SimulationComposite<Ts...>>
{
    static constexpr std::uint32_t value = compositeSyncSize<Ts...>();
};

TEST_CASE("Ringout.SpawnPoints.ReadingTheLevelCostsTheCompositeNothing", "[BrawlerRingout]")
{
    static_assert(FRingoutSpawnPointsCompositeWireSize<simulatableBrawler::State>::value == 338u,
        "simulatableBrawler::State moved. Task 9 adds NO state - it replaces a compiled-in "
        "spawn table with one read off the level's APlayerStart actors, and StaticData has "
        "never been on the wire. If this fires, something else added a field: re-price the "
        "fences in SimulatableBrawlerTest.cpp and RoundVsPacketBudgetTest.cpp. "
        "[movement-sim task 84, 2026-09-20] That is exactly what happened: 335 -> 339 B, "
        "dAttackMachineSimulation::State gained m_attackEndTick (4 B), an append to an EXISTING "
        "slice, and both named files were re-priced in the same diff. "
        "[og-netcode-v2-field-defects task 9, 2026-09-23] 339 -> 338 B, and not this file's task "
        "either: dAttackRadialSimulation::State lost hasHitGuard (1 B) from the middle of the "
        "composite, kWireFormatVersion 3 -> 4, headroom 37 -> 38 B.");
    REQUIRE(FRingoutSpawnPointsCompositeWireSize<simulatableBrawler::State>::value == 338u);

    // And the slice that DOES ride the wire is untouched at 4 B: the point is level data, the
    // index is network data, and task 9 moved only the first of those.
    REQUIRE(syncSize<ringout::InitialConditions>() == 4u);
    STATIC_REQUIRE_FALSE(Serializable<ringout::StaticData>);
}


// ============================================================================================
// ⭐⭐ [ringout task 7, 2026-09-13] THE PROPERTY THE WHOLE ARCHITECTURE EXISTS TO GUARANTEE:
//     A DEATH AND ITS RESPAWN REPLAY IDENTICALLY UNDER RESIMULATION.
//
// Ruling 1 put death and respawn INSIDE the simulation for exactly one reason — so a rollback
// replays them rather than re-deciding them — and put the point award OUTSIDE it because a
// score is a monotonic side effect a rewind cannot undo. Until this block existed, the inside
// half of that split had no test: tasks 1-4 proved the LAW (what `integrate` does on one tick)
// and the WIRING (that the composite reaches it), and neither of those can fail when a
// correction destroys ring-out state on its way in.
//
// ⛔⛔ THE SPECIFIC HAZARD, AND IT HAS ALREADY BITTEN THIS TREE ONCE.
// A correction does NOT restore "the serialized fields only". It restores a WHOLE STRUCT that
// was DEFAULT-CONSTRUCTED and then had the wire fields read over it:
//
//   * `SimulationReconciliation::injectCorrectionState` — `typename T::StateType state;` then
//     `buffer.readInto(state)`. Every field the codec does not write is left at its DEFAULT.
//   * `StateCorrectionCache::tryInsertingCorrectState` stores that whole struct whenever the
//     prediction disagreed (`if (!predictionWasCorrect) m_stateBuffer[i] = std::move(state)`).
//   * `SimulationReconciliation::prepareResimAll` assigns it back WHOLE —
//     `simulatable.editAllState().editState() = cache.getState(idx)`.
//
// That is precisely how movement-sim task 50's off-wire `positionCmd` became a latent
// one-resim-per-tick defect. `brawlerRingout::State::respawnAtTick` and the `kFlagDead` bit ARE
// on the wire, so they SHOULD survive — but that is the claim under test, not an assumption,
// and the cost of it being wrong is silent: a character that respawns on the wrong tick, or
// twice, or never, on the client only, and only after a correction.
//
// ⛔ THE SCORE IS DELIBERATELY ABSENT FROM EVERY ASSERTION BELOW. `brawlerRingout::ScoreSystem`
// holds its table outside sim state and is gated to the authority, and the authority NEVER
// REWINDS (`SimulationManager.h`'s authority callback table states the rewind request and the
// first resim step as "never reached" — ArchitectureFindings F1). A resim assertion about a
// score would therefore be a test of something no production path can execute. Its law is
// `BrawlerRingoutScoreSystemTest.cpp`'s, and it stays there.
//
// ══ WHAT THIS RIG IS, AND WHAT IT STANDS IN FOR ══════════════════════════════════════════════
// Modelled on `SimulatableBrawlerTest.cpp`'s `movementResim` namespace (task 50's
// `ReplayAfterAdoptionReproducesContactClamp` / `AgreeingAnchorKeepsPushOut`), because that is
// the one rig in the tree that already transcribes the correction path faithfully. Three things
// are REAL here and are the reason the cases are evidence at all:
//
//   1. the real `SimulatableBrawler::integrate` — so ring-out runs where production runs it,
//      before the movement block, under the real execution order;
//   2. the real `correctionStateBuffer::write` / `readInto` over a POISONED byte buffer — so a
//      field the codec never touches reads back as 0xCD filler, never as a legitimate-looking
//      zero the round trip smuggled through;
//   3. a DEFAULT-CONSTRUCTED destination for `readInto`, exactly as `injectCorrectionState`
//      uses — which is the whole mechanism of the hazard.
//
// ⚠ WHAT IT DOES NOT REACH, STATED PLAINLY. This is an LLT; `SimulationManager`,
// `StateCorrectionCache`, `SimulationReconciliation` and the Chaos rewind are UE-module or
// og-simulation machinery this target either cannot instantiate or would have to mock into
// meaninglessness. What is modelled by hand is the SEQUENCING those provide — adopt at the
// anchor, `firstResimStep`, replay forward — transcribed from the production functions named
// above. So these cases prove that ring-out state SURVIVES the correction round trip and
// REPLAYS deterministically; they do NOT prove that the manager schedules the resim, picks the
// anchor, or rewinds the Chaos body correctly. Those belong to task 8's PIE run.
//
// ⚠ AND THE ENGINE STEP IS THE RIG'S, NOT THE SIM'S. Nothing in `SimulatableBrawler` advances
// the body — the generic `captureBodyStatesAll` pass does, from whatever the solver produced.
// `FPeer::engineStepAndCapture` stands in for it with the simplest law that makes a character
// fall: `position += velocity * dt`. It is analytic and deterministic, which is what lets a
// replay of the same ticks be compared BYTE FOR BYTE rather than within an epsilon.
// ============================================================================================

namespace
{
namespace resim
{

// ---- Adapters. Named apart from `SimulatableBrawlerTest.cpp`'s and kept inside an anonymous
// namespace: that file's mocks are at GLOBAL scope, and two definitions of one name in two
// translation units is an ODR violation no diagnostic is obliged to report.
struct FRingoutMockPhysics
{
    glm::mat4 getBodyTransform(BodyId) const { return glm::mat4(1.f); }
    void setBodyTransform(BodyId, const glm::mat4&) {}
    void addBodyTorque(BodyId, const glm::vec3&) {}
    void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
    void setBodyLinearVelocity(BodyId, const glm::vec3&) {}
    void addBodyAcceleration(BodyId, const glm::vec3&) {}
    void addBodyVelocityChange(BodyId, const glm::vec3&) {}
    glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
    PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
};
static_assert(PhysicsBodyAdapter<FRingoutMockPhysics>,
    "FRingoutMockPhysics must satisfy PhysicsBodyAdapter.");

struct FRingoutMockQuery
{
    SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) const { return {}; }
    SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) const { return SweepHit{}; }
    void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
    void enableShape(ShapeId) {}
    void disableShape(ShapeId) {}
};
static_assert(SpatialQueryAdapter<FRingoutMockQuery>,
    "FRingoutMockQuery must satisfy SpatialQueryAdapter.");

constexpr float kDt = 1.f / 60.f;

// `FSimulationStateSyncBuffer::kBufferBytes`. Restated rather than included — the UE-side buffer
// is not reachable from this target — and the composite's FIT inside it is fenced separately, in
// `SimulatableBrawlerTest.cpp`. Nothing here depends on the capacity being tight; it only has to
// be large enough that the codec's writes land inside the vector.
constexpr std::uint32_t kStateSyncBufferBytes = 384u;

// The spawn slot every case seats its character in. Neither 0 nor the last entry, so an
// off-by-one in either direction changes the authored point the respawn lands on.
constexpr std::uint32_t kSpawnSlot = 2u;

// The fall every case starts with: 100 cm above the authored plane. At the authored
// -980 cm/s² that reaches the plane in under 30 ticks, which leaves the death comfortably
// inside a short replay window and the respawn (`respawnDelayTicks` later) inside a long one.
constexpr float kStartMarginAbovePlane = 100.f;

// Byte-addressable stand-in for `FSimulationStateSyncBuffer` — the shape
// `correctionStateBuffer`'s `Buffer` concept asks for.
struct FCorrectionProbeBuffer
{
    std::vector<std::uint8_t> bytes;

    template <typename T>
    void writeToBuffer(std::uint32_t off, const T& value)
    { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

    template <typename T>
    T readFromBuffer(std::uint32_t off) const
    { T v; std::memcpy(&v, bytes.data() + off, sizeof(T)); return v; }
};

// ⭐ THE CORRECTION PAYLOAD FOR ONE TICK, AS BYTES. This is what "byte-identical" means in every
// assertion below: not that two C++ objects compare equal under some operator, but that the two
// runs would put the SAME BYTES on the wire. It is at least as strong as `isSimilarTo` and
// deliberately weaker than comparing whole structs (which would also compare off-wire scratch a
// correction is entitled to destroy) — and it is the comparison the architectural claim is
// actually about.
//
// ⚠ DO NOT DESCRIBE `isSimilarTo` AS "THE EPSILON COMPARISON" WITHOUT MEASURING IT FIRST.
// [ringout task 7] `SimulationComparisonGlm.h` declares epsilon overloads of `isSimilarToField`
// for `glm::vec3` / `vec2` / `quat`, but `fieldwiseIsSimilarTo` calls that name DEPENDENTLY from
// inside `SimulationSerialization.h`, and under `/permissive-` (which this target compiles with)
// neither ordinary lookup at the definition point nor ADL — the overloads sit in the GLOBAL
// namespace while the arguments live in `glm` — can reach them. MEASURED on this target: a 1e-5
// difference in one `glm::vec3` component, well inside `kDefaultSimilarityEpsilon` = 1e-4, makes
// `fieldwiseIsSimilarTo` return FALSE, while a direct non-dependent call to `isSimilarToField`
// on the same pair returns TRUE. So `isSimilarTo` is currently an EXACT float comparison for
// every vector field. Reported to the lead, NOT fixed here (og-simulation, another submodule,
// and far outside a ring-out test task).
//
// POISON, NOT ZERO. A field the codec never writes would otherwise read back as a legitimate
// zero, and the round trip would pass on a hole.
std::vector<std::uint8_t> wireOf(const simulatableBrawler::State& state, std::uint32_t tick)
{
    FCorrectionProbeBuffer buffer;
    buffer.bytes.assign(kStateSyncBufferBytes, 0xCDu);
    correctionStateBuffer::write(buffer, state, tick, kNoInputCaptureTick);
    return buffer.bytes;
}

// ⭐ THE ADOPTION, THROUGH THE REAL CODEC AND NOTHING ELSE — `injectCorrectionState`
// transcribed. The DEFAULT-CONSTRUCTED destination is the load-bearing line: it is what makes
// these cases evidence that `respawnAtTick` and the dead bit RIDE THE WIRE, rather than evidence
// about a hand-written field copy.
simulatableBrawler::State adoptOverTheWire(const simulatableBrawler::State& authority,
                                           std::uint32_t tick)
{
    FCorrectionProbeBuffer buffer;
    buffer.bytes.assign(kStateSyncBufferBytes, 0xCDu);
    correctionStateBuffer::write(buffer, authority, tick, kNoInputCaptureTick);

    simulatableBrawler::State adopted;   // DEFAULT-CONSTRUCTED, exactly as production does
    std::uint32_t appliedCaptureTick = 0u;
    const std::uint32_t readTick =
        correctionStateBuffer::readInto(buffer, adopted, appliedCaptureTick);
    REQUIRE(readTick == tick);
    REQUIRE(appliedCaptureTick == kNoInputCaptureTick);
    return adopted;
}

// One tick of a run: the wire bytes, the whole state (so a later tick can be used as a
// correction anchor), and the three decoded quantities the assertions name — so a failure reads
// as "the respawn moved" rather than only as "byte 291 differs".
struct FTraceEntry
{
    std::uint32_t             tick          = 0u;
    std::vector<std::uint8_t> wire;
    simulatableBrawler::State state;
    bool                      dead          = false;
    std::uint32_t             respawnAtTick = 0u;
    glm::vec3                 position{0.f};
};

using FTrace = std::vector<FTraceEntry>;

struct FPeer
{
    simulatableBrawler::StaticData staticData;
    SimulatableBrawler             character;
    FRingoutMockPhysics            phys;
    FRingoutMockQuery              query;

    // ⚠ `simulatableBrawler::StaticData` is non-copyable and non-movable (its sub-StaticData
    // members hold references into its own siblings), so it must be a named member constructed
    // in place — which is also why this struct is neither copied nor returned by value anywhere.
    FPeer() : character(staticData)
    {
        character.setCharacterBindings({ BodyId{1u} });
        // `queryVolumeIds` is left EMPTY on purpose: the movement sub-sim then skips the support
        // sweep, the surface reads Unsupported, and the vertical law is pure authored gravity.
        // That is what makes the fall — and therefore the kill-plane crossing — a deterministic
        // function of the tick index, identical on both peers and through every replay.
    }

    simulatableBrawler::State&       state()       { return character.editAllState().editState(); }
    const simulatableBrawler::State& state() const { return character.getAllState().getState(); }

    brawlerMovementSimulation::State& movement()
    { return state().edit<brawlerMovementSimulation::State>(); }
    const brawlerMovementSimulation::State& movement() const
    { return state().get<brawlerMovementSimulation::State>(); }

    ringout::State&       ringoutState()       { return state().edit<ringout::State>(); }
    const ringout::State& ringoutState() const { return state().get<ringout::State>(); }

    ringout::InitialConditions& ringoutIc()
    { return state().edit<ringout::InitialConditions>(); }

    const ringout::StaticData& ringoutStatic() const { return staticData.m_ringoutStaticData; }

    // See the block comment above: this is the generic body-capture pass, not part of any
    // sub-simulation, and it is the rig's stand-in for the solver.
    void engineStepAndCapture()
    {
        brawlerMovementSimulation::State& s = movement();
        s.bodyState.position = s.bodyState.position + s.velocity * kDt;
    }

    void tick(std::uint32_t t)
    {
        engineStepAndCapture();
        character.integrate(SimulationTimeStep(t, false, false, false, kDt),
                            simulatableBrawler::getZeroPlayerInput(),
                            phys, query, staticData);
    }

    FTraceEntry sample(std::uint32_t t) const
    {
        FTraceEntry e;
        e.tick          = t;
        e.wire          = wireOf(state(), t);
        e.state         = state();
        e.dead          = ringout::isDead(ringoutState());
        e.respawnAtTick = ringoutState().respawnAtTick;
        e.position      = movement().bodyState.position;
        return e;
    }

    // Seat the character just above the authored kill plane, with the spawn slot the authority
    // would have handed it. The height is read from the SHIPPED plane, so retuning `killPlaneZ`
    // moves the fixture with it instead of silently starting the run below the plane.
    void seatAboveThePlane(float margin)
    {
        movement().bodyState.position = glm::vec3(0.f, 0.f, ringoutStatic().killPlaneZ + margin);
        movement().velocity           = glm::vec3(0.f);
        ringoutIc().spawnSlot         = kSpawnSlot;
    }

    void run(std::uint32_t fromTick, std::uint32_t toTick, FTrace* outTrace)
    {
        for (std::uint32_t t = fromTick; t <= toTick; ++t)
        {
            tick(t);
            if (outTrace != nullptr)
                outTrace->push_back(sample(t));
        }
    }
};

// ---- Trace queries ---------------------------------------------------------------------------

// The ticks on which the character went alive -> dead, or dead -> alive. `deadBefore` is the dead
// bit as it stood on the tick BEFORE the trace's first entry, and it is load-bearing for the
// replay traces: without it an edge on the very FIRST replayed tick — which is exactly the
// "`respawnAtTick` was lost" symptom — is invisible to the scan.
std::vector<std::uint32_t> edgeTicks(const FTrace& trace, bool deadBefore, bool toDead)
{
    std::vector<std::uint32_t> ticks;
    bool previous = deadBefore;
    for (const FTraceEntry& e : trace)
    {
        if (e.dead != previous && e.dead == toDead)
            ticks.push_back(e.tick);
        previous = e.dead;
    }
    return ticks;
}

std::vector<std::uint32_t> deathTicks(const FTrace& trace, bool deadBefore = false)
{ return edgeTicks(trace, deadBefore, /*toDead=*/true); }

std::vector<std::uint32_t> respawnTicks(const FTrace& trace, bool deadBefore = false)
{ return edgeTicks(trace, deadBefore, /*toDead=*/false); }

const FTraceEntry& entryAt(const FTrace& trace, std::uint32_t tick)
{
    for (const FTraceEntry& e : trace)
        if (e.tick == tick)
            return e;
    FAIL("no trace entry for tick " << tick);
    return trace.front();
}

FTrace sliceFrom(const FTrace& trace, std::uint32_t firstTick)
{
    FTrace out;
    for (const FTraceEntry& e : trace)
        if (e.tick >= firstTick)
            out.push_back(e);
    return out;
}

// ---- The byte comparison ---------------------------------------------------------------------

struct FTraceDiff
{
    bool          differs    = false;
    std::uint32_t tick       = 0u;
    std::size_t   byteOffset = 0u;
    int           liveByte   = 0;
    int           replayByte = 0;
};

FTraceDiff firstDifference(const FTrace& live, const FTrace& replayed)
{
    FTraceDiff diff;
    const std::size_t n = live.size() < replayed.size() ? live.size() : replayed.size();
    for (std::size_t i = 0u; i < n; ++i)
    {
        const FTraceEntry& a = live[i];
        const FTraceEntry& b = replayed[i];
        if (a.tick != b.tick)
        {
            diff.differs = true;
            diff.tick    = a.tick;
            return diff;
        }
        const std::size_t bytes = a.wire.size() < b.wire.size() ? a.wire.size() : b.wire.size();
        for (std::size_t off = 0u; off < bytes; ++off)
        {
            if (a.wire[off] != b.wire[off])
            {
                diff.differs    = true;
                diff.tick       = a.tick;
                diff.byteOffset = off;
                diff.liveByte   = static_cast<int>(a.wire[off]);
                diff.replayByte = static_cast<int>(b.wire[off]);
                return diff;
            }
        }
    }
    return diff;
}

// ONE assertion per trace comparison rather than one per tick, with the exact divergence in the
// INFO. A per-tick REQUIRE would add ~150 assertions per case and discriminate nothing further:
// the FIRST difference is the whole diagnosis, and every tick after it is downstream of it.
void requireByteIdenticalTraces(const FTrace& live, const FTrace& replayed)
{
    INFO("live trace " << live.size() << " ticks, replayed trace " << replayed.size() << " ticks");
    REQUIRE(live.size() == replayed.size());
    REQUIRE_FALSE(live.empty());

    const FTraceDiff diff = firstDifference(live, replayed);
    INFO("first divergence: tick " << diff.tick
         << ", correction-payload byte " << diff.byteOffset
         << " (live " << diff.liveByte << " vs replayed " << diff.replayByte
         << "); the state composite starts at payload offset "
         << correctionStateBuffer::kPayloadOffset);
    REQUIRE_FALSE(diff.differs);
}

// ⭐ THE ANCHOR RESTORE, in production's order and with production's operations:
//   injectCorrectionState (default-construct + readInto)  ->  prepareResimAll (assign WHOLE)
//   ->  firstResimStep.
void adoptAndPrepareResim(FPeer& client,
                          const simulatableBrawler::State& authorityAtAnchor,
                          std::uint32_t anchorTick)
{
    client.character.editAllState().editState() =
        adoptOverTheWire(authorityAtAnchor, anchorTick);
    client.character.firstResimStep(client.phys, 0);
}

} // namespace resim
} // anonymous namespace

// ============================================================================================
// THE FIXTURE CASE — everything the three resim cases assume about the fall, measured once and
// asserted here, so a retune of gravity or the kill plane fails in ONE obvious place rather than
// making three cases pass vacuously.
// ============================================================================================

TEST_CASE("Ringout.Resim.TheFallFixtureDiesAndRespawnsExactlyOnce", "[BrawlerRingout]")
{
    using namespace resim;

    FPeer peer;
    peer.seatAboveThePlane(kStartMarginAbovePlane);

    const std::uint32_t respawnDelay = peer.ringoutStatic().respawnDelayTicks;

    FTrace trace;
    peer.run(1u, 1u + respawnDelay + 60u, &trace);

    const std::vector<std::uint32_t> deaths   = deathTicks(trace);
    const std::vector<std::uint32_t> respawns = respawnTicks(trace);

    INFO("deaths=" << deaths.size() << " respawns=" << respawns.size()
         << " over " << trace.size() << " ticks");

    // ⛔ THE FIXTURE PREMISE. If gravity, the kill plane or the start margin is retuned so the
    // character never crosses the plane inside the window, every case below would pass
    // VACUOUSLY — a replay of a run in which nothing happens is trivially identical. This is the
    // assertion that stops that.
    REQUIRE(deaths.size() == 1u);
    REQUIRE(respawns.size() == 1u);

    const std::uint32_t deathTick   = deaths.front();
    const std::uint32_t respawnTick = respawns.front();

    // The countdown is the authored one, measured off the trace rather than assumed.
    REQUIRE(respawnTick == deathTick + respawnDelay);
    REQUIRE(entryAt(trace, deathTick).respawnAtTick == respawnTick);

    // The death is a genuine crossing: at or above the plane on the tick before, below it on the
    // tick the dead bit is set.
    REQUIRE(entryAt(trace, deathTick - 1u).position.z >= peer.ringoutStatic().killPlaneZ);
    REQUIRE(entryAt(trace, deathTick).position.z < peer.ringoutStatic().killPlaneZ);

    // And the respawn landed on THIS character's authored point, in the same tick — the movement
    // sub-sim consumes ring-out's teleport seed later in the very tick ring-out wrote it.
    REQUIRE(entryAt(trace, respawnTick).position
            == peer.ringoutStatic().spawnPoints[kSpawnSlot]);

    // There is room before the death for a correction anchor to sit strictly inside the run,
    // which cases 1 and 2 rely on.
    REQUIRE(deathTick > 8u);
}

// ============================================================================================
// CASE 1 — A RESIM ACROSS THE DEATH TICK REPRODUCES THE DEATH, ON THE SAME TICK, BYTE FOR BYTE.
//
// The anchor is BEFORE the crossing and the replay window spans it, so the death is DECIDED
// inside the replay rather than merely restored by it. The client mispredicted its position, so
// the correction takes the ADOPTED path — `tryInsertingCorrectState`'s `if (!predictionWasCorrect)`
// branch, the one that overwrites the slot with a default-constructed-then-readInto struct. That
// is the hazardous arm, and it is the arm all three cases here use.
// ============================================================================================

TEST_CASE("Ringout.Resim.ReplayAcrossTheDeathTickReproducesTheDeath", "[BrawlerRingout]")
{
    using namespace resim;

    FPeer authority;
    FPeer client;
    authority.seatAboveThePlane(kStartMarginAbovePlane);
    client.seatAboveThePlane(kStartMarginAbovePlane);

    constexpr std::uint32_t kLastTick = 60u;

    // ---- The authority's run: the answer every assertion below is measured against.
    FTrace live;
    authority.run(1u, kLastTick, &live);

    const std::vector<std::uint32_t> deaths = deathTicks(live);
    REQUIRE(deaths.size() == 1u);
    const std::uint32_t deathTick = deaths.front();

    // The anchor sits strictly BEFORE the crossing, so the replayed window CONTAINS the death.
    const std::uint32_t anchorTick = deathTick - 6u;
    REQUIRE(anchorTick >= 1u);
    REQUIRE_FALSE(entryAt(live, anchorTick).dead);

    // ---- The client predicts the same ticks, then mispredicts.
    client.run(1u, anchorTick, nullptr);

    const simulatableBrawler::State& authorityAtAnchor = entryAt(live, anchorTick).state;

    // A WIRE field, so the shipped `isSimilarTo` — the production resim trigger — can see the
    // disagreement. (Same technique, same reason, as `SimulationNetSyncTest.cpp`'s `divergeState`
    // and the movement rig's wall-position nudge.)
    client.movement().bodyState.position.x += 5.f;

    // THE PREMISE OF THE WHOLE CASE: the landing really does disagree, so production takes the
    // overwriting branch and the restored slot really is a default-constructed struct.
    REQUIRE_FALSE(client.state().isSimilarTo(authorityAtAnchor));

    adoptAndPrepareResim(client, authorityAtAnchor, anchorTick);

    // The adoption restored every WIRE field exactly. Asserted BEFORE the replay so a failure
    // here reads as "the wire lost it" rather than as "the replay diverged".
    REQUIRE(client.state().isSimilarTo(authorityAtAnchor));

    // ---- REPLAY the rest of the window.
    FTrace replayed;
    client.run(anchorTick + 1u, kLastTick, &replayed);

    // ⭐ THE ASSERTION: every replayed tick would put exactly the same bytes on the wire as the
    // live run put there.
    requireByteIdenticalTraces(sliceFrom(live, anchorTick + 1u), replayed);

    // ...and SPELLED OUT, so a break names the property rather than only the verdict.
    const std::vector<std::uint32_t> replayedDeaths =
        deathTicks(replayed, /*deadBefore=*/entryAt(live, anchorTick).dead);
    REQUIRE(replayedDeaths.size() == 1u);
    REQUIRE(replayedDeaths.front() == deathTick);
    REQUIRE(entryAt(replayed, deathTick).respawnAtTick
            == entryAt(live, deathTick).respawnAtTick);
    REQUIRE(ringout::isDead(client.ringoutState()));
}

// ============================================================================================
// CASE 2 — A RESIM ACROSS THE RESPAWN TICK REPRODUCES THE RESPAWN, AT THE SAME TICK AND THE SAME
//          POSITION.
//
// Same shape as case 1 with the window extended past `respawnAtTick`. The replay therefore has
// to carry the dead bit and the absolute respawn tick across the whole countdown, fire the
// respawn on exactly the tick the live run did, and land on the authored point — while every
// intermediate tick stays byte-identical.
// ============================================================================================

TEST_CASE("Ringout.Resim.ReplayAcrossTheRespawnTickReproducesTheRespawn", "[BrawlerRingout]")
{
    using namespace resim;

    FPeer authority;
    FPeer client;
    authority.seatAboveThePlane(kStartMarginAbovePlane);
    client.seatAboveThePlane(kStartMarginAbovePlane);

    const std::uint32_t lastTick = 1u + authority.ringoutStatic().respawnDelayTicks + 50u;

    FTrace live;
    authority.run(1u, lastTick, &live);

    const std::vector<std::uint32_t> deaths   = deathTicks(live);
    const std::vector<std::uint32_t> respawns = respawnTicks(live);
    REQUIRE(deaths.size() == 1u);
    REQUIRE(respawns.size() == 1u);
    const std::uint32_t deathTick   = deaths.front();
    const std::uint32_t respawnTick = respawns.front();
    REQUIRE(respawnTick < lastTick);

    const std::uint32_t anchorTick = deathTick - 6u;
    client.run(1u, anchorTick, nullptr);

    const simulatableBrawler::State& authorityAtAnchor = entryAt(live, anchorTick).state;
    client.movement().bodyState.position.x += 5.f;
    REQUIRE_FALSE(client.state().isSimilarTo(authorityAtAnchor));

    adoptAndPrepareResim(client, authorityAtAnchor, anchorTick);
    REQUIRE(client.state().isSimilarTo(authorityAtAnchor));

    FTrace replayed;
    client.run(anchorTick + 1u, lastTick, &replayed);

    requireByteIdenticalTraces(sliceFrom(live, anchorTick + 1u), replayed);

    const bool deadAtAnchor = entryAt(live, anchorTick).dead;
    const std::vector<std::uint32_t> replayedRespawns = respawnTicks(replayed, deadAtAnchor);
    REQUIRE(replayedRespawns.size() == 1u);
    REQUIRE(replayedRespawns.front() == respawnTick);

    // ⭐ THE SAME TICK **AND** THE SAME PLACE. The position is the movement sub-sim's, written
    // from ring-out's teleport seed later in the same tick — so this pair of assertions covers
    // the whole ringout -> movement hand-off surviving the replay, not just the dead bit.
    REQUIRE(entryAt(replayed, respawnTick).position
            == authority.ringoutStatic().spawnPoints[kSpawnSlot]);
    REQUIRE(entryAt(replayed, respawnTick).position == entryAt(live, respawnTick).position);
    REQUIRE_FALSE(entryAt(replayed, respawnTick).dead);
}

// ============================================================================================
// ⛔⛔ CASE 3 — THE HAZARD ITSELF: A CORRECTION LANDING **MID-COUNTDOWN**.
//
// This is the case that would have caught movement-sim task 50's defect class in this sub-sim,
// and it is the only one of the three whose anchor carries ring-out information that CANNOT be
// re-derived from anything else in the composite. Between the death tick and the respawn tick the
// entire fact "this character is dead and will respawn at tick R" lives in FIVE BYTES — `flags`
// and `respawnAtTick` — and a correction hands the client a DEFAULT-CONSTRUCTED struct with only
// the wire fields read over it. If either of those five bytes were off the wire, the client would
// be handed `flags = 0, respawnAtTick = 0` and would:
//
//   * with the dead bit lost: RE-DIE on the next replayed tick — it is still below the plane,
//     because it keeps falling while dead — and arm a NEW countdown a full delay further out;
//   * with `respawnAtTick` lost: respawn IMMEDIATELY, because `tick >= 0` holds for every tick.
//     The respawn is not delayed by the loss, it is SKIPPED.
//
// Both are silent, client-only, and only after a correction.
//
// THE DISAGREEMENT IS DELIBERATELY THE RING-OUT STATE ITSELF: this client MISSED THE DEATH.
// Cases 1 and 2 nudge the position, because what they test is replay determinism; this case
// clears the dead bit and the countdown, so the CORRECTION is the only thing that can put them
// back, and the three assertions immediately after the adoption are the direct statement that the
// codec carried them.
// ============================================================================================

TEST_CASE("Ringout.Resim.AMidCountdownCorrectionDoesNotMoveTheRespawnTick", "[BrawlerRingout]")
{
    using namespace resim;

    FPeer authority;
    FPeer client;
    authority.seatAboveThePlane(kStartMarginAbovePlane);
    client.seatAboveThePlane(kStartMarginAbovePlane);

    const std::uint32_t respawnDelay = authority.ringoutStatic().respawnDelayTicks;
    const std::uint32_t lastTick     = 1u + respawnDelay + 50u;

    FTrace live;
    authority.run(1u, lastTick, &live);

    const std::vector<std::uint32_t> deaths   = deathTicks(live);
    const std::vector<std::uint32_t> respawns = respawnTicks(live);
    REQUIRE(deaths.size() == 1u);
    REQUIRE(respawns.size() == 1u);
    const std::uint32_t deathTick   = deaths.front();
    const std::uint32_t respawnTick = respawns.front();

    // ⭐ THE ANCHOR IS STRICTLY INSIDE THE COUNTDOWN — halfway through it, so neither boundary
    // can be the reason this case passes.
    const std::uint32_t anchorTick = deathTick + respawnDelay / 2u;
    REQUIRE(anchorTick > deathTick);
    REQUIRE(anchorTick < respawnTick);
    REQUIRE(entryAt(live, anchorTick).dead);
    REQUIRE(entryAt(live, anchorTick).respawnAtTick == respawnTick);

    // ---- The client runs the same ticks, then LOSES the death.
    client.run(1u, anchorTick, nullptr);
    REQUIRE(ringout::isDead(client.ringoutState()));      // it HAD predicted it correctly...
    client.ringoutState().flags         = 0u;             // ...and now it does not have it.
    client.ringoutState().respawnAtTick = 0u;

    const simulatableBrawler::State& authorityAtAnchor = entryAt(live, anchorTick).state;
    REQUIRE_FALSE(ringout::isDead(client.ringoutState()));
    REQUIRE_FALSE(client.state().isSimilarTo(authorityAtAnchor));

    adoptAndPrepareResim(client, authorityAtAnchor, anchorTick);

    // ⭐⭐ THE THREE LINES THE WHOLE CASE EXISTS FOR — the direct statement that the correction
    // CARRIED the ring-out state across the default-constructed round trip:
    //   - the dead bit survived;
    //   - the absolute respawn tick survived, and equals the authority's;
    //   - and it is NOT the default-constructed 0 the buffer would otherwise have left behind,
    //     which is both the value an off-wire field reads back as AND the value that makes
    //     `tick >= respawnAtTick` fire a respawn on the very next replayed tick.
    REQUIRE(ringout::isDead(client.ringoutState()));
    REQUIRE(client.ringoutState().respawnAtTick == respawnTick);
    REQUIRE_FALSE(client.ringoutState().respawnAtTick == 0u);

    // ---- REPLAY the rest of the countdown and past the respawn.
    FTrace replayed;
    client.run(anchorTick + 1u, lastTick, &replayed);

    // ⛔ NOT TWICE, AND NOT NEVER: exactly ONE dead -> alive edge in the replayed window, and it
    // is on the tick the live run respawned on. `deadBefore = true` is what makes an edge on the
    // very FIRST replayed tick — the "`respawnAtTick` was lost" symptom — visible rather than
    // swallowed by the scan.
    const std::vector<std::uint32_t> replayedRespawns = respawnTicks(replayed, /*deadBefore=*/true);
    INFO("replayed respawn ticks: " << replayedRespawns.size()
         << ", first = " << (replayedRespawns.empty() ? 0u : replayedRespawns.front())
         << ", live respawn tick = " << respawnTick);
    REQUIRE(replayedRespawns.size() == 1u);
    REQUIRE(replayedRespawns.front() == respawnTick);

    // ⛔ AND IT DID NOT RE-DIE ON THE WAY — the "dead bit was lost" symptom. The second assertion
    // is what makes the first one discriminating: the character really IS below the kill plane
    // for the whole countdown, so a cleared dead bit re-arms immediately.
    REQUIRE(deathTicks(replayed, /*deadBefore=*/true).empty());
    REQUIRE(entryAt(replayed, anchorTick + 1u).position.z
            < authority.ringoutStatic().killPlaneZ);

    // ⭐ AND THE WHOLE REPLAYED TRACE IS BYTE-IDENTICAL TO THE LIVE ONE.
    requireByteIdenticalTraces(sliceFrom(live, anchorTick + 1u), replayed);

    REQUIRE(entryAt(replayed, respawnTick).position
            == authority.ringoutStatic().spawnPoints[kSpawnSlot]);
}

// ============================================================================================
// ⛔ TASK 2's COMPOSITE FENCE, RE-QUOTED UNCHANGED — this task adds NO state.
//
// Task 7 is a test task. The composite measured 335 B when task 2 wired ring-out in and must
// still measure 335 B: everything above lives in this translation unit, in no composite, with no
// `SerializableFields` specialization. If this fires as part of a resim change, something has
// been put on the wire — and the budget it would be spent against is ~14 B (ArchitectureFindings
// F13's `PacketBudget` N=3 K=2 row at 2 B per state byte), not the correction buffer's 41 B of
// headroom.
//
// The trait is restated rather than shared so this file is an INDEPENDENT witness to the number,
// exactly as `BrawlerRingoutScoreSystemTest.cpp` is.
// ============================================================================================
template <typename T> struct FRingoutResimCompositeWireSize;
template <typename... Ts> struct FRingoutResimCompositeWireSize<SimulationComposite<Ts...>>
{
    static constexpr std::uint32_t value = compositeSyncSize<Ts...>();
};

TEST_CASE("Ringout.Resim.TheResimTestsCostTheCompositeNothing", "[BrawlerRingout]")
{
    static_assert(FRingoutResimCompositeWireSize<simulatableBrawler::State>::value == 338u,
        "simulatableBrawler::State moved. Task 7 adds NO state - it is a test task. If this "
        "fires as part of a resim change, a field has been added to the composite; re-price the "
        "wire fences in SimulatableBrawlerTest.cpp and RoundVsPacketBudgetTest.cpp, and budget "
        "against ~14 B (ArchitectureFindings F13), not the buffer's 37 B of headroom. "
        "[movement-sim task 84, 2026-09-20] That is exactly what happened: 335 -> 339 B, "
        "dAttackMachineSimulation::State gained m_attackEndTick (4 B), an append to an EXISTING "
        "slice; both named files were re-priced and the headroom went 41 -> 37 B. "
        "[og-netcode-v2-field-defects task 9, 2026-09-23] 339 -> 338 B, and not this file's task "
        "either: dAttackRadialSimulation::State lost hasHitGuard (1 B) from the middle of the "
        "composite, kWireFormatVersion 3 -> 4, headroom 37 -> 38 B.");
    REQUIRE(FRingoutResimCompositeWireSize<simulatableBrawler::State>::value == 338u);

    // ⭐ AND THE FIVE BYTES CASE 3 IS ABOUT ARE STILL ON THE WIRE. This is what makes the
    // mid-countdown case's premise machine-checked rather than assumed: `respawnAtTick` and the
    // flags byte are serialized, so a correction restores them instead of defaulting them.
    REQUIRE(syncSize<ringout::State>() == 5u);
    REQUIRE(syncSize<ringout::InitialConditions>() == 4u);
    STATIC_REQUIRE(Serializable<ringout::State>);
    STATIC_REQUIRE(Serializable<ringout::InitialConditions>);

    // ⛔ AND `DerivedState` IS STILL OFF IT. `diedThisTick` is the edge the score system reads;
    // an edge that could arrive on a correction is an edge that can be scored twice, and the
    // ABSENCE of a `SerializableFields` specialization is what enforces that. `syncSize<T>() == 0`
    // would not say this — it fails to compile for an unspecialized type, and would be
    // indistinguishable from a real declared 0 B anyway (task 2, finding 7).
    STATIC_REQUIRE_FALSE(Serializable<ringout::DerivedState>);
}

} // namespace ringouttests

#endif // WITH_LOW_LEVEL_TESTS

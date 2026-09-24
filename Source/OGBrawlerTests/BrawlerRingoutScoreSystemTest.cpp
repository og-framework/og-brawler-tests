// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================================
// brawlerRingout::ScoreSystem — the authority-only point award [ringout task 4].
//
// SUBJECT: `postIntegrate`, and the two properties that cannot be read off the code by
// inspection — that a NON-AUTHORITY role awards nothing at all, and that two simultaneous
// deaths award two points to each SURVIVOR while both diers get nothing.
//
// ⛔ [ringout task 19] THE ROLE GATE IS NO LONGER IN THIS SYSTEM. `ScoreSystem` declares
// `kRoleAffinity = SystemRoleAffinity::AuthorityOnly` and `SimulationSystemsExecutor` skips it
// off the authority. So `ANonAuthorityRoleAwardsNothing` below drives the real EXECUTOR with an
// explicit role — the rig is `FExecRig`, not `FScoreRig` — because the gate it is about is now
// one branch over there. Every other case still drives the system directly: they are about the
// award's law, which did not move.
//
// ⛔ THESE CASES DRIVE THE SYSTEM THROUGH THE PRODUCTION VIEW, not a hand-built one.
// `StorageView`'s constructor is private and `SimulationObjectStorage::projectTo<>` is its only
// mint, so the rig below hands the system the SAME object the executor does — including the
// unordered-map walk order that the sort-by-id contract exists to neutralise. That is what lets
// `TwoIterationOrdersProduceIdenticalScores` mean something; a rig that fed it a vector would be
// testing a walk order that never ships. Copied wholesale from `BrawlerHitRoutingTest.cpp`,
// which is the shipped precedent for driving a `SimulationSystem` in this suite.
//
// ⛔ THE DEATH IS POSED, NOT SIMULATED. `Rig::poseDeath` writes exactly what
// `brawlerRingout::integrate` writes on a kill-plane crossing — `diedThisTick` on the off-wire
// DerivedState AND `kFlagDead` on the State — because the award reads BOTH and a pose that set
// only the edge would make every dier look alive and award it its own point. The law that
// PRODUCES that pair is `BrawlerRingoutSimulationTest.cpp`'s subject, not this file's;
// `Ringout.CrossingThePlaneKillsOnceAndArmsTheCountdown` is where the pair is checked against
// the real `integrate`.
//
// TAGS: `[BrawlerRingout]`, already in the `[@og]` whitelist (`OgTagAliases.cpp`, appended by
// task 1). These cases move the suite count because they are new cases, not because the filter
// was widened to admit them.
// ============================================================================================

#include <algorithm>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerRingoutScoreSystem.h"
#include "OGBrawler/BrawlerRingoutSimulation.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/StorageView.h"
#include "OGSimulation/SystemsExecutor.h"
#include "OGSimulation/SystemRoleAffinity.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationSerialization.h"

namespace ringoutScoreTests
{

namespace ringout = brawlerRingout;

// Four characters is not an arbitrary rig size: `ASimulationManagerUImpl::kPreDietCharacterCap`
// and `brawlerRingout::kMaxSpawnPoints` are both 4, so it is the largest LEGAL session and the
// one the award has to be right for.
constexpr unsigned int kRigCharacters = 4u;

// For the determinism case's INFO lines: the observed walk order, printable.
inline std::string joinIds(const std::vector<unsigned int>& ids)
{
    std::string out;
    for (const unsigned int id : ids)
    {
        if (!out.empty())
            out += ", ";
        out += std::to_string(id);
    }
    return out;
}

struct FScoreRig
{
    simulatableBrawler::StaticData             staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    ringout::ScoreSystem                        system;

    // `ascendingIds == false` adds the SAME four ids in the reverse order. Nothing about the
    // resulting population differs — only the order the storage's unordered_map happens to walk
    // them in, which is the whole variable `TwoIterationOrdersProduceIdenticalScores` perturbs.
    explicit FScoreRig(bool ascendingIds = true)
    {
        if (ascendingIds)
        {
            for (unsigned int id = 0u; id < kRigCharacters; ++id)
                addCharacter(id);
        }
        else
        {
            for (unsigned int i = 0u; i < kRigCharacters; ++i)
                addCharacter(kRigCharacters - 1u - i);
        }

        // ⛔ DRIVES THE SYSTEM DIRECTLY, which is the AUTHORITY's behaviour: on that role the
        // executor forwards every hook unchanged. The off-authority behaviour is not this rig's
        // to pose — it is the executor's, and `FExecRig` below is where it is driven.
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            this->system.onCharacterRegistered(id, view(), this->staticData);
    }

    void addCharacter(unsigned int id)
    {
        // No CharacterBindings: the award reads the ring-out slices and nothing else. Seeding
        // a body id here would couple this rig to a header a sibling lane holds, for nothing.
        this->storage.add<SimulatableBrawler>(id, SimulatableBrawler(this->staticData));
    }

    // The production projection — see the file banner.
    StorageView<SimulatableBrawler> view()
    { return this->storage.projectTo<SimulatableList<SimulatableBrawler>>(); }

    SimulatableBrawler& brawler(unsigned int id)
    { return this->storage.get<SimulatableBrawler>(id); }

    // What `brawlerRingout::integrate` leaves behind on the tick a character crosses the plane:
    // the one-tick EDGE and the persistent LEVEL, together.
    void poseDeath(unsigned int id)
    {
        auto& all = this->brawler(id).editAllState();
        all.editDerivedState().edit<ringout::DerivedState>().diedThisTick = true;
        auto& state = all.editState().edit<ringout::State>();
        state.flags = static_cast<uint8_t>(state.flags | ringout::kFlagDead);
        state.respawnAtTick = 999u;
    }

    // A character that died on an EARLIER tick and is still waiting out its countdown: the
    // level without the edge.
    void poseAlreadyDead(unsigned int id)
    {
        auto& all = this->brawler(id).editAllState();
        all.editDerivedState().edit<ringout::DerivedState>().diedThisTick = false;
        auto& state = all.editState().edit<ringout::State>();
        state.flags = static_cast<uint8_t>(state.flags | ringout::kFlagDead);
    }

    // `integrate` clears the edge at the top of every step, including every replayed step, so a
    // rig that drives two ticks off one pose would be posing a state the sim cannot produce.
    void clearAllDeathEdges()
    {
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            this->brawler(id).editAllState().editDerivedState()
                .edit<ringout::DerivedState>().diedThisTick = false;
    }

    void step(unsigned int tick, bool isResimulating = false,
              StepKind kind = StepKind::Normal)
    {
        const SimulationTimeStep s(tick, isResimulating, kind);
        auto v = view();
        this->system.postIntegrate(s, v, this->staticData);
    }

    // The order the PRODUCTION view actually walks this storage in. Used as a VACUITY GUARD,
    // never as an expectation: a determinism case whose two arms happen to walk identically
    // proves nothing, and this is what says so out loud.
    std::vector<unsigned int> walkOrder()
    {
        std::vector<unsigned int> ids;
        auto v = view();
        v.forEachSimulatable<SimulatableBrawler>(
            [&ids](unsigned int id, SimulatableBrawler&) { ids.push_back(id); });
        return ids;
    }

    std::vector<uint32_t> allScores() const
    {
        std::vector<uint32_t> out;
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            out.push_back(this->system.scoreOf(id));
        return out;
    }
};

// ⛔ [ringout task 19] THE EXECUTOR RIG. Same four-character population as `FScoreRig`, but
// every hook is reached through the SHIPPED `SimulationSystemsExecutor` with an explicit role.
// That is the whole point: the gate is no longer a statement inside `ScoreSystem` that a rig
// could bypass by calling the hook directly, it is a branch in the executor, and a rig that
// called `system.postIntegrate` would drive straight past it and pass against an unguarded
// build. The type alias below is the same four template arguments `SimulationManagerUImpl.h`
// instantiates, minus the sibling systems.
struct FExecRig
{
    using Exec = SimulationSystemsExecutor<SimulatableList<SimulatableBrawler>,
                                           simulatableBrawler::StaticData,
                                           ringout::ScoreSystem>;

    simulatableBrawler::StaticData              staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    Exec                                        exec;

    FExecRig()
    {
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            this->storage.add<SimulatableBrawler>(id, SimulatableBrawler(this->staticData));
    }

    const ringout::ScoreSystem& system() const { return this->exec.get<ringout::ScoreSystem>(); }

    void registerAll(bool isAuthority)
    {
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            this->exec.notifyCharacterRegistered(id, this->storage, this->staticData, isAuthority);
    }

    void unregister(unsigned int id, bool isAuthority)
    {
        this->exec.notifyCharacterUnregistered(id, this->storage, this->staticData, isAuthority);
    }

    // BOTH step hooks, in the manager's order. `preIntegrate` is a no-op in this system, but the
    // role gate is in front of all four hooks and a case about the gate must drive all four.
    void step(unsigned int tick, bool isAuthority, bool isResimulating = false,
              StepKind kind = StepKind::Normal)
    {
        const SimulationTimeStep s(tick, isResimulating, kind);
        this->exec.firePreIntegrate(s, this->storage, this->staticData, isAuthority);
        this->exec.firePostIntegrate(s, this->storage, this->staticData, isAuthority);
    }

    void poseDeath(unsigned int id)
    {
        auto& all = this->storage.get<SimulatableBrawler>(id).editAllState();
        all.editDerivedState().edit<ringout::DerivedState>().diedThisTick = true;
        auto& state = all.editState().edit<ringout::State>();
        state.flags = static_cast<uint8_t>(state.flags | ringout::kFlagDead);
        state.respawnAtTick = 999u;
    }

    std::vector<uint32_t> allScores() const
    {
        std::vector<uint32_t> out;
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            out.push_back(this->system().scoreOf(id));
        return out;
    }
};

// ============================================================================================
// THE CONCEPT. The executor's own requires-clause would reject a non-conforming system too —
// but it would reject it inside `SimulationManagerUImpl.h`, a UE module file this target cannot
// compile, so the diagnostic would surface only in an Editor build. Asserted here (and at the
// class in its own header) so it surfaces in the LLT target as well.
// ============================================================================================
TEST_CASE("RingoutScore.SatisfiesTheSimulationSystemConcept", "[BrawlerRingout]")
{
    STATIC_REQUIRE(SimulationSystem<ringout::ScoreSystem, simulatableBrawler::StaticData>);
    STATIC_REQUIRE(IsSimulatableList<ringout::ScoreSystem::RequiredSimulatables>);
    STATIC_REQUIRE(std::is_same_v<ringout::ScoreSystem::RequiredSimulatables,
                                  SimulatableList<SimulatableBrawler>>);

    // ⭐ [ringout task 19] THE COMPILE-TIME PIN OF "THIS SYSTEM IS AUTHORITY-ONLY", and it is
    // the one line the retired G-02 could only assert at run time. `kRoleAffinity` is a
    // `static constexpr` on the CLASS, so no object is needed — which is what got around the
    // `C3615` that stopped `constexpr ScoreSystem s;` from compiling.
    // ⚠ NOTHING CHECKS THAT THE VALUE IS RIGHT. This pins what the system SAYS; that saying
    // `AuthorityOnly` actually leaves it inert off the authority is
    // `ANonAuthorityRoleAwardsNothing` below, and that case is the only thing that does.
    STATIC_REQUIRE(ringout::ScoreSystem::kRoleAffinity == SystemRoleAffinity::AuthorityOnly);

    // A fresh system holds nothing. The roster is built by `onCharacterRegistered`, which off
    // the authority the executor never calls.
    const ringout::ScoreSystem fresh;
    REQUIRE(fresh.scoreEntryCount() == 0u);
    REQUIRE(fresh.scoreOf(0u) == 0u);
}

// ============================================================================================
// ⛔⛔ THE DOUBLE-COUNT DEFECT, AND AFTER TASK 19 IT IS THE EXECUTOR'S GATE THAT PREVENTS IT.
// An `AllRoles` system fires on all three roles — the authority tick, a client's forward
// PREDICTION tick, and every replayed tick of every RESIM — and `SimulationTimeStep` cannot
// separate them:
// `getIsResimulating()` is FALSE on both the authority tick and a client's prediction tick, and
// `StepKind` describes the CLOCK, not the role. Ungated, one death would be scored once on the
// authority and then again on every client, once more per replayed tick.
//
// ⛔ THIS CASE DRIVES `FExecRig`, NOT `FScoreRig`, AND THAT IS THE POINT. The gate is one
// branch in `SimulationSystemsExecutor`, in front of all four hooks, keyed off
// `ScoreSystem::kRoleAffinity`. A rig that called `system.postIntegrate(...)` directly would
// walk straight past it and stay green against a build with no gate at all. Stubbing
// `SimulationSystemsExecutor::firesOnRole` to `return true` — the executor-level form of the
// injection that retired G-01 used — turns this case red.
//
// ⭐ THE POSITIVE CONTROL IS THE HALF THAT MAKES THIS EVIDENCE. A rig that poses nothing also
// awards nothing, so the case ends by re-driving the SAME rig and the SAME posed death with
// `isAuthority=true`, and watching the points appear. Without that arm this case would pass
// against a `postIntegrate` whose body was deleted.
//
// ⚠ THE CLIENT ARM OF `UnregisterDropsTheEntryAndARejoinStartsAtZero` MOVED IN HERE (step 4).
// It was about the same property — that nothing on the client role touches the table — and it
// can only be driven through the executor now that the hook itself carries no role.
// ============================================================================================
TEST_CASE("RingoutScore.ANonAuthorityRoleAwardsNothing", "[BrawlerRingout]")
{
    FExecRig rig;

    // 0. THE LIFECYCLE HOOK. A client never even builds a roster — and now not because
    //    `onCharacterRegistered` returns early, but because the executor never calls it.
    rig.registerAll(/*isAuthority=*/false);
    REQUIRE(rig.system().scoreEntryCount() == 0u);

    rig.poseDeath(1u);

    // 1. The client's FORWARD PREDICTION tick. isResimulating == false, exactly as on the
    //    authority — this is the pair `getIsResimulating()` provably cannot tell apart.
    rig.step(100u, /*isAuthority=*/false);

    // 2. Every replayed tick of a resim. The same death is replayed from the rollback anchor;
    //    an ungated award would add a point PER REPLAYED TICK, not per death. ⚠ The executor's
    //    `OG_CHECK` tripwire is NOT reached on these twelve steps: the role gate is taken first,
    //    so an AuthorityOnly system is silent on a client replay rather than asserting.
    for (unsigned int replayTick = 100u; replayTick <= 111u; ++replayTick)
        rig.step(replayTick, /*isAuthority=*/false, /*isResimulating=*/true);

    // 3. A HardResync step, for completeness: the caches are wiped and the integrate step is
    //    treated like Normal, so it is one more shape of tick a client reaches this hook on.
    rig.step(112u, /*isAuthority=*/false, /*isResimulating=*/false, StepKind::HardResync);

    REQUIRE((rig.allScores() == std::vector<uint32_t>{ 0u, 0u, 0u, 0u }));
    REQUIRE(rig.system().scoreEntryCount() == 0u);

    // 4. THE UNREGISTER HOOK, off the authority — moved here from
    //    `UnregisterDropsTheEntryAndARejoinStartsAtZero`. Nothing on this role ever inserted, and
    //    now nothing on this role is even called, so the erase cannot be reached to be wrong.
    rig.unregister(1u, /*isAuthority=*/false);
    REQUIRE(rig.system().scoreEntryCount() == 0u);

    // ---- THE POSITIVE CONTROL ----------------------------------------------------------
    // Same rig, same posed death, role flipped. If this arm does not move, everything above is
    // passing because the rig poses nothing.
    rig.registerAll(/*isAuthority=*/true);
    REQUIRE(rig.system().scoreEntryCount() == kRigCharacters);   // seeded, this time, by a call that happened
    rig.step(113u, /*isAuthority=*/true);
    REQUIRE((rig.allScores() == std::vector<uint32_t>{ 1u, 0u, 1u, 1u }));
    REQUIRE(rig.system().scoreEntryCount() == kRigCharacters);
}

// ============================================================================================
// ⛔ RULING 3, THE SIMULTANEOUS CASE — and it is the one an award that mutates as it walks gets
// WRONG in an order-dependent way. Two die on one tick with two others alive: each SURVIVOR
// takes TWO points (one per death), and each DIER takes NONE — neither its own, nor its twin's.
//
// Ruling 7 is the second half of that and it is asserted separately below: dying is not a
// penalty, so the dier's existing score must be untouched, not reset and not decremented.
// ============================================================================================
TEST_CASE("RingoutScore.TwoSimultaneousDeathsAwardTwoToEachSurvivor", "[BrawlerRingout]")
{
    FScoreRig rig;
    REQUIRE(rig.system.scoreEntryCount() == kRigCharacters);   // registration seeded four zeroes

    rig.poseDeath(1u);
    rig.poseDeath(2u);
    rig.step(200u);

    // Survivors 0 and 3: +2 each. Diers 1 and 2: +0 each — absent from their own award AND
    // from each other's.
    REQUIRE((rig.allScores() == std::vector<uint32_t>{ 2u, 0u, 0u, 2u }));

    // And the roster did not grow: an award inserts nothing that registration did not.
    REQUIRE(rig.system.scoreEntryCount() == kRigCharacters);
}

// ============================================================================================
// RULING 7 — NO PENALTY FOR DYING. The dier's score is not zeroed, not decremented, not
// touched. Asserted against a NON-ZERO prior score, because a dier whose score was 0 before and
// 0 after is indistinguishable from a dier that was reset.
// ============================================================================================
TEST_CASE("RingoutScore.ADierKeepsTheScoreItAlreadyHad", "[BrawlerRingout]")
{
    FScoreRig rig;

    // Give character 1 two points the honest way — two ticks on which someone else died.
    rig.poseDeath(0u);
    rig.step(300u);
    rig.clearAllDeathEdges();
    rig.poseDeath(3u);
    rig.step(301u);
    rig.clearAllDeathEdges();

    // 0 and 3 are dead now; 1 and 2 scored on both ticks, and 3 kept the point it took for 0's
    // death BEFORE it died itself — which is already half of ruling 7.
    REQUIRE((rig.allScores() == std::vector<uint32_t>{ 0u, 2u, 2u, 1u }));

    // Now 1 dies. Its two points survive the death: not zeroed, not decremented, not touched.
    rig.poseDeath(1u);
    rig.step(302u);
    REQUIRE(rig.system.scoreOf(1u) == 2u);

    // ⭐ AND NOBODY SCORED FOR IT EXCEPT THE ONE FIGHTER STILL ALIVE. 0 and 3 are still waiting
    // out their own countdowns, which is the next case's subject.
    REQUIRE((rig.allScores() == std::vector<uint32_t>{ 0u, 2u, 3u, 1u }));
}

// ============================================================================================
// ⚠ "ALIVE AT THE DEATH TICK" IS A LEVEL, AND A FIGHTER WAITING OUT ITS RESPAWN IS NOT ALIVE.
//
// ⛔ THIS IS A READING OF RULING 2, NOT A TRANSCRIPTION OF IT, AND A REVIEWER SHOULD WEIGH IT.
// The ruling says "every fighter ALIVE at the death tick scores — airborne or not"; the clause
// it was written to settle was the AIRBORNE one (the lead proposed excluding doomed fallers and
// the user overruled it). It says nothing explicit about a fighter who is currently dead and
// counting down. Treating that fighter as NOT alive is the plain reading of the word, and it is
// the only reading under which `kFlagDead` means one thing: the alternative would award points
// to a corpse for a kill it could not have been present for.
//
// It costs nothing to reverse if the user rules the other way — it is the single `alive`
// expression in `postIntegrate`.
// ============================================================================================
TEST_CASE("RingoutScore.AFighterAwaitingRespawnDoesNotScore", "[BrawlerRingout]")
{
    FScoreRig rig;

    rig.poseAlreadyDead(3u);   // died some ticks ago, still counting down, no edge this tick
    rig.poseDeath(1u);         // dies now
    rig.step(400u);

    // 0 and 2 are alive and score. 1 is the dier. 3 is dead already and scores nothing.
    REQUIRE((rig.allScores() == std::vector<uint32_t>{ 1u, 0u, 1u, 0u }));

    // ⭐ THE CONTROL: clear 3's dead bit — it respawned — and the SAME pose awards it a point.
    // Without this arm, "3 scored 0" is also what you get from a rig that never reaches 3.
    rig.clearAllDeathEdges();
    auto& state3 = rig.brawler(3u).editAllState().editState().edit<ringout::State>();
    state3.flags = static_cast<uint8_t>(state3.flags & ~ringout::kFlagDead);
    rig.poseDeath(2u);
    rig.step(401u);
    REQUIRE(rig.system.scoreOf(3u) == 1u);
}

// ============================================================================================
// ⛔ WHAT THE SORT-BY-ID CONTRACT BUYS — `SystemsExecutor.h` item 81. Character order within a
// sweep is unordered-map order: unspecified, and machine-varying with REGISTRATION HISTORY. Two
// peers that registered the same four characters in a different order walk them in a different
// order, and any non-commutative per-character effect then diverges permanently, with no input
// difference to blame and nothing downstream able to repair it.
//
// ⭐⭐ THE FIRST `REQUIRE` IS A VACUITY GUARD AND IT IS THE LOAD-BEARING LINE OF THIS CASE.
// The two rigs differ ONLY in the order the four ids were added. If the container happened to
// walk them identically anyway, everything below would pass against a system with no sort, no
// snapshot and no determinism of any kind — so the case asserts that the perturbation ACTUALLY
// PERTURBED before it asserts that the result did not move.
//
// ⚠ AND BE HONEST ABOUT WHICH HALF BITES. The award is `+=` on a per-id counter, which is
// commutative, so removing the sort ALONE leaves this green (measured, task 4 impl notes §RED-2).
// What this case catches is the pairing the library contract actually warns about: an award that
// retires diers AS IT WALKS is non-commutative, and unsorted it gives a different answer per
// walk order. Both defects at once is the shipping hazard, and both at once is what turns this
// red. The snapshot half is independently pinned by
// `TwoSimultaneousDeathsAwardTwoToEachSurvivor`, which goes red on the mutate-as-you-walk award
// whether or not the sort is there.
// ============================================================================================
TEST_CASE("RingoutScore.TwoIterationOrdersProduceIdenticalScores", "[BrawlerRingout]")
{
    FScoreRig ascending(/*ascendingIds=*/true);
    FScoreRig descending(/*ascendingIds=*/false);

    const auto orderA = ascending.walkOrder();
    const auto orderB = descending.walkOrder();
    INFO("ascending-registration walk order:  " << joinIds(orderA));
    INFO("descending-registration walk order: " << joinIds(orderB));
    REQUIRE(orderA.size() == kRigCharacters);
    REQUIRE(orderB.size() == kRigCharacters);
    REQUIRE(orderA != orderB);   // ⛔ VACUITY GUARD — see the banner

    // Identical populations, identical deaths — only the walk order differs.
    for (FScoreRig* rig : { &ascending, &descending })
    {
        rig->poseDeath(1u);
        rig->poseDeath(2u);
        rig->step(500u);
    }

    REQUIRE(ascending.allScores() == descending.allScores());
    REQUIRE((ascending.allScores() == std::vector<uint32_t>{ 2u, 0u, 0u, 2u }));
}

// ============================================================================================
// THE UNREGISTER CONTRACT. Without it the table grows for the life of the session as players
// join and leave, and a recycled id inherits a stranger's points.
//
// The second arm is the one that matters: a REJOIN under the same id starts at zero rather than
// resuming, which is what "forget" has to mean.
// ============================================================================================
TEST_CASE("RingoutScore.UnregisterDropsTheEntryAndARejoinStartsAtZero", "[BrawlerRingout]")
{
    FScoreRig rig;
    rig.poseDeath(0u);
    rig.step(600u);
    REQUIRE(rig.system.scoreOf(2u) == 1u);
    REQUIRE(rig.system.hasScoreEntry(2u));
    REQUIRE(rig.system.scoreEntryCount() == kRigCharacters);

    rig.system.onCharacterUnregistered(2u, rig.view(), rig.staticData);
    REQUIRE_FALSE(rig.system.hasScoreEntry(2u));
    REQUIRE(rig.system.scoreEntryCount() == kRigCharacters - 1u);
    REQUIRE(rig.system.scoreOf(2u) == 0u);

    // Rejoin: a fresh registration under the same id is a fresh scoreboard row, not a resumed one.
    rig.system.onCharacterRegistered(2u, rig.view(), rig.staticData);
    REQUIRE(rig.system.hasScoreEntry(2u));
    REQUIRE(rig.system.scoreOf(2u) == 0u);

    // ⚠ [ringout task 19] THE CLIENT ARM MOVED to `ANonAuthorityRoleAwardsNothing` step 4. It
    // was never about the unregister contract this case pins; it was about the role, and the role
    // is now the executor's, so the arm has to be driven through the executor to mean anything.
}

// ============================================================================================
// THE COMMON CASE. Nobody dies on the overwhelming majority of ticks, and that tick must cost
// one vector walk and no sort — and, more importantly for correctness, must award NOTHING. An
// award keyed off the dead LEVEL rather than the one-tick EDGE would pay every survivor a point
// on every tick a corpse spent counting down.
// ============================================================================================
TEST_CASE("RingoutScore.ATickWithNoDeathEdgeAwardsNothing", "[BrawlerRingout]")
{
    FScoreRig rig;

    // Two characters are dead and counting down. No EDGE anywhere.
    rig.poseAlreadyDead(0u);
    rig.poseAlreadyDead(1u);
    for (unsigned int tick = 700u; tick < 760u; ++tick)
        rig.step(tick);

    REQUIRE((rig.allScores() == std::vector<uint32_t>{ 0u, 0u, 0u, 0u }));
}

// ============================================================================================
// ⛔ THE AWARD COSTS THE COMPOSITE NOTHING — task 2's fence, RE-QUOTED UNCHANGED.
//
// `simulatableBrawler::State` measured 335 B after task 2 wired the ring-out sub-simulation in.
// Task 4 adds a system, not a slice: scores live in `ScoreSystem::m_scores`, which is in no
// composite and has no `SerializableFields` specialization. If a later edit "helpfully" moves a
// score onto the wire, this assert and `DAttack.SimulatableBrawler.WireFootprint` both fire.
//
// ⚠ AND THE BUDGET THIS PROTECTS IS TIGHTER THAN THE BUFFER SUGGESTS: the `PacketBudget` N=3
// K=2 row clears by 29.352 B at 2 B per state byte, so ~14 more state bytes, NOT the correction
// buffer's 41 B of headroom (ArchitectureFindings F13). A score slice would be a real cost.
//
// The trait is the same four lines `SimulatableBrawlerTest.cpp` defines; restated here rather
// than shared so this file is an INDEPENDENT witness to the number rather than a second caller
// of the same helper.
// ============================================================================================
template <typename T> struct FScoreCompositeWireSize;
template <typename... Ts> struct FScoreCompositeWireSize<SimulationComposite<Ts...>>
{
    static constexpr std::uint32_t value = compositeSyncSize<Ts...>();
};

TEST_CASE("RingoutScore.TheAwardCostsTheCompositeNothing", "[BrawlerRingout]")
{
    static_assert(FScoreCompositeWireSize<simulatableBrawler::State>::value == 338u,
        "simulatableBrawler::State moved. Task 4 adds NO state: if this fires as part of a "
        "scoring change, a score has been put on the wire and it is now correctable, "
        "rewindable, and double-countable - which is the exact hazard ruling 1's split exists "
        "to prevent. Budget against ~14 B (F13), not the buffer's 37 B. "
        "[movement-sim task 84, 2026-09-20] 335 -> 339 B and it was NOT this file's task: "
        "dAttackMachineSimulation::State gained m_attackEndTick (4 B), an append to an EXISTING "
        "slice, and the headroom went 41 -> 37 B with it. "
        "[og-netcode-v2-field-defects task 9, 2026-09-23] 339 -> 338 B, and not this file's task "
        "either: dAttackRadialSimulation::State lost hasHitGuard (1 B) from the middle of the "
        "composite, kWireFormatVersion 3 -> 4, headroom 37 -> 38 B.");
    REQUIRE(FScoreCompositeWireSize<simulatableBrawler::State>::value == 338u);

    // The ring-out STATE slice is unchanged too: 5 B, the flags byte plus the respawn tick.
    REQUIRE(syncSize<ringout::State>() == 5u);
    REQUIRE(syncSize<ringout::InitialConditions>() == 4u);

    // And the system itself is not a serializable type at all — it has no `SerializableFields`
    // specialization, which is what "in no composite" means mechanically.
    STATIC_REQUIRE_FALSE(Serializable<ringout::ScoreSystem>);
}

} // namespace ringoutScoreTests

#endif // WITH_LOW_LEVEL_TESTS

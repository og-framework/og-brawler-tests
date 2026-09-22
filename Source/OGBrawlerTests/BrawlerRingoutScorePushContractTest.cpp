// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

// ============================================================================================
// THE SCORE PUSH'S CORE-SIDE CONTRACT [ringout task 5].
//
// WHAT THIS FILE IS FOR, AND WHY IT IS NOT IN `BrawlerRingoutScoreSystemTest.cpp`.
// Task 5 replicates the ring-out score to clients. The push that feeds that replication lives
// in `ASimulationManagerUImpl::OnPostPhysicsStep` — a UE module file this target cannot reach,
// so the replication itself is not testable here and is verified by code reading plus task 8's
// PIE run. ⛔ ONE THING THE PUSH RESTS ON *IS* REACHABLE, AND IT IS THE LOAD-BEARING ONE:
//
//   The push reads `brawlerRingout::ScoreSystem`'s score table on the GAME thread, while
//   `postIntegrate` writes it on the PHYSICS thread. The CROSSING table (the header's former
//   banner, now `SimulationManagerUImpl-rationale.md` section 0) accepts that tear for exactly one structural reason — THE TABLE CANNOT BE
//   RESTRUCTURED UNDER THE READER, because its only insert and erase are
//   `onCharacterRegistered` / `onCharacterUnregistered`, both game-thread.
//
//   ⚠ That is TRUE ONLY WHILE THE AWARD NEVER INSERTS. `postIntegrate` awards through
//   `operator[]`, which inserts — deliberately, so an award to an unseeded id is a real award
//   rather than a silently dropped one. An insert can REHASH, on the physics thread, and that
//   breaks the crossing IN KIND rather than in degree: not a stale number, a reader chasing a
//   freed bucket.
//
// ⇒ These cases pin the precondition, so the header's threading argument is machine-checked
//   rather than prose. They live in their own translation unit because the two existing
//   ring-out test files were held by other workers when task 5 was implemented — and because
//   the subject here is not the award LAW (task 4's file) or the death LAW (task 1's file) but
//   the table's STRUCTURAL STABILITY, which is a third thing.
//
// ⭐ CASE 2 IS THE ISOLATION ARM AND IT IS NOT OPTIONAL. Case 1 asserts a count does not move.
// A count that could never move would make it pass for free, so case 2 drives the SAME rig
// into the insert it claims is unreachable and watches the count move. The pair is the
// discriminator; either alone is not.
//
// TAGS: `[BrawlerRingout]`, whitelisted in `OgTagAliases.cpp` by task 1 — NO whitelist edit.
// These cases move the suite count because they are new cases, not because the filter widened.
// ============================================================================================

#include <cstdint>
#include <vector>

#include "catch_amalgamated.hpp"

#include "OGBrawler/BrawlerRingoutScoreSystem.h"
#include "OGBrawler/BrawlerRingoutSimulation.h"
#include "OGBrawler/SimulatableBrawler.h"
#include "OGBrawler/SimulatableBrawlerTypes.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/StorageView.h"
#include "OGSimulation/SimulationTimeContext.h"

namespace ringoutScorePushTests
{

namespace ringout = brawlerRingout;

// Three characters: enough for one dier and two survivors, which is the smallest population in
// which an award is non-trivial. The legal maximum is 4 (`kPreDietCharacterCap`), and the
// property under test is structural, so the size buys nothing beyond that shape.
constexpr unsigned int kRigCharacters = 3u;

// ⛔ DRIVEN THROUGH THE PRODUCTION VIEW. `StorageView`'s constructor is private and
// `SimulationObjectStorage::projectTo<>` is its only mint, so `postIntegrate` here walks the
// same container, in the same unordered-map order, that the shipped executor hands it. A rig
// that fed it a vector would be answering a question about a walk that never runs.
struct FPushRig
{
    simulatableBrawler::StaticData              staticData;
    SimulationObjectStorage<SimulatableBrawler> storage;
    ringout::ScoreSystem                        system;

    // `seedRoster == false` is the POISON CONFIGURATION case 2 needs: the characters exist and
    // can die, but `onCharacterRegistered` was never called for them, so the roster is empty
    // and the award's `operator[]` has to insert.
    explicit FPushRig(bool seedRoster)
    {
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            this->storage.add<SimulatableBrawler>(id, SimulatableBrawler(this->staticData));

        if (seedRoster)
        {
            for (unsigned int id = 0u; id < kRigCharacters; ++id)
                this->system.onCharacterRegistered(id, view(), this->staticData);
        }
    }

    StorageView<SimulatableBrawler> view()
    { return this->storage.projectTo<SimulatableList<SimulatableBrawler>>(); }

    SimulatableBrawler& brawler(unsigned int id)
    { return this->storage.get<SimulatableBrawler>(id); }

    // Exactly what `brawlerRingout::integrate` leaves behind on a kill-plane crossing: the
    // one-tick EDGE on the off-wire DerivedState and the persistent LEVEL on the State. The
    // award reads both, so posing only one would pose a state the simulation cannot produce.
    void poseDeath(unsigned int id)
    {
        auto& all = this->brawler(id).editAllState();
        all.editDerivedState().edit<ringout::DerivedState>().diedThisTick = true;
        auto& state = all.editState().edit<ringout::State>();
        state.flags        = static_cast<uint8_t>(state.flags | ringout::kFlagDead);
        state.respawnAtTick = 999u;
    }

    void step(unsigned int tick)
    {
        const SimulationTimeStep s(tick, /*isResimulating=*/false, StepKind::Normal);
        auto v = view();
        this->system.postIntegrate(s, v, this->staticData);
    }

    std::vector<uint32_t> allScores() const
    {
        std::vector<uint32_t> out;
        for (unsigned int id = 0u; id < kRigCharacters; ++id)
            out.push_back(this->system.scoreOf(id));
        return out;
    }
};

// ============================================================================================
// CASE 1 — THE PRECONDITION ITSELF.
//
// On a SEEDED roster the award reaches entries that already exist, so it cannot insert and
// cannot rehash. `scoreEntryCount()` is the observable that says so: it is the map's `size()`,
// and a `operator[]` insert is the only thing in `postIntegrate` that could move it.
// ============================================================================================
TEST_CASE("RingoutScorePush.ASeededRosterMakesTheAwardNonInserting", "[BrawlerRingout]")
{
    FPushRig rig(/*seedRoster=*/true);

    const size_t seededCount = rig.system.scoreEntryCount();
    REQUIRE(seededCount == static_cast<size_t>(kRigCharacters));

    rig.poseDeath(0u);
    rig.step(100u);

    // ⭐ THE ASSERTION THE HEADER'S CROSSING ARGUMENT RESTS ON.
    CHECK(rig.system.scoreEntryCount() == seededCount);

    // ⛔ THE VACUITY GUARD. A count that does not move because the award did nothing would pass
    // the line above for the wrong reason. The two survivors must actually have been paid.
    const std::vector<uint32_t> scores = rig.allScores();
    CHECK(scores[0] == 0u);   // the dier scores nothing — ruling 12
    CHECK(scores[1] == 1u);
    CHECK(scores[2] == 1u);
}

// ============================================================================================
// CASE 2 — THE ISOLATION ARM. Same rig, roster NOT seeded.
//
// This is the configuration the precondition rules out, driven deliberately, so that case 1's
// "the count did not move" is known to be a claim a real defect could falsify. It also documents
// what task 4 chose on purpose: the award to an unseeded id is a REAL award, not a dropped one.
// ⛔ IF THIS CASE EVER GOES GREEN-BY-NOT-INSERTING, `postIntegrate` has been changed from
// `operator[]` to a find-and-skip — at which point case 1 is vacuous and the CROSSING entry in
// `SimulationManagerUImpl-rationale.md` (sections 0 and 1) is resting on nothing. Read them as a
// pair, always.
// ============================================================================================
TEST_CASE("RingoutScorePush.AnUnseededRosterMakesTheAwardINSERT", "[BrawlerRingout]")
{
    FPushRig rig(/*seedRoster=*/false);

    REQUIRE(rig.system.scoreEntryCount() == 0u);

    rig.poseDeath(0u);
    rig.step(100u);

    // Two survivors, neither of which had an entry: the table GREW. On the physics thread this
    // is the rehash the game-thread reader must never race.
    CHECK(rig.system.scoreEntryCount() == 2u);
    CHECK(rig.system.scoreOf(1u) == 1u);
    CHECK(rig.system.scoreOf(2u) == 1u);

    // And the dier still got nothing, so the growth is the award's insert and not some other
    // path writing three entries.
    CHECK(rig.system.hasScoreEntry(0u) == false);
}

// ============================================================================================
// CASE 3 — THE TRIPWIRE'S OWN DISCRIMINATION.
//
// The push asserts `hasScoreEntry(id)` before reading `scoreOf(id)`. That tripwire is worth
// nothing unless `hasScoreEntry` can tell "no entry" from "an entry worth zero" — a seeded
// fighter who has never scored reads 0 through `scoreOf`, exactly like a stranger, and a
// tripwire written as `scoreOf(id) != 0` would fire on every character in every match before
// the first death. This pins the distinction the push depends on.
// ============================================================================================
TEST_CASE("RingoutScorePush.TheTripwireTellsAnEmptyEntryFromNoEntry", "[BrawlerRingout]")
{
    FPushRig rig(/*seedRoster=*/true);

    constexpr unsigned int kStranger = 4242u;   // never added, never registered

    // The two ids are INDISTINGUISHABLE through the value the push replicates...
    CHECK(rig.system.scoreOf(0u)        == 0u);
    CHECK(rig.system.scoreOf(kStranger) == 0u);

    // ...and fully distinguishable through the predicate the push guards with.
    CHECK(rig.system.hasScoreEntry(0u)        == true);
    CHECK(rig.system.hasScoreEntry(kStranger) == false);

    // The cast the push performs is lossless over any score a session can reach: one point per
    // death, and this pins the identity at the value a real match produces.
    rig.poseDeath(0u);
    rig.step(100u);
    CHECK(static_cast<int32_t>(rig.system.scoreOf(1u)) == 1);
}

} // namespace ringoutScorePushTests

#endif // WITH_LOW_LEVEL_TESTS

// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

// `[@og]` selects the top-level tags listed below, and nothing else — it is a
// WHITELIST, not the `~[SelfTests]` blacklist. A TEST_CASE whose tags are all absent
// from the list still runs on a direct `exe "[MyTag]"` call but is INVISIBLE to
// `[@og]`: the suite total stays green while those cases never execute.
//
// ⚠ The list covering every tag here is a fact about the tree, not an invariant.
//
// That same absence excludes UE's auto-included Engine framework tests (Core/Async,
// LowLevelTestsRunner self-tests, etc.), which carry unrelated tags like [SelfTests],
// [EditorContext], [EngineFilter] — and it is robust to UE upgrades for the same
// reason: a tag a future engine version introduces cannot match an explicit list.
//
// Run the og-only subset, and check the list is still complete, with:
//     OGBrawlerTests.exe [@og]
// The case count it reports must equal the number of TEST_CASE definitions in this
// module. A shortfall is a tag nobody appended, and it fails nothing.
//
// The og-tools `oglltest brawler` wrapper passes this alias by default.
//
// Maintenance: when adding a TEST_CASE with a new top-level tag category, append it
// to the alias spec below — that append is the only thing keeping those two counts
// equal. Catch2 v3 expands the alias at filter-parse time, so the change is local
// to this file.
CATCH_REGISTER_TAG_ALIAS("[@og]",
    "[DAttack],[CharacterViz],[SimulatableBrawler],[SimulationComposite],[SimulationIntegrationExecutor],[SimulationNetSync],[SimulationReconciliation],[InputSequence],[BrawlerProjectile],[InputPackaging],[PacketBudget],[attackdirection]")

#endif // WITH_LOW_LEVEL_TESTS

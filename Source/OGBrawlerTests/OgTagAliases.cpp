// SPDX-License-Identifier: BUSL-1.1
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

// `[@og]` runs every test in og-brawler-tests. Whitelist of every top-level tag
// used in TEST_CASE definitions in this module. UE's auto-included Engine framework
// tests (Core/Async, LowLevelTestsRunner self-tests, etc.) use unrelated tags like
// [SelfTests], [EditorContext], [EngineFilter] — they are excluded by being absent
// from this whitelist.
//
// Whitelist semantics (vs. the prior `~[SelfTests]` blacklist) is robust to UE
// upgrades: any new framework tag UE introduces will be filtered out automatically
// since it can't match our explicit list.
//
// Run the og-only subset with:
//     OGBrawlerTests.exe [@og]
//
// The og-tools `oglltest brawler` wrapper passes this alias by default.
//
// Maintenance: when adding a TEST_CASE with a new top-level tag category, append
// it to the alias spec below. Catch2 v3 expands the alias at filter-parse time, so
// the change is local to this file.
CATCH_REGISTER_TAG_ALIAS("[@og]",
    "[DAttack],[CharacterViz],[SimulatableBrawler],[SimulationComposite],[SimulationIntegrationExecutor],[SimulationNetSync],[SimulationReconciliation],[InputSequence],[BrawlerProjectile]")

#endif // WITH_LOW_LEVEL_TESTS

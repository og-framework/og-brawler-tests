<!-- SPDX-License-Identifier: BUSL-1.1 -->
# GoldenTraceScene — rationale

Source: `GoldenTraceScene.h`. This file has no guards document: nothing in it needs a `⛔` tag. It holds the
headless brawler-on-Jolt harness that both golden traces run: the flat one (`BrawlerJoltGoldenTraceTest.cpp`,
scenario `GoldenTraceScenario.h`) and the slope one (`BrawlerJoltSlopeGoldenTraceTest.cpp`, scenario
`GoldenTraceSlopeScenario.h`). The trace format, the metrics and the envelope are in `GoldenTraceFormat-rationale.md`.

The harness was moved here from `BrawlerJoltGoldenTraceTest.cpp` by og-ground-aligned-attacks task 3, so that a second
scenario could run through the same code. The flat golden's two `.inl` files and its field names did not change, and the
flat golden still passes in exact mode (§3).

## 1. What a scenario provides

`goldenTraceScene::Scene` is a template on a scenario type, checked by the `goldenTraceScene::Scenario` concept. The
type is a struct of static members, because a template parameter cannot name a namespace:

| member | what the scene does with it |
|---|---|
| `kGenerator` | the generator name written into a recorded golden's header |
| `kDtSeconds`, `kScenarioTicks` | the tick length and the number of ticks a run steps |
| `isSampleTick` | which ticks after tick 0 are recorded |
| `statics` | the queryable static world (the arena) |
| `staticData` | the brawler static data, given a gravity scale |
| `inputAt` | each character's input on each tick |
| `characters` | the character list: id, body slot, field-name prefix, spawn point |
| `spawnTable` | the ring-out spawn points the scene writes into the static data |
| `teleports` | scripted teleports: character, tick, target point (§4) |

The flat test's `FlatScenario` forwards each member to `goldenTraceScenario` (its two characters A and B, its spawn
table of slot 0 at `kSpawnA` and slot 1 at `kRespawnB`, and no teleports). The slope scenario is
`goldenTraceSlopeScenario::Scenario` (`GoldenTraceSlopeScenario-rationale.md`). A `static_assert` beside each one checks
the concept.

## 2. The harness

The `Scene` runs the brawler the way an inline Jolt authority does, with no UE in the loop:

- **World.** The slot template is `joltTickWorldHost::slotTemplateOf` of `SimulatableBrawler`, the world
  config is `joltTickWorldHost::stepWorldConfigOf` (ring depth 0, as on the authority) and the statics are
  built with `joltTickWorldHost::buildStatics`: the three calls `JoltTickWorldHost::build` makes. Gravity is
  the movement StaticData's `gravity` along the frame's up (`simFrame::withHeight`), which in the simulation's
  frame (+Y up) is `(0, g, 0)`, the vector the UE backend gives the Jolt world. No brawler body takes that gravity: each one's
  `BodyDescriptor` has `enableGravity` false, so Jolt gives it a gravity factor of 0, and a wrong world
  gravity does not change the trace (measured: with the literal `(0, 0, g)` in its place at Y-up, the
  bounds comparison prints the same maxima, to every printed digit, as with the correct value). The gravity the trace sees is the movement simulation's own, applied
  along its up axis.
- **Adapters.** `JoltPhysicsBodyAdapter` and `JoltSpatialQueryAdapter` over that world, with the six
  collision categories the brawler maps. A `JoltWorldAccessScope` stands in for the world lock, as in the
  og-simulation-tests Jolt rigs.
- **Registration.** `Scene::registerCharacter` is `JoltTickWorldHost::bind` (one `JoltPhysicsFactory` per
  character, each declaration's body, shapes and query volumes) followed by the brawler roster's
  `BrawlerRosterUImpl::onBodiesCreated` (the capsule binding, the spawn teleport, the ring-out spawn slot),
  then the store, the systems' registration hook and the body-slot occupancy, in the order
  `UESimulationSession::tryRegister` runs them. Characters register in the order of the scenario's character list.
- **Static data.** `simulatableBrawler::StaticData` from the scenario's `staticData`. The ring-out spawn points named by
  the scenario's `spawnTable` are then set, as the roster seeds the spawn table from the level. Nothing else in the
  static data is overridden.
- **The step.** One authority tick is `SimulationManager`'s authority step followed by the step driver's
  physics step: `firePreIntegrate`, `integrateAll`, `firePostIntegrate`, `JoltWorld::step`,
  `captureBodyStatesAll`. Before them, `Scene::applyTeleports` writes the tick's scripted teleports (§4). The systems
  are the game's three (hit detection, hit routing, ring-out score) in the game's order; a `static_assert` checks
  detection fires before routing. Every character's input is taken from the scenario's `inputAt`, in character-list
  order. The tick dt is the scenario's `kDtSeconds`; both scenarios use the project's `AsyncFixedTimeStepSize`
  (0.016667 s).

The harness reads state only after `captureBodyStatesAll`, from the storage, so a sample is what the
authority would publish for that tick. One exception: the hit detector's outputs (the radial's
`getHitsThisTick` and `getGuardBlockedThisTick`, the projectile's `detectedThisTick`) are consumed and cleared
inside the same tick, so they read empty after the step. `Scene::noteDetections` reads them between
`firePreIntegrate` and `integrateAll`, after routing has used them, and keeps cumulative counts with the
last tick and target (`DetectionLog`). It only reads, so the run is unchanged. Cumulative counts also
survive the sparse sampling, where a one-tick event would otherwise fall between samples.

`Scene::record` writes one sample: `Scene::recordCharacter` for every character, in character-list order, each field
name starting with the character's prefix.

## 3. The flat golden is unchanged by the move

The flat run performs the same calls in the same order as before the move: spawn points for slots 0 and 1, then
registration of A and B, then per tick the inputs of A then B, the detections of A then B, and the records of A then
B. The scripted teleports are an empty list for it. Measured at task 3: `BrawlerJoltGoldenTrace.MatchesTheRecordedTrace`
passes in exact mode against the unchanged `.inl` files (361 samples, 120 fields, 0 mismatching), with the
`[GoldenTrace.signedZero]` line all zero, and `BrawlerJoltGoldenTrace.ScenarioCoversEveryScriptedMechanic` prints the
coverage counts `GoldenTraceScenario-rationale.md` §5 records.

## 4. Scripted teleports

A `goldenTraceScene::Teleport` moves one character at the start of one tick. `Scene::applyTeleports` sets the
character's movement initial conditions, `teleportPending` and `teleportPos`, exactly as registration seeds the spawn
point; the movement simulation consumes them in that tick's integrate (it places the capsule, zeroes the velocity and
clears the command flag), the same path the ring-out respawn takes. It is a cut in a test script, not a game
mechanic: it lets one scenario reuse two characters at several places (the slope's crease pairs) within its tick
budget. Nothing else in the state is touched, so a teleport is scheduled only for a character that is idle (not
attacking, not in a hit lockout).

## 5. Runs, nudges and the shared cases

`goldenTraceScene::runScenario` builds a scene, records tick 0, steps every tick and records the sample ticks.
`goldenTraceScene::envelopeNudges` lists the envelope runs: for each character in list order, spawn component 0 then 1,
one ULP up then down. For the flat scenario that is the same eight nudges, in the same order, as the hand-written list
it replaced (the order the flat envelope's `kEnvelopeNudges` records).

The two cases every golden needs are shared:

- `goldenTraceScene::requireMatchesTheRecordedTrace` is the body of a `MatchesTheRecordedTrace` case: the mode from
  `OG_GOLDEN_MODE`, the header checks, the comparison and its printed lines
  (`BrawlerJoltGoldenTraceTest-rationale.md` §2).
- `goldenTraceScene::recordGoldenAndEnvelope` is the body of a `Record` case: it runs the base scenario twice and
  requires the runs to agree bit for bit, runs the nudges, and writes the golden and the envelope under the given file
  names into `OG_GOLDEN_OUT` (`BrawlerJoltGoldenTraceTest-rationale.md` §3). It returns the base trace, which the flat
  test passes to its coverage count.

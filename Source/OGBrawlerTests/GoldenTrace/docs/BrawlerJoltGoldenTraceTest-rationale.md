<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerJoltGoldenTraceTest — rationale

Source: `BrawlerJoltGoldenTraceTest.cpp`. This file has no guards document: nothing in it needs a `⛔`
tag. The format, the metrics and the envelope are in `GoldenTraceFormat-rationale.md`; the scenario is in
`GoldenTraceScenario-rationale.md`.

## 1. The harness

The `Scene` runs the brawler the way an inline Jolt authority does, with no UE in the loop:

- **World.** The slot template is `joltTickWorldHost::slotTemplateOf` of `SimulatableBrawler`, the world
  config is `joltTickWorldHost::stepWorldConfigOf` (ring depth 0, as on the authority) and the statics are
  built with `joltTickWorldHost::buildStatics`: the three calls `JoltTickWorldHost::build` makes. Gravity is
  the movement StaticData's `gravity` along the frame's up (`BasicSimFrame::withHeight` through the `simFrame` alias), which in the shipped
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
  `UESimulationSession::tryRegister` runs them.
- **Static data.** `simulatableBrawler::StaticData` from the scenario's `scenarioStaticData`, with the
  values the shipped game gives: the defaults of the four movement console variables (continuous
  accel/brake, walk 1 m/s, step period 20 ticks, step speed 1 m/s) and the engine's default gravity
  (−9.8 m/s², UE's −980 cm/s²), as native metre literals. The ring-out spawn points of slots 0 and 1 are
  then set from the scenario's native points, as the roster seeds the spawn table from the level. Nothing else in the
  static data is overridden.
- **The step.** One authority tick is `SimulationManager`'s authority step followed by the step driver's
  physics step: `firePreIntegrate`, `integrateAll`, `firePostIntegrate`, `JoltWorld::step`,
  `captureBodyStatesAll`. The systems are the game's three (hit detection, hit routing, ring-out score) in
  the game's order; a `static_assert` checks detection fires before routing. The tick dt is the project's
  `AsyncFixedTimeStepSize` (0.016667 s).

The harness reads state only after `captureBodyStatesAll`, from the storage, so a sample is what the
authority would publish for that tick. One exception: the hit detector's outputs (the radial's
`hitsThisTick` and `guardBlockedThisTick`, the projectile's `detectedThisTick`) are consumed and cleared
inside the same tick, so they read empty after the step. `Scene::noteDetections` reads them between
`firePreIntegrate` and `integrateAll`, after routing has used them, and keeps cumulative counts with the
last tick and target (`DetectionLog`). It only reads, so the run is unchanged. Cumulative counts also
survive the every-fifth-tick sampling after tick 300, where a one-tick event would otherwise fall
between samples.

## 2. The cases

| case | tag | what it does |
|---|---|---|
| `ScriptCoversEveryTickOnce` | `[GoldenTrace]` | every character has exactly one input row on every tick |
| `MatchesTheRecordedTrace` | `[GoldenTrace]` | runs the scenario and compares it with the committed golden, in the mode named by `OG_GOLDEN_MODE`: `exact` (the default) or `bounds`; any other value fails the case |
| `ScenarioCoversEveryScriptedMechanic` | `[GoldenTrace]` | counts, from the live trace in the legacy frame, every mechanic the scenario must reach, and requires each |
| `Record` | `[.GoldenRecord]` | hidden: records the golden and the envelope into the folder named by `OG_GOLDEN_OUT` |

The coverage count reads the trace in the legacy (canonical) frame, whose values the recorder has already
converted, so its thresholds are canonical centimetres too. `kWalkSpeedFloorCmPerS`,
`kKnockbackSpeedFloorCmPerS` and `kHeightSlackCm` are written in centimetres. The scenario's forward
positions and step height (`kFloorStart`, `kRampFoot`, `kRampTop`, `kStepEdge`, `kStepHeight`, native metres)
are converted with `Frame::toLegacyCm`, the converter the recorder uses, so the count and the scene read one
set of constants. Converted, three of them differ from the centimetre floats task 2 compared against by at
most 8e-5 cm (for example `kStepEdge` is 2179.99992 cm, not 2180); the coverage counts are unchanged.

`[GoldenTrace]` is in og-brawler-tests' `[@og]` whitelist (`OgTagAliases.cpp`); `[.GoldenRecord]` is hidden
and runs only when named.

`MatchesTheRecordedTrace` prints one `[GoldenTrace.compare]` line (mode, both knob pairs, samples, fields,
mismatching fields, samples passed before the first mismatch) and the largest metric per tag. In exact
mode it then prints the `[GoldenTrace.signedZero]` line (`GoldenTraceFormat-rationale.md` §5). In bounds
mode it prints one `[GoldenTrace.bound]` line per bound class (`class=`, the largest metric `max=`, the
`bound=`, their ratio `maxOverBound=`, and the tick and field of the largest metric, `worst=`), then the
`[GoldenTrace.envelopeRatio]` diagnostic per tag. On a failure it prints the first mismatch.

Exact mode refuses a golden recorded at other knobs. Bounds mode runs at any knobs, so it is the mode for
a build whose `kUnitsPerMetre` or `kAxisFrame` differs from the golden's.

When the two files are absent (`__has_include`), the case fails at its first `REQUIRE`, so a missing golden
can never pass.

## 3. Recording and re-recording

From the project root, after building OGBrawlerTests:

    mkdir -p "$TMP/golden" && OG_GOLDEN_OUT="$TMP/golden" OG_GOLDEN_SOURCE="<repo and submodule HEADs>" \
      Binaries/Win64/OGBrawlerTests/OGBrawlerTests.exe "BrawlerJoltGoldenTrace.Record"

Then copy the two files it wrote, unedited, into `GoldenTrace/`, rebuild, and run `"[GoldenTrace]"` twice.
The recorder runs the base scenario twice and requires the two runs to agree bit for bit before it writes
anything; it prints the sample, field and value counts, both file sizes, the envelope maximum per tag, the
first tick at which a nudge changed a discrete value, and the recorded coverage counts.

Re-record when a change alters behaviour on purpose (a units or axis flip, once the flipped build passes
in bounds mode), or when the recorded fields change. Never edit either file by hand: a hand edit is exactly what exact mode is
there to catch.

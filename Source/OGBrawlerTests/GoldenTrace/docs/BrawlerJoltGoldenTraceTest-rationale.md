<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerJoltGoldenTraceTest — rationale

Source: `BrawlerJoltGoldenTraceTest.cpp`. This file has no guards document: nothing in it needs a `⛔`
tag. The format, the metrics and the envelope are in `GoldenTraceFormat-rationale.md`; the scenario is in
`GoldenTraceScenario-rationale.md`; the harness is in `GoldenTraceScene-rationale.md`.

## 1. The harness and the refactor

The harness this test runs, `goldenTraceScene::Scene`, lives in `GoldenTraceScene.h` and is described in
`GoldenTraceScene-rationale.md`. Until og-ground-aligned-attacks task 3 it was this file's own `Scene`; task 3 moved it,
with the run, envelope, comparison and recording helpers, into that header as a template on a scenario type, so that the
slope golden (`BrawlerJoltSlopeGoldenTraceTest-rationale.md`) runs through the same code. This file now keeps:

- `FlatScenario`, which hands the scene the flat scenario of `GoldenTraceScenario.h` (characters A and B, spawn slots 0
  and 1 at `kSpawnA` and `kRespawnB`, no scripted teleports, generator `BrawlerJoltGoldenTrace.Record`);
- the recorded golden's views (`recordedGolden`, `recordedEnvelope`) over `BrawlerJoltGolden.inl` and
  `BrawlerJoltGoldenEnvelope.inl`;
- the coverage count (`coverageOf`) and the four cases below.

The move changed no recorded value: the two `.inl` files are byte-unchanged and the case passes in exact mode
(`GoldenTraceScene-rationale.md` §3).

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
are converted with `goldenTrace::toLegacyCm`, the converter the recorder uses, so the count and the scene read one
set of constants. Converted, three of them differ from the centimetre floats task 2 compared against by at
most 8e-5 cm (for example `kStepEdge` is 2179.99992 cm, not 2180); the coverage counts are unchanged.

`[GoldenTrace]` is in og-brawler-tests' `[@og]` whitelist (`OgTagAliases.cpp`); `[.GoldenRecord]` is hidden
and runs only when named.

`MatchesTheRecordedTrace` prints one `[GoldenTrace.compare]` line (mode, the golden's and the build's frame codes, samples, fields,
mismatching fields, samples passed before the first mismatch) and the largest metric per tag. In exact
mode it then prints the `[GoldenTrace.signedZero]` line (`GoldenTraceFormat-rationale.md` §5). In bounds
mode it prints one `[GoldenTrace.bound]` line per bound class (`class=`, the largest metric `max=`, the
`bound=`, their ratio `maxOverBound=`, and the tick and field of the largest metric, `worst=`), then the
`[GoldenTrace.envelopeRatio]` diagnostic per tag. On a failure it prints the first mismatch.

Exact mode refuses a golden recorded in another frame or unit (its `axisFrame` and `unitsPerMetre` codes against
`goldenTrace::kSimulationAxisFrame` and `kSimulationUnitsPerMetre`). Bounds mode runs whatever the codes, so it is
the mode for a golden recorded before a flip.

When the two files are absent (`__has_include`), the case fails at its first `REQUIRE`, so a missing golden
can never pass.

## 3. Recording and re-recording

From the project root, after building OGBrawlerTests:

    mkdir -p "$TMP/golden" && OG_GOLDEN_OUT="$TMP/golden" OG_GOLDEN_SOURCE="<repo and submodule HEADs>" \
      Binaries/Win64/OGBrawlerTests/OGBrawlerTests.exe "BrawlerJoltGoldenTrace.Record"

Then copy the two files it wrote, unedited, into `GoldenTrace/`, rebuild, and run `"[GoldenTrace]"` twice. The
golden is included through `__has_include`: on a first recording the files did not exist at the last build, so the
build's dependency list does not name them. Touch the including test `.cpp` before that rebuild, or the binary still has
no golden (seen at task 3 with the slope golden: the first rebuild after the copy compiled nothing, and its
`MatchesTheRecordedTrace` failed at the recorded-golden check).
The recorder runs the base scenario twice and requires the two runs to agree bit for bit before it writes
anything; it prints the sample, field and value counts, both file sizes, the envelope maximum per tag, the
first tick at which a nudge changed a discrete value, and the recorded coverage counts.

Re-record when a change alters behaviour on purpose (a units or axis flip, once the flipped build passes
in bounds mode), or when the recorded fields change. Never edit either file by hand: a hand edit is exactly what exact mode is
there to catch.

og-ground-aligned-attacks task 10 re-recorded it when the ground-aligned attacks were switched on
(`brawlerGround::kGroundAlignedAttacks`). A value-by-value diff of the two recordings changed one field only:
`A.body1.GuardAxis.rotation` on ticks 212 to 266, the ticks on which A's ground probe stands on the 20° ramp (its capsule
centre 20.45 to 21.30 m along the walk, the ramp spanning 20.5 to 21.32 m; Supported throughout), where the guard now tilts with the ramp (a 20° turn, previously
the identity). Every other field and tick is bit-identical.

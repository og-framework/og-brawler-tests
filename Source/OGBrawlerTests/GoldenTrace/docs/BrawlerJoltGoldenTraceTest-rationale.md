<!-- SPDX-License-Identifier: BUSL-1.1 -->
<!-- lint-external-ref: PrintsTheCoverageUnderLegacy -- a RETIRED Catch2 case -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the table row records it -->
<!-- lint-external-ref: FlatScenarioUnderLegacy -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
<!-- lint-external-ref: legacyAgainstTheRecording -- a RETIRED print tag -- its only printer went with the Legacy print case in og-ground-aligned-attacks task 37 -->
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
| `PrintsTheCoverageUnderLegacy` | `[GoldenTrace]` | runs `FlatScenarioUnderLegacy` (the flat scenario with the movement's support law set to Legacy by its `configureStaticData`), prints its `[GoldenTrace.coverage] legacy …` line and one `[GoldenTrace.legacyAgainstTheRecording] scenario=flat …` line (the exact compare with the recording, which does not fail the case), and requires only that the schemas match. Task 35 wrote it as the V2 print (PrintsTheCoverageUnderV2), while Legacy was the default; task 36 made it the Legacy print, and task 37 deleted it with the Legacy law |
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
(the initiative's compile-time knob, which task 14 deleted without a re-recording). A value-by-value diff of the two recordings changed one field only:
`A.body1.GuardAxis.rotation` on ticks 212 to 266, the ticks on which A's ground probe stands on the 20° ramp (its capsule
centre 20.45 to 21.30 m along the walk, the ramp spanning 20.5 to 21.32 m; Supported throughout), where the guard now tilts with the ramp (a 20° turn, previously
the identity). Every other field and tick is bit-identical.

og-ground-aligned-attacks task 30 re-recorded it when `decomposeVelocity` became exact on every tangent frame. The
scenario takes the new solve on one tick: A on the 20° ramp at tick 233, whose ground normal is 0.66° off the up–forward
plane (`dot(v, up)` = −0.0124; on every other call of the scenario it is within the 1e-6 tolerance, and the unchanged 2×2
solve runs). A
value-by-value diff of the two recordings changed two fields on two samples: `A.body5.CharacterCapsule.linearVelocity` and
`A.movement.velocity` on ticks 233 and 234, in the vertical component only, by −1.49e-5 in the recorded legacy units
(cm/s), that is −1.49e-7 m/s. Every other field and tick is bit-identical, and the envelope changed only in its
source string.

og-ground-aligned-attacks task 14b re-recorded it when the last two flat branches of the attacks were folded into the
general expressions (user rulings of 2026-11-02). Site 2a: the machine's radial initial conditions take the
quaternion round trip on flat ground too (`DAttackMachineSimulation-rationale.md` §16). Site 2b: `wouldGuardBlock`
always projects into the guard's plane (`DAttackRadialSimulation-rationale.md` §7). The recorder printed
`samples=361 fields=120 values=86640 goldenBytes=755727 envelopeBytes=26146 runs=8`. A value-by-value diff of the two
recordings changed four fields, all A's and all from site 2a, on 751 samples, none of them signed-zero only:

* `A.radial.initialAimAngle`: 321 samples, ticks 40–600, at most 2.38e-7 rad (π/2 stored two float steps higher);
* `A.radial.initialAimRotationAxis`: 321 samples, ticks 40–600, at most 5.96e-8;
* `A.body0.WeaponAxis.rotation`: 98 samples, ticks 40–525, at most 5.66e-7, the weapon pose the radial builds from that
  pair;
* `A.body0.WeaponAxis.angularVelocity`: 11 samples, ticks 164–525, at most 4.77e-6 rad/s, the same pose's rate.

No hit, block, machine, movement or capsule field changed. The new recording equals, value for value, the one task 14b's
part 1 recorded with site 2a folded alone, so site 2b changed no value. The envelope changed only in its source string.

og-ground-aligned-attacks task 36 re-recorded it when the movement's support law v2 became the default (og-brawler's
movement rationale, §36). The recorder printed `samples=361 fields=120 values=86640 goldenBytes=752280
envelopeBytes=26112 runs=8`, an envelope position maximum of 5.0e-4 cm and no discrete divergence. Against task 14b's
recording it differs in 1,521 field-samples of 16 fields, A's and B's, none of them signed-zero only, the first at
tick 164 (`B.body5.CharacterCapsule.position`). That is the V2 run task 35's print compared with the old recording,
and the Legacy print now reads the same count against the new one. The live coverage line changed in three counts:
`walkTicks` 121 → 122, `knockbackTicks` 16 → 15 and `ledgeFallTicks` 141 → 143. By mechanism:

* **Flat knockback ticks (the one write, no flicker).** A's hit on B at 164: under Legacy B was Unsupported on 164
  and 166 (the flat-ground flicker, its vertical velocity −0.163 m/s, one tick of gravity, on the hit tick), and under
  V2 it stays Supported until it leaves the ledge at 169. B then falls off the arena three ticks earlier (respawn at
  355, not 358), so B's capsule, weapon, guard and velocity fields differ on 164–385 (`B.ringout.respawnAtTick` to
  the end), A's score turns over
  three ticks earlier (`A.ringout.score`, 235–237), and B lands Supported at 360.
* **The frame normal on the ramp (GJK to the exact face).** On tick 233 the 20° ramp's sweep normal is 0.66° off the
  up–forward plane (task 30's tick, §3 above). Legacy's frame took it, so A gained a sideways velocity of −0.0044 m/s
  and drifted −7.4e-5 m across the walk; V2 takes the exact face, so A stays at 0. A's capsule, weapon and guard
  positions differ from there to the end. Before 233, on the ramp's foot and face (213–232), A's velocity differs by
  at most 8.9e-8 m/s; after it the runs take different paths, and at the ramp's top edge and the step's edge (below)
  they differ by up to 0.31 and 0.77 m/s.
* **Edge ticks.** At the ramp's top edge A's guard stays tilted one tick longer (`A.body1.GuardAxis.rotation`, 267).
  At the step's edge Legacy's frame was the edge's tilted sweep normal, so A's walk split along it (0.86 m/s forward
  and 0.61 m/s down on 310) and A was Unsupported on 320; under V2 A keeps the full walk speed (1 m/s) over the
  edge and falls at gravity, Supported on every sample there.
* **Crease ticks.** The flat scenario has no knockback across a crease.

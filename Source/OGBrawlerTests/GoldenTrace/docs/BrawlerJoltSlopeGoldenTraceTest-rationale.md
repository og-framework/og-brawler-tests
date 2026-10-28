<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerJoltSlopeGoldenTraceTest — rationale

Source: `BrawlerJoltSlopeGoldenTraceTest.cpp`. This file has no guards document: nothing in it needs a `⛔` tag. The
scenario is in `GoldenTraceSlopeScenario-rationale.md`, the harness in `GoldenTraceScene-rationale.md`, the format in
`GoldenTraceFormat-rationale.md`. The test runs the slope scenario through `goldenTraceScene::Scene` and compares it
with `BrawlerJoltSlopeGolden.inl` and `BrawlerJoltSlopeGoldenEnvelope.inl`, recorded at og-ground-aligned-attacks task 3
as today's baseline and re-recorded at task 3b, when the convex crease pair moved (`GoldenTraceSlopeScenario-rationale.md`
§5). §6 describes the crease reach sweep in `CreaseReachTest.cpp`, which chose that placement.

## 1. The golden files and their namespace

The writer (`goldenTrace::writeGoldenInl`, `goldenTrace::writeEnvelopeInl`) puts every golden in the namespaces
`goldenTraceData` and `goldenTraceEnvelopeData`, as `inline constexpr` arrays. An inline variable has one definition per
program, and the flat test includes its own files with the same names: two different definitions of one inline variable
break the one-definition rule, and the linker keeps one of them without a diagnostic. So this test includes its two
files inside `namespace slopeGoldenData`, which makes every name distinct; the files stay exactly as the writer wrote
them. `recordedSlopeGolden` and `recordedSlopeEnvelope` view them; when either file is absent (`__has_include`), the
compare case fails at its first `REQUIRE`.

## 2. The cases

| case | tag | what it does |
|---|---|---|
| `ScriptCoversEveryTickOnce` | `[GoldenTrace]` | every one of the seven characters has exactly one input row on every tick |
| `MatchesTheRecordedTrace` | `[GoldenTrace]` | runs the scenario and compares it with the golden in the mode `OG_GOLDEN_MODE` names, as the flat case does (`goldenTraceScene::requireMatchesTheRecordedTrace`, `BrawlerJoltGoldenTraceTest-rationale.md` §2) |
| `PrintsTheSlopeMeasurements` | `[GoldenTrace]` | runs the scenario once, reading every character after every tick, and prints one `[GoldenTrace.slope]` line (§3) |
| `Record` | `[.GoldenRecord]` | hidden: records the golden and its envelope (28 nudged runs: 7 characters, 2 components, 2 directions) into `OG_GOLDEN_OUT` |

The full names are `BrawlerJoltSlopeGoldenTrace.ScriptCoversEveryTickOnce`, `BrawlerJoltSlopeGoldenTrace.MatchesTheRecordedTrace`,
`BrawlerJoltSlopeGoldenTrace.PrintsTheSlopeMeasurements` and `BrawlerJoltSlopeGoldenTrace.Record`.

The measurement case asserts only that the run reached the slope (`slopeTicks` above zero). It prints numbers for a
reader to compare across the initiative: today's baseline, and after the flip the re-recorded behaviour.

## 3. The measurements

One line, `[GoldenTrace.slope]`, with these fields:

- `idleDriftAM`: A's largest horizontal distance, during the 60 idle ticks, from where it first became Supported.
- `slopeTicks`, `normalDiffersTicks`, `maxNormalDifference`: over every character and tick that is Supported with its
  probe point on the slope face (within 1 mm of the face plane and at least 0.5 m inside its edges), the count, the
  count where the movement's `surfaceNormal` (its derived state) differs from the analytic face normal
  (0, cos 30°, sin 30°) in any component by more than 1e-6, and the largest such difference.
- `unsettledPhaseStarts`: the number of (action tick, character) pairs where the character was not Supported, or moved
  more than 1 mm horizontally in one tick, during the 20 ticks before the action. The actions are the nine slot actions
  and A's crease swing.
- `creaseHits`: crease-pair knockbacks against crease-pair swings in the script.
- `inbound`: per character, ticks it was hit and ticks its own swing was guard-blocked (`wasHitThisTick`,
  `wasGuardBlockedThisTick`).
- One `|` group per knockback, in tick order. A knockback is a tick on which the target's inbound hit is a Knockback;
  the attacker is the character whose detection log names that target on that tick. Its **window** runs from the hit to
  the tick before the target's next teleport or next knockback, or to the end of the run. Over the window:
  `travelM`, the 3-D distance from the target's position on the tick before the hit to its position at the window's
  end; `unsupportedTicks`; `apexM`, the greatest height above that launch point; `airtimeS`, the length of the first
  run of Unsupported ticks; `lockoutEndTick`, the first tick not in HitFlinch; `minClearanceM` and `restClearanceM`, the
  capsule centre's distance to the nearest exposed ground (the slope face, the landing beyond the concave crease, the
  plateau beyond the convex crease), its least value in the window and its value at the launch point. `dir` is the
  inbound hit direction in the (forward, left) chart.

## 4. Today's baseline (task 3, re-recorded at task 3b)

    [GoldenTrace.slope] idleDriftAM=0 slopeTicks=8148 normalDiffersTicks=0 maxNormalDifference=2.98e-08
      unsettledPhaseStarts=0 creaseHits=6/8 knockbacks=8 inbound=A.0/1 B.0/0 C.0/0 D.1/0 E.1/0 F.4/0 G.2/0
      | A.>D.tick=410 dir=(-1.0000,-0.0000) travelM=5.1714 unsupportedTicks=1 apexM=0.0000 ...
      | A.>E.tick=520 dir=(1.0000,-0.0000) travelM=5.0580 unsupportedTicks=22 apexM=2.5293 ...

(wrapped and shortened; the impl notes of tasks 3 and 3b quote it whole). Read with the scenario's slot table:

- A does not drift on the slope (0 m), and the movement's face normal matches the analytic one to 3e-8.
- The side swings at B (1 m above A) and C (1 m below) hit nothing: today the swing axis is vertical, so the ring stays
  at A's height, beyond the band the detector allows (`GoldenTraceSlopeScenario-rationale.md` §5). The swings into C's
  guard reach nothing either, and the two projectiles neither hit nor are blocked.
- D and E are thrown along the fall line in opposite directions. D, thrown down, slides 5.17 m and stays Supported but
  for one tick; E, thrown up, detaches and hops (22 Unsupported ticks, apex 2.53 m).
- B's guard blocks the overhead (A's own inbound shows the one block).
- At the creases six of eight swings land (four at task 3, before the convex pair moved;
  `GoldenTraceSlopeScenario-rationale.md` §5 derives which). None in a long slot launches; the convex hit at 990 hops
  0.84 m (43 Unsupported ticks).

## 5. Recording and re-recording

From the project root, after building OGBrawlerTests:

    mkdir -p "$TMP/golden" && OG_GOLDEN_OUT="$TMP/golden" OG_GOLDEN_SOURCE="<repo and submodule HEADs>" \
      Binaries/Win64/OGBrawlerTests/OGBrawlerTests.exe "BrawlerJoltSlopeGoldenTrace.Record"

Then copy the two files it wrote, unedited, into `GoldenTrace/`, touch `BrawlerJoltSlopeGoldenTraceTest.cpp` (the
golden is included through `__has_include`, `BrawlerJoltGoldenTraceTest-rationale.md` §3), rebuild, and run
`"[GoldenTrace]"` twice. At task 3 the recording printed `samples=537 fields=420 values=451080` (3.8 MB of golden),
and the envelope's largest position metric was 0.0041 cm with no discrete divergence. Task 3b's re-recording printed
the same counts, `goldenBytes=3829129 envelopeBytes=39169`, the same envelope maximum and again no discrete divergence. A one-ULP edit of one recorded
value fails exact mode at that value's first sample.

Re-record when a change alters the slope behaviour on purpose (the initiative's flip, task 10) or the recorded fields
change. Never edit either file by hand.

## 6. The crease reach sweep (`CreaseReachTest.cpp`, task 3b)

`BrawlerJoltSlopeCreaseReach.PrintsTheConnectingWindows` (`[GoldenTrace]`, no golden) predicts, for each crease, role and
swing sense, which placements of the crease pair connect, today and after the flip. It runs no swing.

- **Rest positions are measured.** `SettleScenario` puts characters `kTeleportHeight` (1.2 m) above face points on both
  sides of both creases, 0.05 to 1.00 m from the crease in 0.05 m steps (80 points, eight per scene, 1.5 m apart along
  the contour), plus the scenario's own four crease points, and runs each scene for 60 ticks with no input through
  `goldenTraceScene::Scene` on the slope's statics. Each character must end Supported and moving at most 1 mm per tick
  horizontally. The test keeps the capsule's offset from its face point, its attack ground sample
  (`sampleAttackGround` with the ground-aligned path selected) and its movement probe face (`probeGround`: the hit normal
  when walkable, else world up). Inside the face nothing changes along the contour, so a measured offset is moved to
  the pair's own x (F at −1, G at +1).
- **The initial conditions are production.** Each swing's comes from `groundAlignedMachineRig::integrateOnce`, a direct
  `integrate3` call, with the scenario's own `swing` input row for that side: today with a default `GroundSample`, after
  the flip with the attacker's measured sample. The test requires the machine to start that side's sequence.
- **The band is the detector's expressions, re-typed.** `reachOf` repeats the lines of `detectRadialHits` from the hit
  direction to the in-circle test, with the pivot at capsule + `attachmentOffset`, the axis from the IC and the
  sequence, and the ring from `getAttackCircle`; it then calls `getAttackSegment` on the on-plane direction and requires
  a damaging segment. Three things are assumed, not run: the weapon body sits at capsule + `attachmentOffset` (the
  radial's integrate puts it there; the world step can move it slightly before the next detection), the overlap query
  reports the target (its capsule is about 2.1 m from the pivot, inside the 3 m query sphere), and the weapon reaches the
  target's segment (a damaging segment that lasts at least one tick is visited; the test requires that duration).
- **The launch flag is the design's rule, re-typed.** The tangent is the detector's `swingTangentAt` (the axis crossed
  with the on-plane direction, signed by the sequence's angular velocity in the middle of the target's segment). It
  counts as a launch when its dot with the target's movement probe face exceeds sin 2° (the initiative's design §3.6;
  the movement's own constant comes later in the initiative). It predicts the behaviour after the knockback and flip
  tasks; it is not a measurement.
- **The lines.** `[GoldenTrace.creaseRest]`, per crease and side: the capsule's height above its face point and each
  sample's dot with the face normal. `[GoldenTrace.creaseReach]`, per crease, role, sense and mode: the connecting pairs
  as slope-distance:flat-distance windows, and after the flip the launching ones. `[GoldenTrace.creaseBest]`, per crease
  and number of swings connecting in both modes: the placements with the most predicted launches.
  `[GoldenTrace.creaseScenario]`: every swing at the scenario's own placement.
- **The checks.** At the scenario's placement: concave 2 swings connecting today and 4 after the flip, convex 4 and 4,
  and at least one convex launch. Moving the crease pair without re-running the sweep fails them. With task 3's 0.3 m at
  the convex crease they failed: convex 2 today, 0 after the flip, no launch.

<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerJoltSlopeGoldenTraceTest — rationale

Source: `BrawlerJoltSlopeGoldenTraceTest.cpp`. This file has no guards document: nothing in it needs a `⛔` tag. The
scenario is in `GoldenTraceSlopeScenario-rationale.md`, the harness in `GoldenTraceScene-rationale.md`, the format in
`GoldenTraceFormat-rationale.md`. The test runs the slope scenario through `goldenTraceScene::Scene` and compares it
with `BrawlerJoltSlopeGolden.inl` and `BrawlerJoltSlopeGoldenEnvelope.inl`. They were recorded at og-ground-aligned-attacks
task 3 as the baseline with the attacks built horizontally, re-recorded at task 3b, when the convex crease pair moved
(`GoldenTraceSlopeScenario-rationale.md` §5), and re-recorded at task 10, the flip that turned the ground-aligned attacks
on (the initiative's compile-time knob; task 14 deleted it and the recording stayed exact), and again at task 13b, which gave the movement the attack's ground rule
(only F's convex-crease hits changed, §4), and at task 14b, which folded the attacks' last two flat branches (only the
swing initial conditions of F and G on the flat ground changed, §4), and at task 36, which made the movement's support law v2
the default (§4). Since task 10 five behaviour cases (§7) state what the flip is for. §6
describes the crease reach sweep in `CreaseReachTest.cpp`, which chose the crease placement.

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
| `PrintsTheSlopeMeasurements` | `[GoldenTrace]` | prints one `[GoldenTrace.slope]` line (§3) |
| `PrintsTheSlopeMeasurementsUnderLegacy` | `[GoldenTrace]` | the same measurement with the movement's support law set to Legacy, and how far that run is from the recording (§2.1); task 37 deletes it |
| `SwingsFollowTheGround` | `[GoldenTrace]` | every swing's plane at its start and through the swing (§7) |
| `SideSwingsHitUpAndDownTheSlope` | `[GoldenTrace]` | A's side swings hit B, up the slope, and C, down it (§7) |
| `GuardsBlockInTheGroundPlane` | `[GoldenTrace]` | B's guard blocks A's overhead, C's guard each side swing (§7) |
| `ProjectilesFollowTheSlope` | `[GoldenTrace]` | both shots keep their height above the face, and C's guard blocks A's (§7) |
| `KnockbacksFollowTheAttackersSwing` | `[GoldenTrace]` | same-face slides, crease launches and entries, the overhead's stun (§7) |
| `Record` | `[.GoldenRecord]` | hidden: records the golden and its envelope (28 nudged runs: 7 characters, 2 components, 2 directions) into `OG_GOLDEN_OUT` |

The full names are `BrawlerJoltSlopeGoldenTrace.` followed by the case name.

The measurement and the five behaviour cases read one run of the scenario: `theSlope` runs it the first time a case
asks and keeps the result (a function-local `static`), so the cases cost one run together. The run reads every
character after every tick and, before every tick, re-takes each character's ground the way the tick is about to (§3).

### 2.1 The Legacy print (og-ground-aligned-attacks task 36; task 35's V2 print)

`PrintsTheSlopeMeasurementsUnderLegacy` runs the scenario as `SlopeScenarioUnderLegacy`, the slope scenario with one
addition: its `configureStaticData` sets `m_movementStaticData.supportLaw` to `SupportLaw::Legacy`. The measurement is
the same code (`measureTheSlope` and the two samplers are templates over the scenario), run once more and not cached,
so the `theSlope` run of the cases above stays the shipped law's.
- It prints the `[GoldenTrace.slope]` line with ` law=legacy` after the tag. At task 36 that line equals, byte for
  byte, the unlabelled line of the build before the flip, so it is the flip's before.
- It runs the Legacy scenario a second time through `runScenario` and compares it with the recording in exact mode,
  without failing on a mismatch: one `[GoldenTrace.legacyAgainstTheRecording] scenario=slope samples=… values=…
  mismatchingValues=… samplesPassedBeforeFirstMismatch=… first=…` line. At task 36 it reads `mismatchingValues=2965
  samplesPassedBeforeFirstMismatch=121`, the numbers of task 35's V2 print against the old recording. The case
  requires only that the schemas match.
- Task 35 wrote it as the V2 print (PrintsTheSlopeMeasurementsUnderV2 over SlopeScenarioUnderV2), while Legacy
  was the default, with three checks of what V2 changes at the concave crease. Since task 36 V2 is the run of the
  cases above, so those checks are in `KnockbacksFollowTheAttackersSwing` (§7), and this case keeps the Legacy run
  for comparison until the initiative's task 37 deletes it with the law.

## 3. The measurements

One line, `[GoldenTrace.slope]`, with these fields:

- `idleDriftAM`: A's largest horizontal distance, during the 60 idle ticks, from where it first became Supported.
- `slopeTicks`, `normalDiffersTicks`, `maxNormalDifference`: over every character and tick that is Supported with its
  probe point on the slope face (within 1 mm of the face plane and at least 0.5 m inside its edges), the count, the
  count where the movement's `surfaceNormal` (its derived state) differs from the analytic face normal
  (0, cos 30°, sin 30°) in any component by more than 1e-6, and the largest such difference.
- `unsettledPhaseStarts`: the number of (action tick, character) pairs where the character was not Supported, or moved
  more than 1 mm horizontally in one tick, during the 20 ticks before the action. The actions are slots 1 to 9 and A's
  crease swing (C's overhead in slot 10 is not one: A walks during the 20 ticks before it).
- `creaseHits`: crease-pair knockbacks against crease-pair swings in the script.
- `knockbacks`, `swings`, `tiltedSwings`, `guardBlocks`: the counts of the records below; a swing is tilted when its
  attacker's ground differs from world up by more than 1e-3 in a component.
- `maxSwingUpError` and `weaponOutOfPlaneMax`: the largest values over all swings of the two swing measures (§7). The
  second is the shipped build's counterpart of the initiative's design §7.2 (there modelled at up to 9.2e-7).
- `extraCasts`: the sweeps the shipped attack ground sample issues over the run. Before every tick the test calls
  `brawlerMovementSimulation::sampleAttackGround` on each character's state, through a
  counting wrapper around the scene's query (`CountingQuery`); that is one cast per character per
  tick, the cost the flip added to `SimulatableBrawler::integrate` (0 with the knob off, before task 10; task
  14 deleted the knob).
- `inbound`: per character, ticks it was hit and ticks its own swing was guard-blocked (`wasHitThisTick`,
  `wasGuardBlockedThisTick`).
- One `|` group per shot (`shot.`): its spawn tick, ticks alive, ticks over the face interior, its height above the face
  plane on the first of those (`clearanceM`) and the largest change of that height later (`clearanceDeviationM`), and
  its outcome from the shooter's detection log (hits, blocks, target).
- One `|` group per melee stun (`stun.`): attacker, target, tick, the target's Unsupported ticks and largest horizontal
  distance from where it stood, over its window (below).
- One `|` group per knockback, in tick order. A knockback is a tick on which the target's inbound hit is a Knockback;
  the attacker is the character whose detection log names that target on that tick. Its **window** runs from the hit to
  the tick before the target's next teleport or next hit, or to the end of the run. Over the window:
  `travelM`, the 3-D distance from the target's position on the tick before the hit to its position at the window's
  end; `unsupportedTicks`; `firstUnsupportedTick` and `speedIntoItMS`, the first Unsupported tick and the target's
  speed at the end of the tick before it (the velocity that tick's lift test read); `apexM`, the greatest height above
  that launch point (on the slope this includes the slope's own rise); `peakClearanceRiseM`, the greatest rise of the
  capsule's distance to the nearest exposed ground over its value at the launch point; `airtimeS`, the length of the
  first run of Unsupported ticks; `lockoutEndTick`, the first tick not in HitFlinch; `minClearanceM` and
  `restClearanceM`, the capsule centre's distance to the nearest exposed ground (the slope face, the landing beyond the
  concave crease, the plateau beyond the convex crease), its least value in the window and its value at the launch
  point. `dir` is the inbound hit direction in the (forward, left) chart, `dirWorld` the 3-D direction the movement writes.
  The next field names the **crossing** and gives its value: the dot of `dirWorld` with the target's movement ground on
  the hit tick, which is `brawlerMovementSimulation::attackGroundNormal` of the probe's hit when the probe is walkable
  and world up otherwise, re-taken from the target's state before the tick exactly as the movement's step 2 takes it
  (since task 13b both use that helper; before it the test and step 2 took the exact face with no walkable test).
  Since og-ground-aligned-attacks task 33 the test reads it as step 2 does, from
  `brawlerMovementSimulation::estimateSupport`: its `faceNormal` when `eligible`, else world up. Under
  `SupportLaw::Legacy` those are the same two values as above; under `SupportLaw::V2`, the shipped law since task 36,
  they are the v2 estimate's.
  `leaves` above `kMinLaunchSine` (sin 2°), `enters` below its negative, `inPlane` between.

## 4. The recordings

**Support law v2 (task 36), the current golden.** Task 36 made `SupportLaw::V2` the movement's default (og-brawler's
movement rationale, §36) and re-recorded. The new recording equals the V2 run task 35's print compared with the old
one: against task 14b's recording it differs in 2,965 field-samples, the first at tick 121
(`G.body5.CharacterCapsule.position`), none of them signed-zero only, and the Legacy print (§2.1) reads the same
count against the new one. Only D, F and G differ (5, 11 and 8 fields); A, B, C and E are bit-identical on every
sample. Each difference ends at that character's next teleport (`GoldenTraceSlopeScenario-rationale.md` §4), so the
divergence from tick 121 is G's, and each later one starts again from the recording's state. By mechanism:

* **The frame normal on the face (GJK to the exact face).** No recorded value moved. On this face the sweep normal
  is already within 7.51e-8 of the face (Legacy's `maxNormalDifference`; V2's is 1.55e-8), far below the 1e-4 rad
  the initiative's support-law design allowed for. A's slides of B and C along the contour (120, 300) and of E up the
  fall line (520), and every rest on the face, are bit-identical.
* **Edge ticks (the convex rim).** F stands 0.1 m from the plateau's edge from slot 5 on. Its probe there is an edge
  contact, and the V2 estimate takes the slope's face from the centre probe where Legacy's `attackGroundNormal` gave
  world up. So F's guard follows the face (`F.body1.GuardAxis.rotation` on 645–990 and 1045–1100), its two convex swings are
  tilted (`F.radial.initialAimAngle` and `.initialAimRotationAxis` from 680, `F.body0.WeaponAxis.rotation` and
  `.angularVelocity` on 680–715 and 860–895; `tiltedSwings` 11 → 13), and their tangents at G change
  (`G.hit.inboundDirection` at 700 and 880).
* **A slide's last ticks (the event-driven transitions).** D's fall-line slide from A's hit at 410 no longer leaves the
  face for one tick at 438, where Legacy's per-tick lift test read the hover servo's few cm/s on a 1.96 m/s slide. D's
  fields differ from 440 on, by at most 1.03 cm (its rest), until its teleport at 1316. This is the slope's form of the flat
  golden's knockback flicker, which the one write removed there (`BrawlerJoltGoldenTraceTest-rationale.md` §3).
* **Crease ticks.** Every crease knockback changes (§7): G on 121–240 (F's hit at 120 leaves the slope by its
  direction and launches, 95 Unsupported ticks against 1; G's guard pose in flight on 135–214), G on 300–345 (300
  enters and is redirected along the landing: no hop), F on 410–460 (410 enters and is redirected up the slope: no
  hop), F on 535–640 (520 leaves: 13 Unsupported ticks against 11), G on 700–820 (700 now leaves: 119), G on 880–930
  (880 now enters at the plateau and is redirected along it, off the edge: 34 from 882), F on 990–1040 (990 now
  enters and is redirected up the slope, over the crest: 50 from 991, cut by slot 8's teleport, §7) and F on
  1100–1480 (1100 leaves and falls off the edge: 43; F is not teleported again).

The line now reads `slopeTicks=8204 normalDiffersTicks=0 maxNormalDifference=1.55e-08 ... tiltedSwings=13 ...
extraCasts=10768` (it was `8284`, `7.51e-08`, `11` and `10360`). The 408 extra casts are the centre probe's sweeps
on edge contacts, issued by the attack sample's estimate.

**No flat branch in the attacks (task 14b).** Task 14b folded site 2a (the machine's radial
initial conditions take the quaternion round trip on flat ground too, `DAttackMachineSimulation-rationale.md` §16) and
site 2b (`wouldGuardBlock` always projects into the guard's plane, `DAttackRadialSimulation-rationale.md` §7), as the
user ruled on 2026-11-02. Against task 13b's recording only the swings that F and G start on the flat ground differ: six
fields on 659 samples, none of them signed-zero only, all from site 2a:

* `F.radial.initialAimAngle` (161 samples, ticks 680–1480) and `G.radial.initialAimAngle` (219, ticks 390–1480), at
  most 2.38e-7 rad;
* `F.radial.initialAimRotationAxis` (161, ticks 680–1480) and `G.radial.initialAimRotationAxis` (103, ticks 970–1480),
  at most 5.96e-8;
* `F.body0.WeaponAxis.rotation` (13, ticks 680–880), at most 2.68e-7, and `F.body0.WeaponAxis.angularVelocity` (2,
  ticks 680 and 860), at most 1.19e-7 rad/s: the weapon pose the radial builds from that pair, and its rate.

No hit, block, machine, movement or capsule field changed, and no swing on the face changed (the face is not flat, so it
took the composite before). The new recording equals, value for value, the one task 14b's part 1 recorded with site 2a
folded alone, so site 2b changed no value. The flat golden changed too (`BrawlerJoltGoldenTraceTest-rationale.md` §3).

**One ground rule (task 13b).** Against task 10's recording only F differs: six fields
(`F.body5.CharacterCapsule.position` and `.linearVelocity`, `F.movement.velocity`, the weapon and guard axes'
positions, `F.movement.flags`), on the sampled ticks 990–1040 and 1100–1480 and nothing between (F is teleported
back at slot 8's start). No other character changes. The line differs in `slopeTicks=8284` (2 more `Supported`
ticks over the face interior, F's) and in the two convex hits of G on F, now `inPlane` (§7):

    | G.>F.tick=990 ... inPlane=0.0000 travelM=4.9484 unsupportedTicks=43 firstUnsupportedTick=990 ... apexM=0.8434
      peakClearanceRiseM=0.8398 airtimeS=0.7167 ...
    | G.>F.tick=1100 ... inPlane=0.0000 travelM=5.7493 unsupportedTicks=23 firstUnsupportedTick=1100 ...
      peakClearanceRiseM=0.5602 airtimeS=0.3667 ...

(task 10's: 990 `leaves=0.8503 travelM=5.1714 unsupportedTicks=16 apexM=0.0098 peakClearanceRiseM=0.0063`, 1100
`enters=-0.8503 travelM=5.9959 unsupportedTicks=24 firstUnsupportedTick=1101`). The flat golden is byte-unchanged.

**The flip (task 10):**

    [GoldenTrace.slope] idleDriftAM=0 slopeTicks=8282 normalDiffersTicks=0 maxNormalDifference=7.51e-08
      unsettledPhaseStarts=0 creaseHits=8/8 knockbacks=12 swings=17 tiltedSwings=11 maxSwingUpError=1.79e-07
      weaponOutOfPlaneMax=1.38e-06 extraCasts=10360 guardBlocks=3 inbound=A.1/3 B.1/0 C.1/0 D.2/0 E.1/0 F.4/0 G.4/0 ...

(wrapped and shortened; the impl notes of task 10 quote it whole). Read with the scenario's slot table:

- A's side swings now hit B (up the slope, tick 120) and C (down it, 300): their planes follow the face. B and C slide
  5.17 m along the contour, Supported throughout, and are put back by a teleport before their next role.
- B's guard blocks A's overhead and C's guard both side swings (A's three blocked swings); A's projectile is blocked by
  C's guard and B's projectile hits A, each 1.4729 m above the face to within 4e-7 m.
- D and E slide 5.17 m along the fall line, down and up; E is Supported throughout, D for all but one tick (§7).
- All eight crease swings land. Their outcomes are listed in §7.
- `extraCasts=10360` is 7 characters × 1480 ticks.

**The baseline (task 3, re-recorded at task 3b):**

    [GoldenTrace.slope] idleDriftAM=0 slopeTicks=8148 normalDiffersTicks=0 maxNormalDifference=2.98e-08
      unsettledPhaseStarts=0 creaseHits=6/8 knockbacks=8 inbound=A.0/1 B.0/0 C.0/0 D.1/0 E.1/0 F.4/0 G.2/0
      | A.>D.tick=410 dir=(-1.0000,-0.0000) travelM=5.1714 unsupportedTicks=1 apexM=0.0000 ...
      | A.>E.tick=520 dir=(1.0000,-0.0000) travelM=5.0580 unsupportedTicks=22 apexM=2.5293 ...

With the swings built horizontally the side swings at B (1 m above A) and C (1 m below) hit nothing: the ring stayed at
A's height, beyond the band the detector allows (`GoldenTraceSlopeScenario-rationale.md` §5). The swings into C's guard
reached nothing either, and the two projectiles flew level and neither hit nor were blocked. D, thrown down
horizontally, left the face for one tick; E, thrown up, hopped (22 Unsupported ticks, apex 2.53 m). Six of eight crease
swings landed. That baseline had no teleports for B, C and D and no overhead by C (task 10 added them, the scenario
rationale §3 and §4), so its numbers compare with the flip's only where those changes do not reach.

## 5. Recording and re-recording

From the project root, after building OGBrawlerTests:

    mkdir -p "$TMP/golden" && OG_GOLDEN_OUT="$TMP/golden" OG_GOLDEN_SOURCE="<repo and submodule HEADs>" \
      Binaries/Win64/OGBrawlerTests/OGBrawlerTests.exe "BrawlerJoltSlopeGoldenTrace.Record"

Then copy the two files it wrote, unedited, into `GoldenTrace/`, touch `BrawlerJoltSlopeGoldenTraceTest.cpp` (the
golden is included through `__has_include`, `BrawlerJoltGoldenTraceTest-rationale.md` §3), rebuild, and run
`"[GoldenTrace]"` twice. At task 3 the recording printed `samples=537 fields=420 values=451080` (3.8 MB of golden),
and the envelope's largest position metric was 0.0041 cm with no discrete divergence. Task 3b's re-recording printed
the same counts, `goldenBytes=3829129 envelopeBytes=39169`, the same envelope maximum and again no discrete divergence.
Task 10's printed the same counts, `goldenBytes=3929364 envelopeBytes=39162`, the same position maximum, a planar
maximum of 5.96e-8 (one float step of a chart direction in a nudged run) and no discrete divergence. Task 13b's printed
the same counts, `goldenBytes=3929549 envelopeBytes=39171` and the same two maxima. Task 14b's printed the same counts,
`goldenBytes=3927795 envelopeBytes=39163`, the same two maxima, and an envelope that differs from task 13b's only in its
source string. Task 36's printed the same counts, `goldenBytes=3931012 envelopeBytes=39166`, the same two maxima
(position 0.0041 cm, planar 5.96e-8) and no discrete divergence. A one-ULP edit of
one recorded value fails exact mode at that value's first sample.

Re-record when a change alters the slope behaviour on purpose or the recorded fields change. Never edit either file by
hand.

## 6. The crease reach sweep (`CreaseReachTest.cpp`, task 3b)

`BrawlerJoltSlopeCreaseReach.PrintsTheConnectingWindows` (`[GoldenTrace]`, no golden) predicts, for each crease, role and
swing sense, which placements of the crease pair connect, with the swings built horizontally and with the ground-aligned
initial conditions. It runs no swing, and it selects both modes itself, so the knob (deleted in task 14) never changed it.

- **Rest positions are measured.** `SettleScenario` puts characters `kTeleportHeight` (1.2 m) above face points on both
  sides of both creases, 0.05 to 1.00 m from the crease in 0.05 m steps (80 points, eight per scene, 1.5 m apart along
  the contour), plus the scenario's own four crease points, and runs each scene for 60 ticks with no input through
  `goldenTraceScene::Scene` on the slope's statics. Each character must end Supported and moving at most 1 mm per tick
  horizontally. The test keeps the capsule's offset from its face point, its attack ground sample
  (`sampleAttackGround` with the ground-aligned path selected) and its movement probe face (`probeGround`: the hit normal
  when walkable, else world up). Inside the face nothing changes along the contour, so a measured offset is moved to
  the pair's own x (F at −1, G at +1).
- **The initial conditions are production.** Each swing's comes from `groundAlignedMachineRig::integrateOnce`, a direct
  `integrate3` call, with the scenario's own `swing` input row for that side: horizontal with a default `GroundSample`,
  ground-aligned with the attacker's measured sample. The test requires the machine to start that side's sequence.
- **The band is the detector's own test.** `reachOf` calls `brawlerHitDetection::capsuleOverlapsAttackBand`, the
  detector's band test since og-ground-aligned-attacks task 20, with the pivot at capsule + `attachmentOffset`, the axis
  from the IC and the sequence, the ring from `getAttackCircle` and the target capsule from the movement StaticData.
  It then skips a centre with no on-plane direction, as the detector does, calls `getAttackSegment` on the on-plane
  direction and requires a damaging segment. Until task 20 it re-typed the point test's lines, which could drift from
  the detector; that is why the call replaced them. Three things are assumed, not run: the weapon body sits at capsule + `attachmentOffset` (the
  radial's integrate puts it there; the world step can move it slightly before the next detection), the overlap query
  reports the target (its capsule is about 2.1 m from the pivot, inside the 3 m query sphere), and the weapon reaches the
  target's segment (a damaging segment that lasts at least one tick is visited; the test requires that duration).
- **The launch flag is the design's rule, re-typed.** The tangent is the detector's `swingTangentAt` (the axis crossed
  with the on-plane direction, signed by the sequence's angular velocity in the middle of the target's segment). It
  counts as a launch when its dot with the target's movement probe face exceeds sin 2° (the initiative's design §3.6).
  It was a prediction for the knockback tasks; §7 has what the flipped scenario measures.
- **The lines.** `[GoldenTrace.creaseRest]`, per crease and side: the capsule's height above its face point and each
  sample's dot with the face normal. `[GoldenTrace.creaseReach]`, per crease, role, sense and mode: the connecting pairs
  as slope-distance:flat-distance windows, and in the ground-aligned mode the launching ones. `[GoldenTrace.creaseBest]`,
  per crease and number of swings connecting in both modes: the placements with the most predicted launches.
  `[GoldenTrace.creaseScenario]`: every swing at the scenario's own placement.
- **The checks.** At the scenario's placement: concave 4 swings connecting horizontally and 4 ground-aligned, convex 4
  and 4, and at least one convex launch. Moving the crease pair without re-running the sweep fails them. With task 3's
  0.3 m at the convex crease they failed: convex 2 horizontally, 0 ground-aligned, no launch. The flip measured all 8
  connecting, as predicted. Until og-ground-aligned-attacks task 20 the concave check was 2 horizontally: the slope-side
  attacker's horizontal swing missed G with G's capsule centre 0.512 m below the swing plane (`axisM -0.5120`), outside
  the point test's 0.35 m, while G's capsule reached into the band. The capsule test connects both, and the line now
  reads `today=8/8 flip=8/8 concave(today 4 flip 4 launching 2) convex(today 4 flip 4 launching 1)`.
- **Since og-ground-aligned-attacks task 36** the attack sample is the support law v2's estimate. A slope-side
  capsule 0.05 to 0.20 m from the convex crease samples the slope's face there (`attackSampleDotFace` 1.0000; it was
  0.8660, world up, under Legacy), so the slope-side attacker's ground-aligned up swing at the convex crease launches
  from all 400 placements (320 under Legacy, which needed F 0.25 m or more from the crease), the convex
  `[GoldenTrace.creaseBest]` line lists 400 placements (320), and the scenario line reads `convex(today 4 flip 4
  launching 2)`. The concave crease is unchanged. The case's checks did not move.

## 7. The behaviour cases (task 10)

Each case reads the one run of §2 and asserts what the ground-aligned attacks must do on the slope. With the knob set
back to off every one of them failed (task 10's seen-failing run, quoted in its impl notes; task 14 deleted the knob): the swings report
`|R_ic kUp - n| = 0.5`, A hits neither B nor C, C's guard blocks nothing, the shots drift 3.47 m against the face, D's
and E's knockbacks leave the face, and the overhead hits nobody.

- **`SwingsFollowTheGround`.** A swing starts on the tick the radial's `currenSequenceId` becomes a real sequence other
  than the one before. At that tick the swing's frame up, `mat3(rotate(initialAimAngle, initialAimRotationAxis)) · kUp`
  (the `up=` of `[Radial.setInitialConditions]`), must equal the attacker's ground normal within 1e-6 in each component;
  the normal is `sampleAttackGround` with the ground-aligned path selected, taken from the attacker's state before the
  tick, which is the state the tick samples. Through the swing (every tick the sequence stays active) the weapon body's
  directions perpendicular to the sequence's local rotation axis must stay within 1e-5 of the plane perpendicular to
  that axis's world image at the start (`bladeOutOfPlane`: the larger dot of the two directions, turned by the body's
  recorded rotation, with that axis). A's seven swings on the face must be tilted, so a flat run cannot pass. Measured:
  17 swings, up error at most 1.79e-7, out of plane at most 1.38e-6. 13 are tilted since task 36: F's two at the
  convex crease take the slope's face from the support estimate's centre probe. Under Legacy they sampled the
  plateau's up, and 11 were tilted.
- **`SideSwingsHitUpAndDownTheSlope`.** A knockback from A on B in slot 1 and on C in slot 2.
- **`GuardsBlockInTheGroundPlane`.** In slot 5 (B guards, A's overhead) and slots 6 and 7 (C guards, A's right and left
  swings) A's detection log counts a guard block and A lands no knockback on the guard.
- **`ProjectilesFollowTheSlope`.** Exactly two shots. For each, over at least 3 ticks above the face interior, its
  height above the face plane changes by at most 1e-4 m; and A's shot is blocked with C as the target. Both shots end
  after 4 ticks (the target is 2 m away), at 1.4729 m above the face, deviating by at most 3.9e-7 m.
- **`KnockbacksFollowTheAttackersSwing`** (restated for support law v2 at task 36).
  - D's and E's fall-line knockbacks are `inPlane`, and neither has an Unsupported tick. Until task 36 the case allowed
    one late Unsupported tick, entered below 15 % of the knockback speed, for D's tick 438 of a hit at 410: Legacy's
    per-tick lift test (`detachesFromSupport`, task 13) read the hover servo's few cm/s on a 1.96 m/s slide as more
    than 2° out of the face. V2 changes support only on events (og-brawler's movement rationale, §34), and D stays
    Supported.
  - Crease hits, by the crossing on the hit tick:
    - Every `leaves` hit detaches on its hit tick and stays Unsupported for at least 10 ticks, two at each crease:
      F at G (120, 95 ticks) and G at F (520, 13) at the concave crease, F at G (700, 119) and G at F (1100, 43) at
      the convex one. G's hit on F at 1100 is checked: `leaves`, 43 ticks, a peak clearance rise of 1.6450 m within
      5e-5 m.
    - Both concave `enters` hits, F at G (300) and G at F (410), have no Unsupported tick: the redirect sends the
      velocity along the face at `V · cos θ` (the movement rationale, §35). G's hit on F at 410 is checked: 3.9861 m
      of travel and a 1.8964 m apex, each within 5e-5 m. F rests at its hover with a clearance that prints 0.100000
      but sits a rounding above `rideHeight`, so this holds only while the redirect also covers a written hit tick
      above the hover; without that it reads 2.6684 m and 1.2414 m.
    - Both convex `enters` hits, F at G (880) and G at F (990), are redirected along the ground they hit and leave
      the convex crest after their hit tick for at least 10 ticks (34 from 882, 50 from 991). At 990 F is thrown up
      the slope at 17.42 m/s and flies off the crest: the ski jump of the initiative's support-law design. It is
      checked: `enters`, first Unsupported on 991, a peak clearance rise of 3.7524 m within 5e-5 m. ⚠ That flight
      does not end inside slot 7. On 1040, the slot's last tick, F is still rising (0.34 m/s), and slot 8's teleport
      takes it at 1041, so 50 ticks is what the slot leaves, not the airtime, and the true apex is about 6 mm higher.
    - No crease hit is `inPlane`.
  - Under Legacy (the `law=legacy` line) the concave hit at 120 landed on the slope after one tick, the two concave
    entering hits hopped (300: 18 ticks, a 0.28 m hop; 410: 31 ticks, a 1.96 m rise), and since task 13b all four
    convex hits were `inPlane`. At F's 0.1 m from the edge the sweep finds an edge contact, (0, 0.977, 0.211), and
    `attackGroundNormal` gave world up there, so the movement and this test took the plateau's up as F's ground and
    each convex hit detached from its own hit tick for at least 10 ticks (15, 36, 43 and 23). V2's estimate takes the
    slope's face there from its centre probe (the movement rationale, §33).
  - C's overhead on D in slot 10 is the scenario's one melee stun: D stays Supported and does not move (0 Unsupported
    ticks, 0 m). The overhead's row is a Stun with no knockback, so its downswing tangent writes nothing.

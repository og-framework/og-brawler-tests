<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerJoltSlopeGoldenTraceTest — rationale

Source: `BrawlerJoltSlopeGoldenTraceTest.cpp`. This file has no guards document: nothing in it needs a `⛔` tag. The
scenario is in `GoldenTraceSlopeScenario-rationale.md`, the harness in `GoldenTraceScene-rationale.md`, the format in
`GoldenTraceFormat-rationale.md`. The test runs the slope scenario through `goldenTraceScene::Scene` and compares it
with `BrawlerJoltSlopeGolden.inl` and `BrawlerJoltSlopeGoldenEnvelope.inl`. They were recorded at og-ground-aligned-attacks
task 3 as the baseline with the attacks built horizontally, re-recorded at task 3b, when the convex crease pair moved
(`GoldenTraceSlopeScenario-rationale.md` §5), and re-recorded at task 10, the flip that turned the ground-aligned attacks
on (`brawlerGround::kGroundAlignedAttacks`), and again at task 13b, which gave the movement the attack's ground rule
(only F's convex-crease hits changed, §4). Since task 10 five behaviour cases (§7) state what the flip is for. §6
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
  `brawlerMovementSimulation::sampleAttackGround` with its default (shipped) knob on each character's state, through a
  counting wrapper around the scene's query (`CountingQuery`); with the knob on that is one cast per character per
  tick, the cost the flip adds to `SimulatableBrawler::integrate`, and 0 with it off.
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
  `leaves` above `kMinLaunchSine` (sin 2°), `enters` below its negative, `inPlane` between.

## 4. The recordings

**One ground rule (task 13b), the current golden.** Against task 10's recording only F differs: six fields
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
the same counts, `goldenBytes=3929549 envelopeBytes=39171` and the same two maxima. A one-ULP edit of
one recorded value fails exact mode at that value's first sample.

Re-record when a change alters the slope behaviour on purpose or the recorded fields change. Never edit either file by
hand.

## 6. The crease reach sweep (`CreaseReachTest.cpp`, task 3b)

`BrawlerJoltSlopeCreaseReach.PrintsTheConnectingWindows` (`[GoldenTrace]`, no golden) predicts, for each crease, role and
swing sense, which placements of the crease pair connect, with the swings built horizontally and with the ground-aligned
initial conditions. It runs no swing, and it selects both modes itself, so the knob does not change it.

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
- **The band is the detector's expressions, re-typed.** `reachOf` repeats the lines of `detectRadialHits` from the hit
  direction to the in-circle test, with the pivot at capsule + `attachmentOffset`, the axis from the IC and the
  sequence, and the ring from `getAttackCircle`; it then calls `getAttackSegment` on the on-plane direction and requires
  a damaging segment. Three things are assumed, not run: the weapon body sits at capsule + `attachmentOffset` (the
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
- **The checks.** At the scenario's placement: concave 2 swings connecting horizontally and 4 ground-aligned, convex 4
  and 4, and at least one convex launch. Moving the crease pair without re-running the sweep fails them. With task 3's
  0.3 m at the convex crease they failed: convex 2 horizontally, 0 ground-aligned, no launch. The flip measured all 8
  connecting, as predicted.

## 7. The behaviour cases (task 10)

Each case reads the one run of §2 and asserts what the ground-aligned attacks must do on the slope. With the knob set
back to `false` every one of them fails (task 10's seen-failing run, quoted in its impl notes): the swings report
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
  17 swings (11 tilted; F's at the convex crease sample the plateau), up error at most 1.79e-7, out of plane at most
  1.38e-6.
- **`SideSwingsHitUpAndDownTheSlope`.** A knockback from A on B in slot 1 and on C in slot 2.
- **`GuardsBlockInTheGroundPlane`.** In slot 5 (B guards, A's overhead) and slots 6 and 7 (C guards, A's right and left
  swings) A's detection log counts a guard block and A lands no knockback on the guard.
- **`ProjectilesFollowTheSlope`.** Exactly two shots. For each, over at least 3 ticks above the face interior, its
  height above the face plane changes by at most 1e-4 m; and A's shot is blocked with C as the target. Both shots end
  after 4 ticks (the target is 2 m away), at 1.4729 m above the face, deviating by at most 3.9e-7 m.
- **`KnockbacksFollowTheAttackersSwing`.**
  - D's and E's fall-line knockbacks are `inPlane`, and neither leaves the face on its hit tick or during the fast part
    of its slide: either no Unsupported tick at all, or the first one comes after the hit tick and is entered below 15 %
    of the knockback speed. (The speed is the one on the tick before, so a hop on the hit tick itself would be read at
    the pre-hit speed; the tick condition is what catches it: knob off, E is Unsupported for 22 ticks from its hit tick.)
    Measured: E none; D one, on tick 438 of a hit at 410, two ticks before its lockout ends, entered at 1.96 m/s. That
    tick's lift test (`detachesFromSupport`, task 13) read a velocity of 1.96 m/s whose angle out of the face was above
    2°: the hover servo's vertical correction is a few cm/s, which a 20 m/s slide hides and a 2 m/s one does not. The
    case states the slide; the end-of-decay tick is the movement law's, routed to the initiative's lead.
  - Crease hits, by the crossing on the hit tick. Every `leaves` hit detaches on the hit tick, and the concave crease
    has a `leaves` hit that stays Unsupported for at least 10 ticks (G at F, tick 520, 11 ticks). The convex crease has
    none since task 13b (next bullet). The concave `leaves` hit of F at G (120) is Unsupported for one tick only: G
    is thrown from the landing along F's swing plane, which is the slope's, so the next tick its probe finds the slope with the velocity
    in it, and it slides 5.12 m up the face. Every `enters` hit is Supported on its hit tick (no launch there); the
    concave crease has two. They do not stay Supported: F at G (300, the landing) leaves 2 ticks later for 18 ticks (a 0.28 m
    hop); G at F (410, into the slope) leaves 20 ticks later for 31 (a 1.96 m clearance rise). These two are the
    into-ground behaviour the initiative's design OQ14 left emergent (a dip, then a hop), for the user's G3 check; their
    travel and least clearance are in the line.
  - ⚠ The convex crease, since task 13b. F stands 0.1 m from the edge. Its sweep's GJK normal there is an edge normal,
    (0, 0.977, 0.211), which is walkable, and its exact face is the slope box's upper end face, (0, 0.5, −0.866), 60°
    from world up, which is not. Task 10's movement took that end face as F's ground, so G's horizontal swings counted
    as `leaves` (990) and `enters` (1100) at ±0.8503, while the attack sample gave F world up. Since task 13b the movement
    and this test take the ground from `brawlerMovementSimulation::attackGroundNormal`, which gives world up there, so
    all four convex hits are `inPlane` (F's on G at 700 and 880, as before, and G's on F at 990 and 1100): **the
    convex `leaves` launch does not occur at this placement.** No character was moved to force one. The case asserts
    the convex outcome as measured:
    no `leaves` or `enters` hit there, four `inPlane` hits, each `Unsupported` from its own hit tick for at least 10
    ticks (15, 36, 43 and 23; 700 and 880 are unchanged from task 10). F's detach is the flat-ground law's: F rests
    with a vertical velocity of +1.11e-6 m/s (the hover servo's residue), and on a hit tick the world-up lift test `dot(velocity, up) > 0` reads it. The
    knockback then runs in the tangent frame of the 12.2° edge normal, so thrown onto the plateau (990) F rises 0.84 m
    and is airborne 0.72 s, and thrown down the slope (1100) it falls off the edge for 23 ticks. 990's measurements
    equal those of task 10's knob-off recording of this scenario. With step 2 back on the pre-task face the case fails
    (`1101 == 1100`: the 1100 hit stays `Supported` on its hit tick).
  - C's overhead on D in slot 10 is the scenario's one melee stun: D stays Supported and does not move (0 Unsupported
    ticks, 0 m). The overhead's row is a Stun with no knockback, so its downswing tangent writes nothing.

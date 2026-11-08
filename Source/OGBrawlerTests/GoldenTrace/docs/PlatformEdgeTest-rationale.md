<!-- SPDX-License-Identifier: BUSL-1.1 -->
<!-- lint-external-ref: PlatformEdge.KnockbacksAcrossTheImportedPlatformRims -- a RETIRED Catch2 case -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records what it measured (the Legacy run) -->
<!-- lint-external-ref: supportLawName -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
<!-- lint-external-ref: SupportLaw::Legacy -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
<!-- lint-external-ref: SupportLaw::V2 -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
<!-- lint-external-ref: gSupportLaw -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
<!-- lint-external-ref: SupportLawScope -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
<!-- lint-external-ref: EdgeScenario::configureStaticData -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
<!-- lint-external-ref: lawTag -- a RETIRED token -- og-ground-aligned-attacks task 37 deleted it with the Legacy support law; the prose records its history -->
# PlatformEdgeTest — rationale

Source: `PlatformEdgeTest.cpp`, with its data in `QuarterCylinderPlatform.inl`. This file has no guards document: nothing in
either file needs a `⛔` tag. The harness is described in `GoldenTraceScene-rationale.md`.

`PlatformEdge.KnockbacksAcrossTheImportedPlatformRimsUnderV2` (`[GoldenTrace]`, no golden; until og-ground-aligned-attacks
task 37 also `PlatformEdge.KnockbacksAcrossTheImportedPlatformRims`, the Legacy law's run, §8) is the measuring rig of the
ground-aligned attacks initiative's task 23. It knocks a brawler across the rims of the dev level's 30° slope platform
with real melee swings, records every tick of the target's movement, and prints what changed its vertical speed. It
asserts only that its own instruments agree with the movement law (§5). It is a baseline: tasks 14b and 24 re-run it and
compare its printed lines (§7).

## 1. The platform data

- **What it is.** `quarterCylinderPlatform::kHulls` holds the two convex hulls of one platform of
  `/Game/ThirdPerson/Maps/ThirdPersonMap`, as the level was saved on 2026-11-02: the actors `SM_QuarterCylinder21` and
  `SM_QuarterCylinder22`. Both use `SM_QuarterCylinder` at scale (20, 20, 0.1) and the same location. One is pitched 30°,
  the other is yawed −90° and rolled −30°, so together they form one half-disc slab whose straight edge is the lowest
  line.
- **Where the numbers come from.** The mesh's simple collision (one convex element's VertexData, 10 vertices) was exported
  as ASCII FBX from a UE Python commandlet. The FBX exporter writes each vertex as (X, −Y, Z). The actors' world
  transforms were read from the same commandlet. No production file was edited and no game binary was built.
- **The conversion is the importer's** (`UEStaticGeometryImporter.cpp`, `importBody`, and `UEFrameConversion.h`):
  - each vertex is scaled per axis by the component scale, then converted with `vectorToSim`: UE (X, Y, Z) cm becomes
    sim (Y, Z, −X) / 100 m, narrowed to float;
  - the hull's transform is the component's rotation and translation with unit scale: `vectorToSim` for the
    translation, `rotationToSim` for the quaternion, so sim (w, x, y, z) = UE (w, −y, −z, x);
  - the test builds `localToWorld` as translate × `mat4_cast`, which is what `composeTransform` does at unit scale.
- **One gap.** The importer reads the Chaos convex's vertices, not VertexData. For this 10-point prism every point
  is a hull vertex, so the two sets are expected to be the same points, but the rig did not read the Chaos mesh. Jolt
  builds its own hull from the points either way, with a convex radius of 5 % of the 0.1 m thickness (5 mm,
  `JoltStaticWorldBuilderOptions`).
- **The shape, in the platform's own chart.** s runs up the slope from the straight edge, w across it.
  - The collision is not a round arc. Each quarter is a square with its far corner cut at 45°. So the platform is
    half an octagon whose edges touch the visual circle at 0°, 45° and 90°.
  - The rims are: the low straight edge (s = −0.02 m), two tip faces (w = ±10.02 m, s from 0 to 4.155 m), two chamfers
    (s + |w| = 14.175 m), and the top rim (s = 10.02 m, |w| ≤ 4.155 m).
  - The tip faces are vertical walls, 90° from up. The chamfers are 69.3° from up, the top rim 60°. The low edge's
    face points 30° below horizontal, so it overhangs.
  - The slab is 0.10025 m thick: its points span local y from −0.000125 to 0.100125 m, so the top face sits 0.100125 m
    above the hull origin. Its top face normal is (0, 0.866, 0.5): 30° toward +Z.
  - The two hulls overlap by 2.25 cm across the seam (w = 0). Their top faces agree to 1e-8.

## 2. The scene

- `EdgeScenario` is a two-character `goldenTraceScene::Scenario`: a target (id 1) and an attacker (id 2).
  - The statics are the two hulls plus a 60 m floor box with its top at y = 0. The floor only ends falls; it is not
    the level's floor.
  - The statics use the categories, friction and restitution of `goldenTraceScenario::staticBox`.
  - Static data is `goldenTraceScenario::scenarioStaticData`: the shipped movement, attack and reaction tables. The four
    melee rows knock back at 20 m/s.
- `goldenTraceScene::Scene` merges all statics into one Jolt body, so a hit's body id cannot tell the two hulls apart.
  The rig names the hull by the side of the seam the contact point lies on (`hullSideOf`; 2 = inside the overlap).
- A scenario's functions are static, so `EdgeScenario::set` stores the run's placement in static members before each
  `Scene` is built. The cases run one after another.
- Both characters teleport 1.2 m above their face point at tick 1 and settle for 60 ticks. The attacker swings at tick 61
  (`kSwingTick`); the hit lands at tick 85 in every run. The run ends at tick 200.

## 3. Placements

- **Sites** (`kSites`, 15): the top rim (off the seam and on it), both chamfers at their midpoints, both tip faces at
  s = 0.5 and 2.5, both tip corners, the low edge (w = ±3 and on the seam), and two crossings of the seam at s = 5.
- **Rest distance** (`kDistances`): the target stands d metres inside the rim along the rim's outward normal: −0.15
  (resting past the rim, held up by the edge), 0.3, 1.5, 3.5, 4.5 and 4.9. The larger values reach the rim slowly, since
  a 20 m/s knockback stops after 5 m.
- **Direction** (`kObliqueDeg`): outward along the rim normal, and ±30° oblique.
- **The knockback is production's.** The attacker stands beside the target, perpendicular to the wanted direction, and
  aims at it horizontally. The radial hit's swing tangent is perpendicular to the attacker-to-target line in the
  swing plane, so it points along the wanted direction. Hit routing writes the inbound-hit slice from that tangent, and
  movement's `writesKnockbackVelocity` writes the velocity, exactly as in a match.
- **The attacker search** (`placementFor`). It tries:
  - both sides of the target, at 1.6 m and then 1.2 m;
  - inward shifts of 0, 0.45 and 0.9 m;
  - both swings.
  It keeps the hit whose direction is closest to the wanted one, and requires a cosine of at least 0.82. The printed
  `actualObl` is the angle the knockback really had.
- **A target resting past a raised rim** sits about 0.14 m below the face plane. Until og-ground-aligned-attacks task 20
  its capsule centre was outside the swing band, whose half-thickness is 0.35 m about a pivot 0.3 m above the attacker's
  root, so for d = −0.15 the attacker had to overhang too. Since task 20 a swing hits when the band overlaps the target's
  capsule, and the search's first placement (no inward shift, no flip) reaches it (§7.2).
- **Four placements cannot be reached** and are printed as `[PlatformEdge.unreachable]`: both tip corners at d ≤ 0.3
  outward. All of them are within 1 m of a rim (`kReachableFromDistance`). Before task 30 there were five, from task 30
  seven (§7.1), and task 20 reached the three oblique overhangs (the top rim's seam at +30°, chamferPlus −30°,
  chamferMinus +30°; §7.2).

## 4. The per-tick record and the mirror

- **The probe.** Before each step, `runOnce` copies the target's movement `State` and calls `estimateSupport` (since
  og-ground-aligned-attacks task 32; `probeGround` before) with the movement declaration's bindings. That is the support
  estimate `integrate` makes on that tick, with its one sweep: the query sees statics only, and nothing moves the body
  between the copy and the probe. The record's probe fields are the estimate's `probe`, and its attack ground is the
  estimate's `faceNormal`.
- **The record.** After the step it reads:
  - the support bits, `DerivedState::surfaceNormal` (the basis n), the push-out, and the post-step velocity;
  - the inbound-hit slice and the attack machine's state.
- **The mirror.** `mirrorIntegrate` recomputes the step's velocity from those inputs with the movement's own helpers
  (the `GroundSupport` from `estimateSupport`, then `writesKnockbackVelocity`, `classifySupport`, `buildTangentFrame`, `decomposeVelocity`,
  `moveTowards`, `computeDesiredVelocityUV_ContinuousAccelBrake`) and the servo expression of `integrate`. Since
  og-ground-aligned-attacks task 34 a V2 run took the support law v2's write and classification in place of the
  first two, with the hit-tick channels `currentUV` on every hit tick, as in `integrate`; since task 37 those are the
  only ones, under the two names above. Since
  og-ground-aligned-attacks task 35 a V2 run also applies the redirect into the face (`redirectsIntoFace`, then
  `v −= n · dot(v, n)`) after its classification and before its frame, as `integrate` does, and records the redirect's
  vertical change as `redirectDy` (0 on a tick it does not fire). Since
  og-ground-aligned-attacks task 33 its frame is its own classification's, `SupportDecision::frameNormal`; before, it was
  the recorded `DerivedState::surfaceNormal`. So a frame change inside `classifySupport` or the estimate reaches the
  mirror directly, and `mirrorMismatches` compares it with the stepped velocity. The record still prints
  `DerivedState::surfaceNormal` as n. It splits the vertical change into its parts:
  - `pushDy`: the push-out;
  - `writeDy`: the knockback write;
  - `uvDy`: the in-plane channels' vertical share in the basis n;
  - the servo's clamped term `servoDv`, with its stiffness and damping parts;
  - gravity;
  - the decomposition residual: the channels recomposed minus the velocity they came from (`residualDy`, `residualDv`, and
    `vDotUp`, the second tangent's vertical part). Until task 30 `decomposeVelocity` reproduced the velocity exactly only while that tangent was horizontal, so a
    non-zero residual was velocity the step dropped. Since task 30 it is exact on every frame, and the residual is
    rounding: 0 above 1e-3 m/s on every tick.
  `pushDv` is the push-out's whole change, not only its vertical part.
- **No knob.** The helpers were called with their default template arguments, and the rig never named the compile-time
  knob. So the rig measures whatever law the tree ships, and task 14's deletion of the knob (with those template
  parameters) did not touch it: the summary line was the same before and after task 14.

## 5. What it asserts

- The imported slab is 30° to 1e-3°, its top face sits 0.100125 m above the hull origin (the slab is 0.10025 m thick), and
  its two hulls' top faces agree.
- Every placement at least 1 m from a rim was reached.
- On every tick except the teleport tick:
  - the mirror's velocity equals the stepped velocity to 1e-4 m/s, relative above 1 m/s (`mirrorMismatches`);
  - the stepped support equals the mirror's `classifySupport` support (`supportMismatches`; until task 32 the rig
    computed the Legacy law's walkable-and-not-detached expression itself);
  - the recomputed push-out equals the recorded one (`pushOutMismatches`).
  These three are what make the printed attribution trustworthy. On 2026-11-02 they held on 88,356 ticks of 444 runs,
  after task 30 on 88,953 ticks of 447 runs, and after task 20 on 78,605 ticks of 395 runs.
- Since og-ground-aligned-attacks task 37, the one law's support counters (§8): 214 flips, 17 rim hangs and 308 lip
  ticks over the 266 placements, checked when every site runs. From task 31 to task 36 the pins were the Legacy law's
  (249, 17 and 435, while `supportLawName` was `legacy`); task 37 moved them to the law that ships, the outcome counts
  the rig keeps.
- Since og-ground-aligned-attacks task 34's rework, under V2 only: every reached placement at d = −0.15 on a tip or
  tip-corner site (the six sites whose names start with `tip`, 16 placements) has a run-line `servo` of at most
  `kEventThreshold`, and when every site runs there are exactly 16 of them. These are the targets hanging past a tip,
  on a `Lip`, where a downward hit used to take a +5.0001 m/s servo kick on the hit tick (the movement's rationale §34,
  row 3). The largest is 0.2077 (tipMinusMid +0°): a horizontal hit, `vyEdge` 0, so no kick, only about one tick of
  the servo holding the body against gravity (`g·dt` = 0.1633).
  Seen failing: with the one-sided row 3 restored, `CHECK( e.servo <= kEventThreshold )` failed on tipPlusMid +30°,
  tipMinusMid −30°, tipCornerPlus +30° and tipCornerMinus −30°, each `5.0001f <= 0.25f`; with the site filter
  matching no site, `CHECK( lipHitPlacements == 16u )` failed, `0 == 16`.
- Since og-ground-aligned-attacks task 35's rework 1, under V2 only, by the estimate reference (§8): when every site
  runs there is at most one estimate lift and no estimate down-pull. Every estimate lift, with any site filter, must be
  tipCornerPlus at d = 4.9 m along the fall line (oblique 0°), with an `estimateRedirect` of 0. That one is the slide
  onto the tip's `Lip`, +1.769 m/s, all of it the servo's damping; the redirect into the face does not act on a `Lip`
  (the movement's rationale §35).
  Seen failing: with the redirect's `Face` term deleted, `CHECK( e.estimateRedirect == 0.f )` failed, `1.67458f == 0.0f`
  (anomaly 1.85156, servo 0.176981), and nothing else in `[@og]`: the lift count and the site checks still held, so the
  redirect share is the check that catches that term.
- **Seen failing** (task 23, built and run, then restored byte for byte):
  - poison A removed the servo's clamp from the mirror: `mirrorMismatches=299` and `CHECK( mirrorMismatches == 0u )`
    failed;
  - poison B inverted the detached term of the support check: `supportMismatches=77904` and
    `CHECK( supportMismatches == 0u )` failed.
  Each arm moved only its own counter.

## 6. What it prints, and the definitions

- `[PlatformEdge.run]`, once per placement, only with OG_SUPPORT_LAW_RUNS=1 since task 31 (§8). Its content is unchanged.
  - The rest state at the swing tick: support, clearance, the GJK normal's tilt, and whether the exact face is the top.
  - The departure: the outcome, the edge tick, the departure tick and the report tick.
  - The speed and vertical speed entering the edge, the anomaly, and its parts.
- A **pure-face tick** is Supported, with the exact face and the GJK normal both on the top face (cosine above 0.9999).
- The **edge tick** is the first tick after the last pure-face tick before the final airborne stretch. A run whose
  target is still Supported on its last tick, or that lands on the floor without such a stretch, prints
  `outcome=stays` and class `staysOnPlatform`.
- The **reference** is the velocity entering the edge tick. If the target is already on the edge when it is hit, the
  reference is the written knockback velocity.
- The **anomaly** is the vertical speed at the report tick (the departure plus `kAfterDepartureTicks`, or the tick
  before landing) minus the ballistic value: the reference's vertical speed plus gravity since the edge tick.
  - A rigid body leaving the face at the edge tick has anomaly 0.
  - Class `lift` is above +0.25 m/s, `downPull` below −0.25 m/s.
- The **parts**, summed over the edge tick to the report tick:
  - `servo` (with `stiff` and `damp`);
  - `uvSupTilted` and `uvDetTilted`: `uvDy` on ticks whose basis n is neither world up nor the face normal, Supported or
    not;
  - `uvOther`, `push`, and the summed residuals (`residualDy` and `residualDv` on the run line).
  - Gravity is excluded by construction. A face-following deceleration share counts as anomaly, since a ballistic body
    has none.
- **Counters.** `flips` counts support changes; `detachedWalkable` counts Unsupported ticks whose GJK normal is walkable;
  `disagree` counts ticks whose GJK and exact-face walkability differ; `lipFace` counts ticks whose exact face is not the
  top.
- `[PlatformEdge.site]` and `[PlatformEdge.summary]` total the classes.
- `[PlatformEdge.decomposition]` counts the chosen runs' ticks whose residual exceeds `kResidualTolerance` (1e-3 m/s), how many
  of them were pure-face ticks, and the largest residual. On 2026-11-02 (task 23): 2018 ticks, 0 on the pure face,
  9.2049 m/s. After task 30: 0 ticks, 0.0000 m/s.
- With the environment variable OG_PLATFORM_EDGE_TICKS=1, every run also prints its `[PlatformEdge.tick]` rows and every
  attempt as `[PlatformEdge.attempt]`. OG_PLATFORM_EDGE_SITE=name runs one site.
- `[PlatformEdge.unreachable]` prints only with OG_SUPPORT_LAW_RUNS=1 since task 31. `[PlatformEdge.site]`,
  `[PlatformEdge.summary]` and `[PlatformEdge.decomposition]` always print, with their content unchanged.

## 7. Re-running it for a later task

Task 23's baseline output is kept in the initiative workspace: impl/scratch/ga23/logs/edge_final_runs.txt holds the
run, site and summary lines, and edge_final_ticks.txt beside it adds the tick rows. Its reading is
impl/edge_investigation_ga_23.md in the same workspace. A later task re-runs the case and
compares, per site and distance: the class counts, the anomaly and its parts. The placements and hit ticks are
deterministic, so a changed class is a changed law, not a changed setup. Check the printed `actualObl` and `hit` first:
if those moved, the setup changed.

### 7.1 Task 30's re-run (the exact velocity decomposition)

Task 30 made `decomposeVelocity` exact on every tangent frame (`BrawlerMovementSimulation-rationale.md` §19.1 in
og-brawler). Its before and after outputs are in the same workspace, under impl/scratch/ga30/logs (before_br_og.txt and
after5_br_og.txt, with the run-by-run comparison in run_diff.txt).

The summary and decomposition lines:

```text
before  [PlatformEdge.summary] runs=444 placements=265 unreachable=5 lifts=56 downPulls=23 staysOnPlatform=88 mirroredTicks=88356 mirrorMismatches=0 supportMismatches=0 pushOutMismatches=0
before  [PlatformEdge.decomposition] lossyTicks=2018 lossyOnPureFace=0 largestResidualMps=9.2049
after   [PlatformEdge.summary] runs=447 placements=263 unreachable=7 lifts=66 downPulls=21 staysOnPlatform=79 mirroredTicks=88953 mirrorMismatches=0 supportMismatches=0 pushOutMismatches=0
after   [PlatformEdge.decomposition] lossyTicks=0 lossyOnPureFace=0 largestResidualMps=0.0000
```

The sites that changed (runs, lifts, down-pulls, stays, largest lift, largest down-pull; before → after). The top rim
off the seam, the low edge and the seam crossings are unchanged.

| site | runs | lifts | downPulls | stays | largest lift | largest down-pull |
|---|---|---|---|---|---|---|
| arcTopSeam | 17 | 4 → 3 | 2 | 5 | 0.5422 → 0.5086 | −5.0001 |
| chamferPlus | 18 → 17 | 2 → 3 | 4 → 0 | 6 → 5 | 0.7911 → 0.8925 | −5.0001 → −0.0742 |
| chamferMinus | 18 → 17 | 3 → 5 | 0 → 2 | 6 → 5 | 2.0152 → 0.8924 | −0.1615 → −5.0001 |
| tipPlusLow | 18 | 2 → 3 | 3 | 3 → 2 | 0.3873 → 0.9002 | −5.0001 |
| tipPlusMid | 18 | 4 → 6 | 3 | 5 → 2 | 0.7768 → 0.7804 | −5.0001 |
| tipMinusLow | 18 | 2 → 3 | 3 | 3 → 2 | 0.3846 → 0.8940 | −5.0001 |
| tipMinusMid | 18 | 3 → 7 | 3 | 4 → 2 | 0.7762 → 0.8940 | −5.0001 |
| tipCornerPlus | 16 | 9 | 1 | 0 | 0.7529 → 0.8266 | −5.0001 |
| tipCornerMinus | 16 | 9 | 1 | 0 | 1.4624 → 1.0182 | −5.0001 |

What moved, run by run (256 of the 263 placements in both runs kept their hit tick and knockback direction, so the
setup is the same and a changed class is the changed law):

- **The setup moved only at the chamfer overhangs (d = −0.15).** That is where both characters settle on a rim normal
  tilted sideways, which is the M4 case. The target and the attacker rest about 1-3 mm elsewhere, and the hits change:
  - chamferPlus at +0° and +30°: the target is now at rest when hit (it was not before), and at +0° another attempt
    wins (`actualObl` −4.1° → +3.6°);
  - at the two oblique overhangs the best hit's direction cosine fell from 0.8774 and 0.8775 to 0.7744 (attacker shifted
    0.9 m inward, flipped), under the 0.82 the rig requires, so they are now unreachable.
- **Nine rim hangs now depart.** The tip faces' and chamfers' `staysOnPlatform` runs at d = 4.5 and 4.9 lost speed to the
  decomposition on the rim (task 23: every chamfer hang lost 1.5-10.4 m/s). They now keep it and leave, and leaving a
  rim adds M1's lift (anomaly +0.55 to +0.90 m/s, in `uvSupTilted`; at chamferPlus d = 4.5 also 0.55 m/s of
  `uvDetTilted`), so they count as lifts.
- **The stall is gone.** chamferMinus d = 3.5 at −30°, task 23's M4 example, falls from +2.0152 to +0.8924 m/s: its
  8.95 m/s residual is 0, and what is left is M1 (`uvSupTilted` 0.8924).
- **Down-pulls.** chamferPlus lost its three −5.0001 overhang down-pulls (at d = −0.15 the target now rests before the
  hit) and its −0.29 at d = 3.5. chamferMinus gained two −5.0001 at d = −0.15 (+0° and −30°): before, those runs had a
  6.4 m/s decomposition residual and no down-pull. Now `servo` is −5.0001 with `damp` −14.43: M2's clamp (the hit-tick
  write leaking into the vertical channel), which the M4 loss had hidden there [inferred from the two runs' residuals].
- **Smaller anomalies** moved by up to 0.5 m/s on runs that had a residual (for example tipPlusMid d = 3.5 at −30°, from
  a +0.39 lift to +0.11), and arcTopSeam d = 3.5 at +0° fell from a +0.54 lift to +0.11 (its residual was 0.03 m/s).

### 7.2 Task 20's re-run (the melee capsule hit test)

og-ground-aligned-attacks task 20 made the melee detector test the target's movement capsule against the attack band
instead of its centre (`BrawlerHitDetectionSystem-rationale.md` §11 in og-brawler). Its before and after outputs are in
the initiative workspace, under impl/scratch/ga20_impl/logs (pe_before.txt and pe_final.txt).

```text
before  [PlatformEdge.summary] runs=447 placements=263 unreachable=7 lifts=66 downPulls=21 staysOnPlatform=79 mirroredTicks=88953 mirrorMismatches=0 supportMismatches=0 pushOutMismatches=0
after   [PlatformEdge.summary] runs=395 placements=266 unreachable=4 lifts=66 downPulls=23 staysOnPlatform=79 mirroredTicks=78605 mirrorMismatches=0 supportMismatches=0 pushOutMismatches=0
```

- **258 of the 263 placements in both runs print the same `[PlatformEdge.run]` line, field for field.**
- **The five that moved are all oblique overhangs (d = −0.15, ±30°):** arcTop at both obliques, arcTopSeam at −30°,
  chamferPlus at +30° and chamferMinus at −30°. Before, the chosen hit had a
  direction cosine of 0.875 to 0.895 (`actualObl` within 3.5° of the rim normal). Now the search's first placement
  (inward shift 0, no flip) connects on the other swing side: the target rests 0.138 m (chamferPlus: 0.105 m) below
  the attacker along the face normal (`[PlatformEdge.attempt]` `alongNormalM`, after task 20), and its capsule overlaps
  the band where its centre did not [inferred: before task 20 that first attempt did not win the search]. Cosine
  0.991 to 0.999, `actualObl` ±22.3° to ±27.8°, close to the wanted ±30°. Each keeps its class.
- **Three overhangs are reached for the first time:** arcTopSeam +30° and chamferMinus +30° (both down-pulls, −5.0001)
  and chamferPlus −30° (no anomaly). Hence `placements` 263 → 266, `unreachable` 7 → 4 and `downPulls` 21 → 23.
- **`runs` fell from 447 to 395** because the attacker search stops at the first attempt whose cosine reaches
  `kGoodDirectionCos`, and more first attempts now connect. It counts attempts, not placements.
- Lifts, stays, the decomposition line and the three mismatch counters are unchanged.

## 8. The support-law scorecard additions (task 31)

og-ground-aligned-attacks task 31 made this case the third rig of the support-law scorecard; the other two are in
`SupportLawScorecardTest.cpp` (`SupportLawScorecardTest-rationale.md`). Nothing it measured before changed: the summary,
decomposition and site lines are byte-identical to the tree before task 31, and so are the 234 run lines of the
pre-task capture that could be compared (its other 32 were garbled by interleaved error output).

- **Three counters per placement** (`supportCountersOf`), over the ticks from the hit tick up to, not including, the
  first floor-contact tick (`onFloor`), or to the end of the run:
  - `flips`: ticks whose support differs from the tick before. The hit tick counts, and so does the detach of a
    departure; the landing on the floor does not;
  - `lipTicks`: ticks whose GJK normal is walkable while the exact face is not (`walkable` and not `faceWalkable`);
  - `rimHang` (0 or 1): the placement's outcome is `staysOnPlatform` and the target ends the run past the rim line. Only
    a real rim counts: the rim point must lie on the platform's boundary (`insideMargin` within 1 mm,
    `kRimOnBoundaryM`). That leaves out the two seam crossings, whose "rim" is the seam 5 m inside the slab; without the
    filter they added 30 "hangs".
- **Lines.** `[PlatformEdge.support]` per placement with those three, after its `[PlatformEdge.run]` line, and both only
  with OG_SUPPORT_LAW_RUNS=1. One totals line always prints after the decomposition line:
  `[SupportLaw.scorecard] law=… rig=platformEdge runs=… flips=… lifts=… downPulls=… rimHangs=… lipTicks=…`, where
  `runs` counts the reached placements (266), not the attempts of the summary line (395).
- **One law since og-ground-aligned-attacks task 37.** The case runs once,
  `PlatformEdge.KnockbacksAcrossTheImportedPlatformRimsUnderV2`, under the movement's one law. The law switch, the
  scope and the scenario hook below are deleted, and every line keeps its ` law=v2` label, now a literal in each
  format, so each line still compares byte for byte with the two-law record: all of them equal task 36's. The Legacy
  lines and the Legacy pins are history; the initiative workspace's impl/support_law_baseline.md keeps the numbers.
- **The law** (og-ground-aligned-attacks tasks 33 to 36). The case ran twice in one binary:
  `PlatformEdge.KnockbacksAcrossTheImportedPlatformRims` under `SupportLaw::Legacy` and
  `PlatformEdge.KnockbacksAcrossTheImportedPlatformRimsUnderV2` under `SupportLaw::V2`. Each sets `gSupportLaw` through a
  `SupportLawScope`, and `EdgeScenario::configureStaticData` writes it into the movement StaticData before the scene
  registers the characters (`GoldenTraceScene-rationale.md` §1), so V2 runs register the centre probe too.
  - Every line of a V2 run says so. A line whose format already has a law field prints law=v2; every other line gets
    ` law=v2` right after its bracket tag (`lawTag`), which is empty under Legacy. So the Legacy lines are
    byte-identical to task 32's, and dropping every line that contains law=v2 leaves exactly the Legacy output.
  - Under V2 only, each tick line ends with ` kind=Face`, ` kind=Lip` or ` kind=None` (`kindField`, the estimate's
    `GroundSupport::kind`), and one more totals line prints after the scorecard line:
    `[PlatformEdge.estimate] law=v2 noneTicks=… faceTicks=… lipTicks=…`, the estimate's kinds over the mirrored ticks
    of the chosen runs.
  - ⚠ The scorecard line's `lipTicks` is the geometric counter above (a walkable GJK normal over an unwalkable exact
    face), under both laws; it is not V2's `Lip` kind, which the estimate line counts.
  - The pins (flips 249, rimHangs 17, lipTicks 435) were checked under Legacy only. Since task 37 they are the one
    law's, 214, 17 and 308 (§5).
  - Since task 32 the mirror's ground and classification are the production `estimateSupport` and `classifySupport`,
    and since task 33 its frame is that classification's (§4). So the mirror follows either law, and
    `mirrorMismatches=0` and `supportMismatches=0` hold under both (measured in task 33). The rest of the mirror (the
    push-out, the channels, the servo) is its own copy of `integrate`'s expressions.
- **The V2 edge-tick reference** (since og-ground-aligned-attacks task 34, a lead ruling on task 33's review). Under V2
  the centre probe keeps the estimate on `Face(top)` for a tick or two past the capsule's last pure-face contact, which is
  the edge tick of §6, so one or two ticks of in-face deceleration counted there as a lift or a down-pull. Every departing
  V2 run is therefore measured a second time, from the estimate's last `Face` tick (`isEstimateFace`: `kind` is `Face`,
  the run is supported, and the estimate's face is the top), with the same anomaly and threshold (`classOf`).
  - The run line ends with ` estimateEdge=… estimateEdgeAtHit=… estimateAnomaly=… estimateServo=… estimateRedirect=…
    estimateClass=…` (`estimateField`, empty under Legacy and on a run that stays). `estimateServo` sums the servo's
    clamped term from that tick to the report tick, and `estimateRedirect` (since og-ground-aligned-attacks task 35)
    the redirect's vertical change (`redirectDy`, §4) over the same ticks.
  - After each `[PlatformEdge.site]` line a V2 run prints
    `[PlatformEdge.estimateSite] law=v2 site=… runs=… lifts=… downPulls=… largestLift=… largestDownPull=…`, and after
    `[PlatformEdge.estimate]` one totals line, `[PlatformEdge.estimateSummary] law=v2 lifts=… downPulls=…`.
  - Since task 34's rework one more V2 totals line follows it, `[PlatformEdge.lipHits] law=v2 placements=… largestServo=…`:
    the count and the largest run-line `servo` of the Lip-hit placements that §5's V2 check covers.
  - The §6 definition still prints in every line (`anomaly`, `class`, and the site and summary counts), so the
    numbers stay comparable with tasks 31 to 33. V2 is judged by the estimate's.
- **Seen failing** (task 31, built and run, then restored byte for byte), one poison per counter:
  - flips counting only Unsupported → Supported: `31 == 249`;
  - `rimHang` without the boundary filter: `47 == 17`;
  - lip ticks counting a walkable exact face instead: `10434 == 435`.
- The measured baseline, with the per-site table of these counters, is in the initiative workspace,
  impl/support_law_baseline.md.

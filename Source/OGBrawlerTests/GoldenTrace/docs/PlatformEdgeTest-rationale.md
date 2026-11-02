<!-- SPDX-License-Identifier: BUSL-1.1 -->
# PlatformEdgeTest — rationale

Source: `PlatformEdgeTest.cpp`, with its data in `QuarterCylinderPlatform.inl`. This file has no guards document: nothing in
either file needs a `⛔` tag. The harness is described in `GoldenTraceScene-rationale.md`.

`PlatformEdge.KnockbacksAcrossTheImportedPlatformRims` (`[GoldenTrace]`, no golden) is the measuring rig of the
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
- **A target resting past a raised rim** sits about 0.14 m below the face plane. That is outside the swing band, whose
  half-thickness is 0.35 m about a pivot 0.3 m above the attacker's root. So for d = −0.15 the attacker overhangs too.
- **Seven placements cannot be reached** and are printed as `[PlatformEdge.unreachable]`: both tip corners at d ≤ 0.3
  outward, and three oblique overhangs (d = −0.15): the top rim's seam at +30°, and since task 30 both chamfers at the
  oblique that turns toward their tip (chamferPlus −30°, chamferMinus +30°). All of them are within 1 m of a rim
  (`kReachableFromDistance`). Before task 30 there were five; §7.1 says why the two chamfer overhangs moved.

## 4. The per-tick record and the mirror

- **The probe.** Before each step, `runOnce` copies the target's movement `State` and calls `probeGround` with the
  movement declaration's bindings. That is the sweep `integrate` makes on that tick: the query sees statics only, and
  nothing moves the body between the copy and the probe.
- **The record.** After the step it reads:
  - the support bits, `DerivedState::surfaceNormal` (the basis n), the push-out, and the post-step velocity;
  - the inbound-hit slice and the attack machine's state.
- **The mirror.** `mirrorIntegrate` recomputes the step's velocity from those inputs with the movement's own helpers
  (`attackGroundNormal`, `writesKnockbackVelocity`, `detachesFromSupport`, `buildTangentFrame`, `decomposeVelocity`,
  `moveTowards`, `computeDesiredVelocityUV_ContinuousAccelBrake`) and the servo expression of `integrate`. It splits the
  vertical change into its parts:
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
  - the stepped support equals walkable-and-not-detached (`supportMismatches`);
  - the recomputed push-out equals the recorded one (`pushOutMismatches`).
  These three are what make the printed attribution trustworthy. On 2026-11-02 they held on 88,356 ticks of 444 runs,
  and after task 30 on 88,953 ticks of 447 runs.
- **Seen failing** (task 23, built and run, then restored byte for byte):
  - poison A removed the servo's clamp from the mirror: `mirrorMismatches=299` and `CHECK( mirrorMismatches == 0u )`
    failed;
  - poison B inverted the detached term of the support check: `supportMismatches=77904` and
    `CHECK( supportMismatches == 0u )` failed.
  Each arm moved only its own counter.

## 6. What it prints, and the definitions

- `[PlatformEdge.run]`, once per placement.
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

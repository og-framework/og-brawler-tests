<!-- SPDX-License-Identifier: BUSL-1.1 -->
# SupportEstimateTest — rationale

Source: `SupportEstimateTest.cpp`. This file has no guards document: nothing in it needs a `⛔` tag. The harness is
described in `GoldenTraceScene-rationale.md`, and the estimate it measures in og-brawler's
`BrawlerMovementSimulation-rationale.md` §33.

The six cases (`[GoldenTrace]`, no golden) call `brawlerMovementSimulation::estimateSupport` under `SupportLaw::V2` on
Jolt, at scripted positions, with no simulation step. They are the acceptance cases of og-ground-aligned-attacks
task 33: the V2 estimate's kind and normal on real Jolt geometry. Each prints one or more lines with its measurements.

## 1. The scene and the probe

- **`EstimateScenario`** is a `goldenTraceScene::Scenario` with one character, parked at (80, 40, 80) over nothing. Its
  `world` picks one of three static worlds before each `goldenTraceScene::Scene` is built:
  - `World::Platform`: the dev level's two imported 30° hulls (`QuarterCylinderPlatform.inl`, the same data as
    `PlatformEdgeTest.cpp`, without that rig's floor). It has the 30° top face, the top rim with its 60° lip (the end
    face), the vertical tip walls, and the seam where the two coplanar hulls overlap by 2.25 cm.
  - `World::FlatSlab`: one 10 m × 0.5 m × 10 m box with its top at y = 0; the rim is its +X wall at x = 5.
  - `World::Slope`: the slope golden's statics (`goldenTraceSlopeScenario::slopeStatics`); the concave pair is the
    landing (top y = 2) and the 30° face at the lower crease, z = 10.392304.
- **The law** is `EstimateScenario::law`, `SupportLaw::V2` unless a case says otherwise, written into the movement
  StaticData by `EstimateScenario::configureStaticData` before the character registers (`GoldenTraceScene-rationale.md`
  §1). So the character's bindings carry the ground probe at index 0 and, under V2, the centre probe at index 1.
- **`Fixture::at`** builds a movement `State` at a body position and calls `estimateSupport` through `CountingQuery`, a
  wrapper on the scene's `JoltSpatialQueryAdapter` that counts every sweep and the centre probe's sweeps (task 10's
  `CountingQuery` pattern), and keeps the centre's last hit. A sample also records whether the capsule's contact was an
  edge (`exactContactOf`'s `faceContact` false).
- **Placement over a plane** (`Fixture::bodyOverPlane`): the body is placed so that the probe capsule (radius
  `capsuleRadius − kProbeShrink`, bottom at the body's bottom), swept straight down, first touches the plane through a
  point with normal n after the given clearance. That is the analytic hover for a face.

## 2. The cases

1. **`SupportEstimate.FaceInteriorIsTheExactFaceWithNoCentreSweep`.** 732 samples: face-interior points of the platform
   (at least 0.6 m inside every rim and from the seam), of the flat slab and of the slope face, each at clearances
   0.05, 0.1 and 0.3 m. Each is `Face`, eligible, with `faceNormal` and `frameNormal` bit-equal to the hit's
   `surfaceNormal`, and the centre probe is never swept (`centreSweeps=0`). The line also prints the largest angle
   between the GJK normal and the exact face (`worstGjkRad`) against the edge threshold (2°, 0.034906 rad).
2. **and 3. `SupportEstimate.RimIsFaceThenLipThenNoneWhereWalkableEnds`**, on the platform's top rim (chart point
   s = 10.02, w = 1.5, outward along the face) and the flat slab's rim.
   - The body keeps the height that hovers it over the face at the rim point, and moves outward from x = −0.3 m to
     +0.6 m in 5 mm steps (x along the outward direction, negative before the rim).
   - At x = −0.1 m: `Face` with the top's exact-face bits (taken at x = −0.3 m). On the 30° rim the capsule's contact
     there is already the edge, so the face comes from the centre probe; on the flat rim it is a face contact.
   - At x = +0.05 and +0.2 m: `Lip`, eligible, normals `kWorldUp`.
   - Outward, the first `None` is the first position whose probe is not walkable, with the probe still blocked and its
     GJK normal past the 45° cap. Between the first `Lip` and that position every sample is `Lip`, and after it every
     sample is `None`; no `Face` follows a `Lip`.
4. **`SupportEstimate.WallAndLipFacesAreNeverTheFace`.** The two rim marches and a march over the platform's +w tip wall
   (s = 2.5). Wherever the capsule's or the centre's reported exact face is a wall (normal within 2° of horizontal) or
   the 60° lip (58° to 62° from up), the sample is never `Face` with that normal; every `Face` is walkable. The case
   needs at least one of each.
5. **`SupportEstimate.SeamCrossingsStayOnTheFace`.** 36 crossings of the seam: 18 lines at s = 0.5, 1.0, …, 9.0, each
   crossed in both directions (the reverse crossing offset by half a step), from w = −0.3 to +0.3 m in 1 cm steps at the hover.
   Every sample is `Face`, no `Lip`, with every component of the normal within 1e-6 of the platform's face normal.
6. **`SupportEstimate.ConcaveCreaseTakesTheRisingFaceFromItsFirstContact`.** Three lines (x = −3, 0, 3) from 1.0 m on the
   landing side of the lower crease to 0.6 m up the face, in 5 mm steps. Each body hovers over the higher of the two
   planes. The first sample whose capsule contact lies on the rising face (the impact point on its plane, at least 1 mm
   past the crease line) and every later one is `Face` with the rising face's normal; every sample is `Face`.

**`SupportEstimate.LegacyRegistersOneQueryVolumeAndV2Two`** builds the platform scene under each law: Legacy registers
one query volume and V2 two, and at the top rim a Legacy estimate makes one sweep, a V2 estimate one or two (two only
on an edge contact).

## 3. Measured (task 33)

```text
[SupportEstimate.interior] samples=732 centreSweeps=0 worstGjkRad=0.000000 edgeContactRad=0.034906
[SupportEstimate.rimSummary] rim=platformTop lastFaceX=-0.015 firstLipX=-0.010 firstNoneX=0.335 firstUnwalkableX=0.335 noneBlocked=1 gjkTiltAtNoneDeg=45.041 gjkTiltBeforeNoneDeg=44.191 cosMaxSlopeDeg=45.000
[SupportEstimate.rimSummary] rim=flatSlab lastFaceX=0.010 firstLipX=0.015 firstNoneX=0.290 firstUnwalkableX=0.290 noneBlocked=1 gjkTiltAtNoneDeg=45.017 gjkTiltBeforeNoneDeg=44.037 cosMaxSlopeDeg=45.000
[SupportEstimate.wallAndLip] checked=187 wallSeen=21 lipSeen=166
[SupportEstimate.seam] crossings=36 samples=2196 edges=0 centreSweeps=0 lips=0 worstComponent=5.960e-08
[SupportEstimate.crease] lineX=0.0 firstRisingDzM=0.105 flatFacesBefore=179 centreSweepsBefore=0
```

- **The 30° rim.** At x = −0.1 m the capsule touches the edge (GJK tilt 12.19°, exact face the 60° lip), and the centre
  probe finds the top face (30.000°): `Face`. From x = −0.010 m the centre probe itself touches the edge (its exact
  face the lip), so the estimate is `Lip`. `None` starts at +0.335 m, where the GJK normal passes 45° (45.041°; 44.191°
  one step before), the same position at which `probe.walkable` turns false.
- **The flat rim.** `Face` (a face contact) through +0.010 m, `Lip` from +0.015 m, `None` from +0.290 m (45.017°).
- **The seam** never makes an edge contact on these lines: Jolt reports the top face on both hulls, and the worst
  component difference from the face normal is 6e-8.
- **The crease.** The capsule's first contact with the rising face comes 0.105 m before the crease line, on all three
  lines, while its bottom is still over the landing; from there every sample is the rising face, by a face contact.
  The 179 samples before it are all the flat face.

## 4. Why these are positions and not runs

The estimate is a function of the body's position and the static world, so a scripted position measures it exactly,
with no servo, no knockback and no mirror. The stepped behaviour at the platform's rims under V2 is measured by the
platform-edge rig (`PlatformEdgeTest-rationale.md` §8).

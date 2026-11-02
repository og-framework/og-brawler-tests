<!-- SPDX-License-Identifier: BUSL-1.1 -->
# BrawlerVelocityDecompositionTest — rationale

Source: `BrawlerVelocityDecompositionTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins og-ground-aligned-attacks task 30: `brawlerMovementSimulation::decomposeVelocity` splits a velocity
into `(a, b, c)` with `V == a*u + b*v + c*up` for every tangent frame `buildTangentFrame` makes, not only for
frames whose second tangent `v` is horizontal.

Before task 30 the function was a 2×2 solve that assumed `v` is perpendicular to `up`. That holds only for
normals tilted within the up–forward plane (every ±Z face and flat ground). A normal tilted sideways
(any X component) gives `v` a vertical part, and the 2×2 solve then dropped or injected horizontal speed:
on a 30° face rising along X, a 20 m/s fall came back as (8.66, −20, 0). Since task 30 the function keeps
that 2×2 solve unchanged when `|dot(v, up)| <= kDualBasisFastPathTolerance` and otherwise runs an exact
dual-basis (3×3) solve. The derivation is `BrawlerMovementSimulation-rationale.md` §19 in og-brawler.

All five cases are tagged `[BrawlerMovement]`, which puts them in the `[@og]` whitelist.

## 2. The frames

Every frame comes from `buildTangentFrame`, so the test uses the shipped frames, not hand-built ones.

- **General frames** (`generalFrames`), the ones that take the 3×3 solve:
  - `+X 30` and `−X 30`: the slopes the user plans next, rising along UE Y. In the simulation frame the
    normal is (±0.5, 0.866, 0).
  - `20 about φ` and `40 about φ` for φ in {30°, 45°, 60°, 120°}: the normal tilted from up by 20° or 40°
    about the horizontal axis at azimuth φ. The azimuth is measured from forward toward left, the chart's
    own order; `tiltedAbout` turns up toward `cross(axis, up)`.
- **Fast-path frames** (`fastPathFrames`): flat ground and 30° faces leaning toward ±Z (forward and back).
  Their `v` is exactly horizontal, so they take the unchanged 2×2 solve.

`TheFramesUnderTestTakeTheDualSolve` checks that the general frames are unit length, walkable under the
shipped `cosMaxSlope` (45°), take the dual solve (`|dot(v, up)|` above the tolerance) and have a positive
determinant, and that the fast-path frames take the fast path. Without it, a change to `buildTangentFrame`
could move every general frame onto the fast path, and the other cases would test nothing new.

## 3. The cases

- `EveryFrameRecomposesTheVelocity`: on every frame of both kinds, a pure 20 m/s fall, a 6 m/s walk along `u`,
  one along `v`, and 50 pseudo-random vectors (components in ±25 m/s from a fixed-seed linear congruential
  generator) recompose to the input within 1e-5·|V| on every component.
- `APureFallLeavesTheTangentChannelsEmpty`: on every general frame a pure fall gives `|a|, |b| <= 1e-6·|V|` and
  `c` equal to the fall. Its `SECTION` runs the same fall through `legacyDecompose` on `+X 30` and asserts the
  pre-task result: the tangent channels are not empty, and the velocity recomposes to (8.66, −20, 0). That
  section is the case's seen-failing arm, kept in the file.
- `TheFastPathIsTheLegacySolveBitForBit`: on the three fast-path frames, 10,000 pseudo-random vectors each
  (components in ±30 m/s), `decomposeVelocity` and `legacyDecompose` agree bit for bit (`std::bit_cast`) on
  `a`, `b` and `c`: 30,000 comparisons.
- `PlannedSlopesAlongXRecomposeAFallALandingAndAKnockback`: on `+X 30` and `−X 30`, a pure fall, a landing
  (5 m/s horizontally plus 12 m/s down) and a knockback at the shipped knockback speed (along the face up
  the slope, and diagonally across it) recompose within 1e-5·|V|. Each also checks that `c` is the gap rate
  `dot(V, n) / dot(up, n)`, the speed along up at which the capsule closes on the face: 0 for the two
  knockbacks, which move along the face. Each prints a `[VelocityDecomposition.planned]` line with the
  legacy channels and their recomposition error next to today's.

## 4. `legacyDecompose`

A frozen copy of the five lines `decomposeVelocity` had before task 30: the same operations in the same
order, without the `OG_CHECK`. Like the header, it is compiled with optimisation off (`OGSIM_OPTIMIZE_OFF`),
so its float operations are not re-associated differently from the header's. The bit-identity case uses it
as the reference for the fast path, and the fall case's section uses it as the defect's witness.

## 5. Seen failing (task 30)

Each arm was a probe edit of `kDualBasisFastPathTolerance` in `BrawlerMovementSimulation.h`, built and run, then the
header was restored byte for byte. The full output is in the task's implementation notes.

- **Arm A, the fast path on every frame** (tolerance 2, which is the pre-task law): 4 of the 5 cases failed, 556 of 785
  assertions.
  - The recomposition case failed 510 times. The first was the `+X 30` fall, `8.66025f <= 0.0002f`.
  - The fall case failed 28 times, for example `10.0f <= 0.00002f` for `b`.
  - The planned-slope case failed 8 times: the fall and the landing on both faces.
  - The frames check failed 10 times, because its own threshold is the poisoned constant.
  - The bit-identity case passed, as it must, since that arm's fast path is the legacy solve.
- **Arm B, the dual solve on every frame** (tolerance −1): the bit-identity case failed with 15,645 of 30,000 vectors
  differing (the first: `+Z 30`, `a` 27.2208405 against 27.2208385). The frames check failed on the 3 fast-path frames.

## 6. What the planned-slope lines show

The `[VelocityDecomposition.planned]` lines print the legacy solve next to today's. On `+X 30` and `−X 30` the legacy
solve recomposed both knockbacks exactly, because a velocity along the face lies in the span of `u` and `v`, which the 2×2
solve handles. It lost what lies along `n`: 8.66 m/s of the fall on both faces, and 6.37 m/s (`+X 30`) and 4.02 m/s
(`−X 30`) of the landing. So on those slopes the error is in every tick that has a velocity component into or out of the
face: falls, landings, and the servo's corrections. A knockback becomes one once gravity acts on it.

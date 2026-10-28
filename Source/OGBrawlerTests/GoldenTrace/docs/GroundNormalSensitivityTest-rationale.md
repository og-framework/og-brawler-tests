<!-- SPDX-License-Identifier: BUSL-1.1 -->
# GroundNormalSensitivityTest — rationale

Source: `GroundNormalSensitivityTest.cpp`. This file has no guards document: nothing in it needs a `⛔` tag. The slope
statics are described in `GoldenTraceSlopeScenario-rationale.md`, the harness in `GoldenTraceScene-rationale.md`.

`BrawlerJoltSlopeGroundNormalSensitivity.PrintsTheFlipRates` (`[GoldenTrace]`, no golden) measures one mechanism: a
position error too small to cause a correction by itself changes the attack ground normal, and the changed normal
changes the attack's initial conditions enough to fail the correction comparator. It is the deterministic half of the
ground-aligned attacks initiative's normal-driven correction measurement (its design §9.1, instrument 2). It runs no
tick and no swing.

## 1. The world and the sample

- **The world is the slope golden's.** `ProbeScenario` is a one-character `goldenTraceScene::Scenario` on
  `slopeStatics` (the 30° box, the landing and the plateau). `goldenTraceScene::Scene` builds the Jolt world and its
  query adapter and registers the character. The test never steps the scene: the character's movement `State` and the
  bindings of its movement declaration are read once.
- **The sample is production.** `Sampler::at` copies that `State`, sets `bodyState.position` to the probe position and
  calls `sampleAttackGround` with the ground-aligned path selected (`<true>`), so the movement probe sweep, the face
  choice and the walkability test are the shipped ones. The probe volume is world-only, so the parked character does
  not matter.
- **Two normal sources.** `SampleQuery` wraps the scene's adapter and forwards `sweep`. With `Source::Face` it changes
  nothing (the exact face normal, `SweepHit::surfaceNormal`). With `Source::Gjk` it zeroes `SweepHit::surfaceNormal`, so
  the sample falls back to the GJK contact normal, as an adapter without face normals would. No production code changes.
- **The perturbed-sample tilt.** `SampleQuery::tiltRadians` rotates both normals of the hit about `simFrame::kRight`.
  `measure` applies its `perturbedSampleTiltRad` argument to the sample at P + δ only, never to the sample at P. The
  measured runs pass zero; the control (§4) passes `kPoisonTiltRad`.

## 2. Positions, perturbations and aims

- **Interior:** a 0.25 m grid over the 30° face, at least 1 m from every edge: 41 positions across (±5 m) by 89 up the
  fall line (±11 m), 3649 in all.
- **Crease:** along the fall line at x = 0, every 1 cm from 1 m on the slope side to 1 m on the flat side, for both
  creases: the concave one (slope to landing) and the convex one (slope to plateau). 201 positions each, indexed in
  signed centimetres, negative on the slope side, 0 at the crease line.
- **Height:** every probe position is 1.2 m vertically above its surface point. That is inside the probe's reach on
  both faces and clear of start penetration; the test counts both (§4).
- **Perturbations:** δ of 1e-6, 1e-5, 5e-5 and 1e-4 m along +X, −X, +Y, −Y, +Z and −Z (`offsetAlong` writes one
  component, so no negative zero enters). The largest δ equals `kDefaultSimilarityEpsilon`, the position threshold (a
  `static_assert`). Each (P, P + δ) is one pair: six pairs per position and δ. `noOpPairs` counts pairs whose P + δ
  rounds back to P in float; it is 0.
- **Aims:** 36 headings every 10°, each with a fixed downward component (`kAimHeight`, −0.35 before normalising), as
  a camera ray looks down.

## 3. The initial conditions and the comparator

- **The initial conditions are production.** For a sample, `computeICs` runs task 5's machine rig twice per aim:
  `groundAlignedMachineRig::swingStart` (a left swing from Idle) for the radial initial conditions, and
  `groundAlignedMachineRig::hadoukenLaunch` for the projectile's. Both are direct `integrate3` calls. The test requires
  that every one starts (`notStarted` is 0).
- **The cache is exact.** The machine computes the radial initial conditions and the Hadouken `spawnDir` from the aim
  and the sample's normal only; the capsule position enters only `spawnPos`. So `ICCache` keys the 36 results by the
  normal's bits and computes each distinct normal once. Two pairs with the same normal bits give the same initial
  conditions, so a pair whose normals have the same bits cannot flip and is not evaluated further. The claim is checked:
  97 positions (a 2 m sub-grid of the interior and every 10th crease centimetre) recompute the results with the real capsule position and
  compare them bit for bit with the cached ones (`directChecks`, `directMismatches`, which must be 0).
- **`icFlips` uses the correction comparator's own semantics.** A pair flips when, for any of the 36 aims, the radial
  initial conditions fail `fieldwiseIsSimilarTo` or the `spawnDir` fails `leafIsSimilarToAsFieldwiseDoes`. These are
  the comparator's fold and its exported leaf (`SimulationSerialization.h`), so the test sees what a correction sees: the
  float angle within `kDefaultSimilarityEpsilon`, but a `glm::vec3` field (the rotation axis, `spawnDir`) compared
  exactly, because the fold's dependent call never reaches the epsilon overloads in `SimulationComparisonGlm.h`.
  `spawnPos` is left out: it carries the position error itself.
- **The epsilon reading, for comparison.** `icFlipsDirectEpsilon` counts the same pairs with direct calls to
  `isSimilarToField`, which do reach the `glm::vec3` epsilon overload. The gap between the two counts is what the
  exact vector compare costs; on the GJK interior at δ = 1e-6 it is 268 against 2.

## 4. The lines and the checks

- `[GroundSensitivity]`, one per source (`face`, `gjk`), region (`interior`, `crease` = both creases) and δ:
  `positions` probe positions, `normalDiffers` pairs whose normals differ in any bit, `icFlips` flipping pairs, and
  `flipRate` = `icFlips` / pairs (six pairs per position).
- `[GroundSensitivity.detail]`, per row: `pairs`, `noOpPairs`, `aimPairs` (pairs × 36), `aimFlips` (flipping aims),
  `icFlipsDirectEpsilon`, and the largest angle between two differing normals.
- `[GroundSensitivity.crease]`: the crease rows split into the concave and the convex crease.
- `[GroundSensitivity.interiorSamples]` and `[GroundSensitivity.interiorOffFace]`, per source: how the interior
  samples sit against the face normal (positions further than 1.4e-3 rad from it, start penetration, the angle
  distribution, where the outliers are), the number of distinct normals, and the cache checks.
- `[GroundSensitivity.creaseSurface]`, per source and crease: which surface each centimetre samples (`face`, `flat`,
  `edge` for any other walkable normal, `unsupported` when the chosen face is steeper than the movement's maximum slope
  and the sample falls back to world up), and the **edge band**: the centimetres that sample something other than the
  surface beneath them.
- `[GroundSensitivity.creaseBand]`, per source and crease: the centimetres where any δ = 1e-4 pair differs in its
  normal, their span and its width in metres.
- `[GroundSensitivity.creaseNormalDiffers]`: how many crease pairs differ, and the first twelve.
- `[GroundSensitivity.control]`: the interior run again with the face normal and the perturbed sample tilted by
  `kPoisonTiltRad` (1e-3 rad) at the smallest δ.

The checks, counted in loops and asserted once each: with the face normal, `icFlips` is 0 in the interior for every δ;
every interior face sample is on the face; no interior sample starts penetrating; every machine call starts; the cache
checks ran and found no mismatch; and the control flips every one of its pairs, so the counter can see a flip. With the
measured run's tilt set to `kPoisonTiltRad` the four interior assertions failed (21894 flipping pairs of 21894 at each δ).

## 5. What the sweep measures and what it does not

It measures, on one binary, how often a sub-threshold position difference alone changes the sampled normal's bits and
then the attack's correction fields, per normal source and region. It bounds what freezing or quantising the normal
could save: no more than `normalDiffers`.

It does not measure:
- **Live input timing.** Two peers can start the same swing on different ticks or with different aims; those
  divergences come from the input, not the normal, and are the live attribution's business.
- **Cross-platform arithmetic.** Both samples of a pair come from the same Win64 MSVC binary, so the arithmetic is
  bit-identical by construction. Another compiler, flag set or CPU could compute different normal bits for the same
  position; this sweep cannot see that.
- **Rest positions.** The probe positions are a fixed 1.2 m above the surface, not settled heights; near the creases the
  real rest height differs (`BrawlerJoltSlopeGoldenTraceTest-rationale.md` §6). The crease lines are at x = 0 only.
- **Other consumers of the normal.** The guard pose and the hit and block tests read the normal too; a differing normal
  can change their outcomes, which `normalDiffers` bounds but the test does not evaluate.

## 6. Measured at task 5b

- **Face normal, interior:** 0 differing normals and 0 flips at every δ. The interior samples one normal.
- **Face normal, creases:** the concave crease never differs. At the convex crease only the position 24 cm down the
  slope differs: its ±X pairs at 1e-6 and its +X pair at 1e-4 switch between the slope face and world up (30°). The
  edge band is −23 to 0 cm: there the face-normal sample is the plateau's up or falls back to world up from a face that
  is too steep, so it has the same bits either way. At the concave crease the flat side's first 10 cm sample the slope
  face (edge band 0 to 10 cm).
- **GJK normal, interior:** about 1.1 % to 1.3 % of the pairs flip at every δ (268 at 1e-6, 291 at 1e-4, of 21894).
  41 interior positions are 0.0145 rad off the face; they lie on one line across the face, along = −2 × across − 0.25 m
  (from (−5, 9.75) to (5, −10.25)), which suggests the GJK contact on the face diagonal. The other 3608 are within
  1e-6 rad.
- **GJK normal, creases:** the concave crease never differs; the convex crease flips 7 % to 11 % of its pairs, and its
  δ = 1e-4 band spans −24 to 0 cm (0.24 m), the GJK edge band −23 to −1 cm.

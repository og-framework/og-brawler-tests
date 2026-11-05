<!-- SPDX-License-Identifier: BUSL-1.1 -->
# AimPlaneTest — rationale

Source: `AimPlaneTest.cpp`. This file has no guards document: nothing in it carries a `⛔` tag.

## 1. What this file is for

Pins for og-ground-aligned-attacks task 12 (2026-11-03): the mouse aim intersects the character's ground plane. The
intersection moved out of the Unreal input component into og-brawler's engine-free `dInput::aimPlane`
(`AimPlane.h`), and its plane normal became the guard body's up. The reasoning is in og-brawler's
`AimPlane-rationale.md`. The Unreal caller (the ground read and the deprojection) is in the game module, which this
target cannot compile.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[AimPlane]` selects them on their own.

## 2. The frames and the reference

Rays, capsules and plane normals are in Unreal axes and centimetres, as the caller passes them. `ueToSimAxes` is the
permutation `ueFrame::directionToSim` applies for og-simulation's Y-up frame (Unreal X to −Z, Y to X, Z to Y); a
`static_assert` on `simFrame::kForward`, `simFrame::kRight` and `simFrame::kUp` holds it to that frame, because this
target cannot include the conversion header. The slopes are `upTilted(30)`: tilted toward −X (the level's face) or
toward −Y (a slope rising along Unreal Y), rounded to float as a face normal is.

`todaysMouseAim` is the input component's statements as they were before the task: the plane normal
`FVector(0.f, 0.f, 1.f)`, the near-parallel reject, the float distance, the `IsNearlyZero` reject, then normalize after
the axis conversion. Where Unreal's math is on the include path (the UBT target depends on Core) they are those
statements verbatim on FVector (with UE_KINDA_SMALL_NUMBER, the value the deprecated KINDA_SMALL_NUMBER names); the
standalone CMake build gets a transcription with the same operations in the same order. Under UBT a `static_assert`
also holds both helper tolerances equal to UE_KINDA_SMALL_NUMBER.

The grid (`forEachCameraAndTarget`): four capsule positions (including non-round and negative coordinates), cameras at
600 and 2500 cm, pitches 10°, 30°, 45°, 60° and 80° above the horizon, 24 yaws (7.5° + 15° steps, so no camera sits
exactly on a slope's axis), and 48 targets per camera on a 7 × 7 grid of in-plane offsets from −1500 to 1500 cm
(the centre left out).

## 3. The cases

* `AimPlane.WorldUpIsTodaysMouseAimBitForBit` (AC: world up is today bit for bit): the grid with world up and three
  target heights (−50, 0 and 37.5 cm off the plane), plus seven hand-placed rays (a camera at the plane's height, so
  the distance is zero; a horizontal ray; just inside and just outside the parallel tolerance; a ray straight down
  onto the capsule centre and one 0.25 cm beside it). Both results are compared by their bits, including whether they
  are empty: `compared 138247 mismatched 0 rejected 3`.
* `AimPlane.ThirtyDegreeAimLiesInThePlaneAndPointsAtTheHit` (AC: the 30° aim is in the plane and points at the hit):
  both slopes over the grid, 46080 rays each, none rejected; worst |dot(aim, n)| `6.95005291e-07` (the AC's bound is
  1e-6), worst angle to the in-plane target `8.22e-07` rad (bound 1e-6). The seen-failing arm below fails here.
* `AimPlane.TheSwingHeadingPassesUnderTheCursorOnTheSlope` (the brief's round trip): the aim taken into the ground frame
  (`surfaceFrame::tiltFromSurface`) has a height of at most `7.08e-07` (bound 1e-6), so `simFrame::flatten` is the
  identity on it to rounding, and the swing heading taken back (`surfaceFrame::tiltToSurface`) is at most `7.11e-07`
  rad from the cursor's hit (bound 1e-5). For contrast, the pre-task horizontal plane on the same rays: 6048 rays hit it
  behind the camera, and at 2500 cm / 60° the heading is up to `24.8` degrees from the cursor; the case requires that
  contrast to stay above 1°, so a reference that silently became the slope plane fails it.
* `AimPlane.ANearParallelRayIsRejected` (AC: the first reject): for world up and both slopes, rays along the plane
  and tilted 9e-5 toward or away from it are empty; 1.5e-4 is not.
* `AimPlane.AZeroAimIsRejected` (AC: the second reject): for every capsule and every normal, a ray straight down the
  normal onto the capsule centre and one 0.5e-4 cm beside it are empty (through `unitAim` and `hitOffset`); 0.25 cm
  beside it is not. The camera is 300 cm away: at 2500 cm the float distance alone can move the hit by about 1e-4 cm.
* `AimPlane.TheTiltedPlaneUpIsEmptyOnFlatGround`: world up, the 1-ULP-short `(0, 0.99999994, 0)`, a 5e-7 tilt, zero,
  straight down, a downward tilt and a horizontal up are empty; a 2e-6 tilt, both 30° ups and a 12° up come back with
  their bits.

## 4. Seen failing

The task's poison: `hitOffset` with a hard-coded `(0, 0, 1)` in place of its normal, built and run on 2026-11-03.
`test cases: 6 | 3 passed | 3 failed`, among them
`AimPlane.ThirtyDegreeAimLiesInThePlaneAndPointsAtTheHit`: `CHECK( worstDot <= 1e-6 )` with expansion
`0.499996901 <= 0.000001` and `CHECK( worstAngle <= 1e-6 )` with `3.0592049662 <= 0.000001`, for both slopes. The
world-up case passed under the poison, as it must. The helper was restored byte for byte.

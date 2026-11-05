<!-- SPDX-License-Identifier: BUSL-1.1 -->
# DAttackCameraSlopeFrameTest — rationale

Source: `DAttackCameraSlopeFrameTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for og-ground-aligned-attacks task 21 (2026-11-03): the solo camera's look mode orbits in the ground's frame. The
camera state gained a frame up (`OrbitCameraState::frameUp`), the host renders `dAttackCameraBehaviour::worldView`, the
target up comes from `dAttackCameraBehaviour::cameraTargetUp` (the 30° + 0.1° cut-off and the airborne rule), the
look-mode pull also turns the frame up, and look mode off returns the camera to world up and the target pitch. Task 21b
(2026-11-03, the user's G7 ruling "Full tilt, clamped") replaced task 21's orbit: on a followed slope the world yaw is
the player's, and the world pitch moves by the ground's rise along the view, inside the slope pitch limits. The
reasoning is in og-brawler's `DAttackCamera-rationale.md` §18 and §19. The Unreal host's part (the ground read, the
boom write and the console variables) is in the game module, which this target cannot compile.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[CameraSlopeFrame]` selects them on their own.

## 2. The frames the cases use

Camera space is the host's: Z up, angles in degrees, the pitch below the horizon. `upTiltedTowardMinusX(d)` is a
ground up tilted `d` degrees toward −X, so the slope rises toward +X and local yaw 0 looks up-slope; this is the
level's 30° face, which rises away from the default camera. `upTiltedTowardMinusY(d)` is the same toward −Y, a slope
rising along Unreal Y (the user plans those). Both are built in double and rounded once. "Unclamped" settings put the
slope pitch limits at the hard limits, 1° and 89°.

## 3. The cases

* `DAttackCamera.SlopeFrameWorldUpIsTodaysView` (AC 1): with `frameUp` world up, and with the 1-ULP-short up a flat
  face normal can carry, `worldView` returns the state's yaw and pitch with the same bits over a yaw grid (−180° to
  180° by 7.5°) and a pitch grid (1° to 89° by 0.5°): 17346 views, 0 mismatched. A frame tilted 2e-6 rad, just past
  the flat tolerance, stays within 1e-3° of that (1.1e-4° measured since task 21b, pitches 10°–80°; 6.6e-4° with task
  21's orbit), so the flat branch makes no visible step.
* `DAttackCamera.SlopeFramePitchFollowsTheRiseAlongTheView` (task 21b): at p = 60° the world pitch is up-slope 35°,
  contour 60° and down-slope 75° (± 1e-4°) at the default limits, and 30°, 60° and 89° unclamped, on the 30° face
  rising along X (both contour headings) and along Y; on the 12° face 48°, 60° and 72° either way. Two faces are the
  level's real 30° face: the G3 capture's measured swing ups `up=(0,0.866025448,0.49999997)` and
  `up=(-1.49011612e-08,0.866025388,0.5)` (og-simulation axes, Y up, forward −Z), written in camera space as Unreal's
  (−z, x, y): the face rises along Unreal +X, so yaw 0 looks up-slope and reads 35°, yaw 180° down-slope 75°. Every world yaw is the
  input yaw, bit for bit, at those headings and over a sweep (five frames, yaw −180° to 180° by 2.5°, pitch 1° to 89°
  by 0.5°, both limit settings): 256650 views, 0 mismatched. It replaced task 21's orbit case, SlopeFrameOrbitsAboutTheNormal (deleted).
* `DAttackCamera.SlopeFrameYawIsThePlayersAndThePitchNeverJumps` (task 21b): yaw −180° to 180° in 0.5° steps at
  p = 60° on the 30° face. Every world yaw step equals the input step, bit for bit (largest 0.5°), and the largest pitch
  step is 0.288677°, within the rise's bound `0.5° × tan 30°` = 0.288675° plus 1e-4° of rounding slack; the sweep
  covers 35° to 75° at the defaults and 30° to 89° unclamped. A last check (largest step above half the bound) keeps
  the sweep from passing on a pitch that never moves.
* `DAttackCamera.SlopeFramePitchDoesNotPopWhenTheFrameLeavesWorldUp` (task 21b): the frame up eased from world up to
  the 30° face in 100 steps, at p = 20° and 80° and eight headings. The first view is `p` with the same bits, no step
  is above 1° (0.300007° measured), and the pitch never reverses (0 measured; 1e-5° allowed). It ends at 20° (20°
  up-slope), 50° (20° down-slope), 50° (80° up-slope) and 80° (80° down-slope).
* `DAttackCamera.SlopeFramePitchLimitsAreOrdered` (task 21b): `slopePitchLimits` gives 35° to 75° at the defaults, 40°
  to 40° for a minimum of 50° over a maximum of 40° (the up-slope view reads 40°, the down-slope one stays at 60°),
  the hard limits for −5° and 120°, and 1° to 1° for 0.5° over 0.2°. Limits of 45° and 65° give 45° up-slope and 65°
  down-slope: the settings are what `worldView` reads.
* `DAttackCamera.SlopeFrameTargetFollowsTheGroundUpToThirtyPointOneDegrees` (AC 3): the two `constexpr`s are 30 and
  0.1. Normals at 30°, at the G3 capture's measured extremes 30° − 1.32e-5° and 30° + 6.4e-6°, at 30.09°, 12° and 1e-3°
  follow (the normalized ground up comes back, not world up). Normals at 30.11°, 30.2°, 45°, 60° and 89° give world
  up, bit for bit. World up, the 1-ULP-short up, a 5e-7 horizontal part (inside the flat tolerance), the zero vector,
  a NaN and a downward vector all give world up, bit for bit.
* `DAttackCamera.SlopeFrameTargetIsWorldUpWhenUnsupported` (AC 4): unsupported gives world up at 0°, 12°, 30° and 45°.
* `DAttackCamera.SlopeFrameLookModeMovesTheFrameUpOnlyWithTheYawDrivenPull` (AC 5), at 30, 60, 144 and 240 fps:
  600 still frames in look mode leave the frame up, the pitch and the yaw with the same bits; pitch-only mouse input
  moves the pitch and not the frame up. A 0.5°-per-frame mouse turn (below the cap at every rate) for 60° of yaw: the
  frame up's remaining share of its 30° gap is `exp(−3)` within 1e-3 (0.0497872 against 0.0497871), it tracks the
  pitch's share within 1e-4 on every frame (3.4e-7 measured) and stays on the great circle to the target within 1e-3°
  (3.1e-6° measured). A dominance weight of 0 and a running hold-off leave the frame up alone; after the hold-off the
  turn pulls it.
* `DAttackCamera.SlopeFrameLookModeOffReturnsToWorldUpAtTheIsoPitch` (AC 6): from four starts (pitch 25° on the 30°
  frame, 80° on a 12° frame, 55° on the 30° frame, already at world-up 60°), with full mouse and stick ignored, each
  frame moves the frame up and the pitch by at most `returnDegPerSec × dt` (within 1e-3°), the yaw keeps its bits on
  every frame, and the camera is exactly world up and 60° after `ceil(angle / (90 × dt))` frames, at most one frame
  later (measured: exactly that count everywhere, 7 to 94 frames). `worldView` then returns the state verbatim, and 60
  more frames keep it there. A rate of 0, or a negative one, holds the camera still: the camera before task 21. The
  target up is a 12° face: the frame up returns to world up whatever the target, and below 15° the yaw does not turn.
  Until task 21c (2026-11-04) the target was the 30° face; there the look-mode-off camera now turns to face up the
  slope, pinned by `DAttackCameraSlopeYawTest.cpp`.

## 4. Seen failing

Task 21b's poison, task 21's tilted yaw restored in `worldView` (the yaw of the view tilted into the frame, with its
near-vertical fallback), built and run under the UBT lock on 2026-11-03: `test cases: 9 | 7 passed | 2 failed`,
`assertions: 929 | 910 passed | 19 failed`. The no-swing case fails on both settings:

```
CHECK( yawStepMismatches == 0 )
with expansion:
  720 (0x2d0) == 0
...
CHECK( worstYawJumpDeg == Catch::Approx(kYawStepDeg).margin(1e-4) )
with expansion:
  90.2172088623 == Approx( 0.5 )
```

and `SlopeFramePitchFollowsTheRiseAlongTheView` fails its yaw-bit checks. A second poison, the slope clamp reduced to
`[lo, hi]` (guard G-03 of `DAttackCamera-guards.md`), fails 3 cases: the no-pop case with a 15° first step
(`pitch 20 yaw 0: ends at 35, largest step 15 deg`), case 1's just-tilted sweep with a 25° gap, and the ordering
case (the inverted limits' down-slope view reads 40°, not 60°). Restored and rebuilt: all 9 pass.

Task 21's own poison (`worldView` without the tilt) failed the case this task replaced, SlopeFrameOrbitsAboutTheNormal
(`60.0f == Approx( 30.0 )` up-slope), on 2026-11-03.

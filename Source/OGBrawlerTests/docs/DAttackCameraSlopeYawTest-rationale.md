<!-- SPDX-License-Identifier: BUSL-1.1 -->
# DAttackCameraSlopeYawTest — rationale

Source: `DAttackCameraSlopeYawTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for og-ground-aligned-attacks task 21c (2026-11-04, the user's request at G7): with look mode off, on a followed
slope of 15° to 30° + 0.1°, the solo camera's yaw turns to face up the slope at og.cam.slopeYawDegPerSec (90°/s by
default), along the shorter arc, and keeps that heading when the slope is left. The pitch keeps its return to the 60°
target and the frame up its return to world up, so on the 30° face the camera looks straight down the normal. Since
task 21d (2026-11-04, the user's G7 twitch report) the turn starts only once the up-slope heading has held within 10°
for og.cam.slopeYawDwellSec (0.2 s by default). Since task 21e (2026-11-04, the user's G7 re-check) the turn aims at
the heading the dwell started on, not at each tick's, and eases: it speeds up from rest at og.cam.slopeYawAccelDegPerSec2
(360°/s²), caps at og.cam.slopeYawDegPerSec and slows over the last of the gap with the og.cam.slopeYawEaseSec (0.3 s)
time constant, taking the goal exactly within 0.01°. Since task 21f (2026-11-11, the user's request) supported ground
under 15° turns the camera the same way to the shared iso camera's yaw, and the air and ground past 30.1° still hold
it. The reasoning is in og-brawler's `DAttackCamera-rationale.md` §20.
The Unreal host's part (the console variables, and filling the ground and the iso yaw) is in the game module, which
this target cannot compile.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[CameraSlopeYaw]` selects them on their own,
`[CameraSlopeDwell]` selects task 21d's six, `[CameraSlopeEase]` task 21e's five and `[CameraIsoYaw]` task 21f's seven.

## 2. The frames the cases use

Camera space is the host's: Z up, angles in degrees, the pitch below the horizon. `upTiltedToward(d, ψ)` is a ground up
tilted `d` degrees away from heading `ψ`, so the slope rises toward `ψ`; it is built in double and rounded once. Every
target up goes through `cameraTargetUp`, as the host's does, so the cut-off and the airborne rule are the shipped ones.
The look-mode-off input carries full mouse and stick, which look mode off ignores, and the 60° target pitch. Cases run
at 30, 60, 144 and 240 fps where a rate matters.

## 3. The cases

**Task 21c's five cases run with the dwell, the ease and the acceleration at 0** (`twentyOneCSettings`, the defaults
with `slopeYawDwellSeconds`, `slopeYawEaseSeconds` and `slopeYawAccelDegPerSec2` 0, `REQUIRE`d in the first case), and
are otherwise unchanged. With the easing off the turn is 21d's constant-rate turn (task 21e), and a dwell of 0 is 21c's
camera bit for bit on these inputs (`SlopeDwellZeroIsTwentyOneC` below), so
they keep testing what they were written for: the arrival frames, the threshold, the leave and look mode, at 21c's own
formulas. The dwell's shift is tested separately, at the default, by `SlopeDwellSteadySlopeTurnsAsTwentyOneCAfterTheDwell`.
Restating the five with the shift would have changed every arrival and first-step expectation for no new coverage.

* `DAttackCamera.SlopeYawLookModeOffTurnsToFaceUpTheSlope`: five 30° faces, rising along Unreal +X, +Y and −X (whose
  up-slope heading is the yaw seam, ±180°), and the G3 capture's two measured swing ups in Unreal axes (the two faces of
  DAttackCameraSlopeFrameTest-rationale.md §3). The production goal `upSlopeYawDeg` agrees with a double `atan2(−u.y, −u.x)` within 1e-4°.
  From 8 start yaws (the goal plus 10.1°, −47.3°, 100.7°, −179.1°, 133.3°, −20.2°, 64.9° and −91.9°, none near a whole
  number of steps at any rate) at pitch 40° with the frame up on the face, the yaw reaches the goal's bits after exactly
  `ceil(gap / (90 × dt))` frames (4 to 478), every step goes the shorter way and none exceeds the rate by 1e-4°. It then
  holds the goal's bits for 600 frames, the pitch is 60° and the frame up world up (bits), `worldView` draws the goal
  yaw at 60°, the rise along the view (task 21b's formula) is +30° within 1e-3°, and the view direction equals −n
  within 1e-4 per component.
* `DAttackCamera.SlopeYawTurnsFromFifteenDegreesUpToTheFollowedCutOff`, at 60 fps from yaw 33°: faces of 15°,
  14.9995° (inside the threshold's 0.001° tolerance), 20° and 30.09° turn (the first step is −1.5°, and after 600 frames
  the yaw is the goal's bits, about 0°). Faces of 14.998°, 14.9° and 12°, flat ground, the 1-ULP-short flat up, a 30°
  face while unsupported, and faces of 30.11°, 45° and 60° (past the cut-off) leave the yaw's bits unchanged for 600
  frames while the pitch still returns to 60°. The exact 15° face measures 14.9999762° in float, which is why the
  threshold has a tolerance.
* `DAttackCamera.SlopeYawHeadingStaysWhenTheSlopeIsLeft`: after the turn on the 30° face, a target up of world up, of
  an unsupported 30° face and of a 12° face each leave the yaw's bits unchanged for 600 frames, at every rate.
* `DAttackCamera.SlopeYawLookModeOnTurnsOnlyWithThePlayer`: in look mode with the 30° target, 600 still frames keep the
  yaw's bits, and a 0.2-unit mouse turn gives the same yaw bits on every frame as the same turn with a world-up target.
* `DAttackCamera.SlopeYawRateIsTheSettingAndZeroHolds`: at 45°/s the arrival frame is `ceil(gap / (45 × dt))`; a rate
  of 0 and of −30 leave the yaw's bits unchanged for 600 frames while the pitch and the frame up still return.

Task 21d's six cases (`[CameraSlopeDwell]`) run at the default dwell, 0.2 s, with the ease and the acceleration at 0
(`twentyOneDSettings`, `REQUIRE`d in the first case; `SlopeDwellZeroIsTwentyOneC` starts from it too), so they keep
testing 21d's constant-rate turn unchanged. Their inputs never offer a goal within 10° of a running dwell's heading
except in the tolerance case, which checks only the first turning frame, so aiming at the dwell's heading (task 21e)
changes none of their expectations. They run at 30, 60, 144 and 240 fps (`kFps`;
`dt = 1 / fps`). The dwell's first turning frame is `dwellFrameAt(fps)` = `(fps + 4) / 5`, the integer `ceil(0.2 × fps)`:
6, 12, 29 and 48. `kSlopeUp` is the 30° face rising along Unreal +X (goal 0°), `kEdgeUp` a 30° face whose goal is 40°
off it.

* `DAttackCamera.SlopeDwellFlickeringTargetNeverTurns`: for 1 s (`fps` frames) from yaw 100.7°, the target alternates
  the slope with flat ground every tick and in 2-tick runs (each in both orders), and with an unsupported (airborne)
  ground every tick. The yaw keeps its bits on every frame and the last dwell is at most 2 frames, while the same input
  with the dwell at 0 does move the yaw (the twitch, reproduced), and the pitch, frame up and hold-off are the dwell-0
  run's bits on every frame.
* `DAttackCamera.SlopeDwellEdgeFlickerHoldsUntilTheSlopeIsSteady`: from 170° off the goal, 1 s on the slope (the yaw is
  turning and still more than 10° from the goal), then `fps + 1` ticks alternating the 40°-off goal and the slope,
  starting and ending on the 40°-off one: the yaw keeps its bits from the first of them. Then on the steady slope it keeps
  them for `dwellFrameAt(fps) − 1` frames, and on frame `dwellFrameAt(fps)` it takes exactly 21c's step
  (`slopeYawStep` at 90 × dt). The flicker ends on a 40°-off tick because a slope tick at its end would already count
  as the first steady frame (the first run of this case did that and turned one frame early).
* `DAttackCamera.SlopeDwellToleranceIsTenDegrees`: the slope's goal alternating every tick with one 9.9° or −9.9° off
  turns first on frame `dwellFrameAt(fps)`; 10.1° or −10.1° off never turns in 1 s.
* `DAttackCamera.SlopeDwellSteadySlopeTurnsAsTwentyOneCAfterTheDwell`: the five 30° faces, 8 start yaws and 4 rates of
  the first 21c case. The yaw keeps its start bits for `dwellFrameAt(fps) − 1` frames; from frame `dwellFrameAt(fps)` on
  it equals the dwell-0 run's yaw `dwellFrameAt(fps) − 1` frames earlier, bit for bit, through the arrival and 600 held
  frames; it arrives on frame `dwellFrameAt(fps) − 1 + ceil(gap / (90 × dt))`; the pitch, frame up and hold-off are the
  dwell-0 run's bits on every frame. The state's own dwell sum crosses 0.2 s (with the 1e-4 s tolerance) exactly between
  frames `dwellFrameAt(fps) − 1` and `dwellFrameAt(fps)`, and at 240 fps its 48-frame sum is below 0.2: the reason for
  the tolerance.
* `DAttackCamera.SlopeDwellLookModeOnResetsTheDwell`: after `dwellFrameAt(fps) − 1` and `dwellFrameAt(fps) + 30`
  look-off frames on the slope, one look-mode frame (a still mouse and a 0.2-unit turn) gives the same yaw, pitch,
  frame up and hold-off bits as from the same state with no dwell, and leaves the dwell at 0. Back in look mode off the
  yaw keeps its bits for `dwellFrameAt(fps) − 1` frames and moves on the next.
* `DAttackCamera.SlopeDwellZeroIsTwentyOneC`: with the dwell at 0 and at −1, for 2 s at each rate and from the 8 start
  yaws, a steady slope, the slope/flat flicker every tick and in 2-tick runs, the edge flicker and flat ground give, on
  every frame, the same yaw, pitch, frame up and hold-off bits as 21c's look-mode-off step rebuilt in the test from the
  public helpers (`rotateUpToward`, `stepToward`, `clampPitchDeg`, `slopeYawStep`).

Task 21e's five cases (`[CameraSlopeEase]`) run at the defaults (dwell 0.2 s, ease 0.3 s, acceleration 360°/s², rate
90°/s, arrival snap 0.01°, all `REQUIRE`d in the first), at the four rates. `easeOnASteadySlope` runs one start yaw on
a steady face until the yaw holds the goal's bits (at most `kMaxFrames`), then 600 more frames, and records the first
turning frame, the arrival frame, steps against the first step's direction, frames where the remaining gap changed
sign (overshoots) or grew, the largest state speed and measured speed (the step over dt), the largest rise of the
state speed in one frame, the largest gap between the step and the state speed × dt, and, over the 600 held frames,
the frames whose yaw is not the goal's bits or whose speed is not 0. The turning time is the arrival frame minus the
first turning frame plus one, over the fps.

* `DAttackCamera.SlopeEaseTurnsTowardTheDwellsAnchor`: from 20° off on `kSlopeUp` (goal 0°), 6 s on the steady slope
  arrive at the goal's bits with speed 0. Then `fps + 1` ticks alternate a face whose goal is 5°, −5°, 9.9° or −9.9° off
  with the slope, starting and ending on the off one. The yaw keeps the goal's bits and the speed stays 0 on every
  tick, and the dwell runs on (over 6 s) on the goal's heading. The same alternation from the 20°-off start, while the
  yaw is still turning, gives the steady run's yaw and speed bits on every frame. At 10.1° and −10.1° off the yaw also
  holds, and the dwell has restarted (it is one frame, on the off face's heading).
* `DAttackCamera.SlopeEaseFromFiveDegreesStartsGently`: the five 30° faces from 5° and −5° off. The first turning frame
  is `dwellFrameAt(fps)`, and its step over dt, and the state speed, are at most `min(90, gap / 0.3, 360 × dt)`
  (12, 6, 2.5 and 1.5°/s at the four rates).
* `DAttackCamera.SlopeEaseRampsCapsAndArrivesExactlyAtEveryFrameRate`: the five 30° faces from 5°, −5°, 30°, −91.9°,
  180° and −179.1° off. At every rate: the turn starts on `dwellFrameAt(fps)`; no wrong-way step, overshoot or growing
  gap; the state speed is at most 90 and the measured one at most 90 + 0.01°/s; the state speed rises by at most
  360 × dt + 1e-5 per frame (the 1e-5 covers the float rounding of `rate + accel × dt`, up to half an ULP: 3.8e-6
above 64°/s); every step
  matches the state speed × dt within 1e-4°; the arrival is the goal's bits with speed 0, held for 600 frames; from 90°
  or more off the speed reaches exactly 90. Across the rates, every pair of turning times differs by at most the
  coarser frame plus 2 % of the longer time. The measured times are in og-brawler's §20.
* `DAttackCamera.SlopeEaseRateIsZeroWheneverTheTurnIsGated`: from 120° off, after `dwellFrameAt(fps) + fps / 2`
  frames (the speed above 0), one frame of flat ground, of the air, of a goal 40° off, of look mode with a still mouse,
  or of a turn rate of 0 leaves the yaw's bits and sets the speed to 0 exactly. Back on the slope the turn restarts from
  rest: after `dwellFrameAt(fps)` frames for the first four, which restart the dwell, and on the first frame for the
  rate of 0, which does not; its first speed is at most 360 × dt.
* `DAttackCamera.SlopeEaseNegativeOrNaNSettingsAreZero`: an ease or an acceleration of −1 or NaN, alone or both, gives
  the yaw and speed bits of the same setting at 0 on every frame for 5 s from the six start gaps, and arrives.

**Task 21c's, 21d's and 21e's 16 cases run unchanged** since task 21f. Their inputs set only the first five fields
(`lookOff` and the look-mode inputs), so `groundSupported` is false and no frame has the near-flat goal: the "flat
ground" of those cases is ground the camera is not told it stands on, which holds the yaw as before. Task 21f's seven
cases (`[CameraIsoYaw]`) run at the defaults (dwell 0.2 s, ease 0.3 s, acceleration 360°/s², `REQUIRE`d in the flat
case) at the four rates. `lookOffOnGround(up, supported, isoYaw)` is `lookOff` of `cameraTargetUp(up, supported)`
with the three new fields set, as the host fills them, and `turnToGoal` runs one input until the yaw holds the goal's
bits, then 600 more frames, counting steps against the start gap's sign, gap sign changes (overshoots), the largest
state speed and, over the 600 frames, yaw changes and non-zero speeds.

* `DAttackCamera.IsoYawInputsDefaultToTwentyOneE`: a default input's ground is world up, unsupported, iso yaw 0, and a
  five-value positional input and `lookOff`'s input have no goal.
* `DAttackCamera.IsoYawFlatGroundTurnsToTheIsoYaw`: flat ground, a 0.67° and a 1.9° face, and two 10° faces (rising
  toward 0° and 137°), supported, at iso yaws 0° and 45°, from the 8 start offsets of the first 21c case. The first
  turning frame is `dwellFrameAt(fps)`; the yaw arrives at the wrapped iso yaw's bits with speed 0, with no step
  against the shorter arc and no overshoot; the first step's speed is at most `min(90, gap / 0.3, 360 × dt)` (eased);
  the state speed never exceeds 90; the 600 held frames keep the goal's bits with speed 0, and the pitch is 60°.
* `DAttackCamera.IsoYawSeededCameraIsAlreadyAtTheGoal`: `seedFromUERotation(−60, ueYawDeg)` for og.iso.yaw 0, 45, −135,
  90, 180, −180, 33.3 and 370, on supported flat ground with `isoYawDeg = yawDegFromUEYawDegrees(ueYawDeg)`: the goal is
  the seed's yaw bits, and 600 frames keep them with speed 0 while the dwell runs past 0.2 s.
* `DAttackCamera.IsoYawSlopeBandIsTwentyOneEs`: 16 grounds at iso yaws 0°, 45° and 90°. Flat ground, the 1-ULP-short
  up, 12°, 14.9° and 14.998° (outside the threshold's tolerance) have the iso goal; 14.9995° (inside it), 15°, 20°, 30°
  and 30.09° turn up the slope, with `lookOffYawGoal`'s goal the up-slope heading's bits; 30.11°, 45°, 60° and three
  unsupported grounds have none. For the slope band, 6 s from the 8 start offsets at each rate give the same yaw,
  pitch, frame up, hold-off, dwell and speed bits on every frame as the same input with the new fields unset: task
  21e's camera, whatever the iso yaw.
* `DAttackCamera.IsoYawHoldsUnsupportedAndOnSteepGround`: unsupported flat ground and supported 45°, 30.11° and 60°
  faces, at both iso yaws, from yaw 100.7°: 600 frames keep the yaw's bits, the speed and the dwell stay 0, and the
  pitch returns to 60°.
* `DAttackCamera.IsoYawCreaseKeepsOrRestartsTheAnchor`: the three 30° faces rising along Unreal +X (the built one and
  the two G3 normals), iso yaw 0. Arrived on flat ground, and separately arrived on the slope, the camera then crosses
  slope / flat in 1 s runs twice, then alternates them every tick for 1 s: the yaw keeps the arrival's bits, the
  speed stays 0, the dwell grows on every frame (never restarts) and keeps the arrival's heading. With iso yaw 90°, a
  camera arrived on flat ground at 90° that steps onto the slope restarts the dwell on its first slope frame (`dt` on
  the up-slope heading), keeps the yaw's bits with speed 0 for `dwellFrameAt(fps) − 1` frames, and on frame
  `dwellFrameAt(fps)` steps toward the slope's heading at no more than 360 × dt.
* `DAttackCamera.IsoYawLookModeOnIsUnchanged`: from a dwell part-way through on flat ground at iso yaw 45°, look mode
  with a still mouse keeps the yaw's bits for 600 frames and sets the dwell to 0, and a 0.2-unit mouse turn gives the
  same state bits on every frame as the same input with the new fields unset.

## 4. Seen failing

The task's poison, the goal's sign reversed (`atan2(u.y, u.x)`, the camera turning to face down the slope), built and
run under the UBT lock on 2026-11-04: `test cases: 5 | 3 passed | 2 failed`, `assertions: 1714 | 1381 passed |
333 failed`. The turn case fails the −n check on every face, start and rate:

```
CHECK( gapToMinusNormal <= 1e-4 )
with expansion:
  1.0 <= 0.0001
with messages:
  30 deg rising along Unreal +X: up-slope yaw -180 (double 0)
  ...
  view (-0.5, -6.12323e-17, -0.866025), rise along the view -30 deg, largest
  component gap to -n 1
```

with `CHECK( riseDeg == Catch::Approx(30.0).margin(1e-3) )` giving `-30.0000004453 == Approx( 30.0 )`; the threshold
case fails its first-step and settled-yaw checks. Restored from the copy (md5 identical) and rebuilt: all 5 pass.

Task 21d's poison, the dwell gate removed (the `if (slopeDwellElapsed(...))` line in `integrate` deleted, so every
look-mode-off frame steps the yaw), built and run under the UBT lock on 2026-11-04: `test cases: 20 | 15 passed |
5 failed`, `assertions: 4029 | 3485 passed | 544 failed` (with the `[CameraSlopeFrame]` cases). Five of the six dwell
cases fail; 21c's five and `SlopeDwellZeroIsTwentyOneC` still pass, as they should with no gate. The flicker case:

```
DAttackCameraSlopeYawTest.cpp(494): FAILED:
  CHECK( yawChanges == 0 )
with expansion:
  30 == 0
with message:
  fps 30, slope/flat every tick for 1 s: gated yaw changes 30, with the dwell
  at 0 30, pitch/frame-up mismatches 0, last dwell 0 s
```

Restored from the copy (md5 identical) and rebuilt: all 11 pass. (The line numbers in these quotes are those of the
file at the time.)

Task 21e's poisons, each built and run (`[CameraSlopeYaw]`) under the UBT lock on 2026-11-04 from a copy, then
restored (md5 identical) and rebuilt, after which all 16 pass:

* **the goal is the tick's heading** (task 21d's goal: `integrate` passes `wrapYawDeg(upSlopeYawDeg(input.targetUp))`
  to `slopeYawEase` instead of the dwell's heading): `test cases: 16 | 15 passed | 1 failed`, `assertions: 5432 |
  5384 passed | 48 failed`, all in the anchor case:

  ```
  DAttackCameraSlopeYawTest.cpp(864): FAILED:
    CHECK( yawChanges == 0 )
  with expansion:
    31 == 0
  with message:
    fps 30, up-slope heading alternating 0 / 5 deg every tick for 1 s after
    arriving: yaw changes 31, non-zero rates 31, last dwell 7.03333 s on heading
    0
  ```

  and its while-turning check: `CHECK( turningMismatches == 0 )` gives `171 == 0`.
* **no easing** (the step is `maxRateDegPerSec × dt` whatever the speed): `test cases: 16 | 14 passed | 2 failed`,
  `assertions: 5432 | 5272 passed | 160 failed`. The 5° case:

  ```
  DAttackCameraSlopeYawTest.cpp(913): FAILED:
    CHECK( run.firstStepRateDegPerSec <= boundDegPerSec + kRateToleranceDegPerSec )
  with expansion:
    90.0000024587 <= 12.0100006258
  with message:
    30 deg rising along Unreal +X, fps 30, start 5 deg off: first turning frame 6
    (dwell frame 6), first step 90 deg/s (state 12), bound min(90, 5 / 0.3, 360 *
    dt) = 12
  ```

  and the ramp case's step-against-speed check (`worstStepVsRateDeg`, 120 failures).
* **the Backlog's `|e| / τ`** in place of the exact factor (og-brawler §20): `test cases: 16 | 15 passed | 1 failed`,
  `assertions: 5432 | 5427 passed | 5 failed`, the frame-rate check, once per face:

  ```
  DAttackCameraSlopeYawTest.cpp(978): FAILED:
    CHECK( inconsistentPairs == 0 )
  with expansion:
    1 == 0
  with message:
    30 deg rising along Unreal +X, start -91.9 deg off: turning seconds at 30/60/
    144/240 fps 3.1 / 3.16667 / 3.19444 / 3.20417, worst pair excess over one
    frame + 2 % 0.00675 s
  ```

Task 21f's poisons, each a byte replacement in `DAttackCamera.h`, built and run (`[CameraSlopeYaw]`) under the UBT lock
on 2026-11-11 from a copy, then restored (md5 identical) and rebuilt, after which all 23 pass. Task 21e's quoted line
numbers above are unchanged: task 21f only appended to the file.

* **no near-flat goal** (task 21e's behaviour: the `turnsToIsoYaw` branch deleted from `lookOffYawGoal`):
  `test cases: 23 | 18 passed | 5 failed`, `assertions: 10181 | 8873 passed | 1308 failed`. The flat case fails on
  every ground, iso yaw, rate and start (320 times on each of four lines):

  ```
  DAttackCameraSlopeYawTest.cpp(1228): FAILED:
    CHECK( run.firstTurnFrame == dwellFrameAt(fps) )
  with expansion:
    -1 == 6
  with message:
    flat ground (0 deg), iso yaw 0, fps 30, start 10.1 deg off: first turning
    frame -1 (dwell frame 6), arrival frame -1, first step 0 deg/s (bound 12),
    wrong-way steps 0, overshoots 0, held yaw changes 600, final yaw 10.1
  ```

  and the seed, band, crease and look-mode cases fail where they need the goal. The 16 task 21c–21e cases and the hold
  case pass.
* **the near-flat test reads the target up** (`turnsToIsoYaw(input.targetUp, ...)`, which is world up past 30.1°):
  `test cases: 23 | 21 passed | 2 failed`, `assertions: 10593 | 10512 passed | 81 failed`. The 30.11°, 45° and 60° faces
  get the iso goal; the band case fails `CHECK( goal.turns == (band.expected != kHolds) )` with `true == false` on them, and
  the hold case:

  ```
  DAttackCameraSlopeYawTest.cpp(1355): FAILED:
    CHECK( yawChangesFrom(states, start.yawDeg) == 0 )
  with expansion:
    595 (0x253) == 0
  with message:
    45 deg face, supported, iso yaw 0, fps 30: final yaw 0, final dwell 20 s
  ```

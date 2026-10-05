<!-- SPDX-License-Identifier: BUSL-1.1 -->
<!-- lint-external-ref: FRotator -- the engine's own rotator type; owned by the host engine, not by this repository, and must not resolve here -->
<!-- lint-external-ref: FVector::RightVector -- the engine's own unit right vector; owned by the host engine, not by this repository, and must not resolve here -->
# DAttackOrbitCameraTest — rationale

Source: `DAttackOrbitCameraTest.cpp`. This file has no guards document: nothing in it carries a `⛔` tag.

## 1. What this file is for

Pins for og-attackstatetransition-cleanup tasks 8 and 10: the pure orbit camera `dAttackCameraBehaviour::integrate`
(og-brawler `DAttackCamera.h`, rationale `DAttackCamera-rationale.md`). Every behaviour case runs at five
frame steps, 1/30, 1/60, 1/100, 1/144 and 1/240 s. The camera it replaced was tested at 1/60 only, which is
how its pitch PID could be unstable above about 98.5 fps without a red test.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[OrbitCamera]` selects them on their own.

## 2. The rig

* `runPath(dt, phases, start)` runs `integrate` frame by frame. A phase is a duration, `lookHeld`, a mouse
  RATE in units per second and a stick value. Each frame gets `rate × dt` mouse units, so the same phase
  is the same mouse path at every frame rate (a displacement spread over time, as a real mouse reports
  it). Phase lengths are rounded to whole frames; every duration used is a whole number of frames at all
  five steps except where a case says otherwise.
* `lookActionFromMouseCounts(right, up)` models the Unreal host: it reads the Look action's `Mouse_XY`
  modifier from `buildDefaultContext`, requires it to be `KeyModifier::Negate` (both axes negated) and
  applies the host's 0.07 mouse sensitivity. A change to the mapping fails the sign case here instead of
  silently flipping the camera.
* `yawDifferenceDeg` compares yaws modulo 360°, because the state wraps them.
* `holdoffAndDominanceOn()` returns the default settings with task 8's vertical hold-off (0.6 s) and
  dominance ratio (2) switched back on. Since task 10 both are off by default; the cases that pin the
  mechanisms themselves run with this.

## 3. The cases

* `DAttackOrbitCamera.SamePathSameAnglesAtEveryFrameRate`: twelve paths (stick right, half tilt and left;
  tilt-then-pan, where, with the hold-off on, it runs out part-way through a frame at 100 and 144 fps; a
  30° and a 45° stick diagonal; a capped mouse flick; a mouse diagonal on its own and followed by a pan; a
  slow pan with vertical drift; mouse and stick summed; tilt, release, pan), each run twice: with the
  defaults and with `holdoffAndDominanceOn()`. Each path's final yaw and pitch must be within 0.1° of the
  1/60 s run. The diagonals are the paths where the player's tilt and the pull meet in one frame; with the
  pull applied after the player's step instead of solved together with it (`DAttackCamera-rationale.md`
  §7.1), the 30° stick diagonal ended up to 0.41° away.
* `DAttackOrbitCamera.PullFollowsTheClosedFormBelowTheCap`: three paths whose pull stays below the cap
  (the case requires it): a half-response stick right, a full 45° up-right stick and a 3:5 mouse diagonal,
  all from 30° toward 60°. Every frame matches `p(t) = rest + (30 − rest)·exp(−k t)` within 0.01°, with
  `k = yaw rate / e-fold` and `rest = 60 + pitch rate / k` (60°, 47.78° and 72°). It replaced task 8's
  ResearchPullCurveForAFullRightStick (retired in task 10: it pinned the research's 90° e-fold curve, which
  is no longer the default).
* `DAttackOrbitCamera.AFortyFiveDegreeTurnClosesMostOfTheGap`: a 45° turn in 0.5 s (mouse right, mouse left,
  half-response stick) from 30° toward 60° closes at least 85 % of the gap and never passes 60° (task 10
  user decision: a strong pull). Measured 89.5 %; task 8's defaults closed 39 %.
* `DAttackOrbitCamera.MouseIsADisplacement`: one mouse count (0.07 units) turns 0.175°, at every step;
  twice the units turn twice the yaw, and twice the vertical units twice the pitch.
* `DAttackOrbitCamera.StickDeadzoneAndCurve`: 0.14 and 0.15 give no motion; just past 0.15 the response
  is below 1e-5 and the curve rises monotonically with no step larger than 0.003 on a 0.00085 grid (no
  jump at the edge); half-way tilt gives 0.5^1.5; 1.0 and over give 1. A (0.12, 0.12) stick moves the
  camera, which a per-axis deadzone would not. Full right for 0.5 s turns 90°.
* `DAttackOrbitCamera.PitchClampHoldsUnderTenSecondsOfVerticalInput`: ten seconds of full vertical stick,
  mouse, or both with yaw, never leave [10°, 80°] and end exactly on the limit. Host limits outside the hard
  range are clamped to [1°, 89°], and reversed limits collapse to the minimum.
* `DAttackOrbitCamera.PullConvergesWithoutOvershootWithinTheRateCap`: from below and above the target, by
  stick and by mouse flick, left and right: the pitch moves monotonically, never passes the target, never
  moves faster than the cap (`dAttackCameraBehaviour::kPullMaxDegPerSec`, 240°/s since task 10), and the mouse flicks reach exactly the cap (so the cap is live, not just
  unreached). Left and right pulls are equal, which covers the negative yaw steps.
* `DAttackOrbitCamera.NoPullDuringVerticalInputOrTheHoldoffWhenSwitchedOn` (task 8's
  NoPullDuringVerticalInputOrTheHoldoff, renamed in task 10 and run with `holdoffAndDominanceOn()`): a
  mouse diagonal under the hold-off speed but at a 3:5 slope (dominance weight 0) moves the pitch by exactly
  the player's 18°; after a tilt the pitch does not move for 0.6 s of panning, then moves; a release longer
  than the hold-off lets the first pan frame pull; a 4°/s tremor does not start the hold-off.
* `DAttackOrbitCamera.PullActsDuringDiagonalsAndRightAfterVerticalInput` (task 10, defaults): the default
  hold-off and dominance ratio are both 0; the same 3:5 mouse diagonal ends more than 10° past the player's
  own 18°; a 45° stick diagonal from 60° ends more than 10° above where the player's tilt alone would take
  it, and below 60°; after a full tilt to the 10° limit the very first pan frame already pulls (at least
  0.5° at 240 fps), and 0.5 s of panning ends within 1° of 60°. The hold-off timer stays 0 throughout.
* `DAttackOrbitCamera.HostPullSettingsAreSanitized` (task 10): an e-fold of 0, −5 or 0.001 still pulls
  monotonically to 60° with finite values and never past it; a cap of 0 or −10 means no pull; a negative
  hold-off and a negative dominance ratio give exactly the default result.
* `DAttackOrbitCamera.NoPullWithoutLookInput`: five seconds of `lookHeld` with no input leave yaw and pitch
  exactly unchanged (no idle spring-back). Vertical-only stick and mouse input move the pitch by exactly the
  player's step and leave the yaw alone: no yaw, no pull.
* `DAttackOrbitCamera.NothingMovesWithoutLookHeld`: full mouse and stick without `lookHeld` move nothing on
  any frame, and the hold-off still counts down to 0.
* `DAttackOrbitCamera.SignPins`: the Look binding is `Negate`; mouse right and stick right raise the yaw;
  mouse up and stick up lower the pitch below the horizon (look up); the invert flags flip the pitch only.
  Where Unreal's math is on the include path (§4), `FRotator(-pitch, yaw, 0)` for yaw 90° and pitch 60°
  points along `FVector::RightVector` horizontally and 60° down: a larger yaw looks right, which is what
  "yaw right" means for the host's boom write.
* `DAttackOrbitCamera.BoomLengthKeepsTheOldCurve`: 900 at or above the target, 400 at the horizon, 650
  half-way, with the target clamped into the limits.
* `DAttackOrbitCamera.YawStaysWrapped`: ten seconds of full-right stick keep the yaw inside [−180°, 180°]
  and end at the start yaw plus 1800°, modulo 360.

## 4. The Unreal part is compiled only where Unreal's math is

The last block of the sign case uses Unreal's rotator and vector types. It is inside
`#if OG_ORBIT_CAMERA_TEST_HAS_UNREAL_MATH`, which is 1 when Unreal's Math/Rotator.h header is on the include
path: the UBT test target depends on Core, so it is 1 there. The standalone CMake build of og-brawler-tests
has no Unreal headers and skips that block.

## 5. Seen failing (task 8, 2026-10-04)

The PID camera this replaced was run through a frame-rate sweep before it was deleted (horizontal stick,
5 s, from 0.3 rad and from the iso pitch; the evidence file is kept in the initiative's workspace, not in
this repository). It failed 17 of 30 assertions. At 144 and 240 fps, from both starts, the pitch swung to
within 1° of straight down and of straight up, and ended between −88.2° and −19.6° (seeded at the iso pitch,
144 fps ended at −27.3°). At 100 fps from 0.3 rad it ended at −81.9°; seeded at the iso pitch it held,
but ended 0.11° from the 60 fps run. At 30 fps from 0.3 rad it ended 3.7° from the 60 fps run.

Each poison arm below rewrote one statement of `DAttackCamera.cpp`, rebuilt, and ran `[OrbitCamera]` and
`[CameraIsoSeed]`:

| arm | change | failing cases |
|---|---|---|
| whole-frame hold-off | the pull starts on the first frame that begins with the hold-off at 0 | frame-rate sweep (tilt-then-pan at 100 fps: 0.19° off) |
| mouse × dt | the mouse value scaled by the frame step (×60 so 60 fps is unchanged) | 4 |
| mouse normalized | the old camera's 0.1 threshold and normalization | 4 |
| no dominance weight | weight fixed at 1 | the vertical-input case |
| no cap | the cap removed | the pull case |
| mouse x sign | the minus on the mouse x dropped | 2 (sign, displacement) |
| integer `abs` | the hold-off test's absolute value truncated to an integer | 2 (frame-rate sweep, vertical-input case) |
| pull while released | a small pull while `lookHeld` is false | 2 |

## 6. Seen failing (task 10, 2026-10-05)

The four new cases and the extended `DAttackOrbitCamera.NoPullWithoutLookInput` were run against task 9's
camera (the pre-task-10 `integrate` and defaults: hold-off 0.6 s, dominance 2, e-fold 90°, cap 60°/s). Five
cases failed, 78 assertions:

* `DAttackOrbitCamera.PullFollowsTheClosedFormBelowTheCap`: the mouse diagonal stayed at the player's own 48°
  (closed form 62.63°), the 45° stick diagonal ran into the 10° limit (closed form 47.78°). The half-response
  stick passed: the case reads the e-fold constant, so it followed the old 90° curve on the old code.
* `DAttackOrbitCamera.AFortyFiveDegreeTurnClosesMostOfTheGap`: 39.3 % of the gap closed (41.80° at 30 fps).
* `DAttackOrbitCamera.PullActsDuringDiagonalsAndRightAfterVerticalInput`: the defaults were 0.6 and 2, the
  mouse diagonal ended at 48.0°, the stick diagonal at the player-only 21.1° with the hold-off running.
* `DAttackOrbitCamera.HostPullSettingsAreSanitized`: an e-fold of −5 drove the pitch to 8° at 30 fps (below the 10°
  limit), a cap of −10 moved it 10° away from the target, a hold-off of −1 was stored as the timer.
* `DAttackOrbitCamera.NoPullWithoutLookInput`: a rig error in the first draft (0.2 s is 28.8 frames at
  144 fps, rounded to 29), not the camera. The case now uses 0.5 s, a whole number of frames at all five
  steps, and passes on both versions.

Poison arms on the task-10 `DAttackCamera.cpp`, each a one-statement change, rebuilt, `[OrbitCamera]` and
`[CameraIsoSeed]` run:

| arm | change | failing cases |
|---|---|---|
| sequential pull | the pull applied to the clamped result of the player's step (the plain reading of the brief) | the frame-rate sweep (30° and 45° stick diagonals, mouse diagonal) and the closed form |
| hold-off ungated | the vertical-intent test without the `verticalHoldoffSeconds > 0` gate | the diagonal case, the closed form, the sanitization case |
| e-fold unsanitized | the host e-fold used as given | the sanitization case |

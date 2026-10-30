<!-- SPDX-License-Identifier: BUSL-1.1 -->
# DAttackCameraFollowTest — rationale

Source: `DAttackCameraFollowTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for og-ground-aligned-attacks task 16 (2026-10-29): the shared iso camera's follow step moved out of
the Unreal actor into the engine-free `dAttackCameraBehaviour::followStep` and `dAttackCameraBehaviour::interpTo`,
and its single position deadzone became a hysteresis (start above one distance, keep following until
within a smaller one, then hold). The Unreal actor only calls these functions, and this target cannot
compile it. The reasoning is in og-brawler's `DAttackCamera-rationale.md` §15.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[CameraFollow]` selects them on their own.

## 2. The references are the engine and today's actor

* `todaysInterpTo` is the engine's VInterpTo. Where Unreal's math is on the include path (the UBT target
  depends on Core) it calls the engine function itself; the standalone CMake build gets a transcription
  with the same operations in the same order.
* `todaysActorStep` is the actor's follow statements as they were before task 16: move by VInterpTo only
  when FVector::Dist from the camera to the target is above the deadzone. Under UBT it is those
  statements verbatim.

Both are compared by their bits, so "today's behaviour" means the same double, not a close one.

## 3. The constants are the actor's

`kActorStartCm` (100), `kActorStopCm` (10) and `kActorLerpSpeed` (6) are the Unreal actor's property
defaults (`m_positionDeadzoneCm`, `m_positionStopCm`, `m_positionLerpSpeed` in
SharedIsometricCameraActor.h in the game module), kept equal by hand because this target cannot include
that header. `kReportedSpeedCmPerSecond` (570) is the speed of the fitted distance in the user's report
(about 1 m/s of separation over tan 10°). `kDeltaSeconds` covers 30, 60, 120 and 144 Hz; the steady
error before a step is `v / speed` at every rate (95 cm here), so every rate sits just inside the start
distance.

## 4. The cases

* `DAttackCamera.FollowInterpIsTheEngineVInterpTo`: 3 positions × 6 offsets (zero, two below the snap
  distance, one 95 cm, two large) × 6 time steps (including 0, negative and one that clamps the fraction
  to 1) × 5 speeds (including 0 and negative): 540 calls, every result bit-identical to `todaysInterpTo`.
* `DAttackCamera.FollowWithTheStopAtTheStartIsTodaysActorRule`: 900 ticks of a target whose x is 5.7 m/s or
  1.2 m/s times the elapsed time (switching every 150 ticks), which wobbles 40 cm sideways, sinks 15 cm/s
  and is 4 m further sideways on every 200th tick, at the four rates: with the
  stop equal to the start, and with a stop of 5000 cm (above the start, so capped to it), every position
  is bit-identical to `todaysActorStep`, and both moving and held ticks occur.
* `DAttackCamera.FollowTracksAConstantSpeedTargetWithoutHoldTicks`: a target moving at 5.7 m/s along x for
  3 s from the camera's position. After the first start (tick 6, 11, 22 and 26 at the four rates) the
  camera moves forward on every tick, never past the target and never off the axis, and the error stays
  below 2 m.
* `DAttackCamera.FollowWithOneDeadzoneStopsAndGoesOnAConstantSpeedTarget`: the same target with the stop
  equal to the start (the actor's former rule) holds the camera on some ticks after the first start, and
  still in the last second, at every rate: the user's report, measured (10, 15, 23 and 27 hold ticks).
* `DAttackCamera.FollowNeverStartsOnAJitteringStationaryTarget`: a target alternating sides of a rest
  point by 10 to 40 cm along x, y, z and a diagonal, 600 ticks at each rate: the camera never starts and
  its position keeps its bits.
* `DAttackCamera.FollowChasesAJumpToTheStopThresholdThenHolds`: a 2 m jump at 60 Hz is chased for 29
  ticks (`200 × 0.9^28` is 10.47 cm, `200 × 0.9^29` is 9.42 cm), the error shrinking on every tick, then
  held for 300 ticks at 9.42 cm; an 80 cm sideways move of the target does not restart the camera, a
  150 cm one does.

## 5. Seen failing

Task 16 made `followStep` compare with the start distance whether or not it was tracking (the one
deadzone the actor had before). Two cases failed, 8 of 89 assertions: the constant-speed case at all four
rates (10, 15, 23 and 27 hold ticks after the first start) and the jump case (the camera held after 7
ticks at 95.66 cm, and the 80 cm move restarted it). The engine, today's-rule, one-deadzone and jitter
cases stayed green. The impl notes quote the run.

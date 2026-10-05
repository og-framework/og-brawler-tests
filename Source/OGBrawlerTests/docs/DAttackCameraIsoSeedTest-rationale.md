<!-- SPDX-License-Identifier: BUSL-1.1 -->
<!-- lint-external-ref: FRotator -- the engine's own rotator type; owned by the host engine, not by this repository, and must not resolve here -->
# DAttackCameraIsoSeedTest — rationale

Source: `DAttackCameraIsoSeedTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for og-attackstatetransition-cleanup task 7: the solo third-person camera starts at the shared iso
camera's angle, and the camera's target pitch is the iso pitch. Task 8 replaced the PID camera these cases
were first written against with a pure orbit camera (`DAttackOrbitCameraTest.cpp` pins its rules), and
rewrote the four cases against its state: `dAttackCameraBehaviour::seedFromUERotation`,
`dAttackCameraBehaviour::pitchDegFromUEPitchDegrees` and `dAttackCameraBehaviour::integrate`. The
production seed is in the Unreal character, which this target cannot compile.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[CameraIsoSeed]` selects them on their own.

## 2. The rig

* `runHorizontalLook(dt, state, target, seconds)` runs integrate with the look held and the look stick
  full right, and records the pitch and boom length after every frame.
* `requireHoldsTargetAtFullLength` runs it for 5.25 s at each of the five frame steps (1/30 to 1/240 s). It
  also requires the yaw to have moved by more than 1°: without that, a rig in which integrate never ran
  (no look held) would pass every pitch check. 5.25 s rather than a round number, because 10 s at 180°/s
  is exactly 1800°, which is 0 modulo 360.

## 3. The cases

* `DAttackCamera.IsoTargetPitchStaysInsideThePitchLimits`: an Unreal pitch of −60° is 60° below the
  horizon and −45° is 45°. Every input from +400° to −400° lands inside the default limits [10°, 80°], the
  ends are exact, and narrower host limits are respected.
* `DAttackCamera.IsoSeedHoldsPitchAndFullLengthUnderHorizontalLook`: seeded from the iso rotation at five
  iso yaws (including 180° and 400°, which wraps to 40°): the seed's pitch is the target, its hold-off is 0
  and its yaw is the iso yaw. Under horizontal look the pitch error is exactly 0 on every frame and the boom
  stays exactly 900. The pull is a fraction of the error, so at the target it is 0.
* `DAttackCamera.OffTargetSeedConvergesToTheIsoPitch`: seeded at 17.19° and 74.48° (the 0.3 and 1.3 rad of
  the task-7 version), 4 s of horizontal look end within 0.05° of 60°, never pass it, end more than 10° from
  the retired 0.8 rad (45.8°) target, and leave the boom within 5 of 900. With the 0.3 rad start the PID camera
  was still 0.2° off after 10 s at 60 fps, after overshooting by about 4°.
* `DAttackCamera.UnrealIsoRotatorSeedReadsAsTheTargetPitch`: with Unreal's own rotator, for four iso yaws:
  `FRotator(-60, isoYaw, 0)` seeds a state with pitch 60, and the boom rotator the host writes,
  `FRotator(-pitch, yaw, 0)`, points the same way as the iso rotator (equal forward vectors and
  quaternions, 60° down). Then the seed holds under horizontal look.

## 4. The Unreal case is compiled only where Unreal's math is

The last case uses Unreal's rotator type. It is inside `#if OG_ISO_SEED_TEST_HAS_UNREAL_MATH`, which is 1
when Unreal's Math/Rotator.h header is on the include path. The UBT test target depends on Core, so it is
1 there. The standalone CMake build of og-brawler-tests has no Unreal headers, compiles the other three
cases, and skips this one.

## 5. History

Task 7 (2026-10-04) wrote these cases against the PID camera: a boom quaternion read back through
`glm::eulerAngles`, a radian target clamped to [1°, 89°], and the actor-relative seed rotation. It saw them
fail on three poison arms (the legacy 0.8 rad target, a sign flip, no clamp). The quaternion and the
actor-relative rotation went with that camera. The orbit camera's state has the pitch as a plain number of
degrees below the horizon, so the sign question is a negation, and task 8's poison arms (in
`DAttackOrbitCameraTest-rationale.md` §5) also ran these four cases.

<!-- SPDX-License-Identifier: BUSL-1.1 -->
# DAttackCameraIsoSeedTest — rationale

Source: `DAttackCameraIsoSeedTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for og-attackstatetransition-cleanup task 7: the solo third-person camera starts at the shared iso
camera's angle, and the look PID's target is the iso pitch instead of a literal 0.8 rad. The production
side is `AOGBrawlerUECharacter::Tick` (its rationale §14), which this target cannot compile. The cases
here pin the engine-free half: `dAttackCameraBehaviour::targetPitchFromUEPitchDegrees` and what
`dAttackCameraBehaviour::integrate` does with its result.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[CameraIsoSeed]` selects them on their own.

## 2. The rig

* `seededState(pitch, yaw)` builds the boom the way integrate reads it back: a glm quaternion from the
  euler angles (0, pitch, yaw), with the length set to 900.
* `runHorizontalLook` runs integrate for N ticks at 60 Hz with BlockLook held and a purely horizontal
  look axis (1, 0), and records the pitch and length after every tick.
* `productionPitchPidSettings` is a COPY of the gains at the call site in `AOGBrawlerUECharacter::Tick`
  (0.03, 0.01, 0.01). The gains are a local literal there, so a retune there does not reach this file.
* `requireHoldsTargetAtFullLength` also requires the yaw to move. Without that, a rig in which integrate
  returned early (no BlockLook, or a look axis under its 0.1 dead band) would pass every pitch check.

## 3. The cases

* `DAttackCamera.IsoTargetPitchStaysInsideTheOpenQuarterTurn`: −60° gives π/3, −45° gives π/4, and every
  input from +400° to −400° lands strictly inside (0, π/2). integrate's `std::clamp(currentPitch, 0, targetPitch)`
  is undefined for a negative target.
* `DAttackCamera.IsoSeedHoldsPitchAndFullLengthUnderHorizontalLook`: seeded at the target, at four yaws,
  600 ticks of horizontal look. Measured worst pitch error about 1e-5 rad, shortest length 899.995.
* `DAttackCamera.OffTargetSeedConvergesToTheIsoPitchNotTheLegacyTarget`: seeded at 0.3 and 1.3 rad. After
  600 ticks the pitch is within 0.01 of π/3 (measured 1.0511 and 1.0459) and more than 0.2 from 0.8. The
  production gains settle slowly with an overshoot: from 0.3 the pitch is 0.94 after 1 s and 1.10 after 5 s.
* `DAttackCamera.UnrealIsoRotatorSeedReadsAsTheTargetPitch`: the sign measurement. `FRotator(-60, isoYaw, 0)`
  made relative to `FRotator(0, actorYaw, 0)`, as the character's seed does, then copied raw into a glm
  quaternion as `uglm::toGLMMat4` does. For four actor yaws and two iso yaws the euler pitch is +1.0472
  and the euler roll about 0. Then the same seed holds under horizontal look.

## 4. The Unreal case is compiled only where Unreal's math is

The last case uses Unreal's rotator and quaternion types. It is inside `#if OG_ISO_SEED_TEST_HAS_UNREAL_MATH`,
which is 1 when Unreal's Math/Rotator.h header is on the include path. The UBT test target depends on Core, so it is
1 there. The standalone CMake build of og-brawler-tests has no Unreal headers, compiles the other three
cases, and skips this one. `uglm` itself lives in a UE module this target does not build, so the case
repeats its rotation copy (`glm::quat(W, X, Y, Z)`, no handedness change) instead of calling it.

## 5. Seen failing (task 7, 2026-10-04)

Each arm rewrote the helper's body, rebuilt, and ran `[CameraIsoSeed]`:

| arm | body | result |
|---|---|---|
| legacy | `return 0.8f;` (the old call-site target) | 3 of 4 cases fail: π/3 check, the 0.8 distance check (0.0026 > 0.2), the Unreal sign check |
| sign flip | the clamp without the minus sign | 3 of 4 fail: π/3 check, length 857.8 instead of 900, the Unreal sign check |
| no clamp | the conversion without the clamp | 1 fails: the range check, at +30° |

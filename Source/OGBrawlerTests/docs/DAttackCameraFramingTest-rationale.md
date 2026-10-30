<!-- SPDX-License-Identifier: BUSL-1.1 -->
<!-- lint-external-ref: FMath -- the engine's own math library; owned by the host engine, not by this repository, and must not resolve here -->
# DAttackCameraFramingTest — rationale

Source: `DAttackCameraFramingTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins for og-ground-aligned-attacks task 15 (2026-10-29): the shared iso camera's framing distance moved out
of the Unreal actor into the engine-free `dAttackCameraBehaviour::isoFramingDistance`, with separate
horizontal and vertical margins and a field-of-view axis (`dAttackCameraBehaviour::FovAxis`), and the solo
camera's boom lengths and field of view became settings (`OrbitCameraSettings`). The Unreal actor and
character only call these functions, and this target cannot compile them.

Tags: `[DAttack]` puts every case in the `[@og]` whitelist; `[CameraFraming]` selects them on their own.

## 2. The references are today's code

* `todaysActorDistance` is the iso actor's distance formula as it was before task 15, one margin on both
  axes and a horizontal field of view. Where Unreal's math is on the include path (the UBT target depends
  on Core) it is the actor's statements verbatim, with `FMath`; the standalone CMake build gets a
  transcription with the same operations in the same order (`FMath::Max(A, B)` is `(B < A) ? A : B`, and
  `FMath::Clamp` is a max of a min).
* `todaysBoomLength` is `boomLength` as it was, with the old 900 and 400 as literals.

Both are compared by their bits, so "today's behaviour" means the same float, not a close one.

## 3. The geometry check is independent of the code under test

`visibleOnTheShorterAxis` computes, in double precision, what a camera at the returned distance shows when
the authored field of view spans the shorter screen axis: on a landscape screen the vertical half-extent
is `distance × tan(fov / 2)` and the horizontal one that times the aspect; on a portrait screen the other
way round. It does not call `horizontalFovDeg`.

`renderedHalfExtents` models the renderer instead of trusting the conversion: given the `FieldOfView` the
host writes (the output of `horizontalFovDeg`), it returns both half-extents the engine's projection
shows, transcribed from CalculateProjectionMatrixGivenViewRectangle in
Engine/Source/Runtime/Engine/Private/Camera/CameraStackTypes.cpp (no aspect constraint, perspective,
the non-legacy vertical path). With MaintainXFOV the half-angle is `FieldOfView / 2`, the horizontal
multiplier 1 and the vertical one the window's aspect; with MaintainYFOV the half-angle is
`atan(tan(FieldOfView / 2) / 1.777778)` (the camera component's default reference aspect), the horizontal
multiplier `1 / aspect` and the vertical one 1. The visible half-extent is the distance times the
half-angle's tangent divided by the multiplier (the perspective matrix's diagonal is multiplier / tangent).
`kCamerasAxisConstraint` is MaintainXFOV because both cameras set that override in their constructors
(SharedIsometricCameraActor.cpp and OGBrawlerUECharacter.cpp in the game module, which this target cannot
compile), so it has to be kept equal to them by hand. Comparing the model to the geometry checks that
framing and rendering agree.

## 4. The cases

* `DAttackCamera.IsoFramingOnTheHorizontalAxisIsTodaysActorFormula`: 5 × 5 half-extents, 7 aspects
  (the four target shapes, square, 5:4 and 2560 × 1080), 5 fields of view (including 0.05°, which takes
  the 0.001 tangent floor), 3 margins and 2 distance limits: 5250 calls, every result bit-identical to
  `todaysActorDistance` with both margins equal and `FovAxis::Horizontal`.
* `DAttackCamera.HorizontalFovReturnsTheAuthoredBitsWhereNoConversionApplies`: on the horizontal axis at
  every aspect, and on the shorter axis at a portrait or a square aspect, `horizontalFovDeg` returns
  its input's bits.
* `DAttackCamera.TheShorterAxisDefaultsLookLikeTodayOnSixteenByNine`: the iso default of 11.33° on the
  shorter axis renders within 0.006° of the old 20° horizontal on 16:9 (20.0025°), and the solo camera's
  task-15 default of 58.72° within 0.006° of the old 90° (90.0053°), so not exactly. Since
  og-ground-aligned-attacks task 18 the solo default is 40° (the user's tuned value), which the case also
  pins; 58.72° is now a literal there. The console value of an axis maps 0 and below to horizontal and 1 and above to
  the shorter axis (`fovAxisFromConsoleValue`).
* `DAttackCamera.IsoFramingOnTheShorterAxisTouchesTheBindingAxisAtEveryAspect`: at 4:3, 16:9, 21:9 and
  9:16, three fields of view and five boxes (wide, tall, square, empty, 16:9-shaped), with 400 / 300
  margins: the padded box touches the frame on one axis within 1 cm and fits inside on the other, and both
  rendered half-extents (`renderedHalfExtents` with `kCamerasAxisConstraint`) are within 1 cm of the
  geometric ones.
* `DAttackCamera.UnderTheLocalPlayerDefaultOnlySixteenByNineRendersAsFramed`: at the four aspects, for the
  iso and the solo default, the cameras' constraint renders the geometric half-extents within 1 cm, and
  the local player's default (MaintainYFOV) renders both of them scaled by aspect / 1.777778 (0.75 on 4:3,
  1 on 16:9, 1.3125 on 21:9, 0.316 on 9:16): why the override is there, and what the build before task 15
  showed off 16:9.
* `DAttackCamera.IsoFramingSeparatesTheHorizontalAndVerticalMargins`: a larger margin on the binding axis
  moves the camera back; one on the other axis does not move it.
* `DAttackCamera.IsoFramingHonoursTheDistanceClamp`: on both axes and at every aspect, a tiny box gives the
  minimum, a huge one the maximum, and a box between them the unclamped distance.
* `DAttackCamera.BoomLengthWithTodaysConstantsAsSettingsIsTodaysCurve`: with 900 / 400 as settings, 112
  pitch and target pairs (pitches below 0 and above the target, targets outside the limits) give
  `todaysBoomLength`'s bits; the defaults are 2500 / 550 (since og-ground-aligned-attacks task 18; 1200 / 550
  from task 15).

## 5. Seen failing

Task 15 swapped the aspect test in `horizontalFovDeg` (`aspect > 1` became `aspect < 1`: the conversion
ran on portrait screens and not on landscape ones, so the field of view sat on the longer axis). Three
cases failed, 130 of 204 assertions: the touch case at all four aspects (30 assertions each, 9:16 and 21:9
included), the 16:9 defaults case (11.33 and 58.72 came back unconverted) and the no-conversion case on
the portrait aspect. The horizontal-axis bit case, the margin case, the clamp case and the boom case
stayed green. The impl notes quote the run.

Rework 1 (2026-10-29) set `kCamerasAxisConstraint` to MaintainYFOV, which is what the cameras rendered
before the override: two cases failed, 102 of 296 assertions, the touch case (90) and the local-player
case (12), at 4:3, 21:9 and 9:16 (34 each), with 16:9 green. At 9:16, 11.33° and the (1500, 200) box the
model rendered (601.172, 1068.75) where the framing needs (1900, 3377.78). The first version of the test
modelled the render with `horizontalFovDeg` itself and could not see this.

The no-conversion case has no NaN aspect. The poisoned build returned a converted angle for a NaN aspect
where the comparison read as written says it should not: the build's floating-point model does not keep
NaN comparisons, so such a check cannot pin anything. The host never passes a NaN (the aspect is a ratio
of two viewport sizes, and the iso actor falls back to 1 without a height).

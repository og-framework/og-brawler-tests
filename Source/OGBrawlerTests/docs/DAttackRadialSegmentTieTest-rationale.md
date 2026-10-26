<!-- SPDX-License-Identifier: BUSL-1.1 -->
# DAttackRadialSegmentTieTest — rationale

Source: `DAttackRadialSegmentTieTest.cpp`. This file has no guards document: nothing in it carries a
`⛔` tag.

## 1. What this file is for

Pins og-sim-metres-yup task 26a (user ruling Q13): `DAttackRadialSequence::getAttackSegment` with a
2-D direction maps a direction parallel to the sequence's initial direction to segment 0, whichever
way the sequence turns.

The lookup takes the oriented angle from the initial direction to the given direction and wraps it
into the sequence's sweep. glm's 2-D oriented angle returns the unsigned angle when the 2-D cross
product is strictly positive and its negation otherwise. For a parallel direction whose dot product
with the initial direction rounds to at least 1, the unsigned angle is exactly 0, so the oriented angle
is a zero: −0 when the cross is 0 or negative, +0 when it rounds positive.

* A sequence that turns with positive velocity keeps that zero, which is segment 0.
* A sequence that turns with negative velocity used to test `angle < 0.f`, which is false for both
  zeros, so it took `2π − angle` = 2π, beyond every segment, and returned the "no segment" value
  (index equal to `DAttackRadialSequence::getAttackPointCount`, state Idle). Task 26a made that test
  `<=`, so −0 and +0 both become +0, which is segment 0.

Task 26 mirrors the planar chart, which swaps which branch a sequence takes. Without this change the
parallel case would change segment there.

Tags: `[DAttack]` puts both cases in the `[@og]` whitelist; `[RadialSegmentTie]` selects them on their
own.

## 2. Why the direction is twice the initial direction

The rig reads the initial direction from `DAttackRadialSequence::getDirection` at index 0 (the same
cos/sin of the first keyframe angle the lookup uses) and passes **twice** that vector.

* Scaling by 2 is exact, so the 2-D cross product of the two vectors is exactly 0, and the dot product
  (about 2) is clamped to 1, so the unsigned angle is exactly 0. Measured on the UE test build: the
  oriented angle is `0x80000000` (−0) for sequences 0 to 4.
* The unit copy itself does **not** give that tie. The shipped initial directions are not exactly unit
  length (for ±π/2 the sine is `0x3f7fffff` / `0xbf7fffff`), so the dot product is `0x3f7ffffe` and the
  unsigned angle is `0x3a000000` (2⁻¹¹ rad), with a zero cross. That case is not what task 26a changed,
  and this file does not pin it (§4).

Hit detection passes unit-length directions, so in a session the case needs a direction whose dot
product with the initial direction rounds to at least 1. That is a measure-zero set; the scaled vector
is just an exact way to reach it.

## 3. The cases

* `DAttack.RadialSegmentTie.ParallelDirectionIsSegmentZeroOnDescendingSequences`: sequences 1 and 3.
  Each must give index 0, the state of keyframe 0, and keyframe 0 and 1's angles. **Seen failing** with
  the old `angle < 0.f`: sequence 1 gave index 6 and sequence 3 index 8, each equal to the sequence's
  `getAttackPointCount`, with the state Idle and both angles 0 (8 failed assertions).
* `DAttack.RadialSegmentTie.ParallelDirectionIsSegmentZeroOnAscendingSequences`: sequences 0 and 2, the
  control. They take the positive-velocity branch, which already returned segment 0, so this case passes
  before and after the change. It shows the two branches now agree.

Each case first `REQUIRE`s that keyframe 1's angle is below keyframe 0's (descending) or above it
(ascending). The lookup picks its branch from the sign of the velocity at keyframe 1; the constructor
starts that velocity from rest at keyframe 0, so its sign is the sign of keyframe 1's angle minus
keyframe 0's. The check therefore stops a table edit from silently moving a sequence to the other
branch. Sequence 4, the vertical swing, ascends and behaves like the control; it is not listed.

## 4. What this file does not pin

The unit-length copy of the initial direction (§2): with a zero cross and an unsigned angle of 2⁻¹¹ rad,
the oriented angle is −2⁻¹¹. The positive-velocity branch wraps that to 2π − 2⁻¹¹ (no segment), and the
negative-velocity branch to +2⁻¹¹ (segment 0). Task 26a leaves that difference as it was.

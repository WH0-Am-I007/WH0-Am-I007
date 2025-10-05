"""Collision detection utilities for planar mechanisms."""

from __future__ import annotations

from typing import List, Sequence, Tuple

from .model import Mechanism, Point

Segment = Tuple[Point, Point]
EPSILON = 1e-9


def _orientation(p: Point, q: Point, r: Point) -> float:
    return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])


def _on_segment(p: Point, q: Point, r: Point) -> bool:
    return (
        min(p[0], r[0]) - EPSILON <= q[0] <= max(p[0], r[0]) + EPSILON
        and min(p[1], r[1]) - EPSILON <= q[1] <= max(p[1], r[1]) + EPSILON
    )


def segments_intersect(seg1: Segment, seg2: Segment, *, inclusive: bool = True) -> bool:
    """Return ``True`` if the two line segments intersect."""

    (p1, q1) = seg1
    (p2, q2) = seg2

    o1 = _orientation(p1, q1, p2)
    o2 = _orientation(p1, q1, q2)
    o3 = _orientation(p2, q2, p1)
    o4 = _orientation(p2, q2, q1)

    if abs(o1) <= EPSILON and _on_segment(p1, p2, q1):
        return inclusive
    if abs(o2) <= EPSILON and _on_segment(p1, q2, q1):
        return inclusive
    if abs(o3) <= EPSILON and _on_segment(p2, p1, q2):
        return inclusive
    if abs(o4) <= EPSILON and _on_segment(p2, q1, q2):
        return inclusive

    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def detect_self_collision(segments: Sequence[Segment]) -> List[Tuple[int, int]]:
    """Detect intersecting pairs among ``segments``."""

    collisions: List[Tuple[int, int]] = []
    for i in range(len(segments)):
        for j in range(i + 2, len(segments)):
            # Adjacent links share a joint and are expected to touch.
            if segments_intersect(segments[i], segments[j]):
                collisions.append((i, j))
    return collisions


def mechanism_self_collision(mechanism: Mechanism) -> List[Tuple[int, int]]:
    """Convenience wrapper using a :class:`Mechanism`."""

    segments = mechanism.forward_kinematics()
    return detect_self_collision(segments)

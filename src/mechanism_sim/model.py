"""Data models describing planar mechanisms."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Sequence, Tuple

Point = Tuple[float, float]


def _validate_lengths(lengths: Iterable[float]) -> None:
    for length in lengths:
        if length <= 0:
            raise ValueError("Link lengths must be positive.")


@dataclass
class Link:
    """A rigid link represented by its length."""

    name: str
    length: float

    def __post_init__(self) -> None:
        _validate_lengths([self.length])


@dataclass
class Joint:
    """A revolute joint connecting two links."""

    name: str
    angle: float = 0.0
    angular_velocity: float = 0.0


@dataclass
class Mechanism:
    """A simple serial-chain planar mechanism."""

    base_position: Point
    links: Sequence[Link]
    joints: Sequence[Joint]
    metadata: Dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if len(self.links) != len(self.joints):
            raise ValueError("Mechanism must have the same number of links and joints.")
        _validate_lengths(link.length for link in self.links)

    @property
    def dof(self) -> int:
        """Return the number of degrees of freedom."""

        return len(self.joints)

    def joint_by_name(self, name: str) -> Joint:
        """Retrieve a joint by name."""

        for joint in self.joints:
            if joint.name == name:
                return joint
        raise KeyError(f"Joint '{name}' not found")

    def forward_kinematics(self) -> List[Tuple[Point, Point]]:
        """Compute the start and end positions of each link."""

        x, y = self.base_position
        theta = 0.0
        segments: List[Tuple[Point, Point]] = []
        for link, joint in zip(self.links, self.joints):
            theta += joint.angle
            start = (x, y)
            x += link.length * math.cos(theta)
            y += link.length * math.sin(theta)
            end = (x, y)
            segments.append((start, end))
        return segments

    def snapshot(self) -> Dict[str, object]:
        """Return a serialisable view of the mechanism state."""

        return {
            "base_position": self.base_position,
            "joints": [
                {"name": joint.name, "angle": joint.angle, "angular_velocity": joint.angular_velocity}
                for joint in self.joints
            ],
            "segments": self.forward_kinematics(),
            "metadata": dict(self.metadata),
        }

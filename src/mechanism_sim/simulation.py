"""Simulation utilities for planar mechanisms."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from .model import Mechanism

ControlInput = Dict[str, float]


@dataclass
class SimulationResult:
    """Represents a single simulation step result."""

    time: float
    segments: List[tuple]
    snapshot: Dict[str, object]


@dataclass
class MechanismSimulator:
    """Advance a :class:`Mechanism` through time."""

    mechanism: Mechanism
    time: float = 0.0
    history: List[SimulationResult] = field(default_factory=list)

    def forward_kinematics(self) -> List[tuple]:
        """Compute current link positions."""

        return self.mechanism.forward_kinematics()

    def step(self, dt: float, controls: Optional[ControlInput] = None) -> SimulationResult:
        """Advance the mechanism by ``dt`` seconds.

        Parameters
        ----------
        dt:
            Time increment.
        controls:
            Optional mapping of joint names to angular velocities.
        """

        if dt <= 0:
            raise ValueError("Time step must be positive.")

        if controls is not None:
            self._apply_controls(controls)

        for joint in self.mechanism.joints:
            joint.angle += joint.angular_velocity * dt

        self.time += dt
        snapshot = self.mechanism.snapshot()
        result = SimulationResult(time=self.time, segments=snapshot["segments"], snapshot=snapshot)
        self.history.append(result)
        return result

    def run(self, dt: float, steps: int, controller: Optional[Callable[[float, Mechanism], ControlInput]] = None) -> List[SimulationResult]:
        """Run ``steps`` iterations of size ``dt``.

        A controller function can be provided which receives the current
        simulation time and mechanism, returning control inputs for the next
        step.
        """

        results: List[SimulationResult] = []
        for _ in range(steps):
            if controller is None:
                controls = None
            else:
                controls = controller(self.time, self.mechanism)
            results.append(self.step(dt, controls=controls))
        return results

    def _apply_controls(self, controls: ControlInput) -> None:
        """Update joint angular velocities according to ``controls``."""

        for name, angular_velocity in controls.items():
            joint = self.mechanism.joint_by_name(name)
            joint.angular_velocity = angular_velocity

    def reset(self) -> None:
        """Reset the simulator state and clear history."""

        self.time = 0.0
        self.history.clear()
        for joint in self.mechanism.joints:
            joint.angle = 0.0
            joint.angular_velocity = 0.0

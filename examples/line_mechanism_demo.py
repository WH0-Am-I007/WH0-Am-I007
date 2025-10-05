"""Demonstration of a simple line-based mechanism simulation."""

from __future__ import annotations

import math
from pathlib import Path

from mechanism_sim import Link, Joint, Mechanism, MechanismSimulator, collision


def build_mechanism() -> Mechanism:
    links = [
        Link(name="link1", length=1.0),
        Link(name="link2", length=0.8),
        Link(name="link3", length=0.6),
    ]
    joints = [
        Joint(name="joint1", angle=math.radians(30), angular_velocity=math.radians(15)),
        Joint(name="joint2", angle=math.radians(-20), angular_velocity=math.radians(10)),
        Joint(name="joint3", angle=math.radians(10), angular_velocity=math.radians(-5)),
    ]
    return Mechanism(base_position=(0.0, 0.0), links=links, joints=joints)


def run_demo(steps: int = 10, dt: float = 0.1) -> None:
    mechanism = build_mechanism()
    simulator = MechanismSimulator(mechanism)

    log_path = Path("mechanism_demo_log.txt")
    with log_path.open("w", encoding="utf-8") as log_file:
        for result in simulator.run(dt=dt, steps=steps):
            collisions = collision.detect_self_collision(result.segments)
            log_file.write(
                f"time={result.time:.2f} segments={result.segments} collisions={collisions}\n"
            )
            print(
                f"t={result.time:.2f}s -> end effector at {result.segments[-1][1]}, collisions={collisions}"
            )
    print(f"Simulation log written to {log_path.resolve()}")


if __name__ == "__main__":
    run_demo()

import math

import pytest

from mechanism_sim.model import Link, Joint, Mechanism
from mechanism_sim.simulation import MechanismSimulator


def test_forward_kinematics_chain():
    links = [Link("l1", 1.0), Link("l2", 1.0)]
    joints = [Joint("j1", angle=0.0), Joint("j2", angle=math.pi / 2)]
    mechanism = Mechanism(base_position=(0.0, 0.0), links=links, joints=joints)

    segments = mechanism.forward_kinematics()
    assert segments[0] == ((0.0, 0.0), (1.0, 0.0))
    end_of_second = segments[1][1]
    assert pytest.approx(end_of_second[0], rel=1e-6) == 1.0
    assert pytest.approx(end_of_second[1], rel=1e-6) == 1.0


def test_simulator_step_updates_angles():
    links = [Link("l1", 1.0)]
    joints = [Joint("j1", angle=0.0, angular_velocity=math.pi)]
    mechanism = Mechanism(base_position=(0.0, 0.0), links=links, joints=joints)
    simulator = MechanismSimulator(mechanism)

    simulator.step(0.5)
    assert math.isclose(mechanism.joints[0].angle, math.pi * 0.5)

    simulator.step(0.5, controls={"j1": math.pi / 2})
    assert math.isclose(mechanism.joints[0].angle, math.pi * 0.75)

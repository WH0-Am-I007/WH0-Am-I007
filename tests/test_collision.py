import math

from mechanism_sim import collision
from mechanism_sim.model import Link, Joint, Mechanism


def make_crossing_segments():
    return [((0.0, 0.0), (1.0, 1.0)), ((0.0, 1.0), (1.0, 0.0))]


def test_segments_intersect_crossing():
    seg1, seg2 = make_crossing_segments()
    assert collision.segments_intersect(seg1, seg2)


def test_segments_intersect_touching():
    seg1 = ((0.0, 0.0), (1.0, 0.0))
    seg2 = ((1.0, 0.0), (2.0, 0.0))
    assert collision.segments_intersect(seg1, seg2)


def test_mechanism_self_collision_detects():
    links = [Link("l1", 1.0), Link("l2", 1.0), Link("l3", 1.0)]
    joints = [
        Joint("j1", angle=0.0),
        Joint("j2", angle=math.pi),
        Joint("j3", angle=0.0),
    ]
    mechanism = Mechanism(base_position=(0.0, 0.0), links=links, joints=joints)
    collisions = collision.mechanism_self_collision(mechanism)
    assert collisions == [(0, 2)]


def test_detect_self_collision_skip_neighbors():
    segments = [
        ((0.0, 0.0), (1.0, 0.0)),
        ((1.0, 0.0), (2.0, 0.0)),
        ((2.0, 0.0), (3.0, 0.0)),
    ]
    assert collision.detect_self_collision(segments) == []

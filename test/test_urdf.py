# Copyright (c) 2026 SoftBank Corp.
#
# Regression tests for the go2 leg macros (see issue #6 / PR #5 review).
# Verifies that leg joint frames are pure translations (no rotation) and
# that a link name prefix does not alter the kinematics.

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

LEGS = ('FL', 'FR', 'RL', 'RR')
LEG_OFFSET_X = 0.1934
LEG_OFFSET_Y = 0.0465
THIGH_OFFSET_Y = 0.0955
CALF_OFFSET_Z = -0.213
FRONT_THIGH_LIMITS = (-1.5708, 3.4907)
REAR_THIGH_LIMITS = (-0.5236, 4.5379)


def expand(prefix=''):
    path = get_package_share_directory('go2_description') + '/robots/go2_description.urdf.xacro'
    doc = xacro.process_file(path, mappings={'prefix': prefix})
    return ET.fromstring(doc.toxml())


def joint(root, name):
    j = root.find(f"joint[@name='{name}']")
    assert j is not None, f'joint {name} not found'
    return j


def origin_of(element):
    o = element.find('origin')
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    return xyz, rpy


@pytest.mark.parametrize('prefix', ['', 'robot_'])
def test_leg_joint_origins_are_pure_translations(prefix):
    root = expand(prefix)
    for leg in LEGS:
        for part in ('hip', 'thigh', 'calf', 'foot'):
            _, rpy = origin_of(joint(root, f'{prefix}{leg}_{part}_joint'))
            assert rpy == pytest.approx([0.0, 0.0, 0.0]), \
                f'{prefix}{leg}_{part}_joint origin must not be rotated, got rpy={rpy}'


@pytest.mark.parametrize('prefix', ['', 'robot_'])
def test_leg_joint_positions(prefix):
    root = expand(prefix)
    for leg in LEGS:
        x_sign = 1 if leg[0] == 'F' else -1
        y_sign = 1 if leg[1] == 'L' else -1
        xyz, _ = origin_of(joint(root, f'{prefix}{leg}_hip_joint'))
        assert xyz == pytest.approx([x_sign * LEG_OFFSET_X, y_sign * LEG_OFFSET_Y, 0.0])
        xyz, _ = origin_of(joint(root, f'{prefix}{leg}_thigh_joint'))
        assert xyz == pytest.approx([0.0, y_sign * THIGH_OFFSET_Y, 0.0])
        for part in ('calf', 'foot'):
            xyz, _ = origin_of(joint(root, f'{prefix}{leg}_{part}_joint'))
            assert xyz == pytest.approx([0.0, 0.0, CALF_OFFSET_Z])


@pytest.mark.parametrize('prefix', ['', 'robot_'])
def test_thigh_limits_differ_between_front_and_rear(prefix):
    root = expand(prefix)
    for leg in LEGS:
        limit = joint(root, f'{prefix}{leg}_thigh_joint').find('limit')
        expected = FRONT_THIGH_LIMITS if leg[0] == 'F' else REAR_THIGH_LIMITS
        assert (float(limit.get('lower')), float(limit.get('upper'))) == pytest.approx(expected)


def test_prefix_does_not_change_kinematics():
    plain = expand('')
    prefixed = expand('robot_')
    for leg in LEGS:
        for part in ('hip', 'thigh', 'calf', 'foot'):
            j0 = joint(plain, f'{leg}_{part}_joint')
            j1 = joint(prefixed, f'robot_{leg}_{part}_joint')
            xyz0, rpy0 = origin_of(j0)
            xyz1, rpy1 = origin_of(j1)
            assert xyz0 == pytest.approx(xyz1) and rpy0 == pytest.approx(rpy1)
            assert j0.get('type') == j1.get('type')


def test_prefix_is_applied_exactly_once():
    prefixed = expand('robot_')
    names = [link.get('name') for link in prefixed.findall('link')]
    names += [j.get('name') for j in prefixed.findall('joint')]
    assert not any('robot_robot_' in n for n in names)
    for leg in LEGS:
        assert f'robot_{leg}_foot_link' in names

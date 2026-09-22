"""Static regression tests for the Hege model.

No ROS node, no Gazebo, no simulator of any kind - these expand the xacro and
read the result. Run them with plain pytest:

    PYTHONPATH= python3 -m pytest src/hege_description/test -q

They exist because the model has four `drive` modes that share one file, and
the ways it can go wrong are quiet ones. Every check below stands for a
specific mistake that was actually made, or that costs an afternoon to find:

  - the Harmonic GPS taking degrees where Classic takes metres, which turns
    2 cm of noise into 2.2 km without a single warning;
  - a Classic plugin leaking into a Harmonic mode or the reverse, which fails
    only when that simulator tries to load it;
  - base_link disappearing from the px4 model through fixed-joint lumping,
    after which PX4 subscribes to four topics that never exist;
  - two publishers ending up on the odom -> base_footprint edge, which does
    not error, it just makes the transform flicker.

The idea, and the GPS units test, come from test_simulation_description.py in
oguzissik/hege_gps_navigation.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip('xacro', reason='needs the xacro module')

MODEL = Path(__file__).resolve().parents[1] / 'urdf' / 'hege.urdf.xacro'


def expand(drive):
    """Expand the xacro for one drive mode and return (text, root element)."""
    doc = xacro.process_file(str(MODEL), mappings={'drive': drive})
    text = doc.toprettyxml(indent='  ')
    return text, ET.fromstring(text)


def properties():
    """The xacro <property> values, before expansion."""
    root = ET.parse(MODEL).getroot()
    ns = '{http://www.ros.org/wiki/xacro}'
    out = {}
    for element in root.iter(f'{ns}property'):
        try:
            out[element.attrib['name']] = float(element.attrib['value'])
        except (KeyError, ValueError):
            pass
    return out


# --------------------------------------------------------------- GPS units

def horizontal_stddev(root):
    """The horizontal position noise the GPS sensor is actually configured
    with, whichever simulator's sensor block it lives in."""
    for sensor in root.iter('sensor'):
        if sensor.attrib.get('name') != 'gps_sensor':
            continue
        # <gps> in Classic, <navsat> in Harmonic; the path below it is the same.
        for block in ('gps', 'navsat'):
            found = sensor.find(f'{block}/position_sensing/horizontal/noise/stddev')
            if found is not None:
                return float(found.text)
    raise AssertionError('no gps_sensor horizontal noise found')


def vertical_stddev(root):
    for sensor in root.iter('sensor'):
        if sensor.attrib.get('name') != 'gps_sensor':
            continue
        for block in ('gps', 'navsat'):
            found = sensor.find(f'{block}/position_sensing/vertical/noise/stddev')
            if found is not None:
                return float(found.text)
    raise AssertionError('no gps_sensor vertical noise found')


def test_harmonic_gps_noise_is_converted_to_degrees():
    """gz-sensors8 applies horizontal NavSat noise to the coordinate in
    DEGREES. The metre value has to be divided before it gets there, or 2 cm
    of intended noise becomes 2.2 km of actual noise, silently."""
    values = properties()
    per_degree = values['metres_per_latitude_degree']
    assert 110_000 < per_degree < 112_000, 'not a metres-per-degree figure'

    _, root = expand('gz')
    configured = horizontal_stddev(root)

    assert math.isclose(configured, values['gps_noise'] / per_degree, rel_tol=1e-9)
    # And, independently of the arithmetic above: it must be an angle.
    assert configured < 1.0e-3, 'horizontal noise is still a metre value'


def test_classic_gps_noise_stays_in_metres():
    """The Classic <gps> sensor takes metres, so applying the same conversion
    there would make the simulated receiver implausibly perfect."""
    values = properties()
    _, root = expand('planar')
    assert math.isclose(horizontal_stddev(root), values['gps_noise'], rel_tol=1e-9)


def test_the_two_simulators_agree_on_the_real_error():
    """Different units, same metres on the ground. This is the check that
    would have caught the bug even if both files looked plausible."""
    values = properties()
    _, classic = expand('planar')
    _, harmonic = expand('gz')

    classic_m = horizontal_stddev(classic)
    harmonic_m = horizontal_stddev(harmonic) * values['metres_per_latitude_degree']
    assert math.isclose(classic_m, harmonic_m, rel_tol=1e-6)


def test_vertical_noise_is_metres_in_both():
    """Only the horizontal channel is angular. Converting this one too would
    make the altitude noise 40 000 times too small."""
    values = properties()
    expected = values['gps_noise'] * 2
    for mode in ('planar', 'gz'):
        _, root = expand(mode)
        assert math.isclose(vertical_stddev(root), expected, rel_tol=1e-9), mode


# ------------------------------------------------------- mode isolation

CLASSIC_MARKERS = ['libgazebo_ros', 'gazebo_ros2_control/GazeboSystem']
HARMONIC_MARKERS = ['gz-sim-', 'gz_ros2_control']


@pytest.mark.parametrize('mode', ['planar', 'ros2_control'])
def test_classic_modes_have_no_harmonic_plugins(mode):
    text, _ = expand(mode)
    for marker in HARMONIC_MARKERS:
        assert marker not in text, f'{marker} leaked into {mode}'


@pytest.mark.parametrize('mode', ['gz', 'px4'])
def test_harmonic_modes_have_no_classic_plugins(mode):
    text, _ = expand(mode)
    for marker in CLASSIC_MARKERS:
        assert marker not in text, f'{marker} leaked into {mode}'


def test_px4_mode_has_no_ros2_control():
    """PX4 drives the joints itself over gz-transport; a controller manager in
    that model would fight it."""
    text, _ = expand('px4')
    assert '<ros2_control' not in text


# ------------------------------------------------------------ PX4 naming

def test_px4_model_carries_the_names_px4_hardcodes():
    """GZBridge.cpp subscribes to fixed topic paths. A model that spells any of
    these differently boots, looks healthy, and starves EKF2 of that sensor."""
    text, root = expand('px4')

    links = {element.attrib['name'] for element in root.iter('link')}
    assert 'base_link' in links, 'base_link was lumped away'

    sensors = {element.attrib['name'] for element in root.iter('sensor')}
    for required in ('imu_sensor', 'navsat_sensor',
                     'magnetometer_sensor', 'air_pressure_sensor'):
        assert required in sensors, required

    # Without the preserved base_joint, base_link lumps into base_footprint
    # during URDF to SDF conversion and the link above disappears.
    assert 'preserveFixedJoint' in text


def test_px4_sensors_sit_at_the_measured_heights():
    values = properties()
    _, root = expand('px4')
    poses = {}
    for element in root.iter('sensor'):
        pose = element.find('pose')
        if pose is not None and pose.text:
            poses[element.attrib['name']] = [float(v) for v in pose.text.split()]

    imu_z = values['imu_height'] - values['base_z']
    gps_z = values['gps_height'] - values['base_z']
    assert math.isclose(poses['imu_sensor'][2], imu_z, abs_tol=1e-6)
    assert math.isclose(poses['navsat_sensor'][2], gps_z, abs_tol=1e-6)


# --------------------------------------------------------- ground truth

def test_gz_mode_publishes_ground_truth_without_owning_tf():
    """The odometry publisher is for scoring estimates, not for driving them.
    Its TF output must not reach the odom -> base_footprint edge, which
    robot_localization's local EKF owns."""
    _, root = expand('gz')
    plugins = [element for element in root.iter('plugin')
               if element.attrib.get('name', '').endswith('OdometryPublisher')]
    assert len(plugins) == 1

    plugin = plugins[0]
    assert plugin.findtext('robot_base_frame') == 'base_footprint'
    assert plugin.findtext('odom_frame') == 'world'
    tf_topic = plugin.findtext('tf_topic') or ''
    assert tf_topic and 'tf' in tf_topic and tf_topic != '/tf', \
        'ground truth TF must go somewhere the bridge does not carry'


def test_only_the_gz_mode_publishes_ground_truth():
    for mode in ('planar', 'ros2_control', 'px4'):
        _, root = expand(mode)
        plugins = [element for element in root.iter('plugin')
                   if element.attrib.get('name', '').endswith('OdometryPublisher')]
        assert not plugins, mode


# ------------------------------------------------------------- geometry

def test_every_mode_describes_the_same_vehicle():
    """The drive argument may change plugins. It may not change the robot."""
    values = properties()
    reference = None
    for mode in ('planar', 'ros2_control', 'gz', 'px4'):
        _, root = expand(mode)
        joints = {}
        for joint in root.iter('joint'):
            origin = joint.find('origin')
            if origin is not None and 'xyz' in origin.attrib:
                joints[joint.attrib['name']] = origin.attrib['xyz']
        wheel_joints = {name: xyz for name, xyz in joints.items()
                        if 'wheel' in name or 'steer' in name}
        if reference is None:
            reference = wheel_joints
        else:
            assert wheel_joints == reference, f'{mode} moved a wheel'

    assert len(reference) >= 4

    # And the steering limit has to match the property, because the PX4
    # airframe carries the same number independently.
    _, root = expand('ros2_control')
    limits = [joint.find('limit') for joint in root.iter('joint')
              if joint.attrib.get('name', '').endswith('steer_joint')]
    limits = [limit for limit in limits if limit is not None]
    assert limits
    for limit in limits:
        assert math.isclose(float(limit.attrib['upper']), values['max_steer'],
                            abs_tol=1e-9)

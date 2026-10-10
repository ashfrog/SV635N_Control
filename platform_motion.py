"""Three vertical supports: calibrated plane geometry and bounded target interpolation.

No hardware imports. X is forward, Y right, Z up; pitch raises the front,
roll raises the right. This model requires joints/sliders that permit tilt.
"""
from copy import deepcopy
import math
import time

from control_common import ControlError, MotionCommand, MotionQueue, Move


def finite(value, label, *, positive=False):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid or (positive and value <= 0):
        raise ValueError(f'{label} must be a finite {"positive " if positive else ""}number')
    return float(value)


def validate_platform(options):
    defaults = dict(enabled=False, calibration_id='', legs=[], max_heave_mm=10.,
                    max_pitch_deg=3., max_roll_deg=3., max_leg_speed_mm_s=5.,
                    max_leg_acceleration_mm_s2=10., pose_timeout_s=.25,
                    command_interval_s=.02, allow_motor_debug=False,
                    require_endpoint_sensors=True, max_tracking_error_mm=5., tracking_timeout_s=.3)
    if not isinstance(options, dict) or set(options) - set(defaults):
        raise ValueError('Unknown platform fields or configuration is not an object')
    config = deepcopy(defaults)
    config.update(deepcopy(options))
    for name in ('enabled', 'allow_motor_debug', 'require_endpoint_sensors'):
        if type(config[name]) is not bool:
            raise ValueError(f'platform.{name} must be boolean')
    if not isinstance(config['calibration_id'], str) or len(config['calibration_id']) > 128:
        raise ValueError('platform.calibration_id must be a string of at most 128 characters')
    for name in ('max_heave_mm', 'max_pitch_deg', 'max_roll_deg', 'max_leg_speed_mm_s',
                 'max_leg_acceleration_mm_s2', 'pose_timeout_s', 'command_interval_s'):
        config[name] = finite(config[name], 'platform.' + name, positive=True)
    for name in ('max_tracking_error_mm', 'tracking_timeout_s'):
        config[name] = finite(config[name], 'platform.' + name, positive=True)
    if not .1 <= config['tracking_timeout_s'] <= 2:
        raise ValueError('platform.tracking_timeout_s must be 0.1..2')
    if max(config['max_pitch_deg'], config['max_roll_deg']) >= 30:
        raise ValueError('Platform tilt envelope must be below 30 degrees')
    if not .1 <= config['pose_timeout_s'] <= 1:
        raise ValueError('platform.pose_timeout_s must be 0.1..1')
    if not .01 <= config['command_interval_s'] <= .05:
        raise ValueError('platform.command_interval_s must be 0.01..0.05')
    legs = config['legs']
    if not isinstance(legs, list) or (legs and len(legs) != 3):
        raise ValueError('platform.legs must contain exactly three supports')
    fields = {'order', 'x_mm', 'y_mm', 'mm_per_rev', 'extension_sign', 'min_mm', 'max_mm'}
    for leg in legs:
        if not isinstance(leg, dict) or set(leg) != fields:
            raise ValueError('Each platform leg requires ' + ', '.join(sorted(fields)))
        if type(leg['order']) is not int or leg['order'] < 1:
            raise ValueError('Leg order must be a positive chain position')
        if type(leg['extension_sign']) is not int or leg['extension_sign'] not in (-1, 1):
            raise ValueError('extension_sign must be +1 or -1 in public motor-degree coordinates')
        for name in fields - {'order', 'extension_sign'}:
            leg[name] = finite(leg[name], name, positive=name == 'mm_per_rev')
        if not leg['min_mm'] < 0 < leg['max_mm']:
            raise ValueError('Leg travel bounds must straddle the physically confirmed neutral')
    if legs:
        if [l['order'] for l in legs] != sorted(set(l['order'] for l in legs)):
            raise ValueError('Leg orders must be distinct and ascending')
        a, b, c = legs
        area = ((b['x_mm'] - a['x_mm']) * (c['y_mm'] - a['y_mm'])
                - (c['x_mm'] - a['x_mm']) * (b['y_mm'] - a['y_mm']))
        if not math.isfinite(area) or abs(area) < 1e-6:
            raise ValueError('Support points must be non-collinear')
    if config['enabled'] and (not legs or not config['calibration_id'].strip()):
        raise ValueError('Enabled platform requires three calibrated legs and calibration_id')
    return config


class PlatformGeometry:
    def __init__(self, options):
        self.config = validate_platform(options)
        self.legs = self.config['legs']
        self.orders = [leg['order'] for leg in self.legs]

    def pose_to_targets(self, pose):
        if not self.config['enabled']:
            raise ControlError('Platform mode is not configured')
        if not isinstance(pose, dict) or set(pose) != {'heave_mm', 'pitch_deg', 'roll_deg'}:
            raise ControlError('pose requires heave_mm, pitch_deg and roll_deg')
        try:
            values = {name: finite(value, name) for name, value in pose.items()}
        except ValueError as exc:
            raise ControlError(str(exc)) from exc
        for name, value in values.items():
            if abs(value) > self.config['max_' + name]:
                raise ControlError(f'{name} exceeds calibrated platform envelope')
        pitch, roll = math.radians(values['pitch_deg']), math.radians(values['roll_deg'])
        # Plane normal from Ry(-pitch) Rx(-roll). Intersect vertical support lines.
        lengths = [values['heave_mm'] + leg['x_mm'] * math.tan(pitch)
                   + leg['y_mm'] * math.tan(roll) / math.cos(pitch) for leg in self.legs]
        targets = []
        for leg, length in zip(self.legs, lengths):
            if not math.isfinite(length) or not leg['min_mm'] <= length <= leg['max_mm']:
                raise ControlError(f"Support {leg['order']} exceeds calibrated travel")
            degrees = length / leg['mm_per_rev'] * 360 * leg['extension_sign']
            if not math.isfinite(degrees):
                raise ControlError('Motor target cannot be represented')
            targets.append(degrees)
        return values, lengths, targets

    def profile(self):
        # A common RPM/acceleration must respect the most restrictive support.
        rpm = min(self.config['max_leg_speed_mm_s'] * 60 / l['mm_per_rev'] for l in self.legs)
        acceleration = min(self.config['max_leg_acceleration_mm_s2'] * 60 / l['mm_per_rev']
                           for l in self.legs)
        if not all(math.isfinite(v) and v > 0 for v in (rpm, acceleration)):
            raise ControlError('Platform motion profile cannot be represented')
        return rpm, acceleration


class PlatformMotionQueue(MotionQueue):
    """Latest desired target, interpolated at a fixed maximum dispatch frequency.

    Common progress across all supports limits each target's linear speed.
    Native APS trajectories enforce acceleration; this is not hardware sync.
    """
    stop_at_any_limit = True
    def __init__(self, geometry, clock=time.monotonic):
        rpm, acceleration = geometry.profile()
        super().__init__(geometry.orders, rpm, acceleration)
        self.geometry, self.clock = geometry, clock
        self.output = (0.,) * 3
        self.last_output = clock()

    def mark_ready(self):
        super().mark_ready()
        self.last_output = self.clock()

    def pop(self):
        with self.lock:
            if not self.ready or self.closed or self.output == self.planned:
                return None
            now = self.clock()
            elapsed = now - self.last_output
            if elapsed < self.geometry.config['command_interval_s']:
                return None
            elapsed = min(elapsed, .05)  # A stalled worker must not issue a catch-up jump.
            deltas = [target - old for target, old in zip(self.planned, self.output)]
            fraction = min([1.] + [self.geometry.config['max_leg_speed_mm_s'] * elapsed
                           * 360 / leg['mm_per_rev'] / abs(delta)
                           for leg, delta in zip(self.geometry.legs, deltas) if delta])
            output = tuple(target if fraction == 1 else old + delta * fraction
                           for old, delta, target in zip(self.output, deltas, self.planned))
            self.last_output, self.output = now, output
            self.commands.clear()
            duration = max(Move(order, delta, self.rpm, acceleration_rpm_s=self.acceleration).estimated_seconds
                           for order, delta in zip(self.orders, deltas))
            return MotionCommand(self.number, output, duration)

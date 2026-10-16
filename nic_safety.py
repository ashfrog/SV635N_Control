"""Native-coordinate end-stop memory and conservative stopping envelopes."""
import json
import math
from copy import deepcopy
from pathlib import Path

from control_common import ControlError

MAX_MOTOR_RPM = 3000.0


def delta(position, reference):
    return (position-reference + 2**31) % 2**32 - 2**31


def stopping_distance(velocity, deceleration, reaction_time, margin, *, acceleration=0, velocity_limit=None):
    """Same units throughout: v*t + v²/(2*a) + position uncertainty."""
    speed = abs(velocity)
    if acceleration:
        future = speed+acceleration*reaction_time
        speed = max(speed, min(future, velocity_limit) if velocity_limit is not None else future)
    return speed*reaction_time + speed*speed/(2*deceleration) + margin


class LimitMemory:
    def __init__(self, path=None):
        self.path = Path(path) if path else None
        self.axes = {}
        self.dirty = False
        if self.path and self.path.exists():
            try:
                data = json.loads(self.path.read_text(encoding='utf-8'))
                if data['version'] != 1 or not isinstance(data['axes'], dict):
                    raise ValueError('invalid schema')
                for key, entry in data['axes'].items():
                    if (not isinstance(entry['identity'], list) or any(
                        type(entry.get(side)) is not int or not -2**31 <= entry[side] < 2**31
                        for side in ('positive', 'negative') if side in entry)):
                        raise ValueError('invalid native position')
                    if 'manual' in entry:
                        validate_manual(entry['manual'])
                        if not entry['manual']['enabled']:
                            entry['manual'] = dict(enabled=False)
                self.axes = data['axes']
            except (OSError, ValueError, KeyError, TypeError) as exc:
                raise ControlError(f'限位位置记忆文件损坏，禁止运行；请核对并重新标定：{exc}') from exc

    def entry(self, adapter, device):
        key = f'{adapter}:{device.order}'
        entry = self.axes.get(key)
        if entry and entry['identity'] != list(device.identity):
            raise ControlError(f'电机 {device.order} 身份或传动参数改变，限位位置记忆须重新标定。')
        return entry or {}

    def _commit_edit(self, axes):
        previous, dirty = self.axes, self.dirty
        if self.path and self.path.exists():
            backup = self.path.with_name(self.path.name+'.before-edit.bak')
            if not backup.exists():
                backup.write_bytes(self.path.read_bytes())
        self.axes, self.dirty = axes, True
        try:
            self.save()
        except Exception:
            self.axes, self.dirty = previous, dirty
            raise

    def reset_records(self, adapter, devices):
        axes = deepcopy(self.axes)
        for device in devices:
            key = f'{adapter}:{device.order}'
            entry = axes.get(key, {})
            if entry.get('identity') == list(device.identity) and 'manual' in entry and not entry.get('automatic'):
                axes[key] = dict(identity=list(device.identity), manual=entry['manual'])
            else:
                axes.pop(key, None)
        self._commit_edit(axes)

    def configure_manual(self, adapter, settings, devices, *, automatic=False):
        known = {d.order:d for d in devices}
        if (not isinstance(settings, list) or not settings or any(not isinstance(s, dict) for s in settings)
                or len({s.get('order') for s in settings if type(s.get('order')) is int}) != len(settings)):
            raise ControlError('请提供不同电机的软限位设置。')
        axes = deepcopy(self.axes)
        for supplied in settings:
            if set(supplied)-{'order','enabled','origin','lower_degrees','upper_degrees'}:
                raise ControlError('软限位设置包含未知字段。')
            device = known.get(supplied.get('order'))
            if device is None or type(supplied.get('order')) is not int:
                raise ControlError('软限位电机不在当前扫描结果中。')
            manual = {k:v for k,v in supplied.items() if k != 'order'}
            validate_manual(manual)
            if manual['enabled']:
                manual_bounds(manual, device)
            else:
                manual = dict(enabled=False)
            # Explicit manual reconfiguration replaces the old learned positions
            # for these axes; current hardware P/N-OT inputs remain authoritative.
            axes[f'{adapter}:{device.order}'] = dict(identity=list(device.identity), manual=manual)
            if automatic:
                axes[f'{adapter}:{device.order}']['automatic'] = True
        self._commit_edit(axes)

    def observe(self, adapter, device, position, positive, negative):
        entry = dict(self.entry(adapter, device))
        # A wiring/conflict state is not evidence of two physical endpoints.
        if positive and negative:
            return entry
        changed = False
        for side, active in (('positive', positive), ('negative', negative)):
            if active and side not in entry:
                entry[side] = position
                changed = True
        if changed:
            entry['identity'] = list(device.identity)
            self.axes[f'{adapter}:{device.order}'] = entry
            self.dirty = True
        return entry

    def save(self):
        # Called after cyclic motion has stopped, never block a PDO cycle on disk IO.
        if self.path and self.dirty:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(version=1, axes=self.axes), indent=2), encoding='utf-8')
            temporary.replace(self.path)
            self.dirty = False

    def protect(self, adapter, device, position, sensor):
        entry = self.observe(adapter, device, position,
                             sensor['positive_limit'], sensor['negative_limit'])
        result = dict(sensor)
        # The physical cam can become clear on EITHER side of a narrow flag.
        # Only encoder retreat past the recorded first edge releases its latch.
        hysteresis = max(2, round(device.units_per_rev*.2/360))
        for side in ('positive', 'negative'):
            latched = side in entry and (delta(position, entry[side]) >= -hysteresis
                                        if side == 'positive' else delta(position, entry[side]) <= hysteresis)
            result[f'{side}_latched'] = latched
            result[f'{side}_limit'] = sensor[f'{side}_limit'] or latched
        for prefix, _, _, sign in device.endpoints:
            stem = '' if prefix == 'extension' else 'retraction_'
            active = result['positive_limit' if sign == 1 else 'negative_limit']
            result[stem+'triggered'] = active
            result[stem+'state'] = ('latched' if active and not sensor[
                'positive_limit' if sign == 1 else 'negative_limit'] else 'triggered' if active else 'clear')
        result['conflict'] = bool(result['positive_limit'] and result['negative_limit'])
        return result


def envelope_bounds(envelope, device):
    """Return native endpoints relative to the saved centre, never an inferred home."""
    if envelope is None:
        return None
    try:
        axis = next(a for a in envelope['reference']['axes'] if a['order'] == device.order)
        leg = next(a for a in envelope['legs'] if a['order'] == device.order)
        if (axis['identity'] != list(device.identity) or type(axis['position']) is not int
                or not -2**31 <= axis['position'] < 2**31):
            raise ValueError('identity changed')
        mm_per_rev, low, high = (leg[k] for k in ('mm_per_rev', 'min_mm', 'max_mm'))
        sign = leg['extension_sign'] * (1 if device.positive_direction else -1)
        if (sign != device.extension_sign or not all(type(x) in (int, float) and math.isfinite(x)
            for x in (mm_per_rev, low, high)) or mm_per_rev <= 0 or not low < 0 < high):
            raise ValueError('travel calibration invalid')
        bounds = sorted(round(x/mm_per_rev * device.units_per_rev * sign) for x in (low, high))
        if bounds[1]-bounds[0] >= 2**31:
            raise ValueError('travel exceeds native range')
        return axis['position'], *bounds
    except (KeyError, ValueError, TypeError, StopIteration) as exc:
        raise ControlError(f'电机 {device.order} 缺少有效固定中位和实测软限位，禁止高速运动。') from exc


def validate_manual(manual):
    if not isinstance(manual, dict) or type(manual.get('enabled')) is not bool:
        raise ControlError('手动软限位 enabled 必须为 true/false。')
    if not manual['enabled']:
        return
    if type(manual.get('origin')) is not int or not -2**31 <= manual['origin'] < 2**31:
        raise ControlError('软限位基准必须是有效的 32 位位置计数。')
    values = [manual.get(k) for k in ('lower_degrees', 'upper_degrees')]
    try:
        valid = all(type(v) in (int, float) and math.isfinite(v) for v in values)
    except OverflowError:
        valid = False
    if not valid or values[0] >= values[1]:
        raise ControlError('软下限、上限必须是有限角度，且下限小于上限。')


def manual_bounds(manual, device):
    if not manual or not manual.get('enabled'):
        return None
    validate_manual(manual)
    factor = 1 if device.positive_direction else -1
    raw = [manual[k]*device.units_per_rev*factor/360 for k in ('lower_degrees', 'upper_degrees')]
    if not all(math.isfinite(v) and abs(v) < 2**31 for v in raw):
        raise ControlError('手动软限位超出驱动器可用坐标范围。')
    low, high = sorted(round(v) for v in raw)
    if not 0 < high-low < 2**31:
        raise ControlError('手动软限位跨度不足一个计数或超出可用范围。')
    return manual['origin'], low, high

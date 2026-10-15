"""Persist explicit, stationary three-axis centres in native APS coordinates."""
import json
import math
from pathlib import Path
import uuid

from control_common import ControlError, Move, MAX_MOVE_SECONDS


class FixedReference:
    def __init__(self, path, adapter, backend):
        self.path = Path(path) if path else None
        self.adapter, self.backend = adapter, backend
        self.data, self.error = None, None
        if self.path and self.path.exists():
            try:
                data = json.loads(self.path.read_text(encoding='utf-8'))
                if (data.get('version') != 1 or not isinstance(data.get('id'), str)
                        or not data['id'] or data.get('adapter') != adapter or data.get('backend') != backend
                        or not isinstance(data.get('axes'), list)
                        or [a.get('order') for a in data['axes']] != [1, 2, 3]):
                    raise ValueError('reference schema/controller mismatch')
                for axis in data['axes']:
                    if (not isinstance(axis.get('identity'), list)
                            or type(axis.get('position')) not in (int, float)
                            or not math.isfinite(axis['position'])):
                        raise ValueError('reference axis data invalid')
                self.data = data
            except (OSError, ValueError, TypeError, AttributeError, KeyError, OverflowError) as exc:
                self.error = f'固定初始位文件无效，请重新确认中位并保存：{exc}'

    def validate(self, devices):
        if not self.data:
            raise ControlError(self.error or '尚未保存固定初始位。')
        known = {d.order: d for d in devices}
        for axis in self.data['axes']:
            d = known.get(axis['order'])
            if (d is None or not hasattr(d, 'units_per_rev')
                    or list(d.identity) != axis['identity']):
                raise ControlError('轴映射、方向或每转单位已变化，固定初始位不可使用，请重新标定。')
        return self.data

    def save(self, devices):
        if not self.path:
            raise ControlError('后台未启用固定初始位存储。')
        known = {d.order: d for d in devices}
        axes = []
        for order in (1, 2, 3):
            d = known.get(order)
            if (d is None or not hasattr(d, 'units_per_rev') or not math.isfinite(d.position)
                    or not math.isfinite(d.units_per_rev) or d.units_per_rev <= 0
                    or d.motor_code != 14101):
                raise ControlError('保存中位需要三台有效的 APS SV635N，链路位置为 1/2/3。')
            axes.append(dict(order=order, identity=list(d.identity), position=d.position))
        data = dict(version=1, id=uuid.uuid4().hex, adapter=self.adapter, backend=self.backend, axes=axes)
        temporary = self.path.with_name(self.path.name + '.' + uuid.uuid4().hex + '.tmp')
        try:
            temporary.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')
            temporary.replace(self.path)
        finally:
            temporary.unlink(missing_ok=True)
        self.data, self.error = data, None

    def status(self, devices, feedback):
        error = self.error
        valid = False
        if self.data:
            try:
                self.validate(devices)
                valid = True
            except ControlError as exc:
                error = str(exc)
        heights = []
        known = {d.order: d for d in devices}
        for axis in self.data['axes'] if valid else []:
            d = known[axis['order']]
            position = feedback.get(d.order, {}).get('position', d.position)
            heights.append((position-axis['position']) / d.units_per_rev * 5 * d.extension_sign
                           if d.extension_sign in (-1, 1) else None)
        return dict(available=self.path is not None, saved=self.data is not None, valid=valid,
                    id=self.data['id'] if self.data else None, error=error, heights_mm=heights)


def reference_targets(data, devices, origins, rpm, acceleration):
    known = {d.order: d for d in devices}
    targets = []
    for axis in data['axes']:
        d = known.get(axis['order'])
        if d is None or list(d.identity) != axis['identity']:
            raise ControlError('固定初始位与当前轴映射/传动参数不匹配。')
        target = (axis['position']-origins[d.order]) * 360 / d.units_per_rev * (1 if d.positive_direction else -1)
        if not math.isfinite(target) or Move(d.order, target, rpm, acceleration_rpm_s=acceleration).estimated_seconds > MAX_MOVE_SECONDS:
            raise ControlError('返回固定初始位的位移过大，请先低速手动调整接近中位。')
        targets.append(target)
    return targets


def check_neutral(data, devices):
    for axis in data['axes']:
        d = next((d for d in devices if d.order == axis['order']), None)
        if d is None or list(d.identity) != axis['identity']:
            raise ControlError('固定初始位与当前轴映射不匹配。')
        if abs(d.position-axis['position']) > max(2., d.units_per_rev * .2 / 360):
            raise ControlError('三轴尚未回到保存的固定初始位，请先执行“返回固定初始位”。')

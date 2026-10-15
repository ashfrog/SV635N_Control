"""Commission the measured three-support assembly without commanding motion."""
import json
import math
from pathlib import Path
import uuid

from control_common import ControlError
from platform_motion import PlatformGeometry, finite


def calibrated_geometry(devices, travel_mm):
    if (not isinstance(travel_mm, list) or len(travel_mm) != 3
            or any(not isinstance(leg, dict) or set(leg) != {'order','up_mm','down_mm'}
                   for leg in travel_mm)
            or [leg['order'] for leg in travel_mm] != [1,2,3]
            or any(type(leg['order']) is not int for leg in travel_mm)):
        raise ControlError('请按 1/2/3 号提供每根撑杆的向上、向下可用行程。')
    known = {d.order:d for d in devices}
    altitude = math.sqrt(730**2-335**2)
    points = [(altitude/3, -335), (altitude/3, 335), (-2*altitude/3, 0)]
    legs = []
    for travel, (x,y) in zip(travel_mm, points):
        d = known.get(travel['order'])
        if (d is None or getattr(d, 'extension_sign', 0) not in (-1,1)
                or getattr(d, 'retraction_sign', 0) != -d.extension_sign
                or not getattr(d, 'extension_input', '') or not getattr(d, 'retraction_input', '')):
            raise ControlError('需要扫描三台电机并核对每轴推出、缩回两端传感器的方向配置。')
        try:
            up = finite(travel['up_mm'], '向上行程', positive=True)
            down = finite(travel['down_mm'], '向下行程', positive=True)
        except ValueError as exc:
            raise ControlError('上下行程必须为大于 0 的有限毫米数值。') from exc
        legs.append(dict(order=d.order, x_mm=x, y_mm=y, mm_per_rev=5,
                         extension_sign=d.extension_sign*(1 if d.positive_direction else -1),
                         min_mm=-down, max_mm=up))
    return PlatformGeometry(dict(enabled=True, calibration_id='platform-'+uuid.uuid4().hex,
        max_heave_mm=min(min(-leg['min_mm'], leg['max_mm']) for leg in legs), legs=legs))


def save_platform_configuration(path, platform):
    path = Path(path)
    original = path.read_bytes()
    config = json.loads(original.decode('utf-8-sig'))
    if not isinstance(config, dict):
        raise ControlError('后台配置文件无效，未保存平台配置。')
    config['platform'] = platform.config
    backup = path.with_name(path.name+'.before-platform.bak')
    if not backup.exists():
        backup.write_bytes(original)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)

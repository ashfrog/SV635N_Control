"""Direct-NIC configuration validation without loading hardware libraries."""

def validate_options(options):
    defaults = dict(limit_inputs_connected=True, extension_limits={}, retraction_limits={},
                    stop_deceleration_rpm_s=120.0, reaction_time_s=.05, position_margin_degrees=36.0)
    if not isinstance(options, dict) or set(options) - set(defaults):
        raise ValueError('nic 配置包含未知字段，或不是 JSON 对象。')
    defaults.update(options)
    if type(defaults['limit_inputs_connected']) is not bool:
        raise ValueError('nic.limit_inputs_connected 必须为 true/false。')
    import math
    for key in ('stop_deceleration_rpm_s', 'reaction_time_s', 'position_margin_degrees'):
        value = defaults[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'nic.{key} 必须为正有限数值。')
    if defaults['reaction_time_s'] < .05:
        raise ValueError('nic.reaction_time_s 不得小于 0.05 秒。')
    for key in ('extension_limits', 'retraction_limits'):
        if not isinstance(defaults[key], dict):
            raise ValueError(f'nic.{key} 必须是以链路位置为键的对象。')
        for order, value in defaults[key].items():
            if (not isinstance(order, str) or order not in {str(n) for n in range(1, 17)}
                    or value not in ('di1', 'di2')):
                raise ValueError(f'nic.{key} 格式应为 {{"1":"di2"}}，链路位置从 1 开始。')
    for order in defaults['extension_limits'].keys() & defaults['retraction_limits'].keys():
        if defaults['extension_limits'][order] == defaults['retraction_limits'][order]:
            raise ValueError(f'电机 {order} 推出和缩回限位不能使用同一路 DI。')
    return defaults



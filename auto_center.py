"""Bounded, synchronized two-endpoint calibration; no hardware IO in the planner."""
import math
from control_common import ControlError


def validate_auto_center(supplied=None):
    options = dict(enabled=False, coupled_axes=True, mm_per_rev=5., nominal_travel_mm=100., minimum_travel_mm=10.,
                   seek_rpm=60., latch_rpm=6., center_rpm=600., step_mm=.02, backoff_mm=.5, repeat_tolerance_mm=.1,
                   release_max_mm=5., center_tolerance_mm=1., max_skew_mm=2., timeout_s=1800.)
    if supplied is not None:
        if not isinstance(supplied,dict) or set(supplied)-set(options):
            raise ValueError('auto_center 包含未知字段。')
        options.update(supplied)
    if any(type(options[key]) is not bool for key in ('enabled','coupled_axes')):
        raise ValueError('auto_center.enabled/coupled_axes 必须为 true/false。')
    for key in set(options)-{'enabled','coupled_axes'}:
        if type(options[key]) not in (int,float) or not math.isfinite(options[key]) or options[key] <= 0:
            raise ValueError(f'auto_center.{key} 必须为正有限数值。')
    if (options['seek_rpm'] > 60 or options['latch_rpm'] > 6 or options['center_rpm'] > 600
            or options['latch_rpm'] > options['seek_rpm']
            or options['step_mm'] > .02 or options['max_skew_mm'] > 2
            or options['timeout_s'] > 3600 or options['backoff_mm'] > 2
            or options['backoff_mm'] >= options['nominal_travel_mm']*.4
            or not options['backoff_mm'] < options['release_max_mm'] <= 5
            or options['minimum_travel_mm'] >= options['nominal_travel_mm']*1.2
            or options['center_tolerance_mm'] > 1
            or options['repeat_tolerance_mm'] >= options['backoff_mm']):
        raise ValueError('自动标定速度、小步距、回退或同步偏差超出允许范围。')
    return options


class CenterPlan:
    """Coordinates are mm from each axis's start; positive means extension."""
    def __init__(self, options, positions, lower, upper):
        self.options = options
        self.count = len(positions)
        if self.count != 3:
            raise ControlError('自动回中需要完整的三根推杆。')
        self.phase, self.revision, self.done = 'seek_lower', 0, False
        self.start = list(positions)
        self.commanded = None
        self.release_clear_start = None
        self.release_members = [True]*self.count
        self.release_opposite = [False]*self.count
        self.latch_search_mm = options['backoff_mm']*2
        self.hits = [None]*self.count
        self.first_lower = self.first_upper = self.lower = self.upper = self.mid = None
        self.coarse_lower = self.coarse_upper = None
        self._check_inputs(lower,upper)
        if options['coupled_axes'] and any(lower) and any(upper):
            raise ControlError('启动时不同轴分别处于上下端，禁止自动寻位。')
        if any(lower) or any(upper):
            self._initial_release(any(lower),positions,lower,upper)

    def _initial_release(self, is_lower, positions, lower, upper):
        self._change('release_initial_lower' if is_lower else 'release_initial_upper',positions)
        self.release_members = ([True]*self.count if self.options['coupled_axes']
                                else list(lower if is_lower else upper))
        self.release_opposite = ([False]*self.count if self.options['coupled_axes']
                                 else list(upper if is_lower else lower))

    @staticmethod
    def _check_inputs(lower,upper):
        if any(a and b for a,b in zip(lower,upper)):
            raise ControlError('上下限位同时触发，停止自动标定。')

    def observe(self, positions, lower, upper):
        self._check_inputs(lower,upper)
        if self.phase.startswith(('release_initial_','backoff_')):
            opposite = upper if self.phase.endswith('lower') else lower
            independent_initial = self.phase.startswith('release_initial_') and not self.options['coupled_axes']
            if any(on and not (independent_initial and self.release_opposite[i])
                   for i,on in enumerate(opposite)):
                raise ControlError('离开限位期间触发另一端限位，停止标定。')
            if independent_initial:
                active = lower if self.phase.endswith('lower') else upper
                if any(on and not self.release_members[i] for i,on in enumerate(active)):
                    raise ControlError('启动回退期间保持轴触发新限位，停止标定。')
                self.release_opposite = [allowed and on for allowed,on in zip(self.release_opposite,opposite)]
        travel = [p-s for p,s in zip(positions,self.start)]
        if self.options['coupled_axes'] and max(travel)-min(travel) > self.options['max_skew_mm']:
            raise ControlError('三轴标定进度偏差超过同步允许值，停止全部推杆。')
        maximum = (self.options['release_max_mm'] if self.phase.startswith(('release_initial_','backoff_'))
                   else self.options['nominal_travel_mm']*1.2)
        if max(abs(t) for t in travel) > maximum:
            raise ControlError('寻限位超过最大搜索距离，传感器可能未响应。')
        if self.phase in ('seek_lower','latch_lower','seek_upper','latch_upper'):
            is_lower = self.phase.endswith('lower')
            active, opposite = (lower,upper) if is_lower else (upper,lower)
            # The opposite switch may still be asserted while departing it.
            if any(opposite):
                departing = self.phase == 'seek_upper' and self.lower is not None
                if not departing or any(opposite[i] and positions[i]-self.lower[i] > self.latch_search_mm
                                        for i in range(self.count)):
                    raise ControlError('寻限位方向与触发端不符，或离开端限位未释放。')
            for i,triggered in enumerate(active):
                if triggered and self.hits[i] is None:
                    self.hits[i] = positions[i]

    def _change(self, phase, positions):
        self.phase, self.start = phase, list(positions)
        self.commanded = None
        self.release_clear_start = None
        self.hits = [None]*self.count
        self.revision += 1

    def targets(self, positions, lower, upper):
        """Called only after arrival or a confirmed, stationary Halt."""
        self.observe(positions,lower,upper)
        step, backoff = self.options['step_mm'], self.options['backoff_mm']
        phase = self.phase
        basis = positions if self.commanded is None else self.commanded
        if phase.startswith(('release_initial_','backoff_')):
            is_lower = phase.endswith('lower')
            active = lower if is_lower else upper
            direction = 1 if is_lower else -1
            members = self.release_members if phase.startswith('release_initial_') else [True]*self.count
            if self.release_clear_start is None:
                if any(active):
                    return [basis[i]+direction*step if members[i] else p for i,p in enumerate(positions)]
                self.release_clear_start = list(positions)
            elif any(active):
                raise ControlError('反向离开时限位重新触发，停止标定。')
            goals = [s+direction*backoff if members[i] else positions[i]
                     for i,s in enumerate(self.release_clear_start)]
            if all(abs(p-g) < .003 for p,g in zip(positions,goals)):
                if phase.startswith('release_initial_'):
                    if is_lower and any(upper):
                        self._initial_release(False,positions,lower,upper)
                        return self.targets(positions,lower,upper)
                    next_phase='seek_lower'
                else:
                    edge=(self.first_lower or self.coarse_lower) if is_lower else (self.first_upper or self.coarse_upper)
                    self.latch_search_mm=max(abs(p-e) for p,e in zip(positions,edge))+backoff
                    next_phase='latch_lower' if is_lower else 'latch_upper'
                self._change(next_phase,positions)
                return self.targets(positions,lower,upper)
        elif phase in ('seek_lower','latch_lower','seek_upper','latch_upper'):
            is_lower = phase.endswith('lower')
            active = lower if is_lower else upper
            if all(hit is not None for hit in self.hits):
                if not all(active):
                    raise ControlError('触限停车后信号已消失，请检查挡片长度和停车余量。')
                edges = list(self.hits)
                if phase == 'seek_lower':
                    self.coarse_lower = edges
                    self._change('backoff_lower',positions)
                elif phase == 'latch_lower':
                    if self.first_lower is None:
                        self.first_lower=edges
                        self._change('backoff_lower',positions)
                        return self.targets(positions,lower,upper)
                    if any(abs(a-b) > self.options['repeat_tolerance_mm'] for a,b in zip(edges,self.first_lower)):
                        raise ControlError('下限位复测位置不一致。')
                    self.lower = [max(a,b) for a,b in zip(edges,self.first_lower)]
                    self._change('seek_upper',positions)
                elif phase == 'seek_upper':
                    self.coarse_upper = edges
                    self._change('backoff_upper',positions)
                else:
                    if self.first_upper is None:
                        self.first_upper=edges
                        self._change('backoff_upper',positions)
                        return self.targets(positions,lower,upper)
                    if any(abs(a-b) > self.options['repeat_tolerance_mm'] for a,b in zip(edges,self.first_upper)):
                        raise ControlError('上限位复测位置不一致。')
                    self.upper = [min(a,b) for a,b in zip(edges,self.first_upper)]
                    spans = [u-l for l,u in zip(self.lower,self.upper)]
                    nominal = self.options['nominal_travel_mm']
                    if any(not self.options['minimum_travel_mm'] <= span <= nominal*1.2 for span in spans):
                        raise ControlError('实测上下行程过短或超出允许机械行程，禁止生成软限位。')
                    self.mid = [(l+u)/2 for l,u in zip(self.lower,self.upper)]
                    self._change('center',positions)
                return self.targets(positions,lower,upper)
            if phase.startswith('latch') and max(abs(p-s) for p,s in zip(positions,self.start)) > self.latch_search_mm:
                raise ControlError('限位复测超过允许回退距离。')
            direction = -1 if is_lower else 1
            if phase.startswith('seek_'):
                # One bounded PP trajectory keeps velocity continuous. Raw
                # overtravel stops it at the first edge; latch passes remain
                # small and slow. Never extend a missing-switch search.
                goals=[s+direction*self.options['nominal_travel_mm']*1.2 for s in self.start]
                if any(self.hits[i] is None and abs(p-goals[i]) < .003 for i,p in enumerate(positions)):
                    raise ControlError('寻限位超过最大搜索距离，传感器可能未响应。')
                return [p if self.hits[i] is not None else goals[i] for i,p in enumerate(positions)]
            return [p if self.hits[i] is not None else basis[i]+direction*step for i,p in enumerate(positions)]
        elif phase == 'center':
            goals = self.mid
            if any(lower) or any(upper):
                if any(lower) or any(upper[i] and self.upper[i]-positions[i] > self.latch_search_mm for i in range(self.count)):
                    raise ControlError('回中期间限位未正常释放。')
            if all(abs(p-g) < .003 for p,g in zip(positions,goals)):
                if any(lower) or any(upper):
                    raise ControlError('中位仍有硬限位信号，禁止完成标定。')
                self.done = True
                return None
            return list(goals)
        else:
            raise ControlError('自动标定状态无效。')
        return [p+max(-step,min(step,g-p)) for p,g in zip(basis,goals)]

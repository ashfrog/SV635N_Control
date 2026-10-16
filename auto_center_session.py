"""PDO-only synchronized calibration, using the existing checked NIC session."""
from collections import deque
import time

from auto_center import CenterPlan
from control_common import ControlError, Move
from continuous_control import _ManualSession
from nic_backend import _NICSession, input_state
import struct


class AutoCenterSession(_NICSession):
    def __init__(self,*args,options):
        super().__init__(*args)
        self.options = options
        self.step_axes = []
        self.halted = False
        self.report['auto_center'] = dict(success=False)

    def tick(self,checked=True,cancellable=True):
        # Homing alone bypasses the OLD coordinate envelope. WKC, fault,
        # enable, bounded-step and raw physical P/N-OT checks remain active.
        _ManualSession.tick(self,checked,cancellable)

    def check_travel(self,axis):
        if 'unwrapped_counts' not in axis:
            return
        margin = axis['counts']/self.options['mm_per_rev']*.2
        low,high = sorted((axis['command_start'],axis['command_goal']))
        if not low-margin <= axis['unwrapped_counts'] <= high+margin:
            raise ControlError('自动标定反馈偏离小步进目标，停止全部轴。')

    def stop_deceleration(self,axis,acceleration):
        # Fast search has a separate Halt ramp; retain the normal ramp floor.
        return super().stop_deceleration(axis,max(acceleration,round(axis['counts']*3000/60)))

    def update_feedback(self,axis):
        _ManualSession.update_feedback(self,axis)
        raw = struct.unpack_from('<I',axis['slave'].input,24)[0]
        self.limits[axis['device'].order] = input_state(axis['device'],raw)
        if not self.armed or not axis['request'] or self.cleaning:
            return
        sensor = self.limits[axis['device'].order]
        if sensor['positive_limit'] and sensor['negative_limit']:
            raise ControlError('自动标定中上下限位同时触发。')
        direction = axis.get('step_direction',0)
        if (axis in self.step_axes and not self.halted and
                (direction > 0 and sensor['positive_limit'] or direction < 0 and sensor['negative_limit'])):
            self._halt()

    def _halt(self):
        if self.halted:
            return
        self.halted = True
        for axis in self.selected():
            axis['cw'],axis['target'] = 0x10F,axis['last'][2]

    def _snapshot(self):
        positions,lower,upper = [],[],[]
        for axis in self.selected():
            device = axis['device']
            sensor = self.limits[device.order]
            if not sensor.get('valid'):
                raise ControlError('自动标定限位反馈失效。')
            positions.append(axis['unwrapped_counts']/axis['counts']*self.options['mm_per_rev']*device.extension_sign)
            lower.append(sensor['retraction_triggered'])
            upper.append(sensor['triggered'])
        return positions,lower,upper

    def _check_plan(self):
        self.check_stop()
        if time.monotonic() > self.finish_deadline:
            raise ControlError('自动寻限位回中总时间超时。')
        self.plan.observe(*self._snapshot())

    def _settle(self):
        self._halt()
        samples = {a['device'].order:deque() for a in self.selected()}
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            self.tick()
            self._check_plan()
            stable = True
            now = time.monotonic()
            for axis in self.selected():
                points = samples[axis['device'].order]
                points.append((now,axis['unwrapped_counts']))
                while len(points)>2 and points[1][0] < now-.05:
                    points.popleft()
                stable &= (points[-1][0]-points[0][0] >= .05 and bool(axis['last'][1]&0x400)
                    and max(p for _,p in points)-min(p for _,p in points) <= max(2,axis['counts']*.02/360))
            if stable:
                return
        raise ControlError('自动标定限位停车未能确认停稳。')

    def _pulse(self,goals):
        self.step_axes = []
        phase=self.plan.phase
        long_move=phase.startswith('seek_') or phase=='center'
        if phase=='center' and any(abs(g-m)>1e-8 for g,m in zip(goals,self.plan.mid)):
            raise ControlError('回中目标不是实测中位。')
        if phase.startswith('seek_'):
            direction=-1 if phase.endswith('lower') else 1
            if any(self.plan.hits[i] is None and
                   abs(g-(self.plan.start[i]+direction*self.options['nominal_travel_mm']*1.2))>1e-8
                   for i,g in enumerate(goals)):
                raise ControlError('寻位目标超出固定搜索终点。')
        maximum_mm=self.options['nominal_travel_mm']*1.2 if long_move else self.options['step_mm']
        duration=0
        for axis,mm in zip(self.selected(),goals):
            goal = round(mm/self.options['mm_per_rev']*axis['counts']*axis['device'].extension_sign)
            remaining = goal-axis['unwrapped_counts']
            if abs(remaining) <= axis['tolerance']:
                axis['cw'],axis['target'] = 0x10F,axis['last'][2]
                continue
            # A phase's fixed search endpoint is anchored before Halt. Tiny
            # coast away from it increases remaining distance, not the allowed
            # search endpoint. Keep the same 0.2 mm feedback-path allowance.
            maximum = (maximum_mm+(.2 if long_move else 0))/self.options['mm_per_rev']*axis['counts']+axis['tolerance']+2
            if abs(remaining) > maximum:
                raise ControlError(f'电机 {axis["device"].order} {phase} 标定目标超过单步距离上限。')
            axis.update(command_start=axis['unwrapped_counts'],command_goal=goal,
                        step_direction=1 if remaining>0 else -1,
                        target=(axis['origin']+goal+2**31)%2**32-2**31,cw=0x3F)
            self.step_axes.append(axis)
            duration=max(duration,Move(axis['device'].order,abs(remaining)*360/axis['counts'],
                self.commands.profile()[1],acceleration_rpm_s=self.commands.profile()[2]).estimated_seconds)
        self.halted = False
        if not self.step_axes:
            self.tick()
            return
        deadline = time.monotonic()+max(1,duration+1)
        stage = 'ack'
        while time.monotonic() < deadline:
            self.tick()
            self._check_plan()
            if self.halted:
                self._settle()
                return
            if stage == 'ack' and all(a['last'][1]&0x1000 for a in self.step_axes):
                for axis in self.step_axes:
                    axis['cw'] = 0x2F
                stage = 'clear'
            elif stage == 'clear' and all(not a['last'][1]&0x1000 for a in self.step_axes):
                stage = 'arrive'
            elif stage == 'arrive' and all(a['last'][1]&0x400 and
                    abs(a['unwrapped_counts']-a['command_goal']) <= a['tolerance'] for a in self.step_axes):
                return
        raise ControlError('自动标定单步未到位或确认超时，可能卡阻。')

    def motion(self):
        for axis in self.selected():
            device=axis['device']
            if not device.extension_input or not device.retraction_input or device.extension_sign != -device.retraction_sign:
                raise ControlError('自动回中必须接入两端有效限位。')
            axis.update(origin=axis['last'][2],previous_position=axis['last'][2],
                        unwrapped_counts=0,command_start=0,command_goal=0)
            axis['target']=axis['origin']
        self.armed=True
        for word,state in ((6,0x21),(7,0x23),(15,0x27)):
            _ManualSession.command(self,word)
            self.wait_all(lambda a:a['last'][1]&0x6F == state,1,'自动标定使能')
        for axis in self.selected():
            axis['enable_verified']=True
        self.hold(.1)
        if any(abs(a['unwrapped_counts'])>a['tolerance'] for a in self.selected()):
            raise ControlError('标定使能期间发生意外位移。')
        self.plan=CenterPlan(self.options,*self._snapshot())
        self.finish_deadline=time.monotonic()+self.options['timeout_s']
        self.commands.mark_ready()
        self.emit('continuous_ready',orders=list(self.commands.orders))
        revision=-1
        while not self.plan.done:
            self._check_plan()
            goals=self.plan.targets(*self._snapshot())
            if self.plan.revision != revision:
                revision=self.plan.revision
                # Switch profiles only after a stationary Halt. Slow initial
                # release and repeat latching, continuous search, then a faster
                # transfer to the measured midpoint inside both known ends.
                self._settle()
                rpm=(self.options['seek_rpm'] if self.plan.phase.startswith('seek_') else
                     self.options['center_rpm'] if self.plan.phase=='center' else self.options['latch_rpm'])
                self.commands.update_profile(rpm,600)
                self.apply_profile()
                # PP position-window acknowledgement can precede the final
                # coast by micrometres. Plan the next small step from the
                # stopped feedback, never from the pre-Halt snapshot.
                goals=self.plan.targets(*self._snapshot())
                if self.plan.revision != revision:
                    continue
                self.report['auto_center']['phase']=self.plan.phase
                self.emit('phase',text='自动寻限位回中：'+self.plan.phase+'；Esc 可停止',
                          auto_center_phase=self.plan.phase)
            if goals is not None:
                self._pulse(goals)
                # Advance the ideal shared step, rather than accumulating each
                # axis's permitted encoder arrival error over thousands of steps.
                # A limit Halt interrupts the step, so re-anchor that step only.
                self.plan.commanded = None if self.halted else list(goals)
        # Bit10 is a position-window indication; it can precede the final
        # deceleration by a few micrometres. Let PP finish before asserting
        # Halt, and correct the residual only after a stationary measurement.
        for attempt in range(3):
            end=time.monotonic()+.2
            while time.monotonic()<end:
                self.tick()
                self._check_plan()
            self._settle()
            if all(abs(p-m)<=self.options['center_tolerance_mm'] for p,m in zip(self._snapshot()[0],self.plan.mid)):
                break
            if attempt==2:
                raise ControlError('回中停稳后误差超限，未保存软行程。')
            self._pulse(self.plan.mid)
        axes=[]
        margin=max(self.options['backoff_mm'],self.controller.options['position_margin_degrees']/360*self.options['mm_per_rev'])
        if any(u-l <= 2*margin for l,u in zip(self.plan.lower,self.plan.upper)):
            raise ControlError('实测行程不足以留出两端保护余量，未生成软限位。')
        for i,axis in enumerate(self.selected()):
            device=axis['device']
            def native(mm):
                return (axis['origin']+round(mm/self.options['mm_per_rev']*axis['counts']*device.extension_sign)+2**31)%2**32-2**31
            factor=device.extension_sign*(1 if device.positive_direction else -1)
            bounds=sorted(((self.plan.lower[i]+margin-self.plan.mid[i])*360/self.options['mm_per_rev']*factor,
                           (self.plan.upper[i]-margin-self.plan.mid[i])*360/self.options['mm_per_rev']*factor))
            axes.append(dict(order=device.order,origin=native(self.plan.mid[i]),enabled=True,
                             center_position=axis['last'][2],
                             lower_degrees=bounds[0],upper_degrees=bounds[1],
                             up_mm=self.plan.upper[i]-self.plan.mid[i]-margin,
                             down_mm=self.plan.mid[i]-self.plan.lower[i]-margin,
                             mm_per_rev=self.options['mm_per_rev'],
                             lower_trip=native(self.plan.lower[i]),upper_trip=native(self.plan.upper[i])))
        self.report['auto_center']=dict(success=True,axes=axes,measurements_mm=dict(
            coarse_lower=self.plan.coarse_lower,coarse_upper=self.plan.coarse_upper,
            first_lower=self.plan.first_lower,first_upper=self.plan.first_upper,
            lower=self.plan.lower,upper=self.plan.upper))
        self.report.update(success=True,stopped=True)

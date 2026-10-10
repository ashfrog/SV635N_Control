"""Tk debugging client. All motor operations go through the UDP SDK."""
from concurrent.futures import ThreadPoolExecutor
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from .client import MotorClient


class DebugWindow:
    def __init__(self, root, host, port, auth_key='', exit_callback=None):
        self.root, self.exit_callback = root, exit_callback or root.quit
        self.stop_callback = self.stop
        self.client = MotorClient(host, port, auth_key)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='Debug-UDP')
        self.motion_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='Debug-Motion')
        self.stop_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='UDP-Stop')
        self.stop_pending = False
        self.events = queue.Queue()
        self.pending = False
        self.motion_pending = False
        self.motion_run = None
        self.auto_start = None
        self.start_deadline = 0
        self.start_cancel = threading.Event()
        self.enable_pending = False
        self.latest_target = None
        self.latest_profile = None
        self.profile_run = None
        self.syncing_profile = False
        self.target_values = {}
        self.target_run = None
        self.target_editor = None
        self.target_editor_order = None
        self.slider_orders = [None]
        self.syncing = False
        self.state, self.selected = {}, set()
        self.last_query = 0
        self.last_feedback = 0
        self.last_render = 0
        self.rendered_serial = -1
        self.adapter_names = []
        self.adapter, self.rpm, self.acceleration = tk.StringVar(), tk.StringVar(value='60'), tk.StringVar(value='120')
        self.center, self.span = tk.StringVar(value='0'), tk.StringVar(value='360')
        self.position = tk.StringVar(value='共同目标偏移 0°')
        self.slider_axis = tk.StringVar(value='全部使能轴（共同目标）')
        self.status = tk.StringVar(value='正在连接后台…')
        root.title('SV635N · UDP 电机控制客户端')
        root.geometry('1280x860')
        root.minsize(1180, 820)
        root.protocol('WM_DELETE_WINDOW', self.exit_callback)
        root.bind('<Escape>', lambda _: self.stop_callback())
        style = ttk.Style(root)
        style.theme_use('clam')
        style.configure('.', font=('Microsoft YaHei UI', 10))
        style.configure('Treeview', rowheight=32)
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='UDP 电机控制', font=('Microsoft YaHei UI', 20, 'bold')).pack(anchor='w')
        ttk.Label(frame, text=f'后台地址 {host}:{port} · 所有操作通过 UDP。关闭界面停止本界面开启的运行，后台继续运行。').pack(anchor='w', pady=(5, 14))
        actions = ttk.Frame(frame)
        actions.pack(fill='x')
        self.claim_button = ttk.Button(actions, text='获取调试控制权', command=self.claim)
        self.claim_button.pack(side='left')
        self.release_button = ttk.Button(actions, text='释放给其它程序', command=lambda: self.task(self.client.release))
        self.release_button.pack(side='left', padx=8)
        self.stop_button = ttk.Button(actions, text='关闭使能 / Esc', command=self.stop)
        self.stop_button.pack(side='right')
        network = ttk.Frame(frame)
        network.pack(fill='x', pady=12)
        ttk.Label(network, text='PCIe 控制卡').pack(side='left')
        self.adapter_box = ttk.Combobox(network, textvariable=self.adapter, state='readonly', width=65)
        self.adapter_box.pack(side='left', padx=8, fill='x', expand=True)
        self.adapters_button = ttk.Button(network, text='刷新控制卡', command=self.adapters)
        self.adapters_button.pack(side='left')
        self.scan_button = ttk.Button(network, text='重新扫描并恢复控制', command=self.claim)
        self.scan_button.pack(side='left', padx=(8, 0))
        self.tree = ttk.Treeview(frame, columns=('checked','order','id','name','position','travel','target','state',
                                                'limit','inputs','retraction'), show='headings', height=5)
        for key, label, width in (('checked','选择',50),('order','链路位置',70),('id','ID',50),('name','电机 / 编号',190),
                                  ('position','位置计数',95),('travel','偏移 °',80),('target','目标偏移 °',100),('state','反馈状态',80),
                                  ('limit','推出端限位',145),('inputs','DI1 / DI2 · 正/负限位',170),
                                  ('retraction','缩回端限位',145)):
            self.tree.heading(key, text=label)
            self.tree.column(key, width=width, anchor='center')
        self.tree.pack(fill='x')
        self.tree.tag_configure('limit_triggered', background='#ffcccc', foreground='#8b0000')
        self.tree.tag_configure('limit_unknown', background='#fff1cc', foreground='#634600')
        self.tree.bind('<Button-1>', self.toggle_motor)
        self.tree.bind('<Double-1>', self.edit_target)
        ttk.Label(frame, text='两端限位分别监视；触发后仅允许反向离开，两端同时触发时禁止运动。').pack(anchor='w', pady=(5,0))
        params = ttk.Frame(frame)
        params.pack(fill='x', pady=8)
        params.columnconfigure(2, weight=1)
        ttk.Label(params,text='最高速度 rpm').grid(row=0,column=0,sticky='w')
        self.rpm_entry = ttk.Entry(params,textvariable=self.rpm,width=12)
        self.rpm_entry.grid(row=0,column=1,padx=8,pady=3)
        self.rpm_slider = ttk.Scale(params,from_=1,to=3000,
                                    command=lambda value:self.drag_profile('rpm',value))
        self.rpm_slider.grid(row=0,column=2,sticky='ew',padx=(0,12))
        ttk.Label(params,text='加减速度 rpm/s').grid(row=1,column=0,sticky='w')
        self.acceleration_entry = ttk.Entry(params,textvariable=self.acceleration,width=12)
        self.acceleration_entry.grid(row=1,column=1,padx=8,pady=3)
        self.acceleration_slider = ttk.Scale(params,from_=1,to=3000,
                                             command=lambda value:self.drag_profile('acceleration',value))
        self.acceleration_slider.grid(row=1,column=2,sticky='ew',padx=(0,12))
        self.profile_button = ttk.Button(params,text='应用速度 / 加减速度',command=self.apply_profile)
        self.profile_button.grid(row=0,column=3,rowspan=2,sticky='ew')
        for entry in (self.rpm_entry,self.acceleration_entry):
            entry.bind('<Return>',lambda _:self.apply_profile())
            entry.bind('<FocusOut>',lambda _:self.sync_profile_sliders())
        self.sync_profile_sliders()
        ttk.Label(frame,text='获取控制权后全部电机保持连续使能；限位仅停止运动。停止后可重新获取控制，关闭使能 / Esc 可取消。').pack(anchor='w')
        target_controls = ttk.Frame(frame)
        target_controls.pack(fill='x',pady=(12,0))
        ttk.Label(target_controls,text='滑块控制').pack(side='left')
        self.axis_box = ttk.Combobox(target_controls,textvariable=self.slider_axis,state='disabled',
                                     values=['全部使能轴（共同目标）'],width=32)
        self.axis_box.current(0)
        self.axis_box.pack(side='left',padx=8)
        self.axis_box.bind('<<ComboboxSelected>>',lambda _:self.sync_slider())
        self.targets_button = ttk.Button(target_controls,text='发送各轴目标',command=self.send_targets)
        self.targets_button.pack(side='right')
        ttk.Label(frame,textvariable=self.position,font=('Microsoft YaHei UI',13,'bold')).pack(anchor='w',pady=(16,4))
        self.slider = ttk.Scale(frame,from_=-360,to=360,command=self.drag)
        self.slider.pack(fill='x',pady=8)
        ranges = ttk.Frame(frame)
        ranges.pack(fill='x')
        ttk.Label(ranges,text='显示中心 °').pack(side='left')
        ttk.Entry(ranges,textvariable=self.center,width=10).pack(side='left',padx=8)
        ttk.Label(ranges,text='显示跨度 ±°').pack(side='left')
        ttk.Entry(ranges,textvariable=self.span,width=10).pack(side='left',padx=8)
        ttk.Button(ranges,text='设置显示范围',command=self.set_range).pack(side='left')
        self.zero_button=ttk.Button(ranges,text='所选轴回到使能起点',command=lambda:self.queue_target(0))
        self.zero_button.pack(side='right')
        ttk.Label(frame,text='双击“目标偏移 °”分别输入每轴目标，Enter 发送；滑块可选择单轴或全部使能轴，单轴操作保持其它轴目标。').pack(anchor='w',pady=10)
        ttk.Label(frame,textvariable=self.status,wraplength=980).pack(anchor='w',pady=8)
        self.log = tk.Text(frame,height=4,state='disabled',wrap='word')
        self.log.pack(fill='both',expand=True)
        footer = ttk.Frame(frame)
        footer.pack(fill='x',pady=(10,0))
        self.close_button=ttk.Button(footer,text='关闭控制界面',command=self.exit_callback)
        self.close_button.pack(side='right')
        self.poll_handle = root.after(5,self.poll)

    def show(self):
        self.root.deiconify()
        self.root.lift()

    def hide(self):
        self.root.withdraw()

    def stop(self):
        self.auto_start = None
        self.start_cancel.set()
        self.latest_target = None
        self.latest_profile = None
        if self.stop_pending or not self.client.run_id:
            return
        self.stop_pending = True
        def worker():
            try:
                self.events.put(('result',self.client.disable(),'stop'))
            except Exception as exc:
                self.events.put(('error',str(exc),'stop'))
        # Stop is independent of ordinary requests and target ACK waits.
        self.stop_executor.submit(worker)

    def task(self, function, label=None, *, motion=False):
        busy='motion_pending' if motion else 'pending'
        if getattr(self,busy):
            return
        setattr(self,busy,True)
        def worker():
            try:
                self.events.put(('result',function(),label))
            except Exception as exc:
                self.events.put(('error',str(exc),label))
        (self.motion_executor if motion else self.executor).submit(worker)

    def flush_motion(self):
        if (self.motion_pending or not self.client.run_id or
                self.state.get('phase')!='enabled' or self.client.run_id!=self.state.get('run_id')):
            return
        if self.latest_profile is not None:
            run_id,rpm,acceleration=self.latest_profile
            self.latest_profile=None
            self.motion_run=run_id
            self.task(lambda:self.client.set_profile(rpm,acceleration,expected_run_id=run_id,
                                                     retry_timeout=.03),'profile',motion=True)
        elif self.latest_target is not None:
            targets,self.latest_target=self.latest_target,None
            run_id=self.client.run_id
            self.motion_run=run_id
            self.task(lambda:self.client.target(targets,expected_run_id=run_id,
                                                retry_timeout=.03),'target',motion=True)

    def claim(self):
        if (self.auto_start or self.enable_pending or self.stop_pending or
                self.client.session and self.state.get('phase') not in ('idle','fault')):
            return
        self.start_cancel = threading.Event()
        self.start_deadline = time.monotonic()+15
        self.auto_start = 'wait_idle' if self.client.session else 'claim'

    def advance_start(self):
        """Advance only after fresh ACK/state feedback; never retry an enable."""
        if not self.auto_start or self.pending:
            return
        if time.monotonic()>self.start_deadline:
            self.auto_start=None
            self.start_cancel.set()
            self.events.put(('notice','自动扫描/使能等待超时，已取消自动流程。','auto_start'))
            return
        if self.auto_start=='claim':
            self.auto_start='claiming'
            self.task(lambda:self.client.hello(claim=True),'claim')
        elif self.auto_start=='wait_idle':
            phase=self.state.get('phase')
            if phase in ('idle','fault'):
                self.auto_start='scanning'
                self.scan(automatic=True)
            elif phase in ('enabling','enabled','stopping'):
                self.auto_start=None
                self.events.put(('notice','后台已有运行或正在停止，自动扫描/使能已取消。','auto_start'))
        elif self.auto_start=='scanning':
            phase=self.state.get('phase')
            if phase=='fault':
                self.auto_start=None
            elif phase=='idle':
                self.selected={d['order'] for d in self.state.get('devices',[])}
                if not self.selected:
                    self.auto_start=None
                    self.events.put(('notice','未扫描到电机，未开启使能。','auto_start'))
                    return
                self.auto_start='enabling'
                self.enable(automatic=True)

    def adapters(self):
        self.task(lambda:self.client.request('adapters'),'adapters')

    def scan(self, automatic=False):
        if self.pending:
            return
        if not automatic:
            self.auto_start=None
        index = self.adapter_box.current()
        adapter = self.adapter_names[index] if 0 <= index < len(self.adapter_names) else self.state.get('adapter')
        self.selected.clear()
        self.finish_target_edit()
        self.target_values.clear()
        self.task(lambda:self.client.request('scan',adapter=adapter),'auto_scan' if automatic else 'scan')

    def toggle_motor(self,event):
        if self.auto_start or self.state.get('phase') in ('scanning','enabling','enabled','stopping'):
            return 'break'
        row = self.tree.identify_row(event.y)
        if row:
            order = int(row)
            self.selected.symmetric_difference_update({order})
            self.render(self.state)
        return 'break'

    def enable(self, automatic=False):
        if self.pending:
            return
        if not self.selected:
            messagebox.showerror('电机选择','请先扫描并选择电机。',parent=self.root)
            return
        try:
            rpm, acceleration = float(self.rpm.get()),float(self.acceleration.get())
            if not all(math.isfinite(v) and v>0 for v in (rpm,acceleration)):
                raise ValueError()
        except ValueError:
            self.auto_start=None
            self.events.put(('notice','速度和加减速度必须为大于 0 的有限数值。','enable'))
            return
        if not automatic:
            self.auto_start=None
            self.start_cancel=threading.Event()
        cancel=self.start_cancel
        orders=sorted(self.selected)
        self.latest_target = None
        self.latest_profile = None
        self.target_values = {order:0. for order in self.selected}
        self.enable_pending=True
        def worker():
            if cancel.is_set():
                raise RuntimeError('使能已取消。')
            reply=self.client.enable(orders,rpm,acceleration)
            # Stop/close can arrive while the enable ACK is in flight. Cancel
            # that exact run even if no run_id was available when Stop was pressed.
            with self.client.lock:
                cancel_run=cancel.is_set() and self.client.run_id==reply['run_id']
                if cancel_run:
                    self.client.run_id=None
            if cancel_run:
                self.client.request('disable',run_id=reply['run_id'])
            return reply
        self.task(worker,'auto_enable' if automatic else 'enable')

    def sync_profile_sliders(self):
        self.syncing_profile = True
        try:
            for variable, slider in ((self.rpm,self.rpm_slider),
                                     (self.acceleration,self.acceleration_slider)):
                try:
                    value=float(variable.get())
                    if math.isfinite(value) and value > 0:
                        slider.set(max(float(slider.cget('from')),min(float(slider.cget('to')),value)))
                except ValueError:
                    pass
        finally:
            self.syncing_profile = False

    def drag_profile(self, name, value):
        if self.syncing_profile:
            return
        getattr(self,name).set(f'{float(value):.1f}')
        self.apply_profile()

    def apply_profile(self):
        try:
            rpm, acceleration = float(self.rpm.get()),float(self.acceleration.get())
            if not all(math.isfinite(v) and v > 0 for v in (rpm,acceleration)):
                raise ValueError()
        except ValueError:
            self.status.set('速度和加减速度必须为大于 0 的有限数值。')
            return
        self.sync_profile_sliders()
        if (self.client.session and self.client.run_id and
                self.state.get('phase')=='enabled' and self.client.run_id==self.state.get('run_id')):
            self.latest_profile=(self.client.run_id,rpm,acceleration)

    def set_range(self):
        import math
        try:
            center,span = float(self.center.get()),float(self.span.get())
            low,high = center-span,center+span
            if span<=0 or low>=high or not all(math.isfinite(x) for x in (low,high)):
                raise ValueError()
            self.syncing=True
            try:
                self.slider.configure(from_=low,to=high)
            finally:
                self.syncing=False
        except ValueError:
            messagebox.showerror('显示范围','请输入有限的中心和正的跨度。',parent=self.root)

    def drag(self,value):
        if not self.syncing:
            self.queue_target(float(value))

    def queue_target(self,value):
        if self.state.get('phase') == 'enabled' and self.client.run_id == self.state.get('run_id'):
            index=self.axis_box.current()
            order=self.slider_orders[index] if 0 <= index < len(self.slider_orders) else None
            self.queue_axis_target(order,value)

    def queue_axis_target(self,order,value):
        orders=self.state.get('orders',[])
        if (self.state.get('phase')!='enabled' or not self.client.session
                or not self.client.run_id or self.client.run_id!=self.state.get('run_id')
                or order is not None and order not in orders):
            return
        if not math.isfinite(value):
            raise ValueError('目标必须为有限角度数值。')
        values=self.current_targets()
        for axis in orders if order is None else [order]:
            values[axis]=value
        self.target_values.update(values)
        self.latest_target=[values[axis] for axis in orders]
        for axis in orders:
            if self.tree.exists(str(axis)):
                self.tree.set(str(axis),'target',f'{values[axis]:g}')
        label='共同目标偏移' if order is None else f'电机 {order} 目标偏移'
        self.position.set(f'{label} {value:.2f}°')

    def current_targets(self):
        orders=self.state.get('orders',[])
        reported=dict(zip(orders,self.state.get('targets_deg',[])))
        return {order:self.target_values.get(order,reported.get(order,0.)) for order in orders}

    def sync_slider(self):
        index=self.axis_box.current()
        order=self.slider_orders[index] if 0 <= index < len(self.slider_orders) else None
        values=self.current_targets()
        value=values.get(order,next(iter(values.values()),0.))
        self.syncing=True
        try:
            low,high=float(self.slider.cget('from')),float(self.slider.cget('to'))
            self.slider.set(max(low,min(high,value)))
        finally:
            self.syncing=False
        label='共同目标偏移' if order is None else f'电机 {order} 目标偏移'
        self.position.set(f'{label} {value:.2f}°' if order is not None or len(set(values.values()))<=1
                          else '各轴目标不同；选择单轴调节或拖动统一目标')

    def edit_target(self,event):
        row=self.tree.identify_row(event.y)
        column=self.tree.identify_column(event.x)
        if not row or column!=f"#{list(self.tree['columns']).index('target')+1}":
            return
        order=int(row)
        if (self.state.get('phase')!='enabled' or self.client.run_id!=self.state.get('run_id')
                or order not in self.state.get('orders',[])):
            return 'break'
        self.finish_target_edit()
        box=self.tree.bbox(row,'target')
        if not box:
            return 'break'
        editor=ttk.Entry(self.tree,justify='center')
        editor.insert(0,self.tree.set(row,'target'))
        editor.select_range(0,'end')
        editor.place(x=box[0],y=box[1],width=box[2],height=box[3])
        self.target_editor=editor
        self.target_editor_order=order
        editor.bind('<Return>',lambda _:self.commit_target_edit())
        editor.bind('<Escape>',lambda _:self.finish_target_edit())
        editor.focus_set()
        return 'break'

    def commit_target_edit(self):
        if self.target_editor is None:
            return 'break'
        try:
            value=float(self.target_editor.get())
            if not math.isfinite(value):
                raise ValueError()
        except ValueError:
            messagebox.showerror('各轴目标','请输入有限的角度数值。',parent=self.root)
            return 'break'
        order=self.target_editor_order
        self.finish_target_edit()
        self.queue_axis_target(order,value)
        self.sync_slider()
        return 'break'

    def finish_target_edit(self):
        if self.target_editor is not None:
            self.target_editor.destroy()
            self.target_editor=None
            self.target_editor_order=None

    def send_targets(self):
        if self.target_editor is not None:
            self.commit_target_edit()
            return
        values=self.current_targets()
        if values and self.state.get('phase')=='enabled' and self.client.run_id==self.state.get('run_id'):
            self.latest_target=[values[axis] for axis in self.state['orders']]

    def render(self,state,feedback_time=None):
        self.last_render=time.monotonic()
        self.last_feedback=feedback_time if feedback_time is not None else self.last_render
        self.state=state
        adapter=state.get('adapter')
        if adapter and adapter not in self.adapter_names:
            self.adapter_names=[adapter]
            label=f'ADLINK {adapter}' if adapter.startswith('PCIe-8332:') else adapter
            self.adapter_box.configure(values=[label])
            self.adapter_box.current(0)
        if state.get('run_id')!=self.target_run:
            self.target_run=state.get('run_id')
            self.target_values=dict(zip(state.get('orders',[]),state.get('targets_deg',[])))
            self.latest_target=None
            self.finish_target_edit()
        axes={a['order']:a for a in state.get('axes',[])}
        devices=state.get('devices',[])
        limits={a['order']:a for a in state.get('limits',[])}
        orders={d['order'] for d in devices}
        self.selected.intersection_update(orders)
        for row in self.tree.get_children():
            if int(row) not in orders:
                self.tree.delete(row)
        for d in devices:
            order=d['order']
            a=axes.get(order,{})
            sensor=limits.get(order,{})
            def endpoint_text(state_key, input_key, allowed):
                text={'triggered':f'● 已触发（只可{allowed}）','clear':'○ 未触发',
                      'unconfigured':'未配置传感器','unavailable':'反馈失效',
                      'no_sensor':'无传感器 / 未知'}.get(sensor.get(state_key),'等待反馈')
                if sensor.get(input_key):
                    text=sensor[input_key].upper()+' '+text
                return text
            limit_text=endpoint_text('state','input','缩回')
            retraction_text=endpoint_text('retraction_state','retraction_input','推出')
            if sensor.get('conflict'):
                limit_text=retraction_text='两端同时触发 / 故障'
            def bit(key):
                value=sensor.get(key) if sensor.get('valid') else None
                return '?' if value is None else '1' if value else '0'
            inputs=f"{bit('di1')} / {bit('di2')} · {bit('positive_limit')} / {bit('negative_limit')}"
            error=a.get('error_code',d.get('error_code',0))
            values=('☑' if order in self.selected else '☐',order,d['id'],f"{d['name']} / {d['motor_code']}",
                    a.get('position',d['position']),f"{a.get('travel_degrees') or 0:.3f}",
                    f"{self.target_values.get(order,0.):g}",
                    f"报警 0x{error:04X}" if error else '使能' if a.get('enabled') else '未使能',
                    limit_text,inputs,retraction_text)
            triggered=sensor.get('triggered') or sensor.get('retraction_triggered')
            known=sensor.get('state')=='clear' and sensor.get('retraction_state','no_sensor') in ('clear','no_sensor')
            tags=('limit_triggered',) if triggered else (() if known else ('limit_unknown',))
            if self.tree.exists(str(order)):
                row=self.tree.item(str(order))
                if tuple(str(v) for v in row['values'])!=tuple(str(v) for v in values) or tuple(row['tags'])!=tags:
                    self.tree.item(str(order),values=values,tags=tags)
            else:
                self.tree.insert('','end',iid=str(order),values=values,tags=tags)
        owner=bool(self.client.session)
        idle=not self.auto_start and not self.enable_pending and state.get('phase') not in ('scanning','enabling','enabled','stopping')
        self.claim_button.configure(text='重新获取调试控制' if owner else '获取调试控制权',
                                   state='normal' if idle and not self.stop_pending else 'disabled')
        for button in (self.release_button,self.adapters_button,self.scan_button):
            button.configure(state='normal' if owner and idle else 'disabled')
        self.adapter_box.configure(state='readonly' if owner and idle else 'disabled')
        self.stop_button.configure(state='normal' if self.auto_start or self.enable_pending or
                                   self.client.run_id and state.get('phase') in ('enabling','enabled') else 'disabled')
        controllable=owner and self.client.run_id==state.get('run_id') and state.get('phase')=='enabled'
        for widget in (self.rpm_entry,self.acceleration_entry,self.rpm_slider,self.acceleration_slider):
            widget.configure(state='normal' if owner and (idle or controllable) else 'disabled')
        self.profile_button.configure(state='normal' if controllable else 'disabled')
        if controllable and self.profile_run != state.get('run_id'):
            self.profile_run=state['run_id']
            profile=state.get('profile') or {}
            if profile:
                self.rpm.set(f"{profile['rpm']:g}")
                self.acceleration.set(f"{profile['acceleration_rpm_s']:g}")
                self.sync_profile_sliders()
        self.slider.configure(state='normal' if controllable else 'disabled')
        self.targets_button.configure(state='normal' if controllable else 'disabled')
        self.zero_button.configure(state='normal' if controllable else 'disabled')
        active_orders=state.get('orders',[]) if controllable else []
        if self.slider_orders!=[None]+active_orders:
            self.slider_orders=[None]+list(active_orders)
            self.axis_box.configure(values=['全部使能轴（共同目标）']+
                                     [f'电机 {order}（链路位置 {order}）' for order in active_orders])
            self.axis_box.current(0)
            self.sync_slider()
        self.axis_box.configure(state='readonly' if controllable else 'disabled')
        if not controllable:
            self.latest_target=None
            self.latest_profile=None
            self.finish_target_edit()
        inputs = state.get('input_configuration', {})
        input_text = ' · '.join(label for key, label in
            (('limit_inputs_connected', '通用限位未接入（端点传感器按逐轴配置）'), ('emg_input_connected', '急停未接入（调试）'))
            if inputs.get(key) is False)
        self.status.set(f"{state.get('phase','?')} · {state.get('message','')} · " +
                        (input_text + ' · ' if input_text else '') +
                        ('调试面板持有控制权' if owner else '只读监视；外部程序可获取控制权'))
        if controllable:
            profile=state.get('applied_profile') or {}
            if profile:
                self.status.set(self.status.get()+f" · 已应用 {profile['rpm']:g} rpm / {profile['acceleration_rpm_s']:g} rpm/s（拖动即更新，数值 Enter 应用）")

    def poll(self):
        try:
            while True:
                kind,result,label=self.events.get_nowait()
                if label=='stop':
                    self.stop_pending=False
                elif label in ('target','profile'):
                    self.motion_pending=False
                elif kind!='notice':
                    self.pending=False
                if label in ('enable','auto_enable'):
                    self.enable_pending=False
                if label in ('claim','auto_scan','auto_enable'):
                    if kind=='error':
                        self.auto_start=None
                        self.start_cancel.set()
                    else:
                        self.render(self.client.state)
                        if label=='claim' and self.auto_start=='claiming':
                            self.auto_start='wait_idle'
                        elif label=='auto_enable':
                            self.auto_start=None
                if kind in ('error','notice'):
                    if label=='target' and self.motion_run==self.client.run_id and self.latest_target is None:
                        reported=self.client.state
                        self.target_values=dict(zip(reported.get('orders',[]),reported.get('targets_deg',[])))
                        self.latest_target=None
                        self.sync_slider()
                    if label=='profile' and self.motion_run==self.client.run_id and self.latest_profile is None:
                        profile=self.client.state.get('profile') or {}
                        if profile:
                            self.rpm.set(f"{profile['rpm']:g}")
                            self.acceleration.set(f"{profile['acceleration_rpm_s']:g}")
                            self.sync_profile_sliders()
                    self.status.set(result)
                    self.log.configure(state='normal')
                    self.log.insert('end',result+'\n')
                    # Bound the UI journal during long runs.
                    if int(self.log.index('end-1c').split('.')[0])>200:
                        self.log.delete('1.0','100.0')
                    self.log.configure(state='disabled')
                elif label=='adapters':
                    self.adapter_names=[a['name'] for a in result['adapters']]
                    self.adapter_box.configure(values=[a['description']+' · '+a['name'] for a in result['adapters']])
                    if self.adapter_names:
                        current=self.state.get('adapter')
                        self.adapter_box.current(self.adapter_names.index(current) if current in self.adapter_names else 0)
                elif label=='status':
                    self.render(self.client.state,feedback_time=self.client.state_received)
        except queue.Empty:
            pass
        # Receiver-thread feedback refreshes the display without a blocking RPC.
        with self.client.lock:
            state,serial,received=self.client.state,self.client.state_serial,self.client.state_received
        if received and serial>self.rendered_serial and time.monotonic()-self.last_render>=.025:
            self.render(state,feedback_time=received)
            self.rendered_serial=serial
        if self.last_feedback and time.monotonic()-self.last_feedback > 1:
            for row in self.tree.get_children():
                self.tree.set(row,'limit','反馈失联')
                self.tree.set(row,'retraction','反馈失联')
                self.tree.set(row,'inputs','? / ? · ? / ?')
                self.tree.item(row,tags=('limit_unknown',))
        self.advance_start()
        self.flush_motion()
        if not self.pending:
            if time.monotonic()-self.last_query >= (1. if self.client.session else .2):
                # hello refreshes an idle lease, but never the motor heartbeat.
                self.last_query=time.monotonic()
                self.task(lambda:self.client.hello(claim=bool(self.client.session)),'status')
        self.poll_handle=self.root.after(5,self.poll)

    def close(self):
        self.stop()
        self.finish_target_edit()
        self.root.after_cancel(self.poll_handle)
        self.executor.shutdown(wait=True)
        self.motion_executor.shutdown(wait=True)
        self.stop_executor.shutdown(wait=True)
        self.client.close()
        # Tk variables must be finalized on this UI thread. Destroyed windows
        # may otherwise be collected by a network worker during a later run.
        for name in ('adapter','rpm','acceleration','center','span','position','slider_axis','status'):
            setattr(self,name,None)

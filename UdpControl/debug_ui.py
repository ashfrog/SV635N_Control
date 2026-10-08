"""Tk debugging client. All motor operations go through the UDP SDK."""
from concurrent.futures import ThreadPoolExecutor
import queue
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
        self.stop_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='UDP-Stop')
        self.stop_pending = False
        self.events = queue.Queue()
        self.pending = False
        self.latest_target = None
        self.syncing = False
        self.state, self.selected = {}, set()
        self.last_query = 0
        self.adapter_names = []
        self.adapter, self.rpm, self.acceleration = tk.StringVar(), tk.StringVar(value='60'), tk.StringVar(value='120')
        self.center, self.span = tk.StringVar(value='0'), tk.StringVar(value='360')
        self.position = tk.StringVar(value='共同目标偏移 0°')
        self.status = tk.StringVar(value='正在连接后台…')
        self.ready = tk.BooleanVar(value=False)
        root.title('SV635N · UDP 电机控制客户端')
        root.geometry('1050x720')
        root.minsize(920, 660)
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
        ttk.Label(network, text='EtherCAT 网卡').pack(side='left')
        self.adapter_box = ttk.Combobox(network, textvariable=self.adapter, state='readonly', width=65)
        self.adapter_box.pack(side='left', padx=8, fill='x', expand=True)
        self.adapters_button = ttk.Button(network, text='刷新网卡', command=self.adapters)
        self.adapters_button.pack(side='left')
        self.scan_button = ttk.Button(network, text='扫描电机', command=self.scan)
        self.scan_button.pack(side='left', padx=(8, 0))
        self.tree = ttk.Treeview(frame, columns=('checked','order','id','name','position','travel','state'), show='headings', height=5)
        for key, label, width in (('checked','选择',55),('order','链路位置',80),('id','ID',60),('name','电机 / 编号',220),
                                  ('position','位置计数',130),('travel','偏移 °',130),('state','反馈状态',170)):
            self.tree.heading(key, text=label)
            self.tree.column(key, width=width, anchor='center')
        self.tree.pack(fill='x')
        self.tree.bind('<Button-1>', self.toggle_motor)
        params = ttk.Frame(frame)
        params.pack(fill='x', pady=12)
        ttk.Label(params,text='最高速度 rpm').pack(side='left')
        self.rpm_entry = ttk.Entry(params,textvariable=self.rpm,width=12)
        self.rpm_entry.pack(side='left',padx=8)
        ttk.Label(params,text='加减速度 rpm/s').pack(side='left')
        self.acceleration_entry = ttk.Entry(params,textvariable=self.acceleration,width=12)
        self.acceleration_entry.pack(side='left',padx=8)
        self.enable_button = ttk.Button(params,text='开启连续使能',command=self.enable)
        self.enable_button.pack(side='right')
        self.ready_box = ttk.Checkbutton(frame, text='已确认电机固定、运行条件和可随时断电', variable=self.ready)
        self.ready_box.pack(anchor='w')
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
        ttk.Button(ranges,text='回到使能起点',command=lambda:self.queue_target(0)).pack(side='right')
        ttk.Label(frame,text='滑块同时控制所有勾选电机；目标为各轴本次使能起点的绝对偏移。显示范围不是累计限位。').pack(anchor='w',pady=10)
        ttk.Label(frame,textvariable=self.status,wraplength=980).pack(anchor='w',pady=8)
        self.log = tk.Text(frame,height=4,state='disabled',wrap='word')
        self.log.pack(fill='both',expand=True)
        footer = ttk.Frame(frame)
        footer.pack(fill='x',pady=(10,0))
        ttk.Button(footer,text='关闭控制界面',command=self.exit_callback).pack(side='right')
        self.poll_handle = root.after(20,self.poll)

    def show(self):
        self.root.deiconify()
        self.root.lift()

    def hide(self):
        self.root.withdraw()

    def stop(self):
        self.latest_target = None
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

    def task(self, function, label=None):
        if self.pending:
            return
        self.pending = True
        def worker():
            try:
                self.events.put(('result',function(),label))
            except Exception as exc:
                self.events.put(('error',str(exc),label))
        self.executor.submit(worker)

    def claim(self):
        self.task(lambda:self.client.hello(claim=True),'claim')

    def adapters(self):
        self.task(lambda:self.client.request('adapters'),'adapters')

    def scan(self):
        index = self.adapter_box.current()
        adapter = self.adapter_names[index] if 0 <= index < len(self.adapter_names) else self.state.get('adapter')
        self.selected.clear()
        self.task(lambda:self.client.request('scan',adapter=adapter),'scan')

    def toggle_motor(self,event):
        if self.state.get('phase') in ('scanning','enabling','enabled','stopping'):
            return 'break'
        row = self.tree.identify_row(event.y)
        if row:
            order = int(row)
            self.selected.symmetric_difference_update({order})
            self.render(self.state)
        return 'break'

    def enable(self):
        if not self.ready.get() or not self.selected:
            messagebox.showerror('运行条件','请勾选电机并确认运行条件。',parent=self.root)
            return
        try:
            rpm, acceleration = float(self.rpm.get()),float(self.acceleration.get())
        except ValueError:
            messagebox.showerror('参数','请输入数值。',parent=self.root)
            return
        self.latest_target = None
        self.task(lambda:self.client.enable(sorted(self.selected),rpm,acceleration),'enable')

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
            self.latest_target = [value]*len(self.state['orders'])
            self.position.set(f'共同目标偏移 {value:.2f}°')

    def render(self,state):
        self.state=state
        axes={a['order']:a for a in state.get('axes',[])}
        devices=state.get('devices',[])
        orders={d['order'] for d in devices}
        self.selected.intersection_update(orders)
        for row in self.tree.get_children():
            if int(row) not in orders:
                self.tree.delete(row)
        for d in devices:
            order=d['order']
            a=axes.get(order,{})
            values=('☑' if order in self.selected else '☐',order,d['id'],f"{d['name']} / {d['motor_code']}",
                    a.get('position',d['position']),f"{a.get('travel_degrees') or 0:.3f}",
                    f"报警 {a['error_code']}" if a.get('error_code') else '使能' if a.get('enabled') else '未使能')
            if self.tree.exists(str(order)):
                self.tree.item(str(order),values=values)
            else:
                self.tree.insert('','end',iid=str(order),values=values)
        owner=bool(self.client.session)
        idle=state.get('phase') not in ('scanning','enabling','enabled','stopping')
        self.claim_button.configure(state='disabled' if owner else 'normal')
        for button in (self.release_button,self.adapters_button,self.scan_button,self.rpm_entry,self.acceleration_entry,self.ready_box):
            button.configure(state='normal' if owner and idle else 'disabled')
        self.adapter_box.configure(state='readonly' if owner and idle else 'disabled')
        self.enable_button.configure(state='normal' if owner and state.get('phase')=='idle' else 'disabled')
        self.stop_button.configure(state='normal' if self.client.run_id and state.get('phase') in ('enabling','enabled') else 'disabled')
        controllable=owner and self.client.run_id==state.get('run_id') and state.get('phase')=='enabled'
        self.slider.configure(state='normal' if controllable else 'disabled')
        if not controllable:
            self.latest_target=None
        self.status.set(f"{state.get('phase','?')} · {state.get('message','')} · " +
                        ('调试面板持有控制权' if owner else '只读监视；外部程序可获取控制权'))

    def poll(self):
        try:
            while True:
                kind,result,label=self.events.get_nowait()
                if label=='stop':
                    self.stop_pending=False
                else:
                    self.pending=False
                if kind=='error':
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
                    self.render(result['state'])
        except queue.Empty:
            pass
        if not self.pending:
            if self.latest_target is not None:
                targets,self.latest_target=self.latest_target,None
                self.task(lambda:self.client.target(targets),'target')
            elif time.monotonic()-self.last_query >= .2:
                # hello refreshes an idle lease, but never the motor heartbeat.
                self.last_query=time.monotonic()
                self.task(lambda:self.client.hello(claim=bool(self.client.session)),'status')
        self.poll_handle=self.root.after(20,self.poll)

    def close(self):
        self.root.after_cancel(self.poll_handle)
        self.executor.shutdown(wait=True)
        self.stop_executor.shutdown(wait=True)
        self.client.close()
        # Tk variables must be finalized on this UI thread. Destroyed windows
        # may otherwise be collected by a network worker during a later run.
        for name in ('adapter','rpm','acceleration','center','span','position','status','ready'):
            setattr(self,name,None)

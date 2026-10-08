"""Tk UI only: queues events and delegates hardware operations to core.py."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk

from core import DEFAULT_ADAPTER, EtherCATController, Move
from continuous_control import MotionQueue, run_continuous


def data_folder():
    base = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).parent
    try:
        path = base / 'logs'
        path.mkdir(exist_ok=True)
        probe = path / '.write_test'
        probe.write_text('ok', encoding='utf-8')
        probe.unlink()
        return base
    except OSError:
        base = Path(os.environ.get('LOCALAPPDATA', str(Path.home()))) / 'SV635N_Control'
        (base / 'logs').mkdir(parents=True, exist_ok=True)
        return base


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('SV635N 电机调试工具 v1.3 · 连续控制')
        self.geometry('1080x1000')
        self.minsize(1020, 940)
        self.configure(bg='#eef2f6')
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='EtherCAT')
        self.events = queue.Queue()
        self.stop_token = threading.Event()
        self.busy = self.closing = False
        self.devices = []
        self.selected = set()
        self.base = data_folder()
        self.config_path = self.base / 'settings.json'
        try:
            self.settings = json.loads(self.config_path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            self.settings = {}
        self.adapter_names = []
        self.adapter_var = tk.StringVar()
        self.angle = tk.StringVar(value='30')
        self.rpm = tk.StringVar(value='5')
        self.acceleration = tk.StringVar(value='10')
        self.direction = tk.StringVar(value='forward')
        self.encoder = tk.StringVar(value='A3 · 23位（逐轴核对）')
        self.ready = tk.BooleanVar(value=False)
        self.continuous_enabled = tk.BooleanVar(value=False)
        self.continuous_active = self.continuous_ready = False
        self.motion_queue = None
        self.continuous_limit = tk.StringVar(value='360')
        self.queue_text = tk.StringVar(value='连续使能未开启')
        self.target_values = {}
        self.target_editor = None
        self.status = tk.StringVar(value='请选择专用 EtherCAT 网卡，然后扫描电机。')
        self.selection_text = tk.StringVar(value='尚未扫描')
        self.summary = tk.StringVar()
        self._style()
        self._layout()
        for v in (self.angle, self.rpm, self.acceleration, self.direction):
            v.trace_add('write', self.update_summary)
        self.update_summary()
        self.refresh_adapters()
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.bind('<Escape>', lambda event: self.stop())
        self.poll_handle = self.after(80, self.poll)

    def _style(self):
        s = ttk.Style(self)
        s.theme_use('clam')
        s.configure('.', font=('Microsoft YaHei UI', 10), background='#eef2f6')
        s.configure('TFrame', background='#eef2f6')
        s.configure('Card.TFrame', background='white')
        s.configure('Card.TLabel', background='white', foreground='#172b4d')
        s.configure('Title.TLabel', font=('Microsoft YaHei UI', 21, 'bold'), foreground='#142b49')
        s.configure('Subtitle.TLabel', foreground='#5c6b7d')
        s.configure('TButton', padding=(12, 6))
        s.configure('Start.TButton', background='#1664c0', foreground='white', font=('Microsoft YaHei UI', 12, 'bold'))
        s.map('Start.TButton', background=[('disabled', '#c1cad5'), ('active', '#1357a6')])
        s.configure('Stop.TButton', background='#c53336', foreground='white', font=('Microsoft YaHei UI', 12, 'bold'))
        s.map('Stop.TButton', background=[('disabled', '#c1cad5'), ('active', '#aa2427')])
        s.configure('Treeview', rowheight=35, background='white', fieldbackground='white', font=('Microsoft YaHei UI', 10))
        s.configure('Treeview.Heading', font=('Microsoft YaHei UI', 10, 'bold'), padding=8)

    def _layout(self):
        root = ttk.Frame(self, padding=20)
        root.pack(fill='both', expand=True)
        ttk.Label(root, text='SV635N 电机调试', style='Title.TLabel').pack(anchor='w')
        ttk.Label(root, text='单次调试 · 手动使能开关 · 连续执行单轴 / 多轴指令', style='Subtitle.TLabel').pack(anchor='w', pady=(4, 12))
        network = ttk.Frame(root)
        network.pack(fill='x', pady=(0, 12))
        ttk.Label(network, text='EtherCAT 网卡').pack(side='left', padx=(0, 10))
        self.adapter_box = ttk.Combobox(network, textvariable=self.adapter_var, state='readonly', width=65)
        self.adapter_box.pack(side='left', fill='x', expand=True)
        self.adapter_box.bind('<<ComboboxSelected>>', self.adapter_changed)
        self.refresh_button = ttk.Button(network, text='刷新网卡', command=self.refresh_adapters)
        self.refresh_button.pack(side='left', padx=8)
        self.scan_button = ttk.Button(network, text='扫描电机', command=self.scan)
        self.scan_button.pack(side='left')
        toolbar = ttk.Frame(root)
        toolbar.pack(fill='x')
        ttk.Label(toolbar, text='1  选择需要转动的电机', font=('Microsoft YaHei UI', 11, 'bold')).pack(side='left')
        self.all_button = ttk.Button(toolbar, text='全选', command=lambda: self.select_all(True))
        self.all_button.pack(side='right', padx=(8, 0))
        self.none_button = ttk.Button(toolbar, text='取消全选', command=lambda: self.select_all(False))
        self.none_button.pack(side='right')
        self.tree = ttk.Treeview(root, columns=('choose', 'order', 'id', 'name', 'gear', 'position', 'target', 'state'), show='headings', height=4, selectmode='none')
        for name, label, width in [('choose', '选择', 50), ('order', '链路位置', 75), ('id', 'ID / H0E.21', 95),
                                    ('name', 'H00.00 / 正指令方向', 180), ('gear', '齿轮比', 70), ('position', '位置计数', 125),
                                    ('target', '目标偏移 °', 105), ('state', '状态', 195)]:
            self.tree.heading(name, text=label)
            self.tree.column(name, width=width, anchor='center', stretch=name in ('position', 'state'))
        self.tree.pack(fill='x', pady=8)
        self.tree.bind('<Button-1>', self.toggle)
        self.tree.bind('<Double-1>', self.edit_target)
        self.tree.tag_configure('chosen', background='#e9f2ff')
        self.tree.tag_configure('fault', foreground='#bb2025')
        ttk.Label(root, textvariable=self.selection_text, style='Subtitle.TLabel').pack(anchor='w', pady=(0, 12))
        card = ttk.Frame(root, style='Card.TFrame', padding=12)
        card.pack(fill='x')
        ttk.Label(card, text='2  设置选中电机的运动参数', style='Card.TLabel', font=('Microsoft YaHei UI', 11, 'bold')).grid(row=0, column=0, columnspan=7, sticky='w', pady=(0, 8))
        ttk.Label(card, text='方向', style='Card.TLabel').grid(row=1, column=0, sticky='w')
        self.forward = ttk.Radiobutton(card, text='顺时针（+）', variable=self.direction, value='forward')
        self.forward.grid(row=1, column=1, sticky='w', padx=8)
        self.reverse = ttk.Radiobutton(card, text='逆时针（−）', variable=self.direction, value='reverse')
        self.reverse.grid(row=1, column=2, sticky='w', padx=8)
        ttk.Label(card, text='角度 °', style='Card.TLabel').grid(row=1, column=3, padx=(20, 8))
        self.angle_entry = ttk.Entry(card, textvariable=self.angle, width=10)
        self.angle_entry.grid(row=1, column=4)
        ttk.Label(card, text='最高速度 rpm', style='Card.TLabel').grid(row=1, column=5, padx=(20, 8))
        self.rpm_entry = ttk.Entry(card, textvariable=self.rpm, width=10)
        self.rpm_entry.grid(row=1, column=6)
        ttk.Label(card, text='加减速度', style='Card.TLabel').grid(row=2, column=0, sticky='w', pady=(8, 0))
        self.acceleration_entry = ttk.Entry(card, textvariable=self.acceleration, width=10)
        self.acceleration_entry.grid(row=2, column=1, sticky='w', padx=8, pady=(8, 0))
        ttk.Label(card, text='rpm/s（1～120，加速与减速相同）', style='Card.TLabel').grid(row=2, column=2, columnspan=5, sticky='w', pady=(8, 0))
        ttk.Label(card, text='电机编码器', style='Card.TLabel').grid(row=3, column=0, sticky='w', pady=(8, 0))
        self.encoder_box = ttk.Combobox(card, textvariable=self.encoder, values=('A3 · 23位（逐轴核对）',), state='readonly', width=23)
        self.encoder_box.grid(row=3, column=1, columnspan=2, sticky='w', padx=8, pady=(8, 0))
        ttk.Label(card, text='选中轴须 H00.00=14101；本版本仅支持已核对的 A3。', style='Card.TLabel').grid(row=3, column=3, columnspan=4, sticky='w', pady=(8, 0))
        ttk.Label(card, textvariable=self.summary, style='Card.TLabel', wraplength=960).grid(row=4, column=0, columnspan=7, sticky='w', pady=(10, 0))
        ttk.Label(root, text='角度 0.1～3600°（360° = 1圈） | 速度 0.1～60 rpm | 方向从轴端看，程序按各轴 H02.02 自动换算', style='Subtitle.TLabel').pack(anchor='w', pady=(9, 8))
        self.ready_box = ttk.Checkbutton(root, text='已确认：电机固定、轴端空载、手已离开，且可随时断电', variable=self.ready, command=self.update_buttons)
        self.ready_box.pack(anchor='w')
        actions = ttk.Frame(root)
        actions.pack(fill='x', pady=8)
        self.start_button = ttk.Button(actions, text='开始单次运行', style='Start.TButton', command=lambda: self.run_motion(False))
        self.start_button.pack(side='left', ipadx=15)
        self.prepare_button = ttk.Button(actions, text='验证通信（不转动）', command=lambda: self.run_motion(True))
        self.prepare_button.pack(side='left', padx=10)
        self.stop_button = ttk.Button(actions, text='停止全部 / Esc', style='Stop.TButton', command=self.stop)
        self.stop_button.pack(side='right', ipadx=15)
        continuous = ttk.Frame(root, style='Card.TFrame', padding=10)
        continuous.pack(fill='x', pady=(0, 10))
        self.continuous_switch = ttk.Checkbutton(continuous, text='连续使能（空闲保持使能）',
                                                variable=self.continuous_enabled, command=self.toggle_continuous)
        self.continuous_switch.grid(row=0, column=0, sticky='w', padx=(0, 12))
        ttk.Label(continuous, text='累计限位 ±°', style='Card.TLabel').grid(row=0, column=1, padx=6)
        self.limit_entry = ttk.Entry(continuous, textvariable=self.continuous_limit, width=8)
        self.limit_entry.grid(row=0, column=2)
        self.target_button = ttk.Button(continuous, text='发送各轴目标', command=self.send_targets)
        self.target_button.grid(row=0, column=3, padx=10)
        self.origin_button = ttk.Button(continuous, text='回到使能起点', command=lambda: self.send_targets(origin=True))
        self.origin_button.grid(row=0, column=4)
        ttk.Label(continuous, text='双击列表“目标偏移”填写各轴角度；连续使能后可追加相对指令。'
                  '指令按顺序执行，目标以本次使能起点为零。',
                  style='Card.TLabel', wraplength=960).grid(row=1, column=0, columnspan=5, sticky='w', pady=(8, 0))
        ttk.Label(continuous, textvariable=self.queue_text, style='Card.TLabel').grid(
            row=2, column=0, columnspan=5, sticky='w', pady=(4, 0))
        ttk.Label(root, textvariable=self.status, foreground='#1664a6', wraplength=960).pack(anchor='w', pady=(0, 6))
        self.log = tk.Text(root, height=3, wrap='word', font=('Microsoft YaHei UI', 9), bg='white', fg='#3a4d64', relief='flat', padx=10, pady=6, state='disabled')
        footer = ttk.Frame(root)
        footer.pack(side='bottom', fill='x', pady=(6, 0))
        ttk.Label(footer, text='Windows 调试用途；软件停止不能替代硬件急停。运行时请勿使用其他 EtherCAT 主站。', style='Subtitle.TLabel', font=('Microsoft YaHei UI', 9)).pack(side='left')
        ttk.Button(footer, text='打开记录', command=lambda: os.startfile(str(self.base / 'logs'))).pack(side='right')
        self.log.pack(fill='both', expand=True)

    def append(self, text):
        self.log.configure(state='normal')
        self.log.insert('end', datetime.now().strftime('%H:%M:%S') + '  ' + text + '\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def update_summary(self, *_):
        sign = '顺时针转' if self.direction.get() == 'forward' else '逆时针转'
        try:
            angle, rpm = float(self.angle.get()), float(self.rpm.get())
            if angle <= 0:
                raise ValueError('角度请输入正数。')
            move = Move(1, angle, rpm, acceleration_rpm_s=float(self.acceleration.get()))
            move.validate()
            ending = '连续模式：加入队列，完成后保持使能。' if self.continuous_active else '单次模式：完成后关闭使能。'
            self.summary.set(f'选中电机均{sign} {angle:g}°，预计运动 {move.estimated_seconds:.2f} 秒，'
                             f'峰值约 {move.peak_rpm:.1f} rpm；{ending}')
        except (ValueError, RuntimeError):
            self.summary.set('请输入范围内的角度、最高速度和加减速度。')

    def update_buttons(self):
        available = bool(self.devices and self.selected)
        for widget in (self.scan_button, self.refresh_button, self.all_button, self.none_button,
                       self.rpm_entry, self.acceleration_entry, self.ready_box):
            widget.configure(state='disabled' if self.busy else 'normal')
        for widget in (self.forward, self.reverse, self.angle_entry):
            widget.configure(state='normal' if not self.busy or self.continuous_ready else 'disabled')
        self.adapter_box.configure(state='disabled' if self.busy else 'readonly')
        self.encoder_box.configure(state='disabled' if self.busy else 'readonly')
        self.start_button.configure(text='追加相对指令' if self.continuous_active else '开始单次运行',
                                    state='normal' if self.continuous_ready or
                                    (not self.busy and available and self.ready.get()) else 'disabled')
        self.prepare_button.configure(state='normal' if not self.busy and available else 'disabled')
        self.stop_button.configure(state='normal' if self.busy else 'disabled')
        self.continuous_switch.configure(state='normal' if
            (self.continuous_active and self.motion_queue and not self.motion_queue.closed
             and not self.stop_token.is_set()) or
            (not self.busy and available and self.ready.get()) else 'disabled')
        self.limit_entry.configure(state='disabled' if self.busy else 'normal')
        for button in (self.target_button, self.origin_button):
            button.configure(state='normal' if self.continuous_ready else 'disabled')

    def refresh_adapters(self):
        if self.busy:
            return
        try:
            adapters = EtherCATController.adapters()
            self.adapter_names = [a[0] for a in adapters]
            labels = [f'{a[1]}  ·  {a[0].split("{")[-1].rstrip("}")[:8]}' for a in adapters]
            self.adapter_box.configure(values=labels)
            if adapters:
                preferred = self.settings.get('adapter', DEFAULT_ADAPTER)
                index = self.adapter_names.index(preferred) if preferred in self.adapter_names else 0
                self.adapter_box.current(index)
                self.status.set('网卡已加载。选择接驱动器的网卡后点击“扫描电机”。')
            else:
                self.status.set('未发现可用网卡。请安装 Npcap（WinPcap 兼容模式）并重新打开程序。')
        except Exception as exc:
            self.status.set(f'加载网卡失败：{exc}')
            self.append(str(exc))
        self.update_buttons()

    def adapter(self):
        index = self.adapter_box.current()
        if not 0 <= index < len(self.adapter_names):
            raise ValueError('请先选择网卡。')
        return self.adapter_names[index]

    def adapter_changed(self, *_):
        self.finish_target_edit()
        self.devices = []
        self.selected.clear()
        self.target_values.clear()
        self.tree.delete(*self.tree.get_children())
        self.ready.set(False)
        self.selection_text.set('网卡已改变，请重新扫描。')
        self.update_buttons()

    def submit(self, operation, function):
        if self.busy:
            return
        self.busy = True
        self.stop_token = threading.Event()
        self.update_buttons()
        self.status.set('正在扫描…' if operation == 'scan' else '正在准备…')
        def worker():
            try:
                result = function()
                self.events.put({'kind': 'done', 'operation': operation, 'result': result})
            except Exception as exc:
                self.events.put({'kind': 'failure', 'text': str(exc)})
        self.executor.submit(worker)

    def scan(self):
        if self.busy:
            return
        try:
            controller = EtherCATController(self.adapter(), self.base / 'logs')
            self.settings['adapter'] = self.adapter()
            self.config_path.write_text(json.dumps(self.settings, indent=2), encoding='utf-8')
        except Exception as exc:
            messagebox.showerror('无法扫描', str(exc), parent=self)
            return
        self.finish_target_edit()
        self.ready.set(False)
        self.devices = []
        self.selected.clear()
        self.target_values.clear()
        self.tree.delete(*self.tree.get_children())
        self.selection_text.set('扫描中；电机不会使能。')
        self.submit('scan', controller.scan)

    def toggle(self, event):
        if self.tree.identify_column(event.x) == '#7':
            return 'break'
        if self.busy:
            return 'break'
        row = self.tree.identify_row(event.y)
        if row:
            order = int(row)
            device = next(d for d in self.devices if d.order == order)
            if device.motor_code != 14101:
                messagebox.showerror('该轴参数需要核对', f'ID {device.alias}：H00.00={device.motor_code}，本版本要求与 A3 铭牌匹配的 14101。\n核对并修正驱动器参数、重新上电后再扫描；当前不能选中此轴。', parent=self)
                return 'break'
            if order in self.selected:
                self.selected.remove(order)
            else:
                self.selected.add(order)
            self.render_selection()
        return 'break'

    def select_all(self, yes):
        if self.busy:
            return
        self.selected = {d.order for d in self.devices if d.motor_code == 14101} if yes else set()
        self.render_selection()

    def render_selection(self):
        for d in self.devices:
            chosen = d.order in self.selected
            self.tree.set(str(d.order), 'choose', '☑' if chosen else '☐')
            self.tree.item(str(d.order), tags=('fault',) if d.error_code else ('chosen',) if chosen else ())
        labels = [f'ID {d.alias}（位置 {d.order}）' for d in self.devices if d.order in self.selected]
        self.selection_text.set('已选：' + '、'.join(labels) if labels else '点击任意电机行勾选；可多选同时运行。ID 0 表示自动分配，按链路位置识别。')
        self.update_buttons()

    def run_motion(self, dry_run):
        if self.busy:
            if self.continuous_ready and not dry_run:
                self.queue_relative()
            return
        try:
            if not dry_run and not self.ready.get():
                raise ValueError('请确认固定、空载及可断电条件。')
            angle = float(self.angle.get())
            if angle <= 0:
                raise ValueError('角度请输入正数，方向使用正转/反转选择。')
            degrees = angle * (1 if self.direction.get() == 'forward' else -1)
            bits = 23 if self.encoder.get().startswith('A3') else 26
            moves = [Move(order, degrees, float(self.rpm.get()), bits,
                          acceleration_rpm_s=float(self.acceleration.get())) for order in sorted(self.selected)]
            if not moves:
                raise ValueError('请勾选电机。')
            for p in moves:
                p.validate()
            controller = EtherCATController(self.adapter(), self.base / 'logs')
        except Exception as exc:
            messagebox.showerror('参数检查', str(exc), parent=self)
            return
        snapshot = list(self.devices)
        self.append(('验证通信，不使能。' if dry_run else '开始单次运动：') + self.selection_text.get() +
                    f'；{degrees:+g}° / 最高 {self.rpm.get()} rpm / 加减速 {self.acceleration.get()} rpm/s')
        # Event token is replaced by submit before the worker invokes this closure.
        self.submit('run', lambda: controller.execute(moves, snapshot, self.stop_token,
                                                     self.events.put, dry_run=dry_run))

    def toggle_continuous(self):
        if not self.continuous_enabled.get():
            if self.continuous_active:
                self.stop()
            return
        if self.busy:
            self.continuous_enabled.set(self.continuous_active and not self.stop_token.is_set())
            return
        try:
            if not self.ready.get() or not self.selected:
                raise ValueError('请确认运行条件并选中电机。')
            limit = float(self.continuous_limit.get())
            rpm, acceleration = float(self.rpm.get()), float(self.acceleration.get())
            orders = sorted(self.selected)
            commands = MotionQueue(orders, limit, rpm, acceleration)
            controller = EtherCATController(self.adapter(), self.base / 'logs')
        except Exception as exc:
            self.continuous_enabled.set(False)
            messagebox.showerror('连续控制参数', str(exc), parent=self)
            return
        self.finish_target_edit()
        snapshot = list(self.devices)
        self.motion_queue = commands
        self.continuous_active = True
        self.continuous_ready = False
        for order in orders:
            self.set_target(order, '0')
        self.queue_text.set('正在开启连续使能；不会自动运动')
        self.update_summary()
        self.append(f'开启连续使能：链路位置 {orders}；累计限位 ±{limit:g}°；'
                    f'{rpm:g} rpm / {acceleration:g} rpm/s。')
        self.submit('continuous', lambda: run_continuous(
            controller, snapshot, commands, self.stop_token, self.events.put))

    def set_target(self, order, value):
        self.target_values[order] = value
        if self.tree.exists(str(order)):
            self.tree.set(str(order), 'target', value)

    def edit_target(self, event):
        if self.tree.identify_column(event.x) != '#7' or (self.busy and not self.continuous_ready):
            return 'break'
        row = self.tree.identify_row(event.y)
        if not row:
            return 'break'
        self.finish_target_edit()
        box = self.tree.bbox(row, 'target')
        if not box:
            return 'break'
        editor = ttk.Entry(self.tree, justify='center')
        editor.insert(0, self.target_values.get(int(row), '0'))
        editor.place(x=box[0], y=box[1], width=box[2], height=box[3])
        self.target_editor = (int(row), editor)
        editor.focus_set()
        editor.select_range(0, 'end')
        editor.bind('<Return>', lambda _: self.finish_target_edit())
        editor.bind('<FocusOut>', lambda _: self.finish_target_edit())
        editor.bind('<Escape>', lambda _: self.finish_target_edit(cancel=True))
        return 'break'

    def finish_target_edit(self, cancel=False):
        if self.target_editor:
            order, editor = self.target_editor
            self.target_editor = None
            if not cancel:
                self.set_target(order, editor.get())
            editor.destroy()
        return 'break'

    def show_queued(self, command):
        for order, target in zip(self.motion_queue.orders, command.targets):
            self.set_target(order, f'{target:g}')
        self.queue_text.set(f'已加入指令 #{command.number}；待执行 {self.motion_queue.pending()} 条')
        self.append(f'指令 #{command.number} 已加入队列，目标偏移：{list(command.targets)}°。')

    def queue_relative(self):
        try:
            angle = float(self.angle.get())
            Move(1, angle, self.motion_queue.rpm,
                 acceleration_rpm_s=self.motion_queue.acceleration).validate()
            if angle <= 0:
                raise ValueError('角度请输入正数，方向使用正转/反转选择。')
            degrees = angle * (1 if self.direction.get() == 'forward' else -1)
            command = self.motion_queue.submit([degrees] * len(self.motion_queue.orders), relative=True)
            self.finish_target_edit(cancel=True)
            self.show_queued(command)
        except Exception as exc:
            messagebox.showerror('相对指令检查', str(exc), parent=self)

    def send_targets(self, origin=False):
        if not self.continuous_ready:
            return
        self.finish_target_edit()
        try:
            values = [0.0 if origin else float(self.target_values.get(n, '0'))
                      for n in self.motion_queue.orders]
            self.show_queued(self.motion_queue.submit(values))
        except Exception as exc:
            messagebox.showerror('目标检查', str(exc), parent=self)

    def stop(self):
        if self.busy:
            self.continuous_enabled.set(False)
            self.continuous_ready = False
            self.stop_token.set()
            if self.motion_queue:
                self.motion_queue.close()
            self.finish_target_edit(cancel=True)
            self.queue_text.set('正在停止并关闭使能；待执行指令已清空')
            self.update_buttons()
            self.status.set('已请求停止；正在减速并关闭全部使能，请勿强制结束程序。')
            self.append('停止请求已发送。')

    def poll(self):
        if self.poll_handle is not None:
            self.after_cancel(self.poll_handle)
            self.poll_handle = None
        try:
            while True:
                event = self.events.get_nowait()
                kind = event['kind']
                if kind == 'phase':
                    self.status.set(event['text'])
                    if self.continuous_active and self.motion_queue and self.motion_queue.closed:
                        self.continuous_ready = False
                        self.continuous_enabled.set(False)
                        self.queue_text.set('正在关闭使能并恢复参数；待执行指令已清空')
                        self.finish_target_edit(cancel=True)
                        self.update_buttons()
                elif kind == 'continuous_ready':
                    if self.continuous_active and not self.stop_token.is_set():
                        self.continuous_ready = True
                        self.queue_text.set('已使能；待执行 0 条；空闲保持使能')
                        self.update_buttons()
                elif kind == 'command':
                    if self.continuous_active and not self.stop_token.is_set():
                        text = f"指令 #{event['number']} " + ('执行中' if event['stage'] == 'started' else '已完成，保持使能')
                        self.status.set(text)
                        self.queue_text.set(text + f"；待执行 {self.motion_queue.pending()} 条")
                        self.append(text)
                elif kind == 'status':
                    for axis in event['axes']:
                        row = str(axis['order'])
                        if self.tree.exists(row):
                            self.tree.set(row, 'position', str(axis['position']))
                            state = f"报警 0x{axis['error_code']:04X}" if axis['error_code'] else '使能中' if axis['enabled'] else '未使能'
                            if axis['travel_degrees'] is not None:
                                state += f"  ·  {axis['travel_degrees']:+.2f}°"
                            self.tree.set(row, 'state', state)
                elif kind in ('done', 'failure'):
                    self.busy = False
                    self.continuous_active = self.continuous_ready = False
                    self.continuous_enabled.set(False)
                    self.motion_queue = None
                    self.queue_text.set('连续使能未开启')
                    self.finish_target_edit(cancel=True)
                    self.update_summary()
                    if kind == 'failure':
                        self.status.set('操作失败：' + event['text'])
                        self.append(event['text'])
                    elif event['operation'] == 'scan':
                        self.devices = event['result']
                        for d in self.devices:
                            self.target_values[d.order] = '0'
                            self.tree.insert('', 'end', iid=str(d.order), values=(
                                '☐', d.order, d.alias, f'{d.motor_code} / ' + ('顺时针' if d.positive_direction == 1 else '逆时针'),
                                f'{d.gear_numerator}:{d.gear_denominator}', d.position, '0',
                                f'H00.00={d.motor_code} 待核对，禁止运行' if d.motor_code != 14101 else
                                f'报警 0x{d.error_code:04X}' if d.error_code else '未使能 / PRE-OP'))
                        self.status.set(f'扫描到 {len(self.devices)} 台。请选择电机；扫描不会转动。')
                        self.append(self.status.get())
                        self.render_selection()
                    else:
                        report = event['result']
                        if report['success']:
                            text = '通信验证通过，电机未使能。' if report['dry_run'] else '运动完成，全部关闭使能。'
                        elif report.get('cleanup_errors'):
                            text = '停止/恢复未全部核对成功，请检查驱动器，必要时断电。'
                        elif report.get('stopped'):
                            text = '已停止，全部关闭使能。'
                        else:
                            text = '运行未完成：' + report.get('error', '未知错误')
                        self.status.set(text)
                        self.append(text)
                        if report.get('error'):
                            self.append(report['error'])
                        for a in report['axes']:
                            row = str(a['order'])
                            state = '未使能 / PRE-OP' if a.get('final_state') == 2 and not a.get('final_statusword', 4) & 4 else '请检查驱动器状态'
                            if a.get('final_error_code'):
                                state = f"报警 0x{a['final_error_code']:04X}"
                            if 'measured_degrees' in a:
                                state += f" · {a['measured_degrees']:+.3f}°"
                                self.append(f"ID {a['id']}：本次位移 {a['measured_degrees']:+.4f}°")
                            if self.tree.exists(row):
                                self.tree.set(row, 'state', state)
                        if report.get('cleanup_errors'):
                            self.append('；'.join(report['cleanup_errors']))
                        if report.get('log_path'):
                            self.append('记录：' + report['log_path'])
                    self.update_buttons()
                    if self.closing:
                        self.finish_close()
                        return
        except queue.Empty:
            pass
        self.poll_handle = self.after(80, self.poll)

    def finish_close(self):
        if self.poll_handle is not None:
            self.after_cancel(self.poll_handle)
            self.poll_handle = None
        self.finish_target_edit(cancel=True)
        self.executor.shutdown(wait=False)
        self.destroy()

    def close(self):
        if self.busy:
            self.closing = True
            self.stop()
            self.status.set('正在安全停止并保存记录，完成后窗口自动关闭。')
        else:
            self.finish_close()


if __name__ == '__main__':
    App().mainloop()

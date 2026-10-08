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
        self.title('SV635N 电机调试工具 v1.2')
        self.geometry('1080x900')
        self.minsize(1020, 860)
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
        self.after(80, self.poll)

    def _style(self):
        s = ttk.Style(self)
        s.theme_use('clam')
        s.configure('.', font=('Microsoft YaHei UI', 10), background='#eef2f6')
        s.configure('TFrame', background='#eef2f6')
        s.configure('Card.TFrame', background='white')
        s.configure('Card.TLabel', background='white', foreground='#172b4d')
        s.configure('Title.TLabel', font=('Microsoft YaHei UI', 21, 'bold'), foreground='#142b49')
        s.configure('Subtitle.TLabel', foreground='#5c6b7d')
        s.configure('TButton', padding=(12, 8))
        s.configure('Start.TButton', background='#1664c0', foreground='white', font=('Microsoft YaHei UI', 12, 'bold'))
        s.map('Start.TButton', background=[('disabled', '#c1cad5'), ('active', '#1357a6')])
        s.configure('Stop.TButton', background='#c53336', foreground='white', font=('Microsoft YaHei UI', 12, 'bold'))
        s.map('Stop.TButton', background=[('disabled', '#c1cad5'), ('active', '#aa2427')])
        s.configure('Treeview', rowheight=35, background='white', fieldbackground='white', font=('Microsoft YaHei UI', 10))
        s.configure('Treeview.Heading', font=('Microsoft YaHei UI', 10, 'bold'), padding=8)

    def _layout(self):
        root = ttk.Frame(self, padding=24)
        root.pack(fill='both', expand=True)
        ttk.Label(root, text='SV635N 电机调试', style='Title.TLabel').pack(anchor='w')
        ttk.Label(root, text='选择电机 · 设置方向与位移 · 单次运行后自动关闭使能', style='Subtitle.TLabel').pack(anchor='w', pady=(4, 17))
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
        self.tree = ttk.Treeview(root, columns=('choose', 'order', 'id', 'name', 'gear', 'position', 'state'), show='headings', height=4, selectmode='none')
        for name, label, width in [('choose', '选择', 55), ('order', '链路位置', 80), ('id', 'ID / H0E.21', 105),
                                    ('name', 'H00.00 / 正指令方向', 190), ('gear', '齿轮比', 75), ('position', '位置计数', 150), ('state', '状态', 220)]:
            self.tree.heading(name, text=label)
            self.tree.column(name, width=width, anchor='center', stretch=name in ('position', 'state'))
        self.tree.pack(fill='x', pady=8)
        self.tree.bind('<Button-1>', self.toggle)
        self.tree.tag_configure('chosen', background='#e9f2ff')
        self.tree.tag_configure('fault', foreground='#bb2025')
        ttk.Label(root, textvariable=self.selection_text, style='Subtitle.TLabel').pack(anchor='w', pady=(0, 12))
        card = ttk.Frame(root, style='Card.TFrame', padding=16)
        card.pack(fill='x')
        ttk.Label(card, text='2  设置选中电机的运动参数', style='Card.TLabel', font=('Microsoft YaHei UI', 11, 'bold')).grid(row=0, column=0, columnspan=7, sticky='w', pady=(0, 13))
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
        ttk.Label(card, text='加减速度', style='Card.TLabel').grid(row=2, column=0, sticky='w', pady=(14, 0))
        self.acceleration_entry = ttk.Entry(card, textvariable=self.acceleration, width=10)
        self.acceleration_entry.grid(row=2, column=1, sticky='w', padx=8, pady=(14, 0))
        ttk.Label(card, text='rpm/s（1～120，加速与减速相同）', style='Card.TLabel').grid(row=2, column=2, columnspan=5, sticky='w', pady=(14, 0))
        ttk.Label(card, text='电机编码器', style='Card.TLabel').grid(row=3, column=0, sticky='w', pady=(14, 0))
        self.encoder_box = ttk.Combobox(card, textvariable=self.encoder, values=('A3 · 23位（逐轴核对）',), state='readonly', width=23)
        self.encoder_box.grid(row=3, column=1, columnspan=2, sticky='w', padx=8, pady=(14, 0))
        ttk.Label(card, text='选中轴须 H00.00=14101；本版本仅支持已核对的 A3。', style='Card.TLabel').grid(row=3, column=3, columnspan=4, sticky='w', pady=(14, 0))
        ttk.Label(card, textvariable=self.summary, style='Card.TLabel', wraplength=960).grid(row=4, column=0, columnspan=7, sticky='w', pady=(15, 0))
        ttk.Label(root, text='角度 0.1～3600°（360° = 1圈） | 速度 0.1～60 rpm | 方向从轴端看，程序按各轴 H02.02 自动换算', style='Subtitle.TLabel').pack(anchor='w', pady=(9, 8))
        self.ready_box = ttk.Checkbutton(root, text='已确认：电机固定、轴端空载、手已离开，且可随时断电', variable=self.ready, command=self.update_buttons)
        self.ready_box.pack(anchor='w')
        actions = ttk.Frame(root)
        actions.pack(fill='x', pady=12)
        self.start_button = ttk.Button(actions, text='开始单次运行', style='Start.TButton', command=lambda: self.run_motion(False))
        self.start_button.pack(side='left', ipadx=15)
        self.prepare_button = ttk.Button(actions, text='验证通信（不转动）', command=lambda: self.run_motion(True))
        self.prepare_button.pack(side='left', padx=10)
        self.stop_button = ttk.Button(actions, text='停止全部 / Esc', style='Stop.TButton', command=self.stop)
        self.stop_button.pack(side='right', ipadx=15)
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
            self.summary.set(f'选中电机均{sign} {angle:g}°，预计运动 {move.estimated_seconds:.2f} 秒，'
                             f'峰值约 {move.peak_rpm:.1f} rpm；含加减速，多选同时启动。')
        except (ValueError, RuntimeError):
            self.summary.set('请输入范围内的角度、最高速度和加减速度。')

    def update_buttons(self):
        available = bool(self.devices and self.selected)
        for widget in (self.scan_button, self.refresh_button, self.all_button, self.none_button,
                       self.forward, self.reverse, self.angle_entry, self.rpm_entry, self.acceleration_entry, self.ready_box):
            widget.configure(state='disabled' if self.busy else 'normal')
        self.adapter_box.configure(state='disabled' if self.busy else 'readonly')
        self.encoder_box.configure(state='disabled' if self.busy else 'readonly')
        self.start_button.configure(state='normal' if not self.busy and available and self.ready.get() else 'disabled')
        self.prepare_button.configure(state='normal' if not self.busy and available else 'disabled')
        self.stop_button.configure(state='normal' if self.busy else 'disabled')

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
        self.devices = []
        self.selected.clear()
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
        self.ready.set(False)
        self.devices = []
        self.selected.clear()
        self.tree.delete(*self.tree.get_children())
        self.selection_text.set('扫描中；电机不会使能。')
        self.submit('scan', controller.scan)

    def toggle(self, event):
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

    def stop(self):
        if self.busy:
            self.stop_token.set()
            self.status.set('已请求停止；正在减速并关闭全部使能，请勿强制结束程序。')
            self.append('停止请求已发送。')

    def poll(self):
        try:
            while True:
                event = self.events.get_nowait()
                kind = event['kind']
                if kind == 'phase':
                    self.status.set(event['text'])
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
                    if kind == 'failure':
                        self.status.set('操作失败：' + event['text'])
                        self.append(event['text'])
                    elif event['operation'] == 'scan':
                        self.devices = event['result']
                        for d in self.devices:
                            self.tree.insert('', 'end', iid=str(d.order), values=(
                                '☐', d.order, d.alias, f'{d.motor_code} / ' + ('顺时针' if d.positive_direction == 1 else '逆时针'),
                                f'{d.gear_numerator}:{d.gear_denominator}', d.position,
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
                        self.executor.shutdown(wait=False)
                        self.destroy()
                        return
        except queue.Empty:
            pass
        self.after(80, self.poll)

    def close(self):
        if self.busy:
            self.closing = True
            self.stop()
            self.status.set('正在安全停止并保存记录，完成后窗口自动关闭。')
        else:
            self.executor.shutdown(wait=False)
            self.destroy()


if __name__ == '__main__':
    App().mainloop()

import threading
from contextlib import contextmanager
import gc
import subprocess
import sys
from pathlib import Path
import time
import tkinter as tk
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from core import ControlError, EtherCATController
from test_platform_control import FakeMaster
from UdpControl.debug_ui import DebugWindow
from udp_server import UDPServer
from motor_service import MotorService
from backend import run_tray
from UdpControl.client import MotorClient


class UITests(unittest.TestCase):
    def setUp(self):
        self.masters=[]
        def factory(adapter,logs):
            master=FakeMaster()
            self.masters.append(master)
            return EtherCATController('fake',master_factory=lambda:master)
        self.service=MotorService('fake',controller_factory=factory)
        self.service.adapters=lambda:[dict(name='fake',description='模拟控制卡')]
        self.server=UDPServer(self.service,port=0).start()

    def tearDown(self):
        self.server.close()
        self.service.close()
        gc.collect()

    def pump(self,root,predicate,timeout=6):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            root.update()
            if predicate():
                return
            time.sleep(.01)
        self.fail(str(self.service.snapshot()))

    @contextmanager
    def window(self):
        root=tk.Tk()
        ui=DebugWindow(root,'127.0.0.1',self.server.port)
        try:
            yield root,ui
        finally:
            ui.close()
            root.destroy()
            del ui,root
            gc.collect()

    def test_claim_automatically_scans_and_enables_all_without_motion(self):
        with self.window() as (root,ui):
            self.pump(root,lambda:bool(ui.state) and not ui.pending)
            self.assertFalse(self.masters)  # Opening a read-only panel does not enable.
            ui.claim()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
            self.assertEqual(self.service.orders,[1,2,3,4])
            self.assertEqual(ui.selected,{1,2,3,4})
            self.assertFalse(hasattr(ui,'ready_box'))
            self.assertFalse(hasattr(ui,'enable_button'))
            self.assertEqual(self.service.commands.planned,(0,0,0,0))
            self.assertFalse(any(s.targets for m in self.masters for s in m.slaves))
            self.assertEqual(ui.client.run_id,self.service.run_id)
            ui.stop()
            self.pump(root,lambda:ui.state.get('phase')=='idle' and not ui.pending)
            count=len(self.masters)
            end=time.monotonic()+.6
            while time.monotonic()<end:
                root.update()
                time.sleep(.01)
            self.assertEqual(self.service.phase,'idle')
            self.assertEqual(len(self.masters),count)

    def test_nic_allows_3000_maximum_and_preserves_selected_speed(self):
        with self.window() as (root, ui):
            self.pump(root, lambda: bool(ui.state) and not ui.pending)
            state = self.service.snapshot()
            state['motion_safety'] = dict(max_rpm=3000, unreferenced_max_rpm=3000, calibrated=False,
                                          stop_deceleration_rpm_s=120)
            ui.render(state)
            self.assertEqual(float(ui.rpm.get()), 60)
            self.assertEqual(float(ui.rpm_slider['to']), 3000)
            self.assertIn('3000 rpm', ui.limit_hint['text'])
            ui.rpm.set('1200')
            ui.render(state)
            self.assertEqual(float(ui.rpm.get()), 1200)

    def test_homing_keeps_stop_available_and_normal_speed_selection_unchanged(self):
        with self.window() as (root,ui):
            self.pump(root,lambda:bool(ui.state) and not ui.pending)
            ui.client.session='owned'
            ui.client.run_id='homing-test'
            ui.rpm.set('3000')
            state=self.service.snapshot()
            state.update(phase='enabled',control_mode='homing',run_id='homing-test',
                orders=[1,2,3],auto_center=dict(enabled=True,ready=False),
                profile=dict(rpm=6,acceleration_rpm_s=120,revision=0))
            ui.state=state
            ui.render(state)
            self.assertEqual(str(ui.slider['state']),'disabled')
            self.assertEqual(str(ui.stop_button['state']),'normal')
            self.assertEqual(float(ui.rpm.get()),3000)
            self.assertIn('回中进行中',ui.limit_hint['text'])
            ui.queue_axis_target(1,300)
            ui.flush_motion()
            self.assertIsNone(ui.latest_target)
            ui.client.run_id=None
            ui.client.session=None

    def test_platform_setup_prefills_measured_safe_travel_instead_of_old_nominal_range(self):
        with self.window() as (root,ui):
            self.pump(root,lambda:bool(ui.state) and not ui.pending)
            state=self.service.snapshot()
            state.update(phase='idle',fixed_reference=dict(valid=True,id='measured'),
                auto_center=dict(enabled=True,ready=True,result=dict(axes=[
                    dict(order=i+1,up_mm=31+i,down_mm=30+i) for i in range(3)])))
            ui.state=state
            ui.open_platform_setup()
            dialog=next(w for w in root.winfo_children() if isinstance(w,tk.Toplevel))
            frame=dialog.winfo_children()[0]
            entries=[w for w in frame.winfo_children() if w.winfo_class()=='TEntry']
            self.assertEqual([w.get() for w in entries],['31','30','32','31','33','32'])
            dialog.grab_release()
            dialog.destroy()

    def test_limit_editor_available_from_fault_and_only_sends_explicit_settings(self):
        with self.window() as (root, ui):
            self.pump(root, lambda: bool(ui.state) and not ui.pending)
            state = self.service.snapshot()
            state.update(phase='fault', devices=[dict(order=1,id=0,name='SV635N',motor_code=14101,position=123)],
                limit_settings=dict(available=True, axes=[dict(order=1,position=123,origin=100,
                    current_degrees=2.5,enabled=True,lower_degrees=-90,upper_degrees=90)]))
            ui.render(state)
            ui.selected = {1}
            self.assertIsNone(ui.client.session)
            self.assertEqual(str(ui.reset_limits_button['state']), 'normal')
            self.assertEqual(str(ui.soft_limits_button['state']), 'normal')
            with patch.object(ui.client, 'hello') as claim, patch.object(ui.client, 'reset_limits') as reset, \
                 patch.object(ui, 'task', side_effect=lambda fn, label:fn()):
                ui.reset_limits_button.invoke()
                claim.assert_called_once_with(claim=True)
                reset.assert_called_once_with([1])
            ui.open_soft_limits()
            row = ui.soft_limit_rows[0]
            self.assertEqual(row['origin'].get(), '100')
            row['lower'].set('-180')
            row['upper'].set('360')
            with patch.object(ui.client, 'hello') as claim, patch.object(ui.client, 'configure_soft_limits') as save, \
                 patch.object(ui, 'task', side_effect=lambda fn, label:fn()):
                ui.save_soft_limits()
                claim.assert_called_once_with(claim=True)
                save.assert_called_once_with([dict(order=1,enabled=True,origin=100,lower_degrees=-180.,upper_degrees=360.)])
            state['phase'] = 'enabled'
            ui.render(state)
            self.assertEqual(str(ui.reset_limits_button['state']), 'disabled')
            self.assertEqual(str(ui.soft_limits_button['state']), 'disabled')
            self.assertFalse(self.masters)

    def test_limit_editor_keeps_dialog_open_for_invalid_bounds_without_request(self):
        with self.window() as (root, ui):
            self.pump(root, lambda: bool(ui.state) and not ui.pending)
            state = self.service.snapshot()
            state['limit_settings'] = dict(available=True, axes=[dict(order=1,position=123,origin=123,
                current_degrees=0,enabled=False,lower_degrees=None,upper_degrees=None)])
            ui.render(state)
            ui.open_soft_limits()
            row = ui.soft_limit_rows[0]
            row['enabled'].set(True)
            with patch('UdpControl.debug_ui.messagebox.showerror') as error, patch.object(ui, 'task') as task:
                for low, high in (('0','0'), ('90','-90'), ('-90','nan'), ('-90','inf')):
                    row['lower'].set(low)
                    row['upper'].set(high)
                    ui.save_soft_limits()
                self.assertEqual(error.call_count, 4)
                task.assert_not_called()
            self.assertTrue(ui.soft_limits_dialog.winfo_exists())

    def test_opening_panel_automatically_refreshes_card_without_taking_control(self):
        with MotorClient(port=self.server.port) as owner:
            owner.hello()
            token=self.server.token
            with self.window() as (root,ui):
                self.pump(root,lambda:ui.adapter_server_id==self.server.server_id and not ui.pending)
                self.assertEqual(ui.adapter_box.current(),0)
                self.assertIn('模拟控制卡',ui.adapter.get())
                self.assertIsNone(ui.client.session)
                self.assertEqual(self.server.token,token)
                self.assertFalse(self.masters)
                self.assertEqual(self.service.phase,'idle')
                self.assertEqual(str(ui.adapters_button['state']),'normal')

    def test_read_only_panel_refreshes_card_and_displays_startup_scan_results(self):
        self.service.scan()
        with self.window() as (root,ui):
            self.pump(root,lambda:ui.adapter_server_id==self.server.server_id
                      and len(ui.tree.get_children())==4 and not ui.pending)
            self.assertIn('模拟控制卡',ui.adapter.get())
            self.assertIsNone(ui.client.session)
            self.assertIsNone(self.service.run_id)
            self.assertEqual(self.service.orders,[])
            self.assertTrue(all(not slave.targets for master in self.masters for slave in master.slaves))

    def test_card_refresh_retries_after_transient_failure_without_enabling(self):
        calls=[]
        def adapters():
            calls.append(time.monotonic())
            if len(calls)==1:
                raise ControlError('模拟卡列表刷新暂时失败')
            return [dict(name='fake',description='模拟控制卡')]
        self.service.adapters=adapters
        with self.window() as (root,ui):
            self.pump(root,lambda:ui.adapter_server_id==self.server.server_id and not ui.pending)
            self.assertEqual(len(calls),2)
            self.assertGreaterEqual(calls[1]-calls[0],.9)
            self.assertIsNone(ui.client.session)
            self.assertFalse(self.masters)

    def test_panel_opened_before_backend_refreshes_card_when_service_connects(self):
        port=self.server.port
        self.server.close()
        with self.window() as (root,ui):
            self.pump(root,lambda:'未收到确认' in ui.log.get('1.0','end'))
            self.server=UDPServer(self.service,port=port).start()
            self.pump(root,lambda:ui.adapter_server_id==self.server.server_id and not ui.pending)
            self.assertIn('模拟控制卡',ui.adapter.get())
            self.assertIsNone(ui.client.session)
            self.assertFalse(self.masters)

    def test_same_control_button_restores_all_motors_after_explicit_stop(self):
        with self.window() as (root,ui):
            ui.claim()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
            previous = ui.client.run_id
            session = ui.client.session
            ui.stop()
            self.pump(root,lambda:ui.state.get('phase')=='idle' and not ui.pending and not ui.stop_pending)
            self.assertEqual(str(ui.claim_button.cget('state')),'normal')
            ui.claim_button.invoke()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
            self.assertEqual(ui.client.session,session)
            self.assertNotEqual(ui.client.run_id,previous)
            self.assertEqual(self.service.orders,[1,2,3,4])
            self.assertFalse(any(s.targets for m in self.masters for s in m.slaves))

    def test_scan_results_show_drive_alarm_in_hex_without_axis_feedback(self):
        with self.window() as (root,ui):
            ui.render(dict(phase='idle',devices=[dict(order=3,id=2,name='SV635N',
                           motor_code=14101,position=0,error_code=0x5443)],axes=[]))
            self.assertEqual(ui.tree.set('3','state'),'报警 0x5443')

    def test_nonfault_warning_keeps_enabled_state_visible(self):
        with self.window() as (root,ui):
            ui.render(dict(phase='enabled',devices=[dict(order=1,id=0,name='SV635N',
                           motor_code=14101,position=0,error_code=0x6320,fault=False)],
                           axes=[dict(order=1,position=0,travel_degrees=0,enabled=True,
                                      error_code=0x6320,fault=False)]))
            self.assertEqual(ui.tree.set('1','state'),'使能 · 警告 0x6320')

    def test_limit_hold_revision_clears_stale_local_target_without_changing_run(self):
        with self.window() as (root,ui):
            state=dict(phase='enabled',run_id='same-run',orders=[1],targets_deg=[60],
                       target_reset_revision=0,devices=[dict(order=1,id=0,name='SV635N',
                           motor_code=14101,position=0,error_code=0,fault=False)],
                       axes=[dict(order=1,position=0,travel_degrees=0,enabled=True,error_code=0,fault=False)])
            ui.client.run_id='same-run'
            ui.render(state)
            ui.latest_target=[60]
            ui.render(dict(state,targets_deg=[-4],target_reset_revision=1))
            self.assertEqual(ui.target_values,{1:-4})
            self.assertIsNone(ui.latest_target)
            self.assertEqual(ui.client.run_id,'same-run')

    def test_nic_limit_stop_preserves_free_slider_and_newer_pending_target(self):
        with self.window() as (root, ui):
            state=dict(phase='enabled',run_id='same-run',orders=[1],targets_deg=[60],
                       target_reset_revision=0,target_hold_preserves_request=True,
                       devices=[dict(order=1,id=0,name='SV635N',motor_code=14101,
                                     position=0,error_code=0,fault=False)],
                       axes=[dict(order=1,position=0,travel_degrees=0,enabled=True,error_code=0,fault=False)])
            ui.client.run_id, ui.client.session = 'same-run', 'test-session'
            ui.render(state)
            ui.queue_axis_target(None, 240)
            ui.sync_slider()
            ui.render(dict(state,target_reset_revision=1))
            self.assertEqual(ui.target_values, {1:240})
            self.assertEqual(ui.latest_target, [240])
            self.assertEqual(float(ui.slider.get()), 240)
            self.assertEqual(ui.tree.set('1', 'target'), '240')

    def test_idle_control_keeps_continuous_enable_without_ui_pumping_or_targets(self):
        with self.window() as (root,ui):
            ui.claim()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
            run_id = ui.client.run_id
            time.sleep(1.1)  # Independent heartbeat must survive a blocked/minimized UI.
            self.assertEqual(self.service.phase,'enabled')
            self.assertEqual(self.service.run_id,run_id)
            self.assertTrue(self.service.snapshot()['enabled'])
            self.assertFalse(any(s.targets for m in self.masters for s in m.slaves))

    def test_stop_during_auto_scan_prevents_later_enable(self):
        started,resume=threading.Event(),threading.Event()
        factory=self.service.controller_factory
        def slow_factory(adapter,logs):
            controller=factory(adapter,logs)
            scan=controller.scan
            def delayed_scan():
                started.set()
                resume.wait(3)
                return scan()
            controller.scan=delayed_scan
            return controller
        self.service.controller_factory=slow_factory
        try:
            with self.window() as (root,ui):
                ui.claim()
                self.pump(root,started.is_set)
                self.assertEqual(ui.auto_start,'scanning')
                ui.stop()
                resume.set()
                self.pump(root,lambda:ui.state.get('phase')=='idle' and len(ui.tree.get_children())==4 and not ui.pending)
                self.assertIsNone(ui.auto_start)
                self.assertIsNone(self.service.run_id)
                self.assertEqual(len(self.masters),1)
                self.assertFalse(any(s.words for s in self.masters[0].slaves))
        finally:
            resume.set()

    def test_claim_waits_for_backend_startup_scan_before_auto_scan_and_enable(self):
        started,resume=threading.Event(),threading.Event()
        factory=self.service.controller_factory
        def slow_first_factory(adapter,logs):
            controller=factory(adapter,logs)
            if len(self.masters)==1:
                scan=controller.scan
                def delayed_scan():
                    started.set()
                    resume.wait(3)
                    return scan()
                controller.scan=delayed_scan
            return controller
        self.service.controller_factory=slow_first_factory
        self.service.scan()
        try:
            with self.window() as (root,ui):
                ui.claim()
                self.pump(root,lambda:started.is_set() and ui.auto_start=='wait_idle' and ui.state.get('phase')=='scanning')
                self.assertIsNone(self.service.run_id)
                resume.set()
                self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
                self.assertEqual(self.service.orders,[1,2,3,4])
                self.assertEqual(len(self.masters),3)
                self.assertFalse(any(s.targets for m in self.masters for s in m.slaves))
        finally:
            resume.set()

    def test_stop_with_enable_ack_in_flight_disables_that_run(self):
        held=[]
        release_ack=threading.Event()
        send=self.server._send
        def hold_enable_ack(packet,peer):
            if (packet.get('type')=='ack' and packet.get('run_id') and not release_ack.is_set()
                    and (not held or packet.get('id')==held[0][0]['id'])):
                if not held:
                    held.append((packet,peer))
            else:
                send(packet,peer)
        self.server._send=hold_enable_ack
        try:
            with self.window() as (root,ui):
                ui.claim()
                self.pump(root,lambda:bool(held))
                self.assertIsNone(ui.client.run_id)
                ui.stop()
                release_ack.set()
                send(*held[0])
                self.pump(root,lambda:self.service.phase=='idle' and ui.state.get('phase')=='idle' and not ui.pending)
                self.assertIsNone(ui.auto_start)
                self.assertIsNone(ui.client.run_id)
                self.assertTrue(self.service.snapshot()['result']['all_disabled'])
                self.assertFalse(any(s.targets for m in self.masters for s in m.slaves))
        finally:
            release_ack.set()
            self.server._send=send

    def test_scan_or_device_failure_does_not_retry_auto_enable(self):
        for failure in ('scan','empty','motor'):
            with self.subTest(failure=failure):
                self.masters.clear()
                def factory(adapter,logs):
                    if failure in ('scan','empty'):
                        def scan():
                            if failure=='scan':
                                raise ControlError('模拟扫描失败')
                            return []
                        return SimpleNamespace(scan=scan)
                    master=FakeMaster()
                    master.slaves[0].params[0x2000,1]=999
                    self.masters.append(master)
                    return EtherCATController('fake',master_factory=lambda:master)
                self.service.controller_factory=factory
                with self.window() as (root,ui):
                    ui.claim()
                    self.pump(root,lambda:ui.client.session and not ui.auto_start and not ui.pending)
                    self.assertIsNone(self.service.run_id)
                    self.assertEqual(self.service.phase,'fault' if failure=='scan' else 'idle')
                    self.assertFalse(any(s.words for m in self.masters for s in m.slaves))
                    self.assertFalse(any(s.targets for m in self.masters for s in m.slaves))

    def test_other_owner_rejects_claim_without_scanning(self):
        with MotorClient(port=self.server.port) as owner:
            owner.hello()
            with self.window() as (root,ui):
                ui.claim()
                self.pump(root,lambda:ui.auto_start is None and not ui.pending)
                self.assertIsNone(ui.client.session)
                self.assertFalse(self.masters)

    def test_slow_status_request_does_not_delay_motor_targets(self):
        started,resume=threading.Event(),threading.Event()
        with self.window() as (root,ui):
            try:
                ui.claim()
                self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
                def slow_status():
                    started.set()
                    resume.wait(2)
                    return ui.client.hello(claim=True)
                ui.task(slow_status,'status')
                self.pump(root,started.is_set)
                ui.queue_axis_target(1,5)
                self.pump(root,lambda:self.service.commands.planned==(5,0,0,0),timeout=.3)
                self.assertTrue(ui.pending)
                self.assertFalse(resume.is_set())
                self.assertEqual(ui.client.run_id,self.service.run_id)
            finally:
                resume.set()

    def test_lost_target_ack_retries_without_restarting_the_target(self):
        attempts=[]
        target_ids=set()
        dropped=threading.Event()
        send,handle=self.server._send,self.server.handle
        def track(packet,peer):
            if packet.get('type')=='target':
                target_ids.add(packet['id'])
                attempts.append(packet['id'])
            return handle(packet,peer)
        def lose_one(packet,peer):
            if packet.get('id') in target_ids and not dropped.is_set():
                dropped.set()
                return
            send(packet,peer)
        self.server.handle,self.server._send=track,lose_one
        try:
            with self.window() as (root,ui):
                ui.claim()
                self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
                number=self.service.commands.number
                ui.queue_axis_target(1,5)
                self.pump(root,lambda:len(attempts)>=2 and not ui.motion_pending and ui.latest_target is None)
                self.assertTrue(dropped.is_set())
                self.assertEqual(len(set(attempts)),1)
                self.assertEqual(self.service.commands.number,number+1)
                self.assertEqual(self.service.commands.planned,(5,0,0,0))
        finally:
            self.server._send,self.server.handle=send,handle

    def test_ui_uses_udp_selection_and_hides_without_stopping(self):
        root=tk.Tk()
        ui=DebugWindow(root,'127.0.0.1',self.server.port)
        try:
            ui.task(lambda:ui.client.hello(claim=True),'claim_only')
            self.pump(root,lambda:bool(ui.client.session) and not ui.pending)
            ui.scan()
            self.pump(root,lambda:len(ui.tree.get_children())==4 and not ui.pending)
            ui.selected={1,3}
            ui.enable()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending)
            ui.slider.set(25)
            self.pump(root,lambda:self.service.commands.planned==(25,25))
            self.assertEqual(ui.client.run_id,self.service.run_id)
            self.assertFalse(self.masters[-1].slaves[1].targets)
            number=self.service.commands.number
            ui.center.set('1000')
            ui.span.set('500')
            ui.set_range()
            root.update()
            self.assertEqual(number,self.service.commands.number)
            ui.hide()
            self.assertEqual(root.state(),'withdrawn')
            self.pump(root,lambda:ui.state.get('phase')=='enabled')
            time.sleep(.65)
            self.assertEqual(self.service.phase,'enabled')
            ui.show()
            root.update()
            self.assertGreater(ui.slider.winfo_width(),700)
            self.assertGreater(ui.log.winfo_height(),20)
            ui.stop()
            self.pump(root,lambda:self.service.phase=='idle')
            self.assertEqual(self.service.snapshot()['stop_reason'],'UDP 客户端关闭使能')
        finally:
            ui.close()
            root.destroy()
            del ui, root
            gc.collect()

    def test_display_range_change_preserves_targets_run_and_enable_after_idle_callbacks(self):
        with self.window() as (root, ui):
            ui.claim()
            self.pump(root, lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
            ui.queue_target(25)
            self.pump(root, lambda:self.service.commands.planned==(25,25,25,25))
            run_id, number = self.service.run_id, self.service.commands.number
            with patch.object(ui, 'queue_target', wraps=ui.queue_target) as target:
                for center, span in ((0,3600),(10000,3600),(0,360),(0,3600)):
                    ui.center.set(str(center))
                    ui.span.set(str(span))
                    ui.set_range()
                    root.update_idletasks()
                    root.update()
                time.sleep(.6)  # Independent heartbeat must survive display edits.
                root.update()
                target.assert_not_called()
            self.assertEqual(self.service.commands.number, number)
            self.assertEqual(self.service.commands.planned, (25,25,25,25))
            self.assertEqual(self.service.run_id, run_id)
            self.assertEqual(self.service.phase, 'enabled')
            self.assertAlmostEqual(ui.slider.get(), 25)
            ui.queue_target(30)
            self.pump(root, lambda:self.service.commands.planned==(30,30,30,30))

    def test_backend_tray_launches_separate_client_without_tk(self):
        quit_event=threading.Event()
        timer=threading.Timer(.5,quit_event.set)
        timer.start()
        try:
            with patch('pystray.Icon') as icon, patch('backend.subprocess.Popen') as launch:
                def start_icon():
                    menu=icon.call_args.args[3]
                    list(menu)[0](icon.return_value)
                icon.return_value.run_detached.side_effect=start_icon
                launch.return_value.poll.return_value=None
                run_tray(self.service,self.server,dict(host='127.0.0.1',auth_key=''),quit_event)
                icon.return_value.run_detached.assert_called_once()
                icon.return_value.stop.assert_called_once()
                launch.assert_called_once()
                command=launch.call_args.args[0]
                self.assertEqual(command[1:3],['-m','UdpControl'])
                self.assertIn(str(self.server.port),command)
                launch.return_value.wait.assert_called_once_with(timeout=3)
                launch.return_value.terminate.assert_not_called()
            self.assertFalse(self.service.is_busy())
            self.assertFalse(self.masters)
        finally:
            timer.cancel()

    def test_stuck_tray_child_is_terminated_after_graceful_shutdown_timeout(self):
        quit_event=threading.Event()
        with patch('pystray.Icon') as icon, patch('backend.subprocess.Popen') as launch:
            def launch_then_quit():
                list(icon.call_args.args[3])[0](icon.return_value)
                list(icon.call_args.args[3])[2](icon.return_value)
            icon.return_value.run_detached.side_effect=launch_then_quit
            launch.return_value.poll.return_value=None
            launch.return_value.wait.side_effect=[subprocess.TimeoutExpired('gui',3),0]
            run_tray(self.service,self.server,dict(host='127.0.0.1',auth_key=''),quit_event)
            launch.return_value.terminate.assert_called_once()
            self.assertEqual(launch.return_value.wait.call_count,2)
            self.assertTrue(self.service.shutdown.is_set())

    def test_reopening_tray_panel_keeps_original_child_for_exit_cleanup(self):
        quit_event=threading.Event()
        original=Mock()
        original.poll.return_value=None
        activation=Mock()
        with patch('pystray.Icon') as icon, patch('backend.subprocess.Popen',side_effect=[original,activation]) as launch:
            def reopen_then_quit():
                menu=list(icon.call_args.args[3])
                menu[0](icon.return_value)
                menu[0](icon.return_value)
                menu[2](icon.return_value)
            icon.return_value.run_detached.side_effect=reopen_then_quit
            run_tray(self.service,self.server,dict(host='127.0.0.1',auth_key=''),quit_event)
        self.assertEqual(launch.call_count,2)
        original.wait.assert_called_once_with(timeout=3)
        original.terminate.assert_not_called()
        activation.wait.assert_not_called()
        activation.terminate.assert_not_called()

    def test_running_panel_exits_on_backend_shutdown_and_motors_stop(self):
        with self.window() as (root,ui):
            exited=threading.Event()
            ui.exit_callback=exited.set
            ui.claim()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending and not ui.auto_start)
            self.server.close()
            self.pump(root,exited.is_set)
            self.pump(root,lambda:self.service.phase=='idle')
            self.assertTrue(self.service.snapshot()['result']['all_disabled'])
            self.assertTrue(ui.start_cancel.is_set())

    def test_standalone_gui_process_fully_exits_when_backend_closes(self):
        process=subprocess.Popen([sys.executable,'-m','UdpControl','--host','127.0.0.1',
                                  '--port',str(self.server.port)],
                                 cwd=Path(__file__).resolve().parent,
                                 stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        try:
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                with self.server.lock:
                    connected=bool(self.server.clients)
                if connected:
                    break
                time.sleep(.02)
            self.assertTrue(connected,'GUI did not connect to fake backend')
            duplicate=subprocess.run([sys.executable,'-m','UdpControl','--host','127.0.0.1',
                                      '--port',str(self.server.port)],
                                     cwd=Path(__file__).resolve().parent,
                                     stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=5)
            self.assertEqual(duplicate.returncode,0,duplicate.stderr.decode('utf-8',errors='replace'))
            self.assertIsNone(process.poll())
            with self.server.lock:
                self.assertEqual(len(self.server.clients),1,'Duplicate GUI opened a new UDP client')
            self.server.close()
            process.wait(timeout=5)
            self.assertEqual(process.returncode,0,process.stderr.read().decode('utf-8',errors='replace'))
            self.assertFalse(self.masters)  # This process only observed fake backend state.
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
            process.stderr.close()

    def test_profile_sliders_and_entries_update_while_enabled(self):
        root=tk.Tk()
        ui=DebugWindow(root,'127.0.0.1',self.server.port)
        try:
            ui.task(lambda:ui.client.hello(claim=True),'claim_only')
            self.pump(root,lambda:bool(ui.client.session) and not ui.pending)
            ui.scan()
            self.pump(root,lambda:len(ui.tree.get_children())==4 and not ui.pending)
            ui.selected={1,2,3}
            ui.enable()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending)
            run_id=ui.client.run_id
            self.assertEqual(str(ui.rpm_entry.cget('state')),'normal')
            self.assertEqual(str(ui.acceleration_slider.cget('state')),'normal')
            self.assertEqual(float(ui.rpm_slider.cget('to')),3000)
            self.assertEqual(float(ui.acceleration_slider.cget('to')),3000)
            ui.rpm_slider.set(40)
            ui.acceleration_slider.set(80)
            self.pump(root,lambda:(self.service.snapshot()['applied_profile'] or {}).get('acceleration_rpm_s')==80)
            self.assertEqual(self.service.commands.profile()[1:],(40,80))
            ui.rpm.set('3500')  # Numeric entry may exceed the convenience slider range.
            ui.acceleration.set('7000')
            self.pump(root,lambda:(self.service.snapshot()['applied_profile'] or {}).get('rpm')==3500)
            self.assertEqual(self.service.commands.profile()[1:],(3500,7000))
            self.assertEqual(ui.rpm.get(),'3500')
            self.assertEqual(self.service.commands.planned,(0,0,0))
            self.assertEqual(ui.client.run_id,run_id)
            self.assertFalse(any(s.targets for s in self.masters[-1].slaves))
            self.assertTrue(ui.stop_button.winfo_viewable())
            self.assertLessEqual(ui.stop_button.winfo_rooty()+ui.stop_button.winfo_height(),
                                 root.winfo_rooty()+root.winfo_height())
            ui.stop()
            self.pump(root,lambda:self.service.phase=='idle')
            self.assertIsNone(ui.latest_profile)
        finally:
            ui.close()
            root.destroy()
            del ui,root
            gc.collect()

    def test_three_axis_targets_and_single_axis_slider_preserve_other_axes(self):
        root=tk.Tk()
        ui=DebugWindow(root,'127.0.0.1',self.server.port)
        try:
            ui.task(lambda:ui.client.hello(claim=True),'claim_only')
            self.pump(root,lambda:bool(ui.client.session) and not ui.pending)
            ui.scan()
            self.pump(root,lambda:len(ui.tree.get_children())==4 and not ui.pending)
            ui.selected={1,2,3}
            ui.enable()
            self.pump(root,lambda:ui.state.get('phase')=='enabled' and not ui.pending)
            for order,value in ((1,5),(2,-3),(3,2)):
                ui.queue_axis_target(order,value)
            self.assertEqual(ui.latest_target,[5,-3,2])
            self.pump(root,lambda:self.service.commands.planned==(5,-3,2) and not ui.pending)
            number=self.service.commands.number
            ui.axis_box.current(2)
            ui.sync_slider()
            root.update()
            self.assertEqual(self.service.commands.number,number)
            ui.slider.set(7)
            self.pump(root,lambda:self.service.commands.planned==(5,7,2) and not ui.pending)
            self.assertFalse(self.masters[-1].slaves[3].targets)

            box=ui.tree.bbox('3','target')
            ui.edit_target(SimpleNamespace(x=box[0]+box[2]//2,y=box[1]+box[3]//2))
            self.assertIsNotNone(ui.target_editor)
            ui.target_editor.delete(0,'end')
            ui.target_editor.insert(0,'-4')
            self.pump(root,lambda:self.service.commands.planned==(5,7,-4) and not ui.pending)
            self.assertIsNotNone(ui.target_editor)  # Applying doesn't interrupt ongoing typing.
            ui.target_editor.event_generate('<FocusOut>')
            self.assertIsNone(ui.target_editor)
            self.assertEqual(ui.tree.set('3','target'),'-4')
            self.assertTrue(ui.stop_button.winfo_viewable())
            self.assertLessEqual(ui.stop_button.winfo_rooty()+ui.stop_button.winfo_height(),
                                 root.winfo_rooty()+root.winfo_height())

            box=ui.tree.bbox('1','target')
            ui.edit_target(SimpleNamespace(x=box[0]+box[2]//2,y=box[1]+box[3]//2))
            ui.target_editor.delete(0,'end')
            ui.target_editor.insert(0,'nan')
            number=self.service.commands.number
            with patch('UdpControl.debug_ui.messagebox.showerror') as error:
                self.pump(root,lambda:ui.target_edit_handle is None)
                error.assert_not_called()  # Incomplete/invalid typing never sends a target or opens a dialog.
                self.assertEqual(self.service.commands.number,number)
                ui.commit_target_edit()
                error.assert_called_once()
            self.assertEqual(self.service.commands.number,number)
            ui.target_editor.delete(0,'end')
            ui.target_editor.insert(0,'99')
            ui.stop()
            self.pump(root,lambda:self.service.phase=='idle' and ui.state.get('phase')=='idle')
            self.assertIsNone(ui.target_editor)
            self.assertIsNone(ui.target_edit_handle)
            self.assertEqual(self.service.commands.planned,(5,7,-4))
            self.assertEqual(str(ui.axis_box.cget('state')),'disabled')
        finally:
            ui.close()
            root.destroy()
            del ui,root
            gc.collect()

    def test_sensor_display_distinguishes_trigger_unknown_and_no_retraction_sensor(self):
        root=tk.Tk()
        ui=DebugWindow(root,'127.0.0.1',self.server.port)
        state=dict(phase='idle',adapter='PCIe-8332:0',devices=[dict(order=1,id=0,name='SV635N',motor_code=14101,position=0)],
                   limits=[dict(order=1,state='triggered',triggered=True,valid=True,
                                di1=True,di2=False,positive_limit=True,negative_limit=False,
                                retraction_state='no_sensor')])
        try:
            ui.render(state)
            self.assertEqual(ui.adapter_names,['PCIe-8332:0'])
            self.assertEqual(ui.adapter.get(),'ADLINK PCIe-8332:0')
            self.assertIn('已触发',ui.tree.set('1','limit'))
            self.assertEqual(ui.tree.item('1','tags'),('limit_triggered',))
            self.assertEqual(ui.tree.set('1','retraction'),'无传感器 / 未知')
            self.assertEqual(ui.tree.set('1','inputs'),'1 / 0 · 1 / 0')
            state['limits'][0].update(state='unavailable',triggered=None,valid=False)
            ui.render(state)
            self.assertEqual(ui.tree.set('1','limit'),'反馈失效')
            self.assertEqual(ui.tree.set('1','inputs'),'? / ? · ? / ?')
            ui.last_feedback=time.monotonic()-2
            root.after_cancel(ui.poll_handle)
            ui.poll()
            self.assertEqual(ui.tree.set('1','limit'),'反馈失联')
        finally:
            ui.close()
            root.destroy()
            del ui,root
            gc.collect()

    def test_dual_endpoint_display_and_stale_feedback(self):
        root=tk.Tk()
        ui=DebugWindow(root,'127.0.0.1',self.server.port)
        sensor=dict(order=1,input='di2',state='clear',triggered=False,valid=True,
                    retraction_input='di1',retraction_state='triggered',retraction_triggered=True,
                    di1=True,di2=False,positive_limit=True,negative_limit=False)
        state=dict(phase='idle',devices=[dict(order=1,id=0,name='SV635N',motor_code=14101,position=0)],
                   limits=[sensor])
        try:
            ui.render(state)
            self.assertIn('DI2 ○ 未触发',ui.tree.set('1','limit'))
            self.assertIn('DI1 ● 已触发（只可推出）',ui.tree.set('1','retraction'))
            self.assertEqual(ui.tree.item('1','tags'),('limit_triggered',))
            sensor.update(triggered=True,state='triggered',conflict=True)
            ui.render(state)
            self.assertEqual(ui.tree.set('1','limit'),'两端同时触发 / 故障')
            self.assertEqual(ui.tree.set('1','retraction'),'两端同时触发 / 故障')
            sensor.update(triggered=False,state='unavailable',valid=False,retraction_triggered=None,
                          retraction_state='unavailable',conflict=False)
            ui.render(state)
            self.assertIn('反馈失效',ui.tree.set('1','retraction'))
            ui.last_feedback=time.monotonic()-2
            root.after_cancel(ui.poll_handle)
            ui.poll()
            self.assertEqual(ui.tree.set('1','limit'),'反馈失联')
            self.assertEqual(ui.tree.set('1','retraction'),'反馈失联')
        finally:
            ui.close()
            root.destroy()
            del ui,root
            gc.collect()


if __name__=='__main__':
    unittest.main()

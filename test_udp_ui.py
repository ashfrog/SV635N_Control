import threading
import gc
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from core import EtherCATController
from test_platform_control import FakeMaster
from UdpControl.debug_ui import DebugWindow
from udp_server import UDPServer
from motor_service import MotorService
from backend import run_tray


class UITests(unittest.TestCase):
    def setUp(self):
        self.masters=[]
        def factory(adapter,logs):
            master=FakeMaster()
            self.masters.append(master)
            return EtherCATController('fake',master_factory=lambda:master)
        self.service=MotorService('fake',controller_factory=factory)
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

    def test_ui_uses_udp_selection_and_hides_without_stopping(self):
        root=tk.Tk()
        ui=DebugWindow(root,'127.0.0.1',self.server.port)
        try:
            ui.claim()
            self.pump(root,lambda:bool(ui.client.session) and not ui.pending)
            ui.scan()
            self.pump(root,lambda:len(ui.tree.get_children())==4 and not ui.pending)
            ui.selected={1,3}
            ui.ready.set(True)
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
                run_tray(self.service,self.server,dict(host='127.0.0.1',auth_key=''),quit_event)
                icon.return_value.run_detached.assert_called_once()
                icon.return_value.stop.assert_called_once()
                launch.assert_called_once()
                command=launch.call_args.args[0]
                self.assertEqual(command[1:3],['-m','UdpControl'])
                self.assertIn(str(self.server.port),command)
            self.assertFalse(self.service.is_busy())
            self.assertFalse(self.masters)
        finally:
            timer.cancel()


if __name__=='__main__':
    unittest.main()

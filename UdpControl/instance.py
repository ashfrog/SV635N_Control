"""Windows session-wide panel ownership and activation, without GUI imports."""
import ctypes
from ctypes import wintypes
import hashlib
import os
import socket


class PanelInstance:
    def __init__(self, host, port):
        self.kernel = None
        self.mutex = self.event = None
        self.primary = True
        if os.name != 'nt':
            return
        endpoint = f'{socket.gethostbyname(host)}:{port}'
        name = 'Local\\SV635N_Panel_' + hashlib.sha256(endpoint.encode()).hexdigest()[:32]
        self.kernel = kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.CreateMutexW.argtypes = (ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR)
        kernel.CreateMutexW.restype = wintypes.HANDLE
        kernel.CreateEventW.argtypes = (ctypes.c_void_p, wintypes.BOOL, wintypes.BOOL, wintypes.LPCWSTR)
        kernel.CreateEventW.restype = wintypes.HANDLE
        kernel.SetEvent.argtypes = (wintypes.HANDLE,)
        kernel.SetEvent.restype = wintypes.BOOL
        kernel.WaitForSingleObject.argtypes = (wintypes.HANDLE, wintypes.DWORD)
        kernel.WaitForSingleObject.restype = wintypes.DWORD
        kernel.CloseHandle.argtypes = (wintypes.HANDLE,)
        kernel.CloseHandle.restype = wintypes.BOOL
        try:
            # Create the event first so activation during GUI startup is kept.
            self.event = kernel.CreateEventW(None, False, False, name + '_Show')
            if not self.event:
                raise ctypes.WinError(ctypes.get_last_error())
            self.mutex = kernel.CreateMutexW(None, False, name)
            error = ctypes.get_last_error()
            if not self.mutex:
                raise ctypes.WinError(error)
            self.primary = error != 183  # ERROR_ALREADY_EXISTS
        except Exception:
            self.close()
            raise

    def activate(self):
        if self.event:
            # A user-launched duplicate can grant foreground permission to
            # the original GUI before asking its Tk thread to restore it.
            user32 = ctypes.WinDLL('user32', use_last_error=True)
            user32.AllowSetForegroundWindow.argtypes = (wintypes.DWORD,)
            user32.AllowSetForegroundWindow.restype = wintypes.BOOL
            user32.AllowSetForegroundWindow(0xffffffff)
            if not self.kernel.SetEvent(self.event):
                raise ctypes.WinError(ctypes.get_last_error())

    def requested(self):
        return bool(self.event and self.kernel.WaitForSingleObject(self.event, 0) == 0)

    def close(self):
        # No mutex ownership is acquired; handle lifetime is the process lease.
        if self.kernel:
            for handle in (self.mutex, self.event):
                if handle:
                    self.kernel.CloseHandle(handle)
        self.mutex = self.event = None

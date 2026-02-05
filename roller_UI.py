# -*- coding: utf-8 -*-
"""
roller_UI.py
- Simple PySide6 UI for roller control
- Uses roller.py direct WMX3 connection (no eq1_final modification)
"""

import sys
import time
from typing import Optional, Tuple

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QGridLayout, QSpinBox, QMessageBox
)

# ---- WMX3 DLL helper
try:
    from DllHelper import ensure_dll_exists
    ensure_dll_exists()
except Exception:
    ensure_dll_exists = None

# ---- WMX3 API
try:
    from WMX3ApiPython import WMX3Api, Io, DeviceType
except Exception:
    WMX3Api = None
    Io = None
    DeviceType = None

INFINITE = int(0xFFFFFFFF)

from roller import RollerController, RollerIOMap


class Wmx3Connection:
    """
    Open WMX3 communication and provide Io object.
    """
    def __init__(self, device_name: str = "EQ1_IO_Grid"):
        self.device_name = device_name
        self.wmx = None
        self.io = None
        self.connected = False

    def connect(self) -> Tuple[bool, str]:
        if WMX3Api is None or Io is None:
            return False, "WMX3ApiPython import failed"
        try:
            self.wmx = WMX3Api()
            self.io = Io(self.wmx)
            self.wmx.CreateDevice(r"C:\\Program Files\\SoftServo\\WMX3\\",
                                  DeviceType.DeviceTypeNormal, INFINITE)
            self.wmx.SetDeviceName(self.device_name)
            ret = self.wmx.StartCommunication(INFINITE)
            if ret != 0:
                return False, f"StartCommunication failed(ret={ret})"
            self.connected = True
            return True, "Connected"
        except Exception as e:
            self.connected = False
            return False, f"Connect exception: {e}"

    def disconnect(self) -> None:
        try:
            if self.wmx:
                self.wmx.StopCommunication(INFINITE)
        except Exception:
            pass
        self.connected = False
        self.wmx = None
        self.io = None


class Wmx3IoAdapter:
    """
    Adapter to match RollerController IO signature (port_no, byte, bit).
    """
    def __init__(self, io):
        self._io = io

    def SetOutBit(self, port_no: int, byte: int, bit: int, value: int):
        _ = port_no
        self._io.SetOutBit(byte, bit, int(value))

    def GetInBit(self, port_no: int, byte: int, bit: int):
        _ = port_no
        return self._io.GetInBit(byte, bit)


class RollerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Roller UI")
        self.resize(520, 360)

        self.roller = None
        self.conn = None
        self.iomap = RollerIOMap()

        self._init_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(100)

    def _init_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        # Top bar
        top = QHBoxLayout()
        layout.addLayout(top)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.on_connect)
        top.addWidget(self.btn_connect)

        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("color:#ff6b6b; font-weight:bold;")
        top.addWidget(self.lbl_status)

        top.addStretch(1)

        top.addWidget(QLabel("Poll (ms):"))
        self.spin_poll = QSpinBox()
        self.spin_poll.setRange(10, 2000)
        self.spin_poll.setValue(100)
        self.spin_poll.valueChanged.connect(self.on_poll_changed)
        top.addWidget(self.spin_poll)

        # Control buttons
        gb_ctrl = QGroupBox("Roller Control")
        layout.addWidget(gb_ctrl)
        ctrl = QHBoxLayout(gb_ctrl)

        self.btn_in = QPushButton("Move In")
        self.btn_in.clicked.connect(self.on_move_in)
        ctrl.addWidget(self.btn_in)

        self.btn_out = QPushButton("Move Out (Hold)")
        self.btn_out.clicked.connect(self.on_move_out)
        ctrl.addWidget(self.btn_out)

        self.btn_transfer = QPushButton("Transfer to AMR")
        self.btn_transfer.clicked.connect(self.on_transfer)
        ctrl.addWidget(self.btn_transfer)

        self.btn_stop = QPushButton("STOP")
        self.btn_stop.clicked.connect(self.on_stop)
        ctrl.addWidget(self.btn_stop)

        # IO Status
        gb_io = QGroupBox("IO Status")
        layout.addWidget(gb_io)
        grid = QGridLayout(gb_io)

        font = QFont("Consolas", 10)

        grid.addWidget(QLabel("OUT RUN"), 0, 0)
        self.lbl_out_run = QLabel("-")
        self.lbl_out_run.setFont(font)
        grid.addWidget(self.lbl_out_run, 0, 1)

        grid.addWidget(QLabel("OUT DIR"), 1, 0)
        self.lbl_out_dir = QLabel("-")
        self.lbl_out_dir.setFont(font)
        grid.addWidget(self.lbl_out_dir, 1, 1)

        grid.addWidget(QLabel("IN InwardStop"), 2, 0)
        self.lbl_in_inward = QLabel("-")
        self.lbl_in_inward.setFont(font)
        grid.addWidget(self.lbl_in_inward, 2, 1)

        grid.addWidget(QLabel("IN OutwardStop"), 3, 0)
        self.lbl_in_outward = QLabel("-")
        self.lbl_in_outward.setFont(font)
        grid.addWidget(self.lbl_in_outward, 3, 1)

        self.lbl_last = QLabel("Last: -")
        layout.addWidget(self.lbl_last)

        self._set_controls_enabled(False)

    def _set_controls_enabled(self, enabled: bool):
        self.btn_in.setEnabled(enabled)
        self.btn_out.setEnabled(enabled)
        self.btn_transfer.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)

    def on_poll_changed(self, v: int):
        self.timer.setInterval(int(v))
        if self.roller:
            self.roller.poll_interval_s = max(int(v) / 1000.0, 0.001)

    def on_connect(self):
        if self.conn is not None:
            try:
                self.conn.disconnect()
            finally:
                self.conn = None
                self.roller = None
                self.lbl_status.setText("Disconnected")
                self.lbl_status.setStyleSheet("color:#ff6b6b; font-weight:bold;")
                self.btn_connect.setText("Connect")
                self._set_controls_enabled(False)
            return

        try:
            conn = Wmx3Connection(device_name="EQ1_IO_Grid")
            ok, msg = conn.connect()
            if not ok:
                raise RuntimeError(msg)
            io = Wmx3IoAdapter(conn.io)
            roller = RollerController(io=io, poll_interval_s=self.spin_poll.value() / 1000.0)
            self.roller = roller
            self.conn = conn
            self.lbl_status.setText("Connected")
            self.lbl_status.setStyleSheet("color:#7CFC98; font-weight:bold;")
            self.btn_connect.setText("Disconnect")
            self._set_controls_enabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Connect Error", str(e))

    def _log_last(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.lbl_last.setText(f"Last: [{ts}] {msg}")

    def on_move_in(self):
        if not self.roller:
            return
        ok = self.roller.move_in_until_sensor(timeout_s=10)
        self._log_last(f"Move In -> {'OK' if ok else 'TIMEOUT'}")

    def on_move_out(self):
        if not self.roller:
            return
        ok = self.roller.move_out_until_sensor_hold(timeout_s=10)
        self._log_last(f"Move Out -> {'OK' if ok else 'TIMEOUT'}")

    def on_transfer(self):
        if not self.roller:
            return
        ok = self.roller.transfer_to_amr(post_clear_delay_s=5.0, timeout_wait_clear_s=10)
        self._log_last(f"Transfer -> {'OK' if ok else 'FAIL'}")

    def on_stop(self):
        if not self.roller:
            return
        self.roller.stop()
        self._log_last("STOP")

    def refresh_status(self):
        if not self.conn or not self.roller:
            self.lbl_out_run.setText("-")
            self.lbl_out_dir.setText("-")
            self.lbl_in_inward.setText("-")
            self.lbl_in_outward.setText("-")
            return

        io = self.conn.io
        if io is None:
            return

        # OUT
        try:
            ret, v_run = io.GetOutBit(self.iomap.out_run.byte, self.iomap.out_run.bit)
            ret, v_dir = io.GetOutBit(self.iomap.out_dir.byte, self.iomap.out_dir.bit)
        except Exception:
            v_run = 0
            v_dir = 0

        # IN
        try:
            ret, v_in = io.GetInBit(self.iomap.in_inward_stop.byte, self.iomap.in_inward_stop.bit)
            ret, v_out = io.GetInBit(self.iomap.in_outward_stop.byte, self.iomap.in_outward_stop.bit)
        except Exception:
            v_in = 0
            v_out = 0

        self.lbl_out_run.setText("ON" if int(v_run) == 1 else "OFF")
        self.lbl_out_dir.setText("OUT" if int(v_dir) == 1 else "IN")
        self.lbl_in_inward.setText("1" if int(v_in) == 1 else "0")
        self.lbl_in_outward.setText("1" if int(v_out) == 1 else "0")


def main():
    app = QApplication(sys.argv)
    w = RollerWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

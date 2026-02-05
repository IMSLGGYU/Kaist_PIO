# -*- coding: utf-8 -*-
"""
EQ1 controller UI
- Two tabs (main/other) talking to separate backend processes
"""

import json
import socket
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QPushButton, QLabel, QLineEdit, QTableWidget, QTableWidgetItem,
    QAbstractItemView, QSpinBox, QPlainTextEdit, QButtonGroup, QRadioButton,
    QTabWidget
)


# =========================
# IO Point Definition
# =========================
@dataclass(frozen=True)
class IOPoint:
    tag: str
    addr: str
    desc: str
    direction: str          # "IN" or "OUT"
    byte_index: int
    bit_index: int
    invert: bool = False


# =========================
# PIO Config (Port side)
# =========================
class PIOBits:
    # Port -> Vehicle (OUT)
    L_REQ   = 0
    U_REQ   = 1
    READY   = 3
    HO_AVBL = 6
    ES      = 7

    # Vehicle -> Port (IN)
    VALID   = 0
    CS_0    = 1
    CS_1    = 2
    TR_REQ  = 4
    BUSY    = 5
    COMPT   = 6


def build_channels(profile: str) -> Tuple[Dict[str, int], Dict[str, str]]:
    if profile == "main":
        channels = {
            "EQ1_P1": 0,
            "EQ1_S4": 1,
            "EQ1_S3": 2,
        }
        types = {
            "EQ1_P1": "AMR",
            "EQ1_S4": "OHT",
            "EQ1_S3": "OHT",
        }
        return channels, types
    if profile == "other":
        channels = {
            "EQ1_S1": 0,
            "EQ1_S2": 1,
            "EQ1_S7": 2,
            "EQ1_S8": 3,
        }
        types = {k: "OHT" for k in channels.keys()}
        return channels, types
    raise ValueError(f"Unknown profile: {profile}")


def build_pio_points(channels: Dict[str, int], types: Dict[str, str]) -> Tuple[List[IOPoint], List[IOPoint]]:
    in_list_amr = [
        ("VALID",  PIOBits.VALID,  "PIO 통신 시작(AMR->Port)"),
        ("CS_0",   PIOBits.CS_0,   "동작중(AMR->Port)"),
        ("CS_1",   PIOBits.CS_1,   "미사용(항상0)"),
        ("TR_REQ", PIOBits.TR_REQ, "Transfer 요청(AMR->Port)"),
        ("BUSY",   PIOBits.BUSY,   "Transfer 진행(AMR->Port)"),
        ("COMPT",  PIOBits.COMPT,  "작업 완료(AMR->Port)"),
    ]
    out_list_amr = [
        ("L_REQ",   PIOBits.L_REQ,   "Load 요청(Port->AMR)"),
        ("U_REQ",   PIOBits.U_REQ,   "Unload 요청(Port->AMR)"),
        ("READY",   PIOBits.READY,   "진입/이송 허가(Port->AMR)"),
        ("HO_AVBL", PIOBits.HO_AVBL, "정상/작업가능(상시ON)"),
        ("ES",      PIOBits.ES,      "Vehicle Stop(요청 시 0)"),
    ]
    in_list_oht = [
        ("VALID",  PIOBits.VALID,  "PIO 통신 시작(OHT->Port)"),
        ("CS_0",   PIOBits.CS_0,   "동작중(OHT->Port)"),
        ("CS_1",   PIOBits.CS_1,   "미사용(항상0)"),
        ("TR_REQ", PIOBits.TR_REQ, "Transfer 요청(OHT->Port)"),
        ("BUSY",   PIOBits.BUSY,   "Transfer 진행(OHT->Port)"),
        ("COMPT",  PIOBits.COMPT,  "작업 완료(OHT->Port)"),
    ]
    out_list_oht = [
        ("L_REQ",   PIOBits.L_REQ,   "Load 요청(Port->OHT)"),
        ("U_REQ",   PIOBits.U_REQ,   "Unload 요청(Port->OHT)"),
        ("READY",   PIOBits.READY,   "진입/이송 허가(Port->OHT)"),
        ("HO_AVBL", PIOBits.HO_AVBL, "정상/작업가능(상시ON)"),
        ("ES",      PIOBits.ES,      "Vehicle Stop(요청 시 0)"),
    ]

    in_points: List[IOPoint] = []
    out_points: List[IOPoint] = []
    for st, byte in channels.items():
        use_amr = types.get(st) == "AMR"
        in_list = in_list_amr if use_amr else in_list_oht
        out_list = out_list_amr if use_amr else out_list_oht

        for sig, bit, desc in in_list:
            in_points.append(
                IOPoint(
                    tag=f"{st}_{sig}",
                    addr=f"PIO.{st}.IN.{sig}",
                    desc=desc,
                    direction="IN",
                    byte_index=byte,
                    bit_index=bit,
                )
            )
        for sig, bit, desc in out_list:
            out_points.append(
                IOPoint(
                    tag=f"{st}_{sig}",
                    addr=f"PIO.{st}.OUT.{sig}",
                    desc=desc,
                    direction="OUT",
                    byte_index=byte,
                    bit_index=bit,
                )
            )
    return in_points, out_points


class BackendClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port

    def _request(self, cmd: str, params: Optional[Dict] = None) -> Dict:
        payload = {
            "id": int(time.time() * 1000),
            "cmd": cmd,
            "params": params or {},
        }
        raw = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")

        with socket.create_connection((self.host, self.port), timeout=1.0) as s:
            s.sendall(raw)
            data = b""
            while not data.endswith(b"\n"):
                chunk = s.recv(4096)
                if not chunk:
                    break
                data += chunk
        if not data:
            raise RuntimeError("no response")
        resp = json.loads(data.decode("utf-8").strip())
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error", "request failed"))
        return resp.get("data", {})

    def connect(self):
        return self._request("connect")

    def disconnect(self):
        return self._request("disconnect")

    def get_snapshot(self, since: int = 0) -> Dict:
        return self._request("get_snapshot", {"since": since})

    def set_mode(self, mode: str):
        return self._request("set_mode", {"mode": mode})

    def toggle_out_bit(self, byte: int, bit: int, force: bool = True):
        return self._request("toggle_out_bit", {"byte": byte, "bit": bit, "force": force})

    def set_out_bit(self, byte: int, bit: int, value: int, force: bool = True):
        return self._request("set_out_bit", {"byte": byte, "bit": bit, "value": value, "force": force})

    def clear_forces(self):
        return self._request("clear_forces")

    def reset_all(self):
        return self._request("reset_all")

    def pio_start(self, channel: str, action: str):
        return self._request("pio_start", {"channel": channel, "action": action})


class PioTab(QWidget):
    def __init__(self, profile: str, host: str, port: int):
        super().__init__()
        self.profile = profile
        self.client = BackendClient(host, port)

        self.PIO_CHANNEL, self.channel_types = build_channels(profile)
        self.in_points, self.out_points = build_pio_points(self.PIO_CHANNEL, self.channel_types)
        self.num_bytes = max([p.byte_index for p in (self.in_points + self.out_points)] + [0]) + 1

        self.last_changed_ts: Dict[str, str] = {}
        self.forced_out: Dict[Tuple[int, int], int] = {}
        self.in_map: Dict[Tuple[int, int], str] = {(p.byte_index, p.bit_index): p.tag for p in self.in_points}
        self.out_map: Dict[Tuple[int, int], str] = {(p.byte_index, p.bit_index): p.tag for p in self.out_points}

        self.mode_auto = True
        self.connected = False
        self.sim_fallback = False

        self.last_log_seq = 0

        self._init_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_cycle)
        self.timer.start(100)

    def _init_ui(self):
        self.setStyleSheet("""
            QWidget { background: #1e1f22; }
            QLabel { color: #d6d6d6; }
            QGroupBox { color: #d6d6d6; border: 1px solid #3a3b3d; border-radius: 8px; margin-top: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }
            QLineEdit, QSpinBox { background: #2b2d30; color: #e6e6e6; border: 1px solid #3a3b3d; border-radius: 6px; padding: 6px; }
            QPushButton { background: #2b2d30; color: #e6e6e6; border: 1px solid #3a3b3d; border-radius: 8px; padding: 8px 12px; }
            QPushButton:hover { background: #32343a; }
            QPushButton:disabled { color: #777; }
            QPlainTextEdit { background: #111214; color: #9ef19e; border: 1px solid #3a3b3d; border-radius: 8px; }
            QTableWidget { background: #1b1c1f; color: #e6e6e6; gridline-color: #3a3b3d; }
            QHeaderView::section { background: #2b2d30; color: #d6d6d6; border: 1px solid #3a3b3d; padding: 6px; }
        """)

        root = QVBoxLayout(self)

        # ===== Top Bar =====
        top = QHBoxLayout()
        root.addLayout(top)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.on_connect_clicked)
        top.addWidget(self.btn_connect)

        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("color:#ff6b6b; font-weight:bold;")
        top.addWidget(self.lbl_status)

        top.addSpacing(20)
        top.addWidget(QLabel("Refresh (ms):"))
        self.spin_refresh = QSpinBox()
        self.spin_refresh.setRange(50, 2000)
        self.spin_refresh.setValue(100)
        self.spin_refresh.valueChanged.connect(self.on_refresh_changed)
        top.addWidget(self.spin_refresh)

        top.addSpacing(20)
        top.addWidget(QLabel("Mode:"))

        self.mode_group = QButtonGroup(self)
        self.rb_auto = QRadioButton("AUTO")
        self.rb_manual = QRadioButton("MANUAL")
        self.rb_auto.setChecked(True)
        self.mode_group.addButton(self.rb_auto)
        self.mode_group.addButton(self.rb_manual)
        self.rb_auto.toggled.connect(self.on_mode_changed)
        self.rb_manual.toggled.connect(self.on_mode_changed)
        top.addWidget(self.rb_auto)
        top.addWidget(self.rb_manual)

        top.addStretch(1)

        self.btn_clear_forces = QPushButton("Clear Forces")
        self.btn_clear_forces.clicked.connect(self.clear_forces)
        top.addWidget(self.btn_clear_forces)

        self.btn_reset_all = QPushButton("RESET ALL (Outputs OFF)")
        self.btn_reset_all.clicked.connect(self.reset_all_outputs)
        top.addWidget(self.btn_reset_all)

        # ===== Main Grid =====
        mid = QHBoxLayout()
        root.addLayout(mid, 1)

        # -- Inputs
        gb_in = QGroupBox("Inputs")
        mid.addWidget(gb_in, 1)
        vin = QVBoxLayout(gb_in)

        self.in_filter = QLineEdit()
        self.in_filter.setPlaceholderText("filter(tag/desc/addr)")
        self.in_filter.textChanged.connect(self.apply_filters)
        vin.addWidget(self.in_filter)

        self.tbl_in = QTableWidget()
        self.tbl_in.setColumnCount(5)
        self.tbl_in.setHorizontalHeaderLabels(["Tag", "Addr", "Val", "Updated", "Desc"])
        self.tbl_in.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_in.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl_in.setRowCount(len(self.in_points))
        vin.addWidget(self.tbl_in, 1)

        # -- Outputs
        gb_out = QGroupBox("Outputs (Force: multi + override)")
        mid.addWidget(gb_out, 1)
        vout = QVBoxLayout(gb_out)

        self.out_filter = QLineEdit()
        self.out_filter.setPlaceholderText("filter(tag/desc/addr)")
        self.out_filter.textChanged.connect(self.apply_filters)
        vout.addWidget(self.out_filter)

        self.tbl_out = QTableWidget()
        self.tbl_out.setColumnCount(6)
        self.tbl_out.setHorizontalHeaderLabels(["Tag", "Addr", "Val", "Updated", "Desc", "Force"])
        self.tbl_out.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_out.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tbl_out.setRowCount(len(self.out_points))
        vout.addWidget(self.tbl_out, 1)

        # ===== Raw 8x8 Matrix =====
        mat = QHBoxLayout()
        root.addLayout(mat, 0)

        gb_mat = QGroupBox("Raw IO Matrix (byte 0~7 × bit 0~7)  —  OUT 클릭=토글+Force(유지)")
        mat.addWidget(gb_mat, 1)
        vmat = QHBoxLayout(gb_mat)

        self.tbl_in_mat = QTableWidget()
        self.tbl_in_mat.setRowCount(8)
        self.tbl_in_mat.setColumnCount(8)
        self.tbl_in_mat.setHorizontalHeaderLabels([str(i) for i in range(8)])
        self.tbl_in_mat.setVerticalHeaderLabels([str(i) for i in range(8)])
        self.tbl_in_mat.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_in_mat.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl_in_mat.setSelectionBehavior(QAbstractItemView.SelectItems)
        vmat.addWidget(self.tbl_in_mat, 1)

        self.tbl_out_mat = QTableWidget()
        self.tbl_out_mat.setRowCount(8)
        self.tbl_out_mat.setColumnCount(8)
        self.tbl_out_mat.setHorizontalHeaderLabels([str(i) for i in range(8)])
        self.tbl_out_mat.setVerticalHeaderLabels([str(i) for i in range(8)])
        self.tbl_out_mat.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_out_mat.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl_out_mat.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.tbl_out_mat.cellClicked.connect(self.on_out_matrix_clicked)
        vmat.addWidget(self.tbl_out_mat, 1)

        f = QFont("Consolas", 10)
        for r in range(8):
            for c in range(8):
                it1 = QTableWidgetItem("")
                it1.setFont(f)
                it1.setTextAlignment(Qt.AlignCenter)
                self.tbl_in_mat.setItem(r, c, it1)

                it2 = QTableWidgetItem("")
                it2.setFont(f)
                it2.setTextAlignment(Qt.AlignCenter)
                self.tbl_out_mat.setItem(r, c, it2)

        self.tbl_in_mat.resizeColumnsToContents()
        self.tbl_out_mat.resizeColumnsToContents()

        # ===== Bottom: Quick + Log =====
        bottom = QHBoxLayout()
        root.addLayout(bottom, 0)

        gb_quick = QGroupBox("PIO Quick (Port)")
        bottom.addWidget(gb_quick, 0)
        ql = QGridLayout(gb_quick)

        r = 0
        ql.addWidget(QLabel("Channel"), r, 0)
        ql.addWidget(QLabel("LOAD"), r, 1)
        ql.addWidget(QLabel("UNLOAD"), r, 2)
        r += 1

        self.btn_load: Dict[str, QPushButton] = {}
        self.btn_unload: Dict[str, QPushButton] = {}
        for st in self.PIO_CHANNEL.keys():
            ql.addWidget(QLabel(st), r, 0)
            b1 = QPushButton("Start")
            b2 = QPushButton("Start")
            b1.clicked.connect(lambda checked=False, s=st: self.pio_start(s, "LOAD"))
            b2.clicked.connect(lambda checked=False, s=st: self.pio_start(s, "UNLOAD"))
            self.btn_load[st] = b1
            self.btn_unload[st] = b2
            ql.addWidget(b1, r, 1)
            ql.addWidget(b2, r, 2)
            r += 1

        gb_log = QGroupBox("Alarm / Log")
        bottom.addWidget(gb_log, 1)
        vlog = QVBoxLayout(gb_log)
        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(5000)
        vlog.addWidget(self.txt_log)

        self._init_table_rows()

    def _init_table_rows(self):
        font = QFont("Consolas", 10)

        for i, p in enumerate(self.in_points):
            for c, text in enumerate([p.tag, p.addr, "OFF", "-", p.desc]):
                it = QTableWidgetItem(text)
                it.setFont(font)
                self.tbl_in.setItem(i, c, it)

        for i, p in enumerate(self.out_points):
            for c, text in enumerate([p.tag, p.addr, "OFF", "-", p.desc]):
                it = QTableWidgetItem(text)
                it.setFont(font)
                self.tbl_out.setItem(i, c, it)

            btn = QPushButton("Force")
            btn.clicked.connect(lambda checked=False, row=i: self.on_force_clicked(row))
            self.tbl_out.setCellWidget(i, 5, btn)

        self.tbl_in.resizeColumnsToContents()
        self.tbl_out.resizeColumnsToContents()

    # -------------------------
    # UI Actions
    # -------------------------
    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.txt_log.appendPlainText(f"[{ts}] {msg}")

    def on_connect_clicked(self):
        try:
            if self.connected:
                self.client.disconnect()
            else:
                self.client.connect()
        except Exception as e:
            self.log(f"connect error: {e}")

    def on_refresh_changed(self, v: int):
        self.timer.setInterval(int(v))

    def on_mode_changed(self):
        mode = "AUTO" if self.rb_auto.isChecked() else "MANUAL"
        try:
            self.client.set_mode(mode)
        except Exception as e:
            self.log(f"mode error: {e}")

    def apply_filters(self):
        tin = self.in_filter.text().strip().lower()
        tout = self.out_filter.text().strip().lower()

        for r, p in enumerate(self.in_points):
            hay = f"{p.tag} {p.addr} {p.desc}".lower()
            self.tbl_in.setRowHidden(r, bool(tin) and (tin not in hay))

        for r, p in enumerate(self.out_points):
            hay = f"{p.tag} {p.addr} {p.desc}".lower()
            self.tbl_out.setRowHidden(r, bool(tout) and (tout not in hay))

    def clear_forces(self):
        try:
            self.client.clear_forces()
        except Exception as e:
            self.log(f"clear forces error: {e}")

    def reset_all_outputs(self):
        try:
            self.client.reset_all()
        except Exception as e:
            self.log(f"reset error: {e}")

    def pio_start(self, station: str, mode: str):
        try:
            self.client.pio_start(station, mode)
        except Exception as e:
            self.log(f"pio start error: {e}")

    # Tag Grid Force: 토글 + override 유지
    def on_force_clicked(self, row: int):
        p = self.out_points[row]
        try:
            self.client.toggle_out_bit(p.byte_index, p.bit_index, True)
        except Exception as e:
            self.log(f"force error: {e}")

    # Raw OUT Matrix 클릭: 토글 + override 유지
    def on_out_matrix_clicked(self, row: int, col: int):
        try:
            self.client.toggle_out_bit(row, col, True)
        except Exception as e:
            self.log(f"matrix force error: {e}")

    # -------------------------
    # Refresh loop
    # -------------------------
    def refresh_cycle(self):
        try:
            snap = self.client.get_snapshot(self.last_log_seq)
        except Exception:
            self._set_status(False, False)
            return

        self.connected = bool(snap.get("connected", False))
        self.sim_fallback = bool(snap.get("sim_fallback", False))
        self._set_status(self.connected, self.sim_fallback)

        # mode sync
        mode = snap.get("mode", "AUTO")
        self.rb_auto.blockSignals(True)
        self.rb_manual.blockSignals(True)
        self.rb_auto.setChecked(mode == "AUTO")
        self.rb_manual.setChecked(mode != "AUTO")
        self.rb_auto.blockSignals(False)
        self.rb_manual.blockSignals(False)

        # forced map
        self.forced_out = {(f["byte"], f["bit"]): f["value"] for f in snap.get("forced", [])}

        inb = snap.get("in_bytes", [])
        outb = snap.get("out_bytes", [])

        # UI update
        self._update_table(self.tbl_in, self.in_points, inb, is_output=False)
        self._update_table(self.tbl_out, self.out_points, outb, is_output=True)
        self._update_matrix(self.tbl_in_mat, inb, self.in_map, is_output=False)
        self._update_matrix(self.tbl_out_mat, outb, self.out_map, is_output=True)

        # logs
        for item in snap.get("log_tail", []):
            self.txt_log.appendPlainText(item["msg"])
            self.last_log_seq = max(self.last_log_seq, int(item["seq"]))

    def _set_status(self, connected: bool, sim_fallback: bool):
        if connected:
            if sim_fallback:
                self.lbl_status.setText("SIM Connected")
                self.lbl_status.setStyleSheet("color:#ffd166; font-weight:bold;")
            else:
                self.lbl_status.setText("Connected")
                self.lbl_status.setStyleSheet("color:#7CFC98; font-weight:bold;")
            self.btn_connect.setText("Disconnect")
        else:
            self.lbl_status.setText("Disconnected")
            self.lbl_status.setStyleSheet("color:#ff6b6b; font-weight:bold;")
            self.btn_connect.setText("Connect")

    def _update_table(self, table: QTableWidget, points: List[IOPoint], bytes_: List[int], is_output: bool):
        on_bg = QColor("#1f8f4d")
        off_bg = QColor("#2b2d30")
        force_on_bg = QColor("#7a4b00")
        force_off_bg = QColor("#244a7a")

        padded = list(bytes_) + [0] * max(0, (max([p.byte_index for p in points] + [0]) + 1) - len(bytes_))

        for r, p in enumerate(points):
            val = (padded[p.byte_index] >> p.bit_index) & 1
            if p.invert:
                val = 0 if val else 1

            val_str = "ON" if val else "OFF"
            item_val = table.item(r, 2)
            item_upd = table.item(r, 3)

            prev_key = f"{p.tag}_prev"
            prev = getattr(self, prev_key, None)
            if prev is None or prev != val:
                setattr(self, prev_key, val)
                now = time.strftime("%H:%M:%S")
                self.last_changed_ts[p.tag] = now
                item_upd.setText(now)

            if p.tag in self.last_changed_ts and item_upd.text() == "-":
                item_upd.setText(self.last_changed_ts[p.tag])

            item_val.setText(val_str)

            base_bg = on_bg if val else off_bg
            if is_output and (p.byte_index, p.bit_index) in self.forced_out:
                fv = self.forced_out[(p.byte_index, p.bit_index)]
                base_bg = force_on_bg if fv == 1 else force_off_bg

            for c in range(0, 5):
                it = table.item(r, c)
                if it is not None:
                    it.setBackground(base_bg)

    def _update_matrix(self, table: QTableWidget, bytes_: List[int],
                       mp: Dict[Tuple[int, int], str], is_output: bool):
        on_bg = QColor("#1f8f4d")
        off_bg = QColor("#2b2d30")
        force_on_bg = QColor("#7a4b00")
        force_off_bg = QColor("#244a7a")

        padded = list(bytes_) + [0] * max(0, 8 - len(bytes_))

        for byte in range(8):
            bval = padded[byte]
            for bit in range(8):
                v = (bval >> bit) & 1
                it = table.item(byte, bit)
                if it is None:
                    it = QTableWidgetItem("")
                    it.setTextAlignment(Qt.AlignCenter)
                    table.setItem(byte, bit, it)

                it.setText("1" if v else "0")
                tag = mp.get((byte, bit), "")
                it.setToolTip(tag if tag else "")

                base_bg = on_bg if v else off_bg
                if is_output and (byte, bit) in self.forced_out:
                    fv = self.forced_out[(byte, bit)]
                    base_bg = force_on_bg if fv == 1 else force_off_bg

                it.setBackground(base_bg)


class MainWindow(QMainWindow):
    def __init__(self, host: str, main_port: int, other_port: int):
        super().__init__()
        self.setWindowTitle("EQ1 Controller (Dual)")
        self.resize(1800, 1000)

        tabs = QTabWidget()
        tabs.addTab(PioTab("main", host, main_port), "MAIN (P1/S4/S3)")
        tabs.addTab(PioTab("other", host, other_port), "OTHER (S1/S2/S7/S8)")
        self.setCentralWidget(tabs)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--main-port", type=int, default=9011)
    parser.add_argument("--other-port", type=int, default=9012)
    args = parser.parse_args()

    app = QApplication(sys.argv)
    w = MainWindow(args.host, args.main_port, args.other_port)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

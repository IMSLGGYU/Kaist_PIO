# -*- coding: utf-8 -*-
"""
EQ2 IO Grid UI (PySide6)
- Tag 기반 Inputs/Outputs GRID
- Outputs Force: 여러개 동시 가능 (force override로 유지)
- PIO(Port side) 2채널(EQ3_P1, EQ3_P2) 상태머신(LOAD/UNLOAD)
- Raw 8x8 Matrix (byte 0~7 × bit 0~7) 추가:
  - OUT 매트릭스 셀 클릭 시 토글 + Force(유지)

요구사항 반영:
1) MANUAL -> AUTO 전환 시: 강제(Force) 해제 + PIO 출력 정상 복구(HO_AVBL/ES=1, READY/L/U=0)
2) PIO TIMEOUT 시: ES, HO_AVBL 제외하고 0으로 리셋 (=> HO_AVBL/ES는 1 유지)
"""

import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

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

# ---- PySide6
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QPushButton, QLabel, QLineEdit, QTableWidget, QTableWidgetItem,
    QAbstractItemView, QSpinBox, QPlainTextEdit, QButtonGroup, QRadioButton
)

INFINITE = int(0xFFFFFFFF)


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
# WMX3 IO Driver
# =========================
class WMX3IoDriver:
    def __init__(self, device_name: str = "EQ2_IO_Grid"):
        self.device_name = device_name
        self.wmx = None
        self.io = None
        self.connected = False

    def connect(self) -> Tuple[bool, str]:
        if WMX3Api is None or Io is None:
            return False, "WMX3ApiPython import 실패(시뮬레이션 모드 필요)"
        try:
            self.wmx = WMX3Api()
            self.io = Io(self.wmx)
            self.wmx.CreateDevice(r"C:\Program Files\SoftServo\WMX3\\",
                                  DeviceType.DeviceTypeNormal, INFINITE)
            self.wmx.SetDeviceName(self.device_name)
            ret = self.wmx.StartCommunication(INFINITE)
            if ret != 0:
                return False, f"StartCommunication 실패(ret={ret})"
            self.connected = True
            return True, "Connected"
        except Exception as e:
            self.connected = False
            return False, f"Connect 예외: {e}"

    def disconnect(self) -> None:
        try:
            if self.wmx:
                self.wmx.StopCommunication(INFINITE)
        except Exception:
            pass
        self.connected = False
        self.wmx = None
        self.io = None

    def get_in_bytes(self, start: int, count: int) -> List[int]:
        if not self.connected:
            return [0] * count
        ret, data = self.io.GetInBytes(start, count)
        if ret == 0 and data:
            return list(data)
        return [0] * count

    def get_out_bytes(self, start: int, count: int) -> List[int]:
        if not self.connected:
            return [0] * count
        ret, data = self.io.GetOutBytes(start, count)
        if ret == 0 and data:
            return list(data)
        return [0] * count

    def set_out_bit(self, byte_index: int, bit_index: int, value: int) -> None:
        if not self.connected:
            return
        self.io.SetOutBit(byte_index, bit_index, int(value))

    def get_out_bit(self, byte_index: int, bit_index: int) -> int:
        if not self.connected:
            return 0
        ret, v = self.io.GetOutBit(byte_index, bit_index)
        if ret == 0:
            return int(v)
        return 0


# =========================
# Simulator Driver
# =========================
class SimIoDriver(WMX3IoDriver):
    def __init__(self):
        super().__init__("SIM")
        self.connected = True
        self._in = [0] * 16
        self._out = [0] * 16

    def connect(self) -> Tuple[bool, str]:
        self.connected = True
        return True, "SIM Connected"

    def disconnect(self) -> None:
        self.connected = False

    def get_in_bytes(self, start: int, count: int) -> List[int]:
        return self._in[start:start+count]

    def get_out_bytes(self, start: int, count: int) -> List[int]:
        return self._out[start:start+count]

    def set_out_bit(self, byte_index: int, bit_index: int, value: int) -> None:
        if not self.connected:
            return
        mask = 1 << bit_index
        if value:
            self._out[byte_index] |= mask
        else:
            self._out[byte_index] &= ~mask

    def get_out_bit(self, byte_index: int, bit_index: int) -> int:
        if not self.connected:
            return 0
        return (self._out[byte_index] >> bit_index) & 1


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


# =========================
# PIO State Machine
# =========================
class PIOState:
    IDLE = "IDLE"
    WAIT_VALID = "WAIT_VALID"
    WAIT_TR_REQ = "WAIT_TR_REQ"
    WAIT_BUSY_ON = "WAIT_BUSY_ON"
    WAIT_BUSY_OFF = "WAIT_BUSY_OFF"
    WAIT_TR_OFF = "WAIT_TR_OFF"
    WAIT_COMPT_ON = "WAIT_COMPT_ON"
    WAIT_VALID_OFF = "WAIT_VALID_OFF"


class PIOPortMachine:
    def __init__(self, name: str, byte_index: int, driver: WMX3IoDriver, log_fn):
        self.name = name
        self.byte = byte_index
        self.drv = driver
        self.log = log_fn

        # timeouts (sec)
        self.TP1 = 240
        self.TP2 = 240
        self.TP5 = 240
        self.TP3 = 240
        self.TP4 = 240

        self.state = PIOState.IDLE
        self.mode: Optional[str] = None
        self.t0 = 0.0

        self.set_healthy_outputs()

    def set_healthy_outputs(self):
        # 정상상태 유지: HO_AVBL=1, ES=1
        self.drv.set_out_bit(self.byte, PIOBits.HO_AVBL, 1)
        self.drv.set_out_bit(self.byte, PIOBits.ES, 1)

    def reset_outputs(self):
        # 요청/레디 리셋 + 정상상태 복구
        self.drv.set_out_bit(self.byte, PIOBits.L_REQ, 0)
        self.drv.set_out_bit(self.byte, PIOBits.U_REQ, 0)
        self.drv.set_out_bit(self.byte, PIOBits.READY, 0)
        self.set_healthy_outputs()

    def _in_bit(self, in_bytes: List[int], bit: int) -> int:
        return int((in_bytes[self.byte] >> bit) & 1)

    def _timeout(self, limit: float) -> bool:
        return (time.time() - self.t0) > limit

    def start_load(self):
        if self.state != PIOState.IDLE:
            self.log(f"[{self.name}] busy: state={self.state}")
            return
        self.mode = "LOAD"
        self.state = PIOState.WAIT_VALID
        self.t0 = time.time()
        self.reset_outputs()
        self.log(f"[{self.name}] START LOAD -> WAIT_VALID")

    def start_unload(self):
        if self.state != PIOState.IDLE:
            self.log(f"[{self.name}] busy: state={self.state}")
            return
        self.mode = "UNLOAD"
        self.state = PIOState.WAIT_VALID
        self.t0 = time.time()
        self.reset_outputs()
        self.log(f"[{self.name}] START UNLOAD -> WAIT_VALID")

    def tick(self, in_bytes: List[int]) -> Optional[str]:
        if self.state == PIOState.IDLE:
            self.set_healthy_outputs()
            return None

        valid = self._in_bit(in_bytes, PIOBits.VALID)
        cs0   = self._in_bit(in_bytes, PIOBits.CS_0)
        tr    = self._in_bit(in_bytes, PIOBits.TR_REQ)
        busy  = self._in_bit(in_bytes, PIOBits.BUSY)
        compt = self._in_bit(in_bytes, PIOBits.COMPT)

        if self.state == PIOState.WAIT_VALID:
            if valid == 1 and cs0 == 1:
                if self.mode == "LOAD":
                    self.drv.set_out_bit(self.byte, PIOBits.L_REQ, 1)
                else:
                    self.drv.set_out_bit(self.byte, PIOBits.U_REQ, 1)
                self.state = PIOState.WAIT_TR_REQ
                self.t0 = time.time()
                self.log(f"[{self.name}] REQ ON -> WAIT_TR_REQ(TP1)")
            elif self._timeout(90):
                self.log(f"[{self.name}] WAIT_VALID timeout(30s) -> reset to IDLE")
                self.reset_outputs()
                self.state = PIOState.IDLE
                self.mode = None
            return None

        if self.state == PIOState.WAIT_TR_REQ:
            if tr == 1:
                self.drv.set_out_bit(self.byte, PIOBits.READY, 1)
                self.state = PIOState.WAIT_BUSY_ON
                self.t0 = time.time()
                self.log(f"[{self.name}] READY ON -> WAIT_BUSY_ON(TP2)")
            elif self._timeout(self.TP1):
                self.log(f"[{self.name}] TP1 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_BUSY_ON:
            if busy == 1:
                self.drv.set_out_bit(self.byte, PIOBits.L_REQ, 0)
                self.drv.set_out_bit(self.byte, PIOBits.U_REQ, 0)
                self.state = PIOState.WAIT_BUSY_OFF
                self.t0 = time.time()
                self.log(f"[{self.name}] BUSY ON, REQ OFF -> WAIT_BUSY_OFF(TP3)")
            elif self._timeout(self.TP2):
                self.log(f"[{self.name}] TP2 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_BUSY_OFF:
            if busy == 0:
                self.state = PIOState.WAIT_TR_OFF
                self.t0 = time.time()
                self.log(f"[{self.name}] BUSY OFF -> WAIT_TR_OFF(TP4)")
            elif self._timeout(self.TP3):
                self.log(f"[{self.name}] TP3 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_TR_OFF:
            if tr == 0:
                self.state = PIOState.WAIT_COMPT_ON
                self.t0 = time.time()
                self.log(f"[{self.name}] TR_REQ OFF -> WAIT_COMPT_ON(TP4)")
            elif self._timeout(self.TP4):
                self.log(f"[{self.name}] TP4 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_COMPT_ON:
            if compt == 1:
                self.drv.set_out_bit(self.byte, PIOBits.READY, 0)
                self.state = PIOState.WAIT_VALID_OFF
                self.t0 = time.time()
                self.log(f"[{self.name}] COMPT ON, READY OFF -> WAIT_VALID_OFF(TP5)")
            elif self._timeout(self.TP4):
                self.log(f"[{self.name}] TP4 timeout(wait COMPT) -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_VALID_OFF:
            if valid == 0:
                self.reset_outputs()
                self.state = PIOState.IDLE
                self.mode = None
                self.log(f"[{self.name}] DONE -> IDLE")
            elif self._timeout(self.TP5):
                self.log(f"[{self.name}] TP5 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        return None


# =========================
# Main UI
# =========================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EQ2 Testbed IO (Grid / PySide6)")
        self.resize(1700, 980)

        # ---- Driver
        self.driver: WMX3IoDriver = WMX3IoDriver()
        self.sim_fallback = False

        # ---- PIO 채널(EQ2는 2곳만)
        # ⚠️ 아래 byte index(0,1)는 실제 매핑에 맞게 수정해줘.
        self.PIO_CHANNEL: Dict[str, int] = {
            "EQ3_P1": 0,
            "EQ3_P2": 1,
        }

        # ---- IO points (PIO만 자동 생성)
        self.in_points: List[IOPoint] = []
        self.out_points: List[IOPoint] = []
        self._build_pio_points()

        self.num_bytes = max([p.byte_index for p in (self.in_points + self.out_points)] + [0]) + 1

        # ---- runtime cache
        self.last_changed_ts: Dict[str, str] = {}

        # ---- Force override: 여러개 동시 가능 (byte,bit)->0/1
        self.forced_out: Dict[Tuple[int, int], int] = {}

        # ---- tag lookup (tooltip)
        self.in_map: Dict[Tuple[int, int], str] = {(p.byte_index, p.bit_index): p.tag for p in self.in_points}
        self.out_map: Dict[Tuple[int, int], str] = {(p.byte_index, p.bit_index): p.tag for p in self.out_points}

        # ---- mode
        self.mode_auto = True

        # ---- PIO machines
        self.pio: Dict[str, PIOPortMachine] = {}
        for name, b in self.PIO_CHANNEL.items():
            self.pio[name] = PIOPortMachine(name=name, byte_index=b, driver=self.driver, log_fn=self.log)

        # ---- UI
        self._init_ui()

        # ---- timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_cycle)
        self.timer.start(100)

    def _build_pio_points(self):
        in_list = [
            ("VALID",  PIOBits.VALID,  "PIO 통신 시작(AMR->Port)"),
            ("CS_0",   PIOBits.CS_0,   "동작중(AMR->Port)"),
            ("CS_1",   PIOBits.CS_1,   "미사용(항상0)"),
            ("TR_REQ", PIOBits.TR_REQ, "Transfer 요청(AMR->Port)"),
            ("BUSY",   PIOBits.BUSY,   "Transfer 진행(AMR->Port)"),
            ("COMPT",  PIOBits.COMPT,  "작업 완료(AMR->Port)"),
        ]
        out_list = [
            ("L_REQ",   PIOBits.L_REQ,   "Load 요청(Port->AMR)"),
            ("U_REQ",   PIOBits.U_REQ,   "Unload 요청(Port->AMR)"),
            ("READY",   PIOBits.READY,   "진입/이송 허가(Port->AMR)"),
            ("HO_AVBL", PIOBits.HO_AVBL, "정상/작업가능(상시ON)"),
            ("ES",      PIOBits.ES,      "Vehicle Stop(요청 시 0)"),
        ]

        for st, byte in self.PIO_CHANNEL.items():
            for sig, bit, desc in in_list:
                self.in_points.append(
                    IOPoint(
                        tag=f"{st}_{sig}",
                        addr=f"PIO.{st}.IN.{sig}",
                        desc=desc,
                        direction="IN",
                        byte_index=byte,
                        bit_index=bit
                    )
                )
            for sig, bit, desc in out_list:
                self.out_points.append(
                    IOPoint(
                        tag=f"{st}_{sig}",
                        addr=f"PIO.{st}.OUT.{sig}",
                        desc=desc,
                        direction="OUT",
                        byte_index=byte,
                        bit_index=bit
                    )
                )

    def _init_ui(self):
        # ---- dark-ish style
        self.setStyleSheet("""
            QMainWindow { background: #1e1f22; }
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

        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)

        # ===== Top Bar =====
        top = QHBoxLayout()
        main.addLayout(top)

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
        main.addLayout(mid, 1)

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
        main.addLayout(mat, 0)

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
        main.addLayout(bottom, 0)

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
        if self.driver.connected:
            self.driver.disconnect()
            self.lbl_status.setText("Disconnected")
            self.lbl_status.setStyleSheet("color:#ff6b6b; font-weight:bold;")
            self.btn_connect.setText("Connect")
            self.log("Disconnected")
            return

        ok, msg = self.driver.connect()
        if not ok:
            self.log(f"WMX connect failed: {msg}")
            self.log("Falling back to SIM driver")
            self.driver = SimIoDriver()
            self.sim_fallback = True
            for m in self.pio.values():
                m.drv = self.driver
            self.lbl_status.setText("SIM Connected")
            self.lbl_status.setStyleSheet("color:#ffd166; font-weight:bold;")
            self.btn_connect.setText("Disconnect")
        else:
            self.sim_fallback = False
            for m in self.pio.values():
                m.drv = self.driver
            self.lbl_status.setText("Connected")
            self.lbl_status.setStyleSheet("color:#7CFC98; font-weight:bold;")
            self.btn_connect.setText("Disconnect")
            self.log("Connected")

        # ✅ 연결 직후: 래치된 출력(READY 등) 제거 + 정상상태 복구
        self.reset_all_outputs()

    def on_refresh_changed(self, v: int):
        self.timer.setInterval(int(v))

    def on_mode_changed(self):
        prev = self.mode_auto
        self.mode_auto = self.rb_auto.isChecked()
        self.log(f"Mode -> {'AUTO' if self.mode_auto else 'MANUAL'}")

        # ✅ MANUAL -> AUTO 진입 처리: 강제 해제 + 정상상태 복구
        if (prev is False) and (self.mode_auto is True):
            self.enter_auto_mode()

    def enter_auto_mode(self):
        # force 제거
        self.forced_out.clear()

        # PIO 출력 복구 + 상태 IDLE
        for m in self.pio.values():
            m.state = PIOState.IDLE
            m.mode = None
            m.reset_outputs()

        self.log("ENTER AUTO: cleared forces + reset PIO outputs (READY/L/U=0, HO_AVBL/ES=1)")

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
        self.forced_out.clear()
        self.log("Clear Forces (override off)")

    def reset_all_outputs(self):
        """
        버튼 용: 전체 출력 OFF 후, PIO 정상상태 복구
        """
        # force 제거
        self.forced_out.clear()

        # 전체 OUT 0
        for p in self.out_points:
            self.driver.set_out_bit(p.byte_index, p.bit_index, 0)

        # PIO 상태 IDLE + 정상상태 복구(HO_AVBL/ES=1, READY/L/U=0)
        for m in self.pio.values():
            m.state = PIOState.IDLE
            m.mode = None
            m.reset_outputs()

        self.log("RESET ALL: outputs OFF then restore HO_AVBL/ES=1 (READY/L/U=0)")

    def reset_on_timeout(self):
        """
        요구사항: TIMEOUT 시 ES, HO_AVBL 제외하고 0
        - HO_AVBL/ES는 1 유지
        """
        # force 제거(타임아웃 후에도 강제값 남으면 안 됨)
        self.forced_out.clear()

        # PIO 2채널: READY/L/U=0 + HO_AVBL/ES=1
        for m in self.pio.values():
            m.state = PIOState.IDLE
            m.mode = None
            m.reset_outputs()

        self.log("TIMEOUT RESET: READY/L/U=0, HO_AVBL/ES=1 (forces cleared)")

    def pio_start(self, station: str, mode: str):
        # PIO 시작은 AUTO에서만
        if not self.mode_auto:
            self.log("PIO start blocked: MANUAL mode")
            return
        if station not in self.pio:
            return
        if mode == "LOAD":
            self.pio[station].start_load()
        else:
            self.pio[station].start_unload()

    # Tag Grid Force: 토글 + override 유지
    def on_force_clicked(self, row: int):
        if not self.driver.connected:
            return
        p = self.out_points[row]
        cur = self.driver.get_out_bit(p.byte_index, p.bit_index)
        newv = 0 if cur else 1
        self.driver.set_out_bit(p.byte_index, p.bit_index, newv)
        self.forced_out[(p.byte_index, p.bit_index)] = newv
        self.log(f"Force(Tag): {p.tag} -> {'ON' if newv else 'OFF'} (override)")

    # Raw OUT Matrix 클릭: 토글 + override 유지
    def on_out_matrix_clicked(self, row: int, col: int):
        if not self.driver.connected:
            return
        byte = row
        bit = col
        cur = self.driver.get_out_bit(byte, bit)
        newv = 0 if cur else 1
        self.driver.set_out_bit(byte, bit, newv)
        self.forced_out[(byte, bit)] = newv
        tag = self.out_map.get((byte, bit), "-")
        self.log(f"Force(Matrix): byte={byte} bit={bit} ({tag}) -> {'ON' if newv else 'OFF'} (override)")

    # -------------------------
    # Refresh loop
    # -------------------------
    def refresh_cycle(self):
        if not self.driver.connected:
            return

        inb = self.driver.get_in_bytes(0x00, self.num_bytes)
        outb = self.driver.get_out_bytes(0x00, self.num_bytes)

        # AUTO: PIO tick
        if self.mode_auto:
            for m in self.pio.values():
                res = m.tick(inb)
                if res == "TIMEOUT":
                    self.reset_on_timeout()
                    outb = self.driver.get_out_bytes(0x00, self.num_bytes)
                    break

        # Force override 적용 (AUTO/MANUAL 상관 없이)
        if self.forced_out:
            for (byte, bit), v in self.forced_out.items():
                self.driver.set_out_bit(byte, bit, int(v))
            outb = self.driver.get_out_bytes(0x00, self.num_bytes)

        # UI update
        self._update_table(self.tbl_in, self.in_points, inb, is_output=False)
        self._update_table(self.tbl_out, self.out_points, outb, is_output=True)
        self._update_matrix(self.tbl_in_mat, inb, self.in_map, is_output=False)
        self._update_matrix(self.tbl_out_mat, outb, self.out_map, is_output=True)

    def _update_table(self, table: QTableWidget, points: List[IOPoint], bytes_: List[int], is_output: bool):
        on_bg = QColor("#1f8f4d")
        off_bg = QColor("#2b2d30")
        force_on_bg = QColor("#7a4b00")
        force_off_bg = QColor("#244a7a")

        for r, p in enumerate(points):
            val = (bytes_[p.byte_index] >> p.bit_index) & 1
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


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

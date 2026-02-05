# -*- coding: utf-8 -*-
"""
EQ1 Final UI (PySide6)
- EQ1 Main + EQ1 Other 통합 UI
- 각 섹션 독립 연결/통신
- 확장 IO Grid (byte 0~20, bit 0~7) 토글 표시
"""

import sys
import time
import threading
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
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QPushButton, QLabel, QLineEdit, QTableWidget, QTableWidgetItem,
    QAbstractItemView, QSpinBox, QPlainTextEdit, QButtonGroup, QRadioButton,
    QComboBox
)

from roller import RollerController, RollerIOMap

INFINITE = int(0xFFFFFFFF)

# =========================
# PIO Channel Config (easy edit)
# =========================
PIO_TIMEOUTS = {
    "TP1": 60,
    "TP2": 60,
    "TP3": 60,
    "TP4": 60,
    "TP5": 60,
    "WAIT_VALID": 60,
    # ✅ NEW: 캐리어 엣지가 없을 때 REQ 자동 OFF 시간(초)
    "REQ_FALLBACK_OFF": 30,
    # ✅ NEW: transfer_to_amr 내부 클리어 대기 timeout(초)
    "TRANSFER_TIMEOUT_WAIT_CLEAR": 20,
    # ✅ NEW: transfer_to_amr 종료 후 post delay(초)
    "TRANSFER_POST_CLEAR_DELAY": 5.0,
}

MAIN_PIO_IN_CHANNELS = {
    "EQ1_S3": 0,
    "EQ1_S4": 1,
    "EQ1_P1": 2,
}
MAIN_PIO_OUT_CHANNELS = {
    "EQ1_S3": 0,
    "EQ1_S4": 1,
    "EQ1_P1": 2,
}
MAIN_PIO_KIND = {
    "EQ1_P1": "AMR",
    "EQ1_S3": "OHT",
    "EQ1_S4": "OHT",
}

OTHER_PIO_IN_CHANNELS = {
    "OHT_S1": 6,
    "AMR_S1": 7,
    "OHT_S2": 8,
    "AMR_S2": 9,
    "OHT_S7": 12,
    "AMR_S7": 13,
    "OHT_S8": 14,
    "AMR_S8": 15,
}
OTHER_PIO_OUT_CHANNELS = {
    "OHT_S1": 6,
    "AMR_S1": 7,
    "OHT_S2": 8,
    "AMR_S2": 9,
    "OHT_S7": 12,
    "AMR_S7": 13,
    "OHT_S8": 14,
    "AMR_S8": 15,
}

OTHER_PIO_KIND = {
    "OHT_S1": "OHT",
    "AMR_S1": "AMR",
    "OHT_S2": "OHT",
    "AMR_S2": "AMR",
    "OHT_S7": "OHT",
    "AMR_S7": "AMR",
    "OHT_S8": "OHT",
    "AMR_S8": "AMR",
}

# =========================
# Carrier Input (easy edit)
# Format: channel -> list of (byte, bit)
# =========================
CARRIER_INPUTS_MAIN = {
    "EQ1_S4": [(4, 4)],
    "EQ1_S3": [(4, 3)],
}

CARRIER_INPUTS_OTHER = {
    "OHT_S1": [(11, 0)],
    "OHT_S2": [(10, 0), (10, 1)],
    "OHT_S7": [(16, 0)],
    "OHT_S8": [(17, 0)],
    "AMR_S1": [(11, 0)],
    "AMR_S2": [(10, 0), (10, 1)],
    "AMR_S7": [(16, 0)],
    "AMR_S8": [(17, 0)],
}


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
    def __init__(self, device_name: str = "EQ1_IO_Grid"):
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
    def __init__(self, size: int = 32):
        super().__init__("SIM")
        self.connected = True
        self._in = [0] * size
        self._out = [0] * size

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
# IO Adapter for Roller (uses driver, no extra WMX3 connection)
# =========================
class DriverIoAdapter:
    def __init__(self, driver: WMX3IoDriver):
        self.driver = driver

    def set_driver(self, driver: WMX3IoDriver):
        self.driver = driver

    def SetOutBit(self, port_no: int, byte: int, bit: int, value: int):
        _ = port_no
        self.driver.set_out_bit(byte, bit, int(value))

    def GetInBit(self, port_no: int, byte: int, bit: int):
        _ = port_no
        inb = self.driver.get_in_bytes(0x00, byte + 1)
        if byte < 0 or byte >= len(inb):
            v = 0
        else:
            v = (inb[byte] >> bit) & 1
        return 0, int(v)

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
    def __init__(
        self,
        name: str,
        in_byte: int,
        out_byte: int,
        driver: WMX3IoDriver,
        log_fn,
        timeouts: Optional[Dict[str, float]] = None,
        roller: Optional[RollerController] = None,
        roller_iomap: Optional[RollerIOMap] = None,
        carrier_inputs: Optional[List[Tuple[int, int]]] = None,
    ):
        self.name = name
        self.in_byte = in_byte
        self.out_byte = out_byte
        self.drv = driver
        self.log = log_fn
        self.roller = roller
        self.roller_iomap = roller_iomap or RollerIOMap()
        self.carrier_inputs = carrier_inputs or []
        self._last_carrier: Optional[int] = None
        self._last_carrier_bits: Optional[List[int]] = None

        # timeouts (sec)
        t = timeouts or {}
        self.TP1 = t.get("TP1", 2)
        self.TP2 = t.get("TP2", 2)
        self.TP3 = t.get("TP3", 10)
        self.TP4 = t.get("TP4", 10)
        self.TP5 = t.get("TP5", 2)
        self.WAIT_VALID_TIMEOUT = t.get("WAIT_VALID", 30)

        # ✅ NEW: 캐리어 엣지 없을 때 REQ 자동 OFF
        self.REQ_FALLBACK_OFF = t.get("REQ_FALLBACK_OFF", 30)
        self._req_on_ts: Optional[float] = None
        self._req_off_done: bool = False

        # ✅ NEW: transfer_to_amr timeout 설정
        self.TRANSFER_TIMEOUT_WAIT_CLEAR = float(t.get("TRANSFER_TIMEOUT_WAIT_CLEAR", 20))
        self.TRANSFER_POST_CLEAR_DELAY = float(t.get("TRANSFER_POST_CLEAR_DELAY", 5.0))

        self.state = PIOState.IDLE
        self.mode: Optional[str] = None
        self.t0 = 0.0

        self.set_healthy_outputs()

        # Roller task flags (only for EQ1_P1)
        self._roller_lock = threading.Lock()
        self._roller_load_started = False
        self._roller_unload_started = False
        self._roller_transfer_started = False

        # Sequence tracking (11 steps)
        self.seq_steps = [
            "1. CS0+VALID ON",
            "2. L_REQ/U_REQ ON",
            "3. TR_REQ ON",
            "4. READY ON",
            "5. BUSY ON",
            "6. L_REQ/U_REQ OFF",
            "7. BUSY OFF",
            "8. TR_REQ OFF",
            "9. COMPT ON",
            "10. READY OFF",
            "11. VALID/COMPT/CS0 OFF",
        ]
        self.seq_flags = [False] * len(self.seq_steps)
        self.seq_index = 0

    def _roller_thread(self, fn, after_fn=None):
        def _run():
            try:
                ok = fn()
            except Exception as e:
                self.log(f"[{self.name}] Roller error: {e}")
                ok = False
            if after_fn:
                try:
                    after_fn(ok)
                except Exception as e:
                    self.log(f"[{self.name}] Roller after_fn error: {e}")
        t = threading.Thread(target=_run, daemon=True)
        t.start()

    def set_healthy_outputs(self):
        # 정상상태 유지: HO_AVBL=1, ES=1
        self.drv.set_out_bit(self.out_byte, PIOBits.HO_AVBL, 1)
        self.drv.set_out_bit(self.out_byte, PIOBits.ES, 1)

    def reset_outputs(self):
        # 요청/레디 리셋 + 정상상태 복구
        self.drv.set_out_bit(self.out_byte, PIOBits.L_REQ, 0)
        self.drv.set_out_bit(self.out_byte, PIOBits.U_REQ, 0)
        self.drv.set_out_bit(self.out_byte, PIOBits.READY, 0)
        self.set_healthy_outputs()

    def _in_bit(self, in_bytes: List[int], bit: int) -> int:
        return int((in_bytes[self.in_byte] >> bit) & 1)

    def _timeout(self, limit: float) -> bool:
        return (time.time() - self.t0) > limit

    def _seq_reset(self):
        self.seq_flags = [False] * len(self.seq_steps)
        self.seq_index = 0

    def _seq_mark(self, step_idx: int):
        if self.seq_index == step_idx:
            self.seq_flags[step_idx] = True
            self.seq_index += 1

    def seq_snapshot(self) -> List[bool]:
        return list(self.seq_flags)

    def stop(self):
        self.reset_outputs()
        self.state = PIOState.IDLE
        self.mode = None
        self._last_carrier = None
        self._last_carrier_bits = None
        self._roller_load_started = False
        self._roller_unload_started = False
        self._roller_transfer_started = False
        self._seq_reset()
        # ✅ NEW
        self._req_on_ts = None
        self._req_off_done = False
        self.log(f"[{self.name}] STOP -> IDLE")

    def _carrier_present(self, in_bytes: List[int]) -> Optional[int]:
        if not self.carrier_inputs:
            return None
        for byte, bit in self.carrier_inputs:
            if 0 <= byte < len(in_bytes):
                if ((in_bytes[byte] >> bit) & 1) == 1:
                    return 1
        return 0

    def _carrier_edge(self, in_bytes: List[int]) -> Optional[Tuple[bool, bool]]:
        if not self.carrier_inputs:
            return None
        current_bits: List[int] = []
        for byte, bit in self.carrier_inputs:
            if 0 <= byte < len(in_bytes):
                current_bits.append((in_bytes[byte] >> bit) & 1)
            else:
                current_bits.append(0)

        prev_bits = self._last_carrier_bits
        self._last_carrier_bits = list(current_bits)
        if prev_bits is None:
            return None

        rise = False
        fall = False
        for prev, cur in zip(prev_bits, current_bits):
            if prev == 0 and cur == 1:
                rise = True
            elif prev == 1 and cur == 0:
                fall = True

        if not rise and not fall:
            return None
        return (rise, fall)

    def start_load(self):
        if self.state != PIOState.IDLE:
            self.log(f"[{self.name}] busy: state={self.state}")
            return
        self.mode = "LOAD"
        self.state = PIOState.WAIT_VALID
        self.t0 = time.time()
        self.reset_outputs()
        self._last_carrier = None
        self._last_carrier_bits = None
        self._roller_load_started = False
        self._roller_transfer_started = False
        self._seq_reset()
        # ✅ NEW
        self._req_on_ts = None
        self._req_off_done = False
        self.log(f"[{self.name}] START LOAD -> WAIT_VALID")

    def start_unload(self):
        if self.state != PIOState.IDLE:
            self.log(f"[{self.name}] busy: state={self.state}")
            return
        self.mode = "UNLOAD"
        self.state = PIOState.WAIT_VALID
        self.t0 = time.time()
        self.reset_outputs()
        self._last_carrier = None
        self._last_carrier_bits = None
        self._roller_unload_started = False
        self._roller_transfer_started = False
        self._seq_reset()
        # ✅ NEW
        self._req_on_ts = None
        self._req_off_done = False
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
        _carrier = self._carrier_present(in_bytes)
        carrier_edge = self._carrier_edge(in_bytes)

        # ✅ REQ OFF rule:
        # 1순위) 캐리어 입력 변화(엣지)가 있으면 즉시 OFF
        # 2순위) 엣지가 없으면 REQ ON 이후 REQ_FALLBACK_OFF(기본 30초) 경과 시 OFF
        now = time.time()
        if self.mode in ("LOAD", "UNLOAD") and (self._req_on_ts is not None) and (not self._req_off_done):
            # EQ1_P1: UNLOAD는 transfer_to_amr 종료 시점에만 U_REQ OFF
            if self.name == "EQ1_P1" and self.mode == "UNLOAD":
                pass
            elif self.mode == "LOAD" and carrier_edge and carrier_edge[0]:
                self.drv.set_out_bit(self.out_byte, PIOBits.L_REQ, 0)
                self._req_off_done = True
                self._seq_mark(5)  # "L_REQ/U_REQ OFF"
            elif self.mode == "UNLOAD" and carrier_edge and carrier_edge[1]:
                self.drv.set_out_bit(self.out_byte, PIOBits.U_REQ, 0)
                self._req_off_done = True
                self._seq_mark(5)
            elif (now - self._req_on_ts) >= float(self.REQ_FALLBACK_OFF):
                if self.mode == "LOAD":
                    self.drv.set_out_bit(self.out_byte, PIOBits.L_REQ, 0)
                else:
                    self.drv.set_out_bit(self.out_byte, PIOBits.U_REQ, 0)
                self._req_off_done = True
                self._seq_mark(5)

        if self.state == PIOState.WAIT_VALID:
            if valid == 1 and cs0 == 1:
                self._seq_mark(0)
                if self.mode == "LOAD":
                    self.drv.set_out_bit(self.out_byte, PIOBits.L_REQ, 1)
                    # ✅ NEW: REQ 타이머 시작(ON 시점)
                    self._req_on_ts = time.time()
                    self._req_off_done = False
                else:
                    self.drv.set_out_bit(self.out_byte, PIOBits.U_REQ, 1)
                    # ✅ NEW: REQ 타이머 시작(ON 시점)
                    self._req_on_ts = time.time()
                    self._req_off_done = False
                self._seq_mark(1)
                self.state = PIOState.WAIT_TR_REQ
                self.t0 = time.time()
                self.log(f"[{self.name}] REQ ON -> WAIT_TR_REQ(TP1)")
            return None

        if self.state == PIOState.WAIT_TR_REQ:
            if tr == 1:
                self._seq_mark(2)
                self.drv.set_out_bit(self.out_byte, PIOBits.READY, 1)
                self._seq_mark(3)

                # ✅ LOAD: READY ON 직후 롤러 진입 실행 (move_in)
                if self.mode == "LOAD" and self.roller is not None and (not getattr(self, "_roller_load_started", False)):
                    self._roller_load_started = True
                    self.log(f"[{self.name}] Roller move_in start (after READY ON)")
                    def _after_move_in(ok):
                        # move_in 종료 시점에 L_REQ OFF (요구사항)
                        self.drv.set_out_bit(self.out_byte, PIOBits.L_REQ, 0)
                        self._req_off_done = True
                        self._seq_mark(5)  # "L_REQ/U_REQ OFF"
                    self._roller_thread(lambda: self.roller.move_in_until_sensor(timeout_s=10), after_fn=_after_move_in)

                # ✅ EQ1_P1 UNLOAD: READY ON 직후 transfer_to_amr 실행, 종료 시 U_REQ OFF
                if self.name == "EQ1_P1" and self.mode == "UNLOAD" and self.roller and not self._roller_transfer_started:
                    self._roller_transfer_started = True
                    self.log(f"[{self.name}] transfer_to_amr start (after READY ON)")
                    def _after_transfer(ok):
                        self.drv.set_out_bit(self.out_byte, PIOBits.U_REQ, 0)
                        self._req_off_done = True
                        self._seq_mark(5)
                        self.log(f"[{self.name}] transfer_to_amr done={ok} -> U_REQ OFF")
                    self._roller_thread(
                        lambda: self.roller.transfer_to_amr(
                            post_clear_delay_s=self.TRANSFER_POST_CLEAR_DELAY,
                            timeout_wait_clear_s=self.TRANSFER_TIMEOUT_WAIT_CLEAR,
                            require_initial_sensor_1=True,
                        ),
                        after_fn=_after_transfer
                    )

                self.state = PIOState.WAIT_BUSY_ON
                self.t0 = time.time()
                self.log(f"[{self.name}] READY ON -> WAIT_BUSY_ON(TP2)")
            elif self._timeout(self.TP1):
                self.log(f"[{self.name}] TP1 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_BUSY_ON:
            if busy == 1:
                self._seq_mark(4)

                # ✅ REMOVED:
                # carrier_inputs 없으면 BUSY ON에서 즉시 REQ OFF 하던 로직 삭제
                # (REQ OFF는 오직 "캐리어 엣지 우선, 없으면 30초 fallback" 규칙으로만)

                # EQ1_P1: transfer_to_amr는 READY ON 시점에 시작하도록 변경
                self.state = PIOState.WAIT_BUSY_OFF
                self.t0 = time.time()
                self.log(f"[{self.name}] BUSY ON -> WAIT_BUSY_OFF(TP3)")
            elif self._timeout(self.TP2):
                self.log(f"[{self.name}] TP2 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_BUSY_OFF:
            if busy == 0:
                self._seq_mark(6)
                self.state = PIOState.WAIT_TR_OFF
                self.t0 = time.time()
                self.log(f"[{self.name}] BUSY OFF -> WAIT_TR_OFF(TP4)")
            elif self._timeout(self.TP3):
                self.log(f"[{self.name}] TP3 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_TR_OFF:
            if tr == 0:
                self._seq_mark(7)
                self.state = PIOState.WAIT_COMPT_ON
                self.t0 = time.time()
                self.log(f"[{self.name}] TR_REQ OFF -> WAIT_COMPT_ON(TP4)")
            elif self._timeout(self.TP4):
                self.log(f"[{self.name}] TP4 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_COMPT_ON:
            if compt == 1:
                self._seq_mark(8)
                self.drv.set_out_bit(self.out_byte, PIOBits.READY, 0)
                self._seq_mark(9)
                self.state = PIOState.WAIT_VALID_OFF
                self.t0 = time.time()
                self.log(f"[{self.name}] COMPT ON, READY OFF -> WAIT_VALID_OFF(TP5)")
            elif self._timeout(self.TP4):
                self.log(f"[{self.name}] TP4 timeout(wait COMPT) -> TIMEOUT")
                return "TIMEOUT"
            return None

        if self.state == PIOState.WAIT_VALID_OFF:
            if valid == 0 and cs0 == 0 and compt == 0:
                self._seq_mark(10)
                self.reset_outputs()
                self.state = PIOState.IDLE
                self.mode = None
                self._last_carrier = None
                self._last_carrier_bits = None
                self._roller_load_started = False
                self._roller_unload_started = False
                self._roller_transfer_started = False
                # ✅ NEW
                self._req_on_ts = None
                self._req_off_done = False
                self.log(f"[{self.name}] DONE -> IDLE")
            elif self._timeout(self.TP5):
                self.log(f"[{self.name}] TP5 timeout -> TIMEOUT")
                return "TIMEOUT"
            return None

        return None

# =========================
# IO Panel (Independent)
# =========================
class IOPanel(QWidget):
    log_signal = Signal(str)

    def __init__(
        self,
        panel_name: str,
        device_name: str,
        pio_in_channels: Dict[str, int],
        pio_out_channels: Dict[str, int],
        pio_kind_map: Dict[str, str],
        pio_timeouts: Dict[str, float],
        matrix_bytes: List[int],
        include_roller_io: bool = False,
        carrier_inputs: Optional[Dict[str, List[Tuple[int, int]]]] = None
    ):
        super().__init__()
        self.panel_name = panel_name
        self.device_name = device_name
        self.PIO_IN_CHANNEL = pio_in_channels
        self.PIO_OUT_CHANNEL = pio_out_channels
        self.PIO_KIND = pio_kind_map
        self.PIO_TIMEOUTS = pio_timeouts
        self.matrix_bytes = matrix_bytes
        self.include_roller_io = include_roller_io
        self.carrier_inputs = carrier_inputs or {}

        # ---- Driver
        self.driver: WMX3IoDriver = WMX3IoDriver(device_name=self.device_name)
        self.sim_fallback = False
        self._roller_io = DriverIoAdapter(self.driver)
        self.roller: Optional[RollerController] = None
        self.roller_iomap = RollerIOMap()

        # ---- IO points (PIO만 자동 생성)
        self.in_points: List[IOPoint] = []
        self.out_points: List[IOPoint] = []
        self._build_pio_points()
        if self.include_roller_io:
            self._add_roller_points()
        if self.carrier_inputs:
            self._add_carrier_points()

        max_byte = max(self.matrix_bytes + [p.byte_index for p in (self.in_points + self.out_points)] + [0])
        self.num_bytes = max_byte + 1

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
        for name, out_b in self.PIO_OUT_CHANNEL.items():
            in_b = self.PIO_IN_CHANNEL.get(name, out_b)
            self.pio[name] = PIOPortMachine(
                name=name,
                in_byte=in_b,
                out_byte=out_b,
                driver=self.driver,
                log_fn=self.log,
                timeouts=self.PIO_TIMEOUTS,
                roller=self.roller if name == "EQ1_P1" else None,
                roller_iomap=self.roller_iomap,
                carrier_inputs=self.carrier_inputs.get(name, [])
            )

        # ---- UI
        self._init_ui()

        # ---- log signal (thread-safe)
        self.log_signal.connect(self._append_log)

        # ---- timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_cycle)
        self.timer.start(100)

    def _build_pio_points(self):
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

        for st, byte in self.PIO_IN_CHANNEL.items():
            kind = self.PIO_KIND.get(st, "OHT")
            if kind == "AMR":
                in_list = in_list_amr
                out_list = out_list_amr
            else:
                in_list = in_list_oht
                out_list = out_list_oht

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
        for st, byte in self.PIO_OUT_CHANNEL.items():
            kind = self.PIO_KIND.get(st, "OHT")
            if kind == "AMR":
                out_list = out_list_amr
            else:
                out_list = out_list_oht

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

    def _add_roller_points(self):
        # Outputs
        self.out_points.append(
            IOPoint(
                tag="ROLLER_RUN",
                addr="ROLLER.OUT.RUN",
                desc="Roller RUN (ON/OFF)",
                direction="OUT",
                byte_index=self.roller_iomap.out_run.byte,
                bit_index=self.roller_iomap.out_run.bit,
            )
        )
        self.out_points.append(
            IOPoint(
                tag="ROLLER_DIR",
                addr="ROLLER.OUT.DIR",
                desc="Roller DIR (0=inward, 1=outward)",
                direction="OUT",
                byte_index=self.roller_iomap.out_dir.byte,
                bit_index=self.roller_iomap.out_dir.bit,
            )
        )
        # Inputs
        self.in_points.append(
            IOPoint(
                tag="ROLLER_INWARD_STOP",
                addr="ROLLER.IN.INWARD_STOP",
                desc="Inward stop sensor",
                direction="IN",
                byte_index=self.roller_iomap.in_inward_stop.byte,
                bit_index=self.roller_iomap.in_inward_stop.bit,
            )
        )
        self.in_points.append(
            IOPoint(
                tag="ROLLER_OUTWARD_STOP",
                addr="ROLLER.IN.OUTWARD_STOP",
                desc="Outward stop sensor",
                direction="IN",
                byte_index=self.roller_iomap.in_outward_stop.byte,
                bit_index=self.roller_iomap.in_outward_stop.bit,
            )
        )

    def _add_carrier_points(self):
        for st, pairs in self.carrier_inputs.items():
            for idx, (byte, bit) in enumerate(pairs, start=1):
                tag = f"{st}_CARRIER" if len(pairs) == 1 else f"{st}_CARRIER_{idx}"
                self.in_points.append(
                    IOPoint(
                        tag=tag,
                        addr=f"{st}.IN.CARRIER",
                        desc="Carrier 존재 여부(1=있음, 0=없음)",
                        direction="IN",
                        byte_index=byte,
                        bit_index=bit
                    )
                )

    def _init_ui(self):
        root = QVBoxLayout(self)

        title = QLabel(self.panel_name)
        title.setStyleSheet("color:#ffffff; font-weight:bold; font-size:16px;")
        root.addWidget(title)

        # ===== Top Bar =====
        self.section_top = QWidget()
        top = QHBoxLayout(self.section_top)
        top.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self.section_top)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.on_connect_clicked)
        top.addWidget(self.btn_connect)

        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("color:#ff6b6b; font-weight:bold;")
        top.addWidget(self.lbl_status)

        top.addSpacing(16)
        top.addWidget(QLabel("Refresh (ms):"))
        self.spin_refresh = QSpinBox()
        self.spin_refresh.setRange(50, 2000)
        self.spin_refresh.setValue(100)
        self.spin_refresh.valueChanged.connect(self.on_refresh_changed)
        top.addWidget(self.spin_refresh)

        top.addSpacing(16)
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

        # ===== Sequence View =====
        self.gb_seq = QGroupBox("Sequence Flow")
        self.gb_seq.setVisible(False)
        root.addWidget(self.gb_seq)
        vseq = QVBoxLayout(self.gb_seq)

        seq_top = QHBoxLayout()
        vseq.addLayout(seq_top)
        seq_top.addWidget(QLabel("Channel"))
        self.cb_seq_channel = QComboBox()
        for st in self.PIO_OUT_CHANNEL.keys():
            self.cb_seq_channel.addItem(st)
        seq_top.addWidget(self.cb_seq_channel)
        seq_top.addStretch(1)

        self.seq_labels = []
        for txt in self._seq_step_texts():
            lb = QLabel(txt)
            lb.setStyleSheet("padding:4px; border:1px solid #3a3b3d; border-radius:4px;")
            vseq.addWidget(lb)
            self.seq_labels.append(lb)

        # ===== Main Grid =====
        self.section_mid = QWidget()
        mid = QHBoxLayout(self.section_mid)
        mid.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self.section_mid, 1)

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

        # ===== IO Matrix Toggle =====
        toggle = QHBoxLayout()
        root.addLayout(toggle)
        self.btn_toggle_matrix = QPushButton("Show IO Grid (byte 0~30)")
        self.btn_toggle_matrix.clicked.connect(self.toggle_matrix)
        toggle.addWidget(self.btn_toggle_matrix)
        toggle.addStretch(1)
        self.btn_seq_view = QPushButton("Seq View")
        self.btn_seq_view.clicked.connect(self.toggle_sequence_view)
        toggle.addWidget(self.btn_seq_view)

        # ===== Raw Matrix =====
        self.gb_mat = QGroupBox("Raw IO Matrix (byte 0~30 × bit 0~7)  —  OUT 클릭=토글+Force(유지)")
        self.gb_mat.setVisible(False)
        root.addWidget(self.gb_mat, 1)
        vmat = QHBoxLayout(self.gb_mat)

        self.tbl_in_mat = QTableWidget()
        self.tbl_in_mat.setRowCount(len(self.matrix_bytes))
        self.tbl_in_mat.setColumnCount(8)
        self.tbl_in_mat.setHorizontalHeaderLabels([str(i) for i in range(8)])
        self.tbl_in_mat.setVerticalHeaderLabels([str(i) for i in self.matrix_bytes])
        self.tbl_in_mat.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_in_mat.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl_in_mat.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.tbl_in_mat.horizontalHeader().setDefaultSectionSize(36)
        self.tbl_in_mat.verticalHeader().setDefaultSectionSize(28)
        vmat.addWidget(self.tbl_in_mat, 1)

        self.tbl_out_mat = QTableWidget()
        self.tbl_out_mat.setRowCount(len(self.matrix_bytes))
        self.tbl_out_mat.setColumnCount(8)
        self.tbl_out_mat.setHorizontalHeaderLabels([str(i) for i in range(8)])
        self.tbl_out_mat.setVerticalHeaderLabels([str(i) for i in self.matrix_bytes])
        self.tbl_out_mat.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tbl_out_mat.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tbl_out_mat.setSelectionBehavior(QAbstractItemView.SelectItems)
        self.tbl_out_mat.cellClicked.connect(self.on_out_matrix_clicked)
        self.tbl_out_mat.horizontalHeader().setDefaultSectionSize(36)
        self.tbl_out_mat.verticalHeader().setDefaultSectionSize(28)
        vmat.addWidget(self.tbl_out_mat, 1)

        f = QFont("Consolas", 10)
        for r in range(len(self.matrix_bytes)):
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
        self.section_bottom = QWidget()
        bottom = QHBoxLayout(self.section_bottom)
        bottom.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self.section_bottom, 0)

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
        for st in self.PIO_OUT_CHANNEL.keys():
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
        self.log_signal.emit(f"[{ts}] {msg}")

    def _append_log(self, line: str):
        self.txt_log.appendPlainText(line)

    def toggle_matrix(self):
        visible = not self.gb_mat.isVisible()
        self.gb_mat.setVisible(visible)
        if hasattr(self, 'gb_seq'):
            self.gb_seq.setVisible(False)
        self.section_top.setVisible(not visible)
        self.section_mid.setVisible(not visible)
        self.section_bottom.setVisible(not visible)

        min_b = min(self.matrix_bytes) if self.matrix_bytes else 0
        max_b = max(self.matrix_bytes) if self.matrix_bytes else 0
        self.btn_toggle_matrix.setText(
            'Hide IO Grid' if visible else f'Show IO Grid (byte {min_b}~{max_b})'
        )

    def _seq_step_texts(self):
        return [
            '1. CS0+VALID ON',
            '2. L_REQ/U_REQ ON',
            '3. TR_REQ ON',
            '4. READY ON',
            '5. BUSY ON',
            '6. L_REQ/U_REQ OFF',
            '7. BUSY OFF',
            '8. TR_REQ OFF',
            '9. COMPT ON',
            '10. READY OFF',
            '11. VALID/COMPT/CS0 OFF',
        ]

    def toggle_sequence_view(self):
        visible = not self.gb_seq.isVisible()
        self.gb_seq.setVisible(visible)
        self.gb_mat.setVisible(False)
        self.section_top.setVisible(not visible)
        self.section_mid.setVisible(not visible)
        self.section_bottom.setVisible(not visible)
        min_b = min(self.matrix_bytes) if self.matrix_bytes else 0
        max_b = max(self.matrix_bytes) if self.matrix_bytes else 0
        self.btn_toggle_matrix.setText(f'Show IO Grid (byte {min_b}~{max_b})')

    def on_connect_clicked(self):
        if self.driver.connected:
            self.driver.disconnect()
            self.lbl_status.setText('Disconnected')
            self.lbl_status.setStyleSheet('color:#ff6b6b; font-weight:bold;')
            self.btn_connect.setText('Connect')
            self.log('Disconnected')
            return

        ok, msg = self.driver.connect()
        if not ok:
            self.log(f'WMX connect failed: {msg}')
            self.log('Falling back to SIM driver')
            self.driver = SimIoDriver(size=max(32, self.num_bytes))
            self.sim_fallback = True
            for m in self.pio.values():
                m.drv = self.driver
            self._roller_io.set_driver(self.driver)
            self.lbl_status.setText('SIM Connected')
            self.lbl_status.setStyleSheet('color:#ffd166; font-weight:bold;')
            self.btn_connect.setText('Disconnect')
        else:
            self.sim_fallback = False
            for m in self.pio.values():
                m.drv = self.driver
            self._roller_io.set_driver(self.driver)
            self.lbl_status.setText('Connected')
            self.lbl_status.setStyleSheet('color:#7CFC98; font-weight:bold;')
            self.btn_connect.setText('Disconnect')
            self.log('Connected')

        if self.include_roller_io and self.roller is None:
            self.roller = RollerController(io=self._roller_io)
            if 'EQ1_P1' in self.pio:
                self.pio['EQ1_P1'].roller = self.roller

        self.reset_all_outputs()

    def on_refresh_changed(self, v: int):
        self.timer.setInterval(int(v))

    def on_mode_changed(self):
        prev = self.mode_auto
        self.mode_auto = self.rb_auto.isChecked()
        self.log(f"Mode -> {'AUTO' if self.mode_auto else 'MANUAL'}")

        if (prev is False) and (self.mode_auto is True):
            self.enter_auto_mode()

    def enter_auto_mode(self):
        self.forced_out.clear()
        for m in self.pio.values():
            m.stop()
        self.log('ENTER AUTO: cleared forces + reset PIO outputs (READY/L/U=0, HO_AVBL/ES=1)')

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
        self.log('Clear Forces (override off)')

    def reset_all_outputs(self):
        self.forced_out.clear()
        for p in self.out_points:
            self.driver.set_out_bit(p.byte_index, p.bit_index, 0)
        for m in self.pio.values():
            m.stop()
        self.log('RESET ALL: outputs OFF then restore HO_AVBL/ES=1 (READY/L/U=0)')

    def reset_on_timeout(self):
        self.forced_out.clear()
        for m in self.pio.values():
            m.stop()
        self.log('TIMEOUT RESET: READY/L/U=0, HO_AVBL/ES=1 (forces cleared)')

    def pio_start(self, station: str, mode: str):
        if not self.mode_auto:
            self.log('PIO start blocked: MANUAL mode')
            return
        if station not in self.pio:
            return
        m = self.pio[station]
        if m.state != PIOState.IDLE:
            m.stop()
            return
        if mode == 'LOAD':
            m.start_load()
        else:
            m.start_unload()

    def on_force_clicked(self, row: int):
        if not self.driver.connected:
            return
        p = self.out_points[row]
        cur = self.driver.get_out_bit(p.byte_index, p.bit_index)
        newv = 0 if cur else 1
        self.driver.set_out_bit(p.byte_index, p.bit_index, newv)
        self.forced_out[(p.byte_index, p.bit_index)] = newv
        self.log(f"Force(Tag): {p.tag} -> {'ON' if newv else 'OFF'} (override)")

    def on_out_matrix_clicked(self, row: int, col: int):
        if not self.driver.connected:
            return
        byte = self.matrix_bytes[row]
        bit = col
        cur = self.driver.get_out_bit(byte, bit)
        newv = 0 if cur else 1
        self.driver.set_out_bit(byte, bit, newv)
        self.forced_out[(byte, bit)] = newv
        tag = self.out_map.get((byte, bit), '-')
        self.log(f"Force(Matrix): byte={byte} bit={bit} ({tag}) -> {'ON' if newv else 'OFF'} (override)")

    def refresh_cycle(self):
        if not self.driver.connected:
            return

        inb = self.driver.get_in_bytes(0x00, self.num_bytes)
        outb = self.driver.get_out_bytes(0x00, self.num_bytes)

        for m in self.pio.values():
            if self.mode_auto or m.state != PIOState.IDLE:
                res = m.tick(inb)
                if res == 'TIMEOUT':
                    self.reset_on_timeout()
                    outb = self.driver.get_out_bytes(0x00, self.num_bytes)
                    break

        if self.forced_out:
            for (byte, bit), v in self.forced_out.items():
                self.driver.set_out_bit(byte, bit, int(v))
            outb = self.driver.get_out_bytes(0x00, self.num_bytes)

        self._update_table(self.tbl_in, self.in_points, inb, is_output=False)
        self._update_table(self.tbl_out, self.out_points, outb, is_output=True)
        self._update_matrix(self.tbl_in_mat, inb, self.in_map, is_output=False)
        self._update_matrix(self.tbl_out_mat, outb, self.out_map, is_output=True)
        self._update_pio_buttons()
        self._update_sequence_view(inb, outb)

    def _update_pio_buttons(self):
        for st in self.PIO_OUT_CHANNEL.keys():
            m = self.pio.get(st)
            if not m:
                continue
            b_load = self.btn_load.get(st)
            b_unload = self.btn_unload.get(st)
            if not b_load or not b_unload:
                continue
            if m.state != PIOState.IDLE:
                if m.mode == 'LOAD':
                    b_load.setText('Stop')
                    b_load.setEnabled(True)
                    b_unload.setText('Start')
                    b_unload.setEnabled(False)
                else:
                    b_unload.setText('Stop')
                    b_unload.setEnabled(True)
                    b_load.setText('Start')
                    b_load.setEnabled(False)
            else:
                b_load.setText('Start')
                b_unload.setText('Start')
                b_load.setEnabled(True)
                b_unload.setEnabled(True)

    def _update_sequence_view(self, in_bytes, out_bytes):
        if not self.gb_seq.isVisible():
            return
        st = self.cb_seq_channel.currentText()
        m = self.pio.get(st)
        if not m:
            return
        flags = m.seq_snapshot()
        idx = m.seq_index
        for i, lb in enumerate(self.seq_labels):
            if i < len(flags) and flags[i]:
                lb.setStyleSheet('background:#1f8f4d; color:white; padding:4px; border:1px solid #3a3b3d; border-radius:4px;')
            elif m.state != PIOState.IDLE and i == idx:
                lb.setStyleSheet('background:#244a7a; color:white; padding:4px; border:1px solid #3a3b3d; border-radius:4px;')
            else:
                lb.setStyleSheet('background:#2b2d30; color:#d6d6d6; padding:4px; border:1px solid #3a3b3d; border-radius:4px;')

    def _update_table(self, table: QTableWidget, points: List[IOPoint], bytes_: List[int], is_output: bool):
        on_bg = QColor('#1f8f4d')
        off_bg = QColor('#2b2d30')
        force_on_bg = QColor('#7a4b00')
        force_off_bg = QColor('#244a7a')

        for r, p in enumerate(points):
            val = (bytes_[p.byte_index] >> p.bit_index) & 1
            if p.invert:
                val = 0 if val else 1

            val_str = 'ON' if val else 'OFF'
            item_val = table.item(r, 2)
            item_upd = table.item(r, 3)

            prev_key = f"{p.tag}_prev"
            prev = getattr(self, prev_key, None)
            if prev is None or prev != val:
                setattr(self, prev_key, val)
                now = time.strftime('%H:%M:%S')
                self.last_changed_ts[p.tag] = now
                item_upd.setText(now)

            if p.tag in self.last_changed_ts and item_upd.text() == '-':
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

    def _update_matrix(self, table: QTableWidget, bytes_: List[int], mp: Dict[Tuple[int, int], str], is_output: bool):
        on_bg = QColor('#1f8f4d')
        off_bg = QColor('#2b2d30')
        force_on_bg = QColor('#7a4b00')
        force_off_bg = QColor('#244a7a')

        padded = list(bytes_) + [0] * max(0, (max(self.matrix_bytes) + 1) - len(bytes_))

        for row, byte in enumerate(self.matrix_bytes):
            bval = padded[byte]
            for bit in range(8):
                v = (bval >> bit) & 1
                it = table.item(row, bit)
                if it is None:
                    it = QTableWidgetItem('')
                    it.setTextAlignment(Qt.AlignCenter)
                    table.setItem(row, bit, it)

                it.setText('1' if v else '0')
                tag = mp.get((byte, bit), '')
                it.setToolTip(tag if tag else '')

                base_bg = on_bg if v else off_bg
                if is_output and (byte, bit) in self.forced_out:
                    fv = self.forced_out[(byte, bit)]
                    base_bg = force_on_bg if fv == 1 else force_off_bg

                it.setBackground(base_bg)

# =========================
# Main Window
# =========================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EQ1 Final Testbed IO (Combined)")
        self.resize(1900, 1000)

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
        main = QHBoxLayout(root)

        matrix_bytes = list(range(0, 31))

        # ---- EQ1 Main panel
        self.panel_main = IOPanel(
            panel_name="EQ1 MAIN",
            device_name="EQ1_MAIN_IO",
            pio_in_channels=MAIN_PIO_IN_CHANNELS,
            pio_out_channels=MAIN_PIO_OUT_CHANNELS,
            pio_kind_map=MAIN_PIO_KIND,
            pio_timeouts=PIO_TIMEOUTS,
            matrix_bytes=matrix_bytes,
            include_roller_io=True,
            carrier_inputs=CARRIER_INPUTS_MAIN
        )

        # ---- EQ1 Other panel
        self.panel_other = IOPanel(
            panel_name="EQ1 OTHER",
            device_name="EQ1_OTHER_IO",
            pio_in_channels=OTHER_PIO_IN_CHANNELS,
            pio_out_channels=OTHER_PIO_OUT_CHANNELS,
            pio_kind_map=OTHER_PIO_KIND,
            pio_timeouts=PIO_TIMEOUTS,
            matrix_bytes=matrix_bytes,
            carrier_inputs=CARRIER_INPUTS_OTHER
        )

        main.addWidget(self.panel_main, 1)
        main.addWidget(self.panel_other, 1)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

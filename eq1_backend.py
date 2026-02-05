# -*- coding: utf-8 -*-
"""
EQ1 backend process (headless)
- JSON-over-TCP line protocol
- WMX3 IO + PIO state machine + force override
"""

import argparse
import json
import socket
import socketserver
import threading
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
        self.TP1 = 2
        self.TP2 = 2
        self.TP5 = 2
        self.TP3 = 10
        self.TP4 = 10

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
            elif self._timeout(30):
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


class BackendState:
    def __init__(self, profile: str, sim_mode: bool = False, auto_reconnect: bool = False):
        self.profile = profile
        self.sim_mode = sim_mode
        self.auto_reconnect = auto_reconnect

        self.channels, self.channel_types = build_channels(profile)
        self.in_points, self.out_points = build_pio_points(self.channels, self.channel_types)
        self.num_bytes = max([p.byte_index for p in (self.in_points + self.out_points)] + [0]) + 1

        self.driver = SimIoDriver() if sim_mode else WMX3IoDriver()
        self.sim_fallback = sim_mode
        self.connected = False
        self.mode_auto = True
        self.forced_out: Dict[Tuple[int, int], int] = {}

        self.pio: Dict[str, PIOPortMachine] = {}
        for name, b in self.channels.items():
            self.pio[name] = PIOPortMachine(name=name, byte_index=b, driver=self.driver, log_fn=self.log)

        self.last_in_bytes = [0] * self.num_bytes
        self.last_out_bytes = [0] * self.num_bytes

        self.log_seq = 0
        self.log_lines: List[Tuple[int, str]] = []
        self.log_limit = 2000

        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._io_thread = threading.Thread(target=self._io_loop, daemon=True)
        self._io_thread.start()

    def stop(self):
        self._stop.set()

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        self.log_seq += 1
        self.log_lines.append((self.log_seq, line))
        if len(self.log_lines) > self.log_limit:
            self.log_lines = self.log_lines[-self.log_limit:]

    def connect(self) -> Tuple[bool, str]:
        if self.connected:
            return True, "Already connected"
        if self.sim_mode:
            ok, msg = self.driver.connect()
            self.connected = ok
            if ok:
                self.log("SIM Connected")
            return ok, msg

        ok, msg = self.driver.connect()
        if ok:
            self.connected = True
            self.sim_fallback = False
            self.log("Connected")
            return True, msg

        # fallback to SIM
        self.driver = SimIoDriver()
        self.sim_fallback = True
        for m in self.pio.values():
            m.drv = self.driver
        ok2, msg2 = self.driver.connect()
        self.connected = ok2
        if ok2:
            self.log(f"WMX connect failed: {msg}")
            self.log("Falling back to SIM driver")
            return True, "SIM Connected (fallback)"
        return False, msg

    def disconnect(self):
        if not self.connected:
            return
        try:
            self.driver.disconnect()
        finally:
            self.connected = False
            self.log("Disconnected")

    def set_mode(self, mode: str):
        prev = self.mode_auto
        self.mode_auto = (mode.upper() == "AUTO")
        self.log(f"Mode -> {'AUTO' if self.mode_auto else 'MANUAL'}")
        if (prev is False) and (self.mode_auto is True):
            self.enter_auto_mode()

    def enter_auto_mode(self):
        self.forced_out.clear()
        for m in self.pio.values():
            m.state = PIOState.IDLE
            m.mode = None
            m.reset_outputs()
        self.log("ENTER AUTO: cleared forces + reset PIO outputs (READY/L/U=0, HO_AVBL/ES=1)")

    def clear_forces(self):
        self.forced_out.clear()
        self.log("Clear Forces (override off)")

    def reset_all_outputs(self):
        self.forced_out.clear()
        for p in self.out_points:
            self.driver.set_out_bit(p.byte_index, p.bit_index, 0)
        for m in self.pio.values():
            m.state = PIOState.IDLE
            m.mode = None
            m.reset_outputs()
        self.log("RESET ALL: outputs OFF then restore HO_AVBL/ES=1 (READY/L/U=0)")

    def reset_on_timeout(self):
        self.forced_out.clear()
        for m in self.pio.values():
            m.state = PIOState.IDLE
            m.mode = None
            m.reset_outputs()
        self.log("TIMEOUT RESET: READY/L/U=0, HO_AVBL/ES=1 (forces cleared)")

    def toggle_out_bit(self, byte: int, bit: int, force: bool = True) -> int:
        cur = self.driver.get_out_bit(byte, bit)
        newv = 0 if cur else 1
        self.driver.set_out_bit(byte, bit, newv)
        if force:
            self.forced_out[(byte, bit)] = newv
        else:
            self.forced_out.pop((byte, bit), None)
        self.log(f"Force(Tag/Matrix): byte={byte} bit={bit} -> {'ON' if newv else 'OFF'} (override={force})")
        return newv

    def set_out_bit(self, byte: int, bit: int, value: int, force: bool = True):
        self.driver.set_out_bit(byte, bit, int(value))
        if force:
            self.forced_out[(byte, bit)] = int(value)
        else:
            self.forced_out.pop((byte, bit), None)
        self.log(f"SetOut: byte={byte} bit={bit} value={value} (override={force})")

    def pio_start(self, channel: str, action: str):
        if not self.mode_auto:
            self.log("PIO start blocked: MANUAL mode")
            return
        m = self.pio.get(channel)
        if not m:
            return
        if action.upper() == "LOAD":
            m.start_load()
        else:
            m.start_unload()

    def _io_loop(self):
        while not self._stop.is_set():
            with self.lock:
                if self.connected:
                    inb = self.driver.get_in_bytes(0x00, self.num_bytes)
                    outb = self.driver.get_out_bytes(0x00, self.num_bytes)

                    if self.mode_auto:
                        for m in self.pio.values():
                            res = m.tick(inb)
                            if res == "TIMEOUT":
                                self.reset_on_timeout()
                                outb = self.driver.get_out_bytes(0x00, self.num_bytes)
                                break

                    if self.forced_out:
                        for (byte, bit), v in self.forced_out.items():
                            self.driver.set_out_bit(byte, bit, int(v))
                        outb = self.driver.get_out_bytes(0x00, self.num_bytes)

                    self.last_in_bytes = list(inb)
                    self.last_out_bytes = list(outb)
                else:
                    self.last_in_bytes = [0] * self.num_bytes
                    self.last_out_bytes = [0] * self.num_bytes
            time.sleep(0.1)

    def snapshot(self, since: int = 0) -> Dict:
        logs = [{"seq": seq, "msg": msg} for seq, msg in self.log_lines if seq > since]
        forced = [{"byte": b, "bit": bit, "value": v} for (b, bit), v in self.forced_out.items()]
        return {
            "profile": self.profile,
            "connected": self.connected,
            "sim_fallback": self.sim_fallback,
            "mode": "AUTO" if self.mode_auto else "MANUAL",
            "channels": self.channels,
            "num_bytes": self.num_bytes,
            "in_bytes": list(self.last_in_bytes),
            "out_bytes": list(self.last_out_bytes),
            "forced": forced,
            "log_seq": self.log_seq,
            "log_tail": logs,
        }


class JsonTcpHandler(socketserver.StreamRequestHandler):
    def handle(self):
        while True:
            line = self.rfile.readline()
            if not line:
                break
            try:
                req = json.loads(line.decode("utf-8").strip())
            except Exception:
                self._write({"ok": False, "error": "invalid json"})
                continue

            req_id = req.get("id")
            cmd = req.get("cmd")
            params = req.get("params", {})

            try:
                data = self.server.state_handle(cmd, params)
                resp = {"ok": True, "data": data}
            except Exception as e:
                resp = {"ok": False, "error": str(e)}

            if req_id is not None:
                resp["id"] = req_id
            self._write(resp)

    def _write(self, obj: Dict):
        raw = (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")
        self.wfile.write(raw)


class JsonTcpServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, state: BackendState):
        super().__init__(server_address, handler_class)
        self.state = state

    def state_handle(self, cmd: str, params: Dict):
        with self.state.lock:
            if cmd == "connect":
                ok, msg = self.state.connect()
                return {"ok": ok, "msg": msg}
            if cmd == "disconnect":
                self.state.disconnect()
                return {"ok": True}
            if cmd == "get_snapshot":
                since = int(params.get("since", 0))
                return self.state.snapshot(since=since)
            if cmd == "set_mode":
                self.state.set_mode(params.get("mode", "AUTO"))
                return {"ok": True}
            if cmd == "toggle_out_bit":
                byte = int(params.get("byte", 0))
                bit = int(params.get("bit", 0))
                force = bool(params.get("force", True))
                newv = self.state.toggle_out_bit(byte, bit, force)
                return {"value": newv}
            if cmd == "set_out_bit":
                byte = int(params.get("byte", 0))
                bit = int(params.get("bit", 0))
                value = int(params.get("value", 0))
                force = bool(params.get("force", True))
                self.state.set_out_bit(byte, bit, value, force)
                return {"ok": True}
            if cmd == "clear_forces":
                self.state.clear_forces()
                return {"ok": True}
            if cmd == "reset_all":
                self.state.reset_all_outputs()
                return {"ok": True}
            if cmd == "pio_start":
                channel = params.get("channel", "")
                action = params.get("action", "")
                self.state.pio_start(channel, action)
                return {"ok": True}
        raise ValueError(f"Unknown cmd: {cmd}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=["main", "other"], required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--sim", action="store_true", help="force SIM driver")
    args = parser.parse_args()

    state = BackendState(profile=args.profile, sim_mode=args.sim)
    server = JsonTcpServer((args.host, args.port), JsonTcpHandler, state)

    print(f"[backend:{args.profile}] listening on {args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.stop()
        server.server_close()


if __name__ == "__main__":
    main()

import time
from DllHelper import ensure_dll_exists
ensure_dll_exists()
from WMX3ApiPython import *

# ==========================================
# IO 맵핑 (Port 기준)
# ==========================================
OUT_L_REQ   = 0
OUT_U_REQ   = 1
OUT_READY   = 3
OUT_HO_AVBL = 6
OUT_ES      = 7

IN_VALID    = 0
IN_CS_0     = 1
IN_TR_REQ   = 4
IN_BUSY     = 5
IN_COMPT    = 6

TIMEOUT_VAL = 5.0

class E84PortController:
    def __init__(self):
        self.wmx = Wmx3Lib()
        self.io = IoControl(self.wmx)
        self.cm = CoreMotion(self.wmx)
        
        # WMX3 연결
        self.wmx.CreateDevice('C:\\Program Files\\SoftServo\\WMX3', DeviceType.DeviceTypeNormal, 10000)
        self.wmx.StartCommunication(10000)

        self.step = 0
        self.state_msg = "IDLE - Ready for Manual/Auto"
        self.start_time = 0
        self.is_error = False
        
        # 초기화 실행
        self.reset()

    def set_output(self, bit, val):
        self.io.SetOutBit(0, bit, val)

    def get_output(self, bit):
        ret, val = self.io.GetOutBit(0, bit)
        return val

    def get_input(self, bit):
        ret, val = self.io.GetInBit(0, bit)
        return val

    def toggle_output(self, bit):
        """개별 IO 토글 (수동 제어용)"""
        # 현재 상태를 읽어서 반대로 설정
        curr_val = self.get_output(bit)
        new_val = 1 if curr_val == 0 else 0
        self.set_output(bit, new_val)
        print(f">> MANUAL: Output Bit {bit} -> {new_val}")

    def reset(self):
        self.step = 0
        self.is_error = False
        self.state_msg = "IDLE - Manual Control OK"
        
        # 전체 끄기
        self.set_output(OUT_L_REQ, 0)
        self.set_output(OUT_U_REQ, 0)
        self.set_output(OUT_READY, 0)
        # HO_AVBL은 보통 켜두지만, 수동 제어를 위해 일단 켜둠
        self.set_output(OUT_HO_AVBL, 1)
        self.set_output(OUT_ES, 0)
        
        print(">> SYSTEM RESET")

    def start_load(self):
        if self.step == 0:
            self.step = 1
            self.start_time = time.time()
            self.state_msg = "AUTO: LOAD START"
            print(">> CMD: LOAD START")

    def start_unload(self):
        if self.step == 0:
            self.step = 10
            self.start_time = time.time()
            self.state_msg = "AUTO: UNLOAD START"
            print(">> CMD: UNLOAD START")

    def error_occurred(self, msg):
        self.is_error = True
        self.state_msg = f"ERROR: {msg}"
        print(f"!! {msg}")
        self.set_output(OUT_L_REQ, 0)
        self.set_output(OUT_U_REQ, 0)
        self.set_output(OUT_READY, 0)

    def run_cycle(self):
        # 에러 상태면 로직 중지 (수동 제어는 가능하게 할 수도 있으나 안전상 보통 막음)
        if self.is_error:
            return 

        now = time.time()

        # -----------------------------------------------
        # STEP 0: IDLE (여기서는 아무것도 안함 -> 수동 제어 가능)
        # -----------------------------------------------
        if self.step == 0:
            pass 

        # =================================================================
        # LOAD SEQUENCE (자동 제어 중에는 로직이 IO를 덮어씀)
        # =================================================================
        elif self.step == 1:
            self.set_output(OUT_L_REQ, 1)
            if self.get_input(IN_VALID) == 1:
                self.step = 2
                self.start_time = now
                self.state_msg = "LOAD: Wait TR_REQ"

        elif self.step == 2:
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP1 Timeout")
                return
            if self.get_input(IN_TR_REQ) == 1:
                self.step = 3
                self.start_time = now
                self.state_msg = "LOAD: TR_REQ On -> READY"

        elif self.step == 3:
            self.set_output(OUT_READY, 1)
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP2 Timeout")
                return
            if self.get_input(IN_BUSY) == 1:
                self.step = 4
                self.start_time = now
                self.state_msg = "LOAD: Transferring..."

        elif self.step == 4:
            if self.get_input(IN_COMPT) == 1:
                self.step = 5
                self.start_time = now
                self.state_msg = "LOAD: Finishing"

        elif self.step == 5:
            self.set_output(OUT_READY, 0)
            self.set_output(OUT_L_REQ, 0)
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP5 Timeout")
                return
            if self.get_input(IN_VALID) == 0:
                self.step = 0
                self.state_msg = "LOAD SUCCESS -> IDLE"
                print(">> LOAD SUCCESS")

        # =================================================================
        # UNLOAD SEQUENCE
        # =================================================================
        elif self.step == 10:
            self.set_output(OUT_U_REQ, 1)
            if self.get_input(IN_VALID) == 1:
                self.step = 11
                self.start_time = now
                self.state_msg = "UNLOAD: Wait TR_REQ"

        elif self.step == 11:
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP1 Timeout")
                return
            if self.get_input(IN_TR_REQ) == 1:
                self.step = 12
                self.start_time = now
                self.state_msg = "UNLOAD: TR_REQ On -> READY"

        elif self.step == 12:
            self.set_output(OUT_READY, 1)
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP2 Timeout")
                return
            if self.get_input(IN_BUSY) == 1:
                self.step = 13
                self.start_time = now
                self.state_msg = "UNLOAD: Transferring..."

        elif self.step == 13:
            if self.get_input(IN_COMPT) == 1:
                self.step = 14
                self.start_time = now
                self.state_msg = "UNLOAD: Finishing"

        elif self.step == 14:
            self.set_output(OUT_READY, 0)
            self.set_output(OUT_U_REQ, 0)
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP5 Timeout")
                return
            if self.get_input(IN_VALID) == 0:
                self.step = 0
                self.state_msg = "UNLOAD SUCCESS -> IDLE"
                print(">> UNLOAD SUCCESS")

    def close(self):
        self.wmx.StopCommunication()
        self.wmx.CloseDevice()
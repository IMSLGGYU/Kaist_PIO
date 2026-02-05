import time
from DllHelper import ensure_dll_exists
ensure_dll_exists() # DLL 로드
from WMX3ApiPython import *

# ==========================================
# IO 맵핑 (Port 기준)
# ==========================================
# OUTPUT (Port -> Vehicle)
OUT_L_REQ   = 0
OUT_U_REQ   = 1
OUT_READY   = 3
OUT_HO_AVBL = 6
OUT_ES      = 7

# INPUT (Vehicle -> Port)
IN_VALID    = 0
IN_CS_0     = 1
IN_TR_REQ   = 4
IN_BUSY     = 5
IN_COMPT    = 6

# 타임아웃 설정 (초)
TIMEOUT_VAL = 5.0

class E84PortController:
    def __init__(self):
        # WMX3 초기화
        self.wmx = Wmx3Lib()
        self.io = IoControl(self.wmx)
        self.cm = CoreMotion(self.wmx)
        
        # 장치 생성 및 통신 시작
        self.wmx.CreateDevice('C:\\Program Files\\SoftServo\\WMX3', DeviceType.DeviceTypeNormal, 10000)
        self.wmx.StartCommunication(10000)

        # 상태 변수
        self.step = 0
        self.state_msg = "IDLE"
        self.start_time = 0
        self.is_error = False
        
        # 기본 상태: HO_AVBL은 항상 켜둠
        self.reset()

    def set_output(self, bit, val):
        self.io.SetOutBit(0, bit, val)

    def get_input(self, bit):
        ret, val = self.io.GetInBit(0, bit)
        return val

    def reset(self):
        """에러 리셋 및 초기화"""
        self.step = 0
        self.is_error = False
        self.state_msg = "IDLE - Waiting for Command"
        
        # 모든 신호 OFF
        self.set_output(OUT_L_REQ, 0)
        self.set_output(OUT_U_REQ, 0)
        self.set_output(OUT_READY, 0)
        
        # 상시 ON 신호
        self.set_output(OUT_HO_AVBL, 1) # 설비 가용 상태
        self.set_output(OUT_ES, 0)      # E-Stop 해제 (Active Low인 경우 1로 변경 필요)
        
        print(">> SYSTEM RESET")

    def start_load(self):
        if self.step == 0:
            self.step = 1
            self.start_time = time.time()
            self.state_msg = "LOAD START (L_REQ ON)"
            print(">> CMD: LOAD START")

    def start_unload(self):
        if self.step == 0:
            self.step = 10
            self.start_time = time.time()
            self.state_msg = "UNLOAD START (U_REQ ON)"
            print(">> CMD: UNLOAD START")

    def error_occurred(self, msg):
        self.is_error = True
        self.state_msg = f"ERROR: {msg}"
        print(f"!! {msg}")
        # 에러 시 요청 신호와 Ready 끄기
        self.set_output(OUT_L_REQ, 0)
        self.set_output(OUT_U_REQ, 0)
        self.set_output(OUT_READY, 0)

    def run_cycle(self):
        """UI 타이머에서 0.1초마다 호출하는 함수"""
        if self.is_error:
            return # 에러 상태면 Reset 전까지 동작 안 함

        now = time.time()

        # -----------------------------------------------
        # STEP 0: IDLE
        # -----------------------------------------------
        if self.step == 0:
            pass 

        # =================================================================
        # LOAD SEQUENCE (Step 1 ~ 5)
        # =================================================================
        elif self.step == 1: # L_REQ 켜고 VALID 대기
            self.set_output(OUT_L_REQ, 1)
            
            if self.get_input(IN_VALID) == 1:
                self.step = 2
                self.start_time = now
                self.state_msg = "LOAD Step 2: VALID On -> Wait TR_REQ"

        elif self.step == 2: # TR_REQ 대기 (TP1)
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP1 Timeout (Wait TR_REQ)")
                return
            if self.get_input(IN_TR_REQ) == 1:
                self.step = 3
                self.start_time = now
                self.state_msg = "LOAD Step 3: TR_REQ On -> READY ON"

        elif self.step == 3: # READY 켜고 BUSY 대기 (TP2)
            self.set_output(OUT_READY, 1)
            
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP2 Timeout (Wait BUSY)")
                return
            if self.get_input(IN_BUSY) == 1:
                self.step = 4
                self.start_time = now
                self.state_msg = "LOAD Step 4: BUSY On -> Transferring..."

        elif self.step == 4: # COMPT 대기 (전송 중)
            if self.get_input(IN_COMPT) == 1:
                self.step = 5
                self.start_time = now
                self.state_msg = "LOAD Step 5: COMPT On -> Finishing"

        elif self.step == 5: # 종료 처리 (TP5)
            self.set_output(OUT_READY, 0)
            self.set_output(OUT_L_REQ, 0)
            
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP5 Timeout (Wait VALID Off)")
                return
            if self.get_input(IN_VALID) == 0:
                self.step = 0
                self.state_msg = "LOAD COMPLETE -> IDLE"
                print(">> LOAD SUCCESS")

        # =================================================================
        # UNLOAD SEQUENCE (Step 10 ~ 14)
        # =================================================================
        elif self.step == 10: # U_REQ 켜고 VALID 대기
            self.set_output(OUT_U_REQ, 1)
            
            if self.get_input(IN_VALID) == 1:
                self.step = 11
                self.start_time = now
                self.state_msg = "UNLOAD Step 11: VALID On -> Wait TR_REQ"

        elif self.step == 11: # TR_REQ 대기 (TP1)
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP1 Timeout (Wait TR_REQ)")
                return
            if self.get_input(IN_TR_REQ) == 1:
                self.step = 12
                self.start_time = now
                self.state_msg = "UNLOAD Step 12: TR_REQ On -> READY ON"

        elif self.step == 12: # READY 켜고 BUSY 대기 (TP2)
            self.set_output(OUT_READY, 1)
            
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP2 Timeout (Wait BUSY)")
                return
            if self.get_input(IN_BUSY) == 1:
                self.step = 13
                self.start_time = now
                self.state_msg = "UNLOAD Step 13: BUSY On -> Transferring..."

        elif self.step == 13: # COMPT 대기
            if self.get_input(IN_COMPT) == 1:
                self.step = 14
                self.start_time = now
                self.state_msg = "UNLOAD Step 14: COMPT On -> Finishing"

        elif self.step == 14: # 종료 처리
            self.set_output(OUT_READY, 0)
            self.set_output(OUT_U_REQ, 0)
            
            if now - self.start_time > TIMEOUT_VAL:
                self.error_occurred("TP5 Timeout (Wait VALID Off)")
                return
            if self.get_input(IN_VALID) == 0:
                self.step = 0
                self.state_msg = "UNLOAD COMPLETE -> IDLE"
                print(">> UNLOAD SUCCESS")

    def close(self):
        self.wmx.StopCommunication()
        self.wmx.CloseDevice()
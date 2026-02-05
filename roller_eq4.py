# roller.py
# WMX3 IO 기반 롤러 제어 라이브러리 (하드코딩 X: 주소만 설정으로 교체)
#
# 요구사항 반영:
# 1) Inward:  dir=0, run=1 -> IN(byte5 bit6)=1 되면 run=0 즉시 정지
# 2) Outward: dir=1, run=1 -> IN(byte5 bit7)=1 되면 run=0 즉시 정지(대기 상태)
# 3) Transfer: IN(byte5 bit7)=1 상태에서 dir=1, run=1
#              IN(byte5 bit7)=0 이 되면 5초 delay 후 run=0 정지
#
# IO 의미:
# - OUT byte3 bit2: Roller RUN (ON/OFF)
# - OUT byte3 bit3: Roller DIR (0=inward, 1=outward)

import time
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class BitAddr:
    byte: int
    bit: int


@dataclass
class RollerIOMap:
    """
    다른 공정에서 재사용할 때 여기 주소만 바꾸면 됨.
    """
    # Outputs
    out_run: BitAddr = BitAddr(byte=3, bit=2)   # RUN ON/OFF
    out_dir: BitAddr = BitAddr(byte=3, bit=3)   # DIR (0 inward, 1 outward)

    # Inputs
    in_inward_stop: BitAddr = BitAddr(byte=5, bit=6)   # 1번 동작 멈춤 센서
    in_outward_stop: BitAddr = BitAddr(byte=5, bit=7)  # 2/3번 동작 관련 센서

    # WMX3 Io 포트 번호 (대부분 0)
    io_port_no: int = 0


class RollerController:
    """
    WMX3ApiPython의 Io 객체를 주입받아 사용.
    - 예: io = Io(Wmx3Lib)
         roller = RollerController(io)
    """

    def __init__(
        self,
        io,  # WMX3ApiPython.Io
        iomap: RollerIOMap = RollerIOMap(),
        poll_interval_s: float = 0.005,
    ):
        self.io = io
        self.iomap = iomap
        self.poll_interval_s = max(poll_interval_s, 0.001)

    # -------------------------
    # 기본 IO 래퍼
    # -------------------------
    def _set_outbit(self, addr: BitAddr, value: int) -> None:
        self.io.SetOutBit(self.iomap.io_port_no, addr.byte, addr.bit, int(value))

    def _get_inbit(self, addr: BitAddr) -> int:
        """
        WMX3ApiPython 래퍼에 따라 반환 타입이 (ret, value) 일 수도, value만 줄 수도 있음.
        최대한 호환되게 처리.
        """
        v = self.io.GetInBit(self.iomap.io_port_no, addr.byte, addr.bit)
        # 경우1) 값만 반환
        if isinstance(v, int):
            return v
        # 경우2) (result, value) 또는 리스트/튜플
        if isinstance(v, (tuple, list)) and len(v) >= 2:
            return int(v[1])
        # 경우3) 객체 형태면 value 속성 찾아보기
        if hasattr(v, "value"):
            return int(v.value)
        raise RuntimeError(f"GetInBit 반환 형식을 해석할 수 없음: {type(v)} / {v}")

    def stop(self) -> None:
        """
        롤러 정지: RUN=0 (방향은 그대로 둠)
        """
        self._set_outbit(self.iomap.out_run, 0)

    def _run(self, direction: int) -> None:
        """
        direction: 0=inward, 1=outward
        요구사항대로: 방향 먼저 설정 -> RUN ON
        """
        self._set_outbit(self.iomap.out_dir, 1 if direction else 0)
        self._set_outbit(self.iomap.out_run, 1)

    def _wait_until_inbit(
        self,
        addr: BitAddr,
        target: int,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """
        addr 입력이 target이 될 때까지 폴링 대기.
        timeout_s:
          - None: 무한 대기
          - 값: 초 단위 제한
        반환:
          - True: 조건 만족
          - False: timeout
        """
        start = time.monotonic()
        while True:
            if self._get_inbit(addr) == int(target):
                return True
            if timeout_s is not None and (time.monotonic() - start) >= timeout_s:
                return False
            time.sleep(self.poll_interval_s)

    # ============================================================
    # 1) 안으로 들어가다가 센서 만나면 멈추는 함수
    # ============================================================
    def move_in_until_sensor(
        self,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """
        - OUT DIR=0, RUN=1
        - IN(in_inward_stop)=1 되면 RUN=0 즉시
        반환: True(센서감지정지) / False(timeout)
        """
        self._run(direction=0)

        ok = self._wait_until_inbit(
            self.iomap.in_inward_stop,
            target=1,
            timeout_s=timeout_s,
        )

        # 센서 감지되든 timeout이든 "즉시 정지"는 요구사항상 안전
        self.stop()
        return ok

    # ============================================================
    # 2) 밖으로 나오다가 센서 만나면 멈춰 대기하는 함수
    # ============================================================
    def move_out_until_sensor_hold(
        self,
        timeout_s: Optional[float] = None,
    ) -> bool:
        """
        - OUT DIR=1, RUN=1
        - IN(in_outward_stop)=1 되면 RUN=0 즉시 멈춤(대기 상태)
        반환: True(센서감지정지) / False(timeout)
        """
        self._run(direction=1)

        ok = self._wait_until_inbit(
            self.iomap.in_outward_stop,
            target=1,
            timeout_s=timeout_s,
        )

        self.stop()
        return ok

    # ============================================================
    # 3) AMR로 carrier 전달 (센서 1->0 전환 후 5초 뒤 정지)
    # ============================================================
    def transfer_to_amr(
        self,
        post_clear_delay_s: float = 5.0,
        timeout_wait_clear_s: Optional[float] = None,
        require_initial_sensor_1: bool = True,
    ) -> bool:
        """
        요구사항:
        - 조건: IN(byte5 bit7)=1 상태
        - OUT DIR=1, RUN=1 시작
        - IN(byte5 bit7)=0 되면 delay(5초) 후 RUN=0 정지

        파라미터:
        - post_clear_delay_s: 0으로 떨어진 뒤 추가 구동 시간(기본 5초)
        - timeout_wait_clear_s: 1->0으로 떨어질 때까지 대기 timeout (None이면 무한)
        - require_initial_sensor_1: 시작 시 센서가 1이 아니면 예외 대신 False로 반환할지 선택

        반환:
        - True: 정상(1을 확인했고, 0으로 떨어짐을 감지한 뒤 딜레이 후 정지)
        - False: (require_initial_sensor_1=True인데 시작 센서가 1이 아님) 또는 (timeout 발생)
        """
        # 1) 구동 시작 (OUT)
        self._run(direction=1)

        # 2) 센서가 1이 되는 것을 먼저 확인
        seen_one = self._wait_until_inbit(
            self.iomap.in_outward_stop,
            target=1,
            timeout_s=timeout_wait_clear_s,
        )

        if not seen_one:
            # 1을 못 보면 안전 정지
            self.stop()
            return False

        # 3) 이후 센서가 0으로 떨어질 때까지 대기
        cleared = self._wait_until_inbit(
            self.iomap.in_outward_stop,
            target=0,
            timeout_s=timeout_wait_clear_s,
        )

        if not cleared:
            # timeout이면 안전 정지
            self.stop()
            return False

        # 4) 0 된 후 5초 지연 운전 후 정지
        time.sleep(max(0.0, float(post_clear_delay_s)))
        self.stop()
        return True



# EQ1 Controller (Backend + UI)

## 구성
- `eq1_backend.py`: WMX3/PIO 로직이 들어있는 헤드리스 백엔드 (JSON-over-TCP)
- `eq1_controller_ui.py`: 듀얼 탭 UI (MAIN / OTHER) — 백엔드에 RPC로 연결

## 실행 방법
### 1) 백엔드 2개 실행 (서로 다른 프로세스/포트)
```powershell
# MAIN 프로필
python .\eq1_backend.py --profile main --port 9011

# OTHER 프로필
python .\eq1_backend.py --profile other --port 9012
```

### 2) 컨트롤러 UI 실행
```powershell
python .\eq1_controller_ui.py --main-port 9011 --other-port 9012
```

## SIM 모드
WMX3 연결이 필요 없거나 장비가 없을 때:
```powershell
python .\eq1_backend.py --profile main --port 9011 --sim
python .\eq1_backend.py --profile other --port 9012 --sim
```

## 기능 유지 사항
- Tag 기반 IO 그리드
- Output Force (multi + override 유지)
- AUTO / MANUAL 모드
- PIO 상태머신 LOAD/UNLOAD
- Raw 8x8 matrix 클릭 = 출력 토글 + force
- TIMEOUT 시 HO_AVBL/ES=1 유지, 나머지 0 리셋

## 통신 프로토콜 (요약)
- JSON 한 줄 요청/응답 (TCP)
- 주요 명령: connect, disconnect, get_snapshot, set_mode, toggle_out_bit, set_out_bit,
  clear_forces, reset_all, pio_start

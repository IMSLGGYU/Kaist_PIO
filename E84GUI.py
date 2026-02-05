import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# 파일명 import 수정됨
from E84Logic import E84PortController

class E84GridWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SEMI E84 Controller (Manual + Auto)")
        self.resize(600, 500)

        # 컨트롤러 연결
        try:
            self.controller = E84PortController()
            self.connection_status = True
        except Exception as e:
            QMessageBox.critical(self, "Error", f"WMX3 Init Error: {e}\nSim Mode Active")
            self.connection_status = False

        # 메인 레이아웃
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # 1. 상태 표시창
        self.lbl_state = QLabel("IDLE")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setStyleSheet("background-color: #333; color: #0F0; font-size: 18px; font-weight: bold; padding: 10px;")
        layout.addWidget(self.lbl_state)

        # 2. 설명 라벨
        lbl_info = QLabel("※ Output(Port) 칸을 클릭하여 수동으로 제어할 수 있습니다.")
        lbl_info.setStyleSheet("color: gray; font-style: italic;")
        layout.addWidget(lbl_info)

        # 3. IO 그리드 (테이블)
        self.table = QTableWidget()
        self.setup_table()
        layout.addWidget(self.table)

        # 4. 버튼 영역
        btn_layout = QHBoxLayout()
        
        self.btn_load = QPushButton("LOAD (적재)")
        self.btn_load.setFixedHeight(50)
        self.btn_load.clicked.connect(lambda: self.controller.start_load() if self.connection_status else None)
        
        self.btn_unload = QPushButton("UNLOAD (반출)")
        self.btn_unload.setFixedHeight(50)
        self.btn_unload.clicked.connect(lambda: self.controller.start_unload() if self.connection_status else None)

        self.btn_reset = QPushButton("RESET")
        self.btn_reset.setFixedHeight(50)
        self.btn_reset.setStyleSheet("background-color: #FFDDDD; color: red; font-weight: bold;")
        self.btn_reset.clicked.connect(lambda: self.controller.reset() if self.connection_status else None)

        btn_layout.addWidget(self.btn_load)
        btn_layout.addWidget(self.btn_unload)
        btn_layout.addWidget(self.btn_reset)
        layout.addLayout(btn_layout)

        # 5. 타이머 (0.1초마다 갱신)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_system)
        self.timer.start(100) 

    def setup_table(self):
        self.table.setRowCount(8)
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["IO No", "Output (Port)", "Input (Vehicle)", "Name"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers) # 더블클릭 수정 방지
        
        # [중요] 셀 클릭 이벤트 연결 -> 수동 제어
        self.table.cellClicked.connect(self.on_cell_clicked)

        self.io_names = {
            0: ("L_REQ", "VALID"),
            1: ("U_REQ", "CS_0"),
            2: ("-", "CS_1"),
            3: ("READY", "-"),
            4: ("-", "TR_REQ"),
            5: ("-", "BUSY"),
            6: ("HO_AVBL", "COMPT"),
            7: ("ES", "-")
        }

        for i in range(8):
            self.table.setItem(i, 0, QTableWidgetItem(str(i)))
            self.table.setItem(i, 1, QTableWidgetItem(self.io_names[i][0]))
            self.table.setItem(i, 2, QTableWidgetItem(self.io_names[i][1]))
            desc = f"{self.io_names[i][0]} / {self.io_names[i][1]}"
            self.table.setItem(i, 3, QTableWidgetItem(desc))

    def on_cell_clicked(self, row, col):
        """그리드 셀 클릭 시 호출"""
        if not self.connection_status: return

        # col 1번이 Output(Port) 컬럼입니다.
        if col == 1:
            # IO 수동 토글 실행
            self.controller.toggle_output(row)
            # 반응성을 위해 UI 즉시 갱신 호출
            self.update_system()

    def update_system(self):
        if not self.connection_status:
            return

        self.controller.run_cycle()

        self.lbl_state.setText(self.controller.state_msg)
        if self.controller.is_error:
            self.lbl_state.setStyleSheet("background-color: red; color: white; font-size: 18px; padding: 10px;")
        else:
            self.lbl_state.setStyleSheet("background-color: #333; color: #0F0; font-size: 18px; padding: 10px;")

        # 테이블 색상 갱신
        for i in range(8):
            # Output Check
            try:
                val = self.controller.get_output(i)
                is_on = (val == 1)
                self.set_cell_color(i, 1, is_on)
            except: pass

            # Input Check
            try:
                val = self.controller.get_input(i)
                is_on = (val == 1)
                self.set_cell_color(i, 2, is_on)
            except: pass

    def set_cell_color(self, row, col, is_on):
        item = self.table.item(row, col)
        if item.text() == "-": 
            item.setBackground(QColor(240, 240, 240)) 
            return

        if is_on:
            item.setBackground(QColor(0, 255, 0)) # ON: Green
            item.setForeground(QColor(0, 0, 0))
        else:
            item.setBackground(QColor(255, 255, 255)) # OFF: White
            item.setForeground(QColor(0, 0, 0))

    def closeEvent(self, event):
        if self.connection_status:
            self.controller.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = E84GridWindow()
    window.show()
    sys.exit(app.exec_())
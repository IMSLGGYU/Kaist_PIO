#*****************************************************************************#
#* FILE        : ControlIO_UI.Py                                             *# 
#* DESCRIPTION : IO Monitor UI - Grid view with periodic refresh             *#
#*****************************************************************************#

#*****************************************************************************#
#* Import                                                                    *#
#*****************************************************************************#
from DllHelper import *
ensure_dll_exists()
from WMX3ApiPython import *

import tkinter as tk
from tkinter import ttk, messagebox
import threading

#*****************************************************************************#
#* Constants                                                                 *#
#*****************************************************************************#
INFINITE = int(0xFFFFFFFF)
NUM_BYTES = 8  # 표시할 바이트 수
NUM_BITS = 8   # 바이트당 비트 수

#*****************************************************************************#
#* IO Monitor UI Class                                                       *#
#*****************************************************************************#
class IOMonitorUI:
    def __init__(self, root):
        self.root = root
        self.root.title("WMX3 IO Monitor")
        self.root.geometry("900x700")
        self.root.configure(bg='#2b2b2b')
        
        # WMX3 API
        self.Wmx3Lib = None
        self.Wmx3Lib_Io = None
        self.is_connected = False
        self.is_running = False
        self.refresh_interval = 100  # ms
        
        # IO 데이터 저장
        self.input_data = [0] * NUM_BYTES
        self.output_data = [0] * NUM_BYTES
        
        # UI 요소 저장
        self.input_labels = []
        self.output_buttons = []
        
        self.setup_ui()
        
    def setup_ui(self):
        # 스타일 설정
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#2b2b2b')
        style.configure('TLabel', background='#2b2b2b', foreground='white', font=('Consolas', 10))
        style.configure('TButton', font=('Consolas', 10))
        style.configure('Header.TLabel', font=('Consolas', 12, 'bold'), foreground='#4fc3f7')
        
        # 메인 프레임
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # === 연결 제어 프레임 ===
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.connect_btn = tk.Button(control_frame, text="Connect", command=self.toggle_connection,
                                      bg='#4caf50', fg='white', font=('Consolas', 11, 'bold'),
                                      width=12, height=1)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.status_label = ttk.Label(control_frame, text="Disconnected", foreground='#f44336')
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # 갱신 주기 설정
        ttk.Label(control_frame, text="Refresh (ms):").pack(side=tk.LEFT, padx=(20, 5))
        self.refresh_var = tk.StringVar(value="100")
        refresh_entry = ttk.Entry(control_frame, textvariable=self.refresh_var, width=6)
        refresh_entry.pack(side=tk.LEFT)
        
        apply_btn = tk.Button(control_frame, text="Apply", command=self.apply_refresh,
                               bg='#2196f3', fg='white', font=('Consolas', 9))
        apply_btn.pack(side=tk.LEFT, padx=5)
        
        # === INPUT 그리드 ===
        input_frame = ttk.LabelFrame(main_frame, text=" INPUT (Read Only) ", padding=10)
        input_frame.pack(fill=tk.X, pady=5)
        
        # 헤더
        header_frame = ttk.Frame(input_frame)
        header_frame.pack(fill=tk.X)
        ttk.Label(header_frame, text="Byte", width=6).grid(row=0, column=0)
        ttk.Label(header_frame, text="Hex", width=6).grid(row=0, column=1)
        for bit in range(NUM_BITS-1, -1, -1):
            ttk.Label(header_frame, text=f"B{bit}", width=4).grid(row=0, column=2+NUM_BITS-1-bit)
        
        # INPUT 데이터 행
        self.input_frame_rows = ttk.Frame(input_frame)
        self.input_frame_rows.pack(fill=tk.X, pady=5)
        
        for byte_idx in range(NUM_BYTES):
            row_frame = ttk.Frame(self.input_frame_rows)
            row_frame.pack(fill=tk.X, pady=1)
            
            ttk.Label(row_frame, text=f"0x{byte_idx:02X}", width=6).grid(row=0, column=0)
            
            hex_label = ttk.Label(row_frame, text="0x00", width=6, foreground='#81c784')
            hex_label.grid(row=0, column=1)
            
            bit_labels = []
            for bit in range(NUM_BITS-1, -1, -1):
                lbl = tk.Label(row_frame, text="0", width=3, height=1,
                              bg='#424242', fg='#888888', font=('Consolas', 10, 'bold'),
                              relief=tk.RAISED, bd=1)
                lbl.grid(row=0, column=2+NUM_BITS-1-bit, padx=1)
                bit_labels.append(lbl)
            
            self.input_labels.append({'hex': hex_label, 'bits': bit_labels})
        
        # === OUTPUT 그리드 ===
        output_frame = ttk.LabelFrame(main_frame, text=" OUTPUT (Click to Toggle) ", padding=10)
        output_frame.pack(fill=tk.X, pady=5)
        
        # 헤더
        header_frame2 = ttk.Frame(output_frame)
        header_frame2.pack(fill=tk.X)
        ttk.Label(header_frame2, text="Byte", width=6).grid(row=0, column=0)
        ttk.Label(header_frame2, text="Hex", width=6).grid(row=0, column=1)
        for bit in range(NUM_BITS-1, -1, -1):
            ttk.Label(header_frame2, text=f"B{bit}", width=4).grid(row=0, column=2+NUM_BITS-1-bit)
        
        # OUTPUT 데이터 행
        self.output_frame_rows = ttk.Frame(output_frame)
        self.output_frame_rows.pack(fill=tk.X, pady=5)
        
        for byte_idx in range(NUM_BYTES):
            row_frame = ttk.Frame(self.output_frame_rows)
            row_frame.pack(fill=tk.X, pady=1)
            
            ttk.Label(row_frame, text=f"0x{byte_idx:02X}", width=6).grid(row=0, column=0)
            
            hex_label = ttk.Label(row_frame, text="0x00", width=6, foreground='#ffb74d')
            hex_label.grid(row=0, column=1)
            
            bit_buttons = []
            for bit in range(NUM_BITS-1, -1, -1):
                btn = tk.Button(row_frame, text="0", width=3, height=1,
                               bg='#424242', fg='#888888', font=('Consolas', 10, 'bold'),
                               relief=tk.RAISED, bd=1,
                               command=lambda b=byte_idx, bt=bit: self.toggle_output_bit(b, bt))
                btn.grid(row=0, column=2+NUM_BITS-1-bit, padx=1)
                bit_buttons.append(btn)
            
            self.output_buttons.append({'hex': hex_label, 'bits': bit_buttons})
        
        # === 빠른 제어 버튼 ===
        quick_frame = ttk.LabelFrame(main_frame, text=" Quick Control ", padding=10)
        quick_frame.pack(fill=tk.X, pady=5)
        
        tk.Button(quick_frame, text="All ON", command=lambda: self.set_all_outputs(0xFF),
                  bg='#4caf50', fg='white', font=('Consolas', 10), width=10).pack(side=tk.LEFT, padx=5)
        tk.Button(quick_frame, text="All OFF", command=lambda: self.set_all_outputs(0x00),
                  bg='#f44336', fg='white', font=('Consolas', 10), width=10).pack(side=tk.LEFT, padx=5)
        
        # === 로그 영역 ===
        log_frame = ttk.LabelFrame(main_frame, text=" Log ", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = tk.Text(log_frame, height=8, bg='#1e1e1e', fg='#00ff00',
                                font=('Consolas', 9), state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(self.log_text, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
        
    def log(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        
    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
            
    def connect(self):
        try:
            self.log("Connecting to WMX3...")
            self.Wmx3Lib = WMX3Api()
            self.Wmx3Lib_Io = Io(self.Wmx3Lib)
            
            self.Wmx3Lib.CreateDevice('C:\\Program Files\\SoftServo\\WMX3\\', 
                                       DeviceType.DeviceTypeNormal, INFINITE)
            self.Wmx3Lib.SetDeviceName('ControlIO_UI')
            ret = self.Wmx3Lib.StartCommunication(INFINITE)
            
            self.is_connected = True
            self.is_running = True
            self.connect_btn.config(text="Disconnect", bg='#f44336')
            self.status_label.config(text="Connected", foreground='#4caf50')
            self.log("Connected successfully!")
            
            # 주기적 갱신 시작
            self.update_io_data()
            
        except Exception as e:
            self.log(f"Connection failed: {str(e)}")
            messagebox.showerror("Connection Error", str(e))
            
    def disconnect(self):
        try:
            self.is_running = False
            if self.Wmx3Lib:
                self.Wmx3Lib.StopCommunication(INFINITE)
                self.Wmx3Lib.CloseDevice()
            self.is_connected = False
            self.connect_btn.config(text="Connect", bg='#4caf50')
            self.status_label.config(text="Disconnected", foreground='#f44336')
            self.log("Disconnected.")
        except Exception as e:
            self.log(f"Disconnect error: {str(e)}")
            
    def apply_refresh(self):
        try:
            self.refresh_interval = int(self.refresh_var.get())
            self.log(f"Refresh interval set to {self.refresh_interval}ms")
        except ValueError:
            messagebox.showerror("Error", "Invalid refresh interval")
            
    def update_io_data(self):
        if not self.is_running or not self.is_connected:
            return
            
        try:
            # INPUT 데이터 읽기
            ret, in_bytes = self.Wmx3Lib_Io.GetInBytes(0x00, NUM_BYTES)
            if ret == 0 and in_bytes:
                self.input_data = list(in_bytes)
                
            # OUTPUT 데이터 읽기
            ret, out_bytes = self.Wmx3Lib_Io.GetOutBytes(0x00, NUM_BYTES)
            if ret == 0 and out_bytes:
                self.output_data = list(out_bytes)
                
            # UI 업데이트
            self.update_ui()
            
        except Exception as e:
            self.log(f"Read error: {str(e)}")
            
        # 다음 갱신 예약
        if self.is_running:
            self.root.after(self.refresh_interval, self.update_io_data)
            
    def update_ui(self):
        # INPUT UI 업데이트
        for byte_idx in range(NUM_BYTES):
            value = self.input_data[byte_idx] if byte_idx < len(self.input_data) else 0
            self.input_labels[byte_idx]['hex'].config(text=f"0x{value:02X}")
            
            for bit in range(NUM_BITS):
                bit_value = (value >> bit) & 1
                lbl = self.input_labels[byte_idx]['bits'][NUM_BITS-1-bit]
                if bit_value:
                    lbl.config(text="1", bg='#4caf50', fg='white')
                else:
                    lbl.config(text="0", bg='#424242', fg='#888888')
                    
        # OUTPUT UI 업데이트
        for byte_idx in range(NUM_BYTES):
            value = self.output_data[byte_idx] if byte_idx < len(self.output_data) else 0
            self.output_buttons[byte_idx]['hex'].config(text=f"0x{value:02X}")
            
            for bit in range(NUM_BITS):
                bit_value = (value >> bit) & 1
                btn = self.output_buttons[byte_idx]['bits'][NUM_BITS-1-bit]
                if bit_value:
                    btn.config(text="1", bg='#ff9800', fg='white')
                else:
                    btn.config(text="0", bg='#424242', fg='#888888')
                    
    def toggle_output_bit(self, byte_idx, bit):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected!")
            return
            
        try:
            # 현재 비트 값 읽기
            ret, current = self.Wmx3Lib_Io.GetOutBit(byte_idx, bit)
            new_value = 0 if current else 1
            
            # 비트 값 쓰기
            self.Wmx3Lib_Io.SetOutBit(byte_idx, bit, new_value)
            self.log(f"Set Output Byte[0x{byte_idx:02X}] Bit[{bit}] = {new_value}")
            
        except Exception as e:
            self.log(f"Write error: {str(e)}")
            
    def set_all_outputs(self, value):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected!")
            return
            
        try:
            data = [value] * NUM_BYTES
            self.Wmx3Lib_Io.SetOutBytes(0x00, NUM_BYTES, data)
            self.log(f"Set all outputs to 0x{value:02X}")
        except Exception as e:
            self.log(f"Write error: {str(e)}")
            
    def on_closing(self):
        if self.is_connected:
            self.disconnect()
        self.root.destroy()

#*****************************************************************************#
#* Main                                                                      *#
#*****************************************************************************#
def main():
    root = tk.Tk()
    app = IOMonitorUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()

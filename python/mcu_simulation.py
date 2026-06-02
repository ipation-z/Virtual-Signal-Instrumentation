import sys
import time
import socket
import select
import numpy as np
from PySide6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QComboBox, 
                             QWidget, QLabel, QSpinBox, QDoubleSpinBox, 
                             QGroupBox, QGridLayout, QCheckBox)
from PySide6.QtCore import QThread, Signal, Slot, QTimer

class TcpServerThread(QThread):
    """本地TCP流媒体广播线程：引入 select 多路复用，稳定实现全双工收发"""
    client_status = Signal(str)
    cmd_received = Signal(int, int, int) 

    def __init__(self, host='127.0.0.1', port=8888):
        super().__init__()
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = True
        self.data_to_send = None
        self.recv_buffer = bytearray()

    def run(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.client_status.emit(f"等待上位机连入... (监听 {self.port})")
        except Exception as e:
            self.client_status.emit(f"端口绑定失败: {str(e)}")
            return

        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    self.client_status.emit(f"🟢 上位机已成功接入: {addr[0]}:{addr[1]}")
                except socket.timeout:
                    continue

                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                while self.running and self.client_socket:
                    # 1. 使用 select 进行安全非阻塞读取
                    readable, _, _ = select.select([self.client_socket], [], [], 0.001)
                    if readable:
                        data = self.client_socket.recv(1024)
                        if not data:
                            raise ConnectionResetError("客户端主动断开")
                        
                        self.recv_buffer.extend(data)
                        # 解析上位机发来的 8 字节控制协议
                        while len(self.recv_buffer) >= 8:
                            if self.recv_buffer[0] == 0x5A and self.recv_buffer[1] == 0xA5:
                                wave = self.recv_buffer[3]
                                freq = (self.recv_buffer[4] << 8) | self.recv_buffer[5]
                                vpp = self.recv_buffer[6]
                                chk = self.recv_buffer[7]
                                
                                calc_chk = (0x01 + wave + self.recv_buffer[4] + self.recv_buffer[5] + vpp) & 0xFF
                                if chk == calc_chk:
                                    self.cmd_received.emit(wave, freq, vpp)
                                del self.recv_buffer[:8]
                            else:
                                self.recv_buffer.pop(0)

                    # 2. 向外高速发送 ADC 编码数据
                    if self.data_to_send is not None:
                        try:
                            self.client_socket.sendall(self.data_to_send)
                            self.data_to_send = None 
                        except Exception:
                            self.client_status.emit("🔴 上位机断开连接，重新等待中...")
                            self.client_socket.close()
                            self.client_socket = None
                            break
                            
                    time.sleep(0.005) 

            except Exception as e:
                if not self.running: break
        
        if self.server_socket:
            self.server_socket.close()

    def update_buffer(self, frame_bytes):
        self.data_to_send = frame_bytes

    def stop(self):
        self.running = False
        if self.client_socket: self.client_socket.close()
        if self.server_socket: self.server_socket.close()
        self.wait()


class SimulatorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32/MSPM0 硬件仿真模拟端")
        self.resize(1150, 600)
        
        self.channels = ['CH1', 'CH2', 'CH3', 'CH4']
        self.controls = {} 
        self.time_offset = 0.0 
        self.is_outputting = True
        self.sim_mode = "MCU" 
        
        self.last_received_cmd = None # 存储最新收到但还未应用的指令

        self.set_dark_style()
        self.init_ui()
        
        self.server_thread = TcpServerThread()
        self.server_thread.client_status.connect(self.lbl_status.setText)
        self.server_thread.cmd_received.connect(self.handle_remote_cmd)
        self.server_thread.start()

        self.sample_timer = QTimer(self)
        self.sample_timer.timeout.connect(self.generate_and_broadcast_dma)
        self.sample_timer.start(25) 

    def set_dark_style(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #181818; }
            QWidget { background-color: #181818; color: #e0e0e0; font-family: 'Consolas', sans-serif; }
            QGroupBox { font-weight: bold; border: 2px solid #2d2d2d; border-radius: 6px; margin-top: 10px; padding: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLabel { color: #aaaaaa; font-weight: bold; }
            QComboBox, QSpinBox, QDoubleSpinBox { background-color: #252526; border: 1px solid #3e3e42; color: #ffffff; padding: 4px; }
            QPushButton { border-radius: 4px; font-size: 14px; }
            QCheckBox { color: #4ec9b0; font-weight: bold; font-size: 13px; }
        """)

    def init_ui(self):
        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        # ====== 1. 顶部全局控制区 ======
        top_bar = QHBoxLayout()
        self.btn_output = QPushButton("总开关：停止信号发生 (STOP)")
        self.btn_output.setStyleSheet("background-color: #a61c1c; color: white; font-weight: bold; padding: 10px;")
        self.btn_output.clicked.connect(self.toggle_output)
        
        self.btn_mode = QPushButton("当前协议模式: 单片机仿真 (限制 0-3.3V, 8位)")
        self.btn_mode.setStyleSheet("background-color: #0e639c; color: white; font-weight: bold; padding: 10px;")
        self.btn_mode.clicked.connect(self.toggle_mode)

        self.lbl_status = QLabel("正在初始化服务器...")
        self.lbl_status.setStyleSheet("color: #007acc; font-size: 14px; margin-left: 20px;")

        top_bar.addWidget(self.btn_output)
        top_bar.addWidget(self.btn_mode)
        top_bar.addWidget(self.lbl_status)
        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        # ====== 2. 【全新新增】远程控制中心（信号路由） ======
        remote_group = QGroupBox("远程控制中心 (来自上位机的信号指令路由)")
        remote_layout = QHBoxLayout()
        
        self.lbl_remote_cmd = QLabel("暂未收到任何上位机指令。")
        self.lbl_remote_cmd.setStyleSheet("color: #dcdcaa; font-size: 14px;")
        remote_layout.addWidget(self.lbl_remote_cmd)
        
        remote_layout.addStretch()
        
        remote_layout.addWidget(QLabel("部署目标通道:"))
        self.combo_target_ch = QComboBox()
        self.combo_target_ch.addItems(self.channels)
        self.combo_target_ch.setStyleSheet("font-size: 14px; font-weight: bold; color: #4fc1ff;")
        remote_layout.addWidget(self.combo_target_ch)
        
        self.btn_apply_cmd = QPushButton("应用到选定通道")
        self.btn_apply_cmd.setEnabled(False) # 未收到指令前禁用
        self.btn_apply_cmd.setStyleSheet("background-color: #2d2d2d; color: #7f7f7f; font-weight: bold; padding: 8px 15px;")
        self.btn_apply_cmd.clicked.connect(self.apply_remote_cmd_to_channel)
        remote_layout.addWidget(self.btn_apply_cmd)
        
        remote_group.setLayout(remote_layout)
        main_layout.addWidget(remote_group)

        # ====== 3. 四通道并排布局 ======
        ch_layout = QHBoxLayout()
        colors = ['#4fc1ff', '#4ec9b0', '#c586c0', '#dcdcaa'] 

        for i, ch in enumerate(self.channels):
            group = QGroupBox(f"通道 {ch} 配置")
            group.setStyleSheet(f"QGroupBox::title {{ color: {colors[i]}; font-size: 15px; }}")
            grid = QGridLayout()

            chk_enable = QCheckBox("启用通道输出 (ON)")
            chk_enable.setChecked(True)
            grid.addWidget(chk_enable, 0, 0, 1, 2)

            grid.addWidget(QLabel("信号波形:"), 1, 0)
            combo_type = QComboBox()
            combo_type.addItems(["0: 正弦波", "1: 方波", "2: 三角波", "3: 锯齿波", "4: 直流电(DC)", "5: 白噪声(Noise)"])
            combo_type.setCurrentIndex(0 if i==0 else 4 if i==1 else 1 if i==2 else 5) 
            grid.addWidget(combo_type, 1, 1)

            grid.addWidget(QLabel("频率 (Hz):"), 2, 0)
            spin_freq = QSpinBox()
            spin_freq.setRange(1, 200000)
            spin_freq.setValue(1000 * (i + 1)) 
            grid.addWidget(spin_freq, 2, 1)

            grid.addWidget(QLabel("电压幅值 Vpp:"), 3, 0)
            spin_vpp = QDoubleSpinBox()
            spin_vpp.setRange(0.0, 100.0) 
            spin_vpp.setValue(3.0 - 0.5 * i)
            spin_vpp.setSingleStep(0.1)
            grid.addWidget(spin_vpp, 3, 1)

            grid.addWidget(QLabel("偏置电压 (V):"), 4, 0)
            spin_offset = QDoubleSpinBox()
            spin_offset.setRange(-50.0, 50.0) 
            spin_offset.setValue(1.65 if i != 1 else 0.0)
            grid.addWidget(spin_offset, 4, 1)

            group.setLayout(grid)
            ch_layout.addWidget(group)

            self.controls[ch] = {
                'enable': chk_enable,
                'type': combo_type,
                'freq': spin_freq,
                'vpp': spin_vpp,
                'offset': spin_offset
            }

        main_layout.addLayout(ch_layout)
        self.setCentralWidget(central_widget)

    def toggle_output(self):
        self.is_outputting = not self.is_outputting
        if self.is_outputting:
            self.btn_output.setText("⏹ 总开关：停止信号发生 (STOP)")
            self.btn_output.setStyleSheet("background-color: #a61c1c; color: white; font-weight: bold; padding: 10px;")
        else:
            self.btn_output.setText("▶ 总开关：启动信号发生 (RUN)")
            self.btn_output.setStyleSheet("background-color: #388a34; color: white; font-weight: bold; padding: 10px;")

    def toggle_mode(self):
        if self.sim_mode == "MCU":
            self.sim_mode = "SIGGEN"
            self.btn_mode.setText("当前协议模式: 专业信号源 (不限幅, 负压, 32位)")
            self.btn_mode.setStyleSheet("background-color: #722ed1; color: white; font-weight: bold; padding: 10px;")
        else:
            self.sim_mode = "MCU"
            self.btn_mode.setText("当前协议模式: 单片机仿真 (限制 0-3.3V, 8位)")
            self.btn_mode.setStyleSheet("background-color: #0e639c; color: white; font-weight: bold; padding: 10px;")

    @Slot(int, int, int)
    def handle_remote_cmd(self, wave, freq, vpp):
        """仅记录收到的指令并点亮UI提示，绝对不强行修改通道数据"""
        wave_names = ["正弦波", "方波", "三角波", "锯齿波"]
        wave_name = wave_names[wave] if 0 <= wave <= 3 else "未知波形"
        vpp_volt = (vpp / 255.0) * 3.3
        
        # 缓存指令数据
        self.last_received_cmd = {'wave': wave, 'freq': freq, 'vpp': vpp_volt}
        
        # 更新UI显示
        self.lbl_remote_cmd.setText(f"最新接收指令 —— 波形: {wave_name} | 频率: {freq} Hz | 幅值: {vpp_volt:.2f} V")
        self.lbl_remote_cmd.setStyleSheet("color: #4ec9b0; font-size: 15px; font-weight: bold;")
        
        # 激活部署按钮
        self.btn_apply_cmd.setEnabled(True)
        self.btn_apply_cmd.setStyleSheet("background-color: #388a34; color: white; font-weight: bold; padding: 8px 15px;")

    def apply_remote_cmd_to_channel(self):
        """将缓存的最新指令应用到用户下拉选中的通道"""
        if not self.last_received_cmd:
            return
            
        target_ch = self.combo_target_ch.currentText()
        cmd = self.last_received_cmd
        
        # 将指令写入对应通道的控件
        if 0 <= cmd['wave'] <= 3:
            self.controls[target_ch]['type'].setCurrentIndex(cmd['wave'])
        
        self.controls[target_ch]['freq'].setValue(cmd['freq'])
        self.controls[target_ch]['vpp'].setValue(cmd['vpp'])
        self.controls[target_ch]['offset'].setValue(1.65) # 恢复默认中点偏置
        self.controls[target_ch]['enable'].setChecked(True) # 强制开启该通道
        
        # UI 提示反馈
        self.lbl_status.setText(f"状态: 指令已成功路由至 {target_ch}！")

    def generate_and_broadcast_dma(self):
        if not self.server_thread.client_socket or not self.is_outputting:
            return

        num_samples = 200 
        fs = 500000 
        dt = 1.0 / fs
        t = np.linspace(0, num_samples * dt, num_samples, endpoint=False) + self.time_offset
        self.time_offset += num_samples * dt 

        v_dict = {}
        for ch in self.channels:
            ctrl = self.controls[ch]
            
            if not ctrl['enable'].isChecked():
                v_dict[ch] = np.zeros_like(t)
                continue

            w_type_idx = ctrl['type'].currentIndex()
            freq = ctrl['freq'].value()
            vpp = ctrl['vpp'].value()
            dc_offset = ctrl['offset'].value()
            amplitude = vpp / 2.0
            
            if w_type_idx == 0:   # 正弦
                v = dc_offset + amplitude * np.sin(2 * np.pi * freq * t)
            elif w_type_idx == 1: # 方波
                v = dc_offset + amplitude * np.sign(np.sin(2 * np.pi * freq * t))
            elif w_type_idx == 2: # 三角波
                v = dc_offset + (2 * amplitude / np.pi) * np.arcsin(np.sin(2 * np.pi * freq * t))
            elif w_type_idx == 3: # 锯齿波
                v = dc_offset + amplitude * (2 * (t * freq - np.floor(0.5 + t * freq)))
            elif w_type_idx == 4: # 直流
                v = np.full_like(t, dc_offset)
            else:                 # 白噪声
                v = dc_offset + amplitude * np.random.uniform(-1, 1, size=num_samples)
                
            v_dict[ch] = v

        if self.sim_mode == "MCU":
            packed_data = np.empty(num_samples * 4, dtype=np.uint8)
            for i, ch in enumerate(self.channels):
                v_clipped = np.clip(v_dict[ch], 0.0, 3.3)
                packed_data[i::4] = ((v_clipped / 3.3) * 255).astype(np.uint8)
            raw_bytes = packed_data.tobytes()
            header = bytes([0xAA, 0x55])
        else:
            packed_data = np.empty(num_samples * 4, dtype=np.float32)
            for i, ch in enumerate(self.channels):
                packed_data[i::4] = v_dict[ch].astype(np.float32)
            raw_bytes = packed_data.tobytes()
            header = bytes([0xAB, 0x55])

        data_len = len(raw_bytes)
        header += bytes([(data_len >> 8) & 0xFF, data_len & 0xFF])
        checksum = bytes([0x00]) 
        
        self.server_thread.update_buffer(header + raw_bytes + checksum)

    def closeEvent(self, event):
        self.sample_timer.stop()
        self.server_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SimulatorWindow()
    window.show()
    sys.exit(app.exec())
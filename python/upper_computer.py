import sys
import numpy as np
from collections import deque
from PySide6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QComboBox, 
                             QWidget, QLabel, QSpinBox, QDoubleSpinBox,
                             QMessageBox, QGroupBox, QTabWidget, QCheckBox)
from PySide6.QtCore import Signal, Slot, Qt, QThread, QDateTime, QTimer
from PySide6.QtGui import QMouseEvent, QWheelEvent
import pyqtgraph as pg
import serial
import serial.tools.list_ports

class TriggerButton(QPushButton):
    right_clicked = Signal()
    def __init__(self, text="", parent=None):
        super().__init__(text, parent)
    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.RightButton:
            self.right_clicked.emit()
        else:
            super().mousePressEvent(event)

class IndependentZoomPlot(pg.PlotWidget):
    def __init__(self, parent=None, **kargs):
        super().__init__(parent, **kargs)
        self.setMouseEnabled(x=True, y=True)
    def wheelEvent(self, event: QWheelEvent):
        delta = event.angleDelta().y()
        zoom_factor = 0.8 if delta > 0 else 1.25
        view_box = self.getViewBox()
        mouse_pos = view_box.mapSceneToView(event.position())
        if event.modifiers() & Qt.ShiftModifier:
            view_box.scaleBy(y=zoom_factor, center=mouse_pos)
        else:
            view_box.scaleBy(x=zoom_factor, center=mouse_pos)
        event.accept()

class SerialReadThread(QThread):
    data_received = Signal(dict)
    status_message = Signal(str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False

    def connect_port(self, port_name, baudrate=115200):
        try:
            self.serial_port = serial.Serial(port_name, baudrate, timeout=0.1)
            self.running = True
            return True
        except Exception as e:
            self.status_message.emit(f"连接失败: {str(e)}")
            return False

    def disconnect_port(self):
        self.running = False
        self.wait()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.status_message.emit("通讯链路已断开")

    def send_cmd(self, cmd_bytes):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(cmd_bytes)
            except Exception as e:
                self.status_message.emit(f"发送失败: {str(e)}")

    def run(self):
        buffer = bytearray()
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    waiting = self.serial_port.in_waiting
                    if waiting > 0:
                        buffer.extend(self.serial_port.read(waiting))
                        while len(buffer) >= 5:
                            # 協議 1: 0xAA 0x55 (8位单片机限幅模式)
                            if buffer[0] == 0xAA and buffer[1] == 0x55:
                                data_len = (buffer[2] << 8) | buffer[3]
                                
                                # 🟢 核心安全防護：如果解析出的長度異常巨大（顯然是雜訊干擾導致的錯位解析）
                                # 則直接視為無效數據，彈出 1 位元組引導緩衝區繼續尋找下一個正確的包頭，防止死鎖與閃退！
                                if data_len > 1000:
                                    buffer.pop(0)
                                    continue
                                    
                                total_frame_len = 5 + data_len
                                if len(buffer) >= total_frame_len:
                                    raw_data = list(buffer[4:4+data_len])
                                    adc_tensor = np.array(raw_data)
                                    
                                    # 🟢 健壯性修復：確保切片時不會因為陣列長度非4的倍數引發不對稱崩潰
                                    if len(adc_tensor) >= 4:
                                        ch1 = adc_tensor[0::4] * (3.3 / 255.0)
                                        ch2 = adc_tensor[1::4] * (3.3 / 255.0)
                                        ch3 = adc_tensor[2::4] * (3.3 / 255.0)
                                        ch4 = adc_tensor[3::4] * (3.3 / 255.0)
                                        self.data_received.emit({'CH1': ch1, 'CH2': ch2, 'CH3': ch3, 'CH4': ch4})
                                        
                                    del buffer[:total_frame_len]
                                else:
                                    break
                            # 协议 2: 0xAB 0x55 (32位专业信号源模式，任意量程/负压)
                            elif buffer[0] == 0xAB and buffer[1] == 0x55:
                                data_len = (buffer[2] << 8) | buffer[3]
                                total_frame_len = 5 + data_len
                                if len(buffer) >= total_frame_len:
                                    float_tensor = np.frombuffer(bytes(buffer[4:4+data_len]), dtype=np.float32)
                                    ch1 = float_tensor[0::4]
                                    ch2 = float_tensor[1::4]
                                    ch3 = float_tensor[2::4]
                                    ch4 = float_tensor[3::4]
                                    self.data_received.emit({'CH1': ch1, 'CH2': ch2, 'CH3': ch3, 'CH4': ch4})
                                    del buffer[:total_frame_len]
                                else:
                                    break
                            else:
                                buffer.pop(0)
                except Exception as e:
                    self.status_message.emit(f"读取错误: {str(e)}")
                    self.running = False
            QThread.msleep(2)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Studio")
        self.resize(1260, 800)
        
        self.is_running = True            
        self.single_armed = False         
        self.trig_mode_index = 0 
        self.ch_boxes = {} 
        self.force_reset_x = True  
        self.trigger_t_position = 0.5 
        
        self.max_buf_len = 200000
        self.data_history = {
            'CH1': deque(maxlen=self.max_buf_len),
            'CH2': deque(maxlen=self.max_buf_len),
            'CH3': deque(maxlen=self.max_buf_len),
            'CH4': deque(maxlen=self.max_buf_len)
        }
        for ch in self.data_history:
            self.data_history[ch].extend([1.65] * self.max_buf_len)

        self.backend_thread = SerialReadThread()
        self.backend_thread.data_received.connect(self.process_oscilloscope_data)
        self.backend_thread.status_message.connect(self.show_status)
        
        self.set_vscode_stylesheet()
        self.init_ui()
        self.refresh_ports()
        self.update_generator_preview()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.render_oscilloscope_plots)
        self.ui_timer.start(30) 

    def set_vscode_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e1e; }
            QWidget { background-color: #1e1e1e; color: #d4d4d4; font-family: 'Segoe UI', Helvetica, sans-serif; }
            QGroupBox { font-weight: bold; border: 1px solid #3c3c3c; border-radius: 4px; margin-top: 12px; padding-top: 12px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 3px; color: #569cd6; }
            QLabel { color: #85c5e5; }
            QComboBox, QSpinBox, QDoubleSpinBox { background-color: #3c3c3c; border: 1px solid #6b6b6b; color: #ffffff; padding: 4px; border-radius: 2px; }
            QPushButton { background-color: #0e639c; color: #ffffff; border: none; padding: 6px 12px; border-radius: 2px; font-weight: bold; }
            QPushButton:hover { background-color: #1177bb; }
            QPushButton:disabled { background-color: #2d2d2d; color: #7f7f7f; }
            QCheckBox::indicator { width: 14px; height: 14px; background-color: #3c3c3c; border: 1px solid #6b6b6b; }
            QCheckBox::indicator:checked { background-color: #0e639c; }
            QTabWidget::pane { border: 1px solid #2d2d2d; background-color: #1e1e1e; }
            QTabBar::tab { background: #2d2d2d; color: #969696; padding: 8px 20px; border: 1px solid #2d2d2d; border-bottom: none; }
            QTabBar::tab:selected { background: #1e1e1e; color: #ffffff; border-top: 3px solid #0e639c; }
        """)

    def init_ui(self):
        central_widget = QWidget()
        global_layout = QVBoxLayout(central_widget)
        global_layout.setContentsMargins(15, 10, 15, 15)
        
        top_bar = QHBoxLayout()
        top_bar.addWidget(QLabel("<b>连接设备:</b>"))
        self.combo_ports = QComboBox()
        self.combo_ports.setFixedWidth(160)
        top_bar.addWidget(self.combo_ports)
        
        self.btn_refresh = QPushButton("刷新连接")
        top_bar.addWidget(self.btn_refresh)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        
        self.btn_connect = QPushButton("连接下位机")
        self.btn_connect.setStyleSheet("background-color: #388a34;")
        top_bar.addWidget(self.btn_connect)
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        self.lbl_status = QLabel("状态: 未连接")
        self.lbl_status.setStyleSheet("color: #6a9955; font-weight: bold; margin-left: 20px;")
        top_bar.addWidget(self.lbl_status)
        top_bar.addStretch()
        global_layout.addLayout(top_bar)
        
        self.tab_widget = QTabWidget()
        self.page_generator = QWidget()
        self.page_oscilloscope = QWidget()
        self.tab_widget.addTab(self.page_generator, "  信号发生器 (Output)  ")
        self.tab_widget.addTab(self.page_oscilloscope, "  四通道时域示波器 (Input)  ")
        global_layout.addWidget(self.tab_widget)
        
        self.setup_generator_ui()
        self.setup_oscilloscope_ui()
        self.setCentralWidget(central_widget)

    def setup_generator_ui(self):
        layout = QHBoxLayout(self.page_generator)
        display_layout = QVBoxLayout()
        
        lbl = QLabel("<b>[ 信号发生器输出特征图 (精准单周期捕捉) ]</b>")
        lbl.setStyleSheet("color: #dcdcaa;")
        display_layout.addWidget(lbl)
        
        self.gen_plot = pg.PlotWidget()
        self.gen_plot.setBackground('#1e1e1e')
        self.gen_plot.showGrid(x=True, y=True, alpha=0.15)
        self.gen_plot.setMouseEnabled(x=False, y=False)
        self.gen_plot.getPlotItem().setMenuEnabled(False)
        self.gen_plot.setLabel('bottom', '周期时间域 (Time)', units='s')
        self.gen_plot.setLabel('left', '输出电压', units='V')
        self.gen_plot.setYRange(-0.3, 3.6)
        
        self.gen_curve = self.gen_plot.plot(pen=pg.mkPen('#ce9178', width=2))
        display_layout.addWidget(self.gen_plot, 4)
        
        self.lbl_gen_info = QLabel("当前配置 —— 波形: 正弦波")
        self.lbl_gen_info.setStyleSheet("color: #9cdcfe; background-color: #252526; padding: 8px; border-radius: 2px;")
        display_layout.addWidget(self.lbl_gen_info, 1)
        
        ctrl_panel = QVBoxLayout()
        ctrl_group = QGroupBox("参数配置面板")
        group_layout = QVBoxLayout()
        
        group_layout.addWidget(QLabel("波形选择:"))
        self.combo_wave = QComboBox()
        self.combo_wave.addItems(["正弦波", "方波", "三角波", "锯齿波"])
        self.combo_wave.currentIndexChanged.connect(self.update_generator_preview)
        group_layout.addWidget(self.combo_wave)
        
        group_layout.addWidget(QLabel("目标频率 (Hz):"))
        self.spin_freq = QSpinBox()
        self.spin_freq.setRange(1, 100000)
        self.spin_freq.setValue(1000)
        self.spin_freq.valueChanged.connect(self.update_generator_preview)
        group_layout.addWidget(self.spin_freq)
        
        group_layout.addWidget(QLabel("峰峰值 Vpp (最大3.3V):"))
        self.spin_amp = QDoubleSpinBox()
        self.spin_amp.setRange(0.0, 3.3)
        self.spin_amp.setSingleStep(0.1)
        self.spin_amp.setValue(3.3)
        self.spin_amp.valueChanged.connect(self.update_generator_preview)
        group_layout.addWidget(self.spin_amp)
        
        group_layout.addWidget(QLabel("直流偏置:"))
        self.lbl_offset_hint = QLabel("1.65 V (自动适配单电源)")
        self.lbl_offset_hint.setStyleSheet("color: #7f7f7f; padding-left: 5px;")
        group_layout.addWidget(self.lbl_offset_hint)
        
        self.btn_send_wave = QPushButton("🚀 部署波形到硬件")
        self.btn_send_wave.setEnabled(False)
        self.btn_send_wave.clicked.connect(self.send_wave_command)
        group_layout.addWidget(self.btn_send_wave)
        
        group_layout.addStretch()
        ctrl_group.setLayout(group_layout)
        ctrl_panel.addWidget(ctrl_group)
        layout.addLayout(display_layout, 3)
        layout.addLayout(ctrl_panel, 1)

    def setup_oscilloscope_ui(self):
        layout = QHBoxLayout(self.page_oscilloscope)
        self.osc_plot = IndependentZoomPlot()
        self.osc_plot.setBackground('#1e1e1e')
        self.osc_plot.showGrid(x=True, y=True, alpha=0.15)
        self.osc_plot.setYRange(-5.0, 5.0) # 为了支持信号源任意压，默认拉宽纵坐标
        self.osc_plot.setLabel('bottom', '时间 (Time)', units='s')
        self.osc_plot.setLabel('left', '电压 (Voltage)', units='V')
        
        self.colors = ['#4fc1ff', '#4ec9b0', '#c586c0', '#dcdcaa']
        self.curves = {}
        channels = ['CH1', 'CH2', 'CH3', 'CH4']
        for i, ch in enumerate(channels):
            self.curves[ch] = self.osc_plot.plot(pen=pg.mkPen(self.colors[i], width=2), name=ch)
            
        self.trigger_line = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('#f44336', width=1.5, style=Qt.DashLine))
        self.osc_plot.addItem(self.trigger_line)
        layout.addWidget(self.osc_plot, 3)
        
        ctrl_panel = QVBoxLayout()
        state_group = QGroupBox("捕获控制 (右键切模式)")
        state_layout = QHBoxLayout()
        
        self.btn_run_stop = QPushButton("RUN")
        self.btn_run_stop.setStyleSheet("background-color: #388a34; min-height: 32px;")
        self.btn_run_stop.clicked.connect(self.toggle_run_stop)
        state_layout.addWidget(self.btn_run_stop)
        
        self.btn_mode_toggle = TriggerButton("模式: AUTO")
        self.btn_mode_toggle.setStyleSheet("background-color: #1f4e79; min-height: 32px;")
        self.btn_mode_toggle.clicked.connect(self.on_mode_button_left_click)   
        self.btn_mode_toggle.right_clicked.connect(self.cycle_trigger_mode)   
        state_layout.addWidget(self.btn_mode_toggle)
        state_group.setLayout(state_layout)
        ctrl_panel.addWidget(state_group)
        
        ch_group = QGroupBox("通道可见性")
        ch_layout = QVBoxLayout()
        for ch in channels:
            cb = QCheckBox(f"显示 {ch}")
            cb.setChecked(True)
            ch_layout.addWidget(cb)
            self.ch_boxes[ch] = cb
        ch_group.setLayout(ch_layout)
        ctrl_panel.addWidget(ch_group)
        
        trigger_group = QGroupBox("触发同步 (Trigger)")
        trig_layout = QVBoxLayout()
        trig_layout.addWidget(QLabel("触发源通道:"))
        self.combo_trig_src = QComboBox()
        self.combo_trig_src.addItems(channels)
        trig_layout.addWidget(self.combo_trig_src)
        
        trig_layout.addWidget(QLabel("触发边沿:"))
        self.combo_trig_edge = QComboBox()
        self.combo_trig_edge.addItems(["上升沿 ↑", "下降沿 ↓"])
        trig_layout.addWidget(self.combo_trig_edge)
        
        trig_layout.addWidget(QLabel("触发电平 (V):"))
        self.spin_trig_volt = QDoubleSpinBox()
        self.spin_trig_volt.setRange(-100.0, 100.0) 
        self.spin_trig_volt.setValue(1.65) 
        self.spin_trig_volt.setSingleStep(0.05)
        self.spin_trig_volt.valueChanged.connect(self.on_trigger_voltage_changed)
        trig_layout.addWidget(self.spin_trig_volt)
        trigger_group.setLayout(trig_layout)
        ctrl_panel.addWidget(trigger_group)
        
        self.trigger_line.setValue(self.spin_trig_volt.value())
        
        time_group = QGroupBox("采集时基 (TimeBase)")
        time_layout = QVBoxLayout()
        time_layout.addWidget(QLabel("横向时基 (Time/Div):"))
        self.combo_timebase = QComboBox()
        self.combo_timebase.addItems([
            "50 us/div", "100 us/div", "200 us/div", "500 us/div", 
            "1 ms/div", "2 ms/div", "5 ms/div", "10 ms/div", "20 ms/div", "50 ms/div"
        ])
        self.timebase_values = [50e-6, 100e-6, 200e-6, 500e-6, 1e-3, 2e-3, 5e-3, 1e-2, 2e-2, 5e-2]
        self.combo_timebase.setCurrentIndex(4) 
        self.combo_timebase.currentIndexChanged.connect(self.on_timebase_changed)
        time_layout.addWidget(self.combo_timebase)
        
        time_layout.addWidget(QLabel("采样率配置:"))
        self.combo_rate = QComboBox()
        self.combo_rate.addItems(["100 KSPS", "200 KSPS", "500 KSPS", "1 MSPS"])
        time_layout.addWidget(self.combo_rate)
        time_group.setLayout(time_layout)
        ctrl_panel.addWidget(time_group)
        
        ctrl_panel.addStretch()
        layout.addLayout(ctrl_panel, 1)

    def on_trigger_voltage_changed(self, value):
        self.trigger_line.setValue(value)

    def on_timebase_changed(self, index):
        self.force_reset_x = True

    def cycle_trigger_mode(self):
        self.trig_mode_index = (self.trig_mode_index + 1) % 3
        if self.trig_mode_index == 0: 
            self.single_armed = False
            self.btn_mode_toggle.setText("模式: AUTO")
            self.btn_mode_toggle.setStyleSheet("background-color: #1f4e79; min-height: 32px;")
            self.lbl_status.setText("状态: 自动触发 (Auto)")
        elif self.trig_mode_index == 1: 
            self.single_armed = False
            self.btn_mode_toggle.setText("模式: NORMAL")
            self.btn_mode_toggle.setStyleSheet("background-color: #722ed1; min-height: 32px;")
            self.lbl_status.setText("状态: 普通触发 (Normal)")
        elif self.trig_mode_index == 2: 
            self.btn_mode_toggle.setText("模式: SINGLE")
            self.btn_mode_toggle.setStyleSheet("background-color: #d77a1e; min-height: 32px;")
            self.lbl_status.setText("状态: 单次触发，左键点击开始捕获")

    def on_mode_button_left_click(self):
        if self.trig_mode_index == 2:
            self.single_armed = True 
            self.is_running = True 
            self.update_run_stop_ui()
            self.lbl_status.setText("状态: Single 捕获已就绪，正在强力监测硬件边沿...")

    def toggle_run_stop(self):
        self.is_running = not self.is_running
        if not self.is_running:
            self.single_armed = False 
        self.update_run_stop_ui()

    def update_run_stop_ui(self):
        if self.is_running:
            self.btn_run_stop.setText("RUN")
            self.btn_run_stop.setStyleSheet("background-color: #388a34; min-height: 32px;")
        else:
            self.btn_run_stop.setText("STOP")
            self.btn_run_stop.setStyleSheet("background-color: #a61c1c; min-height: 32px;")

    @Slot(dict)
    def process_oscilloscope_data(self, channels_data):
        if not self.is_running: return
        for ch, data in channels_data.items():
            if len(data) > 0: self.data_history[ch].extend(data)

        if self.single_armed:
            trig_src = self.combo_trig_src.currentText()
            trig_level = self.spin_trig_volt.value()
            trig_edge = self.combo_trig_edge.currentText()
            target_data = channels_data.get(trig_src, np.array([]))
            
            if len(target_data) >= 2:
                if trig_edge == "上升沿 ↑": triggered = np.any((target_data[:-1] < trig_level) & (target_data[1:] >= trig_level))
                else: triggered = np.any((target_data[:-1] > trig_level) & (target_data[1:] <= trig_level))
                    
                if triggered:
                    self.single_armed = False
                    QTimer.singleShot(50, lambda: self.set_single_stop())

    def set_single_stop(self):
        self.is_running = False
        self.update_run_stop_ui()
        self.lbl_status.setText("状态: Single 触发成功！动态波形已捕获锁死 (STOP)")

    def render_oscilloscope_plots(self):
        """核心修复：绝不人为平移画面错乱触发，且时间轴绑定彻底分离"""
        if self.tab_widget.currentIndex() != 1: return
        if not self.is_running: return

        timebase = self.timebase_values[self.combo_timebase.currentIndex()]
        total_time = timebase * 10 
        
        rate_str = self.combo_rate.currentText()
        if "100" in rate_str: fs = 100000
        elif "200" in rate_str: fs = 200000
        elif "500" in rate_str: fs = 500000
        else: fs = 1000000

        if self.force_reset_x:
            self.osc_plot.setXRange(0, total_time, padding=0.0)
            self.force_reset_x = False

        view_box = self.osc_plot.getViewBox()
        x_range = view_box.viewRange()[0]
        view_xmin, view_xmax = x_range[0], x_range[1]

        trig_src = self.combo_trig_src.currentText()
        trig_level = self.spin_trig_volt.value()
        trig_edge = self.combo_trig_edge.currentText()
        
        y_trig_full = np.array(self.data_history[trig_src])
        buf_len = len(y_trig_full)

        # 触发寻址机制与边界保护
        t_trigger_physical = total_time * self.trigger_t_position
        t_end_relative = view_xmax - t_trigger_physical
        idx_end_offset = int(t_end_relative * fs)
        if idx_end_offset < 0: idx_end_offset = 0

        trig_idx = -1
        search_size = min(buf_len - 2, int(total_time * fs * 3))
        if search_size > 10:
            start_search = buf_len - search_size
            y_search = y_trig_full[start_search:]
            
            if trig_edge == "上升沿 ↑": condition = (y_search[:-1] < trig_level) & (y_search[1:] >= trig_level)
            else: condition = (y_search[:-1] > trig_level) & (y_search[1:] <= trig_level)
            
            match_indices = np.where(condition)[0]
            if len(match_indices) > 0:
                # 【关键修复】：如果找到的触发边沿，其右侧还没收到足够填满屏幕的数据，宁可等待也不重绘！
                valid_matches = [m for m in match_indices if (start_search + m + idx_end_offset) <= buf_len]
                if len(valid_matches) > 0: trig_idx = start_search + valid_matches[-1]
                else: return # 完全冻结，等待缓冲区积攒够新一帧的数据

        jitter_offset = 0
        if trig_idx == -1:
            if self.trig_mode_index == 0: 
                self.lbl_status.setText("状态: AUTO (未触发! 波形异步抖动中...)")
                self.lbl_status.setStyleSheet("color: #ffcc00; font-weight: bold;")
                jitter_offset = np.random.randint(-int(timebase * fs * 0.5), int(timebase * fs * 0.5))
                trig_idx = buf_len - int(total_time * fs) + jitter_offset
                if trig_idx < 0: trig_idx = 0
            elif self.trig_mode_index == 1: 
                self.lbl_status.setText("状态: NORMAL (未触发! 保持上一帧)")
                self.lbl_status.setStyleSheet("color: #f44336; font-weight: bold;")
                return  
            else: return
        else:
            if self.trig_mode_index == 0:
                self.lbl_status.setText("状态: AUTO (已触发同步)")
                self.lbl_status.setStyleSheet("color: #4ec9b0; font-weight: bold;")
            elif self.trig_mode_index == 1:
                self.lbl_status.setText("状态: NORMAL (已触发同步)")
                self.lbl_status.setStyleSheet("color: #4ec9b0; font-weight: bold;")

        # 生成与触发点绝对绑定的独立时间轴，防止画面人为强制平移
        t_start_relative = view_xmin - t_trigger_physical
        abs_start_idx = trig_idx + int(t_start_relative * fs)
        abs_end_idx = trig_idx + idx_end_offset

        if abs_start_idx < 0: abs_start_idx = 0
        if abs_end_idx > buf_len: abs_end_idx = buf_len

        raw_points = abs_end_idx - abs_start_idx
        if raw_points <= 0: return

        max_display_points = 2000
        step = max(1, raw_points // max_display_points)
        
        # 【时间轴分离修复】：时间轴的计算直接从绝对索引推导，无视任何左右切片干涉
        actual_indices = np.arange(abs_start_idx, abs_end_idx, step)
        t_axis = t_trigger_physical + (actual_indices - trig_idx) / fs

        for ch in ['CH1', 'CH2', 'CH3', 'CH4']:
            if self.ch_boxes[ch].isChecked():
                ch_array_full = np.array(self.data_history[ch])
                y_display = ch_array_full[abs_start_idx:abs_end_idx:step]
                
                display_len = min(len(t_axis), len(y_display))
                if display_len > 0:
                    self.curves[ch].setData(t_axis[:display_len], y_display[:display_len])
                    self.curves[ch].show()
            else:
                self.curves[ch].hide()

    def refresh_ports(self):
        self.combo_ports.clear()
        self.combo_ports.addItem("127.0.0.1:8888 (模拟器)")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)

    def toggle_connection(self):
        if not self.backend_thread.running:
            port_name = self.combo_ports.currentText()
            if not port_name:
                QMessageBox.warning(self, "警告", "请选择有效的通讯连接！")
                return
            
            if "127.0.0.1" in port_name:
                import socket
                class NetworkSerialAdapter:
                    def __init__(self):
                        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.sock.connect(('127.0.0.1', 8888))
                        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) 
                        self.sock.setblocking(False)
                        self.is_open = True
                    @property
                    def in_waiting(self):
                        import select
                        ready = select.select([self.sock], [], [], 0.001)
                        return 8192 if ready[0] else 0
                    def read(self, size):
                        try: return self.sock.recv(size)
                        except: return b""
                    def write(self, data):
                        try: self.sock.sendall(data)
                        except: pass
                    def close(self):
                        self.sock.close()
                        self.is_open = False
                try:
                    self.backend_thread.serial_port = NetworkSerialAdapter()
                    self.backend_thread.running = True
                    self.backend_thread.start()
                    self.btn_connect.setText("断开连接")
                    self.btn_connect.setStyleSheet("background-color: #a61c1c;")
                    self.btn_send_wave.setEnabled(True)
                    self.lbl_status.setText("已连入本地韧体模拟端")
                except Exception as e:
                    QMessageBox.critical(self, "错误", f"无法连入模拟端，请先开启模拟器软件！\n{str(e)}")
            else:
                if self.backend_thread.connect_port(port_name):
                    self.backend_thread.start()
                    self.btn_connect.setText("断开连接")
                    self.btn_connect.setStyleSheet("background-color: #a61c1c;")
                    self.btn_send_wave.setEnabled(True)
                    self.lbl_status.setText(f"已连接硬件串口 {port_name}")
        else:
            self.backend_thread.disconnect_port()
            self.btn_connect.setText("连接下位机")
            self.btn_connect.setStyleSheet("background-color: #388a34;")
            self.btn_send_wave.setEnabled(False)

    def update_generator_preview(self):
        wave_type = self.combo_wave.currentText()
        freq = self.spin_freq.value()
        vpp = self.spin_amp.value()
        
        offset = 1.65
        amplitude = vpp / 2.0
        period = 1.0 / freq
        t = np.linspace(0, period, 500)
        
        if wave_type == "正弦波": voltages = offset + amplitude * np.sin(2 * np.pi * freq * t)
        elif wave_type == "方波": voltages = offset + amplitude * np.sign(np.sin(2 * np.pi * freq * t))
        elif wave_type == "三角波": voltages = offset + (2 * amplitude / np.pi) * np.arcsin(np.sin(2 * np.pi * freq * t))
        elif wave_type == "锯齿波": voltages = offset + amplitude * (2 * (t * freq - np.floor(0.5 + t * freq)))
            
        voltages = np.clip(voltages, 0.0, 3.3)
        self.gen_plot.setXRange(0, period, padding=0.02)
        self.gen_curve.setData(t, voltages)
        self.lbl_gen_info.setText(f"真实硬件输出预期 —— 波形: {wave_type} | 频率: {freq} Hz | 周期: {period*1000:.3f} ms")

    def send_wave_command(self):
        wave_idx = self.combo_wave.currentIndex()
        
        # 強制轉換為 int，防止 QDoubleSpinBox 引入 float 導致位移崩潰
        freq = int(self.spin_freq.value()) 
        
        freq_high, freq_low = (freq >> 8) & 0xFF, freq & 0xFF
        vpp_int = int((self.spin_amp.value() / 3.3) * 255) & 0xFF
        checksum = (0x01 + wave_idx + freq_high + freq_low + vpp_int) & 0xFF
        
        # 構建 8 位元組標準控制訊號幀
        cmd_frame = bytes([0x5A, 0xA5, 0x01, wave_idx, freq_high, freq_low, vpp_int, checksum])

        # 🟢 修复：直接使用后台读写线程已经建立好的方法发送命令
        self.backend_thread.send_cmd(cmd_frame)
        print(f"【发送成功】控制讯号帧: {cmd_frame.hex().upper()}")

    @Slot(str)
    def show_status(self, text):
        self.lbl_status.setText(f"状态: {text}")
        if "失败" in text or "错误" in text:
            self.toggle_connection()

    def closeEvent(self, event):
        self.ui_timer.stop()
        self.backend_thread.disconnect_port()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
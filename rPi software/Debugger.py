import sys
import collections

import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets, QtCore, QtGui
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Maximum data points to retain on the plot buffer
MAX_HISTORY = 300

# Mapping message types to their respective CSV fields
FIELD_MAP = {
    "$IMU":    ["Gyro X", "Gyro Y", "Gyro Z", "Accel X", "Accel Y", "Accel Z", "Roll", "Pitch", "Yaw"],
    "$DIST":   ["Dist 0 (mm)", "Dist 1 (mm)", "Dist 2 (mm)", "Dist 3 (mm)"],
    "$COLOR":  ["Lux", "Red %", "Green %", "Blue %"],
    "$ADC":    ["ADC 0 (%)", "ADC 1 (%)", "ADC 2 (%)"],
    "$BUTTON": ["Button 1", "Button 2"]
}

# ---- Global vars for PCA servo motion mode & speed ----
# mode: 1 = smooth, 2 = direct, 3 = non-blocking
mode = 1
speed_val_for_pca = 100


class SerialHandler(QtCore.QThread):
    """Thread handling non-blocking serial read & write operations."""

    line_received = QtCore.pyqtSignal(str)

    def __init__(self, port, baudrate=921600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.running:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.line_received.emit(line)
            self.ser.close()
        except Exception as e:
            print(f"Serial Error: {e}")

    def send_command(self, cmd: str):
        """Send command string over serial line."""
        if self.ser and self.ser.is_open:
            full_cmd = (cmd.strip() + "\n").encode('utf-8')
            self.ser.write(full_cmd)

    def stop(self):
        self.running = False
        self.wait()


class MplCanvas(FigureCanvas):
    """Matplotlib Canvas Widget embedded into PyQt5."""

    def __init__(self, parent=None, width=8, height=6, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.ax.set_title("Real-Time Telemetry Plot")
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("Value")
        self.ax.grid(True)


class ColorSwatch(QtWidgets.QWidget):
    """Large color box driven by R/G/B percentages (0-100)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self._rgb = (0, 0, 0)

    def set_percent(self, r_pct, g_pct, b_pct):
        def conv(v):
            return max(0, min(255, int(round(v * 2.55))))
        self._rgb = (conv(r_pct), conv(g_pct), conv(b_pct))
        self.update()

    def rgb(self):
        return self._rgb

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect().adjusted(1, 1, -1, -1)
        p.setBrush(QtGui.QColor(*self._rgb))
        p.setPen(QtGui.QPen(QtGui.QColor(60, 60, 60), 2))
        p.drawRoundedRect(rect, 10, 10)


class SerialPlotterApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Embedded Telemetry & Control Dashboard")
        self.resize(1200, 800)

        # Storage buffers for time-series data
        self.data_buffers = {}
        self.checkboxes = {}
        self.serial_thread = None
        self.led1_state = False
        self.led2_state = False

        self.init_buffers()
        self.init_ui()

        # Update plot timer (30 FPS refresh rate)
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.setInterval(33)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start()

    def init_buffers(self):
        """Initialize data queues for every variable field."""
        for msg, fields in FIELD_MAP.items():
            self.data_buffers[msg] = {}
            for field in fields:
                self.data_buffers[msg][field] = collections.deque(maxlen=MAX_HISTORY)

    def init_ui(self):
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QVBoxLayout(main_widget)

        # --- Top Connection Bar ---
        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Serial Port:"))

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()
        top_bar.addWidget(self.port_combo)

        self.btn_refresh = QtWidgets.QPushButton("Refresh Ports")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        top_bar.addWidget(self.btn_refresh)

        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        top_bar.addWidget(self.btn_connect)

        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        # --- Tab Widget ---
        self.tabs = QtWidgets.QTabWidget()
        main_layout.addWidget(self.tabs)

        # Tab 1: Telemetry Plotter
        tab_telemetry = QtWidgets.QWidget()
        self.init_telemetry_tab(tab_telemetry)
        self.tabs.addTab(tab_telemetry, "Telemetry Plotter")

        # Tab 2: Outputs Control
        tab_outputs = QtWidgets.QWidget()
        self.init_outputs_tab(tab_outputs)
        self.tabs.addTab(tab_outputs, "Outputs")

        # Tab 3: Movement Control
        tab_move = QtWidgets.QWidget()
        self.init_move_tab(tab_move)
        self.tabs.addTab(tab_move, "Move")

        # Tab 4: Color Visualizer
        tab_color = QtWidgets.QWidget()
        self.init_color_tab(tab_color)
        self.tabs.addTab(tab_color, "Color Sensor")

    # ================= Telemetry Tab =================
    def init_telemetry_tab(self, parent):
        layout = QtWidgets.QHBoxLayout(parent)

        # Left Control Panel
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout(control_panel)
        control_panel.setMaximumWidth(320)

        sel_group = QtWidgets.QGroupBox("Plot Selectors")
        sel_layout = QtWidgets.QVBoxLayout(sel_group)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QtWidgets.QWidget()
        scroll_layout = QtWidgets.QVBoxLayout(scroll_content)

        for msg, fields in FIELD_MAP.items():
            msg_box = QtWidgets.QGroupBox(msg)
            msg_box_layout = QtWidgets.QVBoxLayout(msg_box)
            self.checkboxes[msg] = {}
            for field in fields:
                cb = QtWidgets.QCheckBox(field)
                cb.setChecked(False)
                self.checkboxes[msg][field] = cb
                msg_box_layout.addWidget(cb)
            scroll_layout.addWidget(msg_box)

        scroll_content.setLayout(scroll_layout)
        scroll.setWidget(scroll_content)
        sel_layout.addWidget(scroll)
        control_layout.addWidget(sel_group)

        btn_clear = QtWidgets.QPushButton("Clear Buffers")
        btn_clear.clicked.connect(self.clear_buffers)
        control_layout.addWidget(btn_clear)

        # Right Plot Area
        self.canvas = MplCanvas(self)

        layout.addWidget(control_panel)
        layout.addWidget(self.canvas)

    # ================= Outputs Tab =================
    def init_outputs_tab(self, parent):
        layout = QtWidgets.QVBoxLayout(parent)

        # 1. LED Controls Section
        led_group = QtWidgets.QGroupBox("LED Control")
        led_layout = QtWidgets.QHBoxLayout(led_group)

        self.btn_led1_on = QtWidgets.QPushButton("LED 1 ON")
        self.btn_led1_off = QtWidgets.QPushButton("LED 1 OFF")
        self.btn_led2_on = QtWidgets.QPushButton("LED 2 ON")
        self.btn_led2_off = QtWidgets.QPushButton("LED 2 OFF")

        self.btn_led1_on.clicked.connect(lambda: self.set_led(1, True))
        self.btn_led1_off.clicked.connect(lambda: self.set_led(1, False))
        self.btn_led2_on.clicked.connect(lambda: self.set_led(2, True))
        self.btn_led2_off.clicked.connect(lambda: self.set_led(2, False))

        led_layout.addWidget(self.btn_led1_on)
        led_layout.addWidget(self.btn_led1_off)
        led_layout.addSpacing(20)
        led_layout.addWidget(self.btn_led2_on)
        led_layout.addWidget(self.btn_led2_off)
        led_layout.addStretch()
        layout.addWidget(led_group)

        # 2. Buzzer Control Section
        buzzer_group = QtWidgets.QGroupBox("Buzzer Control ($BEEP)")
        buzzer_layout = QtWidgets.QHBoxLayout(buzzer_group)

        buzzer_layout.addWidget(QtWidgets.QLabel("Frequency (Hz):"))
        self.spin_freq = QtWidgets.QSpinBox()
        self.spin_freq.setRange(50, 10000)
        self.spin_freq.setValue(1000)
        self.spin_freq.setSingleStep(100)
        buzzer_layout.addWidget(self.spin_freq)

        buzzer_layout.addWidget(QtWidgets.QLabel("Duration (ms):"))
        self.spin_dur = QtWidgets.QSpinBox()
        self.spin_dur.setRange(10, 5000)
        self.spin_dur.setValue(200)
        self.spin_dur.setSingleStep(50)
        buzzer_layout.addWidget(self.spin_dur)

        btn_play_beep = QtWidgets.QPushButton("Trigger Beep")
        btn_play_beep.clicked.connect(self.send_beep_cmd)
        buzzer_layout.addWidget(btn_play_beep)

        btn_preset_alert = QtWidgets.QPushButton("Preset: Alert (2kHz / 100ms)")
        btn_preset_alert.clicked.connect(lambda: self.send_custom_beep(2000, 100))
        buzzer_layout.addWidget(btn_preset_alert)

        btn_preset_error = QtWidgets.QPushButton("Preset: Low Warning (400Hz / 400ms)")
        btn_preset_error.clicked.connect(lambda: self.send_custom_beep(400, 400))
        buzzer_layout.addWidget(btn_preset_error)

        buzzer_layout.addStretch()
        layout.addWidget(buzzer_group)

        # 3. Servo Controls Section (16 Channels)
        servo_group = QtWidgets.QGroupBox("PCA9685 Servos (0 - 180°)")
        servo_main_layout = QtWidgets.QVBoxLayout(servo_group)

        # --- Motion mode selector: smooth / direct / non-blocking (mutually exclusive) ---
        mode_layout = QtWidgets.QHBoxLayout()
        mode_layout.addWidget(QtWidgets.QLabel("Motion Mode:"))

        self.mode_button_group = QtWidgets.QButtonGroup(self)
        self.mode_button_group.setExclusive(True)

        self.btn_mode_smooth = QtWidgets.QPushButton("Smooth")
        self.btn_mode_direct = QtWidgets.QPushButton("Direct")
        self.btn_mode_nonblocking = QtWidgets.QPushButton("Non-Blocking")

        for btn in (self.btn_mode_smooth, self.btn_mode_direct, self.btn_mode_nonblocking):
            btn.setCheckable(True)
            mode_layout.addWidget(btn)

        self.mode_button_group.addButton(self.btn_mode_smooth, 1)
        self.mode_button_group.addButton(self.btn_mode_direct, 2)
        self.mode_button_group.addButton(self.btn_mode_nonblocking, 3)

        self.btn_mode_smooth.setChecked(True)  # default mode = 1 (smooth)

        self.mode_button_group.idClicked.connect(self.set_pca_mode)

        mode_layout.addStretch()
        servo_main_layout.addLayout(mode_layout)

        # --- Master speed slider (1-200) ---
        speed_layout = QtWidgets.QHBoxLayout()
        speed_layout.addWidget(QtWidgets.QLabel("Master Speed:"))

        self.slider_pca_speed = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_pca_speed.setRange(1, 200)
        self.slider_pca_speed.setValue(speed_val_for_pca)
        speed_layout.addWidget(self.slider_pca_speed)

        self.lbl_pca_speed = QtWidgets.QLabel(str(speed_val_for_pca))
        self.lbl_pca_speed.setMinimumWidth(35)
        speed_layout.addWidget(self.lbl_pca_speed)

        self.slider_pca_speed.valueChanged.connect(self.set_pca_speed)

        servo_main_layout.addLayout(speed_layout)

        scroll_servo = QtWidgets.QScrollArea()
        scroll_servo.setWidgetResizable(True)
        servo_content = QtWidgets.QWidget()
        servo_grid = QtWidgets.QGridLayout(servo_content)

        self.servo_sliders = []
        self.servo_labels = []

        for ch in range(16):
            row = ch // 2
            col_offset = (ch % 2) * 4

            lbl_title = QtWidgets.QLabel(f"<b>Ch {ch}:</b>")
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(90)
            lbl_val = QtWidgets.QLabel("90°")
            lbl_val.setMinimumWidth(35)

            slider.valueChanged.connect(lambda val, l=lbl_val: l.setText(f"{val}°"))

            btn_update = QtWidgets.QPushButton("Update")
            btn_update.clicked.connect(lambda _, c=ch, s=slider: self.send_servo_cmd(c, s.value()))

            servo_grid.addWidget(lbl_title, row, col_offset)
            servo_grid.addWidget(slider, row, col_offset + 1)
            servo_grid.addWidget(lbl_val, row, col_offset + 2)
            servo_grid.addWidget(btn_update, row, col_offset + 3)

            self.servo_sliders.append(slider)
            self.servo_labels.append(lbl_val)

        servo_content.setLayout(servo_grid)
        scroll_servo.setWidget(servo_content)
        servo_main_layout.addWidget(scroll_servo)

        batch_layout = QtWidgets.QHBoxLayout()
        btn_all_90 = QtWidgets.QPushButton("Set All to 90°")
        btn_all_90.clicked.connect(self.set_all_servos_90)
        batch_layout.addWidget(btn_all_90)
        batch_layout.addStretch()
        servo_main_layout.addLayout(batch_layout)

        layout.addWidget(servo_group)

    # ================= Move Tab =================
    def init_move_tab(self, parent):
        layout = QtWidgets.QVBoxLayout(parent)

        # Translation ($MOVE)
        move_group = QtWidgets.QGroupBox("Translation ($MOVE,x-speed,y-speed)")
        move_layout = QtWidgets.QHBoxLayout(move_group)

        move_layout.addWidget(QtWidgets.QLabel("X speed:"))
        self.spin_x = QtWidgets.QSpinBox()
        self.spin_x.setRange(-100, 100)
        self.spin_x.setValue(0)
        self.spin_x.setSingleStep(5)
        move_layout.addWidget(self.spin_x)

        move_layout.addWidget(QtWidgets.QLabel("Y speed:"))
        self.spin_y = QtWidgets.QSpinBox()
        self.spin_y.setRange(-100, 100)
        self.spin_y.setValue(0)
        self.spin_y.setSingleStep(5)
        move_layout.addWidget(self.spin_y)

        btn_move = QtWidgets.QPushButton("Update X / Y")
        btn_move.clicked.connect(self.send_move_cmd)
        move_layout.addWidget(btn_move)

        btn_stop = QtWidgets.QPushButton("Stop (0,0)")
        btn_stop.clicked.connect(self.send_stop_cmd)
        move_layout.addWidget(btn_stop)

        move_layout.addStretch()
        layout.addWidget(move_group)

        # Rotation ($ROTATE)
        rot_group = QtWidgets.QGroupBox("Rotation ($ROTATE,speed,0)")
        rot_layout = QtWidgets.QHBoxLayout(rot_group)

        rot_layout.addWidget(QtWidgets.QLabel("Rotation speed:"))
        self.spin_rot = QtWidgets.QSpinBox()
        self.spin_rot.setRange(-100, 100)
        self.spin_rot.setValue(0)
        self.spin_rot.setSingleStep(5)
        rot_layout.addWidget(self.spin_rot)

        btn_rot = QtWidgets.QPushButton("Update Rotation")
        btn_rot.clicked.connect(self.send_rotate_cmd)
        rot_layout.addWidget(btn_rot)

        btn_rot_stop = QtWidgets.QPushButton("Stop Rotation")
        btn_rot_stop.clicked.connect(lambda: self.send_command("$ROTATE,0,0"))
        rot_layout.addWidget(btn_rot_stop)

        rot_layout.addStretch()
        layout.addWidget(rot_group)

        layout.addStretch()

    # ================= Color Visualizer Tab =================
    def init_color_tab(self, parent):
        layout = QtWidgets.QHBoxLayout(parent)

        # Left: field selectors for the $COLOR message
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(320)
        panel_layout = QtWidgets.QVBoxLayout(panel)

        group = QtWidgets.QGroupBox("$COLOR channels to visualize")
        group_layout = QtWidgets.QVBoxLayout(group)

        self.color_checkboxes = {}
        for field in ["Red %", "Green %", "Blue %"]:
            cb = QtWidgets.QCheckBox(field)
            cb.setChecked(True)
            self.color_checkboxes[field] = cb
            group_layout.addWidget(cb)

        panel_layout.addWidget(group)

        self.lbl_lux = QtWidgets.QLabel("Lux: --")
        panel_layout.addWidget(self.lbl_lux)

        self.lbl_rgb_pct = QtWidgets.QLabel("R: -- %   G: -- %   B: -- %")
        panel_layout.addWidget(self.lbl_rgb_pct)

        self.lbl_rgb_255 = QtWidgets.QLabel("RGB: --, --, --")
        panel_layout.addWidget(self.lbl_rgb_255)

        self.lbl_hex = QtWidgets.QLabel("HEX: --")
        panel_layout.addWidget(self.lbl_hex)

        panel_layout.addStretch()

        # Right: live color box (replaces the graph on this tab)
        self.color_swatch = ColorSwatch()

        layout.addWidget(panel)
        layout.addWidget(self.color_swatch, stretch=1)

    def update_color_view(self):
        buf = self.data_buffers["$COLOR"]

        def last(field):
            d = buf[field]
            return d[-1] if d else 0.0

        r = last("Red %") if self.color_checkboxes["Red %"].isChecked() else 0.0
        g = last("Green %") if self.color_checkboxes["Green %"].isChecked() else 0.0
        b = last("Blue %") if self.color_checkboxes["Blue %"].isChecked() else 0.0

        self.color_swatch.set_percent(r, g, b)
        cr, cg, cb = self.color_swatch.rgb()

        lux = buf["Lux"][-1] if buf["Lux"] else None
        self.lbl_lux.setText(f"Lux: {lux:.1f}" if lux is not None else "Lux: --")
        self.lbl_rgb_pct.setText(f"R: {r:.1f} %   G: {g:.1f} %   B: {b:.1f} %")
        self.lbl_rgb_255.setText(f"RGB: {cr}, {cg}, {cb}")
        self.lbl_hex.setText(f"HEX: #{cr:02X}{cg:02X}{cb:02X}")

    # ================= Command Handlers =================
    def set_led(self, led_num, state):
        if led_num == 1:
            self.led1_state = state
        elif led_num == 2:
            self.led2_state = state
        cmd = f"$LED,{1 if self.led1_state else 0},{1 if self.led2_state else 0}"
        self.send_command(cmd)

    def send_beep_cmd(self):
        self.send_command(f"$BEEP,{self.spin_freq.value()},{self.spin_dur.value()}")

    def send_custom_beep(self, freq, dur):
        self.send_command(f"$BEEP,{freq},{dur}")

    def set_pca_mode(self, mode_id):
        """Update the global PCA servo motion mode (1=smooth, 2=direct, 3=non-blocking)."""
        global mode
        mode = mode_id
        print(f"PCA motion mode set to: {mode}")

    def set_pca_speed(self, value):
        """Update the global PCA servo master speed value."""
        global speed_val_for_pca
        speed_val_for_pca = value
        self.lbl_pca_speed.setText(str(value))

    def send_servo_cmd(self, channel, angle):
        self.send_command(f"$SERVO,{channel},{angle},{mode},{speed_val_for_pca}")

    def set_all_servos_90(self):
        for ch, slider in enumerate(self.servo_sliders):
            slider.setValue(90)
            self.send_servo_cmd(ch, 90)

    def send_move_cmd(self):
        self.send_command(f"$MOVE,{self.spin_x.value()},{self.spin_y.value()}")

    def send_stop_cmd(self):
        self.spin_x.setValue(0)
        self.spin_y.setValue(0)
        self.send_command("$MOVE,0,0")

    def send_rotate_cmd(self):
        self.send_command(f"$ROTATE,{self.spin_rot.value()},0")

    def send_command(self, cmd):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.send_command(cmd)
            print(f"Sent: {cmd}")
        else:
            print(f"Not Connected. Command dropped: {cmd}")

    # ================= Serial & UI Helpers =================
    def refresh_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(p.device)

    def toggle_connection(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread = None
            self.btn_connect.setText("Connect")
            self.port_combo.setEnabled(True)
        else:
            port = self.port_combo.currentText()
            if not port:
                return
            self.serial_thread = SerialHandler(port, baudrate=921600)
            self.serial_thread.line_received.connect(self.parse_line)
            self.serial_thread.start()
            self.btn_connect.setText("Disconnect")
            self.port_combo.setEnabled(False)

    def parse_line(self, line):
        tokens = line.split(',')
        if not tokens:
            return
        header = tokens[0]
        if header in FIELD_MAP:
            fields = FIELD_MAP[header]
            for i, field in enumerate(fields):
                if i + 1 < len(tokens):
                    try:
                        val = float(tokens[i + 1])
                        self.data_buffers[header][field].append(val)
                    except ValueError:
                        pass

    def clear_buffers(self):
        for msg in self.data_buffers:
            for field in self.data_buffers[msg]:
                self.data_buffers[msg][field].clear()

    def update_plot(self):
        current = self.tabs.tabText(self.tabs.currentIndex())

        if current == "Color Sensor":
            self.update_color_view()
            return

        if current != "Telemetry Plotter":
            return

        self.canvas.ax.cla()
        self.canvas.ax.grid(True)
        self.canvas.ax.set_title("Real-Time Telemetry Plot")
        self.canvas.ax.set_xlabel("Samples")
        self.canvas.ax.set_ylabel("Value")

        plotted = False
        for msg, fields in self.checkboxes.items():
            for field, cb in fields.items():
                if cb.isChecked():
                    data = list(self.data_buffers[msg][field])
                    if data:
                        self.canvas.ax.plot(data, label=f"{msg} -> {field}")
                        plotted = True

        if plotted:
            self.canvas.ax.legend(loc="upper left")
        self.canvas.draw()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SerialPlotterApp()
    window.show()
    sys.exit(app.exec_())

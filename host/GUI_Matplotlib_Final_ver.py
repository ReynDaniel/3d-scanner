"""
3D Scanner — Host GUI Application
Daniel Reynolds — REYN Consultancy / La Trobe University, 2024

PyQt5 desktop application for controlling the 3D scanner and visualising
scan data in real time. Communicates with the Pico W firmware over BLE
using the Nordic UART Service (NUS) via the Bleak async library.

Features:
    - BLE device discovery and connection (scans for "DR_PICOW")
    - Configurable scan parameters: pan/tilt range, increment, sensor type
    - Real-time 3D point cloud visualisation (Matplotlib embedded in Qt)
    - Multi-position scanning: 4 corner positions with coordinate transforms
    - Outlier removal (Z-score), data averaging, and mesh detail control
    - Delaunay triangulation and convex hull surface rendering
    - CSV export (raw and processed data)
    - Manual servo control via sliders

Dependencies:
    pip install PyQt5 matplotlib numpy scipy bleak

Usage:
    python GUI_Matplotlib_Final_ver.py
"""

import os
import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QPushButton, QFormLayout, QLabel,
    QLineEdit, QTabWidget, QWidget, QHBoxLayout, QCheckBox, QComboBox,
    QPlainTextEdit, QSizePolicy, QGridLayout, QFileDialog, QFrame, QSplitter,
    QMessageBox, QSlider
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import pyqtSignal, QMetaObject, Qt, Q_ARG, QTimer
import traceback
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np
import matplotlib
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from bleak import BleakClient, BleakScanner
import asyncio
from threading import Thread
import math
import csv
from datetime import datetime
from scipy.spatial import Delaunay, ConvexHull

# BLE UUIDs — Nordic UART Service
UART_TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'  # Notify (Pico → PC)
UART_RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'  # Write  (PC → Pico)
DEVICE_NAME  = 'DR_PICOW'


def create_simulation_room(x_size, y_size, z_size, grid_size):
    """Generate floor grid meshgrid for 3D room visualisation."""
    x = np.linspace(0, x_size, grid_size)
    y = np.linspace(0, y_size, grid_size)
    x, y = np.meshgrid(x, y)
    z = np.zeros_like(x)
    return x, y, z


class OutputRedirector:
    """Redirect stdout to a PyQt QPlainTextEdit widget."""
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, message):
        message = message.strip()
        if message:
            QMetaObject.invokeMethod(
                self.text_widget, "appendPlainText",
                Qt.QueuedConnection, Q_ARG(str, message)
            )

    def flush(self):
        pass


class MatplotlibCanvas(FigureCanvas):
    """Matplotlib 3D canvas embedded in PyQt5."""
    def __init__(self, parent=None):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        super().__init__(self.fig)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.updateGeometry()

    def plot_room(self, x, y, z, x_size, y_size, z_size):
        self.ax.clear()
        self.ax.plot_surface(x, y, z, color='lightblue', alpha=0.3)
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.ax.set_xlim([0, x_size])
        self.ax.set_ylim([0, y_size])
        self.ax.set_zlim([0, z_size])
        self.draw()


class ScannerApp(QMainWindow):
    """Main application window for the 3D scanner GUI."""

    connection_status_signal = pyqtSignal(str)
    data_received_signal = pyqtSignal(str)

    # Angle limits (constrained from firmware max to protect mechanism)
    PAN_ANGLE_MIN  = 0
    PAN_ANGLE_MAX  = 180
    TILT_ANGLE_MIN = 0
    TILT_ANGLE_MAX = 40

    DEFAULT_RAW_CSV_PATH       = 'raw_data.csv'
    DEFAULT_PROCESSED_CSV_PATH = 'processed_data.csv'

    def __init__(self):
        super().__init__()
        self.setWindowTitle('3D Scanner GUI - Matplotlib')
        self.setGeometry(100, 100, 1200, 600)
        self.setMinimumSize(1200, 800)

        # BLE state
        self.ble_client    = None
        self.ble_connected = False
        self.pointer_state = False

        # Data
        self.data_list          = []
        self.csv_file           = None
        self.csv_writer         = None
        self.processed_data     = False
        self.processed_x_vals   = None
        self.processed_y_vals   = None
        self.processed_z_vals   = None
        self.processed_run_ids  = None

        # Scan start angles (used for coordinate adjustment)
        self.pan_start_angle  = 0
        self.tilt_start_angle = 0
        self.scanner_height   = 200

        self.raw_csv_path       = self.DEFAULT_RAW_CSV_PATH
        self.processed_csv_path = self.DEFAULT_PROCESSED_CSV_PATH

        # Build layout
        main_layout = QVBoxLayout()
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.central_widget.setLayout(main_layout)

        splitter            = QSplitter(Qt.Horizontal)
        left_frame_splitter = QSplitter(Qt.Vertical)
        left_frame          = QFrame()
        left_layout         = QVBoxLayout()
        left_frame.setLayout(left_layout)

        # Tabs
        self.tabs = QTabWidget()
        self.room_grid_tab          = QWidget()
        self.run_control_tab        = QWidget()
        self.simulation_control_tab = QWidget()
        self.servo_control_tab      = QWidget()

        self.tabs.addTab(self.room_grid_tab,          "Room Grid")
        self.tabs.addTab(self.run_control_tab,        "Run")
        self.tabs.addTab(self.simulation_control_tab, "Simulation")
        self.tabs.addTab(self.servo_control_tab,      "Servo Angles")

        self.setup_room_grid_tab()
        self.setup_run_control_tab()
        self.setup_simulation_control_tab()
        self.setup_servo_control_tab()

        left_layout.addWidget(self.tabs)

        # Console
        self.console_output = QPlainTextEdit(self)
        self.console_output.setReadOnly(True)
        self.console_output.setMinimumHeight(800)
        self.console_output.setMinimumWidth(200)
        self.console_output.setFont(QFont("Courier New", 10))
        left_layout.addWidget(self.console_output)

        self.data_received_signal.connect(self.update_console_output)
        sys.stdout = OutputRedirector(self.console_output)
        print("Console ready.")

        left_frame_splitter.addWidget(self.tabs)
        left_frame_splitter.addWidget(self.console_output)
        left_frame_splitter.setStretchFactor(0, 3)
        left_frame_splitter.setStretchFactor(1, 1)
        left_layout.addWidget(left_frame_splitter)

        # Right side — 3D canvas
        right_frame  = QFrame()
        right_layout = QVBoxLayout()
        right_frame.setLayout(right_layout)
        self.canvas = MatplotlibCanvas(self)
        right_layout.addWidget(self.canvas)

        splitter.addWidget(left_frame)
        splitter.addWidget(right_frame)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 3)
        main_layout.addWidget(splitter)

        self.run_id = 1
        self.display_room_grid()

    # ─────────────────────────────────────────────
    # Tab setup
    # ─────────────────────────────────────────────

    def setup_room_grid_tab(self):
        layout = QFormLayout()
        self.x_size             = QLineEdit("830")
        self.y_size             = QLineEdit("830")
        self.z_size             = QLineEdit("300")
        self.grid_size          = QLineEdit("20")
        self.min_distance_input = QLineEdit("300")
        self.max_distance_input = QLineEdit("800")

        layout.addRow("X Size",       self.x_size)
        layout.addRow("Y Size",       self.y_size)
        layout.addRow("Z Size",       self.z_size)
        layout.addRow("Grid Size",    self.grid_size)
        layout.addRow("Min Distance", self.min_distance_input)
        layout.addRow("Max Distance", self.max_distance_input)

        room_button = QPushButton("Generate Room Grid")
        room_button.clicked.connect(self.display_room_grid)
        layout.addWidget(room_button)
        self.room_grid_tab.setLayout(layout)

    def setup_run_control_tab(self):
        layout = QVBoxLayout()

        self.connect_button    = QPushButton("Connect")
        self.pointer_button    = QPushButton("Pointer")
        self.scan_button       = QPushButton("Scan")
        self.stop_start_button = QPushButton("Stop/Start")
        self.save_csv_button   = QPushButton("Save CSV")

        layout.addWidget(self.connect_button)
        layout.addWidget(self.pointer_button)
        layout.addWidget(self.scan_button)
        layout.addWidget(self.stop_start_button)
        layout.addWidget(self.save_csv_button)

        self.sensor_ultrasonic = QCheckBox("Ultrasonic Sensor")
        self.sensor_tof        = QCheckBox("TOF Sensor")
        self.position_dropdown = QComboBox()
        self.position_dropdown.addItems([
            "1: Front Left", "2: Front Right",
            "3: Rear Left",  "4: Rear Right"
        ])

        layout.addWidget(self.sensor_ultrasonic)
        layout.addWidget(self.sensor_tof)
        layout.addWidget(QLabel("Scan Position:"))
        layout.addWidget(self.position_dropdown)

        self.connect_button.clicked.connect(self.connect_ble)
        self.pointer_button.clicked.connect(self.toggle_pointer)
        self.scan_button.clicked.connect(self.start_scan)
        self.stop_start_button.clicked.connect(self.stop_start_scan)
        self.save_csv_button.clicked.connect(self.save_csv)
        self.sensor_ultrasonic.stateChanged.connect(self.uncheck_other_sensors)
        self.sensor_tof.stateChanged.connect(self.uncheck_other_sensors)
        self.connection_status_signal.connect(self.update_connection_status)

        self.run_control_tab.setLayout(layout)

    def setup_simulation_control_tab(self):
        layout = QGridLayout()

        self.mesh_detail_slider = QSlider(Qt.Horizontal)
        self.mesh_detail_slider.setMinimum(1)
        self.mesh_detail_slider.setMaximum(10)
        self.mesh_detail_slider.setValue(5)
        self.mesh_detail_slider.valueChanged.connect(self.update_mesh_detail)

        self.bin_size_input    = QLineEdit("0.5")
        self.cone_angle_input  = QLineEdit("30")
        self.num_rays_input    = QLineEdit("100")
        self.num_layers_input  = QLineEdit("10")

        self.outlier_slider = QSlider(Qt.Horizontal)
        self.outlier_slider.setMinimum(1)
        self.outlier_slider.setMaximum(10)
        self.outlier_slider.setValue(3)
        self.outlier_slider.valueChanged.connect(self.update_mesh_detail)

        self.shape_dropdown = QComboBox()
        self.shape_dropdown.addItems(["None", "Box"])
        self.shape_dropdown.currentIndexChanged.connect(self.update_mesh_detail)

        layout.addWidget(QLabel("Bin Size:"),            7,  0)
        layout.addWidget(self.bin_size_input,            7,  1)
        layout.addWidget(QLabel("Cone Angle:"),          8,  0)
        layout.addWidget(self.cone_angle_input,          8,  1)
        layout.addWidget(QLabel("Number of Rays:"),      9,  0)
        layout.addWidget(self.num_rays_input,            9,  1)
        layout.addWidget(QLabel("Number of Layers:"),    10, 0)
        layout.addWidget(self.num_layers_input,          10, 1)
        layout.addWidget(QLabel("Outlier Removal:"),     11, 0)
        layout.addWidget(self.outlier_slider,            11, 1)
        layout.addWidget(QLabel("Object Shape:"),        12, 0)
        layout.addWidget(self.shape_dropdown,            12, 1)
        layout.addWidget(QLabel("Mesh Detail Level:"),   13, 0)
        layout.addWidget(self.mesh_detail_slider,        13, 1)

        self.process_button            = QPushButton("Process Data")
        self.render_button             = QPushButton("Render 3D Object")
        self.plot_button               = QPushButton("Plot Data")
        self.load_csv_button           = QPushButton("Load CSV")
        self.save_processed_csv_button = QPushButton("Save Processed CSV")

        layout.addWidget(self.process_button,            2, 1)
        layout.addWidget(self.render_button,             5, 0)
        layout.addWidget(self.plot_button,               2, 0)
        layout.addWidget(self.save_processed_csv_button, 3, 1)
        layout.addWidget(self.load_csv_button,           3, 0)

        self.render_button.clicked.connect(self.render_3d_object)
        self.process_button.clicked.connect(self.process_data)
        self.plot_button.clicked.connect(self.plot_data)
        self.save_processed_csv_button.clicked.connect(self.save_processed_csv)
        self.load_csv_button.clicked.connect(self.load_csv)

        self.simulation_control_tab.setLayout(layout)

    def setup_servo_control_tab(self):
        self.scanner_height_input = QLineEdit("200")
        self.pan_start  = QLineEdit("40")
        self.pan_end    = QLineEdit("80")
        self.pan_inc    = QLineEdit("2")
        self.tilt_start = QLineEdit("0")
        self.tilt_end   = QLineEdit("16")
        self.tilt_inc   = QLineEdit("2")

        input_layout = QFormLayout()
        input_layout.addRow("Scanner Height (mm)", self.scanner_height_input)
        input_layout.addRow("Pan Start Angle",     self.pan_start)
        input_layout.addRow("Pan End Angle",        self.pan_end)
        input_layout.addRow("Pan Increment",        self.pan_inc)
        input_layout.addRow("Tilt Start Angle",     self.tilt_start)
        input_layout.addRow("Tilt End Angle",       self.tilt_end)
        input_layout.addRow("Tilt Increment",       self.tilt_inc)

        self.manual_pan_slider = QSlider(Qt.Horizontal)
        self.manual_pan_slider.setMinimum(self.PAN_ANGLE_MIN)
        self.manual_pan_slider.setMaximum(self.PAN_ANGLE_MAX)
        self.manual_pan_slider.setValue((self.PAN_ANGLE_MIN + self.PAN_ANGLE_MAX) // 2)
        self.manual_pan_slider.setFixedWidth(180)
        self.manual_pan_slider.valueChanged.connect(self.manual_pan_changed)

        self.manual_tilt_slider = QSlider(Qt.Horizontal)
        self.manual_tilt_slider.setMinimum(self.TILT_ANGLE_MIN)
        self.manual_tilt_slider.setMaximum(self.TILT_ANGLE_MAX)
        self.manual_tilt_slider.setValue((self.TILT_ANGLE_MIN + self.TILT_ANGLE_MAX) // 2)
        self.manual_tilt_slider.setFixedWidth(180)
        self.manual_tilt_slider.valueChanged.connect(self.manual_tilt_changed)

        self.manual_pan_label  = QLabel(f"Pan Angle: {self.manual_pan_slider.value()}°")
        self.manual_tilt_label = QLabel(f"Tilt Angle: {self.manual_tilt_slider.value()}°")

        self.manual_pan_slider.valueChanged.connect(
            lambda v: self.manual_pan_label.setText(f"Pan Angle: {v}°"))
        self.manual_tilt_slider.valueChanged.connect(
            lambda v: self.manual_tilt_label.setText(f"Tilt Angle: {v}°"))

        input_layout.addRow("Manual Pan",  self.manual_pan_slider)
        input_layout.addRow(self.manual_pan_label)
        input_layout.addRow("Manual Tilt", self.manual_tilt_slider)
        input_layout.addRow(self.manual_tilt_label)

        self.servo_control_tab.setLayout(input_layout)

    # ─────────────────────────────────────────────
    # Manual servo control
    # ─────────────────────────────────────────────

    def manual_pan_changed(self, value):
        if self.ble_connected:
            asyncio.run_coroutine_threadsafe(
                self.send_ble_command(f"move_pan={value}"), self.ble_loop)

    def manual_tilt_changed(self, value):
        if self.ble_connected:
            asyncio.run_coroutine_threadsafe(
                self.send_ble_command(f"move_tilt={value}"), self.ble_loop)

    def update_mesh_detail(self):
        self.process_data()
        self.render_3d_object()

    # ─────────────────────────────────────────────
    # Room grid
    # ─────────────────────────────────────────────

    def display_room_grid(self):
        x_size    = float(self.x_size.text())
        y_size    = float(self.y_size.text())
        z_size    = float(self.z_size.text())
        grid_size = int(self.grid_size.text())
        self.room_dimensions = {'x_size': x_size, 'y_size': y_size, 'z_size': z_size}
        x, y, z = create_simulation_room(x_size, y_size, z_size, grid_size)
        self.canvas.plot_room(x, y, z, x_size, y_size, z_size)
        print(f"Room grid: {x_size} x {y_size} x {z_size} mm, grid={grid_size}")

    # ─────────────────────────────────────────────
    # BLE connection
    # ─────────────────────────────────────────────

    def connect_ble(self):
        if not self.ble_connected:
            self.ble_loop   = asyncio.new_event_loop()
            self.ble_thread = Thread(target=self.ble_loop.run_forever)
            self.ble_thread.start()
            asyncio.run_coroutine_threadsafe(self.connect_to_ble_device(), self.ble_loop)
        else:
            asyncio.run_coroutine_threadsafe(self.disconnect_ble(), self.ble_loop)

    async def connect_to_ble_device(self):
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name == DEVICE_NAME:
                self.ble_client = BleakClient(d)
                try:
                    await self.ble_client.connect()
                    self.ble_connected = True
                    self.connection_status_signal.emit("Connected")
                    print(f"Connected to {DEVICE_NAME}")
                    await self.ble_client.start_notify(UART_TX_UUID,
                                                       self.notification_handler)
                except Exception as e:
                    print(f"Failed to connect: {e}")
                    self.connection_status_signal.emit("Disconnected")
                return
        self.connection_status_signal.emit("Device not found")
        print(f"{DEVICE_NAME} not found")

    async def disconnect_ble(self):
        if self.ble_client and self.ble_client.is_connected:
            await self.ble_client.stop_notify(UART_TX_UUID)
            await self.ble_client.disconnect()
        self.ble_connected = False
        self.connection_status_signal.emit("Disconnected")
        print(f"Disconnected from {DEVICE_NAME}")

    def notification_handler(self, sender, data):
        message = data.decode('utf-8').strip()
        print(f"Received: {message}")
        if message == 'scan_complete':
            self.scan_complete()
        else:
            self.data_received_signal.emit(message)
            self.parse_and_store_data(message)

    def scan_complete(self):
        print("Scan complete.")

    def update_console_output(self, message):
        QMetaObject.invokeMethod(
            self.console_output, "appendPlainText",
            Qt.QueuedConnection, Q_ARG(str, message)
        )

    def update_connection_status(self, status):
        print(f"BLE status: {status}")

    async def send_ble_command(self, command):
        if self.ble_client and self.ble_client.is_connected:
            try:
                await self.ble_client.write_gatt_char(UART_RX_UUID, command.encode())
                print(f"Sent: {command}")
            except Exception as e:
                print(f"Send failed: {command} — {e}")
        else:
            print("Not connected")

    # ─────────────────────────────────────────────
    # Scan control
    # ─────────────────────────────────────────────

    def start_scan(self):
        if not self.ble_connected:
            print("Not connected to BLE device")
            return
        if not (self.sensor_ultrasonic.isChecked() or self.sensor_tof.isChecked()):
            print("Please select a sensor before starting the scan.")
            return

        pan_start_angle  = int(self.pan_start.text()  or 20)
        pan_end_angle    = int(self.pan_end.text()    or 110)
        tilt_start_angle = int(self.tilt_start.text() or 0)
        tilt_end_angle   = int(self.tilt_end.text()   or 16)

        self.pan_start_angle  = pan_start_angle
        self.tilt_start_angle = tilt_start_angle
        self.scanner_height   = float(self.scanner_height_input.text())

        if pan_start_angle < 15:
            print("Warning: Pan start angle must be at least 15° for backlash compensation.")
            return

        run_id_text = self.position_dropdown.currentText()
        run_id = int(run_id_text.split(':')[0])
        self.run_id = run_id

        if not self.check_existing_data(run_id):
            return

        async def send_scan_commands():
            sensor_cmd = 'sensor=ultra' if self.sensor_ultrasonic.isChecked() else 'sensor=tof'
            await self.send_ble_command(sensor_cmd)
            await asyncio.sleep(0.1)
            await self.send_ble_command(f"pan_start={pan_start_angle}")
            await self.send_ble_command(f"pan_end={pan_end_angle}")
            await self.send_ble_command(f"tilt_start={tilt_start_angle}")
            await self.send_ble_command(f"tilt_end={tilt_end_angle}")
            await self.send_ble_command(f"min_distance={self.min_distance_input.text()}")
            await self.send_ble_command(f"max_distance={self.max_distance_input.text()}")
            await self.send_ble_command('start')

        asyncio.run_coroutine_threadsafe(send_scan_commands(), self.ble_loop)

    def check_existing_data(self, run_id):
        for data_row in self.data_list:
            if int(data_row[0]) == run_id:
                reply = QMessageBox.question(
                    self, 'Overwrite Data',
                    f'Data for Run ID {run_id} already exists. Overwrite?',
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No
                )
                if reply == QMessageBox.Yes:
                    self.data_list = [d for d in self.data_list if int(d[0]) != run_id]
                    return True
                return False
        return True

    def stop_start_scan(self):
        if not self.ble_connected:
            print("Not connected")
            return
        if not hasattr(self, 'scan_running'):
            self.scan_running = True
        if self.scan_running:
            asyncio.run_coroutine_threadsafe(
                self.send_ble_command('pause'), self.ble_loop)
            self.scan_running = False
            print("Scan paused.")
            self.process_data()
        else:
            asyncio.run_coroutine_threadsafe(
                self.send_ble_command('resume'), self.ble_loop)
            self.scan_running = True
            print("Scan resumed.")

    def toggle_pointer(self):
        if self.ble_connected:
            cmd = 'pointer_on' if not self.pointer_state else 'pointer_off'
            self.pointer_state = not self.pointer_state
            asyncio.run_coroutine_threadsafe(
                self.send_ble_command(cmd), self.ble_loop)

    def uncheck_other_sensors(self):
        if self.sender() == self.sensor_ultrasonic and self.sensor_ultrasonic.isChecked():
            self.sensor_tof.setChecked(False)
            if self.ble_connected:
                asyncio.run_coroutine_threadsafe(
                    self.send_ble_command('sensor=ultra'), self.ble_loop)
        elif self.sender() == self.sensor_tof and self.sensor_tof.isChecked():
            self.sensor_ultrasonic.setChecked(False)
            if self.ble_connected:
                asyncio.run_coroutine_threadsafe(
                    self.send_ble_command('sensor=tof'), self.ble_loop)

    # ─────────────────────────────────────────────
    # CSV handling
    # ─────────────────────────────────────────────

    def create_csv_if_not_exists(self, filename):
        if not os.path.isfile(filename):
            try:
                with open(filename, mode='w', newline='') as f:
                    csv.writer(f).writerow([
                        'Run ID', 'Pan Angle', 'Tilt Angle', 'Distance',
                        'Adjusted Pan Angle', 'Adjusted Tilt Angle', 'X', 'Y', 'Z'
                    ])
                print(f"CSV created: {filename}")
            except OSError as e:
                print(f"CSV create error: {e}")

    def append_to_csv(self, filename, data_row):
        self.create_csv_if_not_exists(filename)
        try:
            with open(filename, mode='a', newline='') as f:
                csv.writer(f).writerow([str(v) for v in data_row])
        except Exception as e:
            print(f"CSV append error: {e}")

    # ─────────────────────────────────────────────
    # Data processing
    # ─────────────────────────────────────────────

    def parse_and_store_data(self, message):
        try:
            if not message.startswith('pan='):
                return
            parts     = dict(p.split('=') for p in message.split(','))
            pan_angle  = float(parts['pan'])
            tilt_angle = float(parts['tilt'])
            distance   = float(parts['distance'])

            adjusted_pan  = max(0, min(90, pan_angle - self.pan_start_angle))
            adjusted_tilt = 16 - tilt_angle

            x, y, z = self.calculate_xyz(adjusted_pan, adjusted_tilt, distance)
            xt, yt, zt = self.transform_coordinates(x, y, z, self.run_id)

            data_row = [self.run_id, pan_angle, tilt_angle, distance,
                        adjusted_pan, adjusted_tilt, xt, yt, zt]
            self.data_list.append(data_row)
            self.append_to_csv(self.raw_csv_path, data_row)
        except Exception as e:
            print(f"Parse error: {e}")

    def calculate_xyz(self, pan_angle, adjusted_tilt_angle, distance):
        """Convert spherical coordinates (pan, tilt, distance) to Cartesian (x, y, z)."""
        try:
            pan_rad  = math.radians(pan_angle)
            tilt_rad = math.radians(-adjusted_tilt_angle)
            x = distance * math.sin(pan_rad) * math.cos(tilt_rad)
            y = distance * math.cos(pan_rad) * math.cos(tilt_rad)
            z = distance * math.sin(tilt_rad) + self.scanner_height
            return x, y, z
        except Exception as e:
            print(f"XYZ calc error: {e}")
            return 0, 0, 0

    def transform_coordinates(self, x, y, z, run_id):
        """
        Apply rotation and translation for multi-position scanning.
        Four corner positions allow full room reconstruction from overlapping scans.
        """
        x_size = self.room_dimensions['x_size']
        y_size = self.room_dimensions['y_size']

        transforms = {
            1: (np.array([0, 0, 0]),          0),    # Front Left
            2: (np.array([x_size, 0, 0]),     90),   # Front Right
            3: (np.array([0, y_size, 0]),    -90),   # Rear Left
            4: (np.array([x_size, y_size, 0]), 180), # Rear Right
        }

        translation, rotation_angle = transforms.get(run_id, (np.array([0,0,0]), 0))
        theta = np.radians(rotation_angle)
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])
        tp = R @ np.array([x, y, z]) + translation
        return tp[0], tp[1], tp[2]

    def average_points(self, x_vals, y_vals, z_vals, bin_size=0.5):
        x_bins = np.arange(min(x_vals), max(x_vals) + bin_size, bin_size)
        y_bins = np.arange(min(y_vals), max(y_vals) + bin_size, bin_size)
        xi     = np.digitize(x_vals, x_bins)
        yi     = np.digitize(y_vals, y_bins)
        grid   = {}
        for x, y, z in zip(xi, yi, z_vals):
            grid.setdefault((x, y), []).append(z)
        ax, ay, az = [], [], []
        for (x, y), zl in grid.items():
            ax.append(x_bins[x-1] + bin_size / 2)
            ay.append(y_bins[y-1] + bin_size / 2)
            az.append(np.mean(zl))
        return np.array(ax), np.array(ay), np.array(az)

    def remove_outliers_with_run_ids(self, x, y, z, run_ids):
        threshold = 0.5 * self.outlier_slider.value()
        coords    = np.vstack((x, y, z)).T
        zs        = np.abs((coords - coords.mean(0)) / coords.std(0))
        mask      = (zs < threshold).all(1)
        return x[mask], y[mask], z[mask], run_ids[mask]

    def remove_outliers(self, x, y, z):
        threshold = 0.5 * self.outlier_slider.value()
        coords    = np.vstack((x, y, z)).T
        zs        = np.abs((coords - coords.mean(0)) / coords.std(0))
        mask      = (zs < threshold).all(1)
        return x[mask], y[mask], z[mask]

    def process_data(self):
        if not self.data_list:
            print("No data to process.")
            return
        detail = self.mesh_detail_slider.value()
        arr    = np.array(self.data_list)
        x, y, z   = arr[:,6].astype(float), arr[:,7].astype(float), arr[:,8].astype(float)
        run_ids   = arr[:,0].astype(int)
        x, y, z, run_ids = self.remove_outliers_with_run_ids(x, y, z, run_ids)
        if detail < 10:
            f = 11 - detail
            x, y, z, run_ids = x[::f], y[::f], z[::f], run_ids[::f]
        bin_size = float(self.bin_size_input.text() or 0.5)
        x, y, z  = self.average_points(x, y, z, bin_size)
        self.processed_data    = True
        self.processed_x_vals  = x
        self.processed_y_vals  = y
        self.processed_z_vals  = z
        self.processed_run_ids = run_ids
        print("Data processed.")

    def plot_data(self):
        if not self.data_list:
            print("No data to plot.")
            return
        arr     = np.array(self.data_list)
        run_ids = arr[:,0].astype(int)
        x, y, z = arr[:,6].astype(float), arr[:,7].astype(float), arr[:,8].astype(float)
        x, y, z, run_ids = self.remove_outliers_with_run_ids(x, y, z, run_ids)
        self.canvas.ax.clear()
        colors = {1: 'red', 2: 'green', 3: 'blue', 4: 'purple'}
        for rid in np.unique(run_ids):
            m = run_ids == rid
            self.canvas.ax.scatter(x[m], y[m], z[m],
                c=colors.get(rid, 'black'), label=f'Run {int(rid)}', s=10)
        self.canvas.ax.set_xlabel('X'); self.canvas.ax.set_ylabel('Y')
        self.canvas.ax.set_zlabel('Z'); self.canvas.ax.legend()
        self.canvas.draw()
        self.processed_data    = True
        self.processed_x_vals  = x
        self.processed_y_vals  = y
        self.processed_z_vals  = z
        self.processed_run_ids = run_ids

    def render_3d_object(self):
        if not self.processed_data:
            print("No processed data to render.")
            return
        x, y, z  = self.processed_x_vals, self.processed_y_vals, self.processed_z_vals
        run_ids  = self.processed_run_ids
        self.canvas.ax.clear()

        if self.shape_dropdown.currentText() == "Box":
            all_faces = []
            for rid in np.unique(run_ids):
                pts = np.vstack((x[run_ids==rid], y[run_ids==rid], z[run_ids==rid])).T
                if len(pts) >= 4:
                    for s in ConvexHull(pts).simplices:
                        all_faces.append([pts[i] for i in s])
            all_pts = np.vstack((x, y, z)).T
            if len(all_pts) >= 4:
                for s in ConvexHull(all_pts).simplices:
                    all_faces.append([all_pts[i] for i in s])
            mesh = Poly3DCollection(all_faces, linewidths=1, alpha=0.5)
            mesh.set_facecolor((0, 0, 1, 0.1))
            self.canvas.ax.add_collection3d(mesh)
            self.canvas.ax.scatter(x, y, z, c='b', s=1)
        else:
            tri = Delaunay(np.vstack((x, y)).T)
            self.canvas.ax.plot_trisurf(x, y, z, triangles=tri.simplices,
                cmap='viridis', edgecolor='grey', alpha=0.8)

        self.canvas.ax.set_xlabel('X'); self.canvas.ax.set_ylabel('Y')
        self.canvas.ax.set_zlabel('Z')
        self.canvas.ax.set_xlim([0, self.room_dimensions['x_size']])
        self.canvas.ax.set_ylim([0, self.room_dimensions['y_size']])
        self.canvas.ax.set_zlim([0, self.room_dimensions['z_size']])
        self.canvas.draw()

    # ─────────────────────────────────────────────
    # CSV save/load
    # ─────────────────────────────────────────────

    def save_csv(self):
        if not self.data_list:
            print("No data to save."); return
        filename = self.raw_csv_path
        if QMessageBox.question(self, 'Save', 'Save to different location?',
                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            filename, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV (*.csv)")
            if not filename: return
        try:
            with open(filename, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['Run ID','Pan Angle','Tilt Angle','Distance',
                            'Adj Pan','Adj Tilt','X','Y','Z'])
                w.writerows(self.data_list)
            print(f"Saved: {filename}")
        except Exception as e:
            print(f"Save error: {e}")

    def save_processed_csv(self):
        if not self.processed_data:
            print("No processed data."); return
        filename = self.processed_csv_path
        if QMessageBox.question(self, 'Save', 'Save to different location?',
                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            filename, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV (*.csv)")
            if not filename: return
        try:
            with open(filename, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['X', 'Y', 'Z'])
                for row in zip(self.processed_x_vals, self.processed_y_vals,
                               self.processed_z_vals):
                    w.writerow(row)
            print(f"Saved: {filename}")
        except Exception as e:
            print(f"Save error: {e}")

    def load_csv(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Open CSV",
                        self.raw_csv_path, "CSV (*.csv)")
        if filename:
            try:
                with open(filename, 'r', newline='') as f:
                    reader = csv.reader(f)
                    next(reader)  # skip header
                    self.data_list = [[float(v) for v in row] for row in reader]
                print(f"Loaded: {filename}")
                self.plot_data()
            except Exception as e:
                print(f"Load error: {e}")

    def closeEvent(self, event):
        if self.csv_file:
            self.csv_file.close()
        event.accept()


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ScannerApp()
    window.show()
    sys.exit(app.exec_())

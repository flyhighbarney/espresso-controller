"""
Real-time espresso extraction dashboard.

Connects to the STM32 via USB CDC serial, decodes COBS-framed telemetry,
and provides live plots of pressure, flow, temperature, puck resistance,
and controller state. Includes gain tuning sliders and shot logging.

Usage:
    python dashboard.py [--port COM3] [--baud 115200] [--simulate]
"""

import argparse
import csv
import struct
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

try:
    import serial
except ImportError:
    serial = None

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                  QHBoxLayout, QLabel, QSlider, QPushButton,
                                  QComboBox, QStatusBar, QGroupBox)
    from PyQt5.QtCore import QTimer, Qt
    import pyqtgraph as pg
    HAS_QT = True
except ImportError:
    HAS_QT = False

# ---- COBS decode (mirrors firmware implementation) ----

def cobs_decode(data: bytes) -> bytes:
    output = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]
        idx += 1
        if code == 0:
            break
        for _ in range(1, code):
            if idx >= len(data):
                break
            output.append(data[idx])
            idx += 1
        if code < 0xFF and idx < len(data):
            output.append(0x00)
    if output and output[-1] == 0x00:
        output = output[:-1]
    return bytes(output)


# ---- Telemetry packet parsing ----

STATE_PKT_FORMAT = '<BIfffffBB'  # matches telem_state_packet_t (without padding issues)
STATE_PKT_FIELDS = [
    'packet_type', 'timestamp_ms',
    'pressure_bar', 'flow_ml_s', 'temperature_c',
    'pump_duty', 'heater_duty', 'setpoint_pressure', 'est_resistance',
    'phase', 'machine_state'
]

# Full struct with proper packing
STATE_PKT_STRUCT = struct.Struct('<B I f f f f f f f B B')

def validate_packet(pkt: dict) -> bool:
    """Reject packets with NaN/Inf or physically implausible values."""
    for key in ('pressure_bar', 'flow_ml_s', 'temperature_c',
                'pump_duty', 'heater_duty', 'setpoint_pressure', 'est_resistance'):
        v = pkt.get(key, 0)
        if not np.isfinite(v):
            return False
    if not (-1 < pkt['pressure_bar'] < 20):
        return False
    if not (-1 < pkt['flow_ml_s'] < 30):
        return False
    if not (-50 < pkt['temperature_c'] < 200):
        return False
    if not (0 <= pkt['machine_state'] <= 5):
        return False
    return True


def parse_state_packet(data: bytes) -> dict:
    if len(data) < STATE_PKT_STRUCT.size:
        return None
    values = STATE_PKT_STRUCT.unpack_from(data)
    pkt = dict(zip(STATE_PKT_FIELDS, values))
    if not validate_packet(pkt):
        return None
    return pkt


# ---- Serial reader with COBS framing ----

class TelemetryReader:
    VALID_PORT_PATTERNS = (
        r'^COM\d+$',           # Windows
        r'^/dev/tty(ACM|USB|S)\d+$',  # Linux
        r'^/dev/cu\.\w+$',     # macOS
    )

    def __init__(self, port: str, baud: int = 115200):
        if serial is None:
            raise ImportError("pyserial required: pip install pyserial")

        import re
        if not any(re.match(p, port) for p in self.VALID_PORT_PATTERNS):
            available = [p.device for p in serial.tools.list_ports.comports()]
            raise ValueError(
                f"Invalid serial port: {port}. Available: {available}")

        if not (1200 <= baud <= 3000000):
            raise ValueError(f"Baud rate out of range: {baud}")

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.buf = bytearray()

    def read_packets(self):
        packets = []
        raw = self.ser.read(1024)
        if not raw:
            return packets

        self.buf.extend(raw)

        while b'\x00' in self.buf:
            idx = self.buf.index(b'\x00')
            frame = bytes(self.buf[:idx])
            self.buf = self.buf[idx + 1:]

            if len(frame) < 2:
                continue

            decoded = cobs_decode(frame)
            if decoded and decoded[0] == 0x01:  # STATE packet
                pkt = parse_state_packet(decoded)
                if pkt:
                    packets.append(pkt)

        return packets

    def close(self):
        self.ser.close()


# ---- Simulated telemetry for testing without hardware ----

class SimulatedReader:
    def __init__(self):
        self.t = 0.0
        self.shot_active = False
        self.shot_time = 0.0

    def read_packets(self):
        packets = []
        for _ in range(10):  # 10 packets per call at 100 Hz
            self.t += 0.01

            if self.shot_active:
                self.shot_time += 0.01
                if self.shot_time > 30.0:
                    self.shot_active = False
                    self.shot_time = 0.0

            # Simulate pressure profile
            if self.shot_active:
                if self.shot_time < 5.0:
                    pressure = 3.0 * (self.shot_time / 5.0)
                elif self.shot_time < 8.0:
                    pressure = 3.0 + 6.0 * ((self.shot_time - 5.0) / 3.0)
                else:
                    pressure = 9.0 + 0.3 * np.sin(self.shot_time * 0.5)
                flow = pressure / (8.0 + 2.0 * np.sin(self.shot_time * 0.3))
                pump_duty = 0.7 + 0.1 * np.sin(self.shot_time * 0.2)
            else:
                pressure = 0.0
                flow = 0.0
                pump_duty = 0.0

            noise = np.random.normal(0, 0.05)
            packets.append({
                'packet_type': 1,
                'timestamp_ms': int(self.t * 1000),
                'pressure_bar': pressure + noise,
                'flow_ml_s': flow + noise * 0.5,
                'temperature_c': 93.0 + np.random.normal(0, 0.3),
                'pump_duty': pump_duty,
                'heater_duty': 0.3,
                'setpoint_pressure': 9.0 if self.shot_active and self.shot_time > 5.0 else 3.0,
                'est_resistance': 8.0 + self.shot_time * 0.2 if self.shot_active else 10.0,
                'phase': min(3, int(self.shot_time / 8.0)) if self.shot_active else 0,
                'machine_state': 3 if self.shot_active else 0,
            })

        return packets

    def start_shot(self):
        self.shot_active = True
        self.shot_time = 0.0

    def close(self):
        pass


# ---- Shot logger ----

class ShotLogger:
    def __init__(self, data_dir: str = None):
        if data_dir is None:
            data_dir = Path(__file__).resolve().parent.parent.parent / 'data'
        self.data_dir = Path(data_dir).resolve()
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.current_shot = []
        self.recording = False

    def start(self):
        self.current_shot = []
        self.recording = True

    def log(self, pkt: dict):
        if self.recording:
            self.current_shot.append(pkt)

    @staticmethod
    def _sanitize_csv_value(val) -> str:
        """Prevent CSV injection by prefixing formula-triggering characters."""
        s = str(val)
        if s and s[0] in ('=', '+', '-', '@', '\t', '\r'):
            return "'" + s
        return s

    def stop_and_save(self) -> str:
        if not self.current_shot:
            self.recording = False
            return None

        self.recording = False
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = self.data_dir / f'shot_{timestamp}.csv'

        with open(filename, 'w', newline='') as f:
            headers = list(self.current_shot[0].keys())
            writer = csv.writer(f, quoting=csv.QUOTE_ALL)
            writer.writerow(headers)
            for pkt in self.current_shot:
                writer.writerow(self._sanitize_csv_value(pkt[h]) for h in headers)

        n = len(self.current_shot)
        self.current_shot = []
        return f"{filename} ({n} samples)"


# ---- PyQt Dashboard ----

if HAS_QT:
    class DashboardWindow(QMainWindow):
        HISTORY = 3000  # 30 seconds at 100 Hz

        def __init__(self, reader, logger):
            super().__init__()
            self.reader = reader
            self.logger = logger
            self.setWindowTitle('Espresso Controller Dashboard')
            self.resize(1200, 800)

            # Data buffers
            self.times = deque(maxlen=self.HISTORY)
            self.pressure = deque(maxlen=self.HISTORY)
            self.flow = deque(maxlen=self.HISTORY)
            self.setpoint = deque(maxlen=self.HISTORY)
            self.resistance = deque(maxlen=self.HISTORY)
            self.pump_duty = deque(maxlen=self.HISTORY)
            self.temperature = deque(maxlen=self.HISTORY)

            self._build_ui()

            self.timer = QTimer()
            self.timer.timeout.connect(self._update)
            self.timer.start(20)  # 50 Hz UI refresh

        def _build_ui(self):
            central = QWidget()
            self.setCentralWidget(central)
            layout = QHBoxLayout(central)

            # Plots
            plot_layout = QVBoxLayout()

            self.pressure_plot = pg.PlotWidget(title='Pressure (bar)')
            self.pressure_plot.setYRange(0, 14)
            self.pressure_plot.addLegend()
            self.pressure_curve = self.pressure_plot.plot(pen='y', name='Measured')
            self.setpoint_curve = self.pressure_plot.plot(pen=pg.mkPen('r', style=Qt.DashLine), name='Setpoint')
            plot_layout.addWidget(self.pressure_plot)

            self.flow_plot = pg.PlotWidget(title='Flow Rate (mL/s)')
            self.flow_plot.setYRange(0, 5)
            self.flow_curve = self.flow_plot.plot(pen='c')
            plot_layout.addWidget(self.flow_plot)

            self.resistance_plot = pg.PlotWidget(title='Est. Puck Resistance')
            self.resistance_plot.setYRange(0, 30)
            self.resistance_curve = self.resistance_plot.plot(pen='g')
            plot_layout.addWidget(self.resistance_plot)

            layout.addLayout(plot_layout, stretch=3)

            # Controls panel
            ctrl_layout = QVBoxLayout()

            # Status
            status_group = QGroupBox('Status')
            status_layout = QVBoxLayout()
            self.lbl_state = QLabel('State: IDLE')
            self.lbl_temp = QLabel('Temp: --.- C')
            self.lbl_pressure = QLabel('Pressure: --.-- bar')
            self.lbl_flow = QLabel('Flow: --.-- mL/s')
            self.lbl_resistance = QLabel('Resistance: --.-- bar*s/mL')
            for lbl in [self.lbl_state, self.lbl_temp, self.lbl_pressure, self.lbl_flow, self.lbl_resistance]:
                status_layout.addWidget(lbl)
            status_group.setLayout(status_layout)
            ctrl_layout.addWidget(status_group)

            # Gain tuning
            gain_group = QGroupBox('PID Gains')
            gain_layout = QVBoxLayout()
            self.sliders = {}
            for name, range_max, default in [('Kp', 50, 12), ('Ki', 50, 6), ('Kd', 20, 1)]:
                row = QHBoxLayout()
                lbl = QLabel(f'{name}: {default / 10:.1f}')
                slider = QSlider(Qt.Horizontal)
                slider.setRange(0, range_max)
                slider.setValue(default)
                slider.valueChanged.connect(lambda v, l=lbl, n=name: l.setText(f'{n}: {v / 10:.1f}'))
                row.addWidget(lbl)
                row.addWidget(slider)
                gain_layout.addLayout(row)
                self.sliders[name] = (slider, lbl)
            gain_group.setLayout(gain_layout)
            ctrl_layout.addWidget(gain_group)

            # Controller mode
            mode_group = QGroupBox('Controller')
            mode_layout = QVBoxLayout()
            self.mode_combo = QComboBox()
            self.mode_combo.addItems(['Baseline PID', 'Adaptive', 'Adaptive + ML'])
            self.mode_combo.setCurrentIndex(2)
            mode_layout.addWidget(self.mode_combo)
            mode_group.setLayout(mode_layout)
            ctrl_layout.addWidget(mode_group)

            # Buttons
            btn_layout = QVBoxLayout()
            self.btn_shot = QPushButton('Start Shot')
            self.btn_shot.clicked.connect(self._on_start_shot)
            btn_layout.addWidget(self.btn_shot)

            self.btn_stop = QPushButton('Stop')
            self.btn_stop.clicked.connect(self._on_stop)
            btn_layout.addWidget(self.btn_stop)

            self.btn_log = QPushButton('Start Logging')
            self.btn_log.setCheckable(True)
            self.btn_log.clicked.connect(self._on_toggle_log)
            btn_layout.addWidget(self.btn_log)

            ctrl_layout.addLayout(btn_layout)
            ctrl_layout.addStretch()

            layout.addLayout(ctrl_layout, stretch=1)

            self.statusBar().showMessage('Ready')

        def _update(self):
            packets = self.reader.read_packets()
            for pkt in packets:
                t = pkt['timestamp_ms'] / 1000.0
                self.times.append(t)
                self.pressure.append(pkt['pressure_bar'])
                self.flow.append(pkt['flow_ml_s'])
                self.setpoint.append(pkt['setpoint_pressure'])
                self.resistance.append(pkt['est_resistance'])
                self.pump_duty.append(pkt['pump_duty'])
                self.temperature.append(pkt['temperature_c'])
                self.logger.log(pkt)

            if not self.times:
                return

            t = list(self.times)
            self.pressure_curve.setData(t, list(self.pressure))
            self.setpoint_curve.setData(t, list(self.setpoint))
            self.flow_curve.setData(t, list(self.flow))
            self.resistance_curve.setData(t, list(self.resistance))

            pkt = packets[-1] if packets else None
            if pkt:
                states = ['IDLE', 'PREHEAT', 'PREINFUSE', 'EXTRACT', 'FLUSH', 'FAULT']
                st = pkt['machine_state']
                self.lbl_state.setText(f"State: {states[st] if st < len(states) else '?'}")
                self.lbl_temp.setText(f"Temp: {pkt['temperature_c']:.1f} C")
                self.lbl_pressure.setText(f"Pressure: {pkt['pressure_bar']:.2f} bar")
                self.lbl_flow.setText(f"Flow: {pkt['flow_ml_s']:.2f} mL/s")
                self.lbl_resistance.setText(f"Resistance: {pkt['est_resistance']:.2f}")

        def _on_start_shot(self):
            if hasattr(self.reader, 'start_shot'):
                self.reader.start_shot()
            self.statusBar().showMessage('Shot started')

        def _on_stop(self):
            self.statusBar().showMessage('Stopped')

        def _on_toggle_log(self, checked):
            if checked:
                self.logger.start()
                self.btn_log.setText('Stop Logging')
                self.statusBar().showMessage('Logging started')
            else:
                result = self.logger.stop_and_save()
                self.btn_log.setText('Start Logging')
                self.statusBar().showMessage(f'Saved: {result}' if result else 'No data')


def main():
    parser = argparse.ArgumentParser(description='Espresso Controller Dashboard')
    parser.add_argument('--port', default=None, help='Serial port (e.g. COM3, /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--simulate', action='store_true', help='Use simulated data')
    args = parser.parse_args()

    if args.simulate or args.port is None:
        reader = SimulatedReader()
        print("Running in simulation mode")
    else:
        reader = TelemetryReader(args.port, args.baud)
        print(f"Connected to {args.port}")

    logger = ShotLogger()

    if not HAS_QT:
        print("PyQt5 and pyqtgraph required for GUI. Install with:")
        print("  pip install PyQt5 pyqtgraph")
        print("\nFalling back to console mode...")

        try:
            while True:
                packets = reader.read_packets()
                for pkt in packets:
                    print(f"t={pkt['timestamp_ms']:8d}ms  "
                          f"P={pkt['pressure_bar']:5.2f}bar  "
                          f"F={pkt['flow_ml_s']:5.2f}mL/s  "
                          f"T={pkt['temperature_c']:5.1f}C  "
                          f"R={pkt['est_resistance']:5.2f}  "
                          f"duty={pkt['pump_duty']:4.2f}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            reader.close()
        return

    app = QApplication(sys.argv)
    window = DashboardWindow(reader, logger)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        reader.close()


if __name__ == '__main__':
    main()

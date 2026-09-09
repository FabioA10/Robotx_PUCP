import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String, Int8
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import QPushButton, QHBoxLayout
from python_qt_binding.QtWidgets import (
    QApplication, QGridLayout, QLabel, QVBoxLayout, QWidget,
)
from python_qt_binding.QtWidgets import QComboBox


class Dashboard(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.values = {}
        self.pending_image = None
        self.pixmap = None
        self.video_time = 0.0
        self.subscriptions = []
        self.setWindowTitle('RobotX | UUV — SOLO LECTURA')
        self.resize(1050, 750)
        self.setStyleSheet(
            'QWidget {background:#17212b; color:#eef2f6;}'
            'QLabel {font-size:16px; padding:6px;}'
        )
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel('UUV Dashboard — sin envío de comandos'))
        grid = QGridLayout()
        layout.addLayout(grid)
        self.labels = {}
        fields = [
            ('connected', 'Conexión', Bool, '/uuv/connected'),
            ('mode', 'Modo', String, '/uuv/mode'),
            ('armed', 'Armado', Bool, '/uuv/armed'),
            ('voltage', 'Batería', Float32, '/uuv/power/voltage'),
            ('current', 'Corriente', Float32, '/uuv/power/current'),
            ('deadman', 'RB del mando', Bool, '/uuv/deadman'),
        ]
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for row, (key, title, msg_type, topic) in enumerate(fields):
            grid.addWidget(QLabel(title), row, 0)
            self.labels[key] = QLabel('Sin datos')
            grid.addWidget(self.labels[key], row, 1)
            self.subscriptions.append(node.create_subscription(
                msg_type, topic,
                lambda msg, k=key: self.record(k, msg.data), qos,
            ))
        self.video = QLabel('Esperando video…')
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(320, 180)
        layout.addWidget(self.video, 1)
        layout.addWidget(QLabel('Profundidad, rumbo y DVL: aún no integrados'))
        self.subscriptions.append(node.create_subscription(
            CompressedImage, '/camera/image/compressed', self.receive_image, qos,
        ))
        self.setWindowTitle('RobotX | UUV — Prueba de accesorios')
        layout.itemAt(0).widget().setText(
            'UUV Dashboard — accesorios; movimiento deshabilitado'
        )

        self.camera_pub = node.create_publisher(
            Float32, '/uuv/control/camera_tilt', 10
        )
        self.lights_pub = node.create_publisher(
            Int8, '/uuv/control/lights_step', 10
        )

        self.previous_camera_command = 0.0

        controls = QHBoxLayout()
        layout.addLayout(controls)

        self.camera_up = QPushButton('Cámara ↑ (mantener)')
        self.camera_down = QPushButton('Cámara ↓ (mantener)')
        self.lights_less = QPushButton('Luz −')
        self.lights_more = QPushButton('Luz +')

        self.accessory_buttons = [
            self.camera_up, self.camera_down,
            self.lights_less, self.lights_more,
        ]
        for button in self.accessory_buttons:
            controls.addWidget(button)
            button.setEnabled(False)

        self.lights_less.clicked.connect(
            lambda: self.change_lights(-1)
        )
        self.lights_more.clicked.connect(
            lambda: self.change_lights(1)
        )

        self.accessory_timer = QTimer(self)
        self.accessory_timer.timeout.connect(self.update_accessories)
        self.accessory_timer.start(100)
        self.mode_request_pub = node.create_publisher(
            String, '/uuv/control/mode_request', 10
        )
        self.pending_mode = None
        self.mode_requested_at = 0.0

        mode_row = QHBoxLayout()
        layout.addLayout(mode_row)

        self.mode_selector = QComboBox()
        self.mode_selector.addItem('Manual', 'MANUAL')
        self.mode_selector.addItem('Mantener profundidad', 'ALT_HOLD')
        self.mode_selector.addItem('Mantener posición', 'POSHOLD')

        self.mode_apply = QPushButton('Solicitar modo')
        self.mode_apply.setEnabled(False)
        self.mode_apply.clicked.connect(self.request_mode)

        mode_row.addWidget(self.mode_selector)
        mode_row.addWidget(self.mode_apply)

        self.mode_status = QLabel('Sin solicitud de modo')
        layout.addWidget(self.mode_status)

        self.mode_timer = QTimer(self)
        self.mode_timer.timeout.connect(self.update_mode_request)
        self.mode_timer.start(100)

        self.setWindowTitle('RobotX | UUV — Accesorios y modos')
        layout.itemAt(0).widget().setText(
            'UUV Dashboard — prueba desarmada; movimiento deshabilitado'
        )
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    def record(self, key, value):
        self.values[key] = (value, time.monotonic())

    def fresh(self, key, timeout=3.0):
        value, stamp = self.values.get(key, (None, 0.0))
        return value if time.monotonic() - stamp < timeout else None

    def receive_image(self, msg):
        self.pending_image = bytes(msg.data)

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        connected = self.fresh('connected')
        texts = {
            'connected': 'Conectado' if connected else (
                'Desconectado' if connected is False else 'Sin datos recientes'),
        }
        for key in ('mode', 'armed', 'voltage', 'current', 'deadman'):
            value = self.fresh(key)
            if key != 'deadman' and connected is not True:
                value = None
            if value is None:
                texts[key] = 'Sin datos recientes'
            elif key == 'mode':
                texts[key] = str(value)
            elif key == 'armed':
                texts[key] = 'ARMADO' if value else 'Desarmado'
            elif key == 'deadman':
                texts[key] = 'Presionado' if value else 'Suelto'
            else:
                unit = 'V' if key == 'voltage' else 'A'
                texts[key] = f'{value:.2f} {unit}'
        recognized = {
            'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE',
            'SURFACE', 'POSHOLD', 'MOTOR_DETECT', 'SURFTRAK',
        }
        for key, label in self.labels.items():
            label.setText(texts[key])
            color = '#eef2f6'
            if key == 'mode':
                mode = texts[key]
                color = '#f1c40f' if mode == 'MANUAL' else (
                    '#2ecc71' if mode in recognized else '#95a5a6')
            label.setStyleSheet(f'color:{color}; font-weight:bold;')
        if self.pending_image is not None:
            frame = QPixmap()
            if frame.loadFromData(self.pending_image):
                self.pixmap = frame
                self.video_time = time.monotonic()
            self.pending_image = None
        if self.pixmap is not None and time.monotonic() - self.video_time < 3.0:
            self.video.setPixmap(self.pixmap.scaled(
                self.video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation,
            ))
        else:
            self.video.setText('Sin video reciente')


    def accessories_ready(self):
        return (
            self.fresh('connected') is True
            and self.fresh('armed') is False
            and self.isActiveWindow()
        )

    def change_lights(self, step):
        if not self.accessories_ready():
            return

        msg = Int8()
        msg.data = step
        self.lights_pub.publish(msg)

    def update_accessories(self):
        ready = self.accessories_ready()

        for button in self.accessory_buttons:
            button.setEnabled(ready)

        command = 0.0

        if ready:
            up = self.camera_up.isDown()
            down = self.camera_down.isDown()

            if up and not down:
                command = 1.0
            elif down and not up:
                command = -1.0

        if command != 0.0 or self.previous_camera_command != 0.0:
            msg = Float32()
            msg.data = command
            self.camera_pub.publish(msg)

        self.previous_camera_command = command

    def mode_request_ready(self):
        return (
            self.accessories_ready()
            and self.fresh('deadman') is False
            and self.pending_mode is None
        )

    def request_mode(self):
        if not self.mode_request_ready():
            return

        requested = self.mode_selector.currentData()
        self.pending_mode = requested
        self.mode_requested_at = time.monotonic()

        msg = String()
        msg.data = requested
        self.mode_request_pub.publish(msg)

        self.mode_status.setText(f'Solicitado: {requested}. Esperando…')

    def update_mode_request(self):
        ready = self.mode_request_ready()
        self.mode_apply.setEnabled(ready)
        self.mode_selector.setEnabled(ready)

        if self.pending_mode is None:
            return

        if self.fresh('connected') is not True:
            self.mode_status.setText('Sin conexión: cambio no confirmado')
            self.pending_mode = None
            return

        mode, stamp = self.values.get('mode', (None, 0.0))

        if (
            mode == self.pending_mode
            and stamp > self.mode_requested_at
            and self.fresh('mode') == mode
        ):
            self.mode_status.setText(f'Modo confirmado por el ROV: {mode}')
            self.pending_mode = None
        elif time.monotonic() - self.mode_requested_at > 5.0:
            self.mode_status.setText(
                'Sin confirmación en 5 s. Revisa el modo real y la terminal.'
            )
            self.pending_mode = None


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = Node('uuv_dashboard')
    window = Dashboard(node)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    window.show()
    try:
        app.exec_()
    finally:
        window.timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

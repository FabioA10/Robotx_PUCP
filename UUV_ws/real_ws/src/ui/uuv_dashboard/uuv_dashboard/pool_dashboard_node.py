"""Pool telemetry, preflight and supervised-manual profile display."""

import json
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Empty, Float32, String, UInt32
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout,
    QHBoxLayout, QLabel, QPushButton, QTabWidget, QTextEdit,
    QVBoxLayout, QWidget,
)

from uuv_dashboard.pool_model import (
    Pose, Telemetry, ekf_description, home_steps, preview,
)
from uuv_dashboard.pool_preflight_model import REQUIRED_CHECKS, evaluate
from uuv_dashboard.pool_operator_model import OPERATOR_MODES, mode_request_allowed


class PoolDashboard(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.operator_profile = bool(
            node.declare_parameter('operator_profile', False).value)
        self.enable_mode_requests = bool(
            node.declare_parameter('enable_mode_requests', False).value)
        self.data = Telemetry()
        self.subscriptions = []
        self.labels = {}
        self.steps = []
        self.home = None
        self.reference = None
        self.pending_image = None
        self.pixmap = None
        self.last_mode = None
        self.live_preview = False
        self.pending_mode = None
        self.mode_requested_at = 0.0
        title_suffix = ('perfil manual supervisado' if self.operator_profile
                        else 'solo lectura')
        self.setWindowTitle(f'RobotX | Preparación de piscina — {title_suffix}')
        self.resize(1120, 780)
        self.setStyleSheet(
            'QWidget {background:#17212b; color:#eef2f6; font-size:14px;}'
            'QPushButton,QComboBox,QDoubleSpinBox {padding:6px;}'
            'QPushButton {background:#304456;}'
            'QLabel {padding:3px;}'
        )
        layout = QVBoxLayout(self)
        title_text = ('Preparación de piscina · mando supervisado; '
                      'secuencias bloqueadas' if self.operator_profile else
                      'Preparación de piscina · SIN órdenes de movimiento')
        title = QLabel(title_text)
        title.setStyleSheet('font-size:21px; font-weight:bold; color:#6cd8e5;')
        layout.addWidget(title)
        self.status = QLabel('Esperando telemetría…')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        tabs = QTabWidget()
        self.tabs = tabs
        layout.addWidget(tabs)
        telemetry = QWidget()
        tl = QVBoxLayout(telemetry)
        grid = QGridLayout()
        tl.addLayout(grid)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        scalar_fields = [
            ('connected', 'Conexión', Bool, '/uuv/connected'),
            ('armed', 'Armado', Bool, '/uuv/armed'),
            ('mode', 'Modo real', String, '/uuv/mode'),
            ('voltage', 'Batería [V]', Float32, '/uuv/power/voltage'),
            ('depth', 'Profundidad estimada [m]*', Float32,
             '/uuv/telemetry/depth_estimate_m'),
            ('heading', 'Rumbo [°]', Float32, '/uuv/telemetry/heading_deg'),
            ('pressure', 'Presión absoluta [hPa]', Float32,
             '/uuv/telemetry/pressure_abs_hpa'),
            ('ekf', 'Flags EKF', UInt32, '/uuv/telemetry/ekf_status_flags'),
            ('output', 'Salida de órdenes ROS habilitada', Bool,
             '/uuv/control/command_output_enabled'),
            ('deadman', 'RB del mando', Bool, '/uuv/deadman'),
            ('motion_allowed', 'Movimiento MAVLink permitido', Bool,
             '/uuv/control/mavlink_motion_allowed'),
        ]
        fields = [(key, title) for key, title, _, _ in scalar_fields]
        fields += [('position', 'Posición N/E/D [m]'),
                   ('velocity', 'Velocidad N/E/D [m/s]'),
                   ('attitude', 'Roll/Pitch/Yaw [°]')]
        for row, (key, label) in enumerate(fields):
            grid.addWidget(QLabel(label), row, 0)
            self.labels[key] = QLabel('Sin datos')
            grid.addWidget(self.labels[key], row, 1)
        for key, _, msg_type, topic in scalar_fields:
            self.subscriptions.append(node.create_subscription(
                msg_type, topic,
                lambda msg, k=key: self.data.put(k, msg.data), qos))
        for key, msg_type, topic, attr in [
            ('position', PointStamped, 'local_position_ned', 'point'),
            ('velocity', Vector3Stamped, 'local_velocity_ned', 'vector'),
            ('attitude', Vector3Stamped, 'attitude_rpy', 'vector'),
        ]:
            self.subscriptions.append(node.create_subscription(
                msg_type, '/uuv/telemetry/' + topic,
                lambda msg, k=key, a=attr: self.vector(k, getattr(msg, a)), qos))
        note = QLabel(
            '* Referencia a superficie pendiente de verificar. NED z y distancia '
            'al fondo no son esta profundidad. La edad indica recepción en ROS, '
            'no antigüedad de la medición del DVL.')
        note.setWordWrap(True)
        tl.addWidget(note)
        self.ekf_text = QLabel('')
        self.ekf_text.setWordWrap(True)
        tl.addWidget(self.ekf_text)
        reference_button = QPushButton('Tomar referencia para medir variación XY')
        reference_button.clicked.connect(self.capture_reference)
        tl.addWidget(reference_button)
        self.drift = QLabel('Sin referencia. Mantén el ROV quieto para medir deriva.')
        tl.addWidget(self.drift)
        tl.addStretch()
        tabs.addTab(telemetry, 'Telemetría')
        tabs.addTab(self.make_preflight(), 'Preflight · piscina')
        if self.operator_profile:
            tabs.addTab(self.make_operator_controls(), 'Mando · modos')
        self.video = QLabel('Esperando video…')
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(320, 180)
        tabs.addTab(self.video, 'Cámara')
        self.subscriptions.append(node.create_subscription(
            CompressedImage, '/camera/image/compressed', self.image, qos))
        self.sequence_request_pub = node.create_publisher(
            String, '/uuv/sequence_dry_run/request', 10)
        self.sequence_cancel_pub = node.create_publisher(
            Empty, '/uuv/sequence_dry_run/cancel', 10)
        self.subscriptions.append(node.create_subscription(
            String, '/uuv/sequence_dry_run/status', self.sequence_status, 10))
        tabs.addTab(self.make_planner(), 'Secuencias · previsualización')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def vector(self, key, value):
        self.data.put(key, (value.x, value.y, value.z))

    def image(self, msg):
        self.pending_image = bytes(msg.data)
        self.data.put('video', True)

    def make_planner(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        note = QLabel(
            'Ensayo geométrico: NO ejecuta ni simula la dinámica del ROV. '
            'Giros positivos a la derecha; profundidad absoluta bajo superficie. '
            'Cada avance sigue el rumbo resultante del paso anterior.')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.source = QComboBox()
        self.source.addItem('Referencia ficticia editable (sin submarino)', 'example')
        self.source.addItem('Referencia de telemetría actual (desarmado)', 'live')
        layout.addWidget(self.source)
        row = QHBoxLayout()
        self.pose_inputs = []
        for title, low, high in [('N [m]', -10000, 10000),
                                 ('E [m]', -10000, 10000),
                                 ('Prof. [m]', 0, 1000),
                                 ('Yaw [°]', -180, 180)]:
            row.addWidget(QLabel(title))
            entry = QDoubleSpinBox()
            entry.setRange(low, high)
            entry.setDecimals(2)
            entry.valueChanged.connect(self.clear_preview)
            row.addWidget(entry)
            self.pose_inputs.append(entry)
        layout.addLayout(row)
        self.surface_verified = QCheckBox(
            'He verificado el cero de profundidad en superficie '
            '(solo referencia del ensayo; no calibra el sensor)')
        self.surface_verified.toggled.connect(self.invalidate_context)
        layout.addWidget(self.surface_verified)
        row = QHBoxLayout()
        self.kind = QComboBox()
        for text, kind in [('Avanzar [m]; negativo = retroceder', 'advance'),
                           ('Girar [°]; positivo = derecha', 'turn'),
                           ('Profundidad absoluta [m]', 'depth'),
                           ('Esperar [s]', 'wait')]:
            self.kind.addItem(text, kind)
        self.amount = QDoubleSpinBox()
        self.amount.setRange(-1000, 1000)
        self.amount.setDecimals(2)
        row.addWidget(self.kind)
        row.addWidget(self.amount)
        for text, callback in [('Añadir', self.add_step),
                               ('Quitar último', self.remove_step),
                               ('Vaciar lista', self.clear_steps)]:
            button = QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        self.step_text = QTextEdit()
        self.step_text.setReadOnly(True)
        self.step_text.setMaximumHeight(140)
        layout.addWidget(self.step_text)
        row = QHBoxLayout()
        for text, callback in [('Previsualizar desde referencia actual', self.show_preview),
                               ('Guardar Home de ensayo', self.save_home),
                               ('Previsualizar regreso', self.show_home),
                               ('Descartar objetivos', self.clear_preview)]:
            button = QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        self.home_label = QLabel('Home de ensayo: sin guardar')
        layout.addWidget(self.home_label)
        self.execution_status = QLabel(
            'Ejecución: salida física bloqueada por defecto')
        self.execution_status.setWordWrap(True)
        layout.addWidget(self.execution_status)
        execution_row = QHBoxLayout()
        validate_button = QPushButton('Validar plan para ejecución futura')
        validate_button.clicked.connect(self.validate_sequence)
        cancel_button = QPushButton('Cancelar / entregar al mando')
        cancel_button.clicked.connect(self.cancel_sequence)
        execution_row.addWidget(validate_button)
        execution_row.addWidget(cancel_button)
        layout.addLayout(execution_row)
        self.result = QTextEdit()
        self.result.setReadOnly(True)
        layout.addWidget(self.result)
        self.source.currentIndexChanged.connect(self.invalidate_context)
        return page

    def make_preflight(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        title = QLabel('Preflight de piscina · sesión de observación y registro')
        title.setStyleSheet('font-size:17px; font-weight:bold; color:#6cd8e5;')
        layout.addWidget(title)
        note = QLabel(
            'Este panel no habilita armado, propulsión ni movimientos. Un estado '
            'LISTO solo permite comenzar la prueba de telemetría/registro con el '
            'ROV desarmado.')
        note.setWordWrap(True)
        layout.addWidget(note)
        checklist = [
            ('vehicle_inspected', 'Carcasas, sellos, penetradores y lastre inspeccionados'),
            ('tether_clear', 'Tether libre, sin tensión ni obstáculos'),
            ('area_clear', 'Zona despejada: sin nadadores ni buceadores'),
            ('recovery_ready', 'Método de recuperación y piloto al mando confirmados'),
            ('recording_ready', 'Terminal de registro preparada antes de inmersión'),
        ]
        self.preflight_checks = {}
        for key, label in checklist:
            check = QCheckBox(label)
            check.toggled.connect(self.update_preflight)
            self.preflight_checks[key] = check
            layout.addWidget(check)
        self.camera_required = QCheckBox('Esta prueba requiere video reciente')
        self.camera_required.toggled.connect(self.update_preflight)
        layout.addWidget(self.camera_required)
        self.preflight_status = QLabel('NO GO · completa la lista y conecta el ROV')
        self.preflight_status.setWordWrap(True)
        layout.addWidget(self.preflight_status)
        self.preflight_detail = QTextEdit()
        self.preflight_detail.setReadOnly(True)
        layout.addWidget(self.preflight_detail)
        return page

    def make_operator_controls(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        title = QLabel('Mando Xbox y cambios de modo supervisados')
        title.setStyleSheet('font-size:17px; font-weight:bold; color:#6cd8e5;')
        layout.addWidget(title)
        note = QLabel(
            'Los sticks no se controlan desde esta pantalla: llegan del mando '
            'Xbox por el nodo probado. Esta pestaña muestra el estado real y '
            'permite solicitar un modo solo con el ROV desarmado y RB suelto.')
        note.setWordWrap(True)
        layout.addWidget(note)
        self.mode_request_pub = self.node.create_publisher(
            String, '/uuv/control/mode_request', 10)
        row = QHBoxLayout()
        self.mode_selector = QComboBox()
        labels = {
            'MANUAL': 'Manual',
            'ALT_HOLD': 'Mantener profundidad',
            'POSHOLD': 'Mantener posición',
            'GUIDED': 'Guided (sin secuencias)',
        }
        for mode in OPERATOR_MODES:
            self.mode_selector.addItem(labels[mode], mode)
        self.mode_apply = QPushButton('Solicitar modo')
        self.mode_apply.clicked.connect(self.request_mode)
        row.addWidget(self.mode_selector)
        row.addWidget(self.mode_apply)
        layout.addLayout(row)
        self.mode_status = QLabel(
            'Cambios de modo deshabilitados en este lanzamiento')
        self.mode_status.setWordWrap(True)
        layout.addWidget(self.mode_status)
        mode_note = QLabel(
            'GUIDED aquí solo es un modo informado por ArduSub. No activa el '
            'ejecutor ni genera setpoints ni movimientos autónomos.')
        mode_note.setWordWrap(True)
        layout.addWidget(mode_note)
        layout.addStretch()
        return page

    def mode_request_ready(self):
        return mode_request_allowed(
            enabled=self.enable_mode_requests,
            connected=self.data.get('connected') is True,
            armed=self.data.get('armed'),
            deadman=self.data.get('deadman'),
        ) and self.pending_mode is None

    def request_mode(self):
        if not self.mode_request_ready():
            return
        requested = self.mode_selector.currentData()
        msg = String()
        msg.data = requested
        self.mode_request_pub.publish(msg)
        self.pending_mode = requested
        self.mode_requested_at = self.data.clock()
        self.mode_status.setText(f'Solicitado: {requested}. Esperando confirmación…')

    def update_mode_request(self):
        if not self.operator_profile:
            return
        ready = self.mode_request_ready()
        self.mode_apply.setEnabled(ready)
        self.mode_selector.setEnabled(ready)
        if not self.enable_mode_requests:
            self.mode_status.setText(
                'Cambios de modo deshabilitados: inicia con mode_control:=true')
            return
        if self.pending_mode is None:
            return
        if self.data.get('connected') is not True:
            self.mode_status.setText('Sin conexión: cambio no confirmado')
            self.pending_mode = None
            return
        mode_stamp = self.data.stamp('mode')
        if (self.data.get('mode') == self.pending_mode
                and mode_stamp is not None
                and mode_stamp > self.mode_requested_at):
            self.mode_status.setText(
                f'Modo confirmado por el ROV: {self.pending_mode}')
            self.pending_mode = None
        elif self.data.clock() - self.mode_requested_at > 5.0:
            self.mode_status.setText(
                'Sin confirmación en 5 s. Revisa el modo real y la terminal.')
            self.pending_mode = None

    def update_preflight(self, *_):
        if not hasattr(self, 'preflight_checks'):
            return
        checks = {key: check.isChecked()
                  for key, check in self.preflight_checks.items()}
        ready, reasons = evaluate(
            checks,
            connected=self.data.get('connected'),
            armed=self.data.get('armed'),
            output_enabled=self.data.get('output'),
            camera_required=self.camera_required.isChecked(),
            camera_available=self.data.get('video') is True,
        )
        if ready:
            self.preflight_status.setText(
                'LISTO PARA OBSERVACIÓN/REGISTRO · ROV desarmado · salida ROS bloqueada')
            self.preflight_status.setStyleSheet('color:#2ecc71; font-weight:bold;')
            self.preflight_detail.setPlainText(
                'Siguiente paso: inicia record_pool.sh en otra terminal, introduce '
                'el ROV en el agua y mantén 60 s de telemetría quieta. Este estado '
                'no autoriza armado ni movimiento.')
        else:
            self.preflight_status.setText('NO GO · no iniciar la sesión')
            self.preflight_status.setStyleSheet('color:#e67e22; font-weight:bold;')
            self.preflight_detail.setPlainText('\n'.join('• ' + reason for reason in reasons))

    def sequence_status(self, msg):
        self.execution_status.setText('Ejecución: ' + msg.data)

    def validate_sequence(self):
        if not self.steps:
            self.execution_status.setText('Ejecución: añade al menos un paso')
            return
        msg = String()
        msg.data = json.dumps({'steps': self.steps})
        self.sequence_request_pub.publish(msg)
        self.execution_status.setText('Ejecución: validando plan; salida bloqueada')

    def cancel_sequence(self):
        self.sequence_cancel_pub.publish(Empty())
        self.execution_status.setText('Ejecución: cancelación solicitada; control al mando')

    def clear_preview(self, *_):
        self.live_preview = False
        if hasattr(self, 'result'):
            self.result.setPlainText('Objetivos descartados; vuelve a previsualizar.')

    def invalidate_context(self, *_):
        self.home = None
        self.home_label.setText('Home de ensayo: sin guardar / invalidado')
        self.clear_preview()
        for entry in self.pose_inputs:
            entry.setEnabled(self.source.currentData() == 'example')

    def add_step(self):
        step = (self.kind.currentData(), self.amount.value())
        try:
            preview(Pose(0, 0, 0, 0), [step])
        except ValueError as exc:
            self.result.setPlainText(str(exc))
            return
        self.steps.append(step)
        self.update_steps()

    def remove_step(self):
        if self.steps:
            self.steps.pop()
        self.update_steps()

    def clear_steps(self):
        self.steps.clear()
        self.update_steps()

    def update_steps(self):
        names = {'advance': 'Avance [m]', 'turn': 'Giro [°]',
                 'depth': 'Profundidad absoluta [m]', 'wait': 'Espera [s]'}
        self.step_text.setPlainText('\n'.join(
            f'{i}. {names[k]}: {v:g}'
            for i, (k, v) in enumerate(self.steps, 1)))
        self.clear_preview()

    def live_pose(self):
        if self.data.get('connected') is not True:
            raise ValueError('No hay conexión reciente')
        if self.data.get('armed') is not False:
            raise ValueError('El ensayo con telemetría requiere el ROV desarmado')
        if not self.surface_verified.isChecked():
            raise ValueError('Falta verificar la referencia de profundidad')
        p, a, depth = (self.data.get(k) for k in ('position', 'attitude', 'depth'))
        flags = self.data.get('ekf')
        if p is None or a is None or depth is None or flags is None:
            raise ValueError('Faltan datos recientes de posición, actitud, profundidad o EKF')
        if not flags & 1 or not flags & (8 | 16) or flags & (128 | 1024 | 32768):
            raise ValueError('EKF no reporta una referencia horizontal válida')
        return Pose(p[0], p[1], depth, math.degrees(a[2]))

    def pose(self):
        if self.source.currentData() == 'live':
            return self.live_pose()
        return Pose(*(entry.value() for entry in self.pose_inputs))

    def render_preview(self, start, steps):
        targets = preview(start, steps)
        context = 'TELEMETRÍA' if self.source.currentData() == 'live' else 'FICTICIO'
        lines = [f'ENSAYO {context} — sin envío al submarino',
                 f'Inicio N={start.north:.2f}, E={start.east:.2f}, '
                 f'prof={start.depth:.2f} m, yaw={start.yaw:.1f}°']
        for i, (step, goal) in enumerate(zip(steps, targets), 1):
            lines.append(f'{i}. {step[0]} {step[1]:g} → '
                         f'N={goal.north:.2f}, E={goal.east:.2f}, '
                         f'prof={goal.depth:.2f} m, yaw={goal.yaw:.1f}°')
        if not targets:
            lines.append('Añade pasos a la lista.')
        self.result.setPlainText('\n'.join(lines))
        self.live_preview = self.source.currentData() == 'live'

    def show_preview(self):
        try:
            self.render_preview(self.pose(), self.steps)
        except ValueError as exc:
            self.clear_preview()
            self.result.setPlainText(str(exc))

    def save_home(self):
        try:
            self.home = self.pose()
            source = self.source.currentData()
            self.home_label.setText(f'Home de ensayo ({source}): {self.home}')
        except ValueError as exc:
            self.result.setPlainText(str(exc))

    def show_home(self):
        try:
            current = self.pose()
            if self.home is None:
                raise ValueError('Primero guarda un Home de ensayo')
            self.render_preview(current, home_steps(current, self.home))
        except ValueError as exc:
            self.clear_preview()
            self.result.setPlainText(str(exc))

    def capture_reference(self):
        position = self.data.get('position')
        if position is not None and self.data.get('connected') is True:
            self.reference = (position, self.data.clock(), 0.0)
        else:
            self.reference = None
            self.drift.setText('No hay posición reciente para tomar referencia')

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        # Drain a bounded amount of queued work without freezing Qt.
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.0)
        connected = self.data.get('connected') is True
        for key, label in self.labels.items():
            value = self.data.get(key)
            age = self.data.age(key)
            if value is None or (not connected and key not in ('connected', 'output')):
                label.setText('Sin datos recientes' if age is None else
                              f'Sin datos válidos / conexión · última recepción {age:.1f} s')
            else:
                if isinstance(value, tuple):
                    values = tuple(math.degrees(v) for v in value) if key == 'attitude' else value
                    text = ' / '.join(f'{v:.3f}' for v in values)
                elif isinstance(value, bool):
                    text = 'Sí' if value else 'No'
                elif isinstance(value, float):
                    text = f'{value:.3f}'
                else:
                    text = str(value)
                label.setText(f'{text} · recibido hace {age:.1f} s')
            if key == 'mode':
                color = '#f1c40f' if connected and value == 'MANUAL' else (
                    '#2ecc71' if connected and value is not None else '#a0a8b0')
                label.setStyleSheet(f'color:{color}; font-weight:bold;')
        flags = self.data.get('ekf') if connected else None
        self.ekf_text.setText('EKF: ' + ekf_description(flags))
        if not connected:
            self.status.setText('Sin conexión reciente. Puedes preparar secuencias ficticias.')
        elif self.data.get('armed') is not False or self.data.get('output') is not False:
            self.status.setText('Revisa el estado: esta prueba requiere desarmado y salida ROS deshabilitada.')
        else:
            self.status.setText('Conectado y desarmado · salida ROS deshabilitada · navegación aún no validada')
        mode = self.data.get('mode') if connected else None
        if self.source.currentData() == 'live':
            try:
                self.live_pose()
            except ValueError:
                if self.home is not None or self.live_preview:
                    self.invalidate_context()
            if mode == 'MANUAL' and self.last_mode != 'MANUAL':
                self.clear_preview()
        self.last_mode = mode
        self.update_mode_request()
        position = self.data.get('position') if connected else None
        if self.reference is not None:
            if position is None:
                self.reference = None
                self.drift.setText('Referencia invalidada por pérdida de posición / conexión')
            else:
                start, stamp, maximum = self.reference
                distance = math.hypot(position[0] - start[0], position[1] - start[1])
                maximum = max(maximum, distance)
                self.reference = (start, stamp, maximum)
                self.drift.setText(f'Variación XY: {distance:.3f} m · máximo: {maximum:.3f} m '
                                   f'· {self.data.clock() - stamp:.0f} s desde referencia')
        if self.pending_image is not None:
            frame = QPixmap()
            if frame.loadFromData(self.pending_image):
                self.pixmap = frame
            self.pending_image = None
        if self.pixmap is not None and self.data.get('video'):
            self.video.setPixmap(self.pixmap.scaled(
                self.video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.video.setText('Sin video reciente')
        self.update_preflight()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    node = Node('uuv_pool_dashboard')
    window = PoolDashboard(node)
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

"""Pool telemetry, preflight and supervised-manual profile display."""

from collections import deque
import json
import math
import signal
import sys
import time

from geometry_msgs.msg import PointStamped, Vector3Stamped
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Bool, Empty, Float32, String, UInt32

from uuv_dashboard.pool_model import (
    ekf_description,
    home_steps,
    Pose,
    preview,
    Telemetry,
)
from uuv_dashboard.pool_operator_model import mode_request_allowed, OPERATOR_MODES
from uuv_dashboard.pool_preflight_model import evaluate
from uuv_dashboard.sonar_view import SonarView
from uuv_dashboard.theme import apply_theme, set_widget_state


class PoolDashboard(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.operator_profile = bool(node.declare_parameter('operator_profile', False).value)
        self.enable_mode_requests = bool(
            node.declare_parameter('enable_mode_requests', False).value
        )
        self.camera_expected = bool(
            node.declare_parameter('camera_expected', True).value
        )
        self.sonar_expected = bool(
            node.declare_parameter('sonar_expected', False).value
        )
        self.sonar_acquisition_range_m = float(
            node.declare_parameter('sonar_acquisition_range_m', 5.0).value
        )
        self.sonar_filter_radius_m = float(
            node.declare_parameter('sonar_filter_radius_m', 2.0).value
        )
        self.sonar_acquisition_threshold = float(
            node.declare_parameter('sonar_acquisition_threshold', 1.0).value
        )
        self.sonar_filter_threshold = float(
            node.declare_parameter('sonar_filter_threshold', 90.0).value
        )
        self.sonar_angle_step = int(
            node.declare_parameter('sonar_angle_step', 4).value
        )
        self.accessory_control_expected = bool(
            node.declare_parameter('accessory_control_expected', False).value
        )
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
        self.current_samples = deque()
        self.voltage_samples = deque()
        self.camera_frame_times = deque()
        self.alerts = deque(maxlen=20)
        self.current_limit_a = None
        self.accessory_status_data = {}
        self.xbox_status_data = {}
        title_suffix = 'perfil manual supervisado' if self.operator_profile else 'solo lectura'
        self.setWindowTitle(f'RobotX | Preparación de piscina — {title_suffix}')
        self.setObjectName('RobotXRoot')
        self.resize(1280, 820)
        self.setMinimumSize(1050, 700)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)
        title_text = (
            'Preparación de piscina · mando supervisado; secuencias bloqueadas'
            if self.operator_profile
            else 'Preparación de piscina · SIN órdenes de movimiento'
        )
        title = QLabel(title_text)
        title.setObjectName('AppTitle')
        layout.addWidget(title)
        self.status = QLabel('Esperando telemetría…')
        self.status.setObjectName('StatusBanner')
        set_widget_state(self.status, 'neutral')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        tabs = QTabWidget()
        tabs.setObjectName('MainTabs')
        self.tabs = tabs
        layout.addWidget(tabs)
        telemetry = QWidget()
        tl = QVBoxLayout(telemetry)
        cards = QHBoxLayout()
        cards.setSpacing(12)
        tl.addLayout(cards)
        vehicle_group = QGroupBox('Estado del vehículo')
        vehicle_grid = QGridLayout(vehicle_group)
        navigation_group = QGroupBox('Navegación y estimación')
        navigation_grid = QGridLayout(navigation_group)
        cards.addWidget(vehicle_group, 1)
        cards.addWidget(navigation_group, 1)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        scalar_fields = [
            ('connected', 'Conexión', Bool, '/uuv/connected'),
            ('armed', 'Armado', Bool, '/uuv/armed'),
            ('mode', 'Modo real', String, '/uuv/mode'),
            ('voltage', 'Batería [V]', Float32, '/uuv/power/voltage'),
            ('current', 'Corriente total [A]', Float32, '/uuv/power/current'),
            (
                'command_scale',
                'Límite Xbox/MANUAL',
                Float32,
                '/uuv/control/command_scale',
            ),
            ('depth', 'Profundidad estimada [m]*', Float32, '/uuv/telemetry/depth_estimate_m'),
            ('heading', 'Rumbo [°]', Float32, '/uuv/telemetry/heading_deg'),
            ('pressure', 'Presión absoluta [hPa]', Float32, '/uuv/telemetry/pressure_abs_hpa'),
            ('ekf', 'Flags EKF', UInt32, '/uuv/telemetry/ekf_status_flags'),
            (
                'output',
                'Salida de órdenes ROS habilitada',
                Bool,
                '/uuv/control/command_output_enabled',
            ),
            (
                'accessory_output',
                'Salida de cámara/luces habilitada',
                Bool,
                '/uuv/accessories/output_enabled',
            ),
            (
                'lights_percent',
                'Luces estimadas [%]',
                Float32,
                '/uuv/accessories/lights_percent',
            ),
            ('deadman', 'RB del mando', Bool, '/uuv/deadman'),
            (
                'motion_allowed',
                'Movimiento MAVLink permitido',
                Bool,
                '/uuv/control/mavlink_motion_allowed',
            ),
        ]
        fields = [(key, title) for key, title, _, _ in scalar_fields]
        fields += [
            ('position', 'Posición N/E/D [m]'),
            ('velocity', 'Velocidad N/E/D [m/s]'),
            ('attitude', 'Roll/Pitch/Yaw [°]'),
        ]
        vehicle_keys = {
            'connected', 'armed', 'mode', 'voltage', 'current', 'command_scale',
            'output', 'accessory_output', 'lights_percent', 'deadman', 'motion_allowed'
        }
        rows = {'vehicle': 0, 'navigation': 0}
        for key, label in fields:
            group = 'vehicle' if key in vehicle_keys else 'navigation'
            grid = vehicle_grid if group == 'vehicle' else navigation_grid
            row = rows[group]
            field_label = QLabel(label)
            field_label.setObjectName('FieldLabel')
            grid.addWidget(field_label, row, 0)
            self.labels[key] = QLabel('Sin datos')
            self.labels[key].setObjectName('TelemetryValue')
            set_widget_state(self.labels[key], 'neutral')
            grid.addWidget(self.labels[key], row, 1)
            rows[group] += 1
        for key, _, msg_type, topic in scalar_fields:
            self.subscriptions.append(
                node.create_subscription(
                    msg_type, topic, lambda msg, k=key: self.scalar(k, msg.data), qos
                )
            )
        self.subscriptions.append(
            node.create_subscription(
                String,
                '/uuv/accessories/status',
                self.receive_accessory_status,
                10,
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                String,
                '/uuv/input/xbox_status',
                self.receive_xbox_status,
                10,
            )
        )
        for key, msg_type, topic, attr in [
            ('position', PointStamped, 'local_position_ned', 'point'),
            ('velocity', Vector3Stamped, 'local_velocity_ned', 'vector'),
            ('attitude', Vector3Stamped, 'attitude_rpy', 'vector'),
        ]:
            self.subscriptions.append(
                node.create_subscription(
                    msg_type,
                    '/uuv/telemetry/' + topic,
                    lambda msg, k=key, a=attr: self.vector(k, getattr(msg, a)),
                    qos,
                )
            )
        note = QLabel(
            '* Referencia a superficie pendiente de verificar. NED z y distancia '
            'al fondo no son esta profundidad. La edad indica recepción en ROS, '
            'no antigüedad de la medición del DVL.'
        )
        note.setWordWrap(True)
        note.setObjectName('MutedText')
        tl.addWidget(note)
        self.ekf_text = QLabel('')
        self.ekf_text.setWordWrap(True)
        self.ekf_text.setObjectName('TelemetryValue')
        tl.addWidget(self.ekf_text)
        self.power_summary = QLabel('Energía: esperando tensión y corriente válidas')
        self.power_summary.setObjectName('TelemetryValue')
        self.power_summary.setWordWrap(True)
        set_widget_state(self.power_summary, 'neutral')
        tl.addWidget(self.power_summary)
        self.alert_summary = QLabel('ArduSub: sin mensajes STATUSTEXT recibidos')
        self.alert_summary.setObjectName('TelemetryValue')
        self.alert_summary.setWordWrap(True)
        set_widget_state(self.alert_summary, 'neutral')
        tl.addWidget(self.alert_summary)
        self.alert_history = QTextEdit()
        self.alert_history.setReadOnly(True)
        self.alert_history.setMaximumHeight(110)
        self.alert_history.setPlaceholderText(
            'Aquí aparecerán avisos de batería, pre-armado y failsafe.'
        )
        tl.addWidget(self.alert_history)
        self.subscriptions.append(
            node.create_subscription(
                String,
                '/uuv/alerts/statustext',
                self.receive_statustext,
                10,
            )
        )
        reference_button = QPushButton('Tomar referencia para medir variación XY')
        reference_button.setObjectName('PrimaryAction')
        reference_button.clicked.connect(self.capture_reference)
        tl.addWidget(reference_button)
        self.drift = QLabel('Sin referencia. Mantén el ROV quieto para medir deriva.')
        self.drift.setObjectName('MutedText')
        tl.addWidget(self.drift)
        tl.addStretch()
        tabs.addTab(telemetry, 'Telemetría')
        tabs.addTab(self.make_preflight(), 'Preflight · piscina')
        if self.operator_profile:
            tabs.addTab(self.make_operator_controls(), 'Mando · modos')
        camera_page = QWidget()
        camera_layout = QVBoxLayout(camera_page)
        self.camera_status = QLabel('Cámara: esperando estado')
        self.camera_status.setObjectName('TelemetryValue')
        self.camera_status.setWordWrap(True)
        set_widget_state(self.camera_status, 'neutral')
        camera_layout.addWidget(self.camera_status)
        self.video = QLabel(
            'CÁMARA SIN DATOS\n\nEsperando /camera/image/compressed'
        )
        self.video.setObjectName('VideoSurface')
        self.video.setAlignment(Qt.AlignCenter)
        self.video.setMinimumSize(320, 180)
        camera_layout.addWidget(self.video, 1)
        tabs.addTab(camera_page, 'Cámara')
        self.subscriptions.append(
            node.create_subscription(CompressedImage, '/camera/image/compressed', self.image, qos)
        )
        self.sonar = SonarView(
            expected=self.sonar_expected,
            acquisition_range_m=self.sonar_acquisition_range_m,
            filter_radius_m=self.sonar_filter_radius_m,
            acquisition_threshold=self.sonar_acquisition_threshold,
            filter_threshold=self.sonar_filter_threshold,
            angle_step=self.sonar_angle_step,
        )
        tabs.addTab(self.sonar, 'Ping360')
        self.subscriptions.append(
            node.create_subscription(
                PointCloud2,
                '/ping360/filtered_points',
                lambda msg: self.sonar.receive('filtered', msg),
                qos,
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                PointCloud2, '/ping360/points', lambda msg: self.sonar.receive('raw', msg), qos
            )
        )
        self.sequence_request_pub = node.create_publisher(
            String, '/uuv/sequence_dry_run/request', 10
        )
        self.sequence_cancel_pub = node.create_publisher(Empty, '/uuv/sequence_dry_run/cancel', 10)
        self.subscriptions.append(
            node.create_subscription(
                String, '/uuv/sequence_dry_run/status', self.sequence_status, 10
            )
        )
        tabs.addTab(self.make_planner(), 'Secuencias · previsualización')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def vector(self, key, value):
        self.data.put(key, (value.x, value.y, value.z))

    def scalar(self, key, value):
        """Store a scalar and retain short power histories for operator peaks."""
        self.data.put(key, value)
        now = self.data.clock()
        if key == 'current' and math.isfinite(float(value)):
            self.current_samples.append((now, float(value)))
        elif key == 'voltage' and math.isfinite(float(value)):
            self.voltage_samples.append((now, float(value)))
        for samples in (self.current_samples, self.voltage_samples):
            while samples and now - samples[0][0] > 60.0:
                samples.popleft()

    def receive_statustext(self, message):
        """Keep recent autopilot warnings visible instead of only in a terminal."""
        try:
            payload = json.loads(message.data)
            severity = int(payload['severity'])
            text = str(payload['text']).strip()
            if not 0 <= severity <= 7 or not text:
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        now = time.monotonic()
        if self.alerts and self.alerts[-1][:2] == (severity, text):
            if now - self.alerts[-1][2] < 1.0:
                return
        self.alerts.append((severity, text, now))
        self.data.put('alert', (severity, text))
        self.alert_history.setPlainText(
            '\n'.join(
                f'[{item_severity}] {item_text}'
                for item_severity, item_text, _stamp in reversed(self.alerts)
            )
        )

    def receive_accessory_status(self, message):
        """Keep the bridge gate visible in the supervised-manual page."""
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                self.accessory_status_data = payload
                self.data.put('accessory_status', True)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def receive_xbox_status(self, message):
        """Keep the last normalized Xbox gesture diagnosis."""
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                self.xbox_status_data = payload
                self.data.put('xbox_status', True)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def image(self, msg):
        self.pending_image = bytes(msg.data)
        self.data.put('video', True)
        now = time.monotonic()
        self.camera_frame_times.append(now)
        while self.camera_frame_times and now - self.camera_frame_times[0] > 2.0:
            self.camera_frame_times.popleft()

    def make_planner(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        note = QLabel(
            'Ensayo geométrico: NO ejecuta ni simula la dinámica del ROV. '
            'Giros positivos a la derecha; profundidad absoluta bajo superficie. '
            'Cada avance sigue el rumbo resultante del paso anterior.'
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        self.source = QComboBox()
        self.source.addItem('Referencia ficticia editable (sin submarino)', 'example')
        self.source.addItem('Referencia de telemetría actual (desarmado)', 'live')
        layout.addWidget(self.source)
        row = QHBoxLayout()
        self.pose_inputs = []
        for title, low, high in [
            ('N [m]', -10000, 10000),
            ('E [m]', -10000, 10000),
            ('Prof. [m]', 0, 1000),
            ('Yaw [°]', -180, 180),
        ]:
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
            '(solo referencia del ensayo; no calibra el sensor)'
        )
        self.surface_verified.toggled.connect(self.invalidate_context)
        layout.addWidget(self.surface_verified)
        row = QHBoxLayout()
        self.kind = QComboBox()
        for text, kind in [
            ('Avanzar [m]; negativo = retroceder', 'advance'),
            ('Girar [°]; positivo = derecha', 'turn'),
            ('Profundidad absoluta [m]', 'depth'),
            ('Esperar [s]', 'wait'),
        ]:
            self.kind.addItem(text, kind)
        self.amount = QDoubleSpinBox()
        self.amount.setRange(-1000, 1000)
        self.amount.setDecimals(2)
        row.addWidget(self.kind)
        row.addWidget(self.amount)
        for text, callback in [
            ('Añadir', self.add_step),
            ('Quitar último', self.remove_step),
            ('Vaciar lista', self.clear_steps),
        ]:
            button = QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        self.step_text = QTextEdit()
        self.step_text.setReadOnly(True)
        self.step_text.setMaximumHeight(140)
        layout.addWidget(self.step_text)
        row = QHBoxLayout()
        for text, callback in [
            ('Previsualizar desde referencia actual', self.show_preview),
            ('Guardar Home de ensayo', self.save_home),
            ('Previsualizar regreso', self.show_home),
            ('Descartar objetivos', self.clear_preview),
        ]:
            button = QPushButton(text)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        self.home_label = QLabel('Home de ensayo: sin guardar')
        layout.addWidget(self.home_label)
        self.execution_status = QLabel('Ejecución: salida física bloqueada por defecto')
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
        title.setObjectName('SectionTitle')
        layout.addWidget(title)
        note = QLabel(
            'Este panel no habilita armado, propulsión ni movimientos. Un estado '
            'LISTO solo permite comenzar la prueba de telemetría/registro con el '
            'ROV desarmado.'
        )
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
        self.preflight_status.setObjectName('StatusBanner')
        set_widget_state(self.preflight_status, 'warning')
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
        title.setObjectName('SectionTitle')
        layout.addWidget(title)
        note = QLabel(
            'Los sticks no se controlan desde esta pantalla: llegan del mando '
            'Xbox por el nodo probado. Esta pestaña muestra el estado real y '
            'permite solicitar un modo solo con el ROV desarmado y RB suelto.'
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        accessory_map = QLabel(
            'D-pad ↑/↓ mueve cámara · D-pad ←/→ cambia luces ±10 %. '
            'Para accesorios, LB y RB deben estar sueltos y los ejes en neutro.'
        )
        accessory_map.setObjectName('MutedText')
        accessory_map.setWordWrap(True)
        layout.addWidget(accessory_map)
        self.operator_accessory_status = QLabel('Accesorios: esperando diagnóstico')
        self.operator_accessory_status.setObjectName('TelemetryValue')
        self.operator_accessory_status.setWordWrap(True)
        set_widget_state(self.operator_accessory_status, 'neutral')
        layout.addWidget(self.operator_accessory_status)
        self.mode_request_pub = self.node.create_publisher(String, '/uuv/control/mode_request', 10)
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
        self.mode_status = QLabel('Cambios de modo deshabilitados en este lanzamiento')
        self.mode_status.setWordWrap(True)
        layout.addWidget(self.mode_status)
        mode_note = QLabel(
            'GUIDED aquí solo es un modo informado por ArduSub. No activa el '
            'ejecutor ni genera setpoints ni movimientos autónomos.'
        )
        mode_note.setWordWrap(True)
        layout.addWidget(mode_note)
        layout.addStretch()
        return page

    def mode_request_ready(self):
        return (
            mode_request_allowed(
                enabled=self.enable_mode_requests,
                connected=self.data.get('connected') is True,
                armed=self.data.get('armed'),
                deadman=self.data.get('deadman'),
            )
            and self.pending_mode is None
        )

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
        self.update_operator_accessories()
        ready = self.mode_request_ready()
        self.mode_apply.setEnabled(ready)
        self.mode_selector.setEnabled(ready)
        if not self.enable_mode_requests:
            self.mode_status.setText(
                'Cambios de modo deshabilitados: inicia con mode_control:=true'
            )
            return
        if self.pending_mode is None:
            return
        if self.data.get('connected') is not True:
            self.mode_status.setText('Sin conexión: cambio no confirmado')
            self.pending_mode = None
            return
        mode_stamp = self.data.stamp('mode')
        if (
            self.data.get('mode') == self.pending_mode
            and mode_stamp is not None
            and mode_stamp > self.mode_requested_at
        ):
            self.mode_status.setText(f'Modo confirmado por el ROV: {self.pending_mode}')
            self.pending_mode = None
        elif self.data.clock() - self.mode_requested_at > 5.0:
            self.mode_status.setText(
                'Sin confirmación en 5 s. Revisa el modo real y la terminal.'
            )
            self.pending_mode = None

    def update_operator_accessories(self):
        """Explain both launch-level and gesture-level accessory blockers."""
        if not self.operator_profile:
            return
        bridge_fresh = self.data.get('accessory_status') is True
        xbox_fresh = self.data.get('xbox_status') is True
        bridge = self.accessory_status_data if bridge_fresh else {}
        xbox = self.xbox_status_data if xbox_fresh else {}
        if not bridge_fresh:
            text, state = 'Accesorios: puente sin diagnóstico reciente', 'warning'
        elif not bridge.get('enabled'):
            text = 'Accesorios bloqueados: inicia con accessory_control:=true'
            state = 'info'
        elif not bridge.get('connected'):
            text = 'Accesorios habilitados, pero el autopiloto no tiene enlace'
            state = 'warning'
        elif not xbox_fresh:
            text = 'Puente de accesorios listo; mando sin mensajes recientes'
            state = 'warning'
        elif not xbox.get('accessory_allowed'):
            text = 'Gesto de accesorios bloqueado: ' + (
                xbox.get('blocker') or 'entrada no válida'
            )
            state = 'warning'
        else:
            text = 'Cámara/luces listas · D-pad disponible'
            state = 'success'
        self.operator_accessory_status.setText(text)
        set_widget_state(self.operator_accessory_status, state)

    def update_preflight(self, *_):
        if not hasattr(self, 'preflight_checks'):
            return
        checks = {key: check.isChecked() for key, check in self.preflight_checks.items()}
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
                'LISTO PARA OBSERVACIÓN/REGISTRO · ROV desarmado · salida ROS bloqueada'
            )
            set_widget_state(self.preflight_status, 'success')
            self.preflight_detail.setPlainText(
                'Siguiente paso: inicia record_pool.sh en otra terminal, introduce '
                'el ROV en el agua y mantén 60 s de telemetría quieta. Este estado '
                'no autoriza armado ni movimiento.'
            )
        else:
            self.preflight_status.setText('NO GO · no iniciar la sesión')
            set_widget_state(self.preflight_status, 'warning')
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
        names = {
            'advance': 'Avance [m]',
            'turn': 'Giro [°]',
            'depth': 'Profundidad absoluta [m]',
            'wait': 'Espera [s]',
        }
        self.step_text.setPlainText(
            '\n'.join(f'{i}. {names[k]}: {v:g}' for i, (k, v) in enumerate(self.steps, 1))
        )
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
        lines = [
            f'ENSAYO {context} — sin envío al submarino',
            f'Inicio N={start.north:.2f}, E={start.east:.2f}, '
            f'prof={start.depth:.2f} m, yaw={start.yaw:.1f}°',
        ]
        for i, (step, goal) in enumerate(zip(steps, targets), 1):
            lines.append(
                f'{i}. {step[0]} {step[1]:g} → '
                f'N={goal.north:.2f}, E={goal.east:.2f}, '
                f'prof={goal.depth:.2f} m, yaw={goal.yaw:.1f}°'
            )
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

    def recent_values(self, samples, horizon_s):
        """Return finite values received inside a short local-arrival window."""
        now = self.data.clock()
        return [value for stamp, value in samples if 0.0 <= now - stamp <= horizon_s]

    def update_power_summary(self):
        """Present current peaks and voltage sag without claiming hardware protection."""
        current = self.data.get('current')
        voltage = self.data.get('voltage')
        current_1s = self.recent_values(self.current_samples, 1.0)
        current_60s = self.recent_values(self.current_samples, 60.0)
        voltage_1s = self.recent_values(self.voltage_samples, 1.0)
        if current is None and voltage is None:
            self.power_summary.setText('Energía: sin tensión/corriente reciente')
            set_widget_state(self.power_summary, 'neutral')
            return
        parts = []
        if current is not None:
            parts.append(f'corriente {current:.2f} A')
        if current_1s:
            parts.append(f'pico 1 s {max(current_1s):.2f} A')
        if current_60s:
            parts.append(f'pico 60 s {max(current_60s):.2f} A')
        if voltage is not None:
            parts.append(f'tensión {voltage:.2f} V')
        if voltage_1s:
            parts.append(f'mínimo 1 s {min(voltage_1s):.2f} V')
        if self.current_limit_a is not None:
            parts.append(f'MOT_BAT_CURR_MAX {self.current_limit_a:g} A')
        self.power_summary.setText('Energía · ' + ' · '.join(parts))
        peak = max(current_1s) if current_1s else current
        if peak is not None and self.current_limit_a is not None:
            if peak >= self.current_limit_a:
                state = 'danger'
            elif peak >= 0.80 * self.current_limit_a:
                state = 'warning'
            else:
                state = 'success'
        else:
            state = 'info'
        set_widget_state(self.power_summary, state)

    def update_alert_summary(self):
        """Render the newest ArduSub message with MAVLink severity."""
        if not self.alerts:
            self.alert_summary.setText('ArduSub: sin mensajes STATUSTEXT recibidos')
            set_widget_state(self.alert_summary, 'neutral')
            return
        severity, text, stamp = self.alerts[-1]
        age = max(0.0, time.monotonic() - stamp)
        self.alert_summary.setText(
            f'ArduSub [{severity}] · {text} · recibido hace {age:.1f} s'
        )
        state = 'danger' if severity <= 3 else 'warning' if severity == 4 else 'info'
        set_widget_state(self.alert_summary, state)

    def update_camera_status(self):
        """Distinguish launch-disabled, missing and stale video states."""
        now = time.monotonic()
        while self.camera_frame_times and now - self.camera_frame_times[0] > 2.0:
            self.camera_frame_times.popleft()
        video_fresh = self.data.get('video') is True
        age = self.data.age('video')
        if video_fresh:
            fps = 0.0
            if len(self.camera_frame_times) >= 2:
                span = self.camera_frame_times[-1] - self.camera_frame_times[0]
                if span > 0:
                    fps = (len(self.camera_frame_times) - 1) / span
            self.camera_status.setText(
                f'Cámara activa · {fps:.1f} FPS recibidos · último frame hace {age:.1f} s'
            )
            set_widget_state(self.camera_status, 'success')
        elif not self.camera_expected:
            self.camera_status.setText('Cámara deshabilitada por camera:=false')
            set_widget_state(self.camera_status, 'info')
        elif age is None:
            self.camera_status.setText(
                'Cámara habilitada, pero todavía no llegaron imágenes'
            )
            set_widget_state(self.camera_status, 'warning')
        else:
            self.camera_status.setText(
                f'Imagen detenida · último frame recibido hace {age:.1f} s'
            )
            set_widget_state(self.camera_status, 'danger')

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
            if value is None or (
                not connected and key not in ('connected', 'output', 'accessory_output')
            ):
                label.setText(
                    'Sin datos recientes'
                    if age is None
                    else f'Sin datos válidos / conexión · última recepción {age:.1f} s'
                )
            else:
                if isinstance(value, tuple):
                    values = tuple(math.degrees(v) for v in value) if key == 'attitude' else value
                    text = ' / '.join(f'{v:.3f}' for v in values)
                elif isinstance(value, bool):
                    text = 'Sí' if value else 'No'
                elif key == 'current':
                    recent = self.recent_values(self.current_samples, 1.0)
                    peak = max(recent) if recent else float(value)
                    text = f'{float(value):.2f} A · pico 1 s {peak:.2f} A'
                elif key == 'command_scale':
                    text = f'{100.0 * float(value):.1f} %'
                elif key == 'lights_percent':
                    text = f'{float(value):.1f} % (estimado)'
                elif isinstance(value, float):
                    text = f'{value:.3f}'
                else:
                    text = str(value)
                label.setText(f'{text} · recibido hace {age:.1f} s')
            if key == 'mode':
                state = (
                    'warning'
                    if connected and value == 'MANUAL'
                    else ('success' if connected and value is not None else 'neutral')
                )
                set_widget_state(label, state)
            elif key == 'connected':
                state = (
                    'success'
                    if value is True
                    else ('danger' if value is False else 'neutral')
                )
                set_widget_state(label, state)
            elif key == 'armed':
                state = (
                    'danger'
                    if connected and value is True
                    else ('success' if connected and value is False else 'neutral')
                )
                set_widget_state(label, state)
            elif key in ('output', 'accessory_output', 'motion_allowed'):
                state = (
                    'warning'
                    if connected and value is True
                    else ('success' if connected and value is False else 'neutral')
                )
                set_widget_state(label, state)
            elif key == 'current':
                peak_values = self.recent_values(self.current_samples, 1.0)
                peak = max(peak_values) if peak_values else value
                if peak is None or self.current_limit_a is None:
                    state = 'neutral'
                elif peak >= self.current_limit_a:
                    state = 'danger'
                elif peak >= 0.80 * self.current_limit_a:
                    state = 'warning'
                else:
                    state = 'success'
                set_widget_state(label, state)
        flags = self.data.get('ekf') if connected else None
        self.ekf_text.setText('EKF: ' + ekf_description(flags))
        self.update_power_summary()
        self.update_alert_summary()
        if not connected:
            self.status.setText(
                'Puente ROS activo · autopiloto sin heartbeat reciente'
            )
            set_widget_state(self.status, 'danger')
        elif self.data.get('armed') is not False or self.data.get('output') is not False:
            self.status.setText(
                'Revisa el estado: esta prueba requiere desarmado y salida ROS deshabilitada.'
            )
            set_widget_state(self.status, 'warning')
        else:
            self.status.setText(
                'Conectado y desarmado · salida ROS deshabilitada · navegación aún no validada'
            )
            set_widget_state(self.status, 'success')
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
                self.drift.setText(
                    f'Variación XY: {distance:.3f} m · máximo: {maximum:.3f} m '
                    f'· {self.data.clock() - stamp:.0f} s desde referencia'
                )
        if self.pending_image is not None:
            frame = QPixmap()
            if frame.loadFromData(self.pending_image):
                self.pixmap = frame
            self.pending_image = None
        if self.pixmap is not None and self.data.get('video'):
            self.video.setPixmap(
                self.pixmap.scaled(self.video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        else:
            self.video.setText(
                (
                    'CÁMARA DESHABILITADA EN EL LANZAMIENTO'
                    if not self.camera_expected
                    else 'CÁMARA SIN DATOS RECIENTES\n\n'
                    'Esperando /camera/image/compressed'
                )
            )
        self.update_camera_status()
        self.sonar.refresh()
        self.update_preflight()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    apply_theme(app)
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

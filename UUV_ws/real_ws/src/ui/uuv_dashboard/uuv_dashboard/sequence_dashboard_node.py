"""Operator-focused controls and live state for measured GUIDED sequences."""

import json
import math
import signal
import sys
import time
import uuid

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QInputDialog,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Int8, String

from .configuration_model import (
    ConfigurationStore,
    default_document,
    delete_profile,
    normalize_profile,
    profile_to_raw_parameters,
    raw_parameters_match,
    rename_profile,
    set_vehicle_snapshot,
    snapshot_matches_vehicle,
    update_profile,
)
from .mission_model import (
    delete_mission,
    duplicate_step,
    empty_document as empty_mission_document,
    mission_metrics,
    MissionStore,
    rectangle_waypoints,
    rename_mission,
    save_mission,
)
from .pool_dashboard_node import PoolDashboard
from .sequence_executor_model import move_step, replace_step
from .theme import apply_theme, set_widget_state
from .ui_state_model import (
    controller_tuning_payload,
    operator_guidance,
    workflow_states,
)


class SequenceDashboard(PoolDashboard):
    def __init__(self, node):
        super().__init__(node)
        self.setWindowTitle('RobotX | Secuencias GUIDED')
        self.resize(1480, 900)
        self.setMinimumSize(1180, 760)
        app_title = self.layout().itemAt(0).widget()
        app_title.setText('RobotX · Control de misión UUV')
        app_title.setObjectName('AppTitle')
        self.status.setObjectName('GlobalStatus')
        self.ui_id = uuid.uuid4().hex
        self.ui_count = 0
        self.action_count = 0
        self.exec_status = None
        self.exec_status_at = 0.0
        self.prepared_signature = None
        self.pending_prepare = None
        self.parameter_status = None
        self.parameter_status_at = 0.0
        self.parameter_request_count = 0
        self.configuration_loading = False
        self.configuration_dirty = False
        self.configuration_error = ''
        self.loaded_profile_name = None
        self.configuration_store = ConfigurationStore()
        try:
            self.configuration_document = self.configuration_store.load()
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.configuration_document = default_document()
            self.configuration_error = str(exc)
        self.request_pub = node.create_publisher(String, '/uuv/sequence/request', 10)
        self.alive_pub = node.create_publisher(String, '/uuv/sequence/ui_alive', 1)
        self.parameter_request_pub = node.create_publisher(
            String, '/uuv/parameters/request', 10
        )
        self.subscriptions.append(
            node.create_subscription(String, '/uuv/sequence/status', self.receive_status, 1)
        )
        self.subscriptions.append(
            node.create_subscription(
                String,
                '/uuv/parameters/status',
                self.receive_parameter_status,
                10,
            )
        )
        self.buttons = {}
        self.relative_steps = self.steps
        self.absolute_steps = []
        self.active_plan_mode = 'relative'
        self.mission_store = MissionStore()
        self.mission_error = ''
        try:
            self.mission_document = self.mission_store.load()
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.mission_document = empty_mission_document()
            self.mission_error = str(exc)
        self.loaded_mission_name = self.mission_document.get('active_mission')
        self.mission_dirty = False
        self.mission_loading = True
        self.accessory_status_data = {}
        self.accessory_status_at = 0.0
        self.xbox_status_data = {}
        self.xbox_status_at = 0.0
        self.camera_tilt_command = 0.0
        self.camera_tilt_pub = node.create_publisher(
            Float32, '/uuv/control/camera_tilt', 10
        )
        self.lights_step_pub = node.create_publisher(
            Int8, '/uuv/control/lights_step', 10
        )
        self.lights_setpoint_pub = node.create_publisher(
            Float32, '/uuv/control/lights_percent_setpoint', 10
        )
        self.lights_off_pub = node.create_publisher(
            Empty, '/uuv/control/lights_off', 10
        )
        self.command_scale_setpoint_pub = node.create_publisher(
            Float32, '/uuv/control/command_scale_setpoint', 10
        )
        configuration_page = QWidget()
        configuration_page.setObjectName('RobotXRoot')
        configuration_layout = QVBoxLayout(configuration_page)
        configuration_layout.setContentsMargins(10, 10, 10, 10)
        configuration_layout.setSpacing(10)
        configuration_title = QLabel('Configuración y seguridad')
        configuration_title.setObjectName('SectionTitle')
        configuration_layout.addWidget(configuration_title)
        configuration_note = QLabel(
            'Guardar conserva el perfil local. Aplicar al ROV es una acción separada, '
            'protegida y verificada mediante lectura MAVLink.'
        )
        configuration_note.setObjectName('MutedText')
        configuration_note.setWordWrap(True)
        configuration_layout.addWidget(configuration_note)
        self.runtime_scope = QLabel(
            'Ámbito: Xbox/MANUAL se limita por command_scale del lanzamiento; '
            'los máximos de sesión afectan solo secuencias; PWM/WPNAV permanecen en ArduSub.'
        )
        self.runtime_scope.setObjectName('MutedText')
        self.runtime_scope.setWordWrap(True)
        configuration_layout.addWidget(self.runtime_scope)
        configuration_cards = QHBoxLayout()
        configuration_cards.setSpacing(10)
        configuration_layout.addLayout(configuration_cards, 1)

        page = QWidget()
        page.setObjectName('RobotXRoot')
        page.setMinimumHeight(690)
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        workflow = QFrame()
        workflow.setObjectName('TopBar')
        workflow_layout = QHBoxLayout(workflow)
        workflow_layout.setContentsMargins(8, 6, 8, 6)
        self.stage_labels = {}
        for stage, text in (
            ('configure', '1  CONFIGURAR'),
            ('prepare', '2  PREPARAR'),
            ('arm', '3  ARMAR'),
            ('guided', '4  GUIDED'),
            ('home', '5  FIJAR HOME'),
            ('execute', '6  EJECUTAR'),
        ):
            label = QLabel(text)
            label.setObjectName('StagePill')
            label.setAlignment(Qt.AlignCenter)
            set_widget_state(label, 'neutral')
            self.stage_labels[stage] = label
            workflow_layout.addWidget(label)
        layout.addWidget(workflow)

        phase_card = QFrame()
        phase_card.setObjectName('PanelCard')
        phase_layout = QVBoxLayout(phase_card)
        self.phase_title = QLabel('SIN DATOS DEL EJECUTOR')
        self.phase_title.setObjectName('StatusBanner')
        set_widget_state(self.phase_title, 'warning')
        self.phase_title.setAlignment(Qt.AlignCenter)
        phase_layout.addWidget(self.phase_title)
        self.phase_detail = QLabel('Esperando estado de la secuencia')
        self.phase_detail.setObjectName('Subtitle')
        self.phase_detail.setWordWrap(True)
        self.phase_detail.setAlignment(Qt.AlignCenter)
        phase_layout.addWidget(self.phase_detail)
        layout.addWidget(phase_card)

        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter, 1)

        planning_panel = QFrame()
        planning_panel.setObjectName('PanelCard')
        planning_layout = QVBoxLayout(planning_panel)
        planning_layout.setContentsMargins(10, 10, 10, 10)
        planning_layout.setSpacing(6)
        planning_title = QLabel('Plan de movimiento')
        planning_title.setObjectName('SectionTitle')
        planning_layout.addWidget(planning_title)
        planner_note = QLabel(
            'Elige un único plan activo. Ambos modos comparten ejecutor, watchdog, '
            'límites físicos y cancelación a MANUAL.'
        )
        planner_note.setObjectName('MutedText')
        planner_note.setWordWrap(True)
        planning_layout.addWidget(planner_note)
        mission_bar = QFrame()
        mission_bar.setObjectName('TopBar')
        mission_layout = QGridLayout(mission_bar)
        mission_layout.setContentsMargins(8, 6, 8, 6)
        mission_layout.addWidget(self.field_label('Misión guardada'), 0, 0)
        self.mission_combo = QComboBox()
        self.mission_combo.setMinimumWidth(210)
        mission_layout.addWidget(self.mission_combo, 0, 1)
        self.load_mission_button = QPushButton('Cargar')
        self.load_mission_button.clicked.connect(self.load_selected_mission)
        mission_layout.addWidget(self.load_mission_button, 0, 2)
        self.save_mission_button = QPushButton('Guardar')
        self.save_mission_button.setObjectName('SuccessAction')
        self.save_mission_button.clicked.connect(self.save_current_mission)
        mission_layout.addWidget(self.save_mission_button, 0, 3)
        self.save_mission_as_button = QPushButton('Guardar como…')
        self.save_mission_as_button.clicked.connect(self.save_mission_as)
        mission_layout.addWidget(self.save_mission_as_button, 0, 4)
        self.rename_mission_button = QPushButton('Renombrar')
        self.rename_mission_button.clicked.connect(self.rename_current_mission)
        mission_layout.addWidget(self.rename_mission_button, 1, 2)
        self.delete_mission_button = QPushButton('Eliminar')
        self.delete_mission_button.clicked.connect(self.delete_current_mission)
        mission_layout.addWidget(self.delete_mission_button, 1, 3)
        self.new_mission_button = QPushButton('Nueva / limpiar')
        self.new_mission_button.clicked.connect(self.new_mission)
        mission_layout.addWidget(self.new_mission_button, 1, 4)
        self.mission_status = QLabel('Borrador automático activo')
        self.mission_status.setObjectName('MutedText')
        self.mission_status.setWordWrap(True)
        mission_layout.addWidget(self.mission_status, 1, 0, 1, 2)
        planning_layout.addWidget(mission_bar)
        self.planner_tabs = QTabWidget()
        relative_page = QWidget()
        relative_layout = QVBoxLayout(relative_page)
        relative_layout.setContentsMargins(4, 4, 4, 4)
        relative_layout.setSpacing(6)

        limits_group = QGroupBox('Límites y referencia de la sesión')
        limits_group.setMinimumWidth(360)
        limits_grid = QGridLayout(limits_group)
        limits_grid.setHorizontalSpacing(12)
        limits_grid.setVerticalSpacing(5)
        limits_grid.addWidget(
            self.field_label(
                'Profundidad del sensor [m]',
                'Desfase vertical entre el sensor de presión y la referencia del vehículo.',
            ),
            0,
            0,
        )
        self.sensor_depth = QDoubleSpinBox()
        self.sensor_depth.setRange(0, 0.5)
        self.sensor_depth.setDecimals(2)
        self.sensor_depth.setSingleStep(0.01)
        limits_grid.addWidget(self.sensor_depth, 0, 1)
        limits_grid.addWidget(
            self.field_label('Profundidad máxima [m]', 'Límite absoluto de esta sesión.'),
            1,
            0,
        )
        self.maximum_depth = QDoubleSpinBox()
        self.maximum_depth.setRange(0.2, 3)
        self.maximum_depth.setDecimals(2)
        self.maximum_depth.setSingleStep(0.01)
        self.maximum_depth.setKeyboardTracking(False)
        self.maximum_depth.setValue(0.2)
        limits_grid.addWidget(self.maximum_depth, 1, 1)
        limits_grid.addWidget(
            self.field_label(
                'Radio desde Home [m]',
                'Desplazamiento horizontal máximo permitido.',
            ),
            2,
            0,
        )
        self.radius = QDoubleSpinBox()
        self.radius.setRange(0.5, 5)
        self.radius.setDecimals(2)
        self.radius.setSingleStep(0.05)
        self.radius.setValue(0.5)
        limits_grid.addWidget(self.radius, 2, 1)
        limits_grid.addWidget(self.field_label('Máx. horizontal [m/s]'), 3, 0)
        self.max_horizontal_speed = QDoubleSpinBox()
        self.max_horizontal_speed.setRange(0.001, 2.0)
        self.max_horizontal_speed.setDecimals(3)
        self.max_horizontal_speed.setSingleStep(0.01)
        self.max_horizontal_speed.setValue(0.20)
        limits_grid.addWidget(self.max_horizontal_speed, 3, 1)
        limits_grid.addWidget(self.field_label('Máx. vertical [m/s]'), 4, 0)
        self.max_vertical_speed = QDoubleSpinBox()
        self.max_vertical_speed.setRange(0.001, 1.0)
        self.max_vertical_speed.setDecimals(3)
        self.max_vertical_speed.setSingleStep(0.01)
        self.max_vertical_speed.setValue(0.10)
        limits_grid.addWidget(self.max_vertical_speed, 4, 1)
        limits_grid.addWidget(self.field_label('Máx. giro [°/s]'), 5, 0)
        self.max_yaw_speed = QDoubleSpinBox()
        self.max_yaw_speed.setRange(0.001, 180.0)
        self.max_yaw_speed.setDecimals(3)
        self.max_yaw_speed.setSingleStep(1.0)
        self.max_yaw_speed.setValue(10.0)
        limits_grid.addWidget(self.max_yaw_speed, 5, 1)
        note = QLabel(
            'El cero de profundidad es local a esta sesión. Los límites pertenecen '
            'al controlador y no sustituyen el margen físico de parada.'
        )
        note.setObjectName('MutedText')
        note.setWordWrap(True)
        limits_grid.addWidget(note, 6, 0, 1, 2)
        configuration_cards.addWidget(limits_group, 1)

        controller_group = QGroupBox('Retención de posición y recuperación')
        controller_group.setMinimumWidth(360)
        controller_grid = QGridLayout(controller_group)
        controller_grid.setHorizontalSpacing(12)
        controller_grid.setVerticalSpacing(5)

        controller_grid.addWidget(
            self.field_label(
                'Tolerancia XY total [m]',
                'Dentro de este error horizontal el objetivo se considera alcanzado.',
            ),
            0,
            0,
        )
        self.xy_tolerance = QDoubleSpinBox()
        self.xy_tolerance.setRange(0.01, 0.15)
        self.xy_tolerance.setDecimals(3)
        self.xy_tolerance.setSingleStep(0.005)
        self.xy_tolerance.setValue(0.03)
        controller_grid.addWidget(self.xy_tolerance, 0, 1)

        controller_grid.addWidget(
            self.field_label(
                'Recuperar desde [m]',
                'Fuera de este radio se activa la recuperación progresiva de posición.',
            ),
            1,
            0,
        )
        self.recovery_radius = QDoubleSpinBox()
        self.recovery_radius.setDecimals(3)
        self.recovery_radius.setSingleStep(0.005)
        self.recovery_radius.setValue(0.08)
        controller_grid.addWidget(self.recovery_radius, 1, 1)

        controller_grid.addWidget(
            self.field_label(
                'Máx. recuperación [m/s]',
                'Velocidad horizontal máxima usada para volver a la posición objetivo.',
            ),
            2,
            0,
        )
        self.recovery_speed = QDoubleSpinBox()
        self.recovery_speed.setDecimals(3)
        self.recovery_speed.setSingleStep(0.005)
        self.recovery_speed.setValue(0.08)
        controller_grid.addWidget(self.recovery_speed, 2, 1)

        controller_grid.addWidget(
            self.field_label(
                'Impulso mínimo [m/s]',
                'Mínimo comando de recuperación, siempre menor o igual que el máximo.',
            ),
            3,
            0,
        )
        self.recovery_min_speed = QDoubleSpinBox()
        self.recovery_min_speed.setDecimals(3)
        self.recovery_min_speed.setSingleStep(0.005)
        self.recovery_min_speed.setValue(0.02)
        controller_grid.addWidget(self.recovery_min_speed, 3, 1)

        controller_grid.addWidget(
            self.field_label(
                'Refuerzo tras respuesta lenta [s]',
                'Espera antes de elevar gradualmente el esfuerzo de recuperación.',
            ),
            4,
            0,
        )
        self.recovery_delay = QDoubleSpinBox()
        self.recovery_delay.setRange(0.5, 10.0)
        self.recovery_delay.setDecimals(1)
        self.recovery_delay.setSingleStep(0.5)
        self.recovery_delay.setValue(2.0)
        controller_grid.addWidget(self.recovery_delay, 4, 1)

        controller_grid.addWidget(
            self.field_label(
                'Ganancia posición [1/s]',
                'Convierte el error de posición en velocidad de corrección.',
            ),
            5,
            0,
        )
        self.position_gain = QDoubleSpinBox()
        self.position_gain.setRange(0.10, 2.0)
        self.position_gain.setDecimals(2)
        self.position_gain.setSingleStep(0.05)
        self.position_gain.setValue(0.80)
        controller_grid.addWidget(self.position_gain, 5, 1)

        controller_grid.addWidget(
            self.field_label(
                'Frenado por velocidad [s]',
                'Amortigua la corrección según la velocidad medida para evitar sobrepasos.',
            ),
            6,
            0,
        )
        self.velocity_damping = QDoubleSpinBox()
        self.velocity_damping.setRange(0.05, 3.0)
        self.velocity_damping.setDecimals(2)
        self.velocity_damping.setSingleStep(0.05)
        self.velocity_damping.setValue(1.20)
        controller_grid.addWidget(self.velocity_damping, 6, 1)
        controller_note = QLabel(
            'Valores conservadores del ejecutor físico. Cambiarlos invalida la preparación '
            'actual y exige preparar de nuevo.'
        )
        controller_note.setObjectName('MutedText')
        controller_note.setWordWrap(True)
        controller_grid.addWidget(controller_note, 7, 0, 1, 2)
        configuration_cards.addWidget(controller_group, 1)

        self.controller_inputs = (
            self.xy_tolerance,
            self.recovery_radius,
            self.recovery_speed,
            self.recovery_min_speed,
            self.recovery_delay,
            self.position_gain,
            self.velocity_damping,
        )
        self.xy_tolerance.valueChanged.connect(self.sync_controller_ranges)
        self.radius.valueChanged.connect(self.sync_controller_ranges)
        self.max_horizontal_speed.valueChanged.connect(self.sync_controller_ranges)
        self.recovery_speed.valueChanged.connect(self.sync_controller_ranges)
        for entry in self.controller_inputs:
            entry.valueChanged.connect(self.invalidate_preparation)
        self.sync_controller_ranges()

        vehicle_group = QGroupBox('Propulsión y controlador ArduSub')
        vehicle_group.setMinimumWidth(360)
        vehicle_grid = QGridLayout(vehicle_group)
        vehicle_grid.setHorizontalSpacing(12)
        vehicle_grid.setVerticalSpacing(7)

        vehicle_grid.addWidget(
            self.field_label(
                'Límite de señal de propulsión [%]',
                'Rango técnico completo: 1–100 % del recorrido simétrico '
                '1100–1900 µs. No equivale directamente a potencia ni corriente.',
            ),
            0,
            0,
        )
        self.propulsion_signal_limit = QDoubleSpinBox()
        self.propulsion_signal_limit.setRange(1.0, 100.0)
        self.propulsion_signal_limit.setDecimals(1)
        self.propulsion_signal_limit.setSingleStep(1.0)
        self.propulsion_signal_limit.setValue(37.5)
        vehicle_grid.addWidget(self.propulsion_signal_limit, 0, 1)

        vehicle_grid.addWidget(
            self.field_label(
                'Escala Xbox/MANUAL de sesión [%]',
                'Ajuste temporal 0–100 % del mando. No modifica parámetros '
                'persistentes y requiere RB liberado al aplicarlo.',
            ),
            1,
            0,
        )
        manual_scale_row = QHBoxLayout()
        self.manual_command_scale = QDoubleSpinBox()
        self.manual_command_scale.setRange(0.0, 100.0)
        self.manual_command_scale.setDecimals(1)
        self.manual_command_scale.setSingleStep(5.0)
        self.manual_command_scale.setValue(20.0)
        manual_scale_row.addWidget(self.manual_command_scale)
        self.apply_manual_scale_button = QPushButton('Aplicar')
        self.apply_manual_scale_button.clicked.connect(
            self.apply_manual_command_scale
        )
        manual_scale_row.addWidget(self.apply_manual_scale_button)
        vehicle_grid.addLayout(manual_scale_row, 1, 1)

        vehicle_grid.addWidget(
            self.field_label(
                'Velocidad máxima de ascenso [m/s]',
                'Se guarda internamente como WPNAV_SPEED_UP en cm/s.',
            ),
            2,
            0,
        )
        self.max_ascent_speed = QDoubleSpinBox()
        self.max_ascent_speed.setRange(0.0, 10.0)
        self.max_ascent_speed.setDecimals(2)
        self.max_ascent_speed.setSingleStep(0.05)
        self.max_ascent_speed.setValue(0.10)
        vehicle_grid.addWidget(self.max_ascent_speed, 2, 1)

        vehicle_grid.addWidget(
            self.field_label(
                'Velocidad máxima de descenso [m/s]',
                'Se guarda internamente como WPNAV_SPEED_DN en cm/s.',
            ),
            3,
            0,
        )
        self.max_descent_speed = QDoubleSpinBox()
        self.max_descent_speed.setRange(0.0, 5.0)
        self.max_descent_speed.setDecimals(2)
        self.max_descent_speed.setSingleStep(0.05)
        self.max_descent_speed.setValue(0.10)
        vehicle_grid.addWidget(self.max_descent_speed, 3, 1)

        vehicle_grid.addWidget(
            self.field_label(
                'Aceleración vertical máxima [m/s²]',
                'Se guarda como WPNAV_ACCEL_Z. El panel expone el rango técnico '
                '0–500 cm/s² para pruebas del propietario.',
            ),
            4,
            0,
        )
        self.max_vertical_accel = QDoubleSpinBox()
        self.max_vertical_accel.setRange(0.0, 5.0)
        self.max_vertical_accel.setDecimals(2)
        self.max_vertical_accel.setSingleStep(0.05)
        self.max_vertical_accel.setValue(0.50)
        vehicle_grid.addWidget(self.max_vertical_accel, 4, 1)

        self.vehicle_inputs = (
            self.propulsion_signal_limit,
            self.max_ascent_speed,
            self.max_descent_speed,
            self.max_vertical_accel,
        )
        self.parameter_raw_values = QLabel(
            'Valores técnicos propuestos: PWM 1350–1650 µs · subida 10 cm/s · '
            'bajada 10 cm/s · aceleración 50 cm/s²'
        )
        self.parameter_raw_values.setObjectName('MutedText')
        self.parameter_raw_values.setWordWrap(True)
        vehicle_grid.addWidget(self.parameter_raw_values, 5, 0, 1, 2)

        self.parameter_state = QLabel('Esperando lectura del autopiloto')
        self.parameter_state.setObjectName('TelemetryValue')
        self.parameter_state.setWordWrap(True)
        set_widget_state(self.parameter_state, 'neutral')
        vehicle_grid.addWidget(self.parameter_state, 6, 0, 1, 2)

        self.parameter_comparison = QLabel('Comparación: esperando lectura del ROV')
        self.parameter_comparison.setObjectName('MutedText')
        self.parameter_comparison.setWordWrap(True)
        vehicle_grid.addWidget(self.parameter_comparison, 7, 0, 1, 2)

        self.electrical_state = QLabel(
            'Protección eléctrica: esperando parámetros de solo lectura'
        )
        self.electrical_state.setObjectName('TelemetryValue')
        self.electrical_state.setWordWrap(True)
        set_widget_state(self.electrical_state, 'neutral')
        vehicle_grid.addWidget(self.electrical_state, 8, 0, 1, 2)

        parameter_buttons = QGridLayout()
        self.read_parameters_button = QPushButton('Leer del ROV')
        self.read_parameters_button.clicked.connect(self.request_vehicle_parameters)
        self.apply_parameters_button = QPushButton('Aplicar al ROV')
        self.apply_parameters_button.setObjectName('GuidedAction')
        self.apply_parameters_button.clicked.connect(self.apply_vehicle_parameters)
        self.restore_parameters_button = QPushButton('Restaurar anterior')
        self.restore_parameters_button.clicked.connect(self.restore_vehicle_parameters)
        parameter_buttons.addWidget(self.read_parameters_button, 0, 0)
        parameter_buttons.addWidget(self.apply_parameters_button, 0, 1)
        parameter_buttons.addWidget(self.restore_parameters_button, 1, 0, 1, 2)
        vehicle_grid.addLayout(parameter_buttons, 9, 0, 1, 2)
        vehicle_warning = QLabel(
            'Estos límites reducen órdenes, pero no garantizan una corriente menor '
            'de 75 A ni eliminan el pico eléctrico de encendido.'
        )
        vehicle_warning.setObjectName('SafetyNote')
        vehicle_warning.setWordWrap(True)
        vehicle_grid.addWidget(vehicle_warning, 10, 0, 1, 2)
        configuration_cards.addWidget(vehicle_group, 1)

        profile_bar = QFrame()
        profile_bar.setObjectName('TopBar')
        profile_layout = QGridLayout(profile_bar)
        profile_layout.setContentsMargins(8, 8, 8, 8)
        profile_layout.addWidget(self.field_label('Perfil local'), 0, 0)
        self.profile_combo = QComboBox()
        self.profile_combo.setMinimumWidth(220)
        profile_layout.addWidget(self.profile_combo, 0, 1, 1, 2)
        self.load_profile_button = QPushButton('Cargar perfil')
        self.load_profile_button.clicked.connect(self.load_selected_profile)
        profile_layout.addWidget(self.load_profile_button, 0, 3)
        self.save_profile_button = QPushButton('Guardar configuración')
        self.save_profile_button.setObjectName('SuccessAction')
        self.save_profile_button.clicked.connect(self.save_current_profile)
        profile_layout.addWidget(self.save_profile_button, 0, 4)
        self.save_profile_as_button = QPushButton('Guardar como…')
        self.save_profile_as_button.clicked.connect(self.save_profile_as)
        profile_layout.addWidget(self.save_profile_as_button, 1, 1)
        self.rename_profile_button = QPushButton('Renombrar')
        self.rename_profile_button.clicked.connect(self.rename_current_profile)
        profile_layout.addWidget(self.rename_profile_button, 1, 2)
        self.delete_profile_button = QPushButton('Eliminar')
        self.delete_profile_button.clicked.connect(self.delete_current_profile)
        profile_layout.addWidget(self.delete_profile_button, 1, 3)
        self.reload_profile_button = QPushButton('Descartar cambios')
        self.reload_profile_button.clicked.connect(self.load_selected_profile)
        profile_layout.addWidget(self.reload_profile_button, 1, 4)
        self.profile_combo.currentTextChanged.connect(self.profile_selection_changed)
        configuration_layout.addWidget(profile_bar)

        self.configuration_status = QLabel('Configuración local aún no cargada')
        self.configuration_status.setObjectName('StatusBanner')
        self.configuration_status.setWordWrap(True)
        set_widget_state(self.configuration_status, 'neutral')
        configuration_layout.addWidget(self.configuration_status)

        self.configuration_inputs = (
            self.sensor_depth,
            self.maximum_depth,
            self.radius,
            self.max_horizontal_speed,
            self.max_vertical_speed,
            self.max_yaw_speed,
        ) + self.controller_inputs + self.vehicle_inputs
        for entry in self.configuration_inputs:
            entry.valueChanged.connect(self.configuration_changed)
        for entry in self.vehicle_inputs:
            entry.valueChanged.connect(self.update_parameter_preview)
        self.sonar.source.currentIndexChanged.connect(self.configuration_changed)
        self.sonar.range_m.valueChanged.connect(self.configuration_changed)
        self.sonar.persistence.valueChanged.connect(self.configuration_changed)
        self.sonar.min_intensity.valueChanged.connect(self.configuration_changed)
        self.update_parameter_preview()

        # Reuse the existing editor and validation of individual steps.
        editor = QGroupBox('Editor de orden')
        editor.setMinimumHeight(168)
        editor_layout = QVBoxLayout(editor)
        editor_fields = QGridLayout()
        editor_layout.addLayout(editor_fields)
        editor_note = QLabel(
            'Los movimientos temporales se limitan a 5 s y se validan con 1 s '
            'adicional de margen de frenado dentro del radio/profundidad de sesión.'
        )
        editor_note.setObjectName('MutedText')
        editor_note.setWordWrap(True)
        editor_layout.addWidget(editor_note)
        self.kind.clear()
        for text, kind in [
            ('Posición · avanzar/retroceder', 'advance'),
            ('Posición · derecha/izquierda', 'lateral'),
            ('Posición · girar por ángulo', 'turn'),
            ('Posición · profundidad absoluta', 'depth'),
            ('Velocidad/tiempo · avanzar/retroceder', 'velocity_surge'),
            ('Velocidad/tiempo · derecha/izquierda', 'velocity_sway'),
            ('Velocidad/tiempo · descender/ascender', 'velocity_heave'),
            ('Velocidad/tiempo · girar', 'velocity_yaw'),
            ('Esperar', 'wait'),
        ]:
            self.kind.addItem(text, kind)
        self.primary_label = QLabel('Distancia [m]')
        self.primary_label.setObjectName('FieldLabel')
        self.secondary_label = QLabel('Velocidad máx. [m/s]')
        self.secondary_label.setObjectName('FieldLabel')
        self.secondary = QDoubleSpinBox()
        self.secondary.setDecimals(3)
        editor_fields.addWidget(self.field_label('Tipo de movimiento'), 0, 0)
        editor_fields.addWidget(self.kind, 0, 1, 1, 3)
        editor_fields.addWidget(self.primary_label, 1, 0)
        editor_fields.addWidget(self.amount, 1, 1)
        editor_fields.addWidget(self.secondary_label, 1, 2)
        editor_fields.addWidget(self.secondary, 1, 3)
        editor_buttons = QGridLayout()
        editor_layout.addLayout(editor_buttons)
        for index, (title, callback, attribute) in enumerate([
            ('Añadir', self.add_step, 'add_step_button'),
            ('Insertar después', self.insert_step_after, 'insert_step_button'),
            ('Duplicar seleccionada', self.duplicate_selected_step, 'duplicate_step_button'),
            ('Actualizar seleccionada', self.update_selected_step, 'update_step_button'),
            ('Eliminar seleccionada', self.remove_selected_step, 'remove_step_button'),
            ('Subir ↑', lambda: self.move_selected_step(-1), 'move_up_button'),
            ('Bajar ↓', lambda: self.move_selected_step(1), 'move_down_button'),
            ('Vaciar', self.clear_editable_steps, 'clear_steps_button'),
        ]):
            button = QPushButton(title)
            button.clicked.connect(callback)
            if attribute == 'add_step_button':
                button.setObjectName('PrimaryAction')
            elif attribute == 'clear_steps_button':
                button.setObjectName('ClearPlanAction')
            editor_buttons.addWidget(button, index // 4, index % 4)
            setattr(self, attribute, button)
        self.plan_editor = editor
        relative_layout.addWidget(editor)
        self.kind.currentIndexChanged.connect(self.configure_step_editor)
        for limit_input in (
            self.radius,
            self.maximum_depth,
            self.max_horizontal_speed,
            self.max_vertical_speed,
            self.max_yaw_speed,
        ):
            limit_input.valueChanged.connect(self.sync_step_editor_limits)
            limit_input.valueChanged.connect(self.sync_absolute_ranges)
        self.configure_step_editor()
        self.step_list = QTableWidget(0, 5)
        self.step_list.setHorizontalHeaderLabels(
            ('#', 'Movimiento', 'Objetivo', 'Límite', 'Duración')
        )
        self.step_list.setMinimumHeight(105)
        self.step_list.setAlternatingRowColors(True)
        self.step_list.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.step_list.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.step_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.step_list.verticalHeader().setVisible(False)
        header = self.step_list.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        for column in (2, 3, 4):
            header.setSectionResizeMode(column, QHeaderView.ResizeToContents)
        self.step_list.currentCellChanged.connect(
            lambda row, _column, _old_row, _old_column: self.load_selected_step(row)
        )
        relative_layout.addWidget(self.step_list, 1)
        self.update_steps()
        self.list_summary = QLabel('Lista: sin pasos')
        self.list_summary.setObjectName('MutedText')
        self.list_summary.setWordWrap(True)
        relative_layout.addWidget(self.list_summary)
        self.planner_tabs.addTab(relative_page, 'Movimientos relativos')
        self.absolute_page = self.build_absolute_planner()
        self.planner_tabs.addTab(self.absolute_page, 'Puntos X-Y-Z-ángulo')
        self.planner_tabs.currentChanged.connect(self.planner_mode_changed)
        planning_layout.addWidget(self.planner_tabs, 1)
        splitter.addWidget(planning_panel)

        operation_panel = QFrame()
        operation_panel.setObjectName('PanelCard')
        operation_layout = QVBoxLayout(operation_panel)
        operation_layout.setContentsMargins(10, 10, 10, 10)
        operation_layout.setSpacing(7)
        operation_title = QLabel('Ejecución supervisada')
        operation_title.setObjectName('SectionTitle')
        operation_layout.addWidget(operation_title)
        instructions = QLabel(
            'Preparar no arma. El armado se confirma desde el mando y GUIDED '
            'retiene la pose actual. Con GUIDED estable, fija Home antes de ejecutar.'
        )
        instructions.setObjectName('MutedText')
        instructions.setWordWrap(True)
        operation_layout.addWidget(instructions)

        self.health_labels = {}
        health = QFrame()
        health.setObjectName('TopBar')
        health_layout = QGridLayout(health)
        health_layout.setContentsMargins(6, 6, 6, 6)
        for index, key in enumerate(
            (
                'executor',
                'link',
                'mode',
                'armed',
                'output',
                'navigation',
                'dvl',
                'current',
                'alert',
            )
        ):
            label = QLabel('—')
            label.setObjectName('HealthChip')
            label.setAlignment(Qt.AlignCenter)
            set_widget_state(label, 'neutral')
            self.health_labels[key] = label
            health_layout.addWidget(label, index // 3, index % 3)
        operation_layout.addWidget(health)

        self.next_action = QLabel('Esperando estado del ejecutor')
        self.next_action.setObjectName('NextAction')
        self.next_action.setWordWrap(True)
        set_widget_state(self.next_action, 'warning')
        operation_layout.addWidget(self.next_action)
        self.blocker_text = QLabel('• No hay un estado reciente del ejecutor.')
        self.blocker_text.setObjectName('BlockerText')
        self.blocker_text.setWordWrap(True)
        operation_layout.addWidget(self.blocker_text)

        actions = QGroupBox('Acciones disponibles')
        action_layout = QVBoxLayout(actions)
        prepare_button = self.button('2 · Preparar y validar plan', 'prepare')
        prepare_button.setObjectName('PrimaryAction')
        guided_button = self.button('4 · Solicitar GUIDED seguro', 'guided')
        guided_button.setObjectName('GuidedAction')
        home_button = self.button('5 · Fijar Home con la pose retenida por GUIDED', 'set_home')
        home_button.setObjectName('PrimaryAction')
        execute_button = self.button('6 · Iniciar plan preparado', 'execute')
        execute_button.setObjectName('SuccessAction')
        action_layout.addWidget(prepare_button)
        arm_note = QLabel('3 · ARMAR se confirma desde el mando; esta interfaz no arma.')
        arm_note.setObjectName('MutedText')
        arm_note.setWordWrap(True)
        action_layout.addWidget(arm_note)
        action_layout.addWidget(guided_button)
        action_layout.addWidget(home_button)
        action_layout.addWidget(execute_button)
        operation_layout.addWidget(actions)

        self.detail_toggle = QToolButton()
        self.detail_toggle.setObjectName('TechnicalToggle')
        self.detail_toggle.setText('▸ Mostrar estado técnico')
        self.detail_toggle.setCheckable(True)
        self.detail_toggle.toggled.connect(self.toggle_technical_details)
        operation_layout.addWidget(self.detail_toggle)
        self.exec_text = QTextEdit()
        self.exec_text.setReadOnly(True)
        self.exec_text.setMinimumHeight(120)
        self.exec_text.setVisible(False)
        operation_layout.addWidget(self.exec_text, 1)
        splitter.addWidget(operation_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([820, 520])

        page_scroll = QScrollArea()
        page_scroll.setWidgetResizable(True)
        page_scroll.setFrameShape(QFrame.NoFrame)
        page_scroll.setWidget(page)
        configuration_scroll = QScrollArea()
        configuration_scroll.setWidgetResizable(True)
        configuration_scroll.setFrameShape(QFrame.NoFrame)
        configuration_scroll.setWidget(configuration_page)
        self.tabs.addTab(self.build_accessory_panel(), 'Mando y accesorios')
        self.tabs.addTab(configuration_scroll, 'Configuración y seguridad')
        self.tabs.addTab(page_scroll, 'Secuencias relativas y absolutas')
        self.tabs.setCurrentWidget(configuration_scroll)
        # Keep inherited widgets alive for its timers, but expose only the
        # operator pages used during a real launch.
        for index in range(self.tabs.count() - 1, -1, -1):
            if self.tabs.tabText(index) == 'Preflight · piscina':
                self.tabs.removeTab(index)
            elif self.tabs.tabText(index) == 'Secuencias · previsualización':
                self.tabs.removeTab(index)
        self.stop_button = self.button('ABORTAR MOVIMIENTO → MANUAL · NO DESARMA', 'manual')
        self.stop_button.setObjectName('DangerAction')
        self.layout().addWidget(self.stop_button)
        disarm_note = QLabel(
            'DESARMAR: botón B del mando. Si no responde, killswitch físico. '
            'No dependas de esta ventana.'
        )
        disarm_note.setObjectName('SafetyNote')
        disarm_note.setAlignment(Qt.AlignCenter)
        self.layout().addWidget(disarm_note)
        self.initialize_missions()
        self.initialize_configuration()
        self.timer.timeout.connect(self.tick_sequence)

    def button(self, title, action):
        button = QPushButton(title)
        button.clicked.connect(lambda: self.send_action(action))
        self.buttons[action] = button
        return button

    def toggle_technical_details(self, visible):
        """Expand raw diagnostics only when the operator requests them."""
        self.exec_text.setVisible(visible)
        self.detail_toggle.setText(
            '▾ Ocultar estado técnico' if visible else '▸ Mostrar estado técnico'
        )

    @staticmethod
    def field_label(text, tooltip=''):
        """Create a compact, non-input-looking form label."""
        label = QLabel(text)
        label.setObjectName('FieldLabel')
        if tooltip:
            label.setToolTip(tooltip)
        return label

    def build_accessory_panel(self):
        """Build one transparent control/diagnostic page for Xbox accessories."""
        page = QWidget()
        page.setObjectName('RobotXRoot')
        layout = QVBoxLayout(page)
        title = QLabel('Mando Xbox · cámara y luces')
        title.setObjectName('SectionTitle')
        layout.addWidget(title)
        mapping = QLabel(
            'D-pad ↑/↓: cámara · D-pad ←/→: luces −/+ 10 % · '
            'LB + D-pad ↑/↓: cambiar modo. Para accesorios, suelta LB y RB y '
            'deja sticks/gatillos en neutro.'
        )
        mapping.setWordWrap(True)
        mapping.setObjectName('MutedText')
        layout.addWidget(mapping)
        self.accessory_gate = QLabel('Esperando estado del puente')
        self.accessory_gate.setObjectName('StatusBanner')
        self.accessory_gate.setWordWrap(True)
        set_widget_state(self.accessory_gate, 'neutral')
        layout.addWidget(self.accessory_gate)

        camera_group = QGroupBox('Inclinación de cámara')
        camera_layout = QHBoxLayout(camera_group)
        self.camera_up_button = QPushButton('Mantener · cámara ↑')
        self.camera_down_button = QPushButton('Mantener · cámara ↓')
        self.camera_up_button.pressed.connect(lambda: self.set_camera_tilt(1.0))
        self.camera_up_button.released.connect(lambda: self.set_camera_tilt(0.0))
        self.camera_down_button.pressed.connect(lambda: self.set_camera_tilt(-1.0))
        self.camera_down_button.released.connect(lambda: self.set_camera_tilt(0.0))
        camera_layout.addWidget(self.camera_up_button)
        camera_layout.addWidget(self.camera_down_button)
        layout.addWidget(camera_group)

        lights_group = QGroupBox('Intensidad de luces')
        lights_layout = QGridLayout(lights_group)
        self.lights_slider = QSlider(Qt.Horizontal)
        self.lights_slider.setRange(0, 100)
        self.lights_slider.setSingleStep(10)
        self.lights_slider.setPageStep(10)
        self.lights_slider.setTickInterval(10)
        self.lights_slider.setTickPosition(QSlider.TicksBelow)
        self.lights_value = QLabel('0 % solicitado')
        self.lights_slider.valueChanged.connect(
            lambda value: self.lights_value.setText(f'{value} % solicitado')
        )
        lights_layout.addWidget(self.lights_slider, 0, 0, 1, 3)
        lights_layout.addWidget(self.lights_value, 0, 3)
        self.lights_less_button = QPushButton('− 10 %')
        self.lights_more_button = QPushButton('+ 10 %')
        self.lights_apply_button = QPushButton('Aplicar intensidad')
        self.lights_apply_button.setObjectName('PrimaryAction')
        self.lights_off_button = QPushButton('Apagar y sincronizar 0 %')
        self.lights_less_button.clicked.connect(lambda: self.change_lights(-1))
        self.lights_more_button.clicked.connect(lambda: self.change_lights(1))
        self.lights_apply_button.clicked.connect(self.apply_lights_percent)
        self.lights_off_button.clicked.connect(
            lambda: self.lights_off_pub.publish(Empty())
        )
        lights_layout.addWidget(self.lights_less_button, 1, 0)
        lights_layout.addWidget(self.lights_more_button, 1, 1)
        lights_layout.addWidget(self.lights_apply_button, 1, 2)
        lights_layout.addWidget(self.lights_off_button, 1, 3)
        note = QLabel(
            'ArduSub no devuelve la intensidad física. “Aplicar” primero envía '
            'suficientes pulsos para llegar a 0 % y luego sube al valor solicitado; '
            'por eso el porcentaje mostrado es una estimación sincronizada por software.'
        )
        note.setObjectName('SafetyNote')
        note.setWordWrap(True)
        lights_layout.addWidget(note, 2, 0, 1, 4)
        layout.addWidget(lights_group)
        self.accessory_detail = QLabel('Sin actividad de accesorios')
        self.accessory_detail.setObjectName('TelemetryValue')
        self.accessory_detail.setWordWrap(True)
        layout.addWidget(self.accessory_detail)
        layout.addStretch()
        self.accessory_buttons = (
            self.camera_up_button,
            self.camera_down_button,
            self.lights_less_button,
            self.lights_more_button,
            self.lights_apply_button,
            self.lights_off_button,
            self.lights_slider,
        )
        for control in self.accessory_buttons:
            control.setEnabled(False)
        return page

    def set_camera_tilt(self, command):
        """Set the held UI command; the main timer supplies the watchdog stream."""
        self.camera_tilt_command = float(command)
        message = Float32()
        message.data = self.camera_tilt_command
        self.camera_tilt_pub.publish(message)

    def change_lights(self, direction):
        message = Int8()
        message.data = int(direction)
        self.lights_step_pub.publish(message)

    def apply_lights_percent(self):
        message = Float32()
        message.data = float(self.lights_slider.value())
        self.lights_setpoint_pub.publish(message)

    def receive_accessory_status(self, message):
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                self.accessory_status_data = payload
                self.accessory_status_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def receive_xbox_status(self, message):
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                self.xbox_status_data = payload
                self.xbox_status_at = time.monotonic()
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def update_accessory_panel(self):
        fresh = time.monotonic() - self.accessory_status_at < 0.75
        status = self.accessory_status_data if fresh else {}
        ready = status.get('ready') is True
        for control in self.accessory_buttons:
            control.setEnabled(ready)
        if not fresh:
            text, state = 'Sin estado reciente del puente de accesorios', 'warning'
        elif ready:
            text, state = 'ACCESORIOS HABILITADOS · cámara y luces disponibles', 'success'
        else:
            text = 'ACCESORIOS BLOQUEADOS · ' + (status.get('blocker') or 'causa desconocida')
            state = 'info' if not status.get('enabled') else 'warning'
        self.accessory_gate.setText(text)
        set_widget_state(self.accessory_gate, state)
        if status:
            synchronized = 'sí' if status.get('lights_synchronized') else 'no'
            xbox_fresh = time.monotonic() - self.xbox_status_at < 0.75
            xbox = self.xbox_status_data if xbox_fresh else {}
            if not xbox_fresh:
                xbox_text = 'mando sin mensajes recientes'
            elif xbox.get('accessory_allowed'):
                xbox_text = 'gesto de accesorios permitido'
            else:
                xbox_text = 'gesto bloqueado: ' + (xbox.get('blocker') or 'entrada inválida')
            self.accessory_detail.setText(
                f'Última entrada: {status.get("last_action", "—")} · '
                f'luces estimadas {status.get("lights_percent_estimate", 0):.1f}% · '
                f'sincronización: {synchronized} · '
                f'cola pendiente: {status.get("pending_light_edges", 0)} · {xbox_text}'
            )
        if ready and abs(self.camera_tilt_command) > 1e-6:
            message = Float32()
            message.data = self.camera_tilt_command
            self.camera_tilt_pub.publish(message)

    def build_absolute_planner(self):
        """Build the fixed mission-frame waypoint editor."""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        explanation = QLabel(
            'Con GUIDED estable, pulsa 5 · Fijar Home: la pose y el rumbo retenidos '
            'al entrar en GUIDED serán X=0, Y=0 y ángulo 0°. La profundidad permanece '
            'absoluta respecto a la superficie calibrada. Cada punto ejecuta: '
            'profundidad → X-Y → ángulo → verificación.'
        )
        explanation.setObjectName('MutedText')
        explanation.setWordWrap(True)
        layout.addWidget(explanation)

        editor = QGroupBox('Editor de punto absoluto local')
        grid = QGridLayout(editor)
        specs = (
            ('X desde origen [m]', 'absolute_x', -5.0, 5.0, 3, 0.05, 0.0),
            ('Y desde origen [m]', 'absolute_y', -5.0, 5.0, 3, 0.05, 0.0),
            ('Profundidad [m]', 'absolute_depth', 0.0, 3.0, 3, 0.02, 0.20),
            ('Ángulo absoluto [°]', 'absolute_yaw', -180.0, 180.0, 1, 5.0, 0.0),
            ('Velocidad X-Y [m/s]', 'absolute_speed', 0.001, 2.0, 3, 0.01, 0.15),
            ('Tolerancia X [m]', 'absolute_tolerance_x', 0.01, 0.15, 3, 0.005, 0.04),
            ('Tolerancia Y [m]', 'absolute_tolerance_y', 0.01, 0.15, 3, 0.005, 0.04),
            (
                'Tolerancia profundidad [m]',
                'absolute_tolerance_depth',
                0.005,
                0.10,
                3,
                0.005,
                0.02,
            ),
            ('Tolerancia angular [°]', 'absolute_tolerance_yaw', 1.0, 20.0, 1, 1.0, 5.0),
            ('Confirmación estable [s]', 'absolute_dwell', 0.5, 5.0, 1, 0.5, 1.0),
        )
        self.absolute_inputs = []
        for index, (label, attribute, minimum, maximum, decimals, step, value) in enumerate(
            specs
        ):
            entry = QDoubleSpinBox()
            entry.setRange(minimum, maximum)
            entry.setDecimals(decimals)
            entry.setSingleStep(step)
            entry.setValue(value)
            setattr(self, attribute, entry)
            self.absolute_inputs.append(entry)
            row, column = divmod(index, 2)
            grid.addWidget(self.field_label(label), row, 2 * column)
            grid.addWidget(entry, row, 2 * column + 1)
        buttons = QGridLayout()
        for index, (title, callback, attribute) in enumerate((
            ('Añadir punto', self.add_absolute_waypoint, 'absolute_add_button'),
            (
                'Insertar después',
                self.insert_absolute_waypoint_after,
                'absolute_insert_button',
            ),
            (
                'Duplicar punto',
                self.duplicate_absolute_waypoint,
                'absolute_duplicate_button',
            ),
            ('Actualizar', self.update_absolute_waypoint, 'absolute_update_button'),
            ('Eliminar', self.remove_absolute_waypoint, 'absolute_remove_button'),
            ('Subir ↑', lambda: self.move_absolute_waypoint(-1), 'absolute_up_button'),
            ('Bajar ↓', lambda: self.move_absolute_waypoint(1), 'absolute_down_button'),
            (
                'Aplicar tolerancias a todos',
                self.apply_absolute_tolerances_to_all,
                'absolute_tolerances_button',
            ),
            (
                'Crear rectángulo…',
                self.create_rectangle_mission,
                'absolute_rectangle_button',
            ),
            ('Vaciar', self.clear_absolute_waypoints, 'absolute_clear_button'),
        )):
            button = QPushButton(title)
            button.clicked.connect(callback)
            if attribute == 'absolute_add_button':
                button.setObjectName('PrimaryAction')
            elif attribute == 'absolute_clear_button':
                button.setObjectName('ClearPlanAction')
            buttons.addWidget(button, index // 3, index % 3)
            setattr(self, attribute, button)
        grid.addLayout(buttons, 5, 0, 1, 4)
        self.absolute_plan_editor = editor
        layout.addWidget(editor)

        self.absolute_list = QTableWidget(0, 8)
        self.absolute_list.setHorizontalHeaderLabels(
            ('#', 'X', 'Y', 'Prof.', 'Ángulo', 'Vel. X-Y', 'Tol. X/Y/Z/°', 'Estable')
        )
        self.absolute_list.setAlternatingRowColors(True)
        self.absolute_list.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.absolute_list.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.absolute_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.absolute_list.verticalHeader().setVisible(False)
        header = self.absolute_list.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        for column in range(1, 8):
            header.setSectionResizeMode(column, QHeaderView.Stretch)
        self.absolute_list.currentCellChanged.connect(
            lambda row, _column, _old_row, _old_column: self.load_absolute_waypoint(row)
        )
        layout.addWidget(self.absolute_list, 1)
        self.absolute_summary = QLabel('Plan absoluto: sin puntos')
        self.absolute_summary.setObjectName('MutedText')
        self.absolute_summary.setWordWrap(True)
        layout.addWidget(self.absolute_summary)
        self.sync_absolute_ranges()
        self.update_absolute_waypoints()
        return page

    def sync_absolute_ranges(self, *_):
        """Keep absolute editor limits aligned with the session envelope."""
        if not hasattr(self, 'absolute_x'):
            return
        radius = self.radius.value()
        self.absolute_x.setRange(-radius, radius)
        self.absolute_y.setRange(-radius, radius)
        self.absolute_depth.setRange(0.0, self.maximum_depth.value())
        self.absolute_speed.setRange(0.001, self.max_horizontal_speed.value())

    def planner_mode_changed(self, index):
        """Select exactly one source of commands and invalidate stale preparation."""
        self.active_plan_mode = 'absolute' if index == 1 else 'relative'
        self.invalidate_preparation()
        self.autosave_mission_draft()

    def active_steps(self):
        return self.absolute_steps if self.active_plan_mode == 'absolute' else self.relative_steps

    def absolute_waypoint_from_editor(self):
        x, y = self.absolute_x.value(), self.absolute_y.value()
        if math.hypot(x, y) > self.radius.value() + 1e-9:
            QMessageBox.warning(
                self,
                'Punto fuera del radio',
                'La distancia radial del punto supera el radio configurado de la sesión.',
            )
            return None
        return {
            'kind': 'absolute_waypoint',
            'x': x,
            'y': y,
            'depth': self.absolute_depth.value(),
            'yaw_deg': self.absolute_yaw.value(),
            'speed': self.absolute_speed.value(),
            'vertical_speed': min(0.08, self.max_vertical_speed.value()),
            'yaw_speed_deg': min(10.0, self.max_yaw_speed.value()),
            'tolerance_x': self.absolute_tolerance_x.value(),
            'tolerance_y': self.absolute_tolerance_y.value(),
            'tolerance_depth': self.absolute_tolerance_depth.value(),
            'tolerance_yaw_deg': self.absolute_tolerance_yaw.value(),
            'dwell': self.absolute_dwell.value(),
        }

    def add_absolute_waypoint(self):
        waypoint = self.absolute_waypoint_from_editor()
        if waypoint is None:
            return
        self.absolute_steps.append(waypoint)
        self.update_absolute_waypoints(len(self.absolute_steps) - 1)
        self.autosave_mission_draft()

    def insert_absolute_waypoint_after(self):
        row = self.absolute_list.currentRow()
        waypoint = self.absolute_waypoint_from_editor()
        if waypoint is None:
            return
        index = row + 1 if row >= 0 else len(self.absolute_steps)
        self.absolute_steps.insert(index, waypoint)
        self.update_absolute_waypoints(index)
        self.autosave_mission_draft()

    def duplicate_absolute_waypoint(self):
        row = self.absolute_list.currentRow()
        if row < 0:
            return
        self.absolute_steps, selected = duplicate_step(self.absolute_steps, row)
        self.update_absolute_waypoints(selected)
        self.autosave_mission_draft()

    def apply_absolute_tolerances_to_all(self):
        if not self.absolute_steps:
            return
        values = {
            'tolerance_x': self.absolute_tolerance_x.value(),
            'tolerance_y': self.absolute_tolerance_y.value(),
            'tolerance_depth': self.absolute_tolerance_depth.value(),
            'tolerance_yaw_deg': self.absolute_tolerance_yaw.value(),
            'dwell': self.absolute_dwell.value(),
        }
        answer = QMessageBox.question(
            self,
            'Aplicar tolerancias',
            '¿Aplicar las tolerancias y permanencia actuales a todos los puntos?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self.absolute_steps = [dict(point, **values) for point in self.absolute_steps]
        self.update_absolute_waypoints(self.absolute_list.currentRow())
        self.autosave_mission_draft()

    def create_rectangle_mission(self):
        """Replace the absolute editor with a closed rectangle from mission origin."""
        point = self.absolute_waypoint_from_editor()
        if point is None:
            return
        radius = self.radius.value()
        width, accepted = QInputDialog.getDouble(
            self,
            'Rectángulo local',
            'Lado X [m] (el signo elige la dirección):',
            min(0.50, radius),
            -radius,
            radius,
            2,
        )
        if not accepted:
            return
        height, accepted = QInputDialog.getDouble(
            self,
            'Rectángulo local',
            'Lado Y [m] (el signo elige la dirección):',
            min(0.50, radius),
            -radius,
            radius,
            2,
        )
        if not accepted:
            return
        if math.hypot(width, height) > radius + 1e-9:
            QMessageBox.warning(
                self,
                'Rectángulo fuera del radio',
                'La esquina diagonal supera el radio configurado de la sesión.',
            )
            return
        try:
            points = rectangle_waypoints(point, width, height)
        except ValueError as exc:
            QMessageBox.warning(self, 'Rectángulo inválido', str(exc))
            return
        if self.absolute_steps:
            answer = QMessageBox.question(
                self,
                'Reemplazar puntos absolutos',
                'La plantilla reemplazará los puntos absolutos actuales. ¿Continuar?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.absolute_steps = points
        self.update_absolute_waypoints(0)
        self.autosave_mission_draft()

    def update_absolute_waypoint(self):
        row = self.absolute_list.currentRow()
        if row < 0:
            return
        waypoint = self.absolute_waypoint_from_editor()
        if waypoint is None:
            return
        self.absolute_steps = replace_step(self.absolute_steps, row, waypoint)
        self.update_absolute_waypoints(row)
        self.autosave_mission_draft()

    def remove_absolute_waypoint(self):
        row = self.absolute_list.currentRow()
        if row < 0:
            return
        del self.absolute_steps[row]
        self.update_absolute_waypoints(min(row, len(self.absolute_steps) - 1))
        self.autosave_mission_draft()

    def move_absolute_waypoint(self, offset):
        row = self.absolute_list.currentRow()
        if row < 0:
            return
        self.absolute_steps, selected = move_step(self.absolute_steps, row, offset)
        self.update_absolute_waypoints(selected)
        self.autosave_mission_draft()

    def clear_absolute_waypoints(self):
        if self.absolute_steps:
            answer = QMessageBox.question(
                self,
                'Vaciar puntos absolutos',
                '¿Eliminar todos los puntos absolutos del plan actual?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.absolute_steps.clear()
        self.update_absolute_waypoints()
        self.autosave_mission_draft()

    def load_absolute_waypoint(self, row):
        selected = 0 <= row < len(self.absolute_steps)
        self.absolute_update_button.setEnabled(selected)
        self.absolute_insert_button.setEnabled(True)
        self.absolute_duplicate_button.setEnabled(selected)
        self.absolute_remove_button.setEnabled(selected)
        self.absolute_up_button.setEnabled(selected and row > 0)
        self.absolute_down_button.setEnabled(selected and row < len(self.absolute_steps) - 1)
        if not selected:
            return
        point = self.absolute_steps[row]
        self.absolute_x.setValue(point['x'])
        self.absolute_y.setValue(point['y'])
        self.absolute_depth.setValue(point['depth'])
        self.absolute_yaw.setValue(point['yaw_deg'])
        self.absolute_speed.setValue(point['speed'])
        self.absolute_tolerance_x.setValue(point['tolerance_x'])
        self.absolute_tolerance_y.setValue(point['tolerance_y'])
        self.absolute_tolerance_depth.setValue(point['tolerance_depth'])
        self.absolute_tolerance_yaw.setValue(point['tolerance_yaw_deg'])
        self.absolute_dwell.setValue(point['dwell'])

    def update_absolute_waypoints(self, selected_row=None):
        if not hasattr(self, 'absolute_list'):
            return
        self.absolute_list.blockSignals(True)
        self.absolute_list.setRowCount(len(self.absolute_steps))
        for row, point in enumerate(self.absolute_steps):
            cells = (
                str(row + 1),
                f'{point["x"]:.3f}',
                f'{point["y"]:.3f}',
                f'{point["depth"]:.3f}',
                f'{point["yaw_deg"]:.1f}°',
                f'{point["speed"]:.3f}',
                (
                    f'{point["tolerance_x"]:.3f}/'
                    f'{point["tolerance_y"]:.3f}/'
                    f'{point["tolerance_depth"]:.3f}/'
                    f'{point["tolerance_yaw_deg"]:.1f}°'
                ),
                f'{point["dwell"]:.1f} s',
            )
            for column, text in enumerate(cells):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                self.absolute_list.setItem(row, column, item)
        self.absolute_list.blockSignals(False)
        if selected_row is not None and 0 <= selected_row < len(self.absolute_steps):
            self.absolute_list.setCurrentCell(selected_row, 0)
        else:
            self.absolute_list.clearSelection()
            self.load_absolute_waypoint(-1)
        self.absolute_summary.setText(
            'Plan absoluto: '
            + (
                ' → '.join(self.format_step(point) for point in self.absolute_steps)
                if self.absolute_steps
                else 'sin puntos'
            )
        )
        self.invalidate_preparation()

    def current_mission(self):
        """Return both editor lists so switching mode never discards work."""
        return {
            'plan_mode': self.active_plan_mode,
            'relative_steps': [dict(step) for step in self.relative_steps],
            'absolute_steps': [dict(step) for step in self.absolute_steps],
        }

    def initialize_missions(self):
        """Restore the automatic draft and populate the named-mission selector."""
        draft = self.mission_document.get('draft', {})
        self.relative_steps = [dict(step) for step in draft.get('relative_steps', [])]
        self.steps = self.relative_steps
        self.absolute_steps = [dict(step) for step in draft.get('absolute_steps', [])]
        mode = draft.get('plan_mode', 'relative')
        self.planner_tabs.setCurrentIndex(1 if mode == 'absolute' else 0)
        self.active_plan_mode = mode
        self.refresh_mission_combo(self.loaded_mission_name)
        self.update_steps()
        self.update_absolute_waypoints()
        if self.mission_error:
            self.mission_status.setText(
                'No se pudo leer el archivo anterior; se abrió un borrador vacío. '
                + self.mission_error
            )
            set_widget_state(self.mission_status, 'warning')
        elif self.relative_steps or self.absolute_steps:
            self.mission_status.setText('Borrador automático restaurado')
        self.mission_loading = False

    def refresh_mission_combo(self, selected=None):
        self.mission_combo.blockSignals(True)
        self.mission_combo.clear()
        self.mission_combo.addItems(sorted(self.mission_document['missions']))
        if selected:
            index = self.mission_combo.findText(selected)
            if index >= 0:
                self.mission_combo.setCurrentIndex(index)
        self.mission_combo.blockSignals(False)
        available = self.mission_combo.count() > 0
        self.load_mission_button.setEnabled(available)
        self.rename_mission_button.setEnabled(available)
        self.delete_mission_button.setEnabled(available)

    def autosave_mission_draft(self):
        """Persist edits atomically without replacing a named mission."""
        if (
            not hasattr(self, 'mission_document')
            or not hasattr(self, 'mission_status')
            or self.mission_loading
        ):
            return
        try:
            document = dict(self.mission_document)
            document['draft'] = self.current_mission()
            self.mission_document = self.mission_store.save(document)
            self.mission_dirty = True
            name = self.loaded_mission_name or 'sin nombre'
            self.mission_status.setText(
                f'Borrador guardado automáticamente · misión cargada: {name} · '
                'cambios aún no guardados con nombre'
            )
            set_widget_state(self.mission_status, 'info')
        except (OSError, ValueError) as exc:
            self.mission_status.setText('No se pudo guardar el borrador: ' + str(exc))
            set_widget_state(self.mission_status, 'danger')

    def load_selected_mission(self):
        name = self.mission_combo.currentText().strip()
        if not name:
            return
        if self.mission_dirty:
            answer = QMessageBox.question(
                self,
                'Cargar otra misión',
                'El borrador actual tiene cambios no guardados con nombre. ¿Cargar igualmente?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        mission = self.mission_document['missions'][name]
        self.mission_loading = True
        try:
            self.relative_steps = [dict(step) for step in mission['relative_steps']]
            self.steps = self.relative_steps
            self.absolute_steps = [dict(step) for step in mission['absolute_steps']]
            self.active_plan_mode = mission['plan_mode']
            self.planner_tabs.setCurrentIndex(1 if self.active_plan_mode == 'absolute' else 0)
            self.update_steps()
            self.update_absolute_waypoints()
        finally:
            self.mission_loading = False
        self.loaded_mission_name = name
        self.mission_dirty = False
        self.mission_document['active_mission'] = name
        self.mission_document['draft'] = self.current_mission()
        self.mission_document = self.mission_store.save(self.mission_document)
        self.mission_status.setText(f'Misión cargada: {name}')
        set_widget_state(self.mission_status, 'success')

    def save_current_mission(self):
        if not self.loaded_mission_name:
            self.save_mission_as()
            return
        self._save_mission(self.loaded_mission_name)

    def save_mission_as(self):
        name, accepted = QInputDialog.getText(self, 'Guardar misión', 'Nombre de la misión:')
        if accepted and name.strip():
            self._save_mission(name)

    def _save_mission(self, name):
        try:
            self.mission_document = save_mission(
                self.mission_document, name, self.current_mission()
            )
            self.mission_document = self.mission_store.save(self.mission_document)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'No se pudo guardar', str(exc))
            return
        self.loaded_mission_name = self.mission_document['active_mission']
        self.mission_dirty = False
        self.refresh_mission_combo(self.loaded_mission_name)
        self.mission_status.setText(f'Misión guardada: {self.loaded_mission_name}')
        set_widget_state(self.mission_status, 'success')

    def rename_current_mission(self):
        current = self.mission_combo.currentText().strip()
        if not current:
            return
        name, accepted = QInputDialog.getText(
            self, 'Renombrar misión', 'Nuevo nombre:', text=current
        )
        if not accepted:
            return
        try:
            self.mission_document = rename_mission(self.mission_document, current, name)
            self.mission_document = self.mission_store.save(self.mission_document)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'No se pudo renombrar', str(exc))
            return
        self.loaded_mission_name = self.mission_document['active_mission']
        self.refresh_mission_combo(self.loaded_mission_name)

    def delete_current_mission(self):
        name = self.mission_combo.currentText().strip()
        if not name:
            return
        answer = QMessageBox.question(
            self,
            'Eliminar misión',
            f'¿Eliminar la misión guardada “{name}”? El borrador actual se conserva.',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        try:
            self.mission_document = delete_mission(self.mission_document, name)
            self.mission_document = self.mission_store.save(self.mission_document)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'No se pudo eliminar', str(exc))
            return
        if self.loaded_mission_name == name:
            self.loaded_mission_name = None
        self.refresh_mission_combo(self.mission_document.get('active_mission'))
        self.mission_status.setText('Misión eliminada; el borrador actual se conserva')

    def new_mission(self):
        answer = QMessageBox.question(
            self,
            'Nueva misión',
            '¿Vaciar ambos planes y comenzar una misión nueva?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self.relative_steps = []
        self.steps = self.relative_steps
        self.absolute_steps = []
        self.loaded_mission_name = None
        self.mission_dirty = True
        self.update_steps()
        self.update_absolute_waypoints()
        self.autosave_mission_draft()

    def initialize_configuration(self):
        """Populate profile controls and load the last explicitly saved values."""
        active = self.configuration_document['active_profile']
        self.refresh_profile_combo(active)
        self._load_profile(active, persist_selection=False)
        if self.configuration_error:
            self.configuration_status.setText(
                'No se pudo leer el archivo anterior; se muestran valores '
                f'conservadores. Detalle: {self.configuration_error}'
            )
            set_widget_state(self.configuration_status, 'warning')
        elif self.configuration_store.last_notice:
            self.configuration_status.setText(self.configuration_store.last_notice)
            set_widget_state(self.configuration_status, 'warning')

    def refresh_profile_combo(self, selected):
        """Refresh profile names without emitting an accidental load."""
        self.profile_combo.blockSignals(True)
        self.profile_combo.clear()
        self.profile_combo.addItems(sorted(self.configuration_document['profiles']))
        index = self.profile_combo.findText(selected)
        self.profile_combo.setCurrentIndex(max(0, index))
        self.profile_combo.blockSignals(False)

    def current_profile(self):
        """Return all values displayed in the configuration page."""
        return normalize_profile(
            {
                'session': {
                    'sensor_depth_m': self.sensor_depth.value(),
                    'maximum_depth_m': self.maximum_depth.value(),
                    'radius_m': self.radius.value(),
                    'max_horizontal_speed_mps': self.max_horizontal_speed.value(),
                    'max_vertical_speed_mps': self.max_vertical_speed.value(),
                    'max_yaw_speed_deg_s': self.max_yaw_speed.value(),
                },
                'controller': {
                    'xy_tolerance_m': self.xy_tolerance.value(),
                    'recovery_radius_m': self.recovery_radius.value(),
                    'recovery_speed_mps': self.recovery_speed.value(),
                    'recovery_min_speed_mps': self.recovery_min_speed.value(),
                    'recovery_delay_s': self.recovery_delay.value(),
                    'position_gain_hz': self.position_gain.value(),
                    'velocity_damping_s': self.velocity_damping.value(),
                },
                'vehicle': {
                    'propulsion_signal_limit_pct': (
                        self.propulsion_signal_limit.value()
                    ),
                    'max_ascent_speed_mps': self.max_ascent_speed.value(),
                    'max_descent_speed_mps': self.max_descent_speed.value(),
                    'max_vertical_accel_mps2': self.max_vertical_accel.value(),
                },
                'display': {
                    'sonar_source': self.sonar.source.currentData(),
                    'sonar_visible_range_m': self.sonar.range_m.value(),
                    'sonar_persistence_s': self.sonar.persistence.value(),
                    'sonar_visual_threshold': self.sonar.min_intensity.value(),
                },
            }
        )

    def display_profile(self, profile):
        """Fill every configuration field from one validated profile."""
        profile = normalize_profile(profile)
        session = profile['session']
        controller = profile['controller']
        vehicle = profile['vehicle']
        display = profile['display']
        self.configuration_loading = True
        try:
            self.sensor_depth.setValue(session['sensor_depth_m'])
            self.maximum_depth.setValue(session['maximum_depth_m'])
            self.radius.setValue(session['radius_m'])
            self.max_horizontal_speed.setValue(session['max_horizontal_speed_mps'])
            self.max_vertical_speed.setValue(session['max_vertical_speed_mps'])
            self.max_yaw_speed.setValue(session['max_yaw_speed_deg_s'])
            self.xy_tolerance.setValue(controller['xy_tolerance_m'])
            self.sync_controller_ranges()
            self.recovery_radius.setValue(controller['recovery_radius_m'])
            self.recovery_speed.setValue(controller['recovery_speed_mps'])
            self.recovery_min_speed.setValue(controller['recovery_min_speed_mps'])
            self.recovery_delay.setValue(controller['recovery_delay_s'])
            self.position_gain.setValue(controller['position_gain_hz'])
            self.velocity_damping.setValue(controller['velocity_damping_s'])
            self.propulsion_signal_limit.setValue(
                vehicle['propulsion_signal_limit_pct']
            )
            self.max_ascent_speed.setValue(vehicle['max_ascent_speed_mps'])
            self.max_descent_speed.setValue(vehicle['max_descent_speed_mps'])
            self.max_vertical_accel.setValue(vehicle['max_vertical_accel_mps2'])
            source_index = self.sonar.source.findData(display['sonar_source'])
            if source_index >= 0:
                self.sonar.source.setCurrentIndex(source_index)
            self.sonar.range_m.setValue(display['sonar_visible_range_m'])
            self.sonar.persistence.setValue(display['sonar_persistence_s'])
            self.sonar.min_intensity.setValue(display['sonar_visual_threshold'])
        finally:
            self.configuration_loading = False
        self.sync_controller_ranges()
        self.configure_step_editor()
        self.invalidate_preparation()
        self.update_parameter_preview()
        self.configuration_dirty = False

    def load_selected_profile(self, _checked=False, persist_selection=True):
        """Restore the selected saved profile into the form."""
        name = self.profile_combo.currentText().strip()
        if self.configuration_dirty:
            answer = QMessageBox.question(
                self,
                'Descartar cambios',
                f'El perfil {self.loaded_profile_name or name} tiene cambios sin guardar. '
                '¿Descartarlos y volver a cargar?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                self.refresh_profile_combo(self.loaded_profile_name or name)
                return
        self._load_profile(name, persist_selection=persist_selection)

    def _load_profile(self, name, persist_selection=True):
        """Load one named profile after the caller has resolved unsaved edits."""
        profile = self.configuration_document['profiles'].get(name)
        if profile is None:
            return
        self.display_profile(profile)
        self.configuration_document['active_profile'] = name
        if persist_selection:
            try:
                self.configuration_document = self.configuration_store.save(
                    self.configuration_document
                )
            except OSError as exc:
                self.configuration_status.setText(f'No se pudo guardar la selección: {exc}')
                set_widget_state(self.configuration_status, 'danger')
                return
        self.loaded_profile_name = name
        self.refresh_profile_combo(name)
        self.configuration_status.setText(f'Perfil cargado: {name}')
        set_widget_state(self.configuration_status, 'success')

    def profile_selection_changed(self, name):
        """Auto-load a selected profile and never save another profile's form."""
        name = name.strip()
        if self.configuration_loading or not name or name == self.loaded_profile_name:
            return
        if self.configuration_dirty:
            answer = QMessageBox.question(
                self,
                'Cambiar de perfil',
                f'Hay cambios sin guardar en {self.loaded_profile_name}. '
                f'¿Descartarlos y cargar {name}?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                self.refresh_profile_combo(self.loaded_profile_name)
                return
        self._load_profile(name)

    def save_current_profile(self):
        """Overwrite the selected local profile after full validation."""
        name = self.loaded_profile_name or self.profile_combo.currentText().strip()
        self._save_profile(name)

    def save_profile_as(self):
        """Prompt for a new local profile name and save the displayed values."""
        suggested = (
            self.loaded_profile_name or self.profile_combo.currentText().strip()
        ) + ' copia'
        name, accepted = QInputDialog.getText(
            self,
            'Guardar perfil',
            'Nombre del perfil:',
            text=suggested,
        )
        if accepted:
            self._save_profile(name)

    def _save_profile(self, name):
        name = ' '.join(str(name).strip().split())
        if (
            name
            and name in self.configuration_document['profiles']
            and name != self.loaded_profile_name
        ):
            answer = QMessageBox.question(
                self,
                'Sobrescribir perfil',
                f'Ya existe el perfil {name}. ¿Sobrescribirlo?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        try:
            document = update_profile(
                self.configuration_document,
                name,
                self.current_profile(),
            )
            self.configuration_document = self.configuration_store.save(document)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'No se pudo guardar', str(exc))
            self.configuration_status.setText(f'Configuración no guardada: {exc}')
            set_widget_state(self.configuration_status, 'danger')
            return
        selected = self.configuration_document['active_profile']
        self.loaded_profile_name = selected
        self.refresh_profile_combo(selected)
        self.configuration_dirty = False
        self.configuration_status.setText(
            f'Configuración guardada · se cargará al abrir: {selected}'
        )
        set_widget_state(self.configuration_status, 'success')

    def rename_current_profile(self):
        """Rename the loaded profile without replacing an existing profile."""
        current = self.loaded_profile_name
        if not current:
            return
        if self.configuration_dirty:
            QMessageBox.information(
                self,
                'Cambios sin guardar',
                'Guarda o descarta los cambios antes de renombrar el perfil.',
            )
            return
        name, accepted = QInputDialog.getText(
            self,
            'Renombrar perfil',
            'Nuevo nombre:',
            text=current,
        )
        if not accepted:
            return
        try:
            document = rename_profile(self.configuration_document, current, name)
            self.configuration_document = self.configuration_store.save(document)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'No se pudo renombrar', str(exc))
            return
        self.loaded_profile_name = self.configuration_document['active_profile']
        self.refresh_profile_combo(self.loaded_profile_name)
        self.configuration_status.setText(f'Perfil renombrado: {self.loaded_profile_name}')
        set_widget_state(self.configuration_status, 'success')

    def delete_current_profile(self):
        """Delete the loaded profile only after explicit confirmation."""
        current = self.loaded_profile_name
        if not current:
            return
        answer = QMessageBox.question(
            self,
            'Eliminar perfil',
            f'¿Eliminar permanentemente el perfil local {current}?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        try:
            document = delete_profile(self.configuration_document, current)
            self.configuration_document = self.configuration_store.save(document)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'No se pudo eliminar', str(exc))
            return
        next_name = self.configuration_document['active_profile']
        self.loaded_profile_name = None
        self.refresh_profile_combo(next_name)
        self._load_profile(next_name, persist_selection=False)
        self.configuration_status.setText(f'Perfil eliminado: {current}')
        set_widget_state(self.configuration_status, 'success')

    def configuration_changed(self, *_):
        """Mark local edits without writing either disk or autopilot."""
        if self.configuration_loading:
            return
        self.configuration_dirty = True
        self.invalidate_preparation()
        self.configuration_status.setText(
            f'{self.loaded_profile_name or "Perfil"} * · cambios sin guardar · '
            'el ROV todavía no fue modificado'
        )
        set_widget_state(self.configuration_status, 'warning')

    def update_parameter_preview(self, *_):
        """Show the exact raw values represented by descriptive controls."""
        try:
            values = profile_to_raw_parameters(self.current_profile())
        except ValueError as exc:
            self.parameter_raw_values.setText(f'Configuración inválida: {exc}')
            return
        self.parameter_raw_values.setText(
            'Valores técnicos propuestos: '
            f'PWM {values["MOT_PWM_MIN"]:.0f}–{values["MOT_PWM_MAX"]:.0f} µs · '
            f'subida {values["WPNAV_SPEED_UP"]:g} cm/s · '
            f'bajada {values["WPNAV_SPEED_DN"]:g} cm/s · '
            f'aceleración {values["WPNAV_ACCEL_Z"]:g} cm/s²'
        )

    def receive_parameter_status(self, message):
        """Store bridge parameter status and persist a verified pre-write snapshot."""
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(status, dict):
            return
        self.parameter_status = status
        self.parameter_status_at = time.monotonic()
        result_key = (status.get('request_id'), status.get('result'))
        if (
            result_key != getattr(self, 'last_parameter_result_key', None)
            and status.get('result') in ('applied', 'restored')
            and status.get('snapshot')
        ):
            try:
                document = set_vehicle_snapshot(
                    self.configuration_document,
                    status['snapshot'],
                    vehicle=status.get('vehicle'),
                    captured_at=status.get('snapshot_captured_at'),
                )
                self.configuration_document = self.configuration_store.save(document)
            except (OSError, ValueError):
                pass
        self.last_parameter_result_key = result_key

    def _parameter_request(self, action, values=None, vehicle=None):
        request_id = f'{self.ui_id}:{self.parameter_request_count}'
        self.parameter_request_count += 1
        packet = {
            'action': action,
            'request_id': request_id,
        }
        if values is not None:
            packet['values'] = values
        if vehicle is not None:
            packet['vehicle'] = vehicle
        self.publish(self.parameter_request_pub, packet)

    def request_vehicle_parameters(self):
        """Request a harmless read of the five parameters."""
        self._parameter_request('read')

    def apply_manual_command_scale(self):
        """Apply the owner-selected runtime scale without persisting ArduSub data."""
        message = Float32()
        message.data = float(self.manual_command_scale.value()) / 100.0
        self.command_scale_setpoint_pub.publish(message)
        self.runtime_scope.setText(
            'Escala Xbox/MANUAL solicitada: '
            f'{self.manual_command_scale.value():.1f} %. Esperando confirmación del bridge.'
        )

    def apply_vehicle_parameters(self):
        """Ask for deliberate guarded application of the displayed vehicle values."""
        try:
            values = profile_to_raw_parameters(self.current_profile())
        except ValueError as exc:
            QMessageBox.warning(self, 'Configuración inválida', str(exc))
            return
        answer = QMessageBox.question(
            self,
            'Aplicar rango técnico al ROV',
            'Se modificarán cinco parámetros persistentes con los valores elegidos por '
            'el propietario. La operación solo continuará '
            'en MANUAL, desarmado y sin secuencia activa.\n\n'
            f'PWM: {values["MOT_PWM_MIN"]:.0f}–{values["MOT_PWM_MAX"]:.0f} µs\n'
            f'Ascenso/descenso: {values["WPNAV_SPEED_UP"]:g}/'
            f'{values["WPNAV_SPEED_DN"]:g} cm/s\n'
            f'Aceleración vertical: {values["WPNAV_ACCEL_Z"]:g} cm/s²\n\n'
            'Los límites altos pueden producir aceleración, consumo y empuje elevados.\n\n'
            '¿Aplicar exactamente estos valores y verificar?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self._parameter_request('apply', values)

    def restore_vehicle_parameters(self):
        """Restore the locally persisted snapshot captured before the last apply."""
        snapshot = self.configuration_document.get('last_vehicle_snapshot')
        if not snapshot:
            QMessageBox.information(
                self,
                'Sin respaldo',
                'Todavía no existe una configuración anterior verificada.',
            )
            return
        current_vehicle = (self.parameter_status or {}).get('vehicle')
        if not snapshot_matches_vehicle(snapshot, current_vehicle):
            QMessageBox.warning(
                self,
                'Respaldo no compatible',
                'La restauración está bloqueada: el respaldo no tiene los mismos '
                'IDs MAVLink y versión de firmware que el ROV conectado.',
            )
            return
        answer = QMessageBox.question(
            self,
            'Restaurar configuración anterior',
            'Se restaurará el respaldo capturado antes de la última aplicación '
            f'confirmada ({snapshot.get("captured_at") or "fecha no disponible"}). '
            '¿Continuar?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self._parameter_request(
                'restore',
                snapshot['values'],
                vehicle=snapshot['vehicle'],
            )

    @staticmethod
    def _parameter_comparison_text(actual, desired):
        """Render requested/readback values so differences are auditable."""
        labels = (
            ('MOT_PWM_MIN', 'PWM mín.', 'µs'),
            ('MOT_PWM_MAX', 'PWM máx.', 'µs'),
            ('WPNAV_SPEED_UP', 'subida', 'cm/s'),
            ('WPNAV_SPEED_DN', 'bajada', 'cm/s'),
            ('WPNAV_ACCEL_Z', 'acel. Z', 'cm/s²'),
        )
        parts = []
        for name, label, unit in labels:
            readback = actual.get(name)
            shown = '—' if readback is None else f'{float(readback):g}'
            parts.append(f'{label} {shown}/{desired[name]:g} {unit}')
        return 'Leído / perfil · ' + ' · '.join(parts)

    def update_electrical_audit(self, status):
        """Explain relevant ArduSub safeguards without claiming SSR protection."""
        values = status.get('read_only_values') or {}
        limit = values.get('MOT_BAT_CURR_MAX')
        self.current_limit_a = (
            float(limit) if limit is not None and float(limit) > 0 else None
        )
        if not values:
            self.electrical_state.setText(
                'Protección eléctrica: esperando parámetros de solo lectura'
            )
            set_widget_state(self.electrical_state, 'neutral')
            return

        facts = []
        warnings = []
        if self.current_limit_a is None:
            warnings.append('límite de corriente ArduSub desactivado/no confirmado')
        else:
            facts.append(f'límite controlador {self.current_limit_a:g} A')
        current_tc = values.get('MOT_BAT_CURR_TC')
        if current_tc is not None:
            facts.append(f'constante corriente {current_tc:g} s')
        monitor = values.get('BATT_MONITOR')
        if monitor == 4:
            facts.append('monitor batería analógico activo')
            warnings.append('calibración de corriente no verificable desde la interfaz')
        elif monitor is not None:
            warnings.append(f'BATT_MONITOR={monitor:g}; confirmar sensor/calibración')
        capacity = values.get('BATT_CAPACITY')
        if capacity is not None and capacity <= 0:
            warnings.append('capacidad de batería no configurada')
        low_voltage = values.get('BATT_LOW_VOLT')
        critical_voltage = values.get('BATT_CRT_VOLT')
        if low_voltage == 0 and critical_voltage == 0:
            warnings.append('umbrales de tensión baja/crítica desactivados')
        low_action = values.get('BATT_FS_LOW_ACT')
        critical_action = values.get('BATT_FS_CRT_ACT')
        if low_action == 0 and critical_action == 0:
            warnings.append('failsafes de batería sin acción automática')
        slew_up = values.get('MOT_SLEW_UP_TIME')
        slew_down = values.get('MOT_SLEW_DN_TIME')
        if slew_up == 0 and slew_down == 0:
            warnings.append('limitadores MOT_SLEW no activos')
        pwm_type = values.get('MOT_PWM_TYPE')
        if pwm_type is not None:
            facts.append(f'MOT_PWM_TYPE={pwm_type:g}')
        arming = values.get('ARMING_CHECK')
        if arming is not None:
            facts.append(f'ARMING_CHECK={arming:g}')
        for name, label in (
            ('FS_GCS_ENABLE', 'GCS'),
            ('FS_PILOT_INPUT', 'piloto'),
            ('FS_LEAK_ENABLE', 'fuga'),
        ):
            if name in values:
                facts.append(f'failsafe {label}={values[name]:g}')

        text = 'Diagnóstico (no garantiza el SSR): ' + ' · '.join(
            facts or ['sin datos completos']
        )
        if warnings:
            text += ' · REVISAR: ' + '; '.join(warnings)
        self.electrical_state.setText(text)
        set_widget_state(self.electrical_state, 'warning' if warnings else 'success')

    def update_parameter_controls(self):
        """Render comparison and gate write buttons from live bridge status."""
        status = self.parameter_status
        fresh = status is not None and time.monotonic() - self.parameter_status_at < 1.5
        if not fresh:
            self.parameter_state.setText('Sin estado reciente del puente de parámetros')
            set_widget_state(self.parameter_state, 'warning')
            self.parameter_comparison.setText(
                'Comparación: sin lectura reciente del ROV'
            )
            self.electrical_state.setText(
                'Protección eléctrica: sin lectura reciente del ROV'
            )
            set_widget_state(self.electrical_state, 'warning')
            self.current_limit_a = None
            self.read_parameters_button.setEnabled(False)
            self.apply_parameters_button.setEnabled(False)
            self.restore_parameters_button.setEnabled(False)
            return

        busy = bool(status.get('busy'))
        connected = bool(status.get('connected'))
        safe = bool(status.get('safe_to_write'))
        complete = bool(status.get('complete'))
        actual = status.get('values') or {}
        try:
            desired = profile_to_raw_parameters(self.current_profile())
            matching = complete and raw_parameters_match(actual, desired)
        except ValueError:
            desired = None
            matching = False

        snapshot = self.configuration_document.get('last_vehicle_snapshot')
        restore_matches = snapshot_matches_vehicle(snapshot, status.get('vehicle'))

        self.read_parameters_button.setEnabled(connected and not busy)
        self.apply_parameters_button.setEnabled(
            safe and complete and not busy and not matching
        )
        self.restore_parameters_button.setEnabled(
            safe
            and not busy
            and restore_matches
        )

        if desired is None:
            self.parameter_comparison.setText('Comparación: perfil mostrado inválido')
        else:
            self.parameter_comparison.setText(
                self._parameter_comparison_text(actual, desired)
            )
        self.update_electrical_audit(status)

        if busy:
            text = status.get('note') or 'Operación de parámetros en curso…'
            state = 'info'
        elif status.get('result') in ('failed_partial', 'rolled_back', 'rejected'):
            text = status.get('note') or 'La operación no fue aplicada.'
            state = 'danger' if status.get('result') == 'failed_partial' else 'warning'
        elif matching:
            text = 'ROV verificado: los cinco parámetros coinciden con este perfil.'
            state = 'success'
        elif complete:
            text = (
                'El ROV difiere del perfil mostrado. Leer no modifica; '
                'Aplicar requiere confirmación.'
            )
            state = 'warning'
        else:
            text = status.get('note') or 'Esperando los cinco valores del ROV.'
            state = 'neutral'
        if not status.get('write_enabled'):
            text += ' Escritura bloqueada por el lanzamiento.'
        if snapshot and not restore_matches:
            text += ' Restaurar bloqueado hasta confirmar IDs MAVLink y firmware.'
        self.parameter_state.setText(text)
        set_widget_state(self.parameter_state, state)

    def validate_sequence(self):
        # The inherited dry-run button must not publish the old request format.
        self.tabs.setCurrentIndex(self.tabs.count() - 1)
        self.send_action('prepare')

    def cancel_sequence(self):
        # Keep the inherited "entregar al mando" button operational in this GUI.
        self.send_action('manual')

    def sequence_status(self, msg):
        # Dry-run status belongs only to the legacy pool dashboard.
        pass

    @staticmethod
    def publish(pub, data):
        msg = String()
        msg.data = json.dumps(data, allow_nan=False)
        pub.publish(msg)

    def receive_status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict) and 'session' in status and 'ticket' in status:
                self.exec_status = status
                self.exec_status_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def sync_controller_ranges(self, *_):
        """Keep dependent tuning values inside the executor's accepted envelope."""
        recovery_radius = self.recovery_radius.value()
        self.recovery_radius.setRange(
            self.xy_tolerance.value() + 0.005,
            min(0.50, self.radius.value()),
        )
        self.recovery_radius.setValue(recovery_radius)

        recovery_speed = self.recovery_speed.value()
        self.recovery_speed.setRange(0.001, self.max_horizontal_speed.value())
        self.recovery_speed.setValue(recovery_speed)

        recovery_min_speed = self.recovery_min_speed.value()
        self.recovery_min_speed.setRange(0.001, self.recovery_speed.value())
        self.recovery_min_speed.setValue(recovery_min_speed)

    def invalidate_preparation(self, *_):
        """Require a new prepare cycle after controller settings change."""
        self.prepared_signature = None
        self.pending_prepare = None

    def controller_tuning(self):
        """Return exactly the tuning keys consumed by the physical executor."""
        return controller_tuning_payload(
            self.xy_tolerance.value(),
            self.recovery_radius.value(),
            self.recovery_speed.value(),
            self.recovery_min_speed.value(),
            self.recovery_delay.value(),
            self.position_gain.value(),
            self.velocity_damping.value(),
        )

    def send_action(self, action):
        s = self.exec_status
        if not s or time.monotonic() - self.exec_status_at > 0.5:
            return
        if action == 'prepare':
            self.prepared_signature = None
            self.pending_prepare = (self.signature(), time.monotonic(), self.action_count)
        active_steps = self.active_steps()
        request = {
            'session': s['session'],
            'ticket': s['ticket'],
            'ui': self.ui_id,
            'serial': self.action_count,
            'action': action,
            'steps': active_steps,
            'plan_mode': self.active_plan_mode,
            'sensor_depth': self.sensor_depth.value(),
            'max_depth': self.maximum_depth.value(),
            'radius': self.radius.value(),
            'max_horizontal_speed': self.max_horizontal_speed.value(),
            'max_vertical_speed': self.max_vertical_speed.value(),
            'max_yaw_speed_deg': self.max_yaw_speed.value(),
        }
        request.update(self.controller_tuning())
        self.publish(self.request_pub, request)
        self.action_count += 1

    def signature(self):
        active_steps = self.active_steps()
        base = (
            self.active_plan_mode,
            json.dumps(active_steps, sort_keys=True),
            self.sensor_depth.value(),
            self.maximum_depth.value(),
            self.radius.value(),
            self.max_horizontal_speed.value(),
            self.max_vertical_speed.value(),
            self.max_yaw_speed.value(),
        )
        return base + tuple(self.controller_tuning().values())

    def sync_step_editor_limits(self, *_):
        """Refresh dependent ranges without replacing the operator's values."""
        self.configure_step_editor(preserve=True)

    def configure_step_editor(self, *_, preserve=False):
        previous_amount = self.amount.value()
        previous_secondary = self.secondary.value()
        kind = self.kind.currentData()
        timed = kind in ('velocity_surge', 'velocity_sway', 'velocity_heave', 'velocity_yaw')
        self.secondary_label.setVisible(kind != 'wait')
        self.secondary.setVisible(kind != 'wait')
        if kind == 'advance':
            self.primary_label.setText('Distancia [m] (+ adelante / − atrás)')
            self.amount.setRange(-self.radius.value(), self.radius.value())
            self.amount.setSingleStep(0.10)
            self.secondary_label.setText('Velocidad máx. [m/s]')
            limit = self.max_horizontal_speed.value()
            self.secondary.setRange(0.001, limit)
            self.secondary.setValue(min(0.15, limit))
        elif kind == 'lateral':
            self.primary_label.setText('Distancia [m] (+ derecha / − izquierda)')
            self.amount.setRange(-self.radius.value(), self.radius.value())
            self.amount.setSingleStep(0.10)
            self.secondary_label.setText('Velocidad máx. [m/s]')
            limit = self.max_horizontal_speed.value()
            self.secondary.setRange(0.001, limit)
            self.secondary.setValue(min(0.15, limit))
        elif kind == 'turn':
            self.primary_label.setText('Ángulo [°] (+ derecha)')
            self.amount.setRange(-180.0, 180.0)
            self.amount.setSingleStep(5.0)
            self.secondary_label.setText('Velocidad máx. [°/s]')
            limit = self.max_yaw_speed.value()
            self.secondary.setRange(0.001, limit)
            self.secondary.setValue(min(8.0, limit))
        elif kind == 'depth':
            self.primary_label.setText('Profundidad absoluta [m]')
            self.amount.setRange(0.0, self.maximum_depth.value())
            self.amount.setSingleStep(0.05)
            self.secondary_label.setText('Velocidad máx. [m/s]')
            limit = self.max_vertical_speed.value()
            self.secondary.setRange(0.001, limit)
            self.secondary.setValue(min(0.08, limit))
        elif timed:
            unit = '°/s' if kind == 'velocity_yaw' else 'm/s'
            self.primary_label.setText(f'Velocidad [{unit}] (+ adelante/derecha/abajo)')
            limit = (
                self.max_yaw_speed.value()
                if kind == 'velocity_yaw'
                else self.max_vertical_speed.value()
                if kind == 'velocity_heave'
                else self.max_horizontal_speed.value()
            )
            self.amount.setRange(-limit, limit)
            self.amount.setSingleStep(0.5 if kind == 'velocity_yaw' else 0.01)
            default = 5.0 if kind == 'velocity_yaw' else 0.10
            self.amount.setValue(min(default, limit))
            self.secondary_label.setText('Duración [s]')
            self.secondary.setRange(0.1, 5.0)
            self.secondary.setValue(2.0)
        else:
            self.primary_label.setText('Duración [s]')
            self.amount.setRange(0.0, 30.0)
            self.amount.setSingleStep(0.5)
            self.amount.setValue(2.0)
        if preserve:
            self.amount.setValue(previous_amount)
            self.secondary.setValue(previous_secondary)

    def step_from_editor(self):
        kind = self.kind.currentData()
        value = self.amount.value()
        if kind == 'wait':
            return {'kind': kind, 'duration': value}
        elif kind.startswith('velocity_'):
            if abs(value) < 1e-9:
                self.result.setPlainText('La velocidad temporal no puede ser cero.')
                return None
            speed = math.radians(value) if kind == 'velocity_yaw' else value
            return {'kind': kind, 'speed': speed, 'duration': self.secondary.value()}
        speed = math.radians(self.secondary.value()) if kind == 'turn' else self.secondary.value()
        return {'kind': kind, 'value': value, 'speed': speed}

    def add_step(self):
        step = self.step_from_editor()
        if step is None:
            return
        self.relative_steps.append(step)
        self.update_steps(len(self.relative_steps) - 1)
        self.autosave_mission_draft()

    def insert_step_after(self):
        step = self.step_from_editor()
        if step is None:
            return
        row = self.step_list.currentRow()
        index = row + 1 if row >= 0 else len(self.relative_steps)
        self.relative_steps.insert(index, step)
        self.steps = self.relative_steps
        self.update_steps(index)
        self.autosave_mission_draft()

    def duplicate_selected_step(self):
        row = self.step_list.currentRow()
        if row < 0:
            self.result.setPlainText('Selecciona una orden para duplicarla.')
            return
        self.relative_steps, selected = duplicate_step(self.relative_steps, row)
        self.steps = self.relative_steps
        self.update_steps(selected)
        self.autosave_mission_draft()

    def update_selected_step(self):
        row = self.step_list.currentRow()
        if row < 0:
            self.result.setPlainText('Selecciona una orden de la lista para actualizarla.')
            return
        step = self.step_from_editor()
        if step is None:
            return
        self.relative_steps = replace_step(self.relative_steps, row, step)
        self.steps = self.relative_steps
        self.update_steps(row)
        self.autosave_mission_draft()

    def remove_selected_step(self):
        row = self.step_list.currentRow()
        if row < 0:
            self.result.setPlainText('Selecciona una orden de la lista para eliminarla.')
            return
        del self.relative_steps[row]
        self.update_steps(min(row, len(self.relative_steps) - 1))
        self.autosave_mission_draft()

    def move_selected_step(self, offset):
        row = self.step_list.currentRow()
        if row < 0:
            self.result.setPlainText('Selecciona una orden para cambiar su posición.')
            return
        self.relative_steps, selected = move_step(self.relative_steps, row, offset)
        self.steps = self.relative_steps
        self.update_steps(selected)
        self.autosave_mission_draft()

    def clear_editable_steps(self):
        if self.relative_steps:
            answer = QMessageBox.question(
                self,
                'Vaciar plan',
                '¿Eliminar todas las órdenes del plan actual?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.relative_steps.clear()
        self.update_steps()
        self.autosave_mission_draft()

    def load_selected_step(self, row):
        selected = 0 <= row < len(self.relative_steps)
        self.update_step_button.setEnabled(selected)
        self.insert_step_button.setEnabled(True)
        self.duplicate_step_button.setEnabled(selected)
        self.remove_step_button.setEnabled(selected)
        self.move_up_button.setEnabled(selected and row > 0)
        self.move_down_button.setEnabled(selected and row < len(self.relative_steps) - 1)
        if not selected:
            return
        step = self.relative_steps[row]
        kind_index = self.kind.findData(step['kind'])
        if kind_index < 0:
            return
        self.kind.blockSignals(True)
        self.kind.setCurrentIndex(kind_index)
        self.kind.blockSignals(False)
        self.configure_step_editor()
        kind = step['kind']
        if kind == 'wait':
            self.amount.setValue(step['duration'])
        elif kind.startswith('velocity_'):
            speed = math.degrees(step['speed']) if kind == 'velocity_yaw' else step['speed']
            self.amount.setValue(speed)
            self.secondary.setValue(step['duration'])
        else:
            speed = math.degrees(step['speed']) if kind == 'turn' else step['speed']
            self.amount.setValue(step['value'])
            self.secondary.setValue(speed)

    @staticmethod
    def format_step(step):
        if step['kind'] == 'absolute_waypoint':
            yaw = (
                step['yaw_deg']
                if 'yaw_deg' in step
                else math.degrees(step['yaw'])
            )
            return (
                f'Punto absoluto X={step["x"]:.2f}, Y={step["y"]:.2f}, '
                f'prof.={step["depth"]:.2f} m, rumbo={yaw:.1f}°, '
                f'máx.={step["speed"]:.2f} m/s'
            )
        names = {
            'advance': 'Distancia adelante/atrás',
            'lateral': 'Distancia derecha/izquierda',
            'turn': 'Giro por ángulo',
            'depth': 'Profundidad absoluta',
            'velocity_surge': 'Velocidad adelante/atrás',
            'velocity_sway': 'Velocidad derecha/izquierda',
            'velocity_heave': 'Velocidad descenso/ascenso',
            'velocity_yaw': 'Velocidad de giro',
            'wait': 'Espera',
        }
        kind = step['kind']
        if kind == 'wait':
            return f'{names[kind]}: {step["duration"]:g} s'
        if kind.startswith('velocity_'):
            speed = math.degrees(step['speed']) if kind == 'velocity_yaw' else step['speed']
            unit = '°/s' if kind == 'velocity_yaw' else 'm/s'
            return f'{names[kind]}: {speed:g} {unit} durante {step["duration"]:g} s'
        speed = math.degrees(step['speed']) if kind == 'turn' else step['speed']
        value_unit = '°' if kind == 'turn' else 'm'
        speed_unit = '°/s' if kind == 'turn' else 'm/s'
        return f'{names[kind]}: {step["value"]:g} {value_unit} · máx. {speed:g} {speed_unit}'

    @staticmethod
    def step_cells(index, step):
        """Return compact table cells for one movement order."""
        names = {
            'advance': 'Avanzar / retroceder',
            'lateral': 'Derecha / izquierda',
            'turn': 'Girar por ángulo',
            'depth': 'Profundidad absoluta',
            'velocity_surge': 'Velocidad longitudinal',
            'velocity_sway': 'Velocidad lateral',
            'velocity_heave': 'Velocidad vertical',
            'velocity_yaw': 'Velocidad de giro',
            'wait': 'Esperar',
        }
        kind = step['kind']
        if kind == 'wait':
            return (str(index), names[kind], '—', '—', f'{step["duration"]:g} s')
        if kind.startswith('velocity_'):
            speed = math.degrees(step['speed']) if kind == 'velocity_yaw' else step['speed']
            unit = '°/s' if kind == 'velocity_yaw' else 'm/s'
            return (
                str(index),
                names[kind],
                f'{speed:g} {unit}',
                '—',
                f'{step["duration"]:g} s',
            )
        speed = math.degrees(step['speed']) if kind == 'turn' else step['speed']
        value_unit = '°' if kind == 'turn' else 'm'
        speed_unit = '°/s' if kind == 'turn' else 'm/s'
        return (
            str(index),
            names[kind],
            f'{step["value"]:g} {value_unit}',
            f'{speed:g} {speed_unit}',
            'estimada',
        )

    def update_steps(self, selected_row=None):
        # PoolDashboard constructs the inherited editor before this page.
        # Keep its early refresh compatible until the selectable list exists.
        if not hasattr(self, 'step_list'):
            self.step_text.setPlainText(
                '\n'.join(
                    f'{index}. {self.format_step(step)}'
                    for index, step in enumerate(self.relative_steps, 1)
                )
            )
            self.clear_preview()
            return
        self.step_list.blockSignals(True)
        self.step_list.setRowCount(len(self.relative_steps))
        for row, step in enumerate(self.relative_steps):
            for column, text in enumerate(self.step_cells(row + 1, step)):
                item = QTableWidgetItem(text)
                if column in (0, 2, 3, 4):
                    item.setTextAlignment(Qt.AlignCenter)
                self.step_list.setItem(row, column, item)
        self.step_list.blockSignals(False)
        if selected_row is not None and 0 <= selected_row < len(self.relative_steps):
            self.step_list.setCurrentCell(selected_row, 0)
        else:
            self.step_list.clearSelection()
            self.step_list.setCurrentCell(-1, -1)
            self.load_selected_step(-1)
        self.prepared_signature = None
        self.pending_prepare = None
        self.clear_preview()

    def tick_sequence(self):
        self.update_parameter_controls()
        self.update_accessory_panel()
        manual_scale = self.data.get('command_scale')
        manual_text = (
            f'{100.0 * float(manual_scale):.1f}%'
            if manual_scale is not None
            else 'sin dato reciente'
        )
        self.runtime_scope.setText(
            f'Ámbito: Xbox/MANUAL usa command_scale={manual_text}; los máximos '
            'de sesión afectan solo secuencias; PWM/WPNAV permanecen en ArduSub.'
        )
        s = self.exec_status
        fresh = s is not None and time.monotonic() - self.exec_status_at < 0.5
        active_steps = self.active_steps()
        metrics = mission_metrics(self.current_mission())
        summary = (
            f'{metrics["steps"]} pasos · recorrido horizontal aprox. '
            f'{metrics["horizontal_distance_m"]:.2f} m · tiempo mínimo ideal '
            f'{metrics["minimum_time_s"]:.1f} s (sin asentamiento dinámico extra)'
            if active_steps
            else 'sin pasos'
        )
        if self.active_plan_mode == 'relative':
            self.list_summary.setText('Plan relativo activo: ' + summary)
        else:
            self.absolute_summary.setText('Plan absoluto activo: ' + summary)
        for button in self.buttons.values():
            button.setEnabled(fresh)
        if not fresh:
            self.update_workflow(None, False)
            self.update_health(None, False)
            self.update_guidance(None, False, False)
            self.stop_button.setText(
                'ABORTO POR INTERFAZ NO DISPONIBLE · USA MANDO O KILLSWITCH'
            )
            self.exec_text.setPlainText(
                'Ejecutor sin datos recientes. No se puede iniciar movimiento.'
            )
            self.phase_title.setText('SIN DATOS DEL EJECUTOR')
            self.phase_detail.setText('No se conoce el estado del movimiento')
            set_widget_state(self.phase_title, 'warning')
            self.execution_status.setText('GUIDED: sin ejecutor reciente; inicio bloqueado')
            self.status.setText('Secuencias: esperando ejecutor y puente')
            set_widget_state(self.status, 'warning')
            return
        self.publish(
            self.alive_pub,
            {
                'session': s['session'],
                'ticket': s['ticket'],
                'ui': self.ui_id,
                'serial': self.ui_count,
            },
        )
        self.ui_count += 1
        running = s['state'] == 'RUNNING'
        if self.pending_prepare is not None:
            signature, sent_at, serial = self.pending_prepare
            if s.get('prepared_by') == [self.ui_id, serial] and s.get('validated'):
                self.prepared_signature = signature
                self.pending_prepare = None
            elif time.monotonic() - sent_at > 2:
                self.pending_prepare = None
        matching = self.prepared_signature == self.signature()
        self.update_workflow(s, matching)
        self.update_health(s, True)
        self.update_guidance(s, True, matching)
        self.stop_button.setText('ABORTAR MOVIMIENTO → MANUAL · NO DESARMA')
        for action in ('prepare', 'guided', 'set_home', 'execute'):
            self.buttons[action].setEnabled(not running)
        self.buttons['prepare'].setEnabled(
            not running
            and s.get('armed') is False
            and s.get('mode') == 'MANUAL'
            and bool(active_steps)
            and not (matching and s.get('validated', False))
        )
        self.buttons['execute'].setEnabled(
            matching
            and not running
            and s.get('validated', False)
            and s.get('home_ready', bool(s.get('home')))
            and s.get('enabled', False)
            and s.get('armed') is True
            and s.get('mode') == 'GUIDED'
            and s.get('navigation_ready') is True
            and not s.get('blockers')
        )
        self.buttons['guided'].setEnabled(
            matching
            and not running
            and s.get('validated', False)
            and s.get('armed') is True
            and s.get('navigation_ready') is True
            and not s.get('blockers')
            and s.get('mode') != 'GUIDED'
        )
        self.buttons['set_home'].setEnabled(
            matching
            and not running
            and s.get('validated', False)
            and not s.get('home_ready', bool(s.get('home')))
            and s.get('armed') is True
            and s.get('mode') == 'GUIDED'
            and s.get('navigation_ready') is True
            and not s.get('guided_entry_pending', False)
            and not s.get('guided_settling', False)
            and not s.get('blockers')
        )
        self.plan_editor.setEnabled(not running and s.get('armed') is False)
        self.step_list.setEnabled(not running and s.get('armed') is False)
        self.absolute_plan_editor.setEnabled(not running and s.get('armed') is False)
        self.absolute_list.setEnabled(not running and s.get('armed') is False)
        self.planner_tabs.setEnabled(not running and s.get('armed') is False)
        configuration_editable = not running and s.get('armed') is False
        for entry in self.configuration_inputs:
            entry.setEnabled(configuration_editable)
        for button in (
            self.load_profile_button,
            self.save_profile_button,
            self.save_profile_as_button,
            self.rename_profile_button,
            self.delete_profile_button,
            self.reload_profile_button,
            self.load_mission_button,
            self.save_mission_button,
            self.save_mission_as_button,
            self.rename_mission_button,
            self.delete_mission_button,
            self.new_mission_button,
        ):
            button.setEnabled(configuration_editable)
        self.profile_combo.setEnabled(configuration_editable)
        output = 'HABILITADA' if s.get('enabled') else 'DESHABILITADA'
        self.status.setText(f'Salida real {output} · {s.get("mode")} · {s["state"]}')
        self.execution_status.setText(self.status.text() + ' · controles en Operación')
        phase = s.get('phase') or {
            'code': 'IDLE',
            'title': s['state'],
            'detail': s.get('note', ''),
        }
        visual_states = {
            'PAUSA': 'warning',
            'FALLA': 'danger',
            'BLOQUEADO': 'danger',
            'ESPERA': 'info',
            'AVANCE': 'success',
            'GIRO': 'success',
            'TEMPORAL': 'success',
            'PROFUNDIDAD': 'success',
            'RUMBO': 'success',
            'ESTABILIZANDO': 'info',
            'CORRIGIENDO': 'warning',
            'RECUPERACION': 'warning',
            'COMPLETADO': 'success',
            'LISTO': 'success',
            'LISTO_GUIDED': 'info',
            'HOME_PENDIENTE': 'info',
            'PREPARADO': 'info',
            'IDLE': 'neutral',
        }
        self.phase_title.setText(phase.get('title', s['state']))
        self.phase_detail.setText(phase.get('detail', ''))
        set_widget_state(
            self.phase_title,
            visual_states.get(phase.get('code'), 'neutral'),
        )
        if phase.get('code') in ('FALLA', 'BLOQUEADO'):
            set_widget_state(self.status, 'danger')
        elif running:
            set_widget_state(self.status, 'info')
        elif s.get('armed') is True:
            set_widget_state(self.status, 'warning')
        else:
            set_widget_state(self.status, 'success')
        lines = [
            f'Salida real: {output}',
            f'Estado: {s["state"]} · Modo: {s.get("mode")} · Armado: {s.get("armed")}',
            (
                f'Energía: {s.get("voltage_v")} V · {s.get("current_a")} A '
                '(telemetría; no mide el pico de conmutación del SSR)'
            ),
            s.get('note', ''),
            f'Referencia superficie: {s.get("surface")} · Home: {s.get("home")}',
            f'Confianza del mensaje DVL: {s.get("dvl_confidence")}',
            f'Último estado del puente: {s.get("bridge_reason")}',
        ]
        lines.append(
            'Navegación lista para iniciar: ' + ('Sí' if s.get('navigation_ready') else 'No')
        )
        if s.get('dvl_velocity_valid') is not None:
            lines.append(
                f'DVL directo válido: {s["dvl_velocity_valid"]} · '
                f'altura: {s.get("dvl_altitude")} m'
            )
        if s.get('diagnostics'):
            lines.append(
                'Avisos de sensores (el estado directo del DVL no detiene):\n'
                + '\n'.join('• ' + item for item in s['diagnostics'])
            )
        pose = s.get('pose')
        if pose:
            lines.append(
                f'Actual: N={pose["n"]:.2f}, E={pose["e"]:.2f}, '
                f'prof={pose["depth"]:.2f} m, yaw={math.degrees(pose["yaw"]):.1f}°'
            )
        if s.get('goal'):
            goal = s['goal']
            errors = s['errors']
            current_step = s.get('current_step')
            step_text = self.format_step(current_step) if current_step else 'desconocido'
            lines += [
                f'Paso {s["step"]}/{s["total"]}: {step_text}',
                f'Objetivo: {goal}',
                (
                    f'Error XY {errors[0]:.2f} m · '
                    f'profundidad {errors[1]:.2f} m · '
                    f'rumbo {errors[2]:.1f}°'
                ),
                f'Tiempo máximo restante: {s["timeout_left"]:.1f} s',
            ]
        if s.get('nav_error'):
            lines.append('Navegación: ' + s['nav_error'])
        if s.get('blockers'):
            lines.append('Bloqueos:\n' + '\n'.join('• ' + b for b in s['blockers']))
        if s.get('stopping'):
            lines.append('Parada solicitada; esperando modo ' + s['stopping'])
        if s.get('event_log_path'):
            lines.append('Registro de este lanzamiento: ' + s['event_log_path'])
        self.exec_text.setPlainText('\n'.join(lines))

    def update_workflow(self, status, matching):
        """Show progress without changing any executor or safety condition."""
        fresh = status is not None
        states = workflow_states(status, fresh, bool(self.active_steps()), matching)
        for name, state in states.items():
            set_widget_state(self.stage_labels[name], state)

    def update_guidance(self, status, fresh, matching):
        """Render one next action and explicit reasons for blocking it."""
        guide = operator_guidance(status, fresh, bool(self.active_steps()), matching)
        self.next_action.setText(guide['message'])
        set_widget_state(self.next_action, guide['state'])
        self.blocker_text.setText(
            '\n'.join('• ' + reason for reason in guide['reasons'])
        )
        self.stop_button.setEnabled(guide['abort_available'])

    def update_health(self, status, fresh):
        """Summarize the state required for an operator decision."""
        if not fresh or status is None:
            values = {
                'executor': ('EJECUTOR · SIN DATOS', 'warning'),
                'link': ('AUTOPILOTO · —', 'neutral'),
                'mode': ('MODO · —', 'neutral'),
                'armed': ('ARMADO · —', 'neutral'),
                'output': ('SALIDA · —', 'neutral'),
                'navigation': ('NAVEGACIÓN · —', 'neutral'),
                'dvl': ('DVL · —', 'neutral'),
                'current': ('CORRIENTE · —', 'neutral'),
                'alert': ('ALERTAS · —', 'neutral'),
            }
        else:
            armed = status.get('armed')
            enabled = status.get('enabled')
            navigation = status.get('navigation_ready')
            dvl = status.get('dvl_confidence')
            connected = status.get('connected') is True
            current = status.get('current_a')
            if current is None:
                current_value = ('CORRIENTE · SIN DATOS', 'warning')
            elif self.current_limit_a is None:
                current_value = (f'CORRIENTE · {current:.1f} A', 'info')
            elif current >= self.current_limit_a:
                current_value = (f'CORRIENTE · {current:.1f} A', 'danger')
            elif current >= 0.80 * self.current_limit_a:
                current_value = (f'CORRIENTE · {current:.1f} A', 'warning')
            else:
                current_value = (f'CORRIENTE · {current:.1f} A', 'success')
            if self.alerts:
                severity, _text, stamp = self.alerts[-1]
                alert_recent = time.monotonic() - stamp < 10.0
                alert_value = (
                    (f'ALERTA MAV · NIVEL {severity}', 'danger' if severity <= 3 else 'warning')
                    if alert_recent and severity <= 4
                    else ('ALERTAS · VER TELEMETRÍA', 'info')
                )
            else:
                alert_value = ('ALERTAS · SIN MENSAJES', 'neutral')
            values = {
                'executor': ('EJECUTOR · ACTIVO', 'success'),
                'link': (
                    'AUTOPILOTO · CONECTADO' if connected else 'AUTOPILOTO · SIN ENLACE',
                    'success' if connected else 'danger',
                ),
                'mode': (
                    f'MODO · {status.get("mode") or "—"}',
                    'success' if status.get('mode') == 'GUIDED' else 'info',
                ),
                'armed': (
                    'ARMADO · SÍ' if armed is True else 'ARMADO · NO',
                    'warning' if armed is True else 'success',
                ),
                'output': (
                    'SALIDA · HABILITADA' if enabled else 'SALIDA · BLOQUEADA',
                    'warning' if enabled else 'info',
                ),
                'navigation': (
                    'NAVEGACIÓN · LISTA' if navigation else 'NAVEGACIÓN · NO LISTA',
                    'success' if navigation else 'warning',
                ),
                'dvl': (
                    f'DVL · {dvl if dvl is not None else "—"}',
                    'info' if dvl is not None else 'neutral',
                ),
                'current': current_value,
                'alert': alert_value,
            }
        for name, (text, state) in values.items():
            self.health_labels[name].setText(text)
            set_widget_state(self.health_labels[name], state)

    def closeEvent(self, event):
        self.camera_tilt_command = 0.0
        neutral = Float32()
        neutral.data = 0.0
        self.camera_tilt_pub.publish(neutral)
        self.autosave_mission_draft()
        self.send_action('cancel')
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    apply_theme(app)
    node = Node('uuv_sequence_dashboard')
    window = SequenceDashboard(node)
    signal.signal(signal.SIGINT, lambda *_: window.close())
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

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
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        self.request_pub = node.create_publisher(String, '/uuv/sequence/request', 10)
        self.alive_pub = node.create_publisher(String, '/uuv/sequence/ui_alive', 1)
        self.subscriptions.append(
            node.create_subscription(String, '/uuv/sequence/status', self.receive_status, 1)
        )
        self.buttons = {}
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
            ('execute', '5  EJECUTAR'),
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

        limits_group = QGroupBox('Límites y referencia de la sesión')
        limits_group.setMinimumHeight(128)
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
            0,
            2,
        )
        self.maximum_depth = QDoubleSpinBox()
        self.maximum_depth.setRange(0.2, 3)
        self.maximum_depth.setDecimals(2)
        self.maximum_depth.setSingleStep(0.01)
        self.maximum_depth.setKeyboardTracking(False)
        self.maximum_depth.setValue(0.2)
        limits_grid.addWidget(self.maximum_depth, 0, 3)
        limits_grid.addWidget(
            self.field_label(
                'Radio desde Home [m]',
                'Desplazamiento horizontal máximo permitido.',
            ),
            1,
            0,
        )
        self.radius = QDoubleSpinBox()
        self.radius.setRange(0.5, 5)
        self.radius.setDecimals(2)
        self.radius.setSingleStep(0.05)
        self.radius.setValue(0.5)
        limits_grid.addWidget(self.radius, 1, 1)
        limits_grid.addWidget(self.field_label('Máx. horizontal [m/s]'), 1, 2)
        self.max_horizontal_speed = QDoubleSpinBox()
        self.max_horizontal_speed.setRange(0.001, sys.float_info.max)
        self.max_horizontal_speed.setDecimals(3)
        self.max_horizontal_speed.setSingleStep(0.01)
        self.max_horizontal_speed.setValue(0.20)
        limits_grid.addWidget(self.max_horizontal_speed, 1, 3)
        limits_grid.addWidget(self.field_label('Máx. vertical [m/s]'), 2, 0)
        self.max_vertical_speed = QDoubleSpinBox()
        self.max_vertical_speed.setRange(0.001, sys.float_info.max)
        self.max_vertical_speed.setDecimals(3)
        self.max_vertical_speed.setSingleStep(0.01)
        self.max_vertical_speed.setValue(0.10)
        limits_grid.addWidget(self.max_vertical_speed, 2, 1)
        limits_grid.addWidget(self.field_label('Máx. giro [°/s]'), 2, 2)
        self.max_yaw_speed = QDoubleSpinBox()
        self.max_yaw_speed.setRange(0.001, sys.float_info.max)
        self.max_yaw_speed.setDecimals(3)
        self.max_yaw_speed.setSingleStep(1.0)
        self.max_yaw_speed.setValue(10.0)
        limits_grid.addWidget(self.max_yaw_speed, 2, 3)
        note = QLabel(
            'El cero de profundidad es local a esta sesión. Los límites pertenecen '
            'al controlador y no sustituyen el margen físico de parada.'
        )
        note.setObjectName('MutedText')
        note.setWordWrap(True)
        limits_grid.addWidget(note, 3, 0, 1, 4)
        planning_layout.addWidget(limits_group)

        controller_group = QGroupBox('Retención de posición y recuperación')
        controller_group.setMinimumHeight(174)
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
            0,
            2,
        )
        self.recovery_radius = QDoubleSpinBox()
        self.recovery_radius.setDecimals(3)
        self.recovery_radius.setSingleStep(0.005)
        self.recovery_radius.setValue(0.08)
        controller_grid.addWidget(self.recovery_radius, 0, 3)

        controller_grid.addWidget(
            self.field_label(
                'Máx. recuperación [m/s]',
                'Velocidad horizontal máxima usada para volver a la posición objetivo.',
            ),
            1,
            0,
        )
        self.recovery_speed = QDoubleSpinBox()
        self.recovery_speed.setDecimals(3)
        self.recovery_speed.setSingleStep(0.005)
        self.recovery_speed.setValue(0.08)
        controller_grid.addWidget(self.recovery_speed, 1, 1)

        controller_grid.addWidget(
            self.field_label(
                'Impulso mínimo [m/s]',
                'Mínimo comando de recuperación, siempre menor o igual que el máximo.',
            ),
            1,
            2,
        )
        self.recovery_min_speed = QDoubleSpinBox()
        self.recovery_min_speed.setDecimals(3)
        self.recovery_min_speed.setSingleStep(0.005)
        self.recovery_min_speed.setValue(0.02)
        controller_grid.addWidget(self.recovery_min_speed, 1, 3)

        controller_grid.addWidget(
            self.field_label(
                'Refuerzo tras respuesta lenta [s]',
                'Espera antes de elevar gradualmente el esfuerzo de recuperación.',
            ),
            2,
            0,
        )
        self.recovery_delay = QDoubleSpinBox()
        self.recovery_delay.setRange(0.5, 10.0)
        self.recovery_delay.setDecimals(1)
        self.recovery_delay.setSingleStep(0.5)
        self.recovery_delay.setValue(2.0)
        controller_grid.addWidget(self.recovery_delay, 2, 1)

        controller_grid.addWidget(
            self.field_label(
                'Ganancia posición [1/s]',
                'Convierte el error de posición en velocidad de corrección.',
            ),
            2,
            2,
        )
        self.position_gain = QDoubleSpinBox()
        self.position_gain.setRange(0.10, 2.0)
        self.position_gain.setDecimals(2)
        self.position_gain.setSingleStep(0.05)
        self.position_gain.setValue(0.80)
        controller_grid.addWidget(self.position_gain, 2, 3)

        controller_grid.addWidget(
            self.field_label(
                'Frenado por velocidad [s]',
                'Amortigua la corrección según la velocidad medida para evitar sobrepasos.',
            ),
            3,
            0,
        )
        self.velocity_damping = QDoubleSpinBox()
        self.velocity_damping.setRange(0.05, 3.0)
        self.velocity_damping.setDecimals(2)
        self.velocity_damping.setSingleStep(0.05)
        self.velocity_damping.setValue(1.20)
        controller_grid.addWidget(self.velocity_damping, 3, 1)
        controller_note = QLabel(
            'Valores conservadores del ejecutor físico. Cambiarlos invalida la preparación '
            'actual y exige preparar de nuevo.'
        )
        controller_note.setObjectName('MutedText')
        controller_note.setWordWrap(True)
        controller_grid.addWidget(controller_note, 3, 2, 1, 2)
        planning_layout.addWidget(controller_group)

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

        # Reuse the existing editor and validation of individual steps.
        editor = QGroupBox('Editor de orden')
        editor.setMinimumHeight(168)
        editor_layout = QVBoxLayout(editor)
        editor_fields = QGridLayout()
        editor_layout.addLayout(editor_fields)
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
            editor_buttons.addWidget(button, index // 3, index % 3)
            setattr(self, attribute, button)
        self.plan_editor = editor
        planning_layout.addWidget(editor)
        self.kind.currentIndexChanged.connect(self.configure_step_editor)
        self.max_horizontal_speed.valueChanged.connect(self.configure_step_editor)
        self.max_vertical_speed.valueChanged.connect(self.configure_step_editor)
        self.max_yaw_speed.valueChanged.connect(self.configure_step_editor)
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
        planning_layout.addWidget(self.step_list, 1)
        self.update_steps()
        self.list_summary = QLabel('Lista: sin pasos')
        self.list_summary.setObjectName('MutedText')
        self.list_summary.setWordWrap(True)
        planning_layout.addWidget(self.list_summary)
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
            'retiene la pose actual antes de habilitar el inicio.'
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
            ('executor', 'mode', 'armed', 'output', 'navigation', 'dvl')
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
        execute_button = self.button('5 · Iniciar plan preparado', 'execute')
        execute_button.setObjectName('SuccessAction')
        action_layout.addWidget(prepare_button)
        arm_note = QLabel('3 · ARMAR se confirma desde el mando; esta interfaz no arma.')
        arm_note.setObjectName('MutedText')
        arm_note.setWordWrap(True)
        action_layout.addWidget(arm_note)
        action_layout.addWidget(guided_button)
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
        self.tabs.addTab(page_scroll, 'Misión GUIDED')
        self.tabs.setCurrentWidget(page_scroll)
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
        request = {
            'session': s['session'],
            'ticket': s['ticket'],
            'ui': self.ui_id,
            'serial': self.action_count,
            'action': action,
            'steps': self.steps,
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
        base = (
            json.dumps(self.steps, sort_keys=True),
            self.sensor_depth.value(),
            self.maximum_depth.value(),
            self.radius.value(),
            self.max_horizontal_speed.value(),
            self.max_vertical_speed.value(),
            self.max_yaw_speed.value(),
        )
        return base + tuple(self.controller_tuning().values())

    def configure_step_editor(self, *_):
        kind = self.kind.currentData()
        timed = kind in ('velocity_surge', 'velocity_sway', 'velocity_heave', 'velocity_yaw')
        self.secondary_label.setVisible(kind != 'wait')
        self.secondary.setVisible(kind != 'wait')
        if kind == 'advance':
            self.primary_label.setText('Distancia [m] (+ adelante / − atrás)')
            self.amount.setRange(-5.0, 5.0)
            self.amount.setSingleStep(0.10)
            self.secondary_label.setText('Velocidad máx. [m/s]')
            limit = self.max_horizontal_speed.value()
            self.secondary.setRange(0.001, limit)
            self.secondary.setValue(min(0.15, limit))
        elif kind == 'lateral':
            self.primary_label.setText('Distancia [m] (+ derecha / − izquierda)')
            self.amount.setRange(-5.0, 5.0)
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
            self.amount.setRange(0.0, 3.0)
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
            self.secondary.setRange(0.1, 30.0)
            self.secondary.setValue(2.0)
        else:
            self.primary_label.setText('Duración [s]')
            self.amount.setRange(0.0, 30.0)
            self.amount.setSingleStep(0.5)
            self.amount.setValue(2.0)

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
        self.steps.append(step)
        self.update_steps(len(self.steps) - 1)

    def update_selected_step(self):
        row = self.step_list.currentRow()
        if row < 0:
            self.result.setPlainText('Selecciona una orden de la lista para actualizarla.')
            return
        step = self.step_from_editor()
        if step is None:
            return
        self.steps = replace_step(self.steps, row, step)
        self.update_steps(row)

    def remove_selected_step(self):
        row = self.step_list.currentRow()
        if row < 0:
            self.result.setPlainText('Selecciona una orden de la lista para eliminarla.')
            return
        del self.steps[row]
        self.update_steps(min(row, len(self.steps) - 1))

    def move_selected_step(self, offset):
        row = self.step_list.currentRow()
        if row < 0:
            self.result.setPlainText('Selecciona una orden para cambiar su posición.')
            return
        self.steps, selected = move_step(self.steps, row, offset)
        self.update_steps(selected)

    def clear_editable_steps(self):
        if self.steps:
            answer = QMessageBox.question(
                self,
                'Vaciar plan',
                '¿Eliminar todas las órdenes del plan actual?',
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.steps.clear()
        self.update_steps()

    def load_selected_step(self, row):
        selected = 0 <= row < len(self.steps)
        self.update_step_button.setEnabled(selected)
        self.remove_step_button.setEnabled(selected)
        self.move_up_button.setEnabled(selected and row > 0)
        self.move_down_button.setEnabled(selected and row < len(self.steps) - 1)
        if not selected:
            return
        step = self.steps[row]
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
                    for index, step in enumerate(self.steps, 1)
                )
            )
            self.clear_preview()
            return
        self.step_list.blockSignals(True)
        self.step_list.setRowCount(len(self.steps))
        for row, step in enumerate(self.steps):
            for column, text in enumerate(self.step_cells(row + 1, step)):
                item = QTableWidgetItem(text)
                if column in (0, 2, 3, 4):
                    item.setTextAlignment(Qt.AlignCenter)
                self.step_list.setItem(row, column, item)
        self.step_list.blockSignals(False)
        if selected_row is not None and 0 <= selected_row < len(self.steps):
            self.step_list.setCurrentCell(selected_row, 0)
        else:
            self.step_list.clearSelection()
            self.step_list.setCurrentCell(-1, -1)
            self.load_selected_step(-1)
        self.prepared_signature = None
        self.pending_prepare = None
        self.clear_preview()

    def tick_sequence(self):
        s = self.exec_status
        fresh = s is not None and time.monotonic() - self.exec_status_at < 0.5
        summary = (
            ' → '.join(self.format_step(step) for step in self.steps)
            if self.steps
            else 'sin pasos'
        )
        self.list_summary.setText('Plan: ' + summary)
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
        for action in ('prepare', 'guided', 'execute'):
            self.buttons[action].setEnabled(not running)
        self.buttons['prepare'].setEnabled(
            not running
            and s.get('armed') is False
            and s.get('mode') == 'MANUAL'
            and bool(self.steps)
            and not (matching and s.get('validated', False))
        )
        self.buttons['execute'].setEnabled(
            matching
            and not running
            and s.get('validated', False)
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
        self.plan_editor.setEnabled(not running and s.get('armed') is False)
        self.step_list.setEnabled(not running and s.get('armed') is False)
        for entry in (
            self.sensor_depth,
            self.maximum_depth,
            self.radius,
            self.max_horizontal_speed,
            self.max_vertical_speed,
            self.max_yaw_speed,
        ) + self.controller_inputs:
            entry.setEnabled(not running and s.get('armed') is False)
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
            'COMPLETADO': 'success',
            'LISTO': 'success',
            'LISTO_GUIDED': 'info',
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
        states = workflow_states(status, fresh, bool(self.steps), matching)
        for name, state in states.items():
            set_widget_state(self.stage_labels[name], state)

    def update_guidance(self, status, fresh, matching):
        """Render one next action and explicit reasons for blocking it."""
        guide = operator_guidance(status, fresh, bool(self.steps), matching)
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
                'mode': ('MODO · —', 'neutral'),
                'armed': ('ARMADO · —', 'neutral'),
                'output': ('SALIDA · —', 'neutral'),
                'navigation': ('NAVEGACIÓN · —', 'neutral'),
                'dvl': ('DVL · —', 'neutral'),
            }
        else:
            armed = status.get('armed')
            enabled = status.get('enabled')
            navigation = status.get('navigation_ready')
            dvl = status.get('dvl_confidence')
            values = {
                'executor': ('EJECUTOR · ACTIVO', 'success'),
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
                    'warning' if enabled else 'success',
                ),
                'navigation': (
                    'NAVEGACIÓN · LISTA' if navigation else 'NAVEGACIÓN · NO LISTA',
                    'success' if navigation else 'warning',
                ),
                'dvl': (
                    f'DVL · {dvl if dvl is not None else "—"}',
                    'info' if dvl is not None else 'neutral',
                ),
            }
        for name, (text, state) in values.items():
            self.health_labels[name].setText(text)
            set_widget_state(self.health_labels[name], state)

    def closeEvent(self, event):
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

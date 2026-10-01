"""Operator-focused controls and live state for measured GUIDED sequences."""

import json
import math
import signal
import sys
import time
import uuid

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .pool_dashboard_node import PoolDashboard
from .sequence_executor_model import move_step, replace_step


class SequenceDashboard(PoolDashboard):
    def __init__(self, node):
        super().__init__(node)
        self.setWindowTitle('RobotX | Secuencias GUIDED')
        self.layout().itemAt(0).widget().setText('RobotX · Secuencias medidas · GUIDED')
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
        layout = QVBoxLayout(page)
        instructions = QLabel(
            '1. Editar plan y límites. 2. Preparar prueba (desarmado, MANUAL). '
            '3. Armado deliberado por el piloto. 4. Solicitar GUIDED. 5. Iniciar. '
            'Preparar nunca arma ni inicia movimientos. '
            'Esperar también puede accionar propulsores.'
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        self.phase_title = QLabel('SIN DATOS DEL EJECUTOR')
        self.phase_title.setStyleSheet(
            'font-size:20px; font-weight:bold; padding:10px; '
            'background:#5b4636; color:white; border-radius:4px;'
        )
        self.phase_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.phase_title)
        self.phase_detail = QLabel('Esperando estado de la secuencia')
        self.phase_detail.setWordWrap(True)
        self.phase_detail.setStyleSheet('font-size:15px; font-weight:bold; padding:4px;')
        layout.addWidget(self.phase_detail)
        row = QHBoxLayout()
        row.addWidget(QLabel('Profundidad conocida del sensor de presión [m]'))
        self.sensor_depth = QDoubleSpinBox()
        self.sensor_depth.setRange(0, 0.5)
        self.sensor_depth.setDecimals(2)
        self.sensor_depth.setSingleStep(0.01)
        row.addWidget(self.sensor_depth)
        layout.addLayout(row)
        note = QLabel(
            'El sensor de presión debe estar a la profundidad indicada y el DVL '
            'sumergido. Este cero es local a la sesión; no calibra ni cambia el autopiloto.'
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        row = QHBoxLayout()
        row.addWidget(QLabel('Límite profundidad [m]'))
        self.maximum_depth = QDoubleSpinBox()
        self.maximum_depth.setRange(0.2, 3)
        self.maximum_depth.setDecimals(2)
        self.maximum_depth.setSingleStep(0.01)
        self.maximum_depth.setKeyboardTracking(False)
        self.maximum_depth.setValue(0.2)
        row.addWidget(self.maximum_depth)
        row.addWidget(QLabel('Radio desde Home/inicio [m]'))
        self.radius = QDoubleSpinBox()
        self.radius.setRange(0.5, 5)
        self.radius.setDecimals(2)
        self.radius.setSingleStep(0.05)
        self.radius.setValue(0.5)
        row.addWidget(self.radius)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel('Máx. horizontal [m/s]'))
        self.max_horizontal_speed = QDoubleSpinBox()
        self.max_horizontal_speed.setRange(0.001, sys.float_info.max)
        self.max_horizontal_speed.setDecimals(3)
        self.max_horizontal_speed.setSingleStep(0.01)
        self.max_horizontal_speed.setValue(0.20)
        row.addWidget(self.max_horizontal_speed)
        row.addWidget(QLabel('Máx. vertical [m/s]'))
        self.max_vertical_speed = QDoubleSpinBox()
        self.max_vertical_speed.setRange(0.001, sys.float_info.max)
        self.max_vertical_speed.setDecimals(3)
        self.max_vertical_speed.setSingleStep(0.01)
        self.max_vertical_speed.setValue(0.10)
        row.addWidget(self.max_vertical_speed)
        row.addWidget(QLabel('Máx. giro [°/s]'))
        self.max_yaw_speed = QDoubleSpinBox()
        self.max_yaw_speed.setRange(0.001, sys.float_info.max)
        self.max_yaw_speed.setDecimals(3)
        self.max_yaw_speed.setSingleStep(1.0)
        self.max_yaw_speed.setValue(10.0)
        row.addWidget(self.max_yaw_speed)
        layout.addLayout(row)
        warning = QLabel(
            'Los límites son del controlador, no una barrera física. '
            'La comprobación en marcha añade 0.15 m de tolerancia. '
            'Las velocidades elegidas se autorizan para toda la ejecución. '
            'Deja margen para el tamaño del ROV y la parada.'
        )
        warning.setWordWrap(True)
        layout.addWidget(warning)
        # Reuse the existing editor and validation of individual steps.
        editor = QWidget()
        editor_row = QHBoxLayout(editor)
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
        self.secondary_label = QLabel('Velocidad máx. [m/s]')
        self.secondary = QDoubleSpinBox()
        self.secondary.setDecimals(3)
        editor_row.addWidget(self.kind)
        editor_row.addWidget(self.primary_label)
        editor_row.addWidget(self.amount)
        editor_row.addWidget(self.secondary_label)
        editor_row.addWidget(self.secondary)
        for title, callback, attribute in [
            ('Añadir', self.add_step, 'add_step_button'),
            ('Actualizar seleccionada', self.update_selected_step, 'update_step_button'),
            ('Eliminar seleccionada', self.remove_selected_step, 'remove_step_button'),
            ('Subir ↑', lambda: self.move_selected_step(-1), 'move_up_button'),
            ('Bajar ↓', lambda: self.move_selected_step(1), 'move_down_button'),
            ('Vaciar', self.clear_editable_steps, 'clear_steps_button'),
        ]:
            button = QPushButton(title)
            button.clicked.connect(callback)
            editor_row.addWidget(button)
            setattr(self, attribute, button)
        self.plan_editor = editor
        layout.addWidget(editor)
        self.kind.currentIndexChanged.connect(self.configure_step_editor)
        self.max_horizontal_speed.valueChanged.connect(self.configure_step_editor)
        self.max_vertical_speed.valueChanged.connect(self.configure_step_editor)
        self.max_yaw_speed.valueChanged.connect(self.configure_step_editor)
        self.configure_step_editor()
        queue_note = QLabel(
            'Haz clic en una orden para cargarla en el editor. Modifica el tipo o '
            'sus valores y pulsa “Actualizar seleccionada”. Usa Subir/Bajar para reordenar.'
        )
        queue_note.setWordWrap(True)
        layout.addWidget(queue_note)
        self.step_list = QListWidget()
        self.step_list.currentRowChanged.connect(self.load_selected_step)
        layout.addWidget(self.step_list)
        self.update_steps()
        self.list_summary = QLabel('Lista: sin pasos')
        self.list_summary.setWordWrap(True)
        layout.addWidget(self.list_summary)
        row = QHBoxLayout()
        row.addWidget(self.button('1 · Preparar prueba', 'prepare'))
        row.addWidget(self.button('Solicitar GUIDED', 'guided'))
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self.button('Iniciar plan preparado…', 'execute'))
        layout.addLayout(row)
        self.exec_text = QTextEdit()
        self.exec_text.setReadOnly(True)
        layout.addWidget(self.exec_text)
        self.tabs.addTab(page, 'Operación')
        self.tabs.setCurrentWidget(page)
        # Keep inherited widgets alive for its timers, but expose only the
        # operator pages used during a real launch.
        for index in range(self.tabs.count() - 1, -1, -1):
            if self.tabs.tabText(index) == 'Preflight · piscina':
                self.tabs.removeTab(index)
            elif self.tabs.tabText(index) == 'Secuencias · previsualización':
                self.tabs.removeTab(index)
        stop = self.button('MANUAL / CANCELAR · no desarma', 'manual')
        stop.setStyleSheet('background:#8b3030; color:white; font-weight:bold;')
        self.layout().addWidget(stop)
        self.layout().addWidget(
            QLabel(
                'DESARMAR: botón B del mando. '
                'Si no responde, killswitch físico. No dependas de esta ventana.'
            )
        )
        self.timer.timeout.connect(self.tick_sequence)

    def button(self, title, action):
        button = QPushButton(title)
        button.clicked.connect(lambda: self.send_action(action))
        self.buttons[action] = button
        return button

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

    def send_action(self, action):
        s = self.exec_status
        if not s or time.monotonic() - self.exec_status_at > 0.5:
            return
        if action == 'prepare':
            self.prepared_signature = None
            self.pending_prepare = (self.signature(), time.monotonic(), self.action_count)
        self.publish(
            self.request_pub,
            {
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
            },
        )
        self.action_count += 1

    def signature(self):
        return (
            json.dumps(self.steps, sort_keys=True),
            self.sensor_depth.value(),
            self.maximum_depth.value(),
            self.radius.value(),
            self.max_horizontal_speed.value(),
            self.max_vertical_speed.value(),
            self.max_yaw_speed.value(),
        )

    def configure_step_editor(self, *_):
        kind = self.kind.currentData()
        timed = kind in ('velocity_surge', 'velocity_sway', 'velocity_heave', 'velocity_yaw')
        self.secondary_label.setVisible(kind != 'wait')
        self.secondary.setVisible(kind != 'wait')
        if kind in ('advance', 'lateral'):
            self.primary_label.setText('Distancia [m] (+ adelante/derecha)')
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
        self.step_list.clear()
        self.step_list.addItems(
            [f'{index}. {self.format_step(step)}' for index, step in enumerate(self.steps, 1)]
        )
        self.step_list.blockSignals(False)
        if selected_row is not None and 0 <= selected_row < len(self.steps):
            self.step_list.setCurrentRow(selected_row)
        else:
            self.step_list.setCurrentRow(-1)
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
            self.exec_text.setPlainText(
                'Ejecutor sin datos recientes. No se puede iniciar movimiento.'
            )
            self.phase_title.setText('SIN DATOS DEL EJECUTOR')
            self.phase_detail.setText('No se conoce el estado del movimiento')
            self.phase_title.setStyleSheet(
                'font-size:20px; font-weight:bold; padding:10px; '
                'background:#8b3030; color:white; border-radius:4px;'
            )
            self.execution_status.setText('GUIDED: sin ejecutor reciente; inicio bloqueado')
            self.status.setText('Secuencias: esperando ejecutor y puente')
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
        for action in ('prepare', 'guided', 'execute'):
            self.buttons[action].setEnabled(not running)
        self.buttons['prepare'].setEnabled(
            not running
            and s.get('armed') is False
            and s.get('mode') == 'MANUAL'
            and bool(self.steps)
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
        ):
            entry.setEnabled(not running and s.get('armed') is False)
        output = 'HABILITADA' if s.get('enabled') else 'DESHABILITADA'
        self.status.setText(f'Salida real {output} · {s.get("mode")} · {s["state"]}')
        self.execution_status.setText(self.status.text() + ' · controles en Operación')
        phase = s.get('phase') or {
            'code': 'IDLE',
            'title': s['state'],
            'detail': s.get('note', ''),
        }
        colors = {
            'PAUSA': '#a65f00',
            'FALLA': '#8b3030',
            'BLOQUEADO': '#8b3030',
            'ESPERA': '#315f88',
            'AVANCE': '#167a50',
            'GIRO': '#167a50',
            'TEMPORAL': '#167a50',
            'PROFUNDIDAD': '#167a50',
            'RUMBO': '#167a50',
            'ESTABILIZANDO': '#315f88',
            'CORRIGIENDO': '#a65f00',
            'COMPLETADO': '#167a50',
            'LISTO': '#167a50',
            'LISTO_GUIDED': '#315f88',
            'PREPARADO': '#315f88',
            'IDLE': '#4b5661',
        }
        self.phase_title.setText(phase.get('title', s['state']))
        self.phase_detail.setText(phase.get('detail', ''))
        self.phase_title.setStyleSheet(
            'font-size:20px; font-weight:bold; padding:10px; '
            f'background:{colors.get(phase.get("code"), "#4b5661")}; '
            'color:white; border-radius:4px;'
        )
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

    def closeEvent(self, event):
        self.send_action('cancel')
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
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

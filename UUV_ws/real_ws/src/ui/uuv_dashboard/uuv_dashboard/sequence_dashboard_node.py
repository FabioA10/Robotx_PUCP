"""Explicit real-sequence controls alongside the existing geometric preview."""

import json
import math
import signal
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from python_qt_binding.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QHBoxLayout, QLabel,
    QMessageBox, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)

from .pool_dashboard_node import PoolDashboard


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
        self.request_pub = node.create_publisher(String, '/uuv/sequence/request', 10)
        self.alive_pub = node.create_publisher(String, '/uuv/sequence/ui_alive', 1)
        self.subscriptions.append(node.create_subscription(
            String, '/uuv/sequence/status', self.receive_status, 1))
        self.buttons = {}
        page = QWidget()
        layout = QVBoxLayout(page)
        instructions = QLabel(
            '1. Guardar referencia y Home desarmado. 2. Preparar lista en Secuencias. '
            '3. Validar. 4. Armado por el piloto en MANUAL. 5. Solicitar GUIDED. '
            '6. Ejecutar. RB o MANUAL interrumpen; B desarma.')
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        row = QHBoxLayout()
        row.addWidget(QLabel('Profundidad conocida del sensor de presión [m]'))
        self.sensor_depth = QDoubleSpinBox()
        self.sensor_depth.setRange(0, 0.5)
        self.sensor_depth.setDecimals(2)
        self.sensor_depth.setSingleStep(0.01)
        row.addWidget(self.sensor_depth)
        row.addWidget(self.button('Guardar superficie', 'surface'))
        row.addWidget(self.button('Guardar Home real', 'home'))
        layout.addLayout(row)
        note = QLabel(
            'El sensor de presión debe estar a la profundidad indicada y el DVL '
            'sumergido. Este cero es local a la sesión; no calibra ni cambia el autopiloto.')
        note.setWordWrap(True)
        layout.addWidget(note)
        row = QHBoxLayout()
        row.addWidget(QLabel('Límite profundidad [m]'))
        self.maximum_depth = QDoubleSpinBox()
        self.maximum_depth.setRange(0.2, 3)
        self.maximum_depth.setValue(1)
        row.addWidget(self.maximum_depth)
        row.addWidget(QLabel('Radio desde Home/inicio [m]'))
        self.radius = QDoubleSpinBox()
        self.radius.setRange(0.5, 5)
        self.radius.setValue(2)
        row.addWidget(self.radius)
        layout.addLayout(row)
        self.checked_limits = QCheckBox(
            'He ajustado estos límites al espacio libre de la piscina y comprobado recuperación y mando')
        layout.addWidget(self.checked_limits)
        self.list_summary = QLabel('Lista: sin pasos')
        self.list_summary.setWordWrap(True)
        layout.addWidget(self.list_summary)
        row = QHBoxLayout()
        row.addWidget(self.button('Validar secuencia', 'validate'))
        row.addWidget(self.button('Validar regreso a Home', 'validate_home'))
        row.addWidget(self.button('Solicitar GUIDED', 'guided'))
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(self.button('Ejecutar validación actual', 'execute'))
        row.addWidget(self.button('Cancelar secuencia', 'cancel'))
        row.addWidget(self.button('MANUAL · cancelar y entregar mando', 'manual'))
        layout.addLayout(row)
        self.exec_text = QTextEdit()
        self.exec_text.setReadOnly(True)
        layout.addWidget(self.exec_text)
        self.tabs.addTab(page, 'Ejecución real')
        self.tabs.setCurrentWidget(page)
        self.timer.timeout.connect(self.tick_sequence)

    def button(self, title, action):
        button = QPushButton(title)
        button.clicked.connect(lambda: self.send_action(action))
        self.buttons[action] = button
        return button

    def validate_sequence(self):
        # The inherited dry-run button must not publish the old request format.
        self.tabs.setCurrentIndex(self.tabs.count() - 1)
        self.send_action('validate')

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
        if action == 'execute':
            kind = 'REGRESO A HOME' if s.get('returning') else 'SECUENCIA'
            answer = QMessageBox.question(
                self, 'Confirmar movimiento real',
                f'Ejecutar {kind} desde la posición actual. El ROV se moverá. '
                '¿Confirmas inicio con el área libre?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return
            s = self.exec_status
            if not s or time.monotonic() - self.exec_status_at > 0.5:
                return
        self.publish(self.request_pub, {
            'session': s['session'], 'ticket': s['ticket'], 'ui': self.ui_id,
            'serial': self.action_count, 'action': action, 'steps': self.steps,
            'sensor_depth': self.sensor_depth.value(),
            'max_depth': self.maximum_depth.value(), 'radius': self.radius.value(),
            'limits_confirmed': self.checked_limits.isChecked(),
        })
        self.action_count += 1

    def tick_sequence(self):
        s = self.exec_status
        fresh = s is not None and time.monotonic() - self.exec_status_at < 0.5
        self.list_summary.setText('Lista de la pestaña Secuencias: ' + (
            ' → '.join(f'{k} {v:g}' for k, v in self.steps) if self.steps else 'sin pasos'))
        for button in self.buttons.values():
            button.setEnabled(fresh)
        if not fresh:
            self.exec_text.setPlainText('Ejecutor sin datos recientes. No se puede iniciar movimiento.')
            self.execution_status.setText('GUIDED: sin ejecutor reciente; inicio bloqueado')
            self.status.setText('Secuencias: esperando ejecutor y puente')
            return
        self.publish(self.alive_pub, {'session': s['session'], 'ticket': s['ticket'],
                                     'ui': self.ui_id, 'serial': self.ui_count})
        self.ui_count += 1
        running = s['state'] == 'RUNNING'
        for action in ('surface', 'home', 'validate', 'validate_home', 'guided', 'execute'):
            self.buttons[action].setEnabled(not running)
        self.buttons['execute'].setEnabled(
            not running and s.get('validated', False) and s.get('enabled', False)
            and s.get('armed') is True and s.get('mode') == 'GUIDED' and not s.get('blockers'))
        self.buttons['guided'].setEnabled(not running and not s.get('blockers'))
        for entry in (self.sensor_depth, self.maximum_depth, self.radius, self.checked_limits):
            entry.setEnabled(not running)
        output = 'HABILITADA' if s.get('enabled') else 'DESHABILITADA'
        self.status.setText(f'Salida real {output} · {s.get("mode")} · {s["state"]}')
        self.execution_status.setText(self.status.text() + ' · controles en Ejecución real')
        lines = [f'Salida real: {output}', f'Estado: {s["state"]} · Modo: {s.get("mode")} · Armado: {s.get("armed")}',
                 s.get('note', ''), f'Referencia superficie: {s.get("surface")} · Home: {s.get("home")}',
                 f'Confianza del mensaje DVL: {s.get("dvl_confidence")}',
                 f'Último estado del puente: {s.get("bridge_reason")}']
        pose = s.get('pose')
        if pose:
            lines.append(f'Actual: N={pose["n"]:.2f}, E={pose["e"]:.2f}, '
                         f'prof={pose["depth"]:.2f} m, yaw={math.degrees(pose["yaw"]):.1f}°')
        if s.get('goal'):
            goal = s['goal']
            errors = s['errors']
            lines += [f'Paso {s["step"]}/{s["total"]} · Objetivo {goal}',
                      f'Error XY {errors[0]:.2f} m · profundidad {errors[1]:.2f} m · rumbo {errors[2]:.1f}°',
                      f'Tiempo máximo restante: {s["timeout_left"]:.1f} s']
        if s.get('nav_error'):
            lines.append('Navegación: ' + s['nav_error'])
        if s.get('blockers'):
            lines.append('Bloqueos:\n' + '\n'.join('• ' + b for b in s['blockers']))
        if s.get('stopping'):
            lines.append('Parada solicitada; esperando modo ' + s['stopping'])
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

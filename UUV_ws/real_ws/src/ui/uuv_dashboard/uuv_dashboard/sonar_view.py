"""Qt polar view for live Ping360 PointCloud2 messages."""

import math
import time

from python_qt_binding.QtCore import QPointF, Qt
from python_qt_binding.QtGui import QColor, QPainter, QPen, QPolygonF
from python_qt_binding.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from .sonar_model import decode_xy_intensity


class SonarCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []
        self.range_m = 5.0
        self.min_intensity = 20.0
        self.age_s = None
        self.persistence_s = 1.5
        self.setMinimumSize(520, 420)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.fillRect(self.rect(), QColor('#071018'))

        margin = 34.0
        diameter = max(10.0, min(self.width(), self.height()) - 2.0 * margin)
        radius_px = diameter / 2.0
        center = QPointF(self.width() / 2.0, self.height() / 2.0)

        painter.setPen(QPen(QColor('#31505f'), 1))
        for ring in range(1, 6):
            ring_radius = radius_px * ring / 5.0
            painter.drawEllipse(center, ring_radius, ring_radius)
            painter.drawText(
                int(center.x() + 5),
                int(center.y() - ring_radius + 15),
                f'{self.range_m * ring / 5.0:.1f} m',
            )
        painter.drawLine(
            QPointF(center.x() - radius_px, center.y()),
            QPointF(center.x() + radius_px, center.y()),
        )
        painter.drawLine(
            QPointF(center.x(), center.y() - radius_px),
            QPointF(center.x(), center.y() + radius_px),
        )
        painter.setPen(QColor('#9fcbd4'))
        painter.drawText(int(center.x() - 24), 20, 'FRENTE')

        if not self.points or self.age_s is None or self.age_s > self.persistence_s:
            painter.setPen(QColor('#b9c3c9'))
            painter.drawText(self.rect(), Qt.AlignCenter, 'Esperando mediciones Ping360…')
            painter.end()
            return

        scale = radius_px / max(0.01, self.range_m)
        usable = [
            point
            for point in self.points
            if point[2] >= self.min_intensity and math.hypot(point[0], point[1]) <= self.range_m
        ]
        if not usable:
            painter.setPen(QColor('#b9c3c9'))
            painter.drawText(
                self.rect(), Qt.AlignCenter, 'Sin retornos con el umbral seleccionado'
            )
            painter.end()
            return

        maximum = max(point[2] for point in usable)
        denominator = max(1.0, maximum - self.min_intensity)
        buckets = [[] for _ in range(16)]
        for x, y, intensity in usable:
            normalized = max(0.0, min(1.0, (intensity - self.min_intensity) / denominator))
            bucket = min(15, int(normalized * 15.0))
            # ROS: +X hacia delante y +Y a babor; frente se dibuja arriba.
            buckets[bucket].append(QPointF(center.x() - y * scale, center.y() - x * scale))

        for index, screen_points in enumerate(buckets):
            if not screen_points:
                continue
            hue = int(220.0 * (1.0 - index / 15.0))
            painter.setPen(QPen(QColor.fromHsv(hue, 255, 255), 2))
            painter.drawPoints(QPolygonF(screen_points))
        painter.end()


class SonarView(QWidget):
    """Controls, liveness state and drawing for raw/filtered sonar clouds."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.frames = {}
        self._last_refresh = 0.0

        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        controls.addWidget(QLabel('Fuente'))
        self.source = QComboBox()
        self.source.addItem('Filtrada', 'filtered')
        self.source.addItem('Cruda', 'raw')
        controls.addWidget(self.source)

        controls.addWidget(QLabel('Alcance visible [m]'))
        self.range_m = QDoubleSpinBox()
        self.range_m.setRange(0.5, 50.0)
        self.range_m.setDecimals(1)
        self.range_m.setSingleStep(0.5)
        self.range_m.setValue(5.0)
        controls.addWidget(self.range_m)

        controls.addWidget(QLabel('Persistencia [s]'))
        self.persistence = QDoubleSpinBox()
        self.persistence.setRange(0.2, 10.0)
        self.persistence.setDecimals(1)
        self.persistence.setSingleStep(0.5)
        self.persistence.setValue(1.5)
        controls.addWidget(self.persistence)

        controls.addWidget(QLabel('Intensidad mínima'))
        self.min_intensity = QDoubleSpinBox()
        self.min_intensity.setRange(0.0, 255.0)
        self.min_intensity.setDecimals(0)
        self.min_intensity.setValue(20.0)
        controls.addWidget(self.min_intensity)
        layout.addLayout(controls)

        self.status = QLabel('Ping360: esperando publicador…')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.canvas = SonarCanvas(self)
        layout.addWidget(self.canvas)

        self.source.currentIndexChanged.connect(self.refresh)
        self.range_m.valueChanged.connect(self.refresh)
        self.persistence.valueChanged.connect(self.refresh)
        self.min_intensity.valueChanged.connect(self.refresh)

    def receive(self, source, msg):
        if source != self.source.currentData():
            return
        self.frames[source] = (decode_xy_intensity(msg), time.monotonic())

    def refresh(self, *_args):
        now = time.monotonic()
        if now - self._last_refresh < 0.08:
            return
        self._last_refresh = now
        selected = self.source.currentData()
        frame = self.frames.get(selected)
        self.canvas.range_m = self.range_m.value()
        self.canvas.persistence_s = self.persistence.value()
        self.canvas.min_intensity = self.min_intensity.value()

        if frame is None:
            self.canvas.points = []
            self.canvas.age_s = None
            self.status.setText(
                f'Ping360: sin mensajes de la fuente {self.source.currentText().lower()}'
            )
        else:
            points, received_at = frame
            age = now - received_at
            self.canvas.points = points
            self.canvas.age_s = age
            if age <= self.persistence.value():
                self.status.setText(
                    f'Ping360 activo · {len(points)} puntos dibujables · '
                    f'último mensaje hace {age:.1f} s'
                )
            else:
                self.status.setText(
                    f'Ping360 sin datos recientes · última recepción hace {age:.1f} s'
                )
        self.canvas.update()

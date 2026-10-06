"""Shared control-room visual language for the RobotX operator dashboards."""

import qdarktheme


CONTROL_ROOM_QSS = """
QWidget {
    color: #e6edf5;
    font-family: "Noto Sans", "DejaVu Sans", sans-serif;
    font-size: 13px;
}

QWidget#RobotXRoot {
    background: #08111f;
}

QLabel#AppTitle {
    color: #f8fafc;
    font-size: 24px;
    font-weight: 700;
    padding: 2px 0 6px 2px;
}

QLabel#Subtitle,
QLabel#MutedText {
    color: #93a4b8;
    padding: 2px;
}

QLabel#FieldLabel {
    background: transparent;
    color: #c5d4e3;
    padding: 2px;
}

QLabel#SectionTitle {
    color: #7dd3fc;
    font-size: 15px;
    font-weight: 700;
    padding: 2px 0 6px 0;
}

QFrame#PanelCard,
QGroupBox {
    background: #0f1c2d;
    border: 1px solid #20344d;
    border-radius: 10px;
}

QGroupBox {
    margin-top: 12px;
    padding: 14px 10px 10px 10px;
    font-weight: 700;
    color: #c9e7f6;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}

QFrame#TopBar {
    background: #0c192a;
    border: 1px solid #1f3855;
    border-radius: 10px;
}

QLabel#StatusBanner {
    background: #26364a;
    border: 1px solid #3d526d;
    border-radius: 8px;
    color: #f8fafc;
    font-size: 18px;
    font-weight: 700;
    padding: 12px;
}

QLabel#StatusBanner[state="success"] {
    background: #0f513b;
    border-color: #1b8a65;
}

QLabel#StatusBanner[state="info"] {
    background: #123f67;
    border-color: #2b77b7;
}

QLabel#StatusBanner[state="warning"] {
    background: #6a4305;
    border-color: #b57912;
}

QLabel#StatusBanner[state="danger"] {
    background: #6c2028;
    border-color: #b94652;
}

QLabel#StatusBanner[state="neutral"] {
    background: #26364a;
    border-color: #3d526d;
}

QLabel#GlobalStatus {
    background: #1b2b3d;
    border: 1px solid #31475f;
    border-radius: 6px;
    color: #d9e6f2;
    font-weight: 700;
    padding: 7px 10px;
}

QLabel#GlobalStatus[state="success"] { border-color: #1b8a65; color: #5ee6a8; }
QLabel#GlobalStatus[state="info"] { border-color: #2b77b7; color: #7dd3fc; }
QLabel#GlobalStatus[state="warning"] { border-color: #b57912; color: #ffd166; }
QLabel#GlobalStatus[state="danger"] {
    background: #4e1d24;
    border-color: #b94652;
    color: #ff9aa3;
}

QLabel#TelemetryValue {
    background: #0a1524;
    border: 1px solid #1c3149;
    border-radius: 6px;
    color: #dce8f5;
    font-weight: 600;
    padding: 7px 9px;
}

QLabel#TelemetryValue[state="success"] { color: #5ee6a8; border-color: #24684f; }
QLabel#TelemetryValue[state="warning"] { color: #ffd166; border-color: #785a1a; }
QLabel#TelemetryValue[state="danger"] { color: #ff7b86; border-color: #7d3038; }
QLabel#TelemetryValue[state="neutral"] { color: #aebdcd; }

QLabel#StagePill {
    background: #13253a;
    border: 1px solid #284661;
    border-radius: 7px;
    color: #b9d8ea;
    font-weight: 700;
    padding: 8px 10px;
}

QLabel#StagePill[state="active"] {
    background: #164e75;
    border-color: #38bdf8;
    color: #e0f2fe;
}

QLabel#StagePill[state="done"] {
    background: #14532d;
    border-color: #34d399;
    color: #d1fae5;
}

QLabel#StagePill[state="blocked"] {
    background: #422006;
    border-color: #d97706;
    color: #fde68a;
}

QLabel#HealthChip {
    background: #0a1524;
    border: 1px solid #294158;
    border-radius: 6px;
    color: #aebdcd;
    font-weight: 700;
    padding: 6px 8px;
}

QLabel#HealthChip[state="success"] { color: #5ee6a8; border-color: #24684f; }
QLabel#HealthChip[state="info"] { color: #7dd3fc; border-color: #285f86; }
QLabel#HealthChip[state="warning"] { color: #ffd166; border-color: #785a1a; }
QLabel#HealthChip[state="danger"] { color: #ff7b86; border-color: #7d3038; }
QLabel#HealthChip[state="neutral"] { color: #aebdcd; }

QLabel#NextAction {
    background: #123f67;
    border: 1px solid #2b77b7;
    border-radius: 7px;
    color: #e5f4ff;
    font-size: 15px;
    font-weight: 700;
    padding: 9px;
}

QLabel#NextAction[state="success"] { background: #0f513b; border-color: #1b8a65; }
QLabel#NextAction[state="warning"] { background: #6a4305; border-color: #b57912; }
QLabel#NextAction[state="danger"] { background: #6c2028; border-color: #b94652; }
QLabel#NextAction[state="info"] { background: #123f67; border-color: #2b77b7; }

QLabel#BlockerText {
    background: #0a1524;
    border-left: 3px solid #d97706;
    color: #c7d2de;
    padding: 7px;
}

QLabel#SafetyNote {
    background: #2a1d08;
    border: 1px solid #7a5312;
    border-radius: 6px;
    color: #f6d78b;
    font-weight: 600;
    padding: 7px;
}

QLabel#VideoSurface {
    background: #050b13;
    border: 1px solid #294158;
    border-radius: 8px;
    color: #8fa6ba;
    font-size: 15px;
    font-weight: 600;
}

QPushButton {
    min-height: 30px;
    background: #172a40;
    border: 1px solid #2b4965;
    border-radius: 7px;
    color: #e8f2fa;
    font-weight: 600;
    padding: 6px 12px;
}

QPushButton:hover { background: #203a56; border-color: #4f7da3; }
QPushButton:pressed { background: #0f2236; }
QPushButton:disabled { color: #64748b; background: #101b29; border-color: #1d2b3b; }

QPushButton#PrimaryAction {
    background: #155f8f;
    border-color: #238ecb;
}

QPushButton#GuidedAction {
    background: #185f61;
    border-color: #2c9b95;
}

QPushButton#SuccessAction {
    background: #176342;
    border-color: #2a9d68;
}

QPushButton#DangerAction {
    min-height: 42px;
    background: #7f1d2d;
    border: 2px solid #dc4c64;
    color: white;
    font-size: 15px;
    font-weight: 800;
}

QPushButton#DangerAction:hover { background: #a52a3d; }

QPushButton#DangerAction:disabled {
    background: #1a202b;
    border-color: #394354;
    color: #788698;
}

QPushButton#ClearPlanAction {
    background: #271820;
    border-color: #7d3038;
    color: #ff9aa3;
}

QToolButton#TechnicalToggle {
    background: transparent;
    border: 1px solid #294158;
    border-radius: 6px;
    color: #7dd3fc;
    font-weight: 700;
    padding: 7px;
}

QToolButton#TechnicalToggle:hover { background: #14283c; }

QPushButton#PrimaryAction:disabled,
QPushButton#GuidedAction:disabled,
QPushButton#SuccessAction:disabled {
    background: #101b29;
    border-color: #26384b;
    color: #64748b;
}

QComboBox,
QDoubleSpinBox,
QListWidget,
QTableWidget,
QTextEdit {
    background: #0a1524;
    border: 1px solid #294158;
    border-radius: 6px;
    color: #e5eef7;
    padding: 6px;
    selection-background-color: #1e6f9f;
}

QComboBox:focus,
QDoubleSpinBox:focus,
QListWidget:focus,
QTableWidget:focus,
QTextEdit:focus {
    border: 1px solid #38a8df;
}

QListWidget::item { padding: 8px; border-bottom: 1px solid #192b3f; }
QListWidget::item:selected { background: #174c70; border-radius: 4px; }

QTableWidget {
    gridline-color: #20344d;
    alternate-background-color: #0d1a2a;
}

QTableWidget::item { padding: 5px; }
QTableWidget::item:selected { background: #174c70; }

QHeaderView::section {
    background: #15283d;
    border: 0;
    border-right: 1px solid #294158;
    color: #b9d8ea;
    font-weight: 700;
    padding: 6px;
}

QTabWidget::pane {
    border: 1px solid #1f354d;
    border-radius: 8px;
    background: #0b1726;
    top: -1px;
}

QTabBar::tab {
    background: #101f31;
    border: 1px solid #1f354d;
    color: #91a8bb;
    padding: 9px 16px;
    margin-right: 2px;
}

QTabBar::tab:selected {
    background: #17334d;
    color: #e9f6fd;
    border-bottom: 2px solid #38bdf8;
}

QSplitter::handle { background: #172a40; width: 6px; }
QScrollArea { background: transparent; border: none; }
QScrollBar:vertical { background: #0b1726; width: 11px; margin: 0; }
QScrollBar::handle:vertical { background: #2a4863; border-radius: 5px; min-height: 28px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""


def apply_theme(app):
    """Apply the common dark base and the RobotX control-room overrides."""
    app.setStyle('Fusion')
    app.setStyleSheet(qdarktheme.load_stylesheet('dark') + CONTROL_ROOM_QSS)


def set_widget_state(widget, state):
    """Update a dynamic visual state and immediately refresh its stylesheet."""
    if widget.property('state') == state:
        return
    widget.setProperty('state', state)
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()

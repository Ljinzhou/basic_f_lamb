#!/usr/bin/env python3
from __future__ import annotations

import csv
import sys
import time
from pathlib import Path
from typing import List, Optional

from debug_backend import SystemSnapshot, export_snapshots_to_csv, flatten_snapshot, snapshot_to_text

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("未安装 PyQt5，请执行: pip install PyQt5") from exc


class PollWorker(QtCore.QObject):
    snapshot_ready = QtCore.pyqtSignal(object)
    error = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, backend, interval_ms: int = 200):
        super().__init__()
        self.backend = backend
        self.interval_ms = interval_ms
        self._running = False

    @QtCore.pyqtSlot()
    def run(self) -> None:
        self._running = True
        while self._running:
            try:
                snapshot = self.backend.read_snapshot()
                self.snapshot_ready.emit(snapshot)
            except Exception as exc:  # pragma: no cover
                self.error.emit(str(exc))
            QtCore.QThread.msleep(self.interval_ms)
        self.finished.emit()

    def stop(self) -> None:
        self._running = False


class HistoryPlotWidget(QtWidgets.QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.series: List[List[float]] = [[], [], []]
        self.colors = [QtGui.QColor("#d1495b"), QtGui.QColor("#00798c"), QtGui.QColor("#edae49")]
        self.setMinimumHeight(180)

    def append_values(self, values) -> None:
        for index, value in enumerate(values[:3]):
            self.series[index].append(float(value))
            if len(self.series[index]) > 120:
                self.series[index] = self.series[index][-120:]
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#111827"))
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QColor("#f3f4f6"))
        painter.drawText(12, 20, self.title)
        plot = self.rect().adjusted(12, 32, -12, -12)
        painter.setPen(QtGui.QColor("#374151"))
        painter.drawRect(plot)
        merged = [value for values in self.series for value in values]
        if not merged:
            painter.drawText(plot, QtCore.Qt.AlignCenter, "No data")
            return
        minimum = min(merged)
        maximum = max(merged)
        if abs(maximum - minimum) < 1e-6:
            maximum += 1.0
            minimum -= 1.0
        for color, values in zip(self.colors, self.series):
            if len(values) < 2:
                continue
            painter.setPen(QtGui.QPen(color, 2))
            path = QtGui.QPainterPath()
            for index, value in enumerate(values):
                x = plot.left() + index * plot.width() / max(len(values) - 1, 1)
                y_ratio = (value - minimum) / (maximum - minimum)
                y = plot.bottom() - y_ratio * plot.height()
                if index == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            painter.drawPath(path)


class DebugMainWindow(QtWidgets.QMainWindow):
    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.snapshots: List[SystemSnapshot] = []
        self.worker_thread: Optional[QtCore.QThread] = None
        self.worker: Optional[PollWorker] = None
        self.setWindowTitle("Robot Debug Reader")
        self.resize(1400, 900)
        self._build_ui()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)

        toolbar = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("开始采集")
        self.stop_button = QtWidgets.QPushButton("停止采集")
        self.export_button = QtWidgets.QPushButton("导出 CSV")
        self.load_button = QtWidgets.QPushButton("加载回放")
        self.filter_edit = QtWidgets.QLineEdit()
        self.filter_edit.setPlaceholderText("过滤/搜索日志")
        toolbar.addWidget(self.start_button)
        toolbar.addWidget(self.stop_button)
        toolbar.addWidget(self.export_button)
        toolbar.addWidget(self.load_button)
        toolbar.addWidget(self.filter_edit, 1)
        layout.addLayout(toolbar)

        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self.overview_text = QtWidgets.QPlainTextEdit()
        self.overview_text.setReadOnly(True)
        self.tabs.addTab(self.overview_text, "系统总览")

        self.motor_table = QtWidgets.QTableWidget(0, 9)
        self.motor_table.setHorizontalHeaderLabels(["名称", "类型", "在线", "使能", "编码器", "角度", "速度", "电流/力矩", "温度"])
        self.motor_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tabs.addTab(self.motor_table, "电机状态")

        gimbal_widget = QtWidgets.QWidget()
        gimbal_layout = QtWidgets.QVBoxLayout(gimbal_widget)
        self.gimbal_label = QtWidgets.QLabel("云台状态")
        self.gimbal_plot = HistoryPlotWidget("Gimbal Yaw / Pitch / YawTotal")
        gimbal_layout.addWidget(self.gimbal_label)
        gimbal_layout.addWidget(self.gimbal_plot)
        self.tabs.addTab(gimbal_widget, "云台")

        imu_widget = QtWidgets.QWidget()
        imu_layout = QtWidgets.QVBoxLayout(imu_widget)
        self.imu_label = QtWidgets.QLabel("IMU 状态")
        self.imu_plot = HistoryPlotWidget("IMU Roll / Pitch / Yaw")
        imu_layout.addWidget(self.imu_label)
        imu_layout.addWidget(self.imu_plot)
        self.tabs.addTab(imu_widget, "IMU")

        self.log_table = QtWidgets.QTableWidget(0, 5)
        self.log_table.setHorizontalHeaderLabels(["时间", "机器人状态", "云台", "IMU", "备注"])
        self.log_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tabs.addTab(self.log_table, "日志/回放")

        self.statusBar().showMessage("就绪")
        self.start_button.clicked.connect(self.start_polling)
        self.stop_button.clicked.connect(self.stop_polling)
        self.export_button.clicked.connect(self.export_logs)
        self.load_button.clicked.connect(self.load_replay)
        self.filter_edit.textChanged.connect(self.apply_filter)

    def start_polling(self) -> None:
        if self.worker_thread is not None:
            return
        self.worker_thread = QtCore.QThread(self)
        self.worker = PollWorker(self.backend)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run)
        self.worker.snapshot_ready.connect(self.handle_snapshot)
        self.worker.error.connect(self.handle_error)
        self.worker.finished.connect(self.worker_thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker_thread.finished.connect(self._cleanup_worker)
        self.worker_thread.start()
        self.statusBar().showMessage("采集中")

    def stop_polling(self) -> None:
        if self.worker:
            self.worker.stop()

    def _cleanup_worker(self) -> None:
        if self.worker_thread:
            self.worker_thread.deleteLater()
        self.worker_thread = None
        self.worker = None
        self.statusBar().showMessage("已停止")

    def closeEvent(self, event) -> None:  # noqa: N802
        self.stop_polling()
        if self.worker_thread:
            self.worker_thread.quit()
            self.worker_thread.wait(1000)
        super().closeEvent(event)

    def handle_error(self, message: str) -> None:
        self.statusBar().showMessage(message)

    def handle_snapshot(self, snapshot: SystemSnapshot) -> None:
        self.snapshots.append(snapshot)
        if len(self.snapshots) > 2000:
            self.snapshots = self.snapshots[-2000:]
        self.overview_text.setPlainText(snapshot_to_text(snapshot))
        self.gimbal_label.setText(
            f"mode={snapshot.gimbal.mode} yaw={snapshot.gimbal.yaw:.2f} pitch={snapshot.gimbal.pitch:.2f} yaw_total={snapshot.gimbal.yaw_total:.2f} yaw_ecd={snapshot.gimbal.yaw_ecd} pitch_ecd={snapshot.gimbal.pitch_ecd}"
        )
        self.imu_label.setText(
            f"roll={snapshot.imu.roll:.2f} pitch={snapshot.imu.pitch:.2f} yaw={snapshot.imu.yaw:.2f} q={snapshot.imu.quaternion} gyro={snapshot.imu.gyro} accel={snapshot.imu.accel}"
        )
        self.gimbal_plot.append_values((snapshot.gimbal.yaw, snapshot.gimbal.pitch, snapshot.gimbal.yaw_total))
        self.imu_plot.append_values((snapshot.imu.roll, snapshot.imu.pitch, snapshot.imu.yaw))
        self._update_motor_table(snapshot)
        self._append_log(snapshot)
        self.apply_filter()

    def _update_motor_table(self, snapshot: SystemSnapshot) -> None:
        self.motor_table.setRowCount(len(snapshot.motors))
        for row, motor in enumerate(snapshot.motors):
            values = [
                motor.name,
                motor.motor_type,
                "在线" if motor.online else "离线",
                "使能" if motor.enabled else "停机",
                str(motor.ecd),
                f"{motor.angle_deg:.2f}",
                f"{motor.speed_deg_s:.2f}",
                f"{motor.current:.2f}",
                f"{motor.temperature:.2f}",
            ]
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                if column == 2:
                    item.setForeground(QtGui.QColor("#10b981" if motor.online else "#ef4444"))
                self.motor_table.setItem(row, column, item)

    def _append_log(self, snapshot: SystemSnapshot) -> None:
        row = self.log_table.rowCount()
        self.log_table.insertRow(row)
        values = [
            time.strftime("%H:%M:%S", time.localtime(snapshot.timestamp)),
            str(snapshot.robot_state),
            f"yaw={snapshot.gimbal.yaw:.1f} pitch={snapshot.gimbal.pitch:.1f}",
            f"r={snapshot.imu.roll:.1f} p={snapshot.imu.pitch:.1f} y={snapshot.imu.yaw:.1f}",
            "; ".join(snapshot.notes),
        ]
        for column, value in enumerate(values):
            self.log_table.setItem(row, column, QtWidgets.QTableWidgetItem(value))

    def apply_filter(self) -> None:
        keyword = self.filter_edit.text().strip().lower()
        for row in range(self.log_table.rowCount()):
            visible = not keyword
            if keyword:
                for column in range(self.log_table.columnCount()):
                    item = self.log_table.item(row, column)
                    if item and keyword in item.text().lower():
                        visible = True
                        break
            self.log_table.setRowHidden(row, not visible)

    def export_logs(self) -> None:
        file_name, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出日志", str(Path.cwd() / "debug_log.csv"), "CSV Files (*.csv)")
        if not file_name:
            return
        export_snapshots_to_csv(Path(file_name), self.snapshots)
        self.statusBar().showMessage(f"已导出: {file_name}")

    def load_replay(self) -> None:
        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(self, "加载回放日志", str(Path.cwd()), "CSV Files (*.csv)")
        if not file_name:
            return
        with Path(file_name).open("r", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        self.log_table.setRowCount(0)
        for row_data in rows:
            row = self.log_table.rowCount()
            self.log_table.insertRow(row)
            values = [
                row_data.get("timestamp", ""),
                row_data.get("robot_state", ""),
                f"yaw={row_data.get('gimbal_yaw', '')} pitch={row_data.get('gimbal_pitch', '')}",
                f"r={row_data.get('imu_roll', '')} p={row_data.get('imu_pitch', '')} y={row_data.get('imu_yaw', '')}",
                "replay",
            ]
            for column, value in enumerate(values):
                self.log_table.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        self.statusBar().showMessage(f"已加载回放: {file_name}")


def launch_gui(backend) -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = DebugMainWindow(backend)
    window.show()
    return app.exec_()

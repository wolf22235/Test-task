import sys
import datetime
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QDoubleSpinBox, QPushButton, QTextEdit, QGroupBox
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPainter, QColor, QBrush, QPen
import matplotlib

matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from src.model import TankModel
from src.pid import PIDController


class MnemonicWidget(QWidget):
    """Кастомный графический виджет динамической мнемосхемы резервуара"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250, 250)
        self.H = 0.0
        self.H_max = 10.0
        self.Q_in = 0.0
        self.Q_out = 0.0
        self.valve_open = 0.0

    def update_state(self, H, H_max, Q_in, Q_out, valve_open):
        self.H = H
        self.H_max = H_max
        self.Q_in = Q_in
        self.Q_out = Q_out
        self.valve_open = valve_open
        self.update()  # Вызывает перерисовку (paintEvent)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        tank_w, tank_h = 120, 160
        tank_x = (w - tank_w) // 2
        tank_y = (h - tank_h) // 2 + 10

        # Геометрия стенок бака
        painter.setPen(QPen(QColor(80, 80, 80), 3))
        painter.setBrush(QBrush(QColor(245, 245, 245)))
        painter.drawRect(tank_x, tank_y, tank_w, tank_h)

        # Отрисовка столба жидкости (синий цвет с прозрачностью)
        fill_ratio = self.H / self.H_max if self.H_max > 0 else 0
        fill_h = int(tank_h * fill_ratio)
        if fill_h > 0:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor(41, 128, 185, 200)))
            painter.drawRect(tank_x + 2, tank_y + tank_h - fill_h, tank_w - 3, fill_h)

        # Текстовые метки
        painter.setPen(QPen(Qt.GlobalColor.black))
        painter.drawText(tank_x + 20, tank_y + tank_h // 2, f"H = {self.H:.2f} м")

        # Стрелка/Линия притока (Зеленый)
        painter.setPen(QPen(QColor(39, 174, 96), 3))
        painter.drawLine(tank_x - 40, tank_y + 30, tank_x, tank_y + 30)
        painter.drawText(tank_x - 55, tank_y + 20, f"Qin: {self.Q_in:.1f}")

        # Стрелка/Линия слива (Красный)
        painter.setPen(QPen(QColor(192, 57, 43), 3))
        painter.drawLine(tank_x + tank_w, tank_y + tank_h - 30, tank_x + tank_w + 40, tank_y + tank_h - 30)
        painter.drawText(tank_x + tank_w + 5, tank_y + tank_h - 45, f"Qout: {self.Q_out:.2f}")
        painter.drawText(tank_x + tank_w + 5, tank_y + tank_h - 15, f"Клапан: {self.valve_open:.1f}%")


class TrendCanvas(FigureCanvas):
    """Виджет реального времени для графиков на базе Matplotlib"""

    def __init__(self, parent=None, width=5, height=3, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        self.axes.set_title("Тренды уровня жидкости (PV / SP)")
        self.axes.set_xlabel("Шаги дискретизации (x0.1 с)")
        self.axes.set_ylabel("Высота уровня, м")
        self.axes.grid(True)

        self.time_data, self.h_data, self.sp_data = [], [], []
        self.line_h, = self.axes.plot([], [], 'b-', linewidth=2, label='Текущий уровень (PV)')
        self.line_sp, = self.axes.plot([], [], 'r--', linewidth=1.5, label='Уставка (SP)')
        self.axes.legend(loc='upper right')

    def add_point(self, t, h, sp):
        self.time_data.append(t)
        self.h_data.append(h)
        self.sp_data.append(sp)

        # Ограничиваем буфер отображения до 200 точек для предотвращения утечек памяти
        if len(self.time_data) > 200:
            self.time_data.pop(0)
            self.h_data.pop(0)
            self.sp_data.pop(0)

        self.line_h.set_data(self.time_data, self.h_data)
        self.line_sp.set_data(self.time_data, self.sp_data)
        self.axes.relim()
        self.axes.autoscale_view()
        self.draw()

    def clear_charts(self):
        self.time_data.clear()
        self.h_data.clear()
        self.sp_data.clear()
        self.line_h.set_data([], [])
        self.line_sp.set_data([], [])
        self.draw()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Имитационная модель резервуара с ПИД-регулятором")
        self.resize(1100, 680)

        self.dt = 0.1  # Временной шаг квантования (секунды)
        self.step_counter = 0
        self.is_running = False

        # Экземпляры расчетных модулей
        self.model = TankModel(S=4.0, H_max=10.0, H_init=3.0, Cv=2.0)
        self.pid = PIDController(kp=20.0, ki=1.2, kd=4.5, dt=self.dt, lim_min=0.0, lim_max=100.0)

        # Технологические уставки аварийной сигнализации
        self.H_HIGH = 8.5
        self.H_LOW = 1.5

        # Высокоточный таймер подсистемы GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.simulation_step)

        self.init_ui()
        self.log_message("SCADA: Система инициализирована в режиме ожидания.")

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Левый сектор (Экран мнемосхемы и тренда)
        left_layout = QVBoxLayout()
        self.mnemonic = MnemonicWidget()
        self.chart = TrendCanvas(self)
        left_layout.addWidget(self.mnemonic, stretch=1)
        left_layout.addWidget(self.chart, stretch=2)

        # Правый сектор (Инпуты параметров процесса и Логгер)
        right_layout = QVBoxLayout()
        ctrl_group = QGroupBox("Панель управления технологическим процессом")
        ctrl_grid = QGridLayout()

        ctrl_grid.addWidget(QLabel("Постоянный приток Q_in (м³/с):"), 0, 0)
        self.spin_qin = QDoubleSpinBox()
        self.spin_qin.setRange(0, 10);
        self.spin_qin.setValue(2.0)
        ctrl_grid.addWidget(self.spin_qin, 0, 1)

        ctrl_grid.addWidget(QLabel("Заданный уровень SP (м):"), 1, 0)
        self.spin_sp = QDoubleSpinBox()
        self.spin_sp.setRange(0.5, 9.5);
        self.spin_sp.setValue(5.0)
        ctrl_grid.addWidget(self.spin_sp, 1, 1)

        ctrl_grid.addWidget(QLabel("Пропорциональный (Kp):"), 2, 0)
        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setRange(0, 200);
        self.spin_kp.setValue(25.0)
        ctrl_grid.addWidget(self.spin_kp, 2, 1)

        ctrl_grid.addWidget(QLabel("Интегральный (Ki):"), 3, 0)
        self.spin_ki = QDoubleSpinBox()
        self.spin_ki.setRange(0, 50);
        self.spin_ki.setValue(1.5)
        ctrl_grid.addWidget(self.spin_ki, 3, 1)

        ctrl_grid.addWidget(QLabel("Дифференциальный (Kd):"), 4, 0)
        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setRange(0, 100);
        self.spin_kd.setValue(5.0)
        ctrl_grid.addWidget(self.spin_kd, 4, 1)

        self.btn_start = QPushButton("Пуск")
        self.btn_start.setStyleSheet("background-color: #2ecc71; font-weight: bold; color: white;")
        self.btn_start.clicked.connect(self.start_sim)
        ctrl_grid.addWidget(self.btn_start, 5, 0)

        self.btn_reset = QPushButton("Сброс")
        self.btn_reset.clicked.connect(self.reset_sim)
        ctrl_grid.addWidget(self.btn_reset, 5, 1)

        ctrl_group.setLayout(ctrl_grid)
        right_layout.addWidget(ctrl_group)

        # Лог событий
        log_group = QGroupBox("Журнал событий / Аварийная сигнализация")
        log_box = QVBoxLayout()
        self.log_txt = QTextEdit()
        self.log_txt.setReadOnly(True)
        self.log_txt.setStyleSheet("background-color: #1e1e1e; color: #ffffff; font-family: Consolas;")
        log_box.addWidget(self.log_txt)
        log_group.setLayout(log_box)
        right_layout.addWidget(log_group, stretch=1)

        main_layout.addLayout(left_layout, stretch=2)
        main_layout.addLayout(right_layout, stretch=1)
        self.update_gui_elements(0.0, 0.0)

    def start_sim(self):
        if not self.is_running:
            self.is_running = True
            self.btn_start.setText("Стоп")
            self.btn_start.setStyleSheet("background-color: #e74c3c; font-weight: bold; color: white;")
            self.timer.start(int(self.dt * 1000))
            self.log_message("АСУТП: Режим АВТО запущен. Контур ПИД замкнут.")
        else:
            self.is_running = False
            self.btn_start.setText("Пуск")
            self.btn_start.setStyleSheet("background-color: #2ecc71; font-weight: bold; color: white;")
            self.timer.stop()
            self.log_message("АСУТП: Контур разомкнут, симуляция остановлена.")

    def reset_sim(self):
        self.model.reset()
        self.pid.reset()
        self.step_counter = 0
        self.chart.clear_charts()
        self.update_gui_elements(0.0, 0.0)
        self.log_message("АСУТП: Выполнен сброс модели и интеграторов к базисным значениям.")

    def simulation_step(self):
        """Основной такт вычисления и отрисовки (Real-time Loop)"""
        # Динамическое обновление параметров регулятора оператором
        self.pid.kp = self.spin_kp.value()
        self.pid.ki = self.spin_ki.value()
        self.pid.kd = self.spin_kd.value()

        qin = self.spin_qin.value()
        sp = self.spin_sp.value()

        # Шаг 1: Работа ПИД регулятора
        valve_open_pct = self.pid.update(setpoint=sp, measurement=self.model.H)

        # Шаг 2: Расчет физики ОУ
        h, qout = self.model.update(Q_in=qin, valve_open_pct=valve_open_pct, dt=self.dt)

        # Шаг 3: Технологический контроль (Аварии)
        self.check_alarms(h)

        # Шаг 4: Рефреш UI компонентов
        self.update_gui_elements(qout, valve_open_pct)
        self.chart.add_point(self.step_counter, h, sp)
        self.step_counter += 1

    def check_alarms(self, h):
        if h >= self.H_HIGH:
            self.log_message(f" КРИТ_АВАРИЯ: Превышен верхний аварийный уровень! H = {h:.2f} м", is_alarm=True)
        elif h <= self.H_LOW:
            self.log_message(f"️ ПРЕДУПРЕЖДЕНИЕ: Нарушен нижний предел уровня! H = {h:.2f} м", is_alarm=True)

    def update_gui_elements(self, qout, valve_open):
        self.mnemonic.update_state(
            H=self.model.H, H_max=self.model.H_max,
            Q_in=self.spin_qin.value(), Q_out=qout, valve_open=valve_open
        )

    def log_message(self, text, is_alarm=False):
        now = datetime.datetime.now().strftime("%H:%M:%S")

        color = "#ff4d4d" if is_alarm else "#2ecc71" if "запущен" in text else "#ffffff"
        prefix = "[ALARM]" if is_alarm else "[INFO]"
        msg = f"{now} {prefix} {text}"
        self.log_txt.append(msg)
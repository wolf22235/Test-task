import numpy as np


class TankModel:
    """
    Математическая модель динамики жидкости в технологическом резервуаре.
    Уравнение материального баланса: dH/dt = (Q_in - Q_out) / S
    """

    def __init__(self, S: float = 4.0, H_max: float = 10.0, H_init: float = 3.0, Cv: float = 2.0):
        self.S = S  # Площадь поперечного сечения бака (м²)
        self.H_max = H_max  # Максимальная конструктивная высота (м)
        self.H = H_init  # Текущий уровень жидкости (м) - динамическое состояние
        self.Cv = Cv  # Коэффициент пропускной способности сливного клапана

    def update(self, Q_in: float, valve_open_pct: float, dt: float) -> tuple[float, float]:
        """
        Расчет состояния объекта на следующем шаге квантования времени.

        :param Q_in: Константный или возмущающий приток в бак (м³/с)
        :param valve_open_pct: Степень открытия регулирующего клапана на сливе (0 - 100%)
        :param dt: Шаг интегрирования (секунды)
        :return: Кортеж (Новый уровень H, Актуальный расход Q_out)
        """
        # 1. Расчет нелинейного расхода через клапан (Закон Торричелли)
        # Расход пропорционален степени открытия И квадратному корню из высоты столба жидкости
        valve_ratio = valve_open_pct / 100.0
        if self.H > 0:
            Q_out = self.Cv * valve_ratio * np.sqrt(self.H)
        else:
            Q_out = 0.0

        # 2. Численное интегрирование дифференциального уравнения методом Эйлера
        # dH = (f(t, ...)) * dt
        dH = ((Q_in - Q_out) / self.S) * dt
        self.H += dH

        # 3. Физические ограничения геометрии резервуара (насыщение)
        if self.H > self.H_max:
            self.H = self.H_max
        elif self.H < 0.0:
            self.H = 0.0

        return self.H, Q_out

    def reset(self, H_init: float = 3.0):
        """Сброс модели к начальным условиям"""
        self.H = H_init

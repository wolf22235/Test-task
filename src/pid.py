class PIDController:
    """
    Дискретный параллельный ПИД-регулятор.
    Формула: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*(de(t)/dt)
    """
    def __init__(self, kp: float, ki: float, kd: float, dt: float, lim_min: float = 0.0, lim_max: float = 100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.lim_min = lim_min  # Ограничение хода клапана (0%)
        self.lim_max = lim_max  # Ограничение хода клапана (100%)

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True

    def update(self, setpoint: float, measurement: float) -> float:
        """
        Расчет управляющего воздействия.
        Поскольку клапан стоит на СЛИВЕ, логика регулятора "обратная":
        если уровень выше уставки (ошибка отрицательная), клапан должен ОТКРЫВАТЬСЯ для слива.
        """
        # Рассчитываем технологическую ошибку рассогласования (e = PV - SP)
        # Обратите внимание: для сливного клапана ошибка инвертирована, чтобы знак Kp оставался положительным
        error = measurement - setpoint

        # Пропорциональная часть
        p_term = self.kp * error

        # Интегральная часть (Численное интегрирование методом левых прямоугольников)
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # Дифференциальная часть (Численное дифференцирование назад)
        if self.first_run:
            d_term = 0.0
            self.first_run = False
        else:
            d_term = self.kd * (error - self.prev_error) / self.dt

        self.prev_error = error

        # Полное управляющее воздействие
        output = p_term + i_term + d_term

        # Алгоритм Anti-Windup (Защита от накопления интегральной ошибки при насыщении)
        if output > self.lim_max:
            output = self.lim_max
            self.integral -= error * self.dt  # Замораживаем интегратор
        elif output < self.lim_min:
            output = self.lim_min
            self.integral -= error * self.dt  # Замораживаем интегратор

        return output

    def reset(self):
        """Сброс накопленных внутренних состояний регулятора"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True

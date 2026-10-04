import unittest
from src.model import TankModel
from src.pid import PIDController

class TestSimulationComponents(unittest.TestCase):
    def test_tank_physics_accumulation(self):
        """Проверка закона сохранения массы в резервуаре"""
        tank = TankModel(S=1.0, H_max=10.0, H_init=5.0, Cv=1.0)
        # Если клапан заперт (0%), а приток равен 2.0, за dt=1.0с уровень должен вырасти ровно на 2.0м
        h_new, q_out = tank.update(Q_in=2.0, valve_open_pct=0.0, dt=1.0)
        self.assertEqual(q_out, 0.0)
        self.assertEqual(h_new, 7.0)

    def test_pid_saturation(self):
        """Проверка работы ограничений исполнительного органа (Anti-Windup)"""
        pid = PIDController(kp=10.0, ki=1.0, kd=0.0, dt=1.0, lim_min=0.0, lim_max=100.0)
        # Огромная ошибка вызовет насыщение. Выход должен замереть ровно на 100.0
        output = pid.update(setpoint=0.0, measurement=50.0)
        self.assertEqual(output, 100.0)

if __name__ == '__main__':
    unittest.main()
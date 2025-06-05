class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e_prev = 0
        self.e_acc = 0

    def control(self, e): 

        self.e_acc += e

        P = self.kp * e
        I = self.ki * self.e_acc
        D = self.kd * (e - self.e_prev)

        self.e_prev = e
        return P + I + D
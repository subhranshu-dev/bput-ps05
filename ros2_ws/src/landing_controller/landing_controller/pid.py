"""PID with output saturation, integral anti-windup and filtered derivative (Phase 18).

    u = Kp*e + Ki*integral(e) + Kd*de/dt
"""


class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, out_limit: float = float('inf'),
                 i_limit: float = float('inf'), d_alpha: float = 0.3):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = abs(out_limit)
        self.i_limit = abs(i_limit)
        self.d_alpha = d_alpha
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_e = None
        self.d_filt = 0.0
        self.last = {'p': 0.0, 'i': 0.0, 'd': 0.0, 'u': 0.0, 'saturated': False}

    def update(self, e: float, dt: float) -> float:
        if dt <= 0:
            return self.last['u']
        if self.prev_e is None:
            d_raw = 0.0
        else:
            d_raw = (e - self.prev_e) / dt
        self.prev_e = e
        self.d_filt = (1 - self.d_alpha) * self.d_filt + self.d_alpha * d_raw
        p = self.kp * e
        d = self.kd * self.d_filt
        # tentative integral; conditional integration = anti-windup
        new_int = max(-self.i_limit, min(self.i_limit, self.integral + e * dt))
        u_unsat = p + self.ki * new_int + d
        u = max(-self.out_limit, min(self.out_limit, u_unsat))
        saturated = u != u_unsat
        if not saturated or (u_unsat > 0) != (e > 0):
            self.integral = new_int
        i = self.ki * self.integral
        u = max(-self.out_limit, min(self.out_limit, p + i + d))
        self.last = {'p': p, 'i': i, 'd': d, 'u': u, 'saturated': saturated}
        return u

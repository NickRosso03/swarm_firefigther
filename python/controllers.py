"""
controllers.py — Controllori P / PI / PID.
"""

from typing import Optional


def saturate(inp: float, sat: float) -> tuple[float, bool]:
    """Applica saturazione simmetrica. Restituisce (valore_saturato, in_sat)."""
    if inp > sat:
        return sat, True
    if inp < -sat:
        return -sat, True
    return inp, False


# ---------------------------------------------------------------------------
# Integrator e Derivator
# ---------------------------------------------------------------------------

class Integrator:
    """Integrazione numerica con metodo di Eulero."""

    def __init__(self):
        self.prev_output = 0.0

    def evaluate(self, delta_t: float, inp: float) -> float:
        self.prev_output += inp * delta_t
        return self.prev_output

    def reset(self):
        self.prev_output = 0.0


class Derivator:
    """Derivazione numerica (differenze finite all'indietro)."""

    def __init__(self):
        self._prev = None

    def evaluate(self, delta_t: float, inp: float) -> float:
        if self._prev is None or delta_t <= 0:
            self._prev = inp
            return 0.0
        out = (inp - self._prev) / delta_t
        self._prev = inp
        return out

    def reset(self):
        self._prev = None


# ---------------------------------------------------------------------------
# Controllori
# ---------------------------------------------------------------------------

class P_Controller:
    """Controllore proporzionale puro."""

    def __init__(self, kp: float, sat: Optional[float] = None):
        self.kp          = kp
        self.saturation  = sat

    def evaluate(self, delta_t: float, error: float) -> float:
        out = self.kp * error
        if self.saturation is not None:
            out, _ = saturate(out, self.saturation)
        return out


class PI_Controller:
    """
    Controllore PI con anti-windup.
    L'integrale si congela quando l'uscita è saturata.
    """

    def __init__(self, kp: float, ki: float, sat: Optional[float] = None):
        self.kp         = kp
        self.ki         = ki
        self.saturation = sat
        self._i         = Integrator()
        self._in_sat    = False

    def evaluate(self, delta_t: float, error: float) -> float:
        p_out = self.kp * error

        if self._in_sat:
            # Integrale congelato: usa il valore precedente
            i_out = self.ki * self._i.prev_output
        else:
            i_out = self.ki * self._i.evaluate(delta_t, error)

        out = p_out + i_out

        if self.saturation is not None:
            out, self._in_sat = saturate(out, self.saturation)

        return out

    def reset(self):
        self._i.reset()
        self._in_sat = False


class PID_Controller(PI_Controller):
    """Controllore PID (PI + termine derivativo)."""

    def __init__(self, kp: float, ki: float, kd: float,
                 sat: Optional[float] = None):
        super().__init__(kp, ki, sat)
        self.kd  = kd
        self._d  = Derivator()

    def evaluate(self, delta_t: float, error: float) -> float:
        pi_out = super().evaluate(delta_t, error)
        d_out  = self._d.evaluate(delta_t, error) * self.kd
        total  = pi_out + d_out
        # Applica saturazione all'uscita TOTALE (PI+D), non solo al PI.
        # Senza questo, il termine D può bypassare la saturazione e
        # causare spike enormi al cambio di target.
        if self.saturation is not None:
            total, _ = saturate(total, self.saturation)
        return total

    def reset(self):
        super().reset()
        self._d.reset()
from __future__ import annotations

import math
import random

from . import constants as C

SUBSTEP_DT = 0.005


class SimulatedPlant:
    def __init__(self) -> None:
        self.y = 0.0
        self.v = 0.0
        self.up = 0.0
        self.yf = 0.0

        self.y_rot = 0.0
        self.v_rot = 0.0
        self.up_rot = 0.0
        self.yf_rot = 0.0

        self.y_pend = 0.0
        self.v_pend = 0.0
        self.yf_pend = 0.0

        self.t = 0.0

    def advance_linear(self, u: float, Ts: float) -> None:
        ua = self.up
        self.up = u
        n = max(1, round(Ts / SUBSTEP_DT))
        for _ in range(n):
            if abs(ua) > C.SIM_UDZ:
                vss = C.SIM_K * (ua - math.copysign(C.SIM_UDZ, ua))
            else:
                vss = 0.0
            self.v = self.v + SUBSTEP_DT / C.SIM_TAU * (vss - self.v)
            self.y = self.y + self.v * SUBSTEP_DT
        self.y = max(-1.0, min(C.MAX_DIST_CM + 1.0, self.y))

    def advance_rotation(self, u_rot: float, Ts: float) -> None:
        ua = self.up_rot
        self.up_rot = u_rot
        n = max(1, round(Ts / SUBSTEP_DT))
        for _ in range(n):
            if abs(ua) > C.SIM_UDZ_BASE:
                vss = C.SIM_K_BASE * (ua - math.copysign(C.SIM_UDZ_BASE, ua))
            else:
                vss = 0.0
            self.v_rot = self.v_rot + SUBSTEP_DT / C.SIM_TAU_BASE * (vss - self.v_rot)
            self.y_rot = self.y_rot + self.v_rot * SUBSTEP_DT

            vss_p = C.SIM_K_PEND * ua
            self.v_pend = self.v_pend + SUBSTEP_DT / C.SIM_TAU_PEND * (vss_p - self.v_pend)
            self.y_pend = self.y_pend + self.v_pend * SUBSTEP_DT
        self.y_rot = max(-C.MAX_ROT_DEG - 1.0, min(C.MAX_ROT_DEG + 1.0, self.y_rot))
        self.y_pend = max(-C.PEND_REF_MAX_DEG - 5.0, min(C.PEND_REF_MAX_DEG + 5.0, self.y_pend))

    def step(self, u_lin: float, u_rot: float, Ts: float):
        self.advance_linear(u_lin, Ts)
        self.advance_rotation(u_rot, Ts)
        self.t += Ts

        raw = self.y + 0.15 * random.gauss(0, 1)
        raw_r = self.y_rot + 0.30 * random.gauss(0, 1)
        raw_p = self.y_pend + 0.30 * random.gauss(0, 1)
        self.yf = 0.5 * raw + 0.5 * self.yf
        self.yf_rot = 0.5 * raw_r + 0.5 * self.yf_rot
        self.yf_pend = 0.5 * raw_p + 0.5 * self.yf_pend
        return self.yf, self.yf_rot, self.yf_pend, self.t

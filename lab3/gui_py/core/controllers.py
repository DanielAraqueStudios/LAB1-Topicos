from __future__ import annotations

import numpy as np

from . import design
from .constants import PlantParams


class PidController:
    def __init__(self, plant: PlantParams, wc: float, error_band: float) -> None:
        self.plant = plant
        self.gains = design.design_pid(plant, wc)
        self.error_band = error_band
        self.I = 0.0

    def reset(self) -> None:
        self.I = 0.0

    def step(self, e: float, y: float, vhat: float, Ts: float) -> float:
        g = self.gains
        uc = g.Kp * e + g.Ki * self.I - g.Kd * vhat
        if abs(e) < self.error_band and abs(uc) < self.plant.ucMax:
            self.I = max(-g.Imax, min(g.Imax, self.I + e * Ts))
        return uc


class RootLocusZController:
    def __init__(self, plant: PlantParams, Ts: float, wc: float) -> None:
        self.plant = plant
        self.coeffs = design.design_lead_z(plant, Ts, plant.tau, wc)
        self._be = np.zeros(len(self.coeffs.Cn))
        self._bu = np.zeros(max(0, len(self.coeffs.Cd) - 1))

    def reset(self) -> None:
        self._be[:] = 0.0
        self._bu[:] = 0.0

    def step(self, e: float, y: float, vhat: float, Ts: float) -> float:
        Cn, Cd = self.coeffs.Cn, self.coeffs.Cd
        self._be = np.concatenate(([e], self._be[:-1]))
        if len(Cd) > 1:
            u = float(Cn @ self._be[: len(Cn)] - Cd[1:] @ self._bu[: len(Cd) - 1])
        else:
            u = float(Cn @ self._be[: len(Cn)])
        u = max(-self.plant.ucMax, min(self.plant.ucMax, u))
        self._bu = np.concatenate(([u], self._bu[:-1]))
        return u


class DeadBeatController:
    def __init__(self, plant: PlantParams, Ts: float, wc: float, od_n: int, od_tipo: str, od_ganancia: float) -> None:
        self.plant = plant
        self.od_n = od_n
        self.coeffs = design.design_dead_beat(plant, Ts, od_n, od_tipo, od_ganancia)
        self._be = np.zeros(len(self.coeffs.Cn))
        self._bu = np.zeros(max(0, len(self.coeffs.Cd) - 1))
        self._cnt = 0
        self._uh = 0.0

    def reset(self) -> None:
        self._be[:] = 0.0
        self._bu[:] = 0.0
        self._cnt = 0
        self._uh = 0.0

    def step(self, e: float, y: float, vhat: float, Ts: float) -> float:
        Cn, Cd, ganancia = self.coeffs.Cn, self.coeffs.Cd, self.coeffs.ganancia
        self._cnt += 1
        if (self._cnt - 1) % self.od_n == 0:
            self._be = np.concatenate(([e], self._be[:-1]))
            u = float(Cn @ self._be[: len(Cn)] - Cd[1:] @ self._bu[: len(Cd) - 1])
            u = max(-self.plant.ucMax, min(self.plant.ucMax, u * ganancia))
            self._bu = np.concatenate(([u / ganancia], self._bu[:-1]))
            self._uh = u
        return self._uh


class ServoController:
    def __init__(self, plant: PlantParams, Ts: float, wc: float, y0: float) -> None:
        self.plant = plant
        self.gains = design.design_servo(plant, Ts, wc, np.array([[y0], [0.0]]))
        self.x = np.array([[y0], [0.0]])
        self.xi = 0.0

    def reset(self, y0: float = 0.0) -> None:
        self.x = np.array([[y0], [0.0]])
        self.xi = 0.0

    def step(self, e: float, y: float, vhat: float, Ts: float) -> float:
        g = self.gains
        u_raw = float(-(g.Ka[:2] @ self.x.flatten()) - g.Ka[2] * self.xi)
        uc = max(-self.plant.ucMax, min(self.plant.ucMax, u_raw))
        if abs(u_raw) <= self.plant.ucMax or np.sign(e) != np.sign(u_raw):
            self.xi = max(-g.ximax, min(g.ximax, self.xi + e))
        innov = y - float((g.Cd @ self.x).item())
        self.x = g.Ad @ self.x + g.Bd * uc + g.L * innov
        return uc


def build_controller(kind: str, plant: PlantParams, Ts: float, wc: float, y0: float,
                      error_band: float, od_n: int, od_tipo: str, od_ganancia: float):
    if kind == "PID":
        return PidController(plant, wc, error_band)
    if kind == "RootLocusZ":
        return RootLocusZController(plant, Ts, wc)
    if kind == "OscMuertas":
        return DeadBeatController(plant, Ts, wc, od_n, od_tipo, od_ganancia)
    if kind == "Servosistema":
        return ServoController(plant, Ts, wc, y0)
    raise ValueError(f"unknown controller kind: {kind}")

from __future__ import annotations

import cmath
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.signal import cont2discrete, ss2tf

from .constants import PlantParams


def _plant_ss(K: float, tau: float):
    A = np.array([[0.0, 1.0], [0.0, -1.0 / tau]])
    B = np.array([[0.0], [K / tau]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])
    return A, B, C, D


def discretize_zoh(K: float, tau: float, Ts: float):
    A, B, C, D = _plant_ss(K, tau)
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), Ts, method="zoh")
    return Ad, Bd, Cd, Dd


def _strip_leading_zeros(num: np.ndarray) -> np.ndarray:
    idx = np.argmax(np.abs(num) > 1e-12) if np.any(np.abs(num) > 1e-12) else 0
    return num[idx:]


def ackermann_place(A: np.ndarray, B: np.ndarray, poles) -> np.ndarray:
    n = A.shape[0]
    cols = [np.linalg.matrix_power(A, i) @ B for i in range(n)]
    Ctrb = np.hstack(cols)
    Ctrb_inv = np.linalg.inv(Ctrb)
    coeffs = np.real(np.poly(np.asarray(poles, dtype=complex)))
    Delta = np.zeros((n, n), dtype=float)
    for i, c in enumerate(coeffs):
        power = n - i
        Delta = Delta + c * np.real(np.linalg.matrix_power(A, power))
    e_n = np.zeros((1, n))
    e_n[0, -1] = 1.0
    K = e_n @ Ctrb_inv @ Delta
    return np.real(K).flatten()


@dataclass
class PidGains:
    Kp: float
    Ki: float
    Kd: float
    Imax: float


@dataclass
class DeadBeatCoeffs:
    Cn: np.ndarray
    Cd: np.ndarray
    ganancia: float


@dataclass
class ServoGains:
    Ad: np.ndarray
    Bd: np.ndarray
    Cd: np.ndarray
    Ka: np.ndarray
    L: np.ndarray
    ximax: float


@dataclass
class LeadZCoeffs:
    Cn: np.ndarray
    Cd: np.ndarray


def design_pid(plant: PlantParams, wc: float) -> PidGains:
    Kp = wc / plant.K
    Ki = Kp * wc / 4.0
    Kd = Kp * 0.25 / wc
    Imax = 0.5 * plant.ucMax / max(Ki, 1e-6)
    return PidGains(Kp=Kp, Ki=Ki, Kd=Kd, Imax=Imax)


def design_dead_beat(plant: PlantParams, Ts: float, od_n: int, od_tipo: str, od_ganancia: float) -> DeadBeatCoeffs:
    Tod = Ts * od_n
    Ad, Bd, Cd, _ = discretize_zoh(plant.K, plant.tau, Tod)
    num, den = ss2tf(Ad, Bd, Cd, np.array([[0.0]]))
    nd = _strip_leading_zeros(np.asarray(num).flatten())
    dd = np.asarray(den).flatten()
    if od_tipo == "rampa":
        Cn = np.convolve([2.0, -1.0], dd)
        Cd_poly = np.convolve([1.0, -2.0, 1.0], nd)
    else:
        Cn = dd
        Cd_poly = np.convolve([1.0, -1.0], nd)
    scale = Cd_poly[0]
    return DeadBeatCoeffs(Cn=Cn / scale, Cd=Cd_poly / scale, ganancia=od_ganancia)


def design_servo(plant: PlantParams, Ts: float, wc: float, x0: np.ndarray) -> ServoGains:
    Ad, Bd, Cd, _ = discretize_zoh(plant.K, plant.tau, Ts)
    Aa = np.block([[Ad, np.zeros((2, 1))], [-Cd, np.array([[1.0]])]])
    Ba = np.vstack([Bd, [[0.0]]])
    wn = 1.5 * wc
    zt = 0.9
    wd = wn * math.sqrt(1 - zt**2)
    poles_c = [complex(-zt * wn, wd), complex(-zt * wn, -wd), complex(-2 * wn, 0.0)]
    poles_d = [cmath.exp(Ts * s) for s in poles_c]
    Ka = ackermann_place(Aa, Ba, poles_d)

    obs_poles_c = [-3 * wn, -3.5 * wn]
    obs_poles_d = [math.exp(Ts * s) for s in obs_poles_c]
    Lrow = ackermann_place(Ad.T, Cd.T, obs_poles_d)
    L = Lrow.reshape(2, 1)

    ximax = 2 * plant.ucMax / max(abs(Ka[2]), 1e-6)
    return ServoGains(Ad=Ad, Bd=Bd, Cd=Cd, Ka=Ka, L=L, ximax=ximax)


def design_lead_z(plant: PlantParams, Ts: float, tau: float, wc: float) -> LeadZCoeffs:
    zt = 0.9
    wn = 1.5 * wc
    sigma = zt * wn
    omega = wn * math.sqrt(1 - zt**2)
    s0 = complex(-sigma, omega)
    Gs0 = plant.K / (s0 * (tau * s0 + 1))
    phiG = cmath.phase(Gs0)
    target = -math.pi - phiG
    phiNeeded = math.atan2(math.sin(target), math.cos(target))

    zc = max(sigma, 0.05)
    angPolo = (math.pi / 2) - phiNeeded
    angPolo = max(min(angPolo, math.pi / 2 - 1e-3), -math.pi / 2 + 1e-3)
    pc = sigma + omega / math.tan(angPolo)
    pc = max(pc, zc + 0.05)
    Kc = abs(s0 + pc) / (abs(s0 + zc) * abs(Gs0))

    num_c = [Kc, Kc * zc]
    den_c = [1.0, pc]
    num_d, den_d, _ = cont2discrete((num_c, den_c), Ts, method="bilinear")
    num_d = np.asarray(num_d).flatten()
    den_d = np.asarray(den_d).flatten()
    return LeadZCoeffs(Cn=num_d / den_d[0], Cd=den_d / den_d[0])

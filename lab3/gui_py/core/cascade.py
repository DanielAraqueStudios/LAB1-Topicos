from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import constants as C
from .controllers import build_controller
from .protocol import Telemetry


def _ramp_toward(current: float, target: float, max_step: float) -> float:
    d = target - current
    if abs(d) <= max_step:
        return target
    return current + math.copysign(max_step, d)


def _deadzone_pwm(uc: float, udz: float, u_max: float) -> float:
    if abs(uc) < 2:
        u = 0.0
    else:
        u = math.copysign(udz + abs(uc), uc)
    return max(-u_max, min(u_max, u))


@dataclass
class AxisState:
    u_prev_c: float = 0.0
    en_banda: bool = False
    meas_y: float = 0.0
    meas_vhat: float = 0.0


@dataclass
class CascadeOutput:
    u_lin: float = 0.0
    pwm_rot: float = 0.0
    pend_ref: float = 0.0
    e_lin: float = 0.0
    e_rot: float = 0.0
    e_pend: float = 0.0


class CascadeController:
    """Ports main.m's loopControl(): ramp -> controller -> rate limit -> deadband -> deadzone."""

    def __init__(self, kind: str, Ts: float = C.SAMPLE_TIME,
                 plant_lin: C.PlantParams | None = None,
                 plant_outer: C.PlantParams | None = None,
                 plant_inner: C.PlantParams | None = None) -> None:
        self.Ts = Ts
        self.kind = kind
        self.plant_lin = plant_lin or C.default_linear_plant()
        self.plant_outer = plant_outer or C.default_outer_plant()
        self.plant_inner = plant_inner or C.default_inner_plant()

        self.dist_ref = 0.0
        self.dist_ref_ramp = 0.0
        self.rot_ref = 0.0
        self.rot_ref_ramp = 0.0

        self.lin = AxisState()
        self.outer = AxisState()
        self.inner = AxisState()

        self.sin_datos = 0
        self.sin_datos_rot = 0

        self._build_controllers()

    def _build_controllers(self) -> None:
        self.c_lin = build_controller(self.kind, self.plant_lin, self.Ts, self.plant_lin.wc, self.lin.meas_y,
                                       error_band=3.0, od_n=C.OD_N, od_tipo=C.OD_TIPO, od_ganancia=C.OD_GANANCIA)
        self.c_outer = build_controller(self.kind, self.plant_outer, self.Ts, self.plant_outer.wc, self.outer.meas_y,
                                         error_band=5.0, od_n=C.OD_N_ROT, od_tipo=C.OD_TIPO_ROT,
                                         od_ganancia=C.OD_GANANCIA_ROT)
        self.c_inner = build_controller(self.kind, self.plant_inner, self.Ts, self.plant_inner.wc, self.inner.meas_y,
                                         error_band=5.0, od_n=C.OD_N_ROT, od_tipo=C.OD_TIPO_ROT,
                                         od_ganancia=C.OD_GANANCIA_ROT)

    def set_kind(self, kind: str) -> None:
        self.kind = kind
        self.lin = AxisState(meas_y=self.lin.meas_y)
        self.outer = AxisState(meas_y=self.outer.meas_y)
        self.inner = AxisState(meas_y=self.inner.meas_y)
        self.dist_ref_ramp = self.dist_ref
        self.rot_ref_ramp = self.rot_ref
        self._build_controllers()

    def set_plants(self, plant_lin: C.PlantParams, plant_outer: C.PlantParams, plant_inner: C.PlantParams) -> None:
        self.plant_lin, self.plant_outer, self.plant_inner = plant_lin, plant_outer, plant_inner
        self._build_controllers()

    def seed_from_measurement(self, y_lin: float, y_rot: float, y_pend: float) -> None:
        self.lin.meas_y = max(0.0, min(C.MAX_DIST_CM, y_lin))
        self.outer.meas_y = max(-C.MAX_ROT_DEG, min(C.MAX_ROT_DEG, y_rot))
        self.inner.meas_y = y_pend
        self.dist_ref = self.dist_ref_ramp = self.lin.meas_y
        self.rot_ref = self.rot_ref_ramp = self.outer.meas_y
        self._build_controllers()

    def update_measurement(self, y_lin: float, y_rot: float, y_pend: float, dt: float) -> None:
        if 0.01 < dt < 0.5:
            self.lin.meas_vhat = 0.7 * self.lin.meas_vhat + 0.3 * (y_lin - self.lin.meas_y) / dt
            self.outer.meas_vhat = 0.7 * self.outer.meas_vhat + 0.3 * (y_rot - self.outer.meas_y) / dt
            self.inner.meas_vhat = 0.7 * self.inner.meas_vhat + 0.3 * (y_pend - self.inner.meas_y) / dt
        self.lin.meas_y, self.outer.meas_y, self.inner.meas_y = y_lin, y_rot, y_pend
        self.sin_datos = 0
        self.sin_datos_rot = 0

    def mark_no_data(self) -> None:
        self.sin_datos += 1
        self.sin_datos_rot += 1

    def tick(self) -> CascadeOutput:
        Ts = self.Ts
        rr = C.RAMP_RATE[self.kind] * Ts
        self.dist_ref_ramp = _ramp_toward(self.dist_ref_ramp, self.dist_ref, rr)
        rrR = C.RAMP_RATE_ROT[self.kind] * Ts
        self.rot_ref_ramp = _ramp_toward(self.rot_ref_ramp, self.rot_ref, rrR)

        e = self.dist_ref_ramp - self.lin.meas_y
        e_ext = self.rot_ref_ramp - self.outer.meas_y

        if self.sin_datos > 10:
            uc = 0.0
        else:
            uc = self.c_lin.step(e, self.lin.meas_y, self.lin.meas_vhat, Ts)
        uc = max(-self.plant_lin.ucMax, min(self.plant_lin.ucMax, uc))
        du = uc - self.lin.u_prev_c
        uc = self.lin.u_prev_c + max(-C.MAX_DELTA_UC, min(C.MAX_DELTA_UC, du))
        if self.lin.en_banda:
            if abs(e) > C.BANDA_OUT:
                self.lin.en_banda = False
        elif abs(e) < C.BANDA_IN and abs(self.lin.meas_vhat) < 1.5:
            self.lin.en_banda = True
        if self.lin.en_banda:
            uc = 0.0
        self.lin.u_prev_c = uc
        u = _deadzone_pwm(uc, self.plant_lin.Udz, C.U_MAX)

        if self.sin_datos_rot > 10:
            pend_ref = 0.0
        else:
            pend_ref = self.c_outer.step(e_ext, self.outer.meas_y, self.outer.meas_vhat, Ts)
        pend_ref = max(-self.plant_outer.ucMax, min(self.plant_outer.ucMax, pend_ref))
        d_ext = pend_ref - self.outer.u_prev_c
        pend_ref = self.outer.u_prev_c + max(-C.MAX_DELTA_UC_EXT, min(C.MAX_DELTA_UC_EXT, d_ext))
        if self.outer.en_banda:
            if abs(e_ext) > C.BANDA_OUT_EXT:
                self.outer.en_banda = False
        elif abs(e_ext) < C.BANDA_IN_EXT and abs(self.outer.meas_vhat) < 3:
            self.outer.en_banda = True
        if self.outer.en_banda:
            pend_ref = 0.0
        self.outer.u_prev_c = pend_ref

        e_int = pend_ref - self.inner.meas_y
        if self.sin_datos_rot > 10:
            uc_int = 0.0
        else:
            uc_int = self.c_inner.step(e_int, self.inner.meas_y, self.inner.meas_vhat, Ts)
        uc_int = max(-self.plant_inner.ucMax, min(self.plant_inner.ucMax, uc_int))
        d_int = uc_int - self.inner.u_prev_c
        uc_int = self.inner.u_prev_c + max(-C.MAX_DELTA_UC_INT, min(C.MAX_DELTA_UC_INT, d_int))
        if self.inner.en_banda:
            if abs(e_int) > C.BANDA_OUT_INT:
                self.inner.en_banda = False
        elif abs(e_int) < C.BANDA_IN_INT and abs(self.inner.meas_vhat) < 3:
            self.inner.en_banda = True
        if self.inner.en_banda:
            uc_int = 0.0
        self.inner.u_prev_c = uc_int
        u_rot = _deadzone_pwm(uc_int, self.plant_inner.Udz, C.U_MAX)

        return CascadeOutput(u_lin=u, pwm_rot=u_rot, pend_ref=pend_ref, e_lin=e, e_rot=e_ext, e_pend=e_int)

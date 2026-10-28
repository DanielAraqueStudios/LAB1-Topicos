from __future__ import annotations

from dataclasses import dataclass

COM_PORT = "COM3"
BAUD_RATE = 115200
SAMPLE_TIME = 0.05

MAX_DIST_CM = 23.0
U_MAX = 230
MAX_DELTA_UC = 60
BANDA_IN = 0.3
BANDA_OUT = 0.6
RAMP_PID = 2.0
RAMP_OD = 2.0
RAMP_SRV = 1.5
RAMP_RLZ = 1.5
TAU_MOTOR = 0.10
OD_N = 4
OD_TIPO = "escalon"
OD_GANANCIA = 0.7
DEF_UDZ = 110
DEF_K = 0.034
DEF_WC = 2.0
SIM_K = 0.040
SIM_UDZ = 125
SIM_TAU = 0.12

MAX_ROT_DEG = 170.0
PEND_REF_MAX_DEG = 15.0
MAX_DELTA_UC_EXT = 10
MAX_DELTA_UC_INT = 60
BANDA_IN_EXT = 1.0
BANDA_OUT_EXT = 2.0
BANDA_IN_INT = 1.0
BANDA_OUT_INT = 2.0
RAMP_PID_ROT = 25.0
RAMP_OD_ROT = 25.0
RAMP_SRV_ROT = 20.0
RAMP_RLZ_ROT = 20.0
TAU_MOTOR_EXT = 0.12
TAU_MOTOR_INT = 0.12
OD_N_ROT = 4
OD_TIPO_ROT = "escalon"
OD_GANANCIA_ROT = 0.7

DEF_K_EXT = 0.60
DEF_WC_EXT = 2.0
DEF_UDZ_INT = 110
DEF_K_INT = 0.60
DEF_WC_INT = 2.0

SIM_K_BASE = 0.70
SIM_UDZ_BASE = 125
SIM_TAU_BASE = 0.15
SIM_K_PEND = 0.35
SIM_TAU_PEND = 0.25

CONTROLLERS = ("PID", "RootLocusZ", "OscMuertas", "Servosistema")

RAMP_RATE = {
    "PID": RAMP_PID,
    "RootLocusZ": RAMP_RLZ,
    "OscMuertas": RAMP_OD,
    "Servosistema": RAMP_SRV,
}
RAMP_RATE_ROT = {
    "PID": RAMP_PID_ROT,
    "RootLocusZ": RAMP_RLZ_ROT,
    "OscMuertas": RAMP_OD_ROT,
    "Servosistema": RAMP_SRV_ROT,
}


@dataclass
class PlantParams:
    K: float
    wc: float
    ucMax: float
    Udz: float = 0.0
    tau: float = TAU_MOTOR


def default_linear_plant() -> PlantParams:
    return PlantParams(K=DEF_K, wc=DEF_WC, ucMax=U_MAX - DEF_UDZ, Udz=DEF_UDZ, tau=TAU_MOTOR)


def default_outer_plant() -> PlantParams:
    return PlantParams(K=DEF_K_EXT, wc=DEF_WC_EXT, ucMax=PEND_REF_MAX_DEG, Udz=0.0, tau=TAU_MOTOR_EXT)


def default_inner_plant() -> PlantParams:
    return PlantParams(K=DEF_K_INT, wc=DEF_WC_INT, ucMax=U_MAX - DEF_UDZ_INT, Udz=DEF_UDZ_INT, tau=TAU_MOTOR_INT)

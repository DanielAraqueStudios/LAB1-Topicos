from __future__ import annotations

from dataclasses import dataclass

HEADER = "Tiempo_s,Referencia_rel_cm,Posicion_rel_cm,PWM_lin,Referencia_rot_deg,Posicion_rot_deg,PWM_rot,Angulo_pendulo_deg,Encoder_lineal_cuentas"
N_FIELDS = 8
N_FIELDS_WITH_ENCODER = 9


@dataclass
class Telemetry:
    t_s: float
    ref_lin_cm: float
    pos_lin_cm: float
    pwm_lin: int
    ref_rot_deg: float
    pos_rot_deg: float
    pwm_rot: int
    pos_pend_deg: float
    enc_lin_count: int | None = None


def format_command(ref_lin_cm: float, pwm_lin: int, ref_rot_deg: float, pwm_rot: int) -> str:
    return f"D{ref_lin_cm:.1f},U{round(pwm_lin)},R{ref_rot_deg:.1f},V{round(pwm_rot)}"


def parse_telemetry(line: str) -> Telemetry | None:
    s = line.strip()
    if not s or s.startswith("OK:") or s.startswith("ERROR:") or s == "PONG" or "Tiempo" in s:
        return None
    parts = s.split(",")
    if len(parts) < N_FIELDS:
        return None
    try:
        values = [float(p) for p in parts[:N_FIELDS]]
    except ValueError:
        return None
    enc_lin_count = None
    if len(parts) >= N_FIELDS_WITH_ENCODER:
        try:
            enc_lin_count = int(float(parts[8]))
        except ValueError:
            enc_lin_count = None
    return Telemetry(
        t_s=values[0],
        ref_lin_cm=values[1],
        pos_lin_cm=values[2],
        pwm_lin=int(values[3]),
        ref_rot_deg=values[4],
        pos_rot_deg=values[5],
        pwm_rot=int(values[6]),
        pos_pend_deg=values[7],
        enc_lin_count=enc_lin_count,
    )


def is_error_line(line: str) -> bool:
    return line.strip().startswith("ERROR:")

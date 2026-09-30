"""Verifies the reference-injection convention for augment_for_tracking: Ghat/Hhat/Chat
do not carry r(k) themselves, so it must be injected externally. Confirms r(k+1) (not
r(k)) added to all `order` tracking states after the state update is what drives the
steady-state tracking error to exactly zero, for order in {1, 2, 3} on generic plants."""
import numpy as np
import pytest

from src.discrete_tools import augment_for_tracking, ackermann_place
from src.system3_model import get_system3_ss
from src.discrete_tools import c2d_zoh, luenberger_observer_gain, observer_poles_ts_rule, map_s_to_z_pole


def _random_stable_siso(rng, n=2):
    A = rng.standard_normal((n, n)) - (n + 1) * np.eye(n)
    B = rng.standard_normal((n, 1))
    C = rng.standard_normal((1, n))
    return A, B, C


def _simulate_tracking(A, B, C, order, ref, steps):
    n0 = A.shape[0]
    Aa, Ba, Ca = augment_for_tracking(A, B, C, order=order)
    n = n0 + order
    desired = [0.2 + 0.03 * i for i in range(n)]
    K = ackermann_place(Aa, Ba, desired).flatten()

    x = np.zeros(n)
    ys = []
    for k in range(steps):
        y = float(np.ravel(C @ x[:n0])[0])
        ys.append(y)
        u = float(-K @ x)
        xn = Aa @ x + Ba.flatten() * u
        xn[n0:n] += ref[k + 1]
        x = xn
    return np.array(ys)


@pytest.mark.parametrize("order,ref_type", [(1, "step"), (2, "ramp"), (3, "parabola")])
@pytest.mark.parametrize("seed", range(5))
def test_r_kplus1_injection_gives_exact_tracking(order, ref_type, seed):
    rng = np.random.default_rng(5000 + 13 * order + seed)
    A, B, C = _random_stable_siso(rng)
    T = 0.1
    steps = 400
    if ref_type == "step":
        ref = np.ones(steps + 1)
    elif ref_type == "ramp":
        ref = np.array([k * T for k in range(steps + 1)])
    else:
        ref = np.array([0.5 * (k * T) ** 2 for k in range(steps + 1)])

    ys = _simulate_tracking(A, B, C, order, ref, steps)
    err = ref[:steps] - ys
    assert np.max(np.abs(err[-10:])) < 1e-6


@pytest.mark.parametrize("order,ref_type", [(2, "ramp"), (3, "parabola")])
def test_r_k_injection_leaves_residual_error(order, ref_type):
    """Contrast case: injecting r(k) (current, not next) instead of r(k+1) leaves a
    nonzero residual error for a time-varying reference - confirms the timing
    convention actually matters (not just any injection point works). Skipped for
    order=1/step since a constant reference has r(k)=r(k+1), making the timing
    indistinguishable there."""
    rng = np.random.default_rng(9000 + order)
    A, B, C = _random_stable_siso(rng)
    T = 0.1
    n0 = A.shape[0]
    Aa, Ba, Ca = augment_for_tracking(A, B, C, order=order)
    n = n0 + order
    desired = [0.2 + 0.03 * i for i in range(n)]
    K = ackermann_place(Aa, Ba, desired).flatten()

    steps = 400
    if ref_type == "step":
        ref = np.ones(steps + 1)
    elif ref_type == "ramp":
        ref = np.array([k * T for k in range(steps + 1)])
    else:
        ref = np.array([0.5 * (k * T) ** 2 for k in range(steps + 1)])

    x = np.zeros(n)
    ys = []
    for k in range(steps):
        y = float(np.ravel(C @ x[:n0])[0])
        ys.append(y)
        u = float(-K @ x)
        xn = Aa @ x + Ba.flatten() * u
        xn[n0:n] += ref[k]  # WRONG timing on purpose
        x = xn
    ys = np.array(ys)
    err = ref[:steps] - ys
    assert np.max(np.abs(err[-10:])) > 1e-4


def test_system3_parabola_state_feedback_observer_zero_error():
    """Regression pinned to Punto 3 (3.6): order=3 augmentation, Ackermann K, Luenberger
    L (factor=10 rule), full closed loop with observer (only [theta_m, theta_w]
    estimated - the tracking states are exact) must track a parabola reference to zero
    error and the observer estimate must converge, verified over 400 samples."""
    T = 0.1
    A, B, C, D = get_system3_ss()
    Ad, Bd, Cd, Dd = c2d_zoh(A, B, C, D, T)
    n0 = 2
    Ghat, Hhat, Chat = augment_for_tracking(Ad, Bd, Cd, order=3)

    zeta, wn = 0.7, 2.0
    s_ctrl = [complex(-zeta * wn, wn * np.sqrt(1 - zeta ** 2)),
              complex(-zeta * wn, -wn * np.sqrt(1 - zeta ** 2))]
    z_ctrl = [complex(map_s_to_z_pole(s, T)) for s in s_ctrl]
    desired = z_ctrl + [0.3, 0.25, 0.2]
    K = ackermann_place(Ghat, Hhat, desired).flatten()

    s_obs = observer_poles_ts_rule(s_ctrl, factor=10.0)
    z_obs = [complex(map_s_to_z_pole(s, T)) for s in s_obs]
    L = luenberger_observer_gain(Ad, Cd, z_obs).flatten()

    steps = 400
    r = np.array([0.5 * (k * T) ** 2 for k in range(steps + 1)])
    x = np.zeros(n0)
    xhat = np.array([0.5, 0.5])
    xtrack = np.zeros(3)
    ys = []
    xerrs = []
    for k in range(steps):
        y = float(np.ravel(Cd @ x)[0])
        yhat = float(np.ravel(Cd @ xhat)[0])
        ys.append(y)
        xerrs.append(np.linalg.norm(x - xhat))
        u = float(-K @ np.concatenate([xhat, xtrack]))
        full_next = Ghat @ np.concatenate([x, xtrack]) + Hhat.flatten() * u
        full_next[n0:] += r[k + 1]
        x = full_next[:n0]
        xtrack = full_next[n0:]
        xhat = Ad @ xhat + Bd.flatten() * u + L * (y - yhat)

    ys = np.array(ys)
    err = r[:steps] - ys
    assert np.max(np.abs(err[-10:])) < 1e-6
    assert np.max(xerrs[-10:]) < 1e-6

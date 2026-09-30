"""Discrete closed-loop simulators (state-feedback/observer and compensator loops)."""
import numpy as np


def simulate_discrete_closed_loop(A, B, C, K, L=None, x0=None, r=None, steps=50,
                                   observer=False, D=None, Br=None):
    """State-feedback (+optional observer, +optional integrator-augmented reference) simulation.

    Br injects r[k] directly into the state update (x[k+1] += Br*r[k]) and switches the
    control law to u=-K@x: use this with augment_with_integrator's Aa/Ba/Ca, passing
    Br=[0,...,0,1] on the integrator state, so the tracking error (r-y) actually drives xi.
    Without Br the simpler feedforward law u=-K@x+r is used (no exact tracking guarantee).
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(-1, 1)
    C = np.asarray(C, dtype=float).reshape(1, -1)
    K = np.asarray(K, dtype=float).reshape(1, -1)
    n = A.shape[0]
    if x0 is None:
        x0 = np.zeros(n)
    if r is None:
        r = np.ones(steps)
    if Br is not None:
        Br = np.asarray(Br, dtype=float).reshape(-1)
    x = np.array(x0, dtype=float)
    xhat = np.zeros(n) if observer else None
    ys, xs, us = [], [], []
    for k in range(steps):
        rk = r[k] if k < len(r) else r[-1]
        if observer:
            u = -K @ xhat
            y = float(np.ravel(C @ x)[0])
            xhat = A @ xhat + B.flatten() * float(np.ravel(u)[0]) + (
                L.flatten() * (y - float(np.ravel(C @ xhat)[0])) if L is not None else 0.0)
        elif Br is not None:
            u = -K @ x
            y = float(np.ravel(C @ x)[0])
        else:
            u = -K @ x + rk
            y = float(np.ravel(C @ x)[0])
        ys.append(y)
        xs.append(x.copy())
        u_val = float(np.ravel(u)[0])
        us.append(u_val)
        x = A @ x + B.flatten() * u_val
        if Br is not None:
            x = x + Br * rk
    return {"y": np.array(ys), "x": np.array(xs), "u": np.array(us)}


class _DirectFormTF:
    """Direct-form-II-like realization of a discrete transfer function num(z)/den(z)."""

    def __init__(self, num, den):
        num = np.asarray(num, dtype=float) / den[0]
        den = np.asarray(den, dtype=float) / den[0]
        n = max(len(num), len(den)) - 1
        self.num = np.concatenate([np.zeros(n + 1 - len(num)), num])
        self.den = np.concatenate([np.zeros(n + 1 - len(den)), den])
        self.w = np.zeros(n)

    def output(self):
        """Causal y[k] from the current (pre-update) state; independent of u[k] for a strictly-proper TF."""
        n = len(self.w)
        if n == 0:
            return 0.0
        return float(np.dot(self.num[1:], self.w))

    def update(self, x_in):
        """Commit input u[k], advancing the internal state to k+1."""
        n = len(self.w)
        if n == 0:
            return
        w_new = x_in - np.dot(self.den[1:], self.w)
        self.w = np.concatenate([[w_new], self.w[:-1]])

    def step(self, x_in):
        """Convenience for open-loop use: feed x_in and return y computed from the OLD state, then update."""
        n = len(self.w)
        if n == 0:
            return self.num[0] * x_in
        w_new = x_in - np.dot(self.den[1:], self.w)
        out = self.num[0] * w_new + np.dot(self.num[1:], self.w)
        self.w = np.concatenate([[w_new], self.w[:-1]])
        return out


def simulate_compensator_loop(numG, denG, numD, denD, T, ref_type="step", steps=50):
    """Discrete compensator D(z) in series with plant G(z), unity-feedback simulation."""
    plant = _DirectFormTF(numG, denG)
    ctrl = _DirectFormTF(numD, denD)

    if ref_type == "step":
        ref = np.ones(steps)
    elif ref_type == "ramp":
        ref = np.array([k * T for k in range(steps)])
    elif ref_type == "parabola":
        ref = np.array([0.5 * (k * T) ** 2 for k in range(steps)])
    else:
        ref = np.ones(steps)

    ys, us = [], []
    for k in range(steps):
        y = plant.output()
        e = ref[k] - y
        u = ctrl.step(e)
        plant.update(u)
        ys.append(y)
        us.append(u)

    return {"y": np.array(ys), "u": np.array(us), "r": ref}

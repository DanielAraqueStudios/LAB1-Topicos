"""System 3: thermal mass (theta_m) coupled via R to a water capacitance (theta_w),
which loses heat to ambient (theta_a) via R_a. Output y = heat flow theta_m -> theta_w."""
import numpy as np

ASSUMPTIONS = {
    "theta_a": "ambient temperature theta_a is taken as an incremental-zero disturbance "
               "(same convention as system4's p_a=0), so it drops out of the linear "
               "incremental state equations.",
    "numeric_values": "schematic gives no numeric R,R_a,C_m,C_w - representative values "
                       "R=1, R_a=1, C_m=1, C_w=1 [consistent units] are chosen for a "
                       "stable, well-damped, non-degenerate response (eigenvalues "
                       "-0.381966 and -2.618034, well separated).",
}

Rn, Ran, Cmn, Cwn = 1.0, 1.0, 1.0, 1.0


def get_system3_ss_symbolic(R, Ra, Cm, Cw):
    """Parametric (A,B,C,D); state=[theta_m,theta_w], input=q_i, output=y=(theta_m-theta_w)/R.

    Heat balance (theta_a=0 incremental):
      Cm*d(theta_m)/dt = q_i - (theta_m-theta_w)/R
      Cw*d(theta_w)/dt = (theta_m-theta_w)/R - (theta_w-theta_a)/Ra
    """
    A = np.array([
        [-1.0 / (R * Cm), 1.0 / (R * Cm)],
        [1.0 / (R * Cw), -(1.0 / R + 1.0 / Ra) / Cw],
    ])
    B = np.array([[1.0 / Cm], [0.0]])
    C = np.array([[1.0 / R, -1.0 / R]])
    D = np.array([[0.0]])
    return A, B, C, D


def get_system3_ss():
    """Numeric (A,B,C,D) using the chosen representative component values."""
    return get_system3_ss_symbolic(Rn, Ran, Cmn, Cwn)


if __name__ == "__main__":
    import sympy as sp
    A, B, C, D = get_system3_ss()
    eig = np.linalg.eigvals(A)
    print("A =", A)
    print("eigenvalues =", eig)
    assert np.all(eig.real < 0), "system3 operating point is degenerate/unstable"
    s = sp.symbols("s")
    An, Bn, Cn, Dn = sp.Matrix(A), sp.Matrix(B), sp.Matrix(C), sp.Matrix(D)
    G = sp.simplify((Cn * (s * sp.eye(2) - An).inv() * Bn + Dn)[0, 0])
    print("G(s) =", G)
    print("Expected: (s+1)/(s**2+3*s+1) =", sp.simplify((s + 1) / (s ** 2 + 3 * s + 1)))

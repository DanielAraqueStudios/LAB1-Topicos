"""System 4: two-tank pneumatic pressure process with linearized turbulent-flow resistances."""
import numpy as np
import sympy as sp

ASSUMPTIONS = {
    "linearized_R": "turbulent orifice flow w=k*sqrt(dP) is linearized about an "
                    "operating point as R = 2*dP0/W0; R1..R4 are given already as "
                    "these linearized resistances.",
    "numeric_values": "schematic gives no numeric component values; representative "
                       "values R1=2, R2=4, R3=1, R4=2 [pressure/flow], C1=1, C2=2 "
                       "[flow/pressure-rate] are chosen for a stable, non-degenerate, "
                       "reasonably damped response.",
}

R1n, R2n, R3n, R4n = 2.0, 4.0, 1.0, 2.0
C1n, C2n = 1.0, 2.0


def get_system4_ss_symbolic(R1, R2, R3, R4, C1, C2):
    """Parametric (A,B,C,D), state=[p1,p2], input=p_i, output=w_o=p2/R4."""
    A = np.array([
        [-(1.0 / R1 + 1.0 / R3) / C1, (1.0 / R3) / C1],
        [(1.0 / R3) / C2, -(1.0 / R2 + 1.0 / R3 + 1.0 / R4) / C2],
    ])
    B = np.array([[1.0 / (R1 * C1)], [1.0 / (R2 * C2)]])
    C = np.array([[0.0, 1.0 / R4]])
    D = np.array([[0.0]])
    return A, B, C, D


def get_system4_ss():
    """Numeric (A,B,C,D) using the chosen representative component values."""
    return get_system4_ss_symbolic(R1n, R2n, R3n, R4n, C1n, C2n)


if __name__ == "__main__":
    A, B, C, D = get_system4_ss()
    eig = np.linalg.eigvals(A)
    print("A =", A)
    print("eigenvalues =", eig)
    assert np.all(eig.real < 0), "system4 operating point is degenerate/unstable"
    s = sp.symbols("s")
    An, Bn, Cn, Dn = sp.Matrix(A), sp.Matrix(B), sp.Matrix(C), sp.Matrix(D)
    G = sp.simplify((Cn * (s * sp.eye(2) - An).inv() * Bn + Dn)[0, 0])
    print("G(s) =", G)

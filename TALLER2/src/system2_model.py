"""System 2: nonlinear-resistor RLC circuit, linearized about an operating point."""
import numpy as np

ASSUMPTIONS = {
    "I0": 1.0,
    "reason": "schematic gives no explicit DC bias for e_i(t); I0=1 A is a "
              "representative operating point consistent with the 6V ideal battery "
              "in parallel with R and C (the battery acts as an AC short in the "
              "incremental/small-signal model by superposition).",
    "R_NL": "6*I0**2 = 6 ohm (d(e_o)/di at I0, e_o = 2*i**3)",
}

L = 2.0
R = 2.0
Cap = 0.5
R_NL = 6.0


def get_system2_ss():
    """Numeric (A,B,C,D) state space, state=[i_L, v_C], output=e_o."""
    return get_system2_ss_symbolic(L, R, Cap, R_NL)


def get_system2_ss_symbolic(L, R, Cap, R_NL):
    """Parametric (A,B,C,D) given L,R,Cap,R_NL; state=[i_L,v_C], output=e_o=R_NL*i_L."""
    A = np.array([[-R_NL / L, -1.0 / L],
                  [1.0 / Cap, -1.0 / (R * Cap)]])
    B = np.array([[1.0 / L], [0.0]])
    C = np.array([[R_NL, 0.0]])
    D = np.array([[0.0]])
    return A, B, C, D


if __name__ == "__main__":
    import sympy as sp
    A, B, C, D = get_system2_ss()
    s = sp.symbols("s")
    An, Bn, Cn, Dn = (sp.Matrix(A), sp.Matrix(B), sp.Matrix(C), sp.Matrix(D))
    G = sp.simplify((Cn * (s * sp.eye(2) - An).inv() * Bn + Dn)[0, 0])
    print("G(s) =", sp.simplify(G))
    print("Expected: 3*(s+1)/(s+2)**2 =", sp.simplify(3 * (s + 1) / (s + 2) ** 2))
    print("Difference:", sp.simplify(G - 3 * (s + 1) / (s + 2) ** 2))

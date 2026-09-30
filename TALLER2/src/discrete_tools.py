"""Generic discrete-time / sampled-data control toolkit (Z-transform methods)."""
import numpy as np
import sympy as sp
from scipy.linalg import expm

from src.discrete_tools_sim import (  # noqa: F401 re-exported for backward compatibility
    simulate_discrete_closed_loop,
    simulate_compensator_loop,
    _DirectFormTF,
)


def c2d_zoh(A, B, C, D, T):
    """Exact zero-order-hold discretization via block matrix exponential."""
    A = np.atleast_2d(np.asarray(A, dtype=float))
    B = np.atleast_2d(np.asarray(B, dtype=float))
    n = A.shape[0]
    m = B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    Md = expm(M * T)
    Ad = Md[:n, :n]
    Bd = Md[:n, n:]
    return Ad, Bd, np.asarray(C, dtype=float), np.asarray(D, dtype=float)


def jury_stability(coeffs):
    """Jury test: analytic for degree<=3, numeric root-magnitude fallback otherwise."""
    coeffs = np.asarray(coeffs, dtype=float)
    coeffs = np.trim_zeros(coeffs, "f")
    n = len(coeffs) - 1
    a = coeffs / coeffs[0]
    conditions = []

    if n <= 3:
        p1 = np.polyval(a, 1.0)
        conditions.append(("P(1) > 0", p1, p1 > 0))
        pm1 = np.polyval(a, -1.0)
        sign = (-1.0) ** n
        val = sign * pm1
        conditions.append(("(-1)^n P(-1) > 0", val, val > 0))
        a0, an = a[-1], a[0]
        conditions.append(("|a0| < a_n", (abs(a0), an), abs(a0) < an))
        if n == 3:
            b0 = a[-1] ** 2 - a[0] ** 2
            b2 = a[-1] * a[1] - a[0] * a[-2]
            conditions.append(("|b0| > |b_n| (Jury row)", (abs(b0), abs(b2)), abs(b0) > abs(b2)))
        is_stable = all(c[2] for c in conditions)
    else:
        roots = np.roots(a)
        mags = np.abs(roots)
        is_stable = bool(np.all(mags < 1.0))
        conditions.append(("numeric fallback: all |root| < 1 (degree>3, no analytic Jury table)",
                            mags.tolist(), is_stable))
    return is_stable, conditions


def map_s_to_z_pole(s_pole, T):
    """z = exp(s*T) pole mapping."""
    return np.exp(np.asarray(s_pole, dtype=complex) * T)


def zeta_wn_from_s_pole(s_pole):
    """Inverse of s = -zeta*wn +- j*wn*sqrt(1-zeta^2): returns (zeta, wn)."""
    s_pole = complex(s_pole)
    wn = abs(s_pole)
    zeta = -s_pole.real / wn if wn != 0 else 0.0
    return zeta, wn


def _ss_transfer_symbolic(A, B, C, D, var):
    """C(zI-A)^-1 B + D as a simplified sympy scalar (SISO)."""
    n = A.shape[0]
    Am = sp.Matrix(A)
    Bm = sp.Matrix(B).reshape(n, 1)
    Cm = sp.Matrix(C).reshape(1, n)
    Dm = sp.Matrix(D).reshape(1, 1)
    G = (Cm * (var * sp.eye(n) - Am).inv() * Bm + Dm)[0, 0]
    return sp.simplify(sp.nsimplify(G, rational=True))


def system_type_and_error_constants(A, B, C, D, is_discrete=True, T=None):
    """Open-loop pole count at z=1 (or s=0) and Kp/Kv/Ka via limit definitions."""
    A = np.atleast_2d(np.asarray(A, dtype=float))
    B = np.atleast_2d(np.asarray(B, dtype=float))
    C = np.atleast_2d(np.asarray(C, dtype=float))
    D = np.atleast_2d(np.asarray(D, dtype=float))
    var = sp.symbols("z" if is_discrete else "s")
    G = _ss_transfer_symbolic(A, B, C, D, var)
    num, den = sp.fraction(sp.together(G))
    num = sp.expand(num)
    den = sp.expand(den)

    pole_val = 1 if is_discrete else 0
    p = sp.Poly(den, var)
    sys_type = 0
    factor = (var - pole_val) if is_discrete else var
    while True:
        q, r = sp.div(p.as_expr(), factor, var)
        if sp.simplify(r) == 0:
            sys_type += 1
            p = sp.Poly(sp.expand(q), var)
        else:
            break

    G_reduced = sp.cancel(num / den * factor ** sys_type)

    def _limit_at_pole(expr, extra_pow=0):
        e = sp.simplify(expr * factor ** extra_pow) if extra_pow else expr
        return sp.limit(e, var, pole_val)

    if is_discrete:
        Kp = complex(sp.limit(G, var, 1)) if sys_type == 0 else float("inf")
        Kv = (complex(sp.limit((var - 1) * G, var, 1)) / T if sys_type == 1
              else (float("inf") if sys_type < 1 else 0.0)) if T else None
        Ka = (complex(sp.limit((var - 1) ** 2 * G, var, 1)) / (T ** 2) if sys_type == 2
              else (float("inf") if sys_type < 2 else 0.0)) if T else None
    else:
        Kp = complex(sp.limit(G, var, 0)) if sys_type == 0 else float("inf")
        Kv = complex(sp.limit(var * G, var, 0)) if sys_type <= 1 else float("inf")
        Ka = complex(sp.limit(var ** 2 * G, var, 0)) if sys_type <= 2 else float("inf")

    def _clean(x):
        if isinstance(x, complex):
            return x.real if abs(x.imag) < 1e-9 else x
        return x

    return {
        "type": sys_type,
        "Kp": _clean(Kp),
        "Kv": _clean(Kv),
        "Ka": _clean(Ka),
    }


def design_compensator(numG, denG, integrators, extra_poles, desired_poles):
    """Pole-placement compensator design by symbolic coefficient matching."""
    z = sp.symbols("z")
    numG = sp.Poly(list(numG), z)
    denG = sp.Poly(list(denG), z)

    fixed_den = sp.Poly([1], z)
    for _ in range(integrators):
        fixed_den = fixed_den * sp.Poly([1, -1], z)
    for p in extra_poles:
        fixed_den = fixed_den * sp.Poly([1, -p], z)

    deg_desired = len(desired_poles)
    deg_num_ctrl = max(deg_desired - fixed_den.degree() - 1, 0)
    qs = sp.symbols(f"q0:{deg_num_ctrl + 1}")
    qs = qs if isinstance(qs, tuple) else (qs,)
    num_ctrl = sp.Poly(list(qs), z)

    charpoly = sp.expand(fixed_den.as_expr() * denG.as_expr() + num_ctrl.as_expr() * numG.as_expr())
    desired = sp.Poly(1, z)
    for p in desired_poles:
        desired = desired * sp.Poly([1, -p], z)
    desired_poly = sp.Poly(sp.expand(desired.as_expr()), z)
    real_coeffs = [sp.re(sp.nsimplify(c, rational=False)) for c in desired_poly.all_coeffs()]
    desired_expr = sum(c * z ** k for c, k in zip(real_coeffs[::-1], range(len(real_coeffs))))

    deg = max(sp.degree(desired_expr, z), sp.degree(charpoly, z))
    eqs = [sp.expand((charpoly.coeff(z, k) if charpoly != 0 else 0)
                      - (desired_expr.coeff(z, k) if desired_expr != 0 else 0))
           for k in range(deg + 1)]

    # linear system A*q = b in the unknown numerator coefficients
    Amat, bvec = sp.linear_eq_to_matrix(eqs, list(qs))
    Amat = np.array(Amat.tolist(), dtype=float)
    bvec = -np.array(bvec.tolist(), dtype=float).flatten()

    sol = sp.solve(eqs, list(qs), dict=True)
    if sol:
        sol = sol[0]
        q_vals = [complex(sol.get(q, 0)) for q in qs]
    else:
        # overdetermined/inconsistent for this fixed-denominator structure: least-squares fit
        q_vals, *_ = np.linalg.lstsq(Amat, bvec, rcond=None)
        q_vals = [complex(v) for v in q_vals]

    num_ctrl_vals = [v.real if abs(v.imag) < 1e-9 else v for v in q_vals]

    den_ctrl_poly = sp.Poly(sp.expand(fixed_den.as_expr()), z)
    den_ctrl_coeffs = [float(c) for c in den_ctrl_poly.all_coeffs()]

    subs_map = dict(zip(qs, num_ctrl_vals))
    closed_charpoly = sp.expand(charpoly.subs(subs_map))
    closed_poly = sp.Poly(closed_charpoly, z)
    closed_coeffs = [complex(c) for c in closed_poly.all_coeffs()]
    closed_coeffs = [c.real if abs(c.imag) < 1e-9 else c for c in closed_coeffs]

    return {
        "num_controller": num_ctrl_vals,
        "den_controller": den_ctrl_coeffs,
        "closed_loop_charpoly": closed_coeffs,
        "exact": bool(sol),
    }


def design_deadbeat(numG, denG, integrators):
    """Deadbeat design: all desired closed-loop poles placed at z=0."""
    n = len(denG) - 1 + integrators
    desired_poles = [0.0] * n
    return design_compensator(numG, denG, integrators, [], desired_poles)


def design_deadbeat_diophantine(numG, denG, integrators, extra_free_poles_n=None, target_pole=0.0):
    """Deadbeat / pole-placement via a Diophantine (Bezout) equation with a FREE
    controller denominator, for cases where the minimal fixed-denominator structure
    used by design_deadbeat (D(z)=N(z)/(z-1)^integrators) is underdetermined.

    Solves simultaneously for N(z) (degree n_c) and a monic extra denominator factor
    D_extra(z) (degree n_c, unknown coefficients) in:

        (z-1)^integrators * D_extra(z) * denG(z) + N(z) * numG(z) = (z - target_pole)^deg

    with deg = integrators + n_c + deg(denG). Both N and D_extra's coefficients enter
    linearly, so the whole system is a single linear solve (2*n_c+1 unknowns).
    n_c defaults to integrators + deg(denG) - 1, the smallest order that matches the
    number of unknowns to the number of closed-loop-root equations.
    """
    z = sp.symbols("z")
    numG_p = sp.Poly(list(numG), z)
    denG_p = sp.Poly(list(denG), z)
    n_g = denG_p.degree()
    n_c = integrators + n_g - 1 if extra_free_poles_n is None else extra_free_poles_n
    n_c = max(n_c, 0)

    fixed = sp.Poly([1], z)
    for _ in range(integrators):
        fixed = fixed * sp.Poly([1, -1], z)

    ds = sp.symbols(f"d0:{n_c}") if n_c > 0 else ()
    ds = ds if isinstance(ds, tuple) else (ds,)
    d_extra = sp.Poly([sp.Integer(1)] + list(ds), z) if n_c > 0 else sp.Poly([1], z)

    qs = sp.symbols(f"q0:{n_c + 1}")
    qs = qs if isinstance(qs, tuple) else (qs,)
    num_ctrl = sp.Poly(list(qs), z)

    full_den_expr = sp.expand(fixed.as_expr() * d_extra.as_expr())
    charpoly = sp.expand(full_den_expr * denG_p.as_expr() + num_ctrl.as_expr() * numG_p.as_expr())

    deg = integrators + n_c + n_g
    target_expr = sp.expand((z - target_pole) ** deg)

    unknowns = list(qs) + list(ds)
    eqs = [sp.expand(charpoly.coeff(z, k) - target_expr.coeff(z, k)) for k in range(deg + 1)]

    Amat, bvec = sp.linear_eq_to_matrix(eqs, unknowns)
    Amat = np.array(Amat.tolist(), dtype=float)
    bvec = -np.array(bvec.tolist(), dtype=float).flatten()

    sol = sp.solve(eqs, unknowns, dict=True)
    if sol:
        sol = sol[0]
        vals = [complex(sol.get(u, 0)) for u in unknowns]
    else:
        vals, *_ = np.linalg.lstsq(Amat, bvec, rcond=None)
        vals = [complex(v) for v in vals]
    vals = [v.real if abs(v.imag) < 1e-9 else v for v in vals]

    subs_map = dict(zip(unknowns, vals))
    q_vals = vals[: n_c + 1]
    d_vals = vals[n_c + 1:]

    den_ctrl_expr = sp.expand(full_den_expr.subs(dict(zip(ds, d_vals))))
    den_ctrl_poly = sp.Poly(den_ctrl_expr, z)
    den_ctrl_coeffs = [complex(c) for c in den_ctrl_poly.all_coeffs()]
    den_ctrl_coeffs = [c.real if abs(c.imag) < 1e-9 else c for c in den_ctrl_coeffs]

    closed_charpoly = sp.expand(charpoly.subs(subs_map))
    closed_poly = sp.Poly(closed_charpoly, z)
    closed_coeffs = [complex(c) for c in closed_poly.all_coeffs()]
    closed_coeffs = [c.real if abs(c.imag) < 1e-9 else c for c in closed_coeffs]

    return {
        "num_controller": q_vals,
        "den_controller": den_ctrl_coeffs,
        "n_c": n_c,
        "closed_loop_charpoly": closed_coeffs,
        "exact": bool(sol),
    }


def _ctrb(A, B):
    n = A.shape[0]
    cols = [B]
    for i in range(1, n):
        cols.append(np.linalg.matrix_power(A, i) @ B)
    return np.hstack(cols)


def ackermann_place(A, B, desired_poles):
    """State-feedback gain K via Ackermann's formula (SISO)."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(-1, 1)
    n = A.shape[0]
    Ctrb = _ctrb(A, B)
    z = sp.symbols("z")
    charpoly = sp.Poly(1, z)
    for p in desired_poles:
        charpoly = charpoly * sp.Poly([1, -p], z)
    coeffs = [complex(c) for c in charpoly.all_coeffs()]
    coeffs = [c.real if abs(c.imag) < 1e-9 else c for c in coeffs]
    phi = np.zeros((n, n))
    An = np.eye(n)
    for c in coeffs[::-1]:
        phi = phi + c * An
        An = An @ A
    e_n = np.zeros((1, n))
    e_n[0, -1] = 1.0
    K = e_n @ np.linalg.inv(Ctrb) @ phi
    return K


def _bass_gura_gain(A, B, desired_poles):
    """Bass-Gura pole-placement gain (SISO), cross-check for Ackermann's formula."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(-1, 1)
    n = A.shape[0]
    Ctrb = _ctrb(A, B)
    a = np.poly(A)[1:]  # open-loop char. poly coeffs a1..an (monic, s^n term implicit)

    z = sp.symbols("z")
    poly = sp.Poly(1, z)
    for p in desired_poles:
        poly = poly * sp.Poly([1, -p], z)
    alpha = [complex(c) for c in poly.all_coeffs()]
    alpha = np.array([c.real if abs(c.imag) < 1e-9 else c for c in alpha])[1:]  # alpha1..alphan

    c = np.zeros(n)
    c[-1] = 1.0
    for m in range(n - 1):
        c[m] = a[n - 2 - m]
    W = np.zeros((n, n))
    for i in range(n):
        for k in range(n):
            if k + i < n:
                W[i, k] = c[k + i]

    T = Ctrb @ W
    diff = (alpha - a)[::-1]  # [alphan-an, ..., alpha1-a1]
    K = diff.reshape(1, -1) @ np.linalg.inv(T)
    return K


def observer_poles_ts_rule(desired_closed_loop_poles, factor=10.0):
    """Observer s-plane poles per tsdo = tsd/factor (default factor=10, faster observer)."""
    return [factor * p for p in desired_closed_loop_poles]


def luenberger_observer_gain(A, C, desired_poles):
    """Observer gain L via Ackermann duality on (A^T, C^T), cross-checked with Bass-Gura."""
    A = np.asarray(A, dtype=float)
    C = np.asarray(C, dtype=float).reshape(1, -1)
    Kt = ackermann_place(A.T, C.T, desired_poles)
    Kt_bg = _bass_gura_gain(A.T, C.T, desired_poles)
    if not np.allclose(Kt, Kt_bg, rtol=1e-4, atol=1e-6):
        raise AssertionError(
            "luenberger_observer_gain: Ackermann and Bass-Gura results disagree "
            f"(Ackermann={Kt}, Bass-Gura={Kt_bg}) - check observability/desired poles"
        )
    return Kt.T


def augment_with_integrator(A, B, C):
    """Augment discrete plant with an integral-of-error state xi[k+1]=xi[k]+(r-y). Legacy/step-only form."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(-1, 1)
    C = np.asarray(C, dtype=float).reshape(1, -1)
    n = A.shape[0]
    Aa = np.zeros((n + 1, n + 1))
    Aa[:n, :n] = A
    Aa[n, :n] = -C
    Aa[n, n] = 1.0
    Ba = np.zeros((n + 1, 1))
    Ba[:n, 0] = B[:, 0]
    Ca = np.zeros((1, n + 1))
    Ca[0, :n] = C
    return Aa, Ba, Ca


def augment_for_tracking(A, B, C, order=1):
    """Augment plant (Ghat,Hhat,Chat) with `order` cascaded tracking states [v,w,z,...].

    order=1: escalon (Type+1), order=2: rampa (Type+2), order=3: parabola (Type+3), per the
    course's whiteboard: row j (0=v outermost .. order-1=z innermost) is
    x_j(k+1) = -C@G@x(k) + sum_{k'>=j} x_{k'}(k), with x(k+1)=G@x(k)+H@u(k)+r injected via Hhat.
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(-1, 1)
    C = np.asarray(C, dtype=float).reshape(1, -1)
    n = A.shape[0]
    m = order
    CG = C @ A
    CH = float((C @ B)[0, 0])

    Aa = np.zeros((n + m, n + m))
    Aa[:n, :n] = A
    for j in range(m):
        Aa[n + j, :n] = -CG
        Aa[n + j, n + j:n + m] = 1.0
    Ba = np.zeros((n + m, 1))
    Ba[:n, 0] = B[:, 0]
    Ba[n:n + m, 0] = -CH
    Ca = np.zeros((1, n + m))
    Ca[0, :n] = C
    return Aa, Ba, Ca


def design_lead_lag_angle(numG, denG, T, zeta, wn, max_section_angle_deg=90.0):
    """Root-locus angle-criterion lead/lag compensator design (discrete z-plane).

    Places the dominant closed-loop pole z_d from the discrete 2nd-order spec
    P(z)=z^2+p1 z+p2 (p1,p2 from zeta,wn,T), measures the plant's angle deficiency
    at z_d, splits it into `n_sections` identical real-axis lead/lag sections
    (zero fixed at Re(z_d), pole solved from the angle condition), and sets the
    overall gain from the magnitude criterion |C(z_d) G(z_d)|=1.
    """
    numG = np.asarray(numG, dtype=float)
    denG = np.asarray(denG, dtype=float)

    p1 = -2.0 * np.exp(-zeta * wn * T) * np.cos(wn * T * np.sqrt(1.0 - zeta ** 2))
    p2 = np.exp(-2.0 * zeta * wn * T)
    roots = np.roots([1.0, p1, p2])
    z_d = roots[np.argmax(roots.imag)]

    def _eval(num, den, z):
        return np.polyval(num, z) / np.polyval(den, z)

    F1 = _eval(numG, denG, z_d)
    alpha = float(np.degrees(np.angle(F1)))
    theta = ((180.0 - alpha + 180.0) % 360.0) - 180.0

    n_sections = max(1, int(np.ceil(abs(theta) / max_section_angle_deg)))
    theta_i = theta / n_sections

    # "vertical zero" placement zc=Re(z_d): with |theta_i|<=max_section_angle_deg<=90
    # (guaranteed by the section split above) the required pole angle 90-theta_i
    # always lies in (0,180), i.e. a real-axis pc always exists. Guaranteed feasible.
    zc = float(z_d.real)
    ang = np.radians(90.0 - theta_i)
    if abs(np.sin(ang)) < 1e-9:
        raise ValueError("design_lead_lag_angle: degenerate angle placement, "
                          "reduce max_section_angle_deg")
    pc = float(z_d.real - z_d.imag / np.tan(ang))
    sections = [(zc, pc, theta_i)] * n_sections

    ratio = (z_d - zc) / (z_d - pc)
    K_c = 1.0 / (abs(F1) * abs(ratio) ** n_sections)

    num_c = K_c * np.poly([zc] * n_sections)
    den_c = np.poly([pc] * n_sections)

    loop = _eval(num_c, den_c, z_d) * F1
    loop_deg = float(np.degrees(np.angle(loop)))
    angle_error = min(abs(((loop_deg - 180.0 + 180.0) % 360.0) - 180.0),
                       abs(((loop_deg + 180.0 + 180.0) % 360.0) - 180.0))
    magnitude_residual = abs(abs(loop) - 1.0)

    assert angle_error < 0.5, f"design_lead_lag_angle: angle criterion failed ({angle_error} deg)"
    assert magnitude_residual < 1e-3, (
        f"design_lead_lag_angle: magnitude criterion failed (residual {magnitude_residual})")

    return {
        "z_d": z_d,
        "alpha_deg": alpha,
        "theta_total_deg": theta,
        "n_sections": n_sections,
        "sections": sections,
        "K_c": K_c,
        "num_c": num_c.tolist(),
        "den_c": den_c.tolist(),
        "angle_error_deg": angle_error,
        "magnitude_residual": magnitude_residual,
    }


def design_type_compensator(numG, denG, T, order, zeta, wn, max_section_angle_deg=90.0,
                             wn_scale_factors=(1.0, 0.75, 0.5, 0.375, 0.25, 0.1875, 0.125, 0.0625)):
    """Angle-criterion lead/lag compensator that also guarantees the tracking Type.

    `design_lead_lag_angle` alone only places the dominant closed-loop pole pair; when
    an integrator cascade `1/(z-1)**order` is added (to satisfy escalon/rampa/parabola
    Type requirements) the OTHER closed-loop poles are left uncontrolled and can go
    unstable even though the dominant-pair angle/magnitude criteria are satisfied.
    This wrapper accounts for the integrator phase in the angle criterion (by running
    it on the extended plant G(z)/(z-1)**order) and, since a too-aggressive dominant-pole
    spec is what destabilizes the non-dominant poles in practice, retries with a slower
    wn (same zeta) until the FULL closed-loop characteristic polynomial (not just the
    dominant pair) is verified Jury-stable.
    """
    numG = np.asarray(numG, dtype=float)
    denG = np.asarray(denG, dtype=float)
    denG_ext = denG.copy()
    for _ in range(order):
        denG_ext = np.polymul(denG_ext, [1.0, -1.0])

    last_exc = None
    for scale in wn_scale_factors:
        try:
            lead = design_lead_lag_angle(numG, denG_ext, T=T, zeta=zeta, wn=wn * scale,
                                          max_section_angle_deg=max_section_angle_deg)
        except (ValueError, AssertionError) as exc:
            last_exc = exc
            continue
        num_ctrl = np.array(lead["num_c"], dtype=float)
        den_ctrl = np.array(lead["den_c"], dtype=float)
        for _ in range(order):
            den_ctrl = np.polymul(den_ctrl, [1.0, -1.0])
        closed = np.polyadd(np.polymul(den_ctrl, denG), np.polymul(num_ctrl, numG))
        stable, _ = jury_stability(closed)
        if stable:
            return {
                "num_controller": num_ctrl.tolist(),
                "den_controller": den_ctrl.tolist(),
                "closed_loop_charpoly": closed.tolist(),
                "wn_used": wn * scale,
                "wn_scale_used": scale,
                "lead_design": lead,
            }
    raise RuntimeError(
        "design_type_compensator: no wn scale factor in wn_scale_factors produced a "
        f"Jury-stable closed loop for order={order} (last attempt raised: {last_exc})"
    )

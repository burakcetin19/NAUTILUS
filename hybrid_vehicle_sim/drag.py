"""Direnc modelleri: fiziksel referans (ITTC-1957 + form faktoru, capraz akis) ve Stonefish
implementasyonu (Faz 0.5: F_p = 1/2 rho C_d A |u|^3, F_f = rho C_f S_t u), ek kutle tahminleri."""
import numpy as np
from scipy import optimize


def ittc57_cf(re):
    return 0.075 / (np.log10(re) - 2.0) ** 2


def hoerner_form_factor(d, l):
    """1 + k = 1 + 1.5 (D/L)^1.5 + 7 (D/L)^3 (govde-of-revolution, Hoerner 1965)."""
    return 1.5 * (d / l) ** 1.5 + 7.0 * (d / l) ** 3


def physical_surge(u, s_wet, length, k, rho, nu):
    """Donus: (toplam, surtunme, basinc) [N]; surtunme = 1/2 rho S C_F u^2, basinc = k * surtunme."""
    u = np.abs(np.asarray(u, dtype=float))
    re = np.maximum(u * length / nu, 1e3)
    fric = 0.5 * rho * s_wet * ittc57_cf(re) * u**2
    return fric * (1 + k), fric, fric * k


def cross_flow(w, cd, a_plan, rho):
    return 0.5 * rho * cd * a_plan * np.asarray(w, dtype=float) ** 2


def stonefish_force(u, cd, cf, a_proj, s_t, rho):
    """Saf eksen hareketi icin Stonefish: (F_p, F_f) buyuklukleri [N]."""
    u = np.abs(np.asarray(u, dtype=float))
    return 0.5 * rho * cd * a_proj * u**3, rho * cf * s_t * u


def fit_cubic_linear(u, target):
    """a*u^3 + b*u ~ target: bant icinde max |goreli hata| en kucuk (minimax), goreli en kucuk
    kareler cozumunden baslayarak. Donus: a, b, max |goreli hata|."""
    u, target = np.asarray(u), np.asarray(target)
    A = np.vstack([u**3 / target, u / target]).T
    x0 = np.linalg.lstsq(A, np.ones_like(u), rcond=None)[0]
    worst = lambda x: np.abs((x[0] * u**3 + x[1] * u) / target - 1).max()
    res = optimize.minimize(worst, x0, method='Nelder-Mead',
                            options=dict(xatol=1e-12, fatol=1e-12, maxiter=5000))
    x = res.x if res.fun < worst(x0) else x0
    return x[0], x[1], worst(x)


def calibrate(u_band, target, a_proj, s_t, rho, n=121):
    u = np.linspace(*u_band, n)
    a, b, err = fit_cubic_linear(u, target(u))
    return dict(cd=2 * a / (rho * a_proj), cf=b / (rho * s_t), a=a, b=b, max_rel_err=err, band=list(u_band))


def lamb_prolate(l, d):
    """Fiziksel prolate sferoid ek kutle katsayilari k1 (eksenel), k2 (yanal), Lamb (1932)."""
    e = np.sqrt(1 - (d / l) ** 2)
    lg = np.log((1 + e) / (1 - e))
    a0 = 2 * (1 - e * e) / e**3 * (0.5 * lg - e)
    b0 = 1 / e**2 - (1 - e * e) / (2 * e**3) * lg
    return a0 / (2 - a0), b0 / (2 - b0)


def stonefish_added_mass(a, b, c, rho):
    """Stonefish SolidEntity::ComputeEllipsoidalApprox + LambKFactor (SolidEntity.cpp:1000-1039),
    kodtaki haliyle (e = 1 - r2^2/r1). Yari-eksenler MVAE fitinden gelir; burada tahmin."""
    r12 = (b + c) / 2
    e = 1 - r12 * r12 / a
    k = 1.0
    if (1 + e) / (1 - e) > 0:
        a0 = 2 * (1 - e * e) / (e * e) * (0.5 * np.log((1 + e) / (1 - e)) - e)
        k = a0 / (2 - a0)
    mx = k * 4 / 3 * np.pi * rho * a * r12 * r12
    return np.array([mx, 4 / 3 * np.pi * rho * c * c * a, 4 / 3 * np.pi * rho * b * b * a])

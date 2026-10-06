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
    kodtaki haliyle (e = 1 - r2^2/r1). Yari-eksenler: mvae.stonefish_ellipsoid."""
    r12 = (b + c) / 2
    e = 1 - r12 * r12 / a
    k = 1.0
    if (1 + e) / (1 - e) > 0:
        a0 = 2 * (1 - e * e) / (e * e) * (0.5 * np.log((1 + e) / (1 - e)) - e)
        k = a0 / (2 - a0)
    mx = k * 4 / 3 * np.pi * rho * a * r12 * r12
    return np.array([mx, 4 / 3 * np.pi * rho * c * c * a, 4 / 3 * np.pi * rho * b * b * a])


def stonefish_added_inertia(a, b, c, rho):
    """Stonefish ek ataleti (SolidEntity.cpp:1004-1006): roll icin 0 (kodda 'THIS SHOULD BE > 0'),
    pitch pi rho b^2 a^3 / 12, yaw pi rho c^2 a^3 / 12. Robotta I_aug = I + aI (:665-669)."""
    return np.array([0.0, np.pi * rho * b * b * a**3 / 12, np.pi * rho * c * c * a**3 / 12])


def stonefish_face_forces(p1, p2, p3, cg, v, omega, cd, cf, rho):
    """Tam dalmis govdede Stonefish yuz bazli direnc (SolidEntity.cpp:1723-1797) + katsayi duzeltmesi
    (CorrectHydrodynamicForces, :1263-1287), govde (origin) ekseninde, durgun akiskan.
    p1..p3: (n, 3) yuz koseleri [m] (origin ekseni), cg: CG [m], v/omega: CG hizi ve acisal hiz (govde ekseni).
    Etkin katsayi C_eff = sum |d_i| C_i, d = ham kuvvet (ya da tork) yonu: egik akista L1 karisimi C_eff'i
    eksen degerinin ustune cikarir; tork icin tork yonu kullanilir (roll torku -> C_x).
    Donus: F_p (form, |v|^3), F_f (surtunme, v), T_p, T_f (CG etrafinda) [N, N m]."""
    fn = np.cross(p2 - p1, p3 - p1)
    ln = np.linalg.norm(fn, axis=1)
    keep = ln > 1e-6
    n1, area = fn[keep] / ln[keep, None], ln[keep] / 2
    r = (p1[keep] + p2[keep] + p3[keep]) / 3 - np.asarray(cg)
    vc = -(np.asarray(v) + np.cross(omega, r))
    vcn = np.einsum('ij,ij->i', vc, n1)
    vt = vc - vcn[:, None] * n1
    front = vcn < -1e-12
    q = vc[front] * np.linalg.norm(vc[front], axis=1)[:, None] * (-vcn[front] * area[front])[:, None]
    tmag = np.einsum('ij,ij->i', vt, vt) > 1e-9
    sk = vt[tmag] * area[tmag, None]
    raw = dict(Fp=q.sum(0), Tp=np.cross(r[front], q).sum(0), Ff=sk.sum(0), Tf=np.cross(r[tmag], sk).sum(0))

    def coeff(x, c):
        nrm = np.linalg.norm(x)
        return float(np.abs(x / nrm) @ np.asarray(c)) if nrm > 0 else 0.0
    return (0.5 * rho * coeff(raw['Fp'], cd) * raw['Fp'], rho * coeff(raw['Ff'], cf) * raw['Ff'],
            0.5 * rho * coeff(raw['Tp'], cd) * raw['Tp'], rho * coeff(raw['Tf'], cf) * raw['Tf'])

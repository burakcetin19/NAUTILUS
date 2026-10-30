"""Hidrostatik: govde-x dilimleme + vektorel cokgen kirpma (Green teoremi).

Mesh kesitleri halkalar arasinda yaricapi dogrusal degisen duzgun N-genlerdir; bu yuzden
dilimleme, Stonefish'in kullandigi ucgen mesh ile ayni geometriyi bagimsiz bir yontemle olcer.
Dunya: NED, serbest yuzey z=0, z>0 su alti. Poz: govde orijininin derinligi z0 ve R (govde->dunya).
"""
import numpy as np
from scipy import optimize


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def clip_polys(P, d):
    """P: (Nx, Nt, 2) CCW cokgenler (govde y, z); d: (Nx, Nt) kose derinligi.
    d >= 0 bolgesinin alani ve birinci momentleri (int y dA, int z dA)."""
    Pj, dj = np.roll(P, -1, axis=1), np.roll(d, -1, axis=1)
    wi, wj = d >= 0, dj >= 0
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(wi != wj, d / (d - dj), 0.0)
    Q = P + t[..., None] * (Pj - P)
    a = np.where(wi[..., None], P, Q)
    b = np.where(wj[..., None], Pj, Q)

    def green(a, b, mask):
        cr = np.where(mask, a[..., 0] * b[..., 1] - b[..., 0] * a[..., 1], 0.0)
        return (0.5 * cr.sum(-1), ((a[..., 0] + b[..., 0]) * cr).sum(-1) / 6,
                ((a[..., 1] + b[..., 1]) * cr).sum(-1) / 6)
    A, My, Mz = green(a, b, wi | wj)
    exit_m, entry_m = wi & ~wj, ~wi & wj                   # konveks: en fazla bir cikis/giris
    E = (Q * exit_m[..., None]).sum(1)
    N = (Q * entry_m[..., None]).sum(1)
    cA, cMy, cMz = green(E[:, None, :], N[:, None, :], exit_m.any(1)[:, None])   # su hatti kirisi
    return A + cA, My + cMy, Mz + cMz


class Slicer:
    def __init__(self, rx, rr, n_theta, n_uniform=4001):
        self.x = np.unique(np.concatenate([rx, np.linspace(rx[0], rx[-1], n_uniform)]))
        self.r = np.interp(self.x, rx, rr)
        th = 2 * np.pi * np.arange(n_theta) / n_theta
        self.P = np.stack([self.r[:, None] * np.cos(th), self.r[:, None] * np.sin(th)], axis=-1)

    def submerged(self, z0, R=np.eye(3)):
        """Donus: V_sub [m3], CB_sub (govde ekseninde)."""
        d = z0 + R[2, 0] * self.x[:, None] + R[2, 1] * self.P[..., 0] + R[2, 2] * self.P[..., 1]
        A, My, Mz = clip_polys(self.P, d)
        V = np.trapz(A, self.x)
        if V <= 0:
            return 0.0, np.zeros(3)
        return V, np.array([np.trapz(self.x * A, self.x), np.trapz(My, self.x), np.trapz(Mz, self.x)]) / V

    def full(self):
        return self.submerged(1e3)

    def equilibrium_depth(self, v_target, R=np.eye(3), half_span=1.0):
        return optimize.brentq(lambda z: self.submerged(z, R)[0] - v_target, -half_span, half_span, xtol=1e-10)


def righting_arm(sl, mass, rho, g, cg, angle, axis, surface):
    """GZ [m] (pozitif = dogrultucu). axis: 'roll' (govde x etrafinda) veya 'pitch' (y etrafinda).
    surface=True: serbest batma ile yuzey dengesi; False: tam dalmis."""
    R = rot_x(angle) if axis == 'roll' else rot_y(angle)
    if surface:
        z0 = sl.equilibrium_depth(mass / rho, R)
        V, cb = sl.submerged(z0, R)
    else:
        V, cb = sl.full()
    lever = R @ (np.asarray(cb) - np.asarray(cg))
    M = np.cross(lever, [0.0, 0.0, -rho * g * V])          # CG etrafinda kaldirma momenti (dunya)
    return -M[0 if axis == 'roll' else 1] / (mass * g)


def surface_state(sl, mass, rho, cg, r):
    """Dik yuzey dengesi: eksen derinligi, su cekimi, fribord, CB_sub, statik trim."""
    z0 = sl.equilibrium_depth(mass / rho)
    V, cb = sl.submerged(z0)
    trim = optimize.brentq(lambda th: righting_arm(sl, mass, rho, 9.81, cg, th, 'pitch', True),
                           np.radians(-10), np.radians(10), xtol=1e-9)
    return dict(axis_depth=z0, draft=z0 + r, freeboard=r - z0, v_sub=V, cb_sub=cb.tolist(), trim_rad=trim)


def waterplane(x, r_of_x, z0):
    """Dik durumda su hatti alani ve atalet momentleri (dairesel kesit, eksen derinligi z0)."""
    b = 2 * np.sqrt(np.clip(r_of_x**2 - z0**2, 0, None))
    A = np.trapz(b, x)
    xf = np.trapz(b * x, x) / A if A > 0 else 0.0
    return dict(area=A, x_f=xf, I_T=np.trapz(b**3 / 12, x), I_L=np.trapz(b * (x - xf) ** 2, x),
                length=np.ptp(x[b > 0]) if (b > 0).any() else 0.0, beam=b.max())

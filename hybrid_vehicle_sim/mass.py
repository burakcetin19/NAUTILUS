"""Kutle modeli: homojen govde (mesh'ten) + nokta balast."""
import numpy as np


def solid_inertia(V, F):
    """Birim yogunluklu kapali mesh: hacim, agirlik merkezi, merkez etrafinda atalet tensoru."""
    p1, p2, p3 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    det = np.einsum('ij,ij->i', p1, np.cross(p2, p3))          # 6 * tetra hacmi (orijin tepe)
    vol = det.sum() / 6
    c = ((p1 + p2 + p3) / 4 * det[:, None]).sum(0) / det.sum()
    s = p1 + p2 + p3
    outer = lambda u: np.einsum('ij,ik->ijk', u, u)
    C = (det[:, None, None] / 120 * (outer(p1) + outer(p2) + outer(p3) + outer(s))).sum(0)  # int x x^T dV
    Cc = C - vol * np.outer(c, c)
    return vol, c, np.trace(Cc) * np.eye(3) - Cc


def vehicle_mass(V, F, total_mass, ballast_mass, bg, r):
    """Govde (m - m_b, homojen) + balast (m_b, x = x_CB, z = BG*m/m_b). CG = CB + (0, 0, BG)."""
    vol, cb, I_unit = solid_inertia(V, F)
    m_h = total_mass - ballast_mass
    z_b = bg * total_mass / ballast_mass
    cg = cb + np.array([0.0, 0.0, bg])
    ballast = cb + np.array([0.0, 0.0, z_b])
    I = I_unit * m_h / vol
    for m, p in ((m_h, cb), (ballast_mass, ballast)):          # paralel eksen, CG'ye gore
        d = p - cg
        I = I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return dict(mass=total_mass, hull_mass=m_h, ballast_mass=ballast_mass, ballast_pos=ballast.tolist(),
                ballast_inside=bool(z_b < 0.8 * r), cg=cg.tolist(), cb=cb.tolist(), inertia=I,
                principal=np.allclose(I, np.diag(np.diag(I)), atol=1e-9 * np.abs(I).max()))

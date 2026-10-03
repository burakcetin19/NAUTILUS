"""Thruster yerlesimi: Stonefish aktuator orijini (xyz, rpy) ve itki dagitim (allocation) matrisi."""
import numpy as np

AXES = ['X', 'Y', 'Z', 'K', 'M', 'N']


def direction_to_rpy(d):
    """Aktuator +x eksenini d yonune ceviren rpy (Stonefish/Bullet ZYX)."""
    d = np.asarray(d, dtype=float) / np.linalg.norm(d)
    return [0.0, float(-np.arcsin(d[2])) + 0.0, float(np.arctan2(d[1], d[0])) + 0.0]


def resolve(units, cb):
    out = []
    for u in units:
        p = np.array(u['position'], dtype=float)
        if u.get('x_relative_to_cb'):
            p[0] += cb[0]
        d = np.array(u['direction'], dtype=float)
        out.append(dict(name=u['name'], position=p, direction=d / np.linalg.norm(d), rpy=direction_to_rpy(d)))
    return out


def allocation(units, cg):
    """6 x n: tau = B @ T. Satirlar X Y Z (kuvvet), K M N (CG etrafinda moment), govde ekseni."""
    B = np.zeros((6, len(units)))
    for i, u in enumerate(units):
        B[:3, i] = u['direction']
        B[3:, i] = np.cross(u['position'] - np.asarray(cg), u['direction'])
    B[np.abs(B) < 1e-12] = 0.0
    return B


def axis_capacity(B, t_max):
    """Her eksende tek basina ulasilabilecek max |kuvvet/moment| (thrusterlar +-t_max)."""
    return np.abs(B).sum(1) * t_max

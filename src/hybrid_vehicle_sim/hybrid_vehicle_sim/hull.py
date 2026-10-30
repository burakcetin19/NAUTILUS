"""Tek parca torpido govdesi: profil, kapali mesh uretimi ve geometrik ozellikler.

Profil (x ekseni boyunca, orijin toplam boyun ortasinda, burun +x):
kic yari-elipsoidi (tail_length) + silindir (mid_length) + burun yari-elipsoidi (nose_length).
Mesh: halka-halka dortgenler + uclarda yelpaze; disa donuk CCW sarim (Stonefish normali
(p2-p1)x(p3-p1) ve isaretli tetrahedron hacmi bu sirayi kullanir).
"""
from collections import Counter
from dataclasses import dataclass
import numpy as np


@dataclass
class HullParams:
    diameter: float
    nose_length: float
    mid_length: float
    tail_length: float
    n_theta: int = 64
    n_cap: int = 24
    mid_step: float = 0.05

    @classmethod
    def from_dict(cls, d):
        m = d.get('mesh', {})
        return cls(d['diameter'], d['nose_length'], d['mid_length'], d['tail_length'],
                   m.get('n_theta', 64), m.get('n_cap', 24), m.get('mid_step', 0.05))

    @property
    def r(self):
        return self.diameter / 2

    @property
    def length(self):
        return self.nose_length + self.mid_length + self.tail_length

    @property
    def x_tail_joint(self):
        return -self.length / 2 + self.tail_length

    @property
    def x_nose_joint(self):
        return self.x_tail_joint + self.mid_length


def radius(p, x):
    """Analitik profil r(x)."""
    x = np.asarray(x, dtype=float)
    r = np.full_like(x, p.r)
    t = x < p.x_tail_joint
    r[t] = p.r * np.sqrt(np.clip(1 - ((x[t] - p.x_tail_joint) / p.tail_length) ** 2, 0, None))
    n = x > p.x_nose_joint
    r[n] = p.r * np.sqrt(np.clip(1 - ((x[n] - p.x_nose_joint) / p.nose_length) ** 2, 0, None))
    return r


def rings(p):
    """Kic ucundan buruna halka (x, r) listesi; uc noktalar haric."""
    out = []
    for i in range(1, p.n_cap + 1):
        psi = i * (np.pi / 2) / p.n_cap
        out.append((p.x_tail_joint - p.tail_length * np.cos(psi), p.r * np.sin(psi)))
    n_mid = max(1, int(round(p.mid_length / p.mid_step)))
    for i in range(1, n_mid + 1):
        out.append((p.x_tail_joint + i * p.mid_length / n_mid, p.r))
    for i in range(1, p.n_cap):
        psi = (p.n_cap - i) * (np.pi / 2) / p.n_cap
        out.append((p.x_nose_joint + p.nose_length * np.cos(psi), p.r * np.sin(psi)))
    return out


def ring_profile(p):
    """Uc noktalar dahil (x, r) dizileri: mesh kesitleri bunlar arasinda dogrusal."""
    rg = rings(p)
    rx = np.concatenate([[-p.length / 2], [x for x, _ in rg], [p.length / 2]])
    rr = np.concatenate([[0.0], [r for _, r in rg], [0.0]])
    return rx, rr


def build_mesh(p):
    rg = rings(p)
    th = 2 * np.pi * np.arange(p.n_theta) / p.n_theta
    verts = [(-p.length / 2, 0.0, 0.0)]
    for x, r in rg:
        verts += [(x, r * np.cos(t), r * np.sin(t)) for t in th]
    verts.append((p.length / 2, 0.0, 0.0))
    V = np.array(verts)
    tip_t, tip_n, nt = 0, len(V) - 1, p.n_theta
    idx = lambda k, j: 1 + k * nt + (j % nt)
    F = [(tip_t, idx(0, j + 1), idx(0, j)) for j in range(nt)]
    for k in range(len(rg) - 1):
        for j in range(nt):
            a, b, c, d = idx(k, j), idx(k, j + 1), idx(k + 1, j + 1), idx(k + 1, j)
            F += [(a, b, c), (a, c, d)]
    kl = len(rg) - 1
    F += [(idx(kl, j), idx(kl, j + 1), tip_n) for j in range(nt)]
    return V, np.array(F)


def check_watertight(F, n_vertices):
    """Her yonlu kenar tam bir kez ve tersi mevcut -> kapali, tutarli yonlu; Euler karakteristigi."""
    directed = Counter()
    for f in F:
        for e in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            directed[e] += 1
    ok = all(c == 1 for c in directed.values()) and all((b, a) in directed for (a, b) in directed)
    euler = n_vertices - len(directed) // 2 + len(F)
    return ok, euler


def face_data(V, F):
    p1, p2, p3 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cr = np.cross(p2 - p1, p3 - p1)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    n = cr / (2 * area[:, None])
    return p1, p2, p3, area, n


def mesh_props(V, F):
    """Mesh-tam ozellikler. a_front: x'e dik izdusum; s_t: surge'de tegetsel alan (sum A(1-n_x^2));
    a_plan / s_t_z: z ekseni icin karsiliklari (heave)."""
    p1, p2, p3, area, n = face_data(V, F)
    vol6 = np.einsum('ij,ij->i', p1, np.cross(p2, p3))
    vol = vol6.sum() / 6
    cb = ((p1 + p2 + p3) / 4 * vol6[:, None]).sum(0) / vol6.sum()
    return dict(volume=vol, surface=area.sum(), cb=cb.tolist(),
                a_front=(n[:, 0].clip(min=0) * area).sum(), s_t=(area * (1 - n[:, 0] ** 2)).sum(),
                a_plan=(n[:, 2].clip(min=0) * area).sum(), s_t_z=(area * (1 - n[:, 2] ** 2)).sum())


def analytic_props(p):
    def half_spheroid_area(a, b):
        if np.isclose(a, b):
            return 2 * np.pi * b * b
        if a > b:
            e = np.sqrt(1 - b * b / (a * a))
            return np.pi * b * b * (1 + a / (b * e) * np.arcsin(e))
        e = np.sqrt(1 - a * a / (b * b))
        return np.pi * b * b * (1 + (1 - e * e) / e * np.arctanh(e))
    r = p.r
    v_cyl, v_n, v_t = np.pi * r**2 * p.mid_length, 2 / 3 * np.pi * r**2 * p.nose_length, 2 / 3 * np.pi * r**2 * p.tail_length
    vol = v_cyl + v_n + v_t
    x_cb = (v_cyl * (p.x_tail_joint + p.mid_length / 2) + v_n * (p.x_nose_joint + 3 * p.nose_length / 8)
            + v_t * (p.x_tail_joint - 3 * p.tail_length / 8)) / vol
    surf = 2 * np.pi * r * p.mid_length + half_spheroid_area(p.nose_length, r) + half_spheroid_area(p.tail_length, r)
    a_plan = 2 * r * p.mid_length + np.pi * r * (p.nose_length + p.tail_length) / 2
    return dict(volume=vol, surface=surf, a_front=np.pi * r**2, cb_x=x_cb, a_plan=a_plan)


def write_obj(path, V, F, header=''):
    _, _, _, _, n = face_data(V, F)
    with open(path, 'w') as f:
        if header:
            f.write(f'# {header}\n')
        f.writelines(f'v {v[0]:.9f} {v[1]:.9f} {v[2]:.9f}\n' for v in V)
        f.writelines(f'vn {m[0]:.9f} {m[1]:.9f} {m[2]:.9f}\n' for m in n)
        f.writelines(f'f {a+1}//{i+1} {b+1}//{i+1} {c+1}//{i+1}\n' for i, (a, b, c) in enumerate(F))

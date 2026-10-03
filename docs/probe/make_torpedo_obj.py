#!/usr/bin/env python3
"""Tek parca, kapali (watertight) torpido OBJ'si uretir ve geometriyi raporlar.

Govde ekseni +x (burun +x ucunda), orijin toplam boyun ortasinda.
Profil: kic yari-elipsoidi (a_t) + silindir (L_mid) + burun yari-elipsoidi (a_n).
Kullanim: python3 make_torpedo_obj.py [cikti.obj]
"""
import json
import sys
from collections import Counter
import numpy as np

R, A_NOSE, L_MID, A_TAIL = 0.1, 0.2, 0.5, 0.3
N_THETA, N_CAP, MID_STEP = 64, 24, 0.05
L = A_NOSE + L_MID + A_TAIL
X_TAIL_J = -L / 2 + A_TAIL          # kic-silindir birlesimi
X_NOSE_J = X_TAIL_J + L_MID         # silindir-burun birlesimi


def rings():
    """Kic ucundan buruna dogru halka (x, r) listesi; uc noktalar haric."""
    out = []
    for i in range(1, N_CAP + 1):                       # kic: psi 0 -> pi/2
        psi = i * (np.pi / 2) / N_CAP
        out.append((X_TAIL_J - A_TAIL * np.cos(psi), R * np.sin(psi)))
    n_mid = int(round(L_MID / MID_STEP))
    for i in range(1, n_mid + 1):                       # silindir
        out.append((X_TAIL_J + i * L_MID / n_mid, R))
    for i in range(1, N_CAP):                           # burun: psi pi/2 -> 0
        psi = (N_CAP - i) * (np.pi / 2) / N_CAP
        out.append((X_NOSE_J + A_NOSE * np.cos(psi), R * np.sin(psi)))
    return out


def build():
    rg = rings()
    th = 2 * np.pi * np.arange(N_THETA) / N_THETA
    verts = [(-L / 2, 0.0, 0.0)]                        # 0: kic ucu
    for x, r in rg:
        verts += [(x, r * np.cos(t), r * np.sin(t)) for t in th]
    verts.append((L / 2, 0.0, 0.0))                     # son: burun ucu
    V = np.array(verts)
    tip_t, tip_n = 0, len(V) - 1
    idx = lambda k, j: 1 + k * N_THETA + (j % N_THETA)
    F = []
    for j in range(N_THETA):                            # kic yelpazesi (normal -x)
        F.append((tip_t, idx(0, j + 1), idx(0, j)))
    for k in range(len(rg) - 1):                        # halkalar arasi dortgenler
        for j in range(N_THETA):
            a, b, c, d = idx(k, j), idx(k, j + 1), idx(k + 1, j + 1), idx(k + 1, j)
            F += [(a, b, c), (a, c, d)]
    kl = len(rg) - 1
    for j in range(N_THETA):                            # burun yelpazesi (normal +x)
        F.append((idx(kl, j), idx(kl, j + 1), tip_n))
    return V, np.array(F), rg


def check_watertight(F, nv):
    directed = Counter()
    for f in F:
        for e in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            directed[e] += 1
    ok_once = all(c == 1 for c in directed.values())
    ok_pair = all((b, a) in directed for (a, b) in directed)
    n_edges = len(directed) // 2
    euler = nv - n_edges + len(F)
    return ok_once and ok_pair, euler


def mesh_props(V, F):
    p1, p2, p3 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cr = np.cross(p2 - p1, p3 - p1)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    n = cr / (2 * area[:, None])
    vol6 = np.einsum('ij,ij->i', p1, np.cross(p2, p3))
    vol = vol6.sum() / 6
    cb = ((p1 + p2 + p3) / 4 * vol6[:, None]).sum(0) / vol6.sum()
    a_front = (n[:, 0].clip(min=0) * area).sum()
    s_t = (area * (1 - n[:, 0] ** 2)).sum()
    return dict(volume=vol, surface=area.sum(), a_front=a_front, s_t=s_t, cb=cb.tolist()), n, area


def analytic():
    def half_spheroid_area(a, b):
        e = np.sqrt(1 - b**2 / a**2)
        return np.pi * b**2 * (1 + a / (b * e) * np.arcsin(e))
    vol = np.pi * R**2 * (L_MID + 2 / 3 * A_NOSE + 2 / 3 * A_TAIL)
    surf = 2 * np.pi * R * L_MID + half_spheroid_area(A_NOSE, R) + half_spheroid_area(A_TAIL, R)
    m_cyl = np.pi * R**2 * L_MID * (X_TAIL_J + L_MID / 2)
    m_n = 2 / 3 * np.pi * R**2 * A_NOSE * (X_NOSE_J + 3 * A_NOSE / 8)
    m_t = 2 / 3 * np.pi * R**2 * A_TAIL * (X_TAIL_J - 3 * A_TAIL / 8)
    return dict(volume=vol, surface=surf, a_front=np.pi * R**2, cb_x=(m_cyl + m_n + m_t) / vol)


def write_obj(path, V, F, n):
    with open(path, 'w') as f:
        f.write(f"# torpido L={L} D={2*R} a_n={A_NOSE} L_mid={L_MID} a_t={A_TAIL}\n")
        for v in V:
            f.write(f"v {v[0]:.9f} {v[1]:.9f} {v[2]:.9f}\n")
        for nn in n:
            f.write(f"vn {nn[0]:.9f} {nn[1]:.9f} {nn[2]:.9f}\n")
        for i, fc in enumerate(F):
            f.write(f"f {fc[0]+1}//{i+1} {fc[1]+1}//{i+1} {fc[2]+1}//{i+1}\n")


def main(out):
    V, F, rg = build()
    tight, euler = check_watertight(F, len(V))
    props, n, _ = mesh_props(V, F)
    ana = analytic()
    assert tight and euler == 2 and props['volume'] > 0, "mesh kapali/yonlu degil"
    write_obj(out, V, F, n)
    info = dict(params=dict(R=R, a_nose=A_NOSE, L_mid=L_MID, a_tail=A_TAIL, L=L,
                            n_theta=N_THETA, n_cap=N_CAP),
                rings_x=[x for x, _ in rg], rings_r=[r for _, r in rg],
                n_vertices=len(V), n_faces=len(F), watertight=tight, euler=euler,
                mesh=props, analytic=ana)
    with open(out.replace('.obj', '.json'), 'w') as f:
        json.dump(info, f, indent=1)
    print(f"Kose {len(V)}, yuz {len(F)}, watertight={tight}, Euler={euler}")
    for k in ('volume', 'surface', 'a_front'):
        print(f"{k:8s} mesh={props[k]:.6f}  analitik={ana[k]:.6f}  fark={100*(props[k]/ana[k]-1):+.3f}%")
    print(f"S_t      mesh={props['s_t']:.6f}")
    print(f"CB_x     mesh={props['cb'][0]:.6f}  analitik={ana['cb_x']:.6f}")


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'meshes/torpedo_L1_D02.obj')

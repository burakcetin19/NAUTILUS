"""Stonefish'in mesh govdeler icin hidrodinamik elipsoid yaklasimi (MVAE), kodtaki haliyle port.

Zincir (Stonefish b21eb8e, SF = ~/stonefish/Library/src):
  1. OBJ yukleme, SF/utils/GeometryFileUtil.cpp:52-250 (normalli, UV'siz dal): konumlar float32;
     bir konum ilk kullanildigi normalden farkli bir normalle tekrar gelirse yeni kose uretilir
     (uretilen havuz (konum, normal) ile tekillestirilir, sonra uretim sirasiyla eklenir).
  2. Refine(mesh, 3), SF/graphics/OpenGLContent.cpp:2833: alan > 3*max(ort. alan, 1e-4) olan yuzler
     bolunur. Port edilmedi; boyle bir yuz varsa hata verilir.
  3. ComputePhysicalProperties, GeometryFileUtil.cpp:335-560: CG (isaretli tetrahedron) ve atalet;
     IsDiagonal (:605) dogruysa asal eksen donusu birim matris. Degilse hata (port edilmedi).
  4. ComputeEllipsoidalApprox, SF/entities/SolidEntity.cpp:849-1020: tum koseler (kopyalar dahil)
     CG cercevesinde; P. Kumar & E.A. Yildirim (2008) eksen hizali MVAE, errorTol 0.2, en fazla 10
     iterasyon; baslangic sigma'si, x0'daki 6 uc noktaya esit olan her koseye 1/6 (std::find).
     k = 0 ise sinirlayici kutu kullanilir.
Cikti: yari-eksenler d = (a, b, c) [m]; varsayilan C_d = (1/d)/max(1/d), C_f = 0.1 C_d.
"""
import numpy as np


def load_obj_vertices(path):
    """Stonefish OBJ yukleyicisinin kose listesi ve yuzleri (kopya koseler dahil)."""
    pos, nrm, faces = [], [], []
    with open(path) as fh:
        for line in fh:
            t = line.split()
            if not t:
                continue
            if t[0] == 'v':
                pos.append([float(x) for x in t[1:4]])
            elif t[0] == 'vn':
                nrm.append([float(x) for x in t[1:4]])
            elif t[0] == 'f':
                if len(t) != 4:
                    raise ValueError('sadece ucgen yuzler destekleniyor')
                faces.append([tuple(int(s) - 1 if s else -1 for s in c.split('/')) for c in t[1:]])
    P = np.array(pos, dtype=np.float32)
    N = np.array(nrm, dtype=np.float32)
    if len(N) == 0:
        raise ValueError('normalsiz OBJ dali port edilmedi')
    n0 = len(P)
    cur = [None] * n0                    # konumun ilk atanan normali
    gen, gen_list = {}, []               # (konum indeksi, normal) -> uretilen indeks
    F = []
    for f in faces:
        ids = []
        for pid, _, nid in f:
            nv = tuple(N[nid].tolist())
            if cur[pid] is None:
                cur[pid] = nv
                ids.append(pid)
            elif cur[pid] == nv:
                ids.append(pid)
            else:
                key = (pid, nv)
                if key not in gen:
                    gen[key] = len(gen_list)
                    gen_list.append(pid)
                ids.append(n0 + gen[key])
        F.append(ids)
    V = np.concatenate([P, P[gen_list]]).astype(np.float64)
    return V, np.array(F)


def check_no_refine(V, F):
    p1, p2, p3 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(p2 - p1, p3 - p1), axis=1)
    thr = 3.0 * max(area.mean(), 1e-4)
    if (area > thr).any():
        raise NotImplementedError(f'Stonefish Refine bu mesh\'i bolerdi (max alan {area.max():.3g} > {thr:.3g})')
    return area.max() / thr


def centroid_and_diagonal(V, F):
    """CG ve IsDiagonal (yogunluk orani etkilemez). Donus: CG, diagonal_mi, atalet (birim yogunluk)."""
    v1, v2, v3 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    v6 = np.einsum('ij,ij->i', v1, np.cross(v2, v3))
    cg = ((v1 + v2 + v3) / 4 * v6[:, None]).sum(0) / v6.sum()
    a, b, c = v1 - cg, v2 - cg, v3 - cg
    v6 = np.einsum('ij,ij->i', a, np.cross(b, c))

    def P(j, k):
        return (v6 * (2 * (a[:, j] * a[:, k] + b[:, j] * b[:, k] + c[:, j] * c[:, k])
                      + a[:, j] * b[:, k] + a[:, k] * b[:, j] + a[:, j] * c[:, k] + a[:, k] * c[:, j]
                      + b[:, j] * c[:, k] + b[:, k] * c[:, j])).sum() / 120
    Pxx, Pyy, Pzz, Pxy, Pxz, Pyz = P(0, 0), P(1, 1), P(2, 2), P(0, 1), P(0, 2), P(1, 2)
    I = np.array([[Pyy + Pzz, -Pxy, -Pxz], [-Pxy, Pxx + Pzz, -Pyz], [-Pxz, -Pyz, Pxx + Pyy]])
    m = np.abs(np.diag(I)).min() / 1e6
    off = np.abs(I[~np.eye(3, dtype=bool)])
    return cg, bool((off < m).all()), I


def mvae(x):
    """SolidEntity::ComputeEllipsoidalApprox iterasyonu. x: (n, 3) CG cercevesinde noktalar.
    Donus: merkez c, yari-eksenler d, iterasyon sayisi k, son epsilon."""
    n = len(x)
    x0 = []
    for k in range(3):
        col = x[:, k]
        i_min = int(np.argmin(col))                          # ilk minimum (std::minmax_element)
        i_max = n - 1 - int(np.argmax(col[::-1]))            # son maksimum
        x0 += [x[i_min], x[i_max]]
    x0 = np.array(x0)
    sigma = np.where((x[:, None, :] == x0[None, :, :]).all(-1).any(1), 1.0 / 6, 0.0)

    def uv(s):
        return (s[:, None] * x * x).sum(0), (s[:, None] * x).sum(0)

    def lam(s):
        u, v = uv(s)
        return ((x - v) ** 2 / (3 * (u - v * v))).sum(1)

    L = lam(sigma)
    i_star = int(np.argmax(L))                               # ilk maksimum (std::max_element)
    eps = L[i_star] - 1
    eps_tol = 1.2 ** (2.0 / 3.0) - 1
    k = 0
    while eps > eps_tol and k < 10:
        beta = eps / (4 * (1 + eps))
        sigma = (1 - beta) * sigma
        sigma[i_star] += beta
        L = lam(sigma)
        i_star = int(np.argmax(L))
        eps = L[i_star] - 1
        k += 1
    if k == 0:
        c = np.array([(x0[0, 0] + x0[1, 0]) / 2, (x0[2, 1] + x0[3, 1]) / 2, (x0[4, 2] + x0[5, 2]) / 2])
        d = np.abs(np.array([x0[0, 0], x0[2, 1], x0[4, 2]]) - c)
    else:
        u, v = uv(sigma)
        c = v
        d = np.sqrt(3 * (u - v * v))
    return c, d, k, eps


def stonefish_ellipsoid(obj_path):
    """OBJ -> Stonefish'in hesapladigi yari-eksenler ve varsayilan katsayilar."""
    V, F = load_obj_vertices(obj_path)
    refine_ratio = check_no_refine(V, F)
    cg, diag, _ = centroid_and_diagonal(V, F)
    if not diag:
        raise NotImplementedError('atalet tensoru kosegen degil: asal eksen donusu port edilmedi')
    c, d, k, eps = mvae(V - cg)
    cd = (1 / d) / (1 / d).max()
    return dict(semi_axes=d.tolist(), center=c.tolist(), iterations=k, epsilon=float(eps),
                n_vertices=len(V), cd_default=cd.tolist(), cf_default=(0.1 * cd).tolist(),
                max_face_area_over_refine_threshold=float(refine_ratio))

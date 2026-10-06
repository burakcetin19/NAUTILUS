#!/usr/bin/env python3
"""Faz 2 statik test analizi (S1-S5) ve kabul kriterleri.

Girdi: bags/faz2/<kosu> (scripts/static_tests.sh), config/vehicle.yaml, config/vehicle_derived.yaml
Cikti: docs/02_statik_tablolar.md, docs/02_statik_sonuclar.json, figures/faz2/*.png|pdf
Kullanim: source /opt/ros/humble/setup.bash && source ~/stonefish_ws/install/setup.bash
          python3 scripts/analyze_static.py
Zaman: DebugPhysics indeks x 0.01 s, odometri indeks x 0.02 s (Faz 0.5). Konular arasi kaba hizalama
icin bag alim zamani (t_rec) kullanilir.
"""
import json
import os
import sys
import numpy as np
import yaml
from scipy import optimize
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from hybrid_vehicle_sim import simlog, hull, hydrostatics as hs, drag, mvae  # noqa: E402
from design_vehicle import style, C1, C2, C3, INK, INK2, GRID, SURF   # noqa: E402

BAGS = os.path.join(ROOT, 'bags', 'faz2')
FIG = os.path.join(ROOT, 'figures', 'faz2')
NS = '/nautilus'
DT, ODT, RATE = 0.01, 0.02, 100.0
Z_SUBMERGED = 1.0        # [m] eksen derinligi > 1 m: tamamen dalmis (yaricap 7.5 cm)


# ---------------------------------------------------------------- tahminler
def predictions(cfg, d):
    rho, g = d['fluid']['rho'], d['fluid']['g']
    ms, mh, sf, hd = d['mass'], d['mesh'], d['stonefish'], d['hydrodynamics']
    m, V = ms['mass'], mh['volume']
    B, W = rho * g * V, m * g
    bg = ms['cg'][2] - mh['cb'][2]
    I = np.array(ms['inertia'])
    aI = np.array(sf['added_inertia'])

    def w_inf(cd_z, cf_z):
        a, b = 0.5 * rho * cd_z * mh['a_plan'], rho * cf_z * mh['s_t_z']
        return optimize.brentq(lambda w: a * w**3 + b * w - (B - W), 1e-6, 10.0), a, b
    w_cal, a_z, b_z = w_inf(hd['quadratic_drag'][2], hd['viscous_drag'][2])
    w_def, a_zd, b_zd = w_inf(sf['cd_default'][2], sf['cf_default'][2])

    # yuzey: trim dahil denge (Stonefish yuzeyi kesen govdede V_sub'u kirpilmis yuzlerden hesapliyor;
    # dilimleme ayni geometriyi bagimsiz olcer, 01_tasarim T8)
    p = hull.HullParams.from_dict(cfg['hull'])
    rx, rr = hull.ring_profile(p)
    sl = hs.Slicer(rx, rr, p.n_theta)
    trim = np.radians(d['surface']['trim_deg'])
    z_eq = sl.equilibrium_depth(m / rho, hs.rot_y(trim))
    xw = np.linspace(rx[0], rx[-1], 6001)
    wp = hs.waterplane(xw, np.interp(xw, rx, rr), d['surface']['axis_depth'])
    cg = np.array(ms['cg'])
    gz = lambda a: hs.righting_arm(sl, m, rho, g, cg, a, 'pitch', True)
    da = np.radians(0.05)
    gm_l = (gz(trim + da) - gz(trim - da)) / (2 * np.sin(da))
    gm_l2 = (gz(trim + np.radians(2)) - gz(trim)) / np.sin(np.radians(2))       # 2 deg sekant
    gm_t = (hs.righting_arm(sl, m, rho, g, cg, da, 'roll', True)
            - hs.righting_arm(sl, m, rho, g, cg, -da, 'roll', True)) / (2 * np.sin(da))
    T = lambda inertia, k: 2 * np.pi * np.sqrt(inertia / k)
    return dict(
        rho=rho, g=g, m=m, V=V, S=mh['surface'], B=B, W=W, bg=bg, I=I, aI=aI, cg=cg, cb=np.array(mh['cb']),
        cd=np.array(hd['quadratic_drag']), cf=np.array(hd['viscous_drag']),
        cd_def=np.array(sf['cd_default']), cf_def=np.array(sf['cf_default']),
        semi=np.array(sf['mvae']['semi_axes']), ma=np.array(sf['added_mass']),
        m_eff=sf['m_eff'], m_eff_phys_heave=m + drag.lamb_prolate(p.length, p.diameter)[1] * rho * V,
        w_cal=w_cal, a_z=a_z, b_z=b_z, w_def=w_def, a_zd=a_zd, b_zd=b_zd,
        z_eq=z_eq, z_eq_upright=d['surface']['axis_depth'], trim=trim, a_wp=wp['area'],
        T_heave_surf=T(sf['m_eff'], rho * g * wp['area']), gm_l=gm_l, gm_l2=gm_l2, gm_t=gm_t,
        T_roll_surf=T(I[0] + aI[0], W * gm_t),
        T_pitch_surf=T(I[1] + aI[1], W * gm_l), T_pitch_surf2=T(I[1] + aI[1], W * gm_l2),
        # notr varyant: W = B, dogrultucu moment B*BG*sin(theta), CG etrafinda donus
        T_roll=T(I[0] + aI[0], B * bg), T_pitch=T(I[1] + aI[1], B * bg),
        T_roll_noadd=T(I[0], B * bg), T_pitch_noadd=T(I[1], B * bg))


# ---------------------------------------------------------------- veri
def load(name, rate=RATE):
    path = os.path.join(BAGS, name)
    if not os.path.isfile(os.path.join(path, 'metadata.yaml')):     # yok ya da kayit suruyor
        return None
    b = simlog.read_bag(path)
    D = simlog.debug_arrays(b[f'{NS}/debug/physics'], 1.0 / rate)
    O = simlog.odom_arrays(b[f'{NS}/odometry'], ODT)
    st = b[f'{NS}/thruster_state']
    thr = np.array([m.thrust for _, m in st])
    tq = np.array([m.torque for _, m in st])
    # debug zamaninda odometri pozu (t_rec ile kaba hizalama, +-birkac ms)
    for k, j in (('z', 2),):
        D[k] = np.interp(D['t_rec'], O['t_rec'], O['pos'][:, j])
    D['roll'] = np.interp(D['t_rec'], O['t_rec'], O['rpy'][:, 0])
    D['pitch'] = np.interp(D['t_rec'], O['t_rec'], O['rpy'][:, 1])
    checks = dict(rate=rate, debug=simlog.count_check(D['t_rec'], rate), odom=simlog.count_check(O['t_rec'], 1 / ODT),
                  thrust_max=float(np.abs(thr).max()), torque_max=float(np.abs(tq).max()),
                  first_ang=float(np.abs(D['ang'][0]).max()), first_lin_xy=float(np.abs(D['lin'][0, :2]).max()))
    return dict(D=D, O=O, P=b.get(f'{NS}/pressure', []), checks=checks)


def damped_fit(t, y, t_end=None):
    """y = A e^{-s t} cos(w t + phi) + c. Donus: w_n = sqrt(w^2 + s^2), w_d, s, zeta, egri, sifir-gecis T."""
    sel = t <= (t_end if t_end is not None else t[-1])
    t, y = t[sel] - t[sel][0], y[sel]

    def crossings(c):
        """Yukari yonlu sifir gecisleri (dogrusal ara degerleme); ardisik farklarin ortalamasi = periyot."""
        e = y - c
        i = np.where((e[:-1] < 0) & (e[1:] >= 0))[0]
        tc = t[i] - e[i] * (t[i + 1] - t[i]) / (e[i + 1] - e[i])
        return tc, (np.mean(np.diff(tc)) if len(tc) > 1 else np.nan)
    _, T0 = crossings(np.mean(y))
    T0 = T0 if np.isfinite(T0) else (t[-1] - t[0]) / 2
    f = lambda tt, A, s, w, ph, c: A * np.exp(-s * tt) * np.cos(w * tt + ph) + c
    p0 = [y[0] - np.mean(y), 0.05 * 2 * np.pi / T0, 2 * np.pi / T0, 0.0, np.mean(y)]
    p, _ = optimize.curve_fit(f, t, y, p0=p0, maxfev=20000)
    A, s, w, ph, c = p
    w = abs(w)
    wn = np.sqrt(w * w + s * s)
    resid = y - f(t, *p)
    tc, T_zc = crossings(c)
    return dict(wn=wn, wd=w, sigma=s, zeta=s / wn, T_n=2 * np.pi / wn, T_d=2 * np.pi / w, T_zc=T_zc,
                amp0=abs(A), c=c, rms_rel=float(np.sqrt(np.mean(resid**2)) / abs(A)),
                t=t, y=y, fit=f(t, *p), n_zc=len(tc))


def torque_gain(r, ang, ax, k0):
    """Kaldirma momenti = -k sin(theta(t - delta)); delta (konular arasi hizalama + 50 Hz tutma) taranir.
    Donus: k / k0 ve en iyi delta [s]."""
    D, O = r['D'], r['O']
    best = None
    for dl in np.arange(-0.06, 0.0601, 0.001):
        th = np.interp(D['t_rec'] - dl, O['t_rec'], O['rpy'][:, ang])
        sel = np.abs(th) > np.radians(0.2)
        x, yv = -np.sin(th[sel]), D['tb'][sel, ax]
        k = np.sum(x * yv) / np.sum(x * x)
        res = np.sum((yv - k * x) ** 2)
        if best is None or res < best[2]:
            best = (k / k0, dl, res)
    return best[0], best[1]


# ---------------------------------------------------------------- testler
def rel(a, b):
    return (a - b) / b


def port_check(D, idx, pr, cd, cf):
    """DebugPhysics F_p/F_f vektorleri ile Stonefish yuz bazli formul portu (drag.stonefish_face_forces).
    Kuvvetler 50 Hz'de yeniden hesaplaniyor: yalnizca kuvvetin degistigi mesajlar; hiz kaymasi veriden secilir."""
    V, F = mvae.load_obj_vertices(os.path.join(ROOT, 'meshes', 'hull.obj'))
    p1, p2, p3 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    ch = np.where(np.abs(np.diff(D['fp'][:, 2])) > 0)[0] + 1
    ch = ch[(ch >= idx.start) & (ch < idx.stop)][::2][:150]
    best = None
    for sh in (0, 1, 2):
        e = []
        for j in ch:
            Pp, Pf, _, _ = drag.stonefish_face_forces(p1, p2, p3, pr['cg'], D['lin'][j - sh], D['ang'][j - sh], cd, cf, pr['rho'])
            e.append([np.linalg.norm(Pp - D['fp'][j]) / np.linalg.norm(D['fp'][j]),
                      np.linalg.norm(Pf - D['ff'][j]) / np.linalg.norm(D['ff'][j])])
        e = np.array(e)
        if best is None or np.median(e) < np.median(best[1]):
            best = (sh, e)
    return dict(shift=best[0], n=len(ch), fp_max=float(best[1][:, 0].max()), ff_max=float(best[1][:, 1].max()),
                fp_med=float(np.median(best[1][:, 0])), ff_med=float(np.median(best[1][:, 1])))


def sigma_model(pr, ax, rate):
    """Dalmis salinim sonumu: sigma = c/(2 I_eff) - w_n^2 tau / 2.
    c: Stonefish yuz bazli surtunme torku portundan (omega = 0.1 rad/s, dogrusal); tau: kaldirma momentinin
    50 Hz tutulmasindan ortalama gecikme (P - 1) dt / 2, P = round(rate / 50) (SimulationManager.cpp:582, 1584).
    Gecikmeli yay -k theta(t - tau) ~ -k theta + k tau theta' -> negatif sonum."""
    V, F = mvae.load_obj_vertices(os.path.join(ROOT, 'meshes', 'hull.obj'))
    om = np.zeros(3)
    om[ax] = 0.1
    _, _, Tp, Tf = drag.stonefish_face_forces(V[F[:, 0]], V[F[:, 1]], V[F[:, 2]], pr['cg'], np.zeros(3), om,
                                               pr['cd'], pr['cf'], pr['rho'])
    c = -(Tp[ax] + Tf[ax]) / 0.1
    I_eff = pr['I'][ax] + pr['aI'][ax]
    wn2 = pr['B'] * pr['bg'] / I_eff
    P = max(int(round(rate / 50.0)), 1)
    tau = (P - 1) / (2.0 * rate)
    return dict(c=c, sigma_phys=c / (2 * I_eff), sigma_neg=wn2 * tau / 2, tau=tau, P=P,
                sigma=c / (2 * I_eff) - wn2 * tau / 2)


def aoa_table(pr, speeds=(0.5, 1.0, 1.5), angles=(0.0, 0.25, 0.5, 1.0, 2.0, 5.0)):
    """Faz 3 ongorusu: kalibre katsayilarla surge direnci (govde x) vs hucum acisi, yuz bazli port ile.
    Akis govdeye gore alpha egik (v = u (cos a, 0, sin a)); L1 karisimi C_eff = |d_x| C_x + |d_z| C_z."""
    V, F = mvae.load_obj_vertices(os.path.join(ROOT, 'meshes', 'hull.obj'))
    p = [V[F[:, i]] for i in range(3)]
    rows = []
    for u in speeds:
        base = None
        for a in angles:
            al = np.radians(a)
            Fp, Ff, _, _ = drag.stonefish_face_forces(*p, pr['cg'], np.array([u * np.cos(al), 0, u * np.sin(al)]),
                                                       np.zeros(3), pr['cd'], pr['cf'], pr['rho'])
            X = -(Fp[0] + Ff[0])
            base = X if base is None else base
            rows.append(dict(u=u, alpha=a, X=X, Fp_x=-Fp[0], Ff_x=-Ff[0], Z=-(Fp[2] + Ff[2]), rel=X / base - 1))
    return rows


def ascent(R, pr, cd_key):
    """Serbest yukselme: M_eff (F_net = M dw/dt), terminal hiz, heave kuvvet formulu."""
    D = R['D']
    w = D['lin'][:, 2]
    fnet = D['fb'][:, 2] + pr['m'] * pr['g'] + D['fp'][:, 2] + D['ff'][:, 2]   # govde z ~ dunya z (pitch kucuk)
    n = int(np.argmax(D['z'] < Z_SUBMERGED)) if (D['z'] < Z_SUBMERGED).any() else len(w)
    M, shift, res = simlog.estimate_mass(w[:n], fnet[:n], DT)
    dts = simlog.dynamic_dt(w[:n], fnet[:n], M, 1.0)
    a_z, b_z = (pr['a_z'], pr['b_z']) if cd_key == 'cal' else (pr['a_zd'], pr['b_zd'])
    w_pred = pr['w_cal'] if cd_key == 'cal' else pr['w_def']
    ss = slice(int(3.0 / DT), n)                                                   # gecis ~5 tau sonra
    w_ss = -w[ss]
    fp_rat = D['fp'][ss, 2] / (a_z * w_ss**3)
    ff_rat = D['ff'][ss, 2] / (b_z * w_ss)
    after = slice(n, None)
    pc = port_check(D, ss, pr, *((pr['cd'], pr['cf']) if cd_key == 'cal' else (pr['cd_def'], pr['cf_def'])))
    return dict(M=M, shift=shift, res=res, n_fit=int((np.abs(fnet[:n]) > 1.0).sum()), port=pc,
                dt_max_over_nom=float(dts.max() / DT), dt_min_over_nom=float(dts.min() / DT),
                w_inf=float(w_ss.mean()), w_inf_std=float(w_ss.std()), w_pred=w_pred,
                fp_ratio=float(fp_rat.mean()), fp_ratio_dev=float(np.abs(fp_rat - 1).max()),
                ff_ratio=float(ff_rat.mean()), ff_ratio_dev=float(np.abs(ff_rat - 1).max()),
                pitch_max_deg=float(np.degrees(np.abs(D['pitch'][:n]).max())),
                u_max=float(np.abs(D['lin'][:n, 0]).max()),
                t_cut=float(D['t'][n - 1]), z_min_after=float(D['z'][after].min()) if n < len(w) else np.nan,
                pitch_range_after_deg=(float(np.degrees(D['pitch'][after].min())),
                                       float(np.degrees(D['pitch'][after].max()))) if n < len(w) else None,
                w=w, fnet=fnet, n=n)


def analyze(cfg, d):
    pr = predictions(cfg, d)
    R = {k: load(k) for k in ('depth_release', 'depth_default', 'surface', 'surface_pitch',
                              'roll_neutral', 'pitch_neutral')}
    R['roll_neutral_500'] = load('roll_neutral_500', 500.0)
    R['roll_neutral_50'] = load('roll_neutral_50', 50.0)
    R['surface_roll'] = load('surface_roll')
    rows, out = [], dict(predictions={k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in pr.items()})

    def row(test, name, meas, pred, err, tol, unit='', fmt='{:.6g}', ok=None):
        passed = (abs(err) <= tol) if ok is None else ok
        rows.append(dict(test=test, name=name, meas=meas, pred=pred, err=err, tol=tol, unit=unit, fmt=fmt,
                         result='PASS' if passed else 'FAIL'))

    # zaman tabani ve itki temizligi (her kosu)
    tb = []
    for k, r in R.items():
        if r is None:
            continue
        c = r['checks']
        tb.append(dict(run=k, **{f'{s}_{q}': c[s][q] for s in ('debug', 'odom') for q in ('n', 'ratio')},
                       thrust_max=c['thrust_max'], torque_max=c['torque_max'],
                       first_ang=c['first_ang'], first_lin_xy=c['first_lin_xy']))
    out['runs'] = tb

    # S1: yukleme ve tutarlilik (depth_release ilk mesaj)
    r1 = R['depth_release']
    if r1:
        D = r1['D']
        for name, meas, pred in (('kütle m [kg]', D['mass'], pr['m']), ('hacim V [m³]', D['volume'], pr['V']),
                                 ('yüzey S [m²]', D['surface'], pr['S']),
                                 ('kaldırma F_b,z [N] (dünya)', D['fb'][0, 2], -pr['B'])):
            row('S1', name, meas, pred, rel(meas, pred), 1e-6)
        for i, ax in enumerate('xyz'):
            row('S1', f'atalet I_{ax}{ax} [kg·m²]', D['inertia'][i], pr['I'][i], rel(D['inertia'][i], pr['I'][i]), 1e-5)
        row('S1', 'CG [m] (max |fark|)', D['cog'].tolist(), pr['cg'].tolist(), float(np.abs(D['cog'] - pr['cg']).max()),
            1e-6, fmt='{}')
        for i, ax in enumerate('xyz'):
            row('S1', f'C_d,{ax}', D['cd'][i], pr['cd'][i], rel(D['cd'][i], pr['cd'][i]), 1e-6)
            row('S1', f'C_f,{ax} [m/s]', D['cf'][i], pr['cf'][i], rel(D['cf'][i], pr['cf'][i]), 1e-6)
        P = r1['P'][:20]
        tp = np.array([tr for tr, _ in P])
        zp = np.interp(tp, r1['O']['t_rec'], r1['O']['pos'][:, 2])
        pp = np.array([m.fluid_pressure for _, m in P])
        e = rel(pp, pr['rho'] * pr['g'] * zp)
        row('S1', 'basınç / ρ g z (ilk 20 örnek, max |fark|)', float(pp[0]), float(pr['rho'] * pr['g'] * zp[0]),
            float(e[np.argmax(np.abs(e))]), 1e-3, unit='Pa')

    # S3: serbest yukselme (kalibre)
    if r1:
        a = ascent(r1, pr, 'cal')
        out['S3'] = {k: v for k, v in a.items() if k not in ('w', 'fnet', 'n')}
        row('S3', 'M_eff [kg] (F_net = M·ẇ)', a['M'], pr['m_eff'], rel(a['M'], pr['m_eff']), 0.01)
        row('S3', 'terminal yükselme hızı w∞ [m/s]', a['w_inf'], a['w_pred'], rel(a['w_inf'], a['w_pred']), 0.005)
        row('S3', 'F_p (vektör) / yüz bazlı port, max göreli fark', a['port']['fp_max'], 0.0, a['port']['fp_max'], 1e-3)
        row('S3', 'F_f (vektör) / yüz bazlı port, max göreli fark', a['port']['ff_max'], 0.0, a['port']['ff_max'], 1e-3)
        row('S3', 'F_p,z / (½ρC_d,z A_plan w³), ortalama (bilgi)', a['fp_ratio'], 1.0, a['fp_ratio'] - 1, np.inf, ok=True)
        row('S3', 'F_f,z / (ρ C_f,z S_t,z w), ortalama (bilgi)', a['ff_ratio'], 1.0, a['ff_ratio'] - 1, np.inf, ok=True)
        row('S3', 'fiziksel Δt / nominal (max)', a['dt_max_over_nom'], 1.0, a['dt_max_over_nom'] - 1, 0.5)
        R['_a1'] = a

    # S2: varsayilan hidrodinamik (MVAE)
    r2 = R['depth_default']
    if r2:
        D = r2['D']
        for i, ax in enumerate('xyz'):
            row('S2', f'varsayılan C_d,{ax} (MVAE portu)', D['cd'][i], pr['cd_def'][i], rel(D['cd'][i], pr['cd_def'][i]), 0.005)
            row('S2', f'varsayılan C_f,{ax} [m/s] (= 0.1 C_d)', D['cf'][i], pr['cf_def'][i], rel(D['cf'][i], pr['cf_def'][i]), 0.005)
        a2 = ascent(r2, pr, 'def')
        out['S2'] = {k: v for k, v in a2.items() if k not in ('w', 'fnet', 'n')}
        row('S2', 'M_eff [kg] (override\'dan bağımsız)', a2['M'], pr['m_eff'], rel(a2['M'], pr['m_eff']), 0.01)
        if r1:
            row('S2', 'M_eff (varsayılan) / M_eff (kalibre)', a2['M'] / R['_a1']['M'], 1.0, a2['M'] / R['_a1']['M'] - 1, 0.005)
        row('S2', 'terminal w∞ [m/s] (varsayılan C_d/C_f)', a2['w_inf'], a2['w_pred'], rel(a2['w_inf'], a2['w_pred']), 0.005)
        row('S2', 'F_p / F_f (vektör) / yüz bazlı port, max göreli fark', a2['port']['fp_max'], 0.0,
            max(a2['port']['fp_max'], a2['port']['ff_max']), 1e-3)
        R['_a2'] = a2

    # S4: yuzey dengesi ve yuzey periyotlari
    r3 = R['surface']
    if r3:
        O = r3['O']
        last = O['t'] >= O['t'][-1] - 30.0
        z, pitch, roll = O['pos'][last, 2].mean(), O['rpy'][last, 1].mean(), O['rpy'][last, 0].mean()
        lastd = r3['D']['t'] >= r3['D']['t'][-1] - 30.0
        vsub = r3['D']['vsub'][lastd].mean()
        res_osc = float(O['pos'][last, 2].std())
        row('S4', 'eksen (orijin) derinliği [cm], son 30 s', 100 * z, 100 * pr['z_eq'], 100 * (z - pr['z_eq']), 0.05, 'cm')
        row('S4', 'trim [°], son 30 s', np.degrees(pitch), np.degrees(pr['trim']), np.degrees(pitch - pr['trim']), 0.05, '°')
        row('S4', 'roll [°], son 30 s', np.degrees(roll), 0.0, np.degrees(roll), 0.05, '°')
        row('S4', 'V_sub [m³] = m/ρ, son 30 s', vsub, pr['m'] / pr['rho'], rel(vsub, pr['m'] / pr['rho']), 1e-3)
        hf = damped_fit(O['t'], O['pos'][:, 2], 30.0)
        out['S4'] = dict(z=z, pitch=pitch, roll=roll, vsub=vsub, z_std_last30=res_osc,
                         heave={k: v for k, v in hf.items() if not isinstance(v, np.ndarray)})
        row('S4', 'yüzey heave periyodu T_n [s] (bilgi)', hf['T_n'], pr['T_heave_surf'], rel(hf['T_n'], pr['T_heave_surf']),
            np.inf, 's', ok=True)
        R['_hf'] = hf
    r3b = R['surface_pitch']
    if r3b:
        O = r3b['O']
        pf = damped_fit(O['t'], O['rpy'][:, 1], 20.0)
        out['S4']['pitch'] = {k: v for k, v in pf.items() if not isinstance(v, np.ndarray)} if 'S4' in out else None
        row('S4', 'yüzey pitch periyodu T_n [s] (bilgi; GM_L ±0.05°)', pf['T_n'], pr['T_pitch_surf'],
            rel(pf['T_n'], pr['T_pitch_surf']), np.inf, 's', ok=True)
        row('S4', 'yüzey pitch periyodu T_n [s] (bilgi; GM_L 2° sekant)', pf['T_n'], pr['T_pitch_surf2'],
            rel(pf['T_n'], pr['T_pitch_surf2']), np.inf, 's', ok=True)
        R['_pf'] = pf
    r3c = R['surface_roll']
    if r3c:
        O = r3c['O']
        rf = damped_fit(O['t'], O['rpy'][:, 0], 4.0)
        last5 = O['t'] >= O['t'][-1] - 5.0
        # model: k = W GM_T(yuzey); c = DebugPhysics roll sonum torku / p (yuzeyde kirpilmis yuzler, port yok).
        # Yuzeyde torkun y bileseni var -> L1 karisimi C_eff ~ 8 C_f,x (dalmista ~1.1 C_f,x)
        Dd = r3c['D']
        sel = np.abs(Dd['ang'][:, 0]) > 0.2
        c_s = float(-np.sum((Dd['tf'][sel, 0] + Dd['tp'][sel, 0]) * Dd['ang'][sel, 0]) / np.sum(Dd['ang'][sel, 0] ** 2))
        dv = Dd['tf'][sel] / np.linalg.norm(Dd['tf'][sel], axis=1)[:, None]
        ceff_ratio = float(np.median(np.abs(dv) @ pr['cf'] / pr['cf'][0]))
        sm = sigma_model(pr, 0, RATE)
        I_eff = pr['I'][0] + pr['aI'][0]
        sig = c_s / (2 * I_eff) - (pr['W'] * pr['gm_t'] / I_eff) * sm['tau'] / 2
        out.setdefault('S4', {})['roll'] = dict(fit={k: v for k, v in rf.items() if not isinstance(v, np.ndarray)},
                                                sigma_model=sig, c_surface=c_s, c_submerged=sm['c'], ceff_ratio=ceff_ratio,
                                                amp_first_deg=float(np.degrees(np.abs(O['rpy'][O['t'] <= 1.0, 0]).max())),
                                                amp_last5_deg=float(np.degrees(np.abs(O['rpy'][last5, 0]).max())))
        row('S4', 'yüzey roll periyodu T_n [s] (bilgi; I_xx, W·GM_T)', rf['T_n'], pr['T_roll_surf'],
            rel(rf['T_n'], pr['T_roll_surf']), np.inf, 's', ok=True)
        row('S4', 'yüzey roll sönümü σ [1/s] / model (c DebugPhysics\'ten)', rf['sigma'], sig, rf['sigma'] - sig, 0.02, '1/s')
        row('S4', 'yüzey roll max |açı| [°]: ilk 1 s → son 5 s (bilgi)', out['S4']['roll']['amp_first_deg'],
            out['S4']['roll']['amp_last5_deg'], np.nan, np.inf, '°', ok=True)
        R['_rf'] = rf

    # S5: dalmis salinim (notr varyant)
    out['S5'] = {}
    for key, ang, ax, Tp, Tp0, tend, tol in (('roll_neutral', 0, 0, pr['T_roll'], pr['T_roll_noadd'], 4.0, 0.01),
                                              ('pitch_neutral', 1, 1, pr['T_pitch'], pr['T_pitch_noadd'], 40.0, 0.02)):
        r = R[key]
        if not r:
            continue
        O, D = r['O'], r['D']
        fit = damped_fit(O['t'], O['rpy'][:, ang], tend)
        I_eff = pr['B'] * pr['bg'] / fit['wn'] ** 2
        # kaldirma momenti (dunya ekseni, CG etrafinda) = -B BG sin(theta)
        k_rel, k_lag = torque_gain(r, ang, ax, pr['B'] * pr['bg'])
        k_tb = k_rel * pr['B'] * pr['bg']
        lbl = 'roll' if ang == 0 else 'pitch'
        last5 = O['t'] >= O['t'][-1] - 5.0
        out['S5'][lbl] = dict(fit={k: v for k, v in fit.items() if not isinstance(v, np.ndarray)},
                              I_eff=I_eff, aI_meas=I_eff - pr['I'][ax], k_tb=k_tb, k_lag=k_lag,
                              amp_first_deg=float(np.degrees(np.abs(O['rpy'][O['t'] <= 1.0, ang]).max())),
                              amp_last5_deg=float(np.degrees(np.abs(O['rpy'][last5, ang]).max())), t_end=float(O['t'][-1]))
        row('S5', f'{lbl} doğal periyodu T_n [s] (I + aI)', fit['T_n'], Tp, rel(fit['T_n'], Tp), tol, 's')
        row('S5', f'{lbl} T_n / T_n(ek atalet yok)', fit['T_n'] / Tp0, Tp / Tp0, rel(fit['T_n'] / Tp0, Tp / Tp0), tol)
        row('S5', f'{lbl} ek atalet aI [kg·m²]', I_eff - pr['I'][ax], pr['aI'][ax], (I_eff - pr['I'][ax]) - pr['aI'][ax],
            2 * tol * (pr['I'][ax] + pr['aI'][ax]), 'kg·m²')
        row('S5', f'{lbl} kaldırma momenti / (B·BG·sinθ) (gecikme taramalı)', k_rel, 1.0, k_rel - 1, 1e-3)
        sm = sigma_model(pr, ax, RATE)
        out['S5'][lbl]['sigma_model'] = sm
        row('S5', f'{lbl} sönüm σ [1/s] (− = büyüyen) / model', fit['sigma'], sm['sigma'], fit['sigma'] - sm['sigma'], 0.02, '1/s')
        row('S5', f'{lbl} max |açı| [°]: ilk 1 s → son 5 s (bilgi)', out['S5'][lbl]['amp_first_deg'],
            out['S5'][lbl]['amp_last5_deg'], np.nan, np.inf, '°', ok=True)
        R['_' + lbl] = fit
    for hz in (500, 50):
        r = R[f'roll_neutral_{hz}']
        if not r:
            continue
        O, lbl = r['O'], f'roll_{hz}'
        fit = damped_fit(O['t'], O['rpy'][:, 0], 4.0)
        last5 = O['t'] >= O['t'][-1] - 5.0
        out['S5'][lbl] = dict(fit={k: v for k, v in fit.items() if not isinstance(v, np.ndarray)},
                              amp_first_deg=float(np.degrees(np.abs(O['rpy'][O['t'] <= 1.0, 0]).max())),
                              amp_last5_deg=float(np.degrees(np.abs(O['rpy'][last5, 0]).max())))
        row('S5', f'roll T_n [s], {hz} Hz sim', fit['T_n'], pr['T_roll'], rel(fit['T_n'], pr['T_roll']), 0.01, 's')
        sm = sigma_model(pr, 0, float(hz))
        out['S5'][lbl]['sigma_model'] = sm
        row('S5', f'roll sönüm σ [1/s], {hz} Hz sim / model', fit['sigma'], sm['sigma'], fit['sigma'] - sm['sigma'], 0.02, '1/s')
        row('S5', f'roll max |açı| [°], {hz} Hz: ilk 1 s → son 5 s (bilgi)', out['S5'][lbl]['amp_first_deg'],
            out['S5'][lbl]['amp_last5_deg'], np.nan, np.inf, '°', ok=True)
        R['_' + lbl] = fit
    out['aoa'] = aoa_table(pr)
    out['rows'] = [{k: v for k, v in rw.items() if k != 'fmt'} for rw in rows]
    return pr, R, rows, out


# ---------------------------------------------------------------- cikti
def fmt(v, f='{:.6g}'):
    if isinstance(v, (list, tuple)):
        return '(' + ', '.join(f'{x:.6g}' for x in v) + ')'
    return f.format(v) if not isinstance(v, str) else v


def tables(pr, R, rows, out):
    L = []
    L += ['## Koşular, zaman tabanı ve itki temizliği', '',
          'Sayım oranı = mesaj sayısı / (rate × kayıt süresi + 1); 1 = düşme yok. İtki/tork max: ThrusterState\'te '
          'kaydedilen en büyük |değer| (sıfır setpoint\'e rağmen; bkz. SimpleThruster bulgusu). İlk durum: ilk kaydedilen '
          'DebugPhysics mesajında max |açısal hız| ve |u|, |v|.', '',
          '| Koşu | Debug N / oran | Odometri N / oran | max \\|itki\\| [N] / \\|tork\\| [N·m] | İlk \\|ω\\| [rad/s] / \\|u,v\\| [m/s] |',
          '|---|---|---|---|---|']
    for t in out['runs']:
        L.append(f'| {t["run"]} | {t["debug_n"]} / {t["debug_ratio"]:.4f} | {t["odom_n"]} / {t["odom_ratio"]:.4f} | '
                 f'{t["thrust_max"]:.3g} / {t["torque_max"]:.3g} | {t["first_ang"]:.2g} / {t["first_lin_xy"]:.2g} |')
    L += ['']
    names = dict(S1='S1. Yükleme ve tutarlılık (depth_release, ilk mesaj)', S2='S2. Varsayılan hidrodinamik (depth_default)',
                 S3='S3. Serbest yükselme (depth_release)', S4='S4. Yüzey dengesi (surface, surface_pitch, surface_roll)',
                 S5='S5. Dalmış salınım, nötr varyant (roll_neutral[_500, _50], pitch_neutral)')
    for s in ('S1', 'S2', 'S3', 'S4', 'S5'):
        rs = [r for r in rows if r['test'] == s]
        if not rs:
            continue
        L += [f'## {names[s]}', '', '| Büyüklük | Ölçülen | Tahmin | Fark | Tolerans | Sonuç |', '|---|---|---|---|---|---|']
        for r in rs:
            tol = '—' if not np.isfinite(r['tol']) else f'{r["tol"]:.2g}'
            res = r['result'] if np.isfinite(r['tol']) else 'bilgi'
            err = '—' if not np.isfinite(r['err']) else f'{r["err"]:+.3g}'
            pred = '—' if isinstance(r['pred'], float) and not np.isfinite(r['pred']) else fmt(r['pred'], r['fmt'])
            L.append(f'| {r["name"]} | {fmt(r["meas"], r["fmt"])} | {pred} | {err} | {tol} | {res} |')
        L += ['']
    a1, a2 = out.get('S3'), out.get('S2')
    if a1:
        L += [f'S3 ayrıntı: M_eff fitinde {a1["n_fit"]} örnek (|F_net| > 1 N), kuvvet kayması {a1["shift"]}, artık std '
              f'{a1["res"]:.3g} N. Fiziksel Δt / nominal: {a1["dt_min_over_nom"]:.3f} … {a1["dt_max_over_nom"]:.3f}. '
              f'Yüz bazlı port kıyası: {a1["port"]["n"]} kuvvet güncellemesi, hız kayması {a1["port"]["shift"]} mesaj, '
              f'medyan göreli fark F_p {a1["port"]["fp_med"]:.1e} / F_f {a1["port"]["ff_med"]:.1e}. '
              f'Yükselmede max |pitch| {a1["pitch_max_deg"]:.2f}°, max |u| {a1["u_max"]:.3g} m/s. '
              f'Fiziksel heave M (m + m_a,z, Lamb) = {pr["m_eff_phys_heave"]:.2f} kg; ek kütlesiz m = {pr["m"]:.2f} kg.', '']
        if a1['pitch_range_after_deg']:
            L += [f'Yüzeye çıkış (bilgi, Faz 6): eksen derinliği min {100*a1["z_min_after"]:.2f} cm '
                  f'(negatif = eksen su üstünde), pitch aralığı {a1["pitch_range_after_deg"][0]:.2f}° … '
                  f'{a1["pitch_range_after_deg"][1]:.2f}°.', '']
    if 'S4' in out:
        s4 = out['S4']
        L += [f'S4 ayrıntı: dik (trimsiz) denge eksen derinliği tahmini {100*pr["z_eq_upright"]:.3f} cm; trim dahil '
              f'{100*pr["z_eq"]:.3f} cm (orijin). Son 30 s z std {1000*s4["z_std_last30"]:.3f} mm. '
              f'A_wp = {1e4*pr["a_wp"]:.1f} cm², GM_L (±0.05°) = {100*pr["gm_l"]:.2f} cm, GM_L (2° sekant) = '
              f'{100*pr["gm_l2"]:.2f} cm. Heave fiti: ζ = {s4["heave"]["zeta"]:.3f}, göreli rms {s4["heave"]["rms_rel"]:.3f}.'
              + (f' Pitch fiti: ζ = {s4["pitch"]["zeta"]:.3f}, göreli rms {s4["pitch"]["rms_rel"]:.3f}.' if s4.get('pitch') else '')
              + (f' GM_T (yüzey, ±0.05°) = {100*pr["gm_t"]:.3f} cm. Yüzey roll fiti: σ = {s4["roll"]["fit"]["sigma"]:+.4f} 1/s '
                 f'(model {s4["roll"]["sigma_model"]:+.4f}), göreli rms {s4["roll"]["fit"]["rms_rel"]:.3f}. Yüzeyde roll sönüm '
                 f'katsayısı c = {s4["roll"]["c_surface"]:.4g} N·m·s (dalmışta port {s4["roll"]["c_submerged"]:.4g}); '
                 f'sürtünme torku yönünden C_eff / C_f,x medyanı {s4["roll"]["ceff_ratio"]:.2f}.'
                 if s4.get('roll') else ''), '']
    for lbl in ('roll', 'pitch', 'roll_500', 'roll_50'):
        s = out['S5'].get(lbl)
        if s:
            f = s['fit']
            L += [f'S5 {lbl}: ω_d = {f["wd"]:.4f} rad/s, σ = {f["sigma"]:.4f} 1/s, ζ = {f["zeta"]:.4f}, '
                  f'sıfır geçiş periyodu {f["T_zc"]:.4f} s ({f["n_zc"]} geçiş), göreli rms {f["rms_rel"]:.3f}.'
                  + (f' I_eff = B·BG/ω_n² = {s["I_eff"]:.4f} kg·m²; moment fitinde gecikme {1000*s["k_lag"]:.0f} ms.'
                     if 'I_eff' in s else '')
                  + (f' Sönüm modeli: c = {s["sigma_model"]["c"]:.4g} N·m·s (port), σ_fiz = {s["sigma_model"]["sigma_phys"]:+.4f}, '
                     f'σ_gecikme = −{s["sigma_model"]["sigma_neg"]:.4f} (P = {s["sigma_model"]["P"]}, '
                     f'τ = {1000*s["sigma_model"]["tau"]:.1f} ms) → σ = {s["sigma_model"]["sigma"]:+.4f} 1/s.'
                     if 'sigma_model' in s else ''), '']
    L += ['## Faz 3 öngörüsü: surge direnci ve hücum açısı (yüz bazlı port, kalibre C_d/C_f)', '',
          'Akış gövdeye göre α kadar eğik. X: gövde x direnci; artış α = 0\'a göre. Port S2/S3\'te DebugPhysics ile '
          'doğrulandı (medyan 2e-5); bu tablo canlı surge testi değil (Faz 3).', '',
          '| u [m/s] | α [°] | X [N] | F_p,x / F_f,x [N] | Artış | Z [N] |', '|---|---|---|---|---|---|']
    for a in out['aoa']:
        L.append(f'| {a["u"]:.1f} | {a["alpha"]:.2f} | {a["X"]:.4f} | {a["Fp_x"]:.4f} / {a["Ff_x"]:.4f} | '
                 f'{100*a["rel"]:+.1f}% | {a["Z"]:+.4f} |')
    L += ['']
    n_fail = sum(1 for r in rows if r['result'] == 'FAIL')
    L += [f'**Özet:** {sum(1 for r in rows if r["result"] == "PASS" and np.isfinite(r["tol"]))} PASS, {n_fail} FAIL, '
          f'{sum(1 for r in rows if not np.isfinite(r["tol"]))} bilgi satırı.', '']
    return '\n'.join(L)


def save(fig, name):
    os.makedirs(FIG, exist_ok=True)
    for ext in ('png', 'pdf'):
        meta = {'CreationDate': None} if ext == 'pdf' else {}
        fig.savefig(os.path.join(FIG, f'{name}.{ext}'), dpi=200, bbox_inches='tight', facecolor=SURF, metadata=meta)
    plt.close(fig)


def figures(pr, R):
    if '_a1' in R:
        fig, axes = plt.subplots(1, 3, figsize=(13, 3.9))
        for ax in axes:
            style(ax)
        for key, ak, col, lbl, wp in (('depth_release', '_a1', C1, 'Kalibre C_d/C_f', pr['w_cal']),
                                      ('depth_default', '_a2', C2, 'Varsayılan C_d/C_f', pr['w_def'])):
            if ak not in R:
                continue
            D = R[key]['D']
            axes[0].plot(D['t'], -D['lin'][:, 2], color=col, lw=1.6, label=lbl)
            axes[0].axhline(wp, color=col, ls='--', lw=1.0)
            axes[1].plot(D['t'], D['z'], color=col, lw=1.6, label=lbl)
        axes[0].set_ylabel('Yükselme hızı −w [m/s]', color=INK)
        axes[0].set_title('Serbest yükselme (kesik: tahmini w∞)', color=INK, fontsize=10, loc='left')
        axes[1].axhline(0, color=INK2, lw=0.8)
        axes[1].invert_yaxis()
        axes[1].set_ylabel('Eksen derinliği z [m] (NED)', color=INK)
        axes[1].set_title('Derinlik; yüzeye çıkış', color=INK, fontsize=10, loc='left')
        for ax in axes[:2]:
            ax.set_xlabel('t [s] (ilk kaydedilen mesajdan)', color=INK)
            ax.legend(fontsize=8, frameon=False, labelcolor=INK)
        a = R['_a1']
        n = a['n']
        acc = np.diff(a['w'][:n]) / DT
        f = a['fnet'][a['shift']:n - 1 + a['shift']]
        sel = np.abs(f) > 1.0
        ax = axes[2]
        ax.plot(acc[sel], f[sel], '.', ms=3, color=C1, label='Sim (kalibre)')
        aa = np.linspace(0, acc[sel].min() * 1.05, 10)
        for M, col, ls, lbl in ((a['M'], INK, '-', f'Fit: M = {a["M"]:.2f} kg'),
                                (pr['m'], C3, '--', f'm (ek kütlesiz) = {pr["m"]:.2f} kg'),
                                (pr['m_eff_phys_heave'], C2, ':', f'Fiziksel m + m_a,z = {pr["m_eff_phys_heave"]:.2f} kg')):
            ax.plot(aa, M * aa, ls, color=col, lw=1.4, label=lbl)
        ax.set_xlabel('ẇ [m/s²]', color=INK)
        ax.set_ylabel('F_net,z [N]', color=INK)
        ax.set_title('Etkin kütle (Stonefish: m + ort(m_a))', color=INK, fontsize=10, loc='left')
        ax.legend(fontsize=8, frameon=False, labelcolor=INK)
        fig.tight_layout()
        save(fig, '02_yukselme')

    if 'surface' in R and R['surface']:
        fig, axes = plt.subplots(1, 2, figsize=(12, 3.8))
        for ax in axes:
            style(ax)
        O = R['surface']['O']
        axes[0].plot(O['t'], 100 * O['pos'][:, 2], color=C1, lw=1.2, label='Sim')
        axes[0].axhline(100 * pr['z_eq'], color=INK, ls='--', lw=1.0, label='Tahmin (trim dahil)')
        axes[0].invert_yaxis()
        axes[0].set_ylabel('Eksen derinliği [cm] (NED)', color=INK)
        axes[0].set_title('Yüzey dengesi (başlangıç: denge + 1 cm)', color=INK, fontsize=10, loc='left')
        if '_pf' in R:
            pf = R['_pf']
            axes[1].plot(pf['t'], np.degrees(pf['y']), color=C1, lw=1.2, label='Sim')
            axes[1].plot(pf['t'], np.degrees(pf['fit']), color=INK, ls='--', lw=1.0, label=f'Sönümlü sinüs, T_n = {pf["T_n"]:.3f} s')
            axes[1].set_ylabel('Pitch [°]', color=INK)
            axes[1].set_title('Yüzeyde pitch salınımı (başlangıç +2°)', color=INK, fontsize=10, loc='left')
        for ax in axes:
            ax.set_xlabel('t [s]', color=INK)
            ax.legend(fontsize=8, frameon=False, labelcolor=INK)
        fig.tight_layout()
        save(fig, '02_yuzey')

    if '_roll' in R or '_pitch' in R:
        fig, axes = plt.subplots(1, 3, figsize=(16, 3.8))
        ax = axes[2]
        style(ax)
        for key, col, lbl in (('roll_neutral_500', C2, '500 Hz'), ('roll_neutral', C1, '100 Hz'), ('roll_neutral_50', C3, '50 Hz')):
            if R.get(key):
                O = R[key]['O']
                ax.plot(O['t'], np.degrees(O['rpy'][:, 0]), color=col, lw=0.8, label=lbl)
        ax.set_xlabel('t [s]', color=INK)
        ax.set_ylabel('Roll [°]', color=INK)
        ax.set_title('Roll, tüm kayıt (nötr, 5 m): sim hızına bağlı büyüme', color=INK, fontsize=10, loc='left')
        ax.legend(fontsize=8, frameon=False, labelcolor=INK)
        for ax, lbl, Tp, Tp0 in ((axes[0], 'roll', pr['T_roll'], pr['T_roll_noadd']),
                                 (axes[1], 'pitch', pr['T_pitch'], pr['T_pitch_noadd'])):
            style(ax)
            if '_' + lbl not in R:
                continue
            f = R['_' + lbl]
            ax.plot(f['t'], np.degrees(f['y']), color=C1, lw=1.4, label='Sim (nötr varyant, 5 m)')
            ax.plot(f['t'], np.degrees(f['fit']), color=INK, ls='--', lw=1.0, label=f'Fit: T_n = {f["T_n"]:.3f} s')
            ax.set_title(f'{lbl.capitalize()} salınımı. Tahmin T_n: {Tp:.3f} s (I + aI), {Tp0:.3f} s (I)',
                         color=INK, fontsize=10, loc='left')
            ax.set_xlabel('t [s]', color=INK)
            ax.set_ylabel(f'{lbl.capitalize()} [°]', color=INK)
            ax.legend(fontsize=8, frameon=False, labelcolor=INK)
        fig.tight_layout()
        save(fig, '02_salinim')


def plain(o):
    if isinstance(o, dict):
        return {k: plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple, np.ndarray)):
        return [plain(v) for v in o]
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    if isinstance(o, float) and not np.isfinite(o):
        return None
    return o


def main():
    cfg = yaml.safe_load(open(os.path.join(ROOT, 'config', 'vehicle.yaml')))
    d = yaml.safe_load(open(os.path.join(ROOT, 'config', 'vehicle_derived.yaml')))
    pr, R, rows, out = analyze(cfg, d)
    md = tables(pr, R, rows, out)
    with open(os.path.join(ROOT, 'docs', '02_statik_tablolar.md'), 'w') as fh:
        fh.write('# Faz 2 statik test tabloları\n\n> OTOMATİK ÜRETİLDİ: `scripts/static_tests.sh` + '
                 '`python3 scripts/analyze_static.py`. Tahminler `config/vehicle_derived.yaml`\'dan.\n\n' + md + '\n')
    with open(os.path.join(ROOT, 'docs', '02_statik_sonuclar.json'), 'w') as fh:
        json.dump(plain(out), fh, indent=1, ensure_ascii=False)
    figures(pr, R)
    print(md)


if __name__ == '__main__':
    main()

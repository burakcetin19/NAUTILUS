#!/usr/bin/env python3
"""Faz 0.5 analizi: direnc ussu, Cd/Cf override, zaman tabani, tek parca mesh.

Kullanim: python3 analyze_phase0b.py   (bags/ altindaki r1/r2/r3 kosularini okur)
Cikti: stdout ozet, bags/phase0b_results.json, ../../figures/faz0b/*.png|pdf
"""
import json
import os
import numpy as np
from scipy import stats, optimize
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_probe import read_bag, stamp

HERE = os.path.dirname(os.path.abspath(__file__))
BAGS = os.path.join(HERE, 'bags')
FIG = os.path.normpath(os.path.join(HERE, '..', '..', 'figures', 'faz0b'))
RHO, G = 1000.0, 9.81
LEVELS = [5.0, 10.0, 20.0, 35.0, 50.0, 80.0]
WIN = 5.0                                  # kararli hal penceresi [s]
CYL = dict(a_front=np.pi * 0.1**2, s_t=0.629331, volume=np.pi * 0.1**2)  # analyze_probe ile ayni
MESH = json.load(open(os.path.join(HERE, 'meshes', 'torpedo_L1_D02.json')))
OVR = dict(cd=(0.3, 0.9, 0.9), cf=(0.004, 0.02, 0.02))
# dataviz referans paleti (acik mod), ilk 3 kategorik slot + metin/izgara
COL = {'cyl_def': '#2a78d6', 'cyl_ovr': '#eb6834', 'torp_sub': '#1baf7a'}
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#ffffff'
LABEL = {'cyl_def': 'Silindir, varsayılan Cd/Cf', 'cyl_ovr': 'Silindir, override Cd/Cf',
         'torp_sub': 'Torpido mesh (dalmış)'}


# ---------------------------------------------------------------- veri
def load_robot(d, dbg_t, odom_t, thr_t=None):
    dbg = d[dbg_t]
    r = dict(
        t_st=np.array([stamp(m) for _, m in dbg]),
        t_rec=np.array([tr for tr, _ in dbg]),
        u=np.array([m.velocity.linear.x for _, m in dbg]),
        fp=np.array([m.damping.force.x for _, m in dbg]),
        ff=np.array([m.skin_friction.force.x for _, m in dbg]),
        fb=np.array([m.buoyancy.force.z for _, m in dbg]),
        vsub=np.array([m.submerged_volume for _, m in dbg]),
        mass=dbg[0][1].mass, volume=dbg[0][1].volume, surface=dbg[0][1].surface,
        cd=np.array([dbg[0][1].damping_coeff.x, dbg[0][1].damping_coeff.y, dbg[0][1].damping_coeff.z]),
        cf=np.array([dbg[0][1].skin_friction_coeff.x, dbg[0][1].skin_friction_coeff.y,
                     dbg[0][1].skin_friction_coeff.z]))
    od = d[odom_t]
    r['o_st'] = np.array([stamp(m) for _, m in od])
    r['o_rec'] = np.array([tr for tr, _ in od])
    r['o_p'] = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for _, m in od])
    r['o_q'] = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                          m.pose.pose.orientation.z, m.pose.pose.orientation.w] for _, m in od])
    r['o_v'] = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for _, m in od])
    if thr_t:
        th = d[thr_t]
        T = np.array([m.thrust[0] for _, m in th])
        ts = np.array([stamp(m) for _, m in th])
        # ikisi de her sim adiminda ard arda yayinlaniyor -> indeks hizalama;
        # kayit baslangici/sonu kaynakli birkac mesajlik kaymayi damga eslemesiyle bul
        n = len(r['t_st'])

        def overlap(k):   # debug[i] <-> thrust[i + k]
            lo, lo_t = max(-k, 0), max(k, 0)
            return lo, lo_t, min(n - lo, len(T) - lo_t)

        def cost(k):
            lo, lo_t, m_ = overlap(k)
            return np.mean(np.abs(ts[lo_t:lo_t + m_] - r['t_st'][lo:lo + m_]))
        best = min(range(-3, 4), key=cost)
        lo, lo_t, m_ = overlap(best)
        Ta = np.full(n, np.nan)
        Ta[lo:lo + m_] = T[lo_t:lo_t + m_]
        r['T'] = np.nan_to_num(Ta, nan=0.0)
        r['T_shift'] = best
    return r


def estimate_mass(u, fnet, dt):
    """M: F_net = M*du/dt regresyonu; kuvvetin hangi mesajla hizalandigi veriden secilir."""
    best = None
    for shift in (0, 1):          # 0: F_i -> (u_{i+1}-u_i); 1: F_{i+1} -> (u_{i+1}-u_i)
        a = np.diff(u) / dt
        f = fnet[shift:len(fnet) - 1 + shift]
        sel = np.abs(f) > 1.0
        M = np.sum(f[sel] * a[sel]) / np.sum(a[sel] ** 2)
        res = np.std(f[sel] - M * a[sel])
        if best is None or res < best[2]:
            best = (M, shift, res)
    return best


# ---------------------------------------------------------------- madde 1
def loglog_fit(u, F):
    x, y = np.log10(np.abs(u)), np.log10(np.abs(F))
    res = stats.linregress(x, y)
    n = len(x)
    ci = stats.t.ppf(0.975, n - 2) * res.stderr if n > 2 else np.nan
    return dict(n=res.slope, c=res.intercept, r2=res.rvalue**2, ci95=ci, N=n)


def steady_points(r, dt):
    M, shift, _ = estimate_mass(r['u'], r['T'] + r['fp'] + r['ff'], dt)
    nwin = int(round(WIN / dt))
    pts = []
    for T in LEVELS:
        idx = np.where(np.abs(r['T'] - T) < 1e-6)[0]
        if idx.size < nwin:
            continue
        w = idx[-nwin:]
        tt = np.arange(nwin) * dt
        slope = np.polyfit(tt, r['u'][w], 1)[0]
        # odometri capraz kontrolu (ayni duvar-zamani araligi)
        ow = np.where((r['o_rec'] >= r['t_rec'][w[0]]) & (r['o_rec'] <= r['t_rec'][w[-1]]))[0]
        u_tw = r['o_v'][ow, 0].mean()
        path = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(r['o_p'][ow], axis=0), axis=1))])
        u_pos = np.polyfit(np.arange(len(ow)) * 0.02, path, 1)[0]
        pts.append(dict(T=T, u=r['u'][w].mean(), fp=r['fp'][w].mean(), ff=r['ff'][w].mean(),
                        steady=abs(slope) * M / T, u_odom_twist=u_tw, u_odom_pos=u_pos))
    return pts, M, shift


def item1(r, dt):
    pts, M, shift = steady_points(r, dt)
    u = np.array([p['u'] for p in pts])
    out = dict(points=pts, M=M, force_shift=shift,
               fp=loglog_fit(u, [p['fp'] for p in pts]),
               ff=loglog_fit(u, [p['ff'] for p in pts]))
    sel = (np.abs(r['u']) > 0.02) & (np.abs(r['fp']) > 1e-9) & (np.abs(r['ff']) > 1e-9)
    out['fp_dense'] = loglog_fit(r['u'][sel], r['fp'][sel])
    out['ff_dense'] = loglog_fit(r['u'][sel], r['ff'][sel])
    return out


# ---------------------------------------------------------------- madde 4
def timebase(r, dt, odom_dt, M=None):
    out = {}
    span = r['t_rec'][-1] - r['t_rec'][0]
    out['n_debug'], out['span_wall'] = len(r['u']), span
    out['n_debug_expected'] = span / dt + 1
    out['rtf'] = (len(r['u']) - 1) * dt / span
    ospan = r['o_rec'][-1] - r['o_rec'][0]
    out['n_odom'], out['n_odom_expected'] = len(r['o_p']), ospan / odom_dt + 1
    out['stamp_minus_rec_ms'] = dict(mean=1e3 * np.mean(r['t_st'] - r['t_rec']),
                                     std=1e3 * np.std(r['t_st'] - r['t_rec']))
    hd = np.diff(r['o_st'])
    out['odom_header_dt_ms'] = dict(mean=1e3 * hd.mean(), std=1e3 * hd.std(), min=1e3 * hd.min(), max=1e3 * hd.max())
    hdd = np.diff(r['t_st'])
    out['debug_header_dt_ms'] = dict(mean=1e3 * hdd.mean(), std=1e3 * hdd.std(), min=1e3 * hdd.min(), max=1e3 * hdd.max())
    # kinematik sim-zamani: |dp| / ort.hiz  (formulden bagimsiz)
    s = np.linalg.norm(r['o_v'], axis=1)
    sbar = 0.5 * (s[1:] + s[:-1])
    dp = np.linalg.norm(np.diff(r['o_p'], axis=0), axis=1)
    sel = sbar > 0.05
    dt_kin = dp[sel] / sbar[sel]
    out['odom_kin_dt_ms'] = dict(mean=1e3 * dt_kin.mean(), std=1e3 * dt_kin.std(),
                                 min=1e3 * dt_kin.min(), max=1e3 * dt_kin.max(), N=int(sel.sum()))
    out['odom_drops'] = int(np.sum(dt_kin > 1.5 * odom_dt))
    out['_hd'], out['_kin'] = hd, dt_kin
    # dinamik sim-zamani: du*M/F_net (DebugPhysics, her adim)
    if 'T' in r:
        if M is None:
            M = estimate_mass(r['u'], r['T'] + r['fp'] + r['ff'], dt)[0]
        M_, shift, _ = estimate_mass(r['u'], r['T'] + r['fp'] + r['ff'], dt)
        fnet = (r['T'] + r['fp'] + r['ff'])[shift:len(r['u']) - 1 + shift]
        du = np.diff(r['u'])
        selr = np.abs(fnet) > 2.0
        dt_dyn = du[selr] * M_ / fnet[selr]
        out['debug_dyn_dt_ms'] = dict(mean=1e3 * dt_dyn.mean(), std=1e3 * dt_dyn.std(),
                                      min=1e3 * dt_dyn.min(), max=1e3 * dt_dyn.max(), N=int(selr.sum()))
        out['debug_drops'] = int(np.sum(dt_dyn > 1.5 * dt))
        out['_hdd'], out['_dyn'] = hdd, dt_dyn
    return out


# ---------------------------------------------------------------- madde 5
RX = np.concatenate([[-MESH['params']['L'] / 2], MESH['rings_x'], [MESH['params']['L'] / 2]])
RR = np.concatenate([[0.0], MESH['rings_r'], [0.0]])
NT = MESH['params']['n_theta']
TH = 2 * np.pi * np.arange(NT) / NT
XG = np.unique(np.concatenate([RX, np.linspace(RX[0], RX[-1], 4001)]))


def clip_area(P, dep):
    out = []
    for i in range(len(P)):
        j = (i + 1) % len(P)
        if dep[i] >= 0:
            out.append(P[i])
        if (dep[i] >= 0) != (dep[j] >= 0):
            out.append(P[i] + dep[i] / (dep[i] - dep[j]) * (P[j] - P[i]))
    if len(out) < 3:
        return 0.0
    q = np.array(out)
    return 0.5 * abs(np.dot(q[:, 0], np.roll(q[:, 1], -1)) - np.dot(q[:, 1], np.roll(q[:, 0], -1)))


def vsub_slices(z0, Rm):
    """Govde x'i boyunca dilimleyip su alti (derinlik>0) kesit alanini integre eder."""
    A = np.empty_like(XG)
    for k, x in enumerate(XG):
        r = np.interp(x, RX, RR)
        P = np.stack([r * np.cos(TH), r * np.sin(TH)], axis=1)           # govde (y, z)
        dep = z0 + Rm[2, 0] * x + Rm[2, 1] * P[:, 0] + Rm[2, 2] * P[:, 1]
        A[k] = clip_area(P, dep) if r > 0 else 0.0
    return np.trapz(A, XG)


def item5(rs, rf, dt):
    m = MESH['mesh']
    sub = dict(volume_sim=rs['volume'], volume_mesh=m['volume'], surface_sim=rs['surface'],
               surface_mesh=m['surface'], mass=rs['mass'], fb_sim=rs['fb'][0], fb_mesh=-RHO * G * m['volume'],
               cd=rs['cd'].tolist(), cf=rs['cf'].tolist())
    pts, _, _ = steady_points(rs, dt)
    for p in pts:
        p['fp_formula'] = -0.5 * RHO * rs['cd'][0] * m['a_front'] * p['u'] ** 2 * abs(p['u'])
        p['ff_formula'] = -RHO * rs['cf'][0] * m['s_t'] * p['u']
    sub['points'] = pts
    # yuzer torpido: son 40 s
    nwin = int(round(40.0 / dt))
    w = slice(len(rf['u']) - nwin, None)
    ow = rf['o_rec'] >= rf['t_rec'][-nwin]
    z = rf['o_p'][ow, 2].mean()
    q = rf['o_q'][ow].mean(axis=0)
    q /= np.linalg.norm(q)
    rot = Rotation.from_quat(q)
    Rm = rot.as_matrix()
    roll, pitch, _ = rot.as_euler('xyz', degrees=True)
    v_target = rf['mass'] / RHO
    z_eq = optimize.brentq(lambda zz: vsub_slices(zz, np.eye(3)) - v_target, -0.1, 0.1, xtol=1e-7)
    flt = dict(mass=rf['mass'], v_target=v_target, vsub_sim=rf['vsub'][w].mean(),
               vsub_slices_at_pose=vsub_slices(z, Rm), fb_sim=rf['fb'][w].mean(), weight=rf['mass'] * G,
               z_sim=z, z_eq_pred=z_eq, roll_deg=roll, pitch_deg=pitch,
               z_std=rf['o_p'][ow, 2].std())
    return sub, flt


# ---------------------------------------------------------------- grafikler
def style(ax):
    ax.set_facecolor(SURF)
    ax.grid(True, which='both', color=GRID, lw=0.6)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.xaxis.label.set_color(INK)
    ax.yaxis.label.set_color(INK)


def save(fig, name):
    os.makedirs(FIG, exist_ok=True)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(FIG, f'{name}.{ext}'), dpi=200, bbox_inches='tight', facecolor=SURF)
    plt.close(fig)


def fig_loglog(res, robots):
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.6))
    # referans egim cizgileri: veri olmayan sag-alt bolgede, bir dekad boyunca
    for ax, key, title, refs, f0 in ((axes[0], 'fp', 'Basınç (form) direnci  $|F_p|$', (2, 3), 2e-4),
                                     (axes[1], 'ff', 'Sürtünme direnci  $|F_f|$', (1, 2), 0.04)):
        style(ax)
        for name in ('cyl_def', 'cyl_ovr', 'torp_sub'):
            r, it = robots[name], res[name]
            sel = (np.abs(r['u']) > 0.02)
            ax.loglog(np.abs(r['u'][sel][::5]), np.abs(r[key][sel][::5]), '.', ms=1.2, color=COL[name], alpha=0.25)
            u = np.array([p['u'] for p in it['points']])
            F = np.abs([p[key] for p in it['points']])
            f = it[key]
            uu = np.linspace(u.min() * 0.8, u.max() * 1.2, 50)
            ax.loglog(uu, 10 ** f['c'] * uu ** f['n'], '-', lw=1.6, color=COL[name])
            ax.loglog(u, F, 'o', ms=6.5, color=COL[name], mec=SURF, mew=1.2,
                      label=f"{LABEL[name]}:  n = {f['n']:.4f},  R² = {f['r2']:.7f}")
        ref_u = np.array([0.25, 2.5])
        for n in refs:
            ax.loglog(ref_u, f0 * (ref_u / ref_u[0]) ** n, '--', lw=1.0, color=INK2)
            ax.annotate(f'eğim {n}', (ref_u[1], f0 * 10.0 ** n), xytext=(4, 0), textcoords='offset points',
                        color=INK2, fontsize=8, va='center')
        ax.set_xlabel('Hız  u  [m/s]')
        ax.set_ylabel('Kuvvet  |F|  [N]')
        ax.set_title(title, color=INK, fontsize=11, loc='left')
        ax.legend(fontsize=8, frameon=False, loc='upper center', bbox_to_anchor=(0.5, -0.16), labelcolor=INK)
    fig.suptitle('Stonefish DebugPhysics: kararlı hal noktaları (●), geçiş örnekleri (·), log-log fit (—), '
                 'referans eğimler (- -)', color=INK, fontsize=10, x=0.01, ha='left')
    fig.tight_layout()
    save(fig, '00b_loglog_direnc')


def fig_timebase(tb):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    for ax, a, b, nom, title in ((axes[0], '_hd', '_kin', 20.0, 'Odometri (50 Hz)'),
                                 (axes[1], '_hdd', '_dyn', 10.0, 'DebugPhysics (her sim adımı, 100 Hz)')):
        style(ax)
        bins = np.linspace(0, 2.6 * nom, 105)
        ax.hist(1e3 * tb[a], bins=bins, histtype='step', lw=1.6, color='#2a78d6', label='Header damgası farkı')
        ax.hist(1e3 * tb[b], bins=bins, histtype='step', lw=1.6, color='#eb6834',
                label='Fiziksel Δt (kinematik: |Δp| / ū)' if a == '_hd' else 'Fiziksel Δt (dinamik: Δu·M / F_net)')
        ax.axvline(nom, color=INK2, lw=1.0, ls='--')
        ax.axvline(2 * nom, color=INK2, lw=0.8, ls=':')
        ax.text(2 * nom, ax.get_ylim()[1] * 0.5, '  1 mesaj düşse\n  burada olurdu', color=INK2, fontsize=8)
        ax.set_yscale('log')
        ax.set_xlabel('Ardışık mesajlar arası süre  [ms]')
        ax.set_ylabel('Örnek sayısı')
        ax.set_title(title, color=INK, fontsize=11, loc='left')
        ax.legend(fontsize=8, frameon=False, labelcolor=INK, loc='upper center', bbox_to_anchor=(0.5, -0.17))
    fig.tight_layout()
    save(fig, '00b_zaman_tabani')


def fig_float(rf, flt, dt):
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    style(ax)
    t = np.arange(len(rf['o_p'])) * 0.02
    ax.plot(t, 100 * rf['o_p'][:, 2], lw=1.6, color='#2a78d6', label='Sim: eksen derinliği (odometri z)')
    ax.axhline(100 * flt['z_eq_pred'], color=INK2, lw=1.2, ls='--',
               label=f"Öngörü (mesh dilimleme, V_sub = 0.6V): {100*flt['z_eq_pred']:.3f} cm")
    ax.set_xlabel('Sim zamanı (indeks × 0.02 s)  [s]')
    ax.set_ylabel('Derinlik z  [cm]  (NED, + aşağı)')
    ax.set_title('Yüzen torpido (ρ = 600 kg/m³): denge su çekimi', color=INK, fontsize=11, loc='left')
    ax.set_xlim(0, 30)
    ax.legend(fontsize=8, frameon=False, labelcolor=INK)
    fig.tight_layout()
    save(fig, '00b_mesh_yuzen')


# ---------------------------------------------------------------- ana
def main():
    dt = 0.01
    results = {}
    robots = {}
    for run in ('r1_gui100', 'r2_nogpu100'):
        path = os.path.join(BAGS, run)
        if not os.path.isdir(path):
            continue
        d = read_bag(path)
        rr = {n: load_robot(d, f'/p05/{n}/debug', f'/p05/{n}/odometry', f'/p05/{n}/thruster_state')
              for n in ('cyl_def', 'cyl_ovr', 'torp_sub')}
        rr['torp_flt'] = load_robot(d, '/p05/torp_flt/debug', '/p05/torp_flt/odometry')
        res = {n: item1(rr[n], dt) for n in ('cyl_def', 'cyl_ovr', 'torp_sub')}
        res['coef'] = {n: dict(cd=rr[n]['cd'].tolist(), cf=rr[n]['cf'].tolist()) for n in rr}
        res['timebase'] = timebase(rr['cyl_def'], dt, 0.02)
        res['mesh_sub'], res['mesh_flt'] = item5(rr['torp_sub'], rr['torp_flt'], dt)
        results[run] = res
        if run == 'r1_gui100':
            robots = rr
            fig_loglog(res, rr)
            fig_timebase(res['timebase'])
            fig_float(rr['torp_flt'], res['mesh_flt'], dt)
    p3 = os.path.join(BAGS, 'r3_gui500')
    if os.path.isdir(p3):
        d = read_bag(p3)
        r3 = load_robot(d, '/probe/debug/physics', '/probe/odometry', '/probe/thruster_state')
        results['r3_gui500'] = dict(timebase=timebase(r3, 0.002, 0.02))

    def clean(o):
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items() if not k.startswith('_')}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        return o
    with open(os.path.join(BAGS, 'phase0b_results.json'), 'w') as f:
        json.dump(clean(results), f, indent=1)
    print(json.dumps(clean(results), indent=1))


if __name__ == '__main__':
    main()

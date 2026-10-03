#!/usr/bin/env python3
"""Faz 1: config/vehicle.yaml -> mesh, turetilmis parametreler, tasarim tablolari, grafikler.

Kullanim: python3 scripts/design_vehicle.py [config/vehicle.yaml]
Ciktilar: meshes/hull.obj|json, config/vehicle_derived.yaml, docs/01_tasarim_tablolar.md,
          figures/faz1/*.png|pdf
"""
import json
import os
import sys
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, ROOT)
from hybrid_vehicle_sim import hull, hydrostatics as hs, mass as ms, drag, thrusters as th  # noqa: E402

FIG = os.path.join(ROOT, 'figures', 'faz1')
C1, C2, C3 = '#2a78d6', '#eb6834', '#1baf7a'          # dataviz referans paleti, ilk 3 slot
INK, INK2, GRID, SURF, HULL = '#0b0b0b', '#52514e', '#e4e3df', '#ffffff', '#dcdad3'


def f(x, n=4):
    return f'{x:.{n}f}'


# ------------------------------------------------------------------ hesap
def compute(cfg):
    fl, mc, hc = cfg['fluid'], cfg['mass'], cfg['hydrodynamics']
    rho, nu, g = fl['rho'], fl['nu'], fl['g']
    p = hull.HullParams.from_dict(cfg['hull'])
    V, F = hull.build_mesh(p)
    tight, euler = hull.check_watertight(F, len(V))
    if not (tight and euler == 2):
        raise RuntimeError('mesh kapali/tutarli yonlu degil')
    mp, ap = hull.mesh_props(V, F), hull.analytic_props(p)
    rx, rr = hull.ring_profile(p)
    sl = hs.Slicer(rx, rr, p.n_theta)
    v_full, cb_full = sl.full()

    m = rho * mp['volume'] / (1 + mc['reserve_buoyancy'])
    mm = ms.vehicle_mass(V, F, m, mc['ballast_mass'], mc['bg'], p.r)
    cg = np.array(mm['cg'])

    # yuzey dengesi ve stabilite
    surf = hs.surface_state(sl, m, rho, cg, p.r)
    xw = np.linspace(rx[0], rx[-1], 6001)
    wp = hs.waterplane(xw, np.interp(xw, rx, rr), surf['axis_depth'])
    kb = p.r - surf['cb_sub'][2]
    kg = p.r - cg[2]
    gm_formula = dict(T=kb + wp['I_T'] / surf['v_sub'] - kg, L=kb + wp['I_L'] / surf['v_sub'] - kg)
    da = np.radians(0.05)                                     # merkezi fark: GZ(0)'daki trim momenti duser
    gm_num = {ax: (hs.righting_arm(sl, m, rho, g, cg, da, ax, True)
                   - hs.righting_arm(sl, m, rho, g, cg, -da, ax, True)) / (2 * np.sin(da)) for ax in ('roll', 'pitch')}
    gm_sub = hs.righting_arm(sl, m, rho, g, cg, da, 'roll', False) / np.sin(da)
    ang_r = np.radians(np.arange(0, 61, 5))
    ang_p = np.radians(np.arange(-20, 21, 2.5))
    gz = dict(roll_ang=ang_r, pitch_ang=ang_p,
              roll_surf=np.array([hs.righting_arm(sl, m, rho, g, cg, a, 'roll', True) for a in ang_r]),
              roll_sub=np.array([hs.righting_arm(sl, m, rho, g, cg, a, 'roll', False) for a in ang_r]),
              pitch_surf=np.array([hs.righting_arm(sl, m, rho, g, cg, a, 'pitch', True) for a in ang_p]),
              pitch_sub=np.array([hs.righting_arm(sl, m, rho, g, cg, a, 'pitch', False) for a in ang_p]))

    # direnc ve kalibrasyon
    k = drag.hoerner_form_factor(p.diameter, p.length)
    surge_phys = lambda u: drag.physical_surge(u, mp['surface'], p.length, k, rho, nu)[0]
    cal_x = drag.calibrate(hc['surge_band'], surge_phys, mp['a_front'], mp['s_t'], rho)
    heave_phys = lambda w: drag.cross_flow(w, hc['cross_flow_cd'], mp['a_plan'], rho)
    cal_z = drag.calibrate(hc['heave_band'], heave_phys, mp['a_plan'], mp['s_t_z'], rho)
    cd = [cal_x['cd'], cal_z['cd'], cal_z['cd']]
    cf = [cal_x['cf'], cal_z['cf'], cal_z['cf']]
    semi = np.array([p.length / 2, p.r, p.r])                 # MVAE tahmini (Faz 2'de olculecek)
    cd_def = (1 / semi) / (1 / semi).max()
    cf_def = 0.1 * cd_def

    # ek kutle
    k1, k2 = drag.lamb_prolate(p.length, p.diameter)
    ma_sf = drag.stonefish_added_mass(*semi, rho)

    # thrusterlar
    units = th.resolve(cfg['thrusters']['units'], mm['cb'])
    B = th.allocation(units, cg)
    t_max = cfg['thrusters']['max_thrust']
    for u in units:
        R = np.eye(3)
        u['depth_at_surface'] = surf['axis_depth'] + (R @ u['position'])[2]

    return dict(p=p, V=V, F=F, tight=tight, euler=euler, mp=mp, ap=ap, rx=rx, rr=rr, v_full=v_full,
                cb_full=cb_full, rho=rho, nu=nu, g=g, m=m, mm=mm, cg=cg, surf=surf, wp=wp, kb=kb, kg=kg,
                gm_formula=gm_formula, gm_num=gm_num, gm_sub=gm_sub, gz=gz, k=k, cal_x=cal_x, cal_z=cal_z,
                cd=cd, cf=cf, cd_def=cd_def, cf_def=cf_def, k1=k1, k2=k2, ma_sf=ma_sf, semi=semi,
                units=units, B=B, t_max=t_max, cfg=cfg)


# ------------------------------------------------------------------ ciktilar
def write_outputs(r):
    p, mp, cfg = r['p'], r['mp'], r['cfg']
    os.makedirs(os.path.join(ROOT, 'meshes'), exist_ok=True)
    hull.write_obj(os.path.join(ROOT, 'meshes', 'hull.obj'), r['V'], r['F'],
                   f"NAUTILUS govde L={p.length} D={p.diameter} a_n={p.nose_length} L_mid={p.mid_length} a_t={p.tail_length}")
    with open(os.path.join(ROOT, 'meshes', 'hull.json'), 'w') as fh:
        json.dump(dict(params=p.__dict__, n_vertices=len(r['V']), n_faces=len(r['F']), watertight=r['tight'],
                       euler=r['euler'], mesh=mp, analytic=r['ap'],
                       rings_x=r['rx'].tolist(), rings_r=r['rr'].tolist()), fh, indent=1)
    I = r['mm']['inertia']
    derived = dict(
        generated_by='scripts/design_vehicle.py', source='config/vehicle.yaml', name=cfg['name'],
        fluid=cfg['fluid'],
        mesh=dict(path='meshes/hull.obj', volume=mp['volume'], surface=mp['surface'], a_front=mp['a_front'],
                  s_t=mp['s_t'], a_plan=mp['a_plan'], s_t_z=mp['s_t_z'], cb=mp['cb']),
        mass=dict(mass=r['m'], cg=r['mm']['cg'], inertia=[I[0, 0], I[1, 1], I[2, 2]],
                  ballast_mass=r['mm']['ballast_mass'], ballast_pos=r['mm']['ballast_pos']),
        hydrodynamics=dict(quadratic_drag=r['cd'], viscous_drag=r['cf'],
                           surge_fit={k: r['cal_x'][k] for k in ('a', 'b', 'band', 'max_rel_err')},
                           heave_fit={k: r['cal_z'][k] for k in ('a', 'b', 'band', 'max_rel_err')}),
        thrusters=[dict(name=u['name'], origin_xyz=u['position'].tolist(), origin_rpy=u['rpy'],
                        max_thrust=r['t_max']) for u in r['units']],
        vbs=cfg['vbs'] | dict(x=r['mm']['cb'][0] + cfg['vbs']['x_relative_to_cb']),
        surface=dict(axis_depth=r['surf']['axis_depth'], draft=r['surf']['draft'],
                     freeboard=r['surf']['freeboard'], trim_deg=float(np.degrees(r['surf']['trim_rad']))))

    def plain(o):
        if isinstance(o, dict):
            return {k: plain(v) for k, v in o.items()}
        if isinstance(o, (list, tuple, np.ndarray)):
            return [plain(v) for v in o]
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        return o
    with open(os.path.join(ROOT, 'config', 'vehicle_derived.yaml'), 'w') as fh:
        fh.write('# OTOMATIK URETILDI - elle duzenlemeyin. Kaynak: config/vehicle.yaml\n')
        yaml.safe_dump(plain(derived), fh, sort_keys=False, allow_unicode=True)


def tables(r):
    p, mp, ap, mm, rho, g = r['p'], r['mp'], r['ap'], r['mm'], r['rho'], r['g']
    m, W, Bf = r['m'], r['m'] * g, rho * g * mp['volume']
    I = mm['inertia']
    s, wp = r['surf'], r['wp']
    L = []
    L += ['## T1. Geometri', '', '| Büyüklük | Analitik | Mesh-tam | Fark |', '|---|---|---|---|']
    L += [f'| Boy L / çap D / L/D | {p.length:.3f} m / {p.diameter:.3f} m / {p.length/p.diameter:.2f} | | |',
          f'| Burun / silindir / kıç | {p.nose_length} / {p.mid_length} / {p.tail_length} m | | |']
    for key, name, unit, sc in (('volume', 'Hacim V', 'L', 1e3), ('surface', 'Islak alan S', 'm²', 1),
                                ('a_front', 'Ön kesit A_ön', 'cm²', 1e4), ('a_plan', 'Plan alanı A_plan', 'm²', 1)):
        L.append(f'| {name} [{unit}] | {ap[key]*sc:.4f} | {mp[key]*sc:.4f} | {100*(mp[key]/ap[key]-1):+.3f}% |')
    L += [f'| S_t (surge teğetsel, ΣA(1−n_x²)) [m²] | | {mp["s_t"]:.4f} | |',
          f'| S_t,z (heave teğetsel) [m²] | | {mp["s_t_z"]:.4f} | |',
          f'| x_CB [cm] | {100*ap["cb_x"]:.3f} | {100*mp["cb"][0]:.3f} | |',
          f'| Mesh | {len(r["V"])} köşe, {len(r["F"])} üçgen, watertight={r["tight"]}, Euler={r["euler"]} | | |', '']
    L += ['## T2. Kütle ve yüzdürme', '', '| Büyüklük | Değer |', '|---|---|',
          f'| Kütle m | {m:.3f} kg |', f'| Ağırlık W | {W:.2f} N |',
          f'| Kaldırma (tam dalmış) B | {Bf:.2f} N |',
          f'| Net yüzdürme B−W | {Bf-W:+.3f} N ({100*(Bf-W)/W:.2f}% W) |',
          f'| Gövde (homojen) / balast | {mm["hull_mass"]:.3f} kg / {mm["ballast_mass"]:.3f} kg |',
          f'| Balast konumu (x, z) | ({100*mm["ballast_pos"][0]:.2f}, {100*mm["ballast_pos"][2]:.2f}) cm, gövde içinde: {mm["ballast_inside"]} |',
          f'| CB (gövde) | ({100*mm["cb"][0]:.3f}, 0, 0) cm |',
          f'| CG (gövde) | ({100*mm["cg"][0]:.3f}, 0, {100*mm["cg"][2]:.3f}) cm  → BG = {100*(mm["cg"][2]-mm["cb"][2]):.2f} cm |',
          f'| Atalet (CG, asal) Ixx, Iyy, Izz | {I[0,0]:.4f}, {I[1,1]:.4f}, {I[2,2]:.4f} kg·m² (köşegen: {mm["principal"]}) |', '']
    L += ['## T3. Yüzey dengesi ve stabilite', '', '| Büyüklük | Değer |', '|---|---|',
          f'| Eksen derinliği (dik, NED) | {100*s["axis_depth"]:.3f} cm |',
          f'| Su çekimi T | {100*s["draft"]:.3f} cm |', f'| Fribord (silindir üstü) | {1000*s["freeboard"]:.2f} mm |',
          f'| Su altı hacim oranı | {100*s["v_sub"]/mp["volume"]:.2f}% |',
          f'| Statik trim (yüzey) | {np.degrees(s["trim_rad"]):+.3f}° (+ = burun yukarı) |',
          f'| Su hattı L_wl / B_wl / A_wp | {wp["length"]:.3f} m / {100*wp["beam"]:.2f} cm / {1e4*wp["area"]:.1f} cm² |',
          f'| KB / KG | {100*r["kb"]:.3f} cm / {100*r["kg"]:.3f} cm |',
          f'| BM_T / BM_L | {100*wp["I_T"]/s["v_sub"]:.3f} cm / {100*wp["I_L"]/s["v_sub"]:.2f} cm |',
          f'| GM_T yüzey: formül / sayısal (GZ eğimi, ±0.05°) | {100*r["gm_formula"]["T"]:.3f} / {100*r["gm_num"]["roll"]:.3f} cm |',
          f'| GM_L yüzey: formül / sayısal (±0.05°) | {100*r["gm_formula"]["L"]:.2f} / {100*r["gm_num"]["pitch"]:.2f} cm |',
          f'| GM su altı (roll = pitch), sayısal | {100*r["gm_sub"]:.3f} cm (BG×B/W = {100*mc_bg(r)*Bf/W:.3f}) |',
          f'| Roll periyodu (su altı, ek atalet yok) | {2*np.pi*np.sqrt(I[0,0]/(Bf*mc_bg(r))):.2f} s |',
          f'| Pitch periyodu (su altı, ek atalet yok) | {2*np.pi*np.sqrt(I[1,1]/(Bf*mc_bg(r))):.2f} s |', '']
    L += ['## T4. Thruster yerleşimi', '',
          '| Ad | Konum xyz [m] | Yön | rpy [rad] | Yüzey dengesinde derinlik |', '|---|---|---|---|---|']
    for u in r['units']:
        L.append(f'| {u["name"]} | ({u["position"][0]:+.4f}, {u["position"][1]:+.3f}, {u["position"][2]:+.3f}) | '
                 f'({u["direction"][0]:.0f}, {u["direction"][1]:.0f}, {u["direction"][2]:.0f}) | '
                 f'({u["rpy"][0]:.4f}, {u["rpy"][1]:.4f}, {u["rpy"][2]:.4f}) | {100*u["depth_at_surface"]:.2f} cm |')
    B = r['B']
    cap = th.axis_capacity(B, r['t_max'])
    L += ['', f'Allocation matrisi B (τ = B·T, CG etrafında; T_max = ±{r["t_max"]} N, rank = {np.linalg.matrix_rank(B)}):', '',
          '| Eksen | ' + ' | '.join(u['name'] for u in r['units']) + ' | Kapasite |',
          '|---|' + '---|' * (len(r['units']) + 1)]
    unit = ['N', 'N', 'N', 'N·m', 'N·m', 'N·m']
    for i, ax in enumerate(th.AXES):
        L.append(f'| {ax} | ' + ' | '.join(f'{v:+.4f}' for v in B[i]) + f' | {cap[i]:.2f} {unit[i]} |')
    L += ['']
    # direnc
    rho, nu, k = r['rho'], r['nu'], r['k']
    L += ['## T5. Surge direnci', '', f'Form faktörü (Hoerner): 1+k = {1+k:.4f}. '
          f'Stonefish kalibre: C_d,x = {r["cd"][0]:.5f}, C_f,x = {r["cf"][0]:.6f} m/s '
          f'(bant {r["cal_x"]["band"]} m/s, max göreli hata {100*r["cal_x"]["max_rel_err"]:.1f}%). '
          f'Stonefish varsayılan (tahmini MVAE): C_d,x = {r["cd_def"][0]:.4f}, C_f,x = {r["cf_def"][0]:.5f}.', '',
          '| u [m/s] | Re | C_F | R_sürt. [N] | R_bas. [N] | R_top. [N] | SF kalibre F_f / F_p [N] | SF kalibre top. [N] | Hata | SF varsayılan top. [N] |',
          '|---|---|---|---|---|---|---|---|---|---|']
    for u in (0.3, 0.5, 1.0, 1.5, 2.0):
        tot, fr, pr = drag.physical_surge(u, mp['surface'], r['p'].length, k, rho, nu)
        sp, sf = drag.stonefish_force(u, r['cd'][0], r['cf'][0], mp['a_front'], mp['s_t'], rho)
        dp, df = drag.stonefish_force(u, r['cd_def'][0], r['cf_def'][0], mp['a_front'], mp['s_t'], rho)
        re = u * r['p'].length / nu
        L.append(f'| {u} | {re:.3g} | {drag.ittc57_cf(re):.5f} | {fr:.4f} | {pr:.4f} | {tot:.4f} | '
                 f'{sf:.4f} / {sp:.4f} | {sf+sp:.4f} | {100*((sf+sp)/tot-1):+.1f}% | {dp+df:.2f} |')
    L += ['', f'Heave (y = z): çapraz akış C_D,c = {r["cfg"]["hydrodynamics"]["cross_flow_cd"]} (varsayım), '
          f'A_plan = {mp["a_plan"]:.4f} m². Kalibre: C_d,z = {r["cd"][2]:.4f}, C_f,z = {r["cf"][2]:.5f} m/s '
          f'(bant {r["cal_z"]["band"]} m/s, max göreli hata {100*r["cal_z"]["max_rel_err"]:.1f}%).', '']
    # tam itkide hiz
    T = th.axis_capacity(B, r['t_max'])[0]
    from scipy import optimize
    u_phys = optimize.brentq(lambda u: drag.physical_surge(u, mp['surface'], r['p'].length, k, rho, nu)[0] - T, 0.01, 30)
    u_sf = optimize.brentq(lambda u: sum(drag.stonefish_force(u, r['cd'][0], r['cf'][0], mp['a_front'], mp['s_t'], rho)) - T, 0.01, 30)
    w_need = (Bf - W) + drag.cross_flow(0.3, r['cfg']['hydrodynamics']['cross_flow_cd'], mp['a_plan'], rho)
    L += [f'Tam surge itkisi {T:.0f} N ile terminal hız: fiziksel (çıplak gövde) {u_phys:.2f} m/s, '
          f'Stonefish kalibre {u_sf:.2f} m/s (bant dışı). Yüzeyden dalış + w = 0.3 m/s için gereken dikey itki '
          f'≈ {w_need:.1f} N (kapasite {th.axis_capacity(B, r["t_max"])[2]:.0f} N).', '']
    # ek kutle
    ma_sf = r['ma_sf']
    Vv = mp['volume']
    L += ['## T6. Ek kütle', '', '| | Eksenel (x) | Yanal (y, z) | Surge yönünde etkin atalet |', '|---|---|---|---|',
          f'| Fiziksel (prolate, L/D={r["p"].length/r["p"].diameter:.0f}, Lamb) | k₁ = {r["k1"]:.4f} → {r["k1"]*rho*Vv:.3f} kg | '
          f'k₂ = {r["k2"]:.4f} → {r["k2"]*rho*Vv:.3f} kg | m + m_a,x = {m + r["k1"]*rho*Vv:.2f} kg |',
          f'| Stonefish (tahmini MVAE yarı-eksenleri {r["semi"].round(4).tolist()}) | {ma_sf[0]:.3f} kg | '
          f'{ma_sf[1]:.3f} kg | m + ort(m_a) = {m + ma_sf.mean():.2f} kg |', '']
    vb = r['cfg']['vbs']
    L += ['## T7. VBS (opsiyonel, şu an kapalı)', '',
          f'Max {1e3*vb["max_volume"]:.2f} L → net yüzdürme {Bf-W:+.2f} N ile '
          f'{Bf-W-rho*g*vb["max_volume"]:+.2f} N arası (Stonefish VBS sadece ağırlık ekler, ataleti değiştirmez).', '']
    # capraz kontroller
    L += ['## T8. Çapraz kontroller', '', '| Kontrol | Sonuç |', '|---|---|',
          f'| Mesh kapalı + tutarlı yönlü, Euler = 2 | {r["tight"]}, {r["euler"]} |',
          f'| Dilimleme (tam dalmış) V / mesh V − 1 | {r["v_full"]/mp["volume"]-1:+.2e} |',
          f'| Dilimleme CB_x − mesh CB_x | {1e3*(r["cb_full"][0]-mp["cb"][0]):+.4f} mm |',
          f'| GM_T yüzey (sayısal) / BG (dairesel kesit → metasantr eksende) | {r["gm_num"]["roll"]/mc_bg(r):.4f} |',
          f'| GM_T formül / sayısal | {r["gm_formula"]["T"]/r["gm_num"]["roll"]:.4f} |',
          f'| Balast gövde içinde | {mm["ballast_inside"]} |',
          f'| Atalet tensörü köşegen | {mm["principal"]} |',
          f'| Allocation rank (X, Z, M, N) | {np.linalg.matrix_rank(B)} |',
          f'| Tüm thrusterlar yüzey dengesinde su altında | {all(u["depth_at_surface"] > 0 for u in r["units"])} |', '']
    return '\n'.join(L)


def mc_bg(r):
    return r['mm']['cg'][2] - r['mm']['cb'][2]


# ------------------------------------------------------------------ grafikler
def style(ax):
    ax.set_facecolor(SURF)
    ax.grid(True, color=GRID, lw=0.6)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)


def save(fig, name):
    os.makedirs(FIG, exist_ok=True)
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(FIG, f'{name}.{ext}'), dpi=200, bbox_inches='tight', facecolor=SURF)
    plt.close(fig)


def fig_layout(r):
    p = r['p']
    x = np.linspace(-p.length / 2, p.length / 2, 801)
    rr = hull.radius(p, x)
    fig, axes = plt.subplots(2, 1, figsize=(10, 4.6), sharex=True, gridspec_kw=dict(hspace=0.35))
    for ax, (j, lab, title) in zip(axes, ((2, 'z [m] (aşağı +)', 'Yandan görünüş (x–z)'),
                                         (1, 'y [m] (sancak +)', 'Üstten görünüş (x–y)'))):
        style(ax)
        ax.fill_between(x, -rr, rr, color=HULL, ec=INK2, lw=1.0, zorder=1)
        hz = [u for u in r['units'] if abs(u['direction'][2]) < 0.5]
        vt = [u for u in r['units'] if abs(u['direction'][2]) >= 0.5]
        for grp, col, lbl in ((hz, C1, 'Yatay thruster (surge/yaw)'), (vt, C2, 'Dikey thruster (heave/pitch)')):
            px = [u['position'][0] for u in grp]
            py = [u['position'][j] for u in grp]
            ax.quiver(px, py, [0.09 * u['direction'][0] for u in grp], [0.09 * u['direction'][j] for u in grp],
                      angles='xy', scale_units='xy', scale=1, color=col, width=0.004, zorder=4)
            ax.plot(px, py, 'o', ms=8, color=col, mec=SURF, mew=1.2, zorder=5, label=lbl)
        cb, cg, bl = r['mm']['cb'], r['mm']['cg'], r['mm']['ballast_pos']
        ax.plot(bl[0], bl[j], 's', ms=8, color=C3, mec=SURF, mew=1.2, zorder=5, label='Balast')
        ax.plot(cb[0], cb[j], 'o', ms=9, mfc=SURF, mec=INK, mew=1.5, zorder=6, label='CB')
        ax.plot(cg[0], cg[j], 'P', ms=9, color=INK, mec=SURF, zorder=6, label='CG')
        if j == 2:
            ax.axhline(-r['surf']['axis_depth'], color=INK2, ls='--', lw=1.0, label='Su hattı (yüzey dengesi)')
            ax.set_ylim(0.12, -0.12)
        else:
            ax.set_ylim(-0.12, 0.12)
        ax.set_aspect('equal')
        ax.set_ylabel(lab, color=INK)
        ax.set_title(title, color=INK, fontsize=10, loc='left')
    axes[1].set_xlabel('x [m] (burun +)', color=INK)
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, fontsize=8, frameon=False, loc='lower center', ncol=6, labelcolor=INK, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle(f'NAUTILUS: L = {p.length} m, D = {p.diameter} m, m = {r["m"]:.2f} kg', color=INK, fontsize=11, x=0.01, ha='left')
    fig.subplots_adjust(bottom=0.2)
    save(fig, '01_govde_yerlesim')


def fig_gz(r):
    gz = r['gz']
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    panels = ((axes[0], 'roll_ang', [('roll_surf', C1, 'Yüzey'), ('roll_sub', C2, 'Su altı')], 'Roll (yalpa)'),
              (axes[1], 'pitch_ang', [('pitch_surf', C1, 'Yüzey')], 'Pitch (baş-kıç), yüzey'),
              (axes[2], 'pitch_ang', [('pitch_sub', C2, 'Su altı')], 'Pitch (baş-kıç), su altı'))
    for ax, a_key, series, title in panels:
        style(ax)
        for key, col, lbl in series:
            ls = '--' if key == 'roll_sub' else '-'
            ax.plot(np.degrees(gz[a_key]), 1e3 * gz[key], ls, color=col, lw=2, marker='o', ms=4, label=lbl)
        ax.axhline(0, color=INK2, lw=0.8)
        ax.set_xlabel('Açı [°]', color=INK)
        ax.set_ylabel('GZ [mm] (+ = doğrultucu)', color=INK)
        ax.set_title(title, color=INK, fontsize=10, loc='left')
        if len(series) > 1:
            ax.legend(fontsize=8, frameon=False, labelcolor=INK)
    fig.tight_layout()
    save(fig, '01_gz_egrileri')


def fig_drag(r):
    mp, rho, nu, k, p = r['mp'], r['rho'], r['nu'], r['k'], r['p']
    u = np.linspace(0.05, 2.0, 400)
    tot, fr, _ = drag.physical_surge(u, mp['surface'], p.length, k, rho, nu)
    sp, sf = drag.stonefish_force(u, r['cd'][0], r['cf'][0], mp['a_front'], mp['s_t'], rho)
    dp, df = drag.stonefish_force(u, r['cd_def'][0], r['cf_def'][0], mp['a_front'], mp['s_t'], rho)
    band = r['cal_x']['band']
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    for ax in axes:
        style(ax)
        ax.axvspan(*band, color=GRID, alpha=0.6, lw=0)
        ax.set_xlabel('Hız u [m/s]', color=INK)
    ax = axes[0]
    ax.semilogy(u, tot, color=C1, lw=2, label='Fiziksel: ITTC-57 × (1+k)')
    ax.semilogy(u, sp + sf, color=C2, lw=2, label='Stonefish, kalibre')
    ax.semilogy(u, dp + df, color=C3, lw=2, label='Stonefish, varsayılan (tahmini)')
    ax.set_ylabel('Toplam direnç [N]', color=INK)
    ax.set_title('Surge direnci', color=INK, fontsize=10, loc='left')
    ax.legend(fontsize=8, frameon=False, labelcolor=INK)
    ax = axes[1]
    ax.plot(u, 100 * fr / tot, color=C1, lw=2, label='Fiziksel')
    ax.plot(u, 100 * sf / (sp + sf), color=C2, lw=2, label='Stonefish, kalibre')
    ax.set_ylim(0, 105)
    ax.set_ylabel('Sürtünmenin toplamdaki payı [%]', color=INK)
    ax.set_title('Sürtünme / basınç ayrımı', color=INK, fontsize=10, loc='left')
    ax.legend(fontsize=8, frameon=False, labelcolor=INK)
    ax = axes[2]
    ax.plot(u, 100 * ((sp + sf) / tot - 1), color=C2, lw=2)
    ax.axhline(0, color=INK2, lw=0.8)
    ax.set_ylim(-40, 60)
    ax.set_ylabel('Stonefish kalibre / fiziksel − 1 [%]', color=INK)
    ax.set_title(f'Kalibrasyon hatası (bant içi max {100*r["cal_x"]["max_rel_err"]:.1f}%)', color=INK, fontsize=10, loc='left')
    fig.suptitle('Gri bant: kalibrasyon aralığı', color=INK2, fontsize=9, x=0.01, ha='left')
    fig.tight_layout()
    save(fig, '01_direnc_kalibrasyon')


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'config', 'vehicle.yaml')
    cfg = yaml.safe_load(open(path))
    r = compute(cfg)
    write_outputs(r)
    md = tables(r)
    with open(os.path.join(ROOT, 'docs', '01_tasarim_tablolar.md'), 'w') as fh:
        fh.write('# Faz 1 tasarım tabloları\n\n> OTOMATİK ÜRETİLDİ: `python3 scripts/design_vehicle.py`. '
                 'Girdi: `config/vehicle.yaml`.\n\n' + md + '\n')
    fig_layout(r)
    fig_gz(r)
    fig_drag(r)
    print(md)


if __name__ == '__main__':
    main()

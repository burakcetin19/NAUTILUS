#!/usr/bin/env python3
"""Faz 0 probe analizi: DebugPhysics kuvvetlerini kaynak koddan cikarilan formullerle kiyaslar.

Kullanim: python3 analyze_probe.py <bag_dizini>
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RHO, G = 1000.0, 9.81
R, L, SLICES = 0.1, 1.0, 32  # probe silindiri; slices = max(ceil(2*pi*r/0.1), 32)


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out = {k: [] for k in types}
    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        out[topic].append((t_rec * 1e-9, deserialize_message(data, types[topic])))
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main(path):
    d = read_bag(path)

    # Mesh geometrisi (OpenGLContent::BuildCylinder: alan koruyan duzeltilmis yaricap)
    dphi = 2 * np.pi / SLICES
    r_mesh = R * np.sqrt(dphi / np.sin(dphi))
    a_front = 0.5 * SLICES * r_mesh**2 * np.sin(dphi)          # = pi R^2
    s_side = SLICES * 2 * r_mesh * np.sin(dphi / 2) * L         # yan yuzey (v_t = u)
    vol = np.pi * R**2 * L
    m = RHO * vol
    m_a = RHO * np.pi * r_mesh**2 * L                           # silindir yaklasimi; m1 = m2 sayisal olarak esit (L=1)
    m_eff = m + m_a                                             # m + (m1+m2+m2)/3

    dbg = d['/probe/debug/physics']
    t = np.array([stamp(x) for _, x in dbg])
    t_rec = np.array([tr for tr, _ in dbg])
    u = np.array([x.velocity.linear.x for _, x in dbg])
    fq = np.array([x.damping.force.x for _, x in dbg])
    ff = np.array([x.skin_friction.force.x for _, x in dbg])
    fb = np.array([x.buoyancy.force.z for _, x in dbg])
    cd = dbg[0][1].damping_coeff.x
    cf = dbg[0][1].skin_friction_coeff.x

    ths = d['/probe/thruster_state']
    t_th = np.array([stamp(x) for _, x in ths])
    thrust = np.interp(t, t_th, np.array([x.thrust[0] for _, x in ths]))

    print(f"Geometri: r_mesh={r_mesh:.6f} A_on={a_front:.6f} S_yan={s_side:.6f} V={vol:.6f}")
    print(f"Cd_x={cd:.6f} Cf_x={cf:.6f}")
    print(f"(a) Kaldirma: olculen {fb[0]:.3f} N, rho*g*V = {RHO*G*vol:.3f} N")

    print("(b) Kararli hal (adimin son 3 s'si):")
    print("   T[N]   u[m/s]  Fq_sim  Fq_formul  Ff_sim  Ff_formul  T+Fq+Ff  u_tahmin")
    for T in (20.0, 50.0):
        idx = np.where(np.isclose(thrust, T, atol=1e-6))[0]
        if idx.size == 0:
            continue
        w = idx[t[idx] > t[idx[-1]] - 3.0]
        um, fqm, ffm = u[w].mean(), fq[w].mean(), ff[w].mean()
        # SolidEntity.cpp:1776-1779: vc_n normalize degil -> F_q = 1/2 rho Cd A |u|^3 (kubik)
        kq, kf = 0.5 * RHO * cd * a_front, RHO * cf * s_side
        roots = np.roots([kq, 0.0, kf, -T])
        u_pred = roots[np.isreal(roots)].real.max()
        print(f"   {T:5.1f}  {um:.4f}  {fqm:7.3f}  {-kq*um**2*abs(um):9.3f}  {ffm:7.3f}  {-kf*um:9.3f}"
              f"  {T+fqm+ffm:7.4f}  {u_pred:.4f}")
        print(f"          Fq/u^2 = {-fqm/um**2:.3f} (kuadratik olsaydi sabit {kq:.3f}),"
              f" Fq/u^3 = {-fqm/um**3:.3f}")

    # (c) Etkin atalet: F_net = M * du/dt (gecis bolgeleri, nominal dt = 0.01 s)
    fnet = thrust + fq + ff
    for label, tt in (("damga", t), ("nominal", np.arange(len(t)) * 0.01)):
        a = np.gradient(u, tt)
        sel = np.abs(fnet) > 1.0
        M = np.sum(fnet[sel] * a[sel]) / np.sum(a[sel] ** 2)
        print(f"(c) Etkin atalet ({label} zaman): M = {M:.3f} kg  | tahmin m+mean(m_a) = {m_eff:.3f} kg,"
              f" m = {m:.3f} kg")

    # (d) Zaman damgasi jitter'i
    for name, tt, nominal in (("debug", t, 0.01), ("debug(kayit)", t_rec, 0.01),
                              ("odometry", np.array([stamp(x) for _, x in d['/probe/odometry']]), 0.02)):
        dt = np.diff(tt)
        print(f"(d) {name:13s} dt: ort={dt.mean()*1e3:.2f} ms std={dt.std()*1e3:.2f} ms "
              f"min={dt.min()*1e3:.2f} max={dt.max()*1e3:.2f} (nominal {nominal*1e3:.0f} ms)")
    span = t[-1] - t[0]
    print(f"(d) debug mesaj sayisi {len(t)}, damga araligi {span:.2f} s -> {len(t)/span:.1f} Hz")

    prs = d['/probe/pressure']
    print(f"(e) Basinc: {prs[0][1].fluid_pressure:.1f} Pa, rho*g*5 m = {RHO*G*5.0:.1f} Pa")


if __name__ == '__main__':
    main(sys.argv[1])

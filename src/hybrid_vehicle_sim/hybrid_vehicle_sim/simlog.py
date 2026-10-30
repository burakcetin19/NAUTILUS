"""Bag okuma ve zaman tabani (Faz 0.5 kurallari).

- Header damgalari duvar saati ve burst halinde: turev/ivme icin kullanilmaz.
- Zaman ekseni = mesaj indeksi x dt (DebugPhysics her sim adiminda bir kez; sensorlerde 1/rate).
- Her kosuda sayim kontrolu (N ~ rate x kayit suresi + 1) ve dinamik dt kontrolu (2 x nominal yok).
read_bag/stamp: docs/probe/analyze_probe.py; estimate_mass: docs/probe/analyze_phase0b.py (probe betikleri
degismeden kaliyor).
"""
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation


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


def _v3(v):
    return [v.x, v.y, v.z]


def debug_arrays(msgs, dt):
    """DebugPhysics listesi -> diziler. Hiz ve kuvvetler govde ekseninde, kaldirma dunya ekseninde
    (Faz 0, 00_kesif.md §9)."""
    m0 = msgs[0][1]
    get = lambda f: np.array([f(m) for _, m in msgs])
    return dict(
        t=np.arange(len(msgs)) * dt, t_rec=np.array([tr for tr, _ in msgs]),
        lin=get(lambda m: _v3(m.velocity.linear)), ang=get(lambda m: _v3(m.velocity.angular)),
        fb=get(lambda m: _v3(m.buoyancy.force)), tb=get(lambda m: _v3(m.buoyancy.torque)),
        fp=get(lambda m: _v3(m.damping.force)), tp=get(lambda m: _v3(m.damping.torque)),
        ff=get(lambda m: _v3(m.skin_friction.force)), tf=get(lambda m: _v3(m.skin_friction.torque)),
        vsub=get(lambda m: m.submerged_volume), swet=get(lambda m: m.wetted_surface),
        mass=m0.mass, volume=m0.volume, surface=m0.surface, inertia=np.array(_v3(m0.inertia)),
        cog=np.array(_v3(m0.cog)), cd=np.array(_v3(m0.damping_coeff)), cf=np.array(_v3(m0.skin_friction_coeff)))


def odom_arrays(msgs, dt):
    """Odometri: konum NED, twist govde ekseninde; rpy = ZYX Euler (roll, pitch, yaw) [rad]."""
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                   m.pose.pose.orientation.w] for _, m in msgs])
    ypr = Rotation.from_quat(q).as_euler('ZYX')
    return dict(t=np.arange(len(msgs)) * dt, t_rec=np.array([tr for tr, _ in msgs]),
                pos=np.array([_v3(m.pose.pose.position) for _, m in msgs]), rpy=ypr[:, ::-1],
                lin=np.array([_v3(m.twist.twist.linear) for _, m in msgs]),
                ang=np.array([_v3(m.twist.twist.angular) for _, m in msgs]))


def count_check(t_rec, rate):
    """Kayit (alim) suresine gore beklenen mesaj sayisi; oran ~1 olmali (mesaj dusmesi yok)."""
    span = t_rec[-1] - t_rec[0]
    expected = span * rate + 1
    return dict(n=len(t_rec), expected=expected, ratio=len(t_rec) / expected)


def dynamic_dt(v, fnet, M, thr):
    """Fiziksel adim suresi: dv*M/F_net (|F_net| > thr). Mesaj dusseydi 2 x nominal gorunurdu.
    Kuvvet hizalamasi (F_i veya F_{i+1}) veriden secilir."""
    dv = np.diff(v)
    best = None
    for shift in (0, 1):
        f = fnet[shift:len(fnet) - 1 + shift]
        sel = np.abs(f) > thr
        d = dv[sel] * M / f[sel]
        if best is None or np.std(d) < np.std(best):
            best = d
    return best


def estimate_mass(u, fnet, dt, thr=1.0):
    """M: F_net = M*du/dt regresyonu; kuvvetin hangi mesajla hizalandigi veriden secilir.
    Donus: (M, kayma, artik std)."""
    best = None
    for shift in (0, 1):          # 0: F_i -> (u_{i+1}-u_i); 1: F_{i+1} -> (u_{i+1}-u_i)
        a = np.diff(u) / dt
        f = fnet[shift:len(fnet) - 1 + shift]
        sel = np.abs(f) > thr
        M = np.sum(f[sel] * a[sel]) / np.sum(a[sel] ** 2)
        res = np.std(f[sel] - M * a[sel])
        if best is None or res < best[2]:
            best = (M, shift, res)
    return best

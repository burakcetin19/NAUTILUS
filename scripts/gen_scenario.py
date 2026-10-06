#!/usr/bin/env python3
"""config/vehicle_derived.yaml -> Stonefish senaryosu (.scn).

Kullanilan etiketlerin dogrulama kaynagi (ScenarioParser.cpp / ROS2ScenarioParser.cpp):
  ocean/water/waves/particles :643-692 | robot, base_link, world_transform: ParseRobot
  model/physical/mesh/origin :2054-2106 | mass/inertia/cg :1942-1950, 2113-2122
  hydrodynamics :1953-1959, 2123 | simple_thruster/propeller/specs :3045-3105 | watchdog :2444-2447
  odometry/imu/pressure :3415, 3625, 3653 | ros_subscriber/ros_publisher/ros_debug: ROS2ScenarioParser :303-452
Mesh yollari data dizinine gore goreli (ScenarioParser.cpp:5131-5136, GetDataPath() + yol; kopru sona "/"
ekliyor, stonefish_simulator*.cpp:67): simulasyon simulation_data:=<paket koku> ile baslatilmali.
scripts/run_sim.sh ve scripts/run_case.sh bunu yapiyor.

Kullanim: python3 scripts/gen_scenario.py [--depth Z] [--rpy R P Y] [--default-hydro] [--neutral] [--out X.scn]
  --default-hydro  <hydrodynamics> yazilmaz: Stonefish'in MVAE tabanli varsayilan C_d/C_f'si (Faz 2 S2)
  --neutral        kutle = rho*V (B = W); CG ve atalet ayni. Sadece test varyanti (Faz 2 S5)
"""
import argparse
import math
import os
import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
THRUSTER_MESH = 'meshes/thruster.obj'          # gorsel pod (design_vehicle.py); pervane yalnizca gorsel


def v3(v):
    """10 anlamli basamak: S1'de override degerleri 1e-6 icinde birebir kontrol ediliyor."""
    return ' '.join(f'{(x if abs(x) > 1e-12 else 0.0):.10g}' for x in v)


def thruster_xml(t, ns):
    return f'''
		<actuator name="{t['name']}" type="simple_thruster">
			<link name="Hull"/>
			<origin xyz="{v3(t['origin_xyz'])}" rpy="{v3(t['origin_rpy'])}"/>
			<specs lower_thrust_limit="{-t['max_thrust']:g}" upper_thrust_limit="{t['max_thrust']:g}"/>
			<propeller right="true">
				<mesh filename="{THRUSTER_MESH}" scale="1.0"/>
				<material name="Neutral"/>
				<look name="propeller"/>
			</propeller>
			<watchdog timeout="1.0"/>
		</actuator>'''


def scenario(d, depth, rpy_deg=(0.0, 0.0, 0.0), default_hydro=False, neutral=False):
    ns = d['name']
    hd, ms = d['hydrodynamics'], d['mass']
    mesh = d['mesh']['path']
    mass = d['fluid']['rho'] * d['mesh']['volume'] if neutral else ms['mass']
    hydro = '' if default_hydro else (
        f'\n\t\t\t<hydrodynamics viscous_drag="{v3(hd["viscous_drag"])}" quadratic_drag="{v3(hd["quadratic_drag"])}"/>')
    rpy = [math.radians(a) for a in rpy_deg]
    thr = ''.join(thruster_xml(t, ns) for t in d['thrusters'])
    order = ', '.join(t['name'] for t in d['thrusters'])
    variant = ', '.join(v for v, on in (('varsayilan hidrodinamik', default_hydro), ('NOTR KUTLE (test)', neutral)) if on)
    return f'''<?xml version="1.0"?>
<!-- OTOMATIK URETILDI: scripts/gen_scenario.py <- config/vehicle_derived.yaml. Elle duzenlemeyin.
     Baslangic: eksen derinligi {depth:.4f} m, rpy {rpy_deg[0]:g} {rpy_deg[1]:g} {rpy_deg[2]:g} deg. Varyant: {variant or 'yok'}
     Thruster sirasi ({ns}/thrusters, Float64MultiArray, [N]): {order}
     Dikey thrusterlarda + itki = asagi (dalis). -->
<scenario>
	<environment>
		<ned latitude="41.0" longitude="29.0"/>
		<ocean>
			<water density="{d['fluid']['rho']:.1f}" jerlov="0.2"/>
			<waves height="0.0"/>
			<particles enabled="true"/>
		</ocean>
		<atmosphere>
			<sun azimuth="20.0" elevation="50.0"/>
		</atmosphere>
	</environment>

	<materials>
		<material name="Hull" density="{mass / d['mesh']['volume']:.10g}" restitution="0.5"/>
		<material name="Neutral" density="1000.0" restitution="0.5"/>
	</materials>

	<looks>
		<look name="hull" rgb="1.0 0.72 0.0" roughness="0.3" metalness="0.1"/>
		<look name="propeller" rgb="0.15 0.15 0.15" roughness="0.4" metalness="0.6"/>
	</looks>

	<robot name="{ns}" fixed="false" self_collisions="false">
		<base_link name="Hull" type="model" physics="submerged" buoyant="true">
			<physical>
				<mesh filename="{mesh}" scale="1.0"/>
				<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
			</physical>
			<material name="Hull"/>
			<look name="hull"/>
			<mass value="{mass:.10g}"/>
			<inertia xyz="{v3(ms['inertia'])}"/>
			<cg xyz="{v3(ms['cg'])}" rpy="0.0 0.0 0.0"/>{hydro}
		</base_link>
{thr}

		<sensor name="odom" type="odometry" rate="50.0">
			<link name="Hull"/>
			<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
			<ros_publisher topic="/{ns}/odometry"/>
		</sensor>
		<sensor name="imu" type="imu" rate="50.0">
			<link name="Hull"/>
			<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
			<ros_publisher topic="/{ns}/imu"/>
		</sensor>
		<sensor name="pressure" type="pressure" rate="10.0">
			<link name="Hull"/>
			<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
			<ros_publisher topic="/{ns}/pressure"/>
		</sensor>

		<world_transform xyz="0.0 0.0 {depth:.10g}" rpy="{v3(rpy)}"/>
		<ros_subscriber thrusters="/{ns}/thrusters"/>
		<ros_publisher thrusters="/{ns}/thruster_state"/>
		<ros_debug physics="/{ns}/debug/physics"/>
	</robot>
</scenario>
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--depth', type=float, default=None,
                    help='baslangic eksen derinligi [m] (NED, + asagi); varsayilan: hesaplanan yuzey dengesi')
    ap.add_argument('--rpy', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('R', 'P', 'Y'),
                    help='baslangic roll pitch yaw [deg] (NED govde; + pitch = burun yukari)')
    ap.add_argument('--default-hydro', action='store_true', help='<hydrodynamics> override yazma')
    ap.add_argument('--neutral', action='store_true', help='test varyanti: kutle = rho*V')
    ap.add_argument('--out', default=os.path.join(ROOT, 'scenarios', 'nautilus.scn'))
    a = ap.parse_args()
    d = yaml.safe_load(open(os.path.join(ROOT, 'config', 'vehicle_derived.yaml')))
    if a.depth is None:
        a.depth = d['surface']['axis_depth']
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write(scenario(d, a.depth, a.rpy, a.default_hydro, a.neutral))
    print(a.out)


if __name__ == '__main__':
    main()

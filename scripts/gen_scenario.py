#!/usr/bin/env python3
"""config/vehicle_derived.yaml -> Stonefish senaryosu (.scn).

Kullanilan etiketlerin dogrulama kaynagi (ScenarioParser.cpp / ROS2ScenarioParser.cpp):
  ocean/water/waves/particles :643-692 | robot, base_link, world_transform: ParseRobot
  model/physical/mesh/origin :2054-2106 | mass/inertia/cg :1942-1950, 2113-2122
  hydrodynamics :1953-1959, 2123 | simple_thruster/propeller/specs :3045-3105 | watchdog :2444-2447
  odometry/imu/pressure :3415, 3625, 3653 | ros_subscriber/ros_publisher/ros_debug: ROS2ScenarioParser :303-452
Kullanim: python3 scripts/gen_scenario.py [--depth Z] [--out scenarios/nautilus.scn]
"""
import argparse
import os
import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
PROPELLER = os.path.expanduser('~/stonefish/Tests/Data/propeller.obj')   # 0.18 m cap, olcek 0.4 -> ~7 cm


def v3(v, n=6):
    return ' '.join(f'{(x if abs(x) > 1e-12 else 0.0):.{n}f}' for x in v)


def thruster_xml(t, ns):
    return f'''
		<actuator name="{t['name']}" type="simple_thruster">
			<link name="Hull"/>
			<origin xyz="{v3(t['origin_xyz'])}" rpy="{v3(t['origin_rpy'])}"/>
			<specs lower_thrust_limit="{-t['max_thrust']:.3f}" upper_thrust_limit="{t['max_thrust']:.3f}"/>
			<propeller right="true">
				<mesh filename="{PROPELLER}" scale="0.4"/>
				<material name="Neutral"/>
				<look name="propeller"/>
			</propeller>
			<watchdog timeout="1.0"/>
		</actuator>'''


def scenario(d, depth):
    ns = d['name']
    hd, ms = d['hydrodynamics'], d['mass']
    mesh = os.path.join(ROOT, d['mesh']['path'])
    thr = ''.join(thruster_xml(t, ns) for t in d['thrusters'])
    order = ', '.join(t['name'] for t in d['thrusters'])
    return f'''<?xml version="1.0"?>
<!-- OTOMATIK URETILDI: scripts/gen_scenario.py <- config/vehicle_derived.yaml. Elle duzenlemeyin.
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
		<material name="Hull" density="{ms['mass'] / d['mesh']['volume']:.3f}" restitution="0.5"/>
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
			<mass value="{ms['mass']:.6f}"/>
			<inertia xyz="{v3(ms['inertia'])}"/>
			<cg xyz="{v3(ms['cg'])}" rpy="0.0 0.0 0.0"/>
			<hydrodynamics viscous_drag="{v3(hd['viscous_drag'], 8)}" quadratic_drag="{v3(hd['quadratic_drag'], 8)}"/>
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

		<world_transform xyz="0.0 0.0 {depth:.3f}" rpy="0.0 0.0 0.0"/>
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
    ap.add_argument('--out', default=os.path.join(ROOT, 'scenarios', 'nautilus.scn'))
    a = ap.parse_args()
    d = yaml.safe_load(open(os.path.join(ROOT, 'config', 'vehicle_derived.yaml')))
    if a.depth is None:
        a.depth = d['surface']['axis_depth']
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write(scenario(d, a.depth))
    print(a.out)


if __name__ == '__main__':
    main()

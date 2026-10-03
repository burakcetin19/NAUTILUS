#!/usr/bin/env python3
"""Sabit itki basamaklari yayinlar (Float64MultiArray, tek thruster'li robotlar).

Kullanim: python3 thrust_steps.py --levels 5 10 20 35 50 80 --hold 20 --tail 10 \
              --topics /p05/cyl_def/thrusters /p05/cyl_ovr/thrusters
"""
import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--levels', type=float, nargs='+', required=True)
    ap.add_argument('--hold', type=float, default=20.0, help='seviye basina sure [s]')
    ap.add_argument('--tail', type=float, default=10.0, help='sonda 0 N suresi [s]')
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--topics', nargs='+', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = Node('thrust_steps')
    pubs = [node.create_publisher(Float64MultiArray, t, 10) for t in args.topics]
    schedule = [(T, args.hold) for T in args.levels] + [(0.0, args.tail)]
    time.sleep(1.0)  # abonelik eslesmesi icin
    for T, dur in schedule:
        node.get_logger().info(f'T = {T:.1f} N, {dur:.0f} s')
        t_end = time.monotonic() + dur
        while time.monotonic() < t_end:
            for p in pubs:
                p.publish(Float64MultiArray(data=[T]))
            time.sleep(1.0 / args.rate)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

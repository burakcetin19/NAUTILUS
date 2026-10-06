#!/usr/bin/env python3
"""Sabit itki vektoru yayinlar (Float64MultiArray, [N], senaryodaki thruster sirasi), SIGINT/SIGTERM'e kadar.

Neden: SimpleThruster kurucusu sThrust/sTorque'u baslatmiyor (SimpleThruster.cpp:37-48); ilk setpoint ya da
1 s'lik watchdog gelene kadar bellekteki cop deger itki ve tork olarak uygulaniyor (Faz 2'de 4.3e89 N goruldu).
run_case.sh bunu sim'den ONCE baslatir; sim abone olur olmaz setpoint 0 olur.

Kullanim: python3 scripts/thrust_cmd.py [--vector 0 0 0 0] [--rate 100] [--topic /nautilus/thrusters]
"""
import argparse
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vector', type=float, nargs='+', default=[0.0, 0.0, 0.0, 0.0])
    ap.add_argument('--rate', type=float, default=100.0)
    ap.add_argument('--topic', default='/nautilus/thrusters')
    a = ap.parse_args()
    rclpy.init()
    node = Node('thrust_cmd')
    pub = node.create_publisher(Float64MultiArray, a.topic, 10)
    msg = Float64MultiArray(data=a.vector)
    node.create_timer(1.0 / a.rate, lambda: pub.publish(msg))
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

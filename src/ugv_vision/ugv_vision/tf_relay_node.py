#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""로봇별 TF 를 전역 /tf 로 중계한다 — 시각화 전용.

왜 필요한가
-----------
2대 구성에서는 각 로봇이 자기 이름공간의 tf 로 발행한다(/ugv1/tf).
Nav2 와 SLAM 은 같은 이름공간에 있으니 문제가 없다. 그런데 RViz 는
전역 /tf 를 본다. 그래서 화면에 로봇 본체도, 경로선도 안 나온다.

프레임 이름은 이미 로봇별로 갈려 있으므로(ugv1/base_link) 한 트리에
합쳐도 안 겹친다. 그대로 옮겨 주기만 하면 된다.

계산은 하지 않는다 — 받은 걸 그대로 다시 낸다. 시각화용이라 부하가
문제 되면 이 노드만 끄면 된다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__('tf_relay_node')
        self.declare_parameter('robots', ['ugv1', 'ugv2'])
        robots = [r for r in self.get_parameter('robots').value if r]

        # tf_static 은 늦게 들어온 구독자도 받아야 하므로 transient_local.
        static_qos = QoSProfile(depth=100,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.pub_static = self.create_publisher(TFMessage, '/tf_static',
                                                static_qos)
        for r in robots:
            self.create_subscription(
                TFMessage, f'/{r}/tf',
                lambda m: self.pub.publish(m), 100)
            self.create_subscription(
                TFMessage, f'/{r}/tf_static',
                lambda m: self.pub_static.publish(m), static_qos)
        self.get_logger().info(
            f'TF 중계 시작 — {", ".join(robots)} → 전역 /tf (시각화용)')


def main():
    rclpy.init()
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

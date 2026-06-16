#!/usr/bin/env python3
"""
tf_filter.py
─────────────────────────────────────────────────────────────────────────────
Suscribe a /tf_unfiltered, elimina los transforms que gestiona el nodo de
ground truth y republica el resto a /tf.

Se usa junto con ground_truth_tf_rebroadcaster.py:
  - gz_bridge publica /model/robot_{i}/tf  →  /tf_unfiltered  (ver launch file)
  - este nodo filtra y reenvía a /tf
  - ground_truth_tf_rebroadcaster publica odom→footprint directamente a /tf

Uso directo:
    python3 tf_filter.py <num_robots>

Uso desde launch file:
    from tf_filter import TFFilterNode
    node = TFFilterNode(num_robots=3)
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_msgs.msg import TFMessage


INPUT_TOPIC  = "/tf_unfiltered"
OUTPUT_TOPIC = "/tf"

# Pares (frame_id, child_frame_id) que se eliminan para cada robot_{i}
BLOCKED_TEMPLATES: list[tuple[str, str]] = [
    ("map",             "robot_{i}/odom"),
    ("robot_{i}/odom",  "robot_{i}/base_footprint_link"),
    # El frame world de Gazebo tampoco pertenece al árbol ROS
    ("cocina_robotica_recreada", "robot_{i}"),
]


class TFFilterNode(Node):
    def __init__(self, num_robots: int):
        super().__init__("tf_filter")

        self._blocked: set[tuple[str, str]] = set()
        for i in range(num_robots):
            for pt, ct in BLOCKED_TEMPLATES:
                pair = (pt.format(i=i), ct.format(i=i))
                self._blocked.add(pair)
                self.get_logger().info(f"Bloqueado: '{pair[0]}' → '{pair[1]}'")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self._pub = self.create_publisher(TFMessage, OUTPUT_TOPIC, qos)
        self._sub = self.create_subscription(TFMessage, INPUT_TOPIC, self._cb, qos)
        self.get_logger().info(f"Filtrando '{INPUT_TOPIC}' → '{OUTPUT_TOPIC}'")

    def _cb(self, msg: TFMessage):
        filtered = [
            tf for tf in msg.transforms
            if (tf.header.frame_id, tf.child_frame_id) not in self._blocked
        ]
        if filtered:
            out = TFMessage()
            out.transforms = filtered
            self._pub.publish(out)


def main(num_robots: int = 1):
    rclpy.init()
    node = TFFilterNode(num_robots=num_robots)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    main(num_robots=n)

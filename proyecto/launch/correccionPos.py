#!/usr/bin/env python3
"""
correccionPos.py
─────────────────────────────────────────────────────────────────────────────
Lee /robot_{i}/ground_truth y publica la posición global directamente como
robot_{i}/odom → robot_{i}/base_footprint_link.

Funciona porque odom está fijo en (0,0,0) respecto a map, por lo que
la posición en el mundo ES la posición relativa a odom.

Además publica la posición xyz en /robot_{i}/pos_actual
(geometry_msgs/msg/PointStamped).

Árbol TF:
    map
    └── robot_{i}/odom  (0,0,0) ← static_transform_publisher identidad
        └── robot_{i}/base_footprint_link  ← posición real del robot

Uso:
    python3 correccionPos.py <num_robots>
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import TransformBroadcaster


GT_CHILD_TEMPLATE  = "robot_{i}"
GT_TOPIC_TEMPLATE  = "/robot_{i}/ground_truth"
ODOM_TEMPLATE      = "robot_{i}/odom"
FOOTPRINT_TEMPLATE = "robot_{i}/base_footprint_link"
POS_TOPIC_TEMPLATE = "/robot_{i}/pos_actual"


class CorreccionPosNode(Node):
    def __init__(self, num_robots: int):
        super().__init__("correccion_pos")

        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pos_publishers = {}

        for i in range(num_robots):
            topic      = GT_TOPIC_TEMPLATE.format(i=i)
            gt_child   = GT_CHILD_TEMPLATE.format(i=i)
            odom_frame = ODOM_TEMPLATE.format(i=i)
            foot_frame = FOOTPRINT_TEMPLATE.format(i=i)
            pos_topic  = POS_TOPIC_TEMPLATE.format(i=i)

            # Publicador de posición xyz
            self._pos_publishers[i] = self.create_publisher(
                PointStamped, pos_topic, 10
            )

            self.create_subscription(
                TFMessage, topic,
                lambda msg, gt=gt_child, odom=odom_frame, foot=foot_frame, ri=i:
                    self._cb(msg, gt, odom, foot, ri),
                qos,
            )
            self.get_logger().info(
                f"[robot_{i}] {topic} → {odom_frame} → {foot_frame} | pos: {pos_topic}"
            )

    def _cb(self, msg: TFMessage, gt_child, odom_frame, foot_frame, robot_id):
        gt = next((tf for tf in msg.transforms if tf.child_frame_id == gt_child), None)
        if gt is None:
            self.get_logger().warn(
                f"[robot_{robot_id}] '{gt_child}' no encontrado",
                throttle_duration_sec=5.0,
            )
            return

        # ── Publicar TF ───────────────────────────────────────────────────
        new_tf = TransformStamped()
        new_tf.header.stamp    = gt.header.stamp
        new_tf.header.frame_id = odom_frame
        new_tf.child_frame_id  = foot_frame
        new_tf.transform       = gt.transform

        self.tf_broadcaster.sendTransform(new_tf)

        # ── Publicar posición xyz en /robot_{i}/pos_actual ────────────────
        point = PointStamped()
        point.header.stamp    = gt.header.stamp
        point.header.frame_id = "map"
        point.point.x = gt.transform.translation.x
        point.point.y = gt.transform.translation.y
        point.point.z = gt.transform.translation.z

        self._pos_publishers[robot_id].publish(point)

        self.get_logger().debug(
            f"[robot_{robot_id}] x={point.point.x:.3f} "
            f"y={point.point.y:.3f} z={point.point.z:.3f}"
        )


def main(num_robots: int = 1):
    rclpy.init()
    node = CorreccionPosNode(num_robots=num_robots)
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
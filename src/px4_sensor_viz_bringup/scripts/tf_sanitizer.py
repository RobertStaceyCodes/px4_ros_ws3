#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def _short_frame(frame_id: str) -> str:
    value = frame_id.strip().strip("/")
    if "::" in value:
        value = value.split("::")[-1]
    if "/" in value:
        value = value.split("/")[-1]
    return value


class TfSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("tf_sanitizer")
        self._sub = self.create_subscription(TFMessage, "/tf_raw", self._on_tf, 50)
        self._pub = self.create_publisher(TFMessage, "/tf", 50)
        self._world_frame = self.declare_parameter("world_frame", "world").value.strip("/")
        self._base_frame = self.declare_parameter("base_frame", "base_link").value.strip("/")
        self._warned = False

    def _on_tf(self, msg: TFMessage) -> None:
        out = TFMessage()
        dropped = 0
        recovered_base_pose = False

        for t in msg.transforms:
            parent = t.header.frame_id.strip().strip("/")
            child = t.child_frame_id.strip().strip("/")

            # Gazebo Pose_V -> TFMessage bridge on this setup can arrive
            # with empty frame IDs. Recover the first entry as world->base_link.
            if not parent and not child and not recovered_base_pose:
                recovered = copy.deepcopy(t)
                recovered.header.frame_id = self._world_frame
                recovered.child_frame_id = self._base_frame
                out.transforms.append(recovered)
                recovered_base_pose = True
                continue

            if not parent or not child or parent == child:
                dropped += 1
                continue

            out.transforms.append(t)

            short_parent = _short_frame(parent)
            short_child = _short_frame(child)
            if short_parent and short_child and (short_parent != parent or short_child != child):
                alias = copy.deepcopy(t)
                alias.header.frame_id = short_parent
                alias.child_frame_id = short_child
                if alias.header.frame_id != alias.child_frame_id:
                    out.transforms.append(alias)

        if dropped > 0 and not self._warned:
            self.get_logger().warn(
                "Dropped malformed TF transforms from /tf_raw. "
                "Further warnings suppressed."
            )
            self._warned = True

        if out.transforms:
            self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = TfSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def _short_frame(frame_id: str) -> str:
    value = frame_id.strip().strip("/")
    if "::" in value:
        value = value.split("::")[-1]
    if "/" in value:
        value = value.split("/")[-1]
    return value


class TfSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("tf_sanitizer")
        self._sub = self.create_subscription(TFMessage, "/tf_raw", self._on_tf, 50)
        self._pub = self.create_publisher(TFMessage, "/tf", 50)
        self._warned = False

    def _on_tf(self, msg: TFMessage) -> None:
        out = TFMessage()
        dropped = 0

        for t in msg.transforms:
            parent = t.header.frame_id.strip().strip("/")
            child = t.child_frame_id.strip().strip("/")

            # Drop malformed transforms from Gazebo bridge.
            if not parent or not child or parent == child:
                dropped += 1
                continue

            out.transforms.append(t)

            # Add simplified alias frames for RViz usability, e.g.:
            # x500_oak_tfluna_d500_0::base_link -> base_link
            short_parent = _short_frame(parent)
            short_child = _short_frame(child)
            if short_parent and short_child and (short_parent != parent or short_child != child):
                alias = copy.deepcopy(t)
                alias.header.frame_id = short_parent
                alias.child_frame_id = short_child
                if alias.header.frame_id != alias.child_frame_id:
                    out.transforms.append(alias)

        if dropped > 0 and not self._warned:
            self.get_logger().warn(
                "Dropped malformed TF transforms from /tf_raw "
                "(empty/self transforms). Further warnings suppressed."
            )
            self._warned = True

        if out.transforms:
            self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = TfSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

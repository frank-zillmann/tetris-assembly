import select
import sys
import termios
import time
import tty
from pathlib import Path

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")
        self.rgb_topics = [
            "/camera/color/image_raw/compressed",
            "/gripper_camera/image_raw/compressed",
        ]
        self.depth_topic = "/camera/depth/image_raw/compressedDepth"
        self.ir_topic = "/camera/ir/image_raw/compressed"
        self.output_dir = Path("data/data_collection")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.latest = {}

        for topic in self.rgb_topics:
            self.create_subscription(
                CompressedImage, topic, lambda msg, t=topic: self._store(msg, t), 10
            )
        if self.depth_topic:
            self.create_subscription(
                CompressedImage,
                self.depth_topic,
                lambda msg, t=self.depth_topic: self._store(msg, t),
                10,
            )
        if self.ir_topic:
            self.create_subscription(
                CompressedImage,
                self.ir_topic,
                lambda msg, t=self.ir_topic: self._store(msg, t),
                10,
            )

        self.get_logger().info("Data collector: space to save (Ctrl+C to exit)")
        self.get_logger().info("Saving to: %s" % str(self.output_dir))

    def _store(self, msg, topic):
        self.latest[topic] = msg

    def capture(self):
        stamp = str(time.time_ns())
        capture_dir = self.output_dir / stamp
        capture_dir.mkdir(parents=True, exist_ok=True)
        topics = list(self.rgb_topics)
        if self.depth_topic:
            topics.append(self.depth_topic)
        if self.ir_topic:
            topics.append(self.ir_topic)

        saved = 0
        for topic in topics:
            msg = self.latest.get(topic)
            if msg is None:
                self.get_logger().warning("No image yet for %s" % topic)
                continue
            label = topic.strip("/").replace("/", "_")
            path = capture_dir / (label + ".png")
            if topic == self.depth_topic:
                depth = self._decode_image(msg, is_color=False)
                saved += self._save_image(depth, path)
                saved += self._save_depth_vis(depth, capture_dir / (label + "_vis.png"))
            elif topic in self.rgb_topics:
                saved += self._save_compressed(msg, path, is_color=True)
            else:
                saved += self._save_compressed(msg, path, is_color=False)
        if saved:
            self.get_logger().info("Saved %d images at %s" % (saved, stamp))

    def _save_compressed(self, msg, path, is_color):
        try:
            image = self._decode_image(msg, is_color)
            return self._save_image(image, path)
        except Exception as exc:
            self.get_logger().error("Failed to save %s: %s" % (str(path), str(exc)))
        return 0

    def _save_image(self, image, path):
        if image is None or image.size == 0:
            self.get_logger().error("Failed to decode %s" % str(path))
            return 0
        if cv2.imwrite(str(path), image):
            return 1
        return 0

    def _save_depth_vis(self, depth, path):
        try:
            if depth is None or depth.size == 0:
                return 0
            depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_u8 = depth_norm.astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
            if cv2.imwrite(str(path), depth_color):
                return 1
        except Exception as exc:
            self.get_logger().error("Failed to save %s: %s" % (str(path), str(exc)))
        return 0

    def _decode_image(self, msg, is_color):
        if "compressedDepth" in msg.format:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if raw.size <= 12:
                return None
            return cv2.imdecode(raw[12:], cv2.IMREAD_UNCHANGED)
        encoding = "bgr8" if is_color else "passthrough"
        return self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding=encoding)


def _keyboard_loop(node):
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == "\x03":
                    break
                if key == " ":
                    node.capture()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        _keyboard_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Read-only preview of either Lite3 ROS2 camera."""

import argparse
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VideoPreview(Node):
    def __init__(self, camera):
        super().__init__("lite3_camera_{}_preview".format(camera))
        self.topic = '/camera/color/image_raw' if camera == 1 else '/front_camera/image_raw'
        self.bridge = CvBridge()
        self.last_frame_time = None
        self.smoothed_fps = 0.0
        self.frame = None
        self.create_subscription(
            Image,
            self.topic,
            self.on_image,
            qos_profile_sensor_data,
        )

    def on_image(self, message):
        now = time.monotonic()
        if self.last_frame_time is not None and now > self.last_frame_time:
            instant_fps = 1.0 / (now - self.last_frame_time)
            self.smoothed_fps = (
                instant_fps if self.smoothed_fps == 0.0
                else 0.9 * self.smoothed_fps + 0.1 * instant_fps
            )
        self.last_frame_time = now
        self.frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")


def main():
    parser = argparse.ArgumentParser(description='Lite3 camera 1=D435i, 2=LRCP F1080P')
    parser.add_argument('--camera', type=int, choices=(1, 2), default=1)
    args = parser.parse_args()
    rclpy.init()
    node = VideoPreview(args.camera)
    window = "Lite3 Camera {} ({}) - ESC/q to close".format(
        args.camera, 'D435i' if args.camera == 1 else 'LRCP F1080P')
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print("Waiting for {} ...".format(node.topic), flush=True)
    started = time.monotonic()
    warned = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is not None:
                image = node.frame.copy()
                height, width = image.shape[:2]
                label = f"{width}x{height}  {node.smoothed_fps:.1f} fps"
                if time.monotonic() - node.last_frame_time > 2:
                    label += '  STALE / disconnected'
                cv2.putText(
                    image, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2, cv2.LINE_AA,
                )
                cv2.imshow(window, image)
            elif not warned and time.monotonic() - started > 10:
                print('No frames: check camera publisher and ROS2 network/domain. Camera 2 needs start_front_camera.sh on Jetson.', flush=True)
                warned = True
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

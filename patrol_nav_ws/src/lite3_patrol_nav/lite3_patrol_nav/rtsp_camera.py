"""Read-only RTSP bridge. Stamps are reception times, NOT exposure times."""
import os
from array import array
import threading
import time

os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage


class RtspCamera(Node):
    def __init__(self):
        super().__init__('front_camera_bridge')
        self.declare_parameter('url', 'rtsp://192.168.1.120:8554/test')
        self.declare_parameter('image_topic', '/front_camera/image_raw')
        self.declare_parameter('frame_id', 'front_camera_optical_frame')
        self.declare_parameter('publish_rate_hz', 10.0)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not 0 < rate <= 30:
            raise ValueError('publish_rate_hz must be in (0, 30]')
        self.pub = self.create_publisher(Image, self.get_parameter('image_topic').value,
                                         qos_profile_sensor_data)
        self.compressed_pub = self.create_publisher(
            CompressedImage, self.get_parameter('image_topic').value + '/compressed',
            qos_profile_sensor_data)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.thread = threading.Thread(target=self.capture, daemon=True)
        self.thread.start()
        self.create_timer(1.0 / rate, self.publish_latest)
        self.get_logger().warning('UNCALIBRATED: no CameraInfo or TF published; image stamps are reception times. Not enabled for navigation.')

    def capture(self):
        while not self.stop.is_set():
            cap = None
            try:
                cap = cv2.VideoCapture(self.get_parameter('url').value, cv2.CAP_FFMPEG,
                    [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
                     cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000])
                if not cap.isOpened():
                    raise RuntimeError('RTSP open failed')
                self.get_logger().info('RTSP connected')
                while not self.stop.is_set():
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('RTSP read failed')
                    stamp = self.get_clock().now().to_msg()
                    with self.lock:
                        self.latest = (frame, stamp, time.monotonic())
            except Exception as exc:
                if not self.stop.is_set():
                    self.get_logger().warning(str(exc) + '; reconnecting in 2 s')
            finally:
                if cap is not None:
                    cap.release()
                with self.lock:
                    self.latest = None
            self.stop.wait(2.0)

    def publish_latest(self):
        with self.lock:
            sample, self.latest = self.latest, None
        if sample is None or time.monotonic() - sample[2] > 0.5:
            return
        frame, stamp, _ = sample
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        payload = array('B')
        payload.frombytes(frame.tobytes())
        msg.data = payload
        self.pub.publish(msg)
        if self.compressed_pub.get_subscription_count():
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok:
                compressed = CompressedImage()
                compressed.header = msg.header
                compressed.format = 'bgr8; jpeg compressed bgr8'
                payload = array('B')
                payload.frombytes(jpeg.tobytes())
                compressed.data = payload
                self.compressed_pub.publish(compressed)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=7.0)


def main(args=None):
    rclpy.init(args=args)
    node = RtspCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

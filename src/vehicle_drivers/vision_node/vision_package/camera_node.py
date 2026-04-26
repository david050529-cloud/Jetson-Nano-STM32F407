#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)

        camera_id    = self.get_parameter('camera_id').get_parameter_value().integer_value
        width        = self.get_parameter('width').get_parameter_value().integer_value
        height       = self.get_parameter('height').get_parameter_value().integer_value
        fps          = self.get_parameter('fps').get_parameter_value().integer_value
        self.quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value

        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {camera_id}')
            rclpy.shutdown()
            return

        # 使用 MJPEG 采集：摄像头硬件直接输出 JPEG，大幅降低 USB 带宽占用
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # 缓冲区设为 1，始终读取最新帧，避免积压导致延迟
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.publisher = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', 10)

        self._latest_frame = None
        self._lock = threading.Lock()
        self._stopped = False

        # 独立采集线程持续抓帧，与发布解耦
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.timer = self.create_timer(1.0 / fps, self._publish_callback)

    def _capture_loop(self):
        while not self._stopped:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._latest_frame = frame

    def _publish_callback(self):
        with self._lock:
            frame = self._latest_frame

        if frame is None:
            return

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        success, buffer = cv2.imencode('.jpg', frame, encode_params)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()
        self.publisher.publish(msg)

    def destroy_node(self):
        self._stopped = True
        self._capture_thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

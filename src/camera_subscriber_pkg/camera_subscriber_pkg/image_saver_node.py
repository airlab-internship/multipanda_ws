#!/usr/bin/env python3
"""카메라 토픽(sensor_msgs/Image)을 구독해서 PNG 파일로 저장하는 노드."""
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        # 파라미터: 코드 수정 없이 실행할 때 바꿀 수 있는 값
        self.declare_parameter('image_topic', '/mujoco_server/cameras/hand_cam/rgb/image_raw')
        self.declare_parameter('save_dir', os.path.expanduser('~/multipanda_ws/data/hand_cam'))
        topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.count = 0
        # (메시지 타입, 토픽 이름, 콜백 함수, 큐 크기)
        self.create_subscription(Image, topic, self.callback, 10)
        self.get_logger().info(f'{topic} -> {self.save_dir}')

    def callback(self, msg):
        # MuJoCo는 rgb8로 발행한다. OpenCV는 BGR 순서로 저장하므로 bgr8로 변환
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(os.path.join(self.save_dir, f'frame_{self.count:06d}.png'), img)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

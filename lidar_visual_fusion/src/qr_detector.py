#!/usr/bin/env python3
"""
qr_detector.py
Читает /camera/image_raw, декодирует QR-коды,
берёт текущую позицию из /Odometry и публикует
результат в /inventory/detections
"""
import rospy
import cv2
import json
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class QRDetector:
    def __init__(self):
        rospy.init_node('qr_detector', anonymous=False)

        self.bridge = CvBridge()
        self.current_pose = None
        self.detector = cv2.QRCodeDetector()

        # уже замеченные QR чтобы не дублировать
        self.seen = {}   # qr_id -> количество детекций

        # параметры
        self.min_detections = rospy.get_param('~min_detections', 3)
        self.camera_topic   = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.odom_topic     = rospy.get_param('~odom_topic',   '/Odometry')

        # publishers
        self.pub_detection = rospy.Publisher(
            '/inventory/detections', String, queue_size=10)
        self.pub_debug = rospy.Publisher(
            '/inventory/debug_image', Image, queue_size=5)

        # subscribers
        rospy.Subscriber(self.odom_topic,   Odometry, self._odom_cb)
        rospy.Subscriber(self.camera_topic, Image,    self._image_cb)

        rospy.loginfo('[qr_detector] Запущен. Жду изображения...')
        rospy.spin()

    def _odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def _image_cb(self, msg):
        if self.current_pose is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5, f'[qr_detector] cv_bridge ошибка: {e}')
            return

        data, bbox, _ = self.detector.detectAndDecode(frame)

        if not data:
            return

        # Парсим JSON из QR
        try:
            info = json.loads(data)
            item_id = info.get('id', data)
        except json.JSONDecodeError:
            item_id = data
            info = {'id': data}

        # Считаем детекции — публикуем только после min_detections
        self.seen[item_id] = self.seen.get(item_id, 0) + 1

        if self.seen[item_id] == self.min_detections:
            p = self.current_pose.position
            result = {
                'id':   item_id,
                'desc': info.get('desc', ''),
                'x':    round(p.x, 3),
                'y':    round(p.y, 3),
                'z':    round(p.z, 3),
                'time': rospy.Time.now().to_sec(),
            }
            self.pub_detection.publish(String(data=json.dumps(result)))
            rospy.loginfo(f'[qr_detector] ✓ {item_id} на [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]')

        # Debug: рисуем bbox на кадре
        if bbox is not None and self.pub_debug.get_num_connections() > 0:
            bbox = bbox.astype(int)
            cv2.polylines(frame, [bbox], True, (0, 255, 0), 2)
            cv2.putText(frame, item_id, tuple(bbox[0][0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            try:
                self.pub_debug.publish(
                    self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            except Exception:
                pass


if __name__ == '__main__':
    try:
        QRDetector()
    except rospy.ROSInterruptException:
        pass

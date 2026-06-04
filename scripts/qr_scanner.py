#!/usr/bin/env python3
"""
QR Code Scanner с интеграцией OpenCV
Детектирует QR-коды, извлекает информацию о товарах
"""

import rospy
import cv2
import json
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import threading

# Попытаемся импортировать pyzbar для QR-детекции
try:
    from pyzbar.pyzbar import decode
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False
    rospy.logwarn('[QR] pyzbar not installed. Install: pip install pyzbar python-opencv')

class QRScanner:
    def __init__(self):
        rospy.init_node('qr_scanner')
        
        self.bridge = CvBridge()
        self.detected_qr_codes = {}
        self.last_detection_time = 0
        
        # Подписка на камеру
        rospy.Subscriber('/main_camera/image_raw', Image, self.image_callback, queue_size=1)
        
        # Публикатор результатов детекции
        self.detection_pub = rospy.Publisher('/inventory/detections', String, queue_size=1)
        
        rospy.loginfo('[QR] ✓ QR Scanner initialized')
        
        if not HAS_PYZBAR:
            rospy.logwarn('[QR] ⚠ pyzbar not available - using dummy detection')
    
    def image_callback(self, msg):
        """Получаем кадр с камеры"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            rospy.logwarn('[QR] Image conversion error: %s' % str(e))
    
    def process_frame(self, frame):
        """Обработка кадра для поиска QR-кодов"""
        current_time = rospy.Time.now().to_sec()
        
        # Дебаунс - не сканируем каждый кадр (максимум 2 раза в секунду)
        if current_time - self.last_detection_time < 0.5:
            return
        
        self.last_detection_time = current_time
        
        if HAS_PYZBAR:
            self._detect_qr_pyzbar(frame)
        else:
            self._detect_qr_dummy(frame)
    
    def _detect_qr_pyzbar(self, frame):
        """Детекция QR с использованием pyzbar"""
        try:
            # Конвертируем в grayscale для лучшей детекции
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Декодируем QR-коды
            qr_codes = decode(gray)
            
            if len(qr_codes) > 0:
                for qr in qr_codes:
                    qr_data = qr.data.decode('utf-8')
                    
                    try:
                        # Предполагаем что QR содержит JSON
                        item_data = json.loads(qr_data)
                        item_id = item_data.get('id', 'UNKNOWN')
                        
                        rospy.loginfo('[QR] ✓ Detected: %s' % item_id)
                        
                        # Публикуем результат
                        self.detection_pub.publish(String(data=json.dumps({
                            'id': item_id,
                            'timestamp': current_time,
                            'data': item_data
                        })))
                        
                        self.detected_qr_codes[item_id] = item_data
                    
                    except json.JSONDecodeError:
                        # Если не JSON, просто используем текст как ID
                        rospy.loginfo('[QR] ✓ Detected QR: %s' % qr_data)
                        self.detection_pub.publish(String(data=json.dumps({
                            'id': qr_data,
                            'timestamp': current_time
                        })))
        
        except Exception as e:
            rospy.logwarn('[QR] Detection error: %s' % str(e))
    
    def _detect_qr_dummy(self, frame):
        """
        Dummy детекция QR для тестирования
        Ищет контуры нужного размера (имитирует QR)
        """
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Применяем пороговое значение
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
            
            # Находим контуры
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            detected_items = set()
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # QR коды будут иметь определённый размер
                if 500 < area < 20000:
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Проверяем что это примерно квадрат
                    if 0.7 < float(w) / float(h) < 1.3:
                        # Имитируем детекцию на основе позиции
                        cx = x + w // 2
                        cy = y + h // 2
                        
                        # Простое сопоставление позиции с ID товара
                        item_id = self._position_to_item_id(cx, cy)
                        
                        if item_id and item_id not in detected_items:
                            rospy.loginfo('[QR] ~ Detected (simulated): %s (area=%d)' % (item_id, area))
                            self.detection_pub.publish(String(data=json.dumps({
                                'id': item_id,
                                'timestamp': rospy.Time.now().to_sec(),
                                'confidence': 0.7
                            })))
                            detected_items.add(item_id)
        
        except Exception as e:
            rospy.logwarn('[QR] Dummy detection error: %s' % str(e))
    
    def _position_to_item_id(self, x, y):
        """Простое сопоставление позиции пикселя с ID товара (для теста)"""
        # Разделяем кадр на сектора и сопоставляем с товарами
        frame_h, frame_w = 480, 640
        
        # Просто примерная логика - в реальности нужна калибровка
        if x < frame_w // 3:
            return 'ITEM_LEFT'
        elif x < 2 * frame_w // 3:
            return 'ITEM_CENTER'
        else:
            return 'ITEM_RIGHT'
    
    def get_detected_count(self):
        """Возвращает количество обнаруженных товаров"""
        return len(self.detected_qr_codes)


if __name__ == '__main__':
    try:
        scanner = QRScanner()
        rospy.loginfo('[QR] Ready to scan QR codes')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('[QR] Shutdown')

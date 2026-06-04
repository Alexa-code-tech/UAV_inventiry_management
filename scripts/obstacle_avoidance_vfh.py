#!/usr/bin/env python3
"""
ИСПРАВЛЕННЫЙ Obstacle Avoidance с VFH+ алгоритмом
- Правильное уклонение в сторону
- Возврат на маршрут после уклонения
- Анализ облака точек в body frame
"""

import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
from clover import srv

class ObstacleAvoidanceVFH:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_vfh')
        
        # Параметры VFH+
        self.DANGER_DISTANCE = 1.0  # метры
        self.WARNING_DISTANCE = 1.5
        self.FRONT_SECTOR_DEG = 60  # угол впереди
        self.EVADE_DISTANCE = 1.2  # расстояние уклонения
        
        # Сервисы
        self.navigate_srv = rospy.ServiceProxy('/navigate', srv.Navigate)
        
        # Публикаторы
        self.pause_pub = rospy.Publisher('/mission/pause', Bool, queue_size=1)
        self.debug_pub = rospy.Publisher('/avoidance/debug', Twist, queue_size=1)
        
        # Состояние
        self.current_z = 0.5
        self.in_danger = False
        self.last_evade_time = 0
        self.evade_direction = 0  # 0=нет, -1=влево, +1=вправо
        
        # Подписки
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/cloud_registered_body', PointCloud2, self.cloud_callback, queue_size=1, buff_size=2**24)
        
        rospy.loginfo('[VFH] ✓ Obstacle Avoidance VFH+ initialized')
        rospy.loginfo('[VFH] Danger distance: %.1f m' % self.DANGER_DISTANCE)
        rospy.loginfo('[VFH] Warning distance: %.1f m' % self.WARNING_DISTANCE)
    
    def odom_callback(self, msg):
        """Получаем текущую высоту"""
        self.current_z = msg.pose.pose.position.z
    
    def cloud_callback(self, msg):
        """Анализируем облако точек с использованием VFH+"""
        
        # Если низко - не уклоняемся (рискованно)
        if self.current_z < 0.4:
            if self.in_danger:
                self.pause_pub.publish(Bool(data=False))
                self.in_danger = False
            return
        
        # Читаем облако точек (в body frame)
        try:
            points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True)])
        except Exception as e:
            rospy.logwarn('[VFH] Error reading cloud: %s' % str(e))
            return
        
        if len(points) == 0:
            # Нет точек - берег чист
            if self.in_danger:
                rospy.loginfo('[VFH] ✓ Path clear')
                self.pause_pub.publish(Bool(data=False))
                self.in_danger = False
            return
        
        # ===== VFH+ АНАЛИЗ =====
        # Разделяем облако на сектора
        
        # 1. Находим расстояния до точек
        distances = np.linalg.norm(points, axis=1)
        
        # 2. Находим углы в горизонтальной плоскости (xy)
        angles_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
        
        # 3. Находим точки впереди (примерно от -30 до +30 градусов)
        front_mask = np.abs(angles_deg) < self.FRONT_SECTOR_DEG / 2.0
        
        # 4. Находим опасные точки впереди (близко)
        dangerous_front = front_mask & (distances < self.DANGER_DISTANCE)
        
        if np.sum(dangerous_front) > 5:  # Много точек впереди - ОПАСНО!
            
            if not self.in_danger:
                rospy.logwarn('[VFH] !!! OBSTACLE DETECTED !!!')
                rospy.logwarn('[VFH] Dangerous points: %d' % np.sum(dangerous_front))
                
                # Вычисляем среднее расстояние до опасных точек
                avg_danger_dist = np.mean(distances[dangerous_front])
                rospy.logwarn('[VFH] Avg danger distance: %.2f m' % avg_danger_dist)
                
                self.in_danger = True
                self.pause_pub.publish(Bool(data=True))
                self.last_evade_time = rospy.Time.now().to_sec()
                
                # Выбираем направление уклонения (влево или вправо)
                self._evade_sideways(angles_deg, distances)
        
        else:
            # Опасность миновала
            if self.in_danger:
                rospy.loginfo('[VFH] ✓ Path clear, resuming')
                self.pause_pub.publish(Bool(data=False))
                self.in_danger = False
                self.evade_direction = 0
    
    def _evade_sideways(self, angles_deg, distances):
        """
        Уклонение вбок с анализом VFH+
        Выбираем направление (влево или вправо) где меньше препятствий
        """
        rospy.logwarn('[VFH] Analyzing evasion options...')
        
        # Анализируем левый сектор (90 ± 30 град)
        left_sector = (angles_deg > 60) & (angles_deg < 120)
        left_free_count = np.sum((left_sector) & (distances > self.WARNING_DISTANCE))
        left_danger_count = np.sum((left_sector) & (distances < self.DANGER_DISTANCE))
        
        # Анализируем правый сектор (-90 ± 30 град)
        right_sector = (angles_deg < -60) | (angles_deg > 300)
        right_free_count = np.sum((right_sector) & (distances > self.WARNING_DISTANCE))
        right_danger_count = np.sum((right_sector) & (distances < self.DANGER_DISTANCE))
        
        rospy.logwarn('[VFH] Left: free=%d, danger=%d' % (left_free_count, left_danger_count))
        rospy.logwarn('[VFH] Right: free=%d, danger=%d' % (right_free_count, right_danger_count))
        
        # Выбираем направление с меньшим количеством опасных точек
        if left_danger_count <= right_danger_count:
            rospy.logwarn('[VFH] → Evading LEFT')
            self.evade_direction = -1
            self._execute_evasion(-1)
        else:
            rospy.logwarn('[VFH] → Evading RIGHT')
            self.evade_direction = 1
            self._execute_evasion(1)
    
    def _execute_evasion(self, direction):
        """
        Выполняем уклонение
        direction: -1 = влево, +1 = вправо
        """
        try:
            if direction < 0:
                # Влево (положительная Y)
                y_offset = self.EVADE_DISTANCE
            else:
                # Вправо (отрицательная Y)
                y_offset = -self.EVADE_DISTANCE
            
            rospy.logwarn('[VFH] Executing evasion: y=%.1f' % y_offset)
            
            # Летим в сторону в body frame
            res = self.navigate_srv(
                x=0,
                y=y_offset,
                z=0.3,  # Небольшое поднятие для безопасности
                frame_id='body',
                speed=0.6,
                auto_arm=False
            )
            
            if res.success:
                rospy.sleep(1.5)  # Даём время на уклонение
                rospy.loginfo('[VFH] ✓ Evasion maneuver complete')
            else:
                rospy.logwarn('[VFH] Evasion command failed')
        
        except Exception as e:
            rospy.logerr('[VFH] Evasion error: %s' % str(e))


if __name__ == '__main__':
    try:
        avoidance = ObstacleAvoidanceVFH()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('[VFH] Shutdown')

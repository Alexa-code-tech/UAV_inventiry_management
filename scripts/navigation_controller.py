#!/usr/bin/env python3
"""
ИСПРАВЛЕННЫЙ Navigation Controller
- Правильная обработка паузы от obstacle_avoidance
- Возврат на маршрут после уклонения
- Полный контроль над состоянием дрона
"""

import rospy
import math
import json
import yaml
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Bool, Float32
from clover import srv
from std_srvs.srv import Trigger

class NavigationController:
    def __init__(self):
        rospy.init_node('navigation_controller', log_level=rospy.INFO)
        
        # Загружаем конфиг
        config_path = os.path.expanduser('~/.ros/inventory_config.yaml')
        if not os.path.exists(config_path):
            config_path = os.path.dirname(__file__) + '/../config.yaml'
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Сервисы навигации
        self.navigate = rospy.ServiceProxy('/navigate', srv.Navigate)
        self.land = rospy.ServiceProxy('/land', Trigger)
        
        # Состояние
        self.current_pos = None
        self.current_yaw = 0.0
        self.pause_flag = False
        self.mission_running = False
        self.detected_items = {}
        
        # Подписки
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/mission/pause', Bool, self.pause_callback, queue_size=1)
        rospy.Subscriber('/inventory/detections', String, self.detection_callback, queue_size=1)
        
        # Публикаторы
        self.path_pub = rospy.Publisher('/mission/path', Path, queue_size=1)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=1)
        self.progress_pub = rospy.Publisher('/mission/progress', Float32, queue_size=1)
        
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        rospy.loginfo('[NAV] ✓ Navigation Controller initialized')
        
    def odom_callback(self, msg):
        """Получаем текущую позицию"""
        self.current_pos = msg.pose.pose.position
        
        # Публикуем текущий путь
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        
        if len(self.path.poses) % 10 == 0:
            self.path_pub.publish(self.path)
    
    def pause_callback(self, msg):
        """Получаем сигнал паузы от obstacle_avoidance"""
        self.pause_flag = msg.data
        if self.pause_flag:
            rospy.logwarn('[NAV] ⚠ PAUSE from obstacle avoidance!')
        else:
            rospy.loginfo('[NAV] ✓ RESUME after obstacle')
    
    def detection_callback(self, msg):
        """Получаем результаты детекции QR-кодов"""
        try:
            data = json.loads(msg.data)
            item_id = data.get('id')
            if item_id:
                self.detected_items[item_id] = data
                rospy.loginfo('[NAV] ✓ Detected: %s' % item_id)
        except Exception as e:
            rospy.logwarn('[NAV] Detection parse error: %s' % str(e))
    
    def distance_to(self, x, y, z):
        """Расстояние до точки"""
        if self.current_pos is None:
            return 999.0
        dx = self.current_pos.x - x
        dy = self.current_pos.y - y
        dz = self.current_pos.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def publish_status(self, status):
        """Публикуем статус"""
        self.status_pub.publish(String(data=status))
        rospy.loginfo('[NAV] Status: %s' % status)
    
    def publish_progress(self, current, total):
        """Публикуем прогресс (0-100%)"""
        progress = (float(current) / float(total)) * 100.0
        self.progress_pub.publish(Float32(data=progress))
    
    def takeoff(self, height=None):
        """Взлёт на высоту"""
        if height is None:
            height = self.config['navigation']['takeoff_height']
        
        rospy.loginfo('[NAV] 🚀 TAKEOFF to %.2f meters' % height)
        self.publish_status('TAKEOFF')
        
        try:
            res = self.navigate(
                x=0, y=0, z=height,
                frame_id='body',
                auto_arm=True,
                speed=0.5
            )
            
            if not res.success:
                rospy.logerr('[NAV] Takeoff failed!')
                return False
            
            rospy.sleep(3)
            
            # Проверяем что достигли высоты
            start_time = rospy.Time.now().to_sec()
            while not rospy.is_shutdown():
                if self.current_pos and abs(self.current_pos.z - height) < 0.1:
                    rospy.loginfo('[NAV] ✓ Takeoff complete at %.2f m' % self.current_pos.z)
                    return True
                
                if rospy.Time.now().to_sec() - start_time > 15:
                    rospy.logerr('[NAV] Takeoff timeout!')
                    return False
                
                rospy.sleep(0.1)
        
        except Exception as e:
            rospy.logerr('[NAV] Takeoff error: %s' % str(e))
        
        return False
    
    def goto_waypoint(self, x, y, z, item_id, scan_time=0):
        """
        Летим к точке сканирования с уважением к паузе.
        
        Args:
            x, y, z: координаты целевой точки
            item_id: ID товара для логирования
            scan_time: время на сканирование в точке (секунды)
        """
        nav_config = self.config['navigation']
        
        rospy.loginfo('[NAV] → Navigating to %s (%.2f, %.2f, %.2f)' % (item_id, x, y, z))
        
        try:
            res = self.navigate(
                x=x, y=y, z=z,
                yaw=float('nan'),
                speed=nav_config['flight_speed'],
                frame_id='map',
                auto_arm=False
            )
            
            if not res.success:
                rospy.logwarn('[NAV] Navigate service returned False for %s' % item_id)
                return False
        
        except Exception as e:
            rospy.logerr('[NAV] Navigate error: %s' % str(e))
            return False
        
        # Ждём достижения точки ИЛИ паузы
        tolerance = nav_config['waypoint_tolerance']
        timeout = nav_config['waypoint_timeout']
        start_time = rospy.Time.now().to_sec()
        last_dist = 999.0
        stuck_counter = 0
        
        while not rospy.is_shutdown():
            
            # ГЛАВНОЕ: если пауза - ждём
            if self.pause_flag:
                rospy.loginfo('[NAV] ⏸ Paused at %s, waiting for obstacle avoidance...' % item_id)
                rospy.sleep(0.5)
                continue
            
            dist = self.distance_to(x, y, z)
            elapsed = rospy.Time.now().to_sec() - start_time
            
            # Логируем каждую секунду
            if int(elapsed) % 2 == 0 and elapsed > 1:
                rospy.loginfo('[NAV] → Distance to %s: %.2f m (elapsed: %.1f s)' % (item_id, dist, elapsed))
            
            # Проверяем застревание (если расстояние не уменьшается)
            if dist >= last_dist:
                stuck_counter += 1
            else:
                stuck_counter = 0
            last_dist = dist
            
            # Достигли точки
            if dist <= tolerance:
                rospy.loginfo('[NAV] ✓ Reached %s (dist=%.2f m)' % (item_id, dist))
                
                # Сканируем товар
                if scan_time > 0:
                    rospy.loginfo('[NAV] 📸 Scanning %s for %.1f seconds' % (item_id, scan_time))
                    rospy.sleep(scan_time)
                
                return True
            
            # Timeout
            if elapsed > timeout:
                rospy.logwarn('[NAV] Timeout for %s (dist=%.2f m)' % (item_id, dist))
                return False
            
            # Если застряли - пробуем восстановиться
            if stuck_counter > 10:
                rospy.logwarn('[NAV] Stuck, retrying navigation to %s' % item_id)
                try:
                    self.navigate(
                        x=x, y=y, z=z,
                        yaw=float('nan'),
                        speed=nav_config['flight_speed'] * 0.8,
                        frame_id='map',
                        auto_arm=False
                    )
                    stuck_counter = 0
                except:
                    pass
            
            rospy.sleep(0.1)
        
        return False
    
    def land(self):
        """Посадка"""
        rospy.loginfo('[NAV] 🛬 LANDING')
        self.publish_status('LANDING')
        
        try:
            res = self.land()
            rospy.sleep(3)
            rospy.loginfo('[NAV] ✓ Landing complete')
            return True
        except Exception as e:
            rospy.logerr('[NAV] Landing error: %s' % str(e))
        
        return False
    
    def run_mission(self):
        """Основная миссия"""
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        rospy.loginfo('[NAV] INVENTORY SCANNING MISSION STARTED')
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        
        self.mission_running = True
        
        # Ждём сервисов
        try:
            rospy.wait_for_service('/navigate', timeout=30)
            rospy.wait_for_service('/land', timeout=30)
        except rospy.ROSException:
            rospy.logerr('[NAV] ROS services not available!')
            return False
        
        rospy.sleep(2)
        
        # ===== PHASE 1: TAKEOFF =====
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        rospy.loginfo('[NAV] PHASE 1: TAKEOFF')
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        
        if not self.takeoff():
            rospy.logerr('[NAV] Takeoff failed!')
            return False
        
        # ===== PHASE 2: SCANNING =====
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        rospy.loginfo('[NAV] PHASE 2: SCANNING ROUTE')
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        
        waypoints = self.config['navigation']['scanning_waypoints']
        
        for idx, wp in enumerate(waypoints):
            if rospy.is_shutdown():
                break
            
            # Публикуем прогресс
            self.publish_progress(idx, len(waypoints))
            
            x, y, z = wp['x'], wp['y'], wp['z']
            item_id = wp['description']
            scan_time = wp.get('pause', 0)
            
            # Летим к точке
            if not self.goto_waypoint(x, y, z, item_id, scan_time):
                rospy.logwarn('[NAV] Could not reach %s' % item_id)
        
        self.publish_progress(len(waypoints), len(waypoints))
        
        # ===== PHASE 3: LANDING =====
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        rospy.loginfo('[NAV] PHASE 3: LANDING')
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        
        self.land()
        
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        rospy.loginfo('[NAV] ✓ MISSION COMPLETE')
        rospy.loginfo('[NAV] Detected items: %d' % len(self.detected_items))
        rospy.loginfo('[NAV] ════════════════════════════════════════')
        
        self.mission_running = False
        self.publish_status('MISSION_COMPLETE')
        
        return True


if __name__ == '__main__':
    try:
        controller = NavigationController()
        controller.run_mission()
    except rospy.ROSInterruptException:
        rospy.loginfo('[NAV] Mission interrupted')

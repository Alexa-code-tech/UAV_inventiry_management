#!/usr/bin/env python3
"""
Inventory Database Manager
Сохраняет результаты инвенторизации в SQLite и JSON
Сравнивает с эталонной БД и определяет расхождения
"""

import rospy
import json
import sqlite3
import os
from datetime import datetime
from std_msgs.msg import String
import yaml

class InventoryDatabase:
    def __init__(self):
        rospy.init_node('inventory_database')
        
        # Пути к БД
        self.db_dir = os.path.expanduser('~/.ros/inventory_data')
        if not os.path.exists(self.db_dir):
            os.makedirs(self.db_dir)
        
        self.db_file = os.path.join(self.db_dir, 'inventory.db')
        self.json_file = os.path.join(self.db_dir, 'inventory_scan.json')
        
        # Загружаем конфиг с эталонными позициями товаров
        config_path = os.path.expanduser('~/.ros/inventory_config.yaml')
        if not os.path.exists(config_path):
            config_path = os.path.dirname(__file__) + '/../config.yaml'
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.reference_inventory = self.config.get('inventory', {}).get('items', {})
        
        # Текущая инвентаризация
        self.current_scan = {
            'timestamp': datetime.now().isoformat(),
            'detected_items': {},
            'missing_items': {},
            'extra_items': {},
            'misplaced_items': {}
        }
        
        # Инициализируем БД
        self._init_database()
        
        # Подписка на детекции
        rospy.Subscriber('/inventory/detections', String, self.detection_callback, queue_size=10)
        rospy.Subscriber('/mission/status', String, self.status_callback, queue_size=1)
        
        # Публикатор результатов
        self.report_pub = rospy.Publisher('/inventory/report', String, queue_size=1)
        
        rospy.loginfo('[DB] ✓ Inventory Database Manager initialized')
        rospy.loginfo('[DB] Database: %s' % self.db_file)
        rospy.loginfo('[DB] Reference items: %d' % len(self.reference_inventory))
    
    def _init_database(self):
        """Инициализируем SQLite БД"""
        conn = sqlite3.connect(self.db_file)
        cursor = conn.cursor()
        
        # Таблица сканирований
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS scans (
                id INTEGER PRIMARY KEY,
                timestamp TEXT,
                status TEXT,
                total_items_found INTEGER,
                missing_items INTEGER,
                misplaced_items INTEGER
            )
        ''')
        
        # Таблица обнаруженных товаров
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS detected_items (
                id INTEGER PRIMARY KEY,
                scan_id INTEGER,
                item_id TEXT,
                timestamp TEXT,
                x REAL, y REAL, z REAL,
                confidence REAL,
                FOREIGN KEY(scan_id) REFERENCES scans(id)
            )
        ''')
        
        # Таблица расхождений
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS discrepancies (
                id INTEGER PRIMARY KEY,
                scan_id INTEGER,
                item_id TEXT,
                status TEXT,
                expected_x REAL, expected_y REAL, expected_z REAL,
                actual_x REAL, actual_y REAL, actual_z REAL,
                distance_error REAL,
                FOREIGN KEY(scan_id) REFERENCES scans(id)
            )
        ''')
        
        conn.commit()
        conn.close()
        
        rospy.loginfo('[DB] ✓ Database initialized')
    
    def detection_callback(self, msg):
        """Получаем детекцию QR-кода"""
        try:
            data = json.loads(msg.data)
            item_id = data.get('id')
            
            if item_id:
                rospy.loginfo('[DB] Recorded detection: %s' % item_id)
                self.current_scan['detected_items'][item_id] = {
                    'timestamp': data.get('timestamp'),
                    'data': data.get('data', {})
                }
        
        except Exception as e:
            rospy.logwarn('[DB] Detection parse error: %s' % str(e))
    
    def status_callback(self, msg):
        """Получаем статус миссии"""
        status = msg.data
        
        if status == 'MISSION_COMPLETE':
            rospy.loginfo('[DB] Mission complete - generating report')
            self._finalize_scan()
            self._generate_report()
    
    def _finalize_scan(self):
        """Завершаем сканирование и анализируем расхождения"""
        
        # Находим пропущенные товары
        detected_ids = set(self.current_scan['detected_items'].keys())
        reference_ids = set(self.reference_inventory.keys())
        
        missing_ids = reference_ids - detected_ids
        extra_ids = detected_ids - reference_ids
        
        self.current_scan['missing_items'] = {
            item_id: self.reference_inventory[item_id]
            for item_id in missing_ids
        }
        
        self.current_scan['extra_items'] = {
            item_id: self.current_scan['detected_items'][item_id]
            for item_id in extra_ids
        }
        
        # Находим неправильно размещённые товары
        for item_id in detected_ids & reference_ids:
            detected_data = self.current_scan['detected_items'][item_id].get('data', {})
            reference_data = self.reference_inventory[item_id]
            
            detected_pos = (detected_data.get('x'), detected_data.get('y'), detected_data.get('z'))
            reference_pos = (reference_data.get('x'), reference_data.get('y'), reference_data.get('z'))
            
            # Если есть координаты - проверяем расстояние
            if all(detected_pos) and all(reference_pos):
                import math
                dx = detected_pos[0] - reference_pos[0]
                dy = detected_pos[1] - reference_pos[1]
                dz = detected_pos[2] - reference_pos[2]
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Если расстояние > 0.5 м - товар неправильно размещён
                if distance > 0.5:
                    self.current_scan['misplaced_items'][item_id] = {
                        'expected': reference_pos,
                        'actual': detected_pos,
                        'distance_error': distance
                    }
        
        rospy.loginfo('[DB] Scan finalized:')
        rospy.loginfo('[DB]   Found: %d items' % len(detected_ids))
        rospy.loginfo('[DB]   Missing: %d items' % len(missing_ids))
        rospy.loginfo('[DB]   Extra: %d items' % len(extra_ids))
        rospy.loginfo('[DB]   Misplaced: %d items' % len(self.current_scan['misplaced_items']))
    
    def _generate_report(self):
        """Генерируем отчёт"""
        
        # Сохраняем в JSON
        with open(self.json_file, 'w') as f:
            json.dump(self.current_scan, f, indent=2, ensure_ascii=False)
        
        rospy.loginfo('[DB] ✓ JSON report saved: %s' % self.json_file)
        
        # Сохраняем в БД
        conn = sqlite3.connect(self.db_file)
        cursor = conn.cursor()
        
        # Вставляем запись о сканировании
        cursor.execute('''
            INSERT INTO scans (timestamp, status, total_items_found, missing_items, misplaced_items)
            VALUES (?, ?, ?, ?, ?)
        ''', (
            self.current_scan['timestamp'],
            'COMPLETE',
            len(self.current_scan['detected_items']),
            len(self.current_scan['missing_items']),
            len(self.current_scan['misplaced_items'])
        ))
        
        scan_id = cursor.lastrowid
        
        # Вставляем обнаруженные товары
        for item_id, data in self.current_scan['detected_items'].items():
            cursor.execute('''
                INSERT INTO detected_items (scan_id, item_id, timestamp, confidence)
                VALUES (?, ?, ?, ?)
            ''', (
                scan_id,
                item_id,
                data.get('timestamp'),
                data.get('data', {}).get('confidence', 0.8)
            ))
        
        # Вставляем расхождения
        for item_id in self.current_scan['missing_items']:
            ref = self.reference_inventory[item_id]
            cursor.execute('''
                INSERT INTO discrepancies (scan_id, item_id, status, expected_x, expected_y, expected_z)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (
                scan_id,
                item_id,
                'MISSING',
                ref.get('x'), ref.get('y'), ref.get('z')
            ))
        
        for item_id, data in self.current_scan['misplaced_items'].items():
            exp_pos = data['expected']
            act_pos = data['actual']
            cursor.execute('''
                INSERT INTO discrepancies (scan_id, item_id, status, 
                    expected_x, expected_y, expected_z,
                    actual_x, actual_y, actual_z, distance_error)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                scan_id,
                item_id,
                'MISPLACED',
                exp_pos[0], exp_pos[1], exp_pos[2],
                act_pos[0], act_pos[1], act_pos[2],
                data['distance_error']
            ))
        
        conn.commit()
        conn.close()
        
        rospy.loginfo('[DB] ✓ Database updated')
        
        # Публикуем отчёт
        report = self._format_report()
        self.report_pub.publish(String(data=report))
    
    def _format_report(self):
        """Форматируем читаемый отчёт"""
        report = []
        report.append('╔════════════════════════════════════════════════════╗')
        report.append('║        INVENTORY SCANNING REPORT                   ║')
        report.append('╚════════════════════════════════════════��═══════════╝')
        report.append('')
        report.append('Timestamp: %s' % self.current_scan['timestamp'])
        report.append('')
        report.append('SUMMARY:')
        report.append('  ✓ Found items: %d' % len(self.current_scan['detected_items']))
        report.append('  ✗ Missing items: %d' % len(self.current_scan['missing_items']))
        report.append('  ? Extra items: %d' % len(self.current_scan['extra_items']))
        report.append('  ⚠ Misplaced items: %d' % len(self.current_scan['misplaced_items']))
        report.append('')
        
        if self.current_scan['missing_items']:
            report.append('MISSING ITEMS:')
            for item_id, data in self.current_scan['missing_items'].items():
                report.append('  - %s (%s) at (%.2f, %.2f, %.2f)' % (
                    item_id,
                    data.get('name', 'Unknown'),
                    data.get('x', 0), data.get('y', 0), data.get('z', 0)
                ))
            report.append('')
        
        if self.current_scan['misplaced_items']:
            report.append('MISPLACED ITEMS:')
            for item_id, data in self.current_scan['misplaced_items'].items():
                exp = data['expected']
                act = data['actual']
                report.append('  - %s' % item_id)
                report.append('    Expected: (%.2f, %.2f, %.2f)' % (exp[0], exp[1], exp[2]))
                report.append('    Actual:   (%.2f, %.2f, %.2f)' % (act[0], act[1], act[2]))
                report.append('    Error: %.2f m' % data['distance_error'])
            report.append('')
        
        if self.current_scan['extra_items']:
            report.append('EXTRA ITEMS:')
            for item_id in self.current_scan['extra_items'].keys():
                report.append('  - %s (not in reference inventory)' % item_id)
            report.append('')
        
        report.append('╔════════════════════════════════════════════════════╗')
        report.append('  Full report saved to: %s' % self.json_file)
        report.append('╚════════════════════════════════════════════════════╝')
        
        return '\n'.join(report)


if __name__ == '__main__':
    try:
        db = InventoryDatabase()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('[DB] Shutdown')

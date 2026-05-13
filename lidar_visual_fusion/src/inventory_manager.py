#!/usr/bin/env python3
"""
inventory_manager.py
Собирает детекции из /inventory/detections,
строит таблицу, сохраняет в CSV и публикует статус.
"""
import rospy
import json
import csv
import os
from std_msgs.msg import String
from datetime import datetime


class InventoryManager:
    def __init__(self):
        rospy.init_node('inventory_manager', anonymous=False)

        self.save_path = rospy.get_param(
            '~save_path',
            os.path.expanduser('~/inventory_result.csv'))

        # {item_id: {desc, x, y, z, time}}
        self.inventory = {}

        self.pub_status = rospy.Publisher(
            '/inventory/status', String, queue_size=5)

        rospy.Subscriber('/inventory/detections', String, self._detection_cb)

        # Сохраняем CSV при выходе
        rospy.on_shutdown(self._save_csv)

        # Статус каждые 10 секунд
        rospy.Timer(rospy.Duration(10.0), self._status_cb)

        rospy.loginfo(f'[inventory] Запущен. Результат: {self.save_path}')
        rospy.spin()

    def _detection_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        item_id = data['id']

        if item_id not in self.inventory:
            self.inventory[item_id] = data
            rospy.loginfo(
                f'[inventory] Новый товар: {item_id} | {data.get("desc","")} '
                f'| pos=[{data["x"]}, {data["y"]}, {data["z"]}]'
            )
            # Сохраняем сразу при каждой новой детекции
            self._save_csv()

            status = f'Найдено: {len(self.inventory)} товаров. Последний: {item_id}'
            self.pub_status.publish(String(data=status))

    def _status_cb(self, _event):
        n = len(self.inventory)
        rospy.loginfo(f'[inventory] Статус: найдено {n} / 26 товаров')
        if n == 26:
            rospy.loginfo('[inventory] ✅ ВСЕ ТОВАРЫ НАЙДЕНЫ!')

    def _save_csv(self):
        if not self.inventory:
            return
        try:
            with open(self.save_path, 'w', newline='') as f:
                writer = csv.DictWriter(
                    f, fieldnames=['id', 'desc', 'x', 'y', 'z', 'time'])
                writer.writeheader()
                for item in sorted(self.inventory.values(),
                                   key=lambda x: x['id']):
                    writer.writerow(item)
            rospy.loginfo(f'[inventory] CSV сохранён: {self.save_path}')
        except Exception as e:
            rospy.logerr(f'[inventory] Ошибка сохранения CSV: {e}')


if __name__ == '__main__':
    try:
        InventoryManager()
    except rospy.ROSInterruptException:
        pass

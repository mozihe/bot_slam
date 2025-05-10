import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sqlite3
from datetime import datetime
from ultralytics import YOLO

import sqlite3

def init_db():
    conn = sqlite3.connect('detections.db')
    c = conn.cursor()
    c.execute('''
        CREATE TABLE IF NOT EXISTS tracking_records (
            track_id INTEGER PRIMARY KEY,
            label TEXT,
            first_seen TEXT,
            last_seen TEXT
        )
    ''')
    conn.commit()
    conn.close()

def insert_new_tracking_record(track_id, label, first_seen):
    conn = sqlite3.connect('detections.db')
    c = conn.cursor()
    c.execute('''
        INSERT INTO tracking_records (track_id, label, first_seen, last_seen) VALUES (?, ?, ?, ?)
    ''', (track_id, label, first_seen, first_seen))
    conn.commit()
    conn.close()

def update_tracking_record(track_id, last_seen):
    conn = sqlite3.connect('detections.db')
    c = conn.cursor()
    c.execute('''
        UPDATE tracking_records SET last_seen = ? WHERE track_id = ?
    ''', (last_seen, track_id))
    conn.commit()
    conn.close()

def check_if_track_exists(track_id):
    conn = sqlite3.connect('detections.db')
    c = conn.cursor()
    c.execute("SELECT 1 FROM tracking_records WHERE track_id = ?", (track_id,))
    result = c.fetchone()
    conn.close()
    return result is not None

def get_max_tracker_id(db_path='detections.db'):
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute("SELECT MAX(track_id) FROM tracking_records")
    max_id = c.fetchone()[0]
    conn.close()
    return (max_id if max_id is not None else 0) + 1

class YoloTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/fishbot_camera_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/yolo/tracked_image', 10)
        
        init_db()
        self.yolo_model = YOLO('yolov8n.pt')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.yolo_model.track(cv_image, persist=True)
        annotated_frame = results[0].plot()

        if results[0].boxes.id is not None:
            current_tracks = results[0].boxes.id.int().cpu().numpy()
            labels = results[0].boxes.cls.int().cpu().numpy()

            for track_id, label_idx in zip(current_tracks, labels):
                track_id = int(track_id) 
                label = self.yolo_model.names[label_idx]
                timestamp = datetime.now().isoformat()

                if check_if_track_exists(track_id):
                    update_tracking_record(track_id, timestamp)
                else:
                    insert_new_tracking_record(track_id, label, timestamp)

        output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

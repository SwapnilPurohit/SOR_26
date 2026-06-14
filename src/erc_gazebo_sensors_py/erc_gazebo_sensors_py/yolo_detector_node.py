import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Load YOLOv8 model
        # Options: yolov8n.pt (fastest), yolov8s.pt, yolov8m.pt, yolov8l.pt, yolov8x.pt (most accurate)
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8 model loaded')

        self.subscription = self.create_subscription(
            Image,
            'camera/image',       # ← change to your topic
            self.image_callback,
            1
        )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def stop(self):
        self.running = False
        self.spin_thread.join()

    def display_image(self):
        cv2.namedWindow('YOLO Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('YOLO Detection', 800, 600)

        while rclpy.ok():
            with self.frame_lock:
                frame = self.latest_frame
                self.latest_frame = None

            if frame is not None:
                result_frame = self.run_yolo(frame)
                cv2.imshow('YOLO Detection', result_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        cv2.destroyAllWindows()
        self.running = False

    def run_yolo(self, img):
        """Run YOLOv8 inference and draw boxes on the frame."""

        CONF_THRESHOLD = 0.6      # ← minimum confidence to show a box
        TARGET_CLASSES = None     # ← None = detect all. Set e.g. ['person', 'cup'] to filter

        results = self.model(img, conf=CONF_THRESHOLD, verbose=False)

        for result in results:
            for box in result.boxes:
                # Get box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Get class name and confidence
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                # Skip if not in target classes
                if TARGET_CLASSES is not None and class_name not in TARGET_CLASSES:
                    continue

                # Pick a consistent color per class
                color = self.class_color(class_id)

                # Draw bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

                # Draw label background
                label = f'{class_name} {confidence:.2f}'
                (lw, lh), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(img, (x1, y1 - lh - baseline - 5), (x1 + lw, y1), color, -1)

                # Draw label text
                cv2.putText(img, label, (x1, y1 - baseline - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Draw centroid dot
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.circle(img, (cx, cy), 5, color, -1)

                self.get_logger().info(
                    f'Detected: {class_name} | conf: {confidence:.2f} | '
                    f'box: ({x1},{y1}) → ({x2},{y2})'
                )

        # Draw detection count on frame
        count = len(results[0].boxes) if results else 0
        cv2.putText(img, f'Objects: {count}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        return img

    def class_color(self, class_id):
        """Generate a unique BGR color per class id."""
        np.random.seed(class_id)
        return tuple(int(c) for c in np.random.randint(100, 255, 3))


def main(args=None):
    print('OpenCV version: %s' % cv2.__version__)
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        node.display_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
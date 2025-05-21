import rclpy
from rclpy.node import Node
from cv2 import VideoCapture, imshow, waitKey, destroyAllWindows
import numpy as np
from qreader import QReader
from std_srvs.srv import Trigger
import threading

class QRCodeDroneService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')
        self.srv = self.create_service(Trigger, 'start_stop_qr_detection', self.start_stop_callback)
        self.cap = None
        self.camera_index = 0
        self.is_detecting = False
        self.current_obj = 0
        self.direction = ''
        self.thread = None

    def start_stop_callback(self, request, response):
        """
        Service callback to start or stop QR code detection.
        """
        if self.is_detecting:
            self.is_detecting = False
            response.success = True
            response.message = "QR Code detection stopped."
        else:
            success = self.start_detection()
            response.success = success
            response.message = "QR Code detection started." if success else "Failed to start QR detection."
        return response

    def start_detection(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                self.get_logger().error(f"Error: Could not open camera at index {self.camera_index}")
                return False
        self.is_detecting = True
        self.get_logger().info("QR Code detection started.")

        # Start the camera feed in a separate thread
        self.thread = threading.Thread(target=self.camera_feed)
        self.thread.daemon = True
        self.thread.start()
        return True

    def camera_feed(self):
        """
        Reads frames from the camera and processes them in real-time.
        Runs in a separate thread.
        """
        while self.is_detecting:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to grab frame,  stopping detection.")
                # self.stop_detection()
                break

            qreader_out = self.decode_qr_from_frame(frame)
            degree_info = self.detect_degree_information(frame)
            cleaned_list = [x for x in qreader_out if x is not None]
            if cleaned_list:
                self.get_logger().info(f"QR Code Detected: {cleaned_list}  {degree_info}")
                try:
                    for r in cleaned_list:
                        sequence = r.split(',')
                        num = int(sequence[-1])
                        if self.current_obj == 0:
                            self.current_obj = num
                            self.get_logger().info(f"Set initial object: {self.current_obj}  {degree_info}")
                        if num == self.current_obj:
                            self.direction = sequence[num - 1]
                            self.get_logger().info(f"Direction: {self.direction} {degree_info}")

                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f"Error processing QR code data: {e}")

            # Display frame (optional)
            imshow("QR Code Scanner", frame)
            if waitKey(1) & 0xFF == ord('q'):
                # self.stop_detection()  # Stop detection on 'q'
                break

        # Clean up when the loop exits
        self.cap.release()
        destroyAllWindows()

    def calculate_rotation_degrees(self, corners):
        try:
            corners = np.array(corners)
            if corners.shape != (4, 2):
                return (None, None)  # Invalid input

            v1 = corners[1] - corners[0]
            v2 = corners[3] - corners[0]

            v1_normalized = v1 / np.linalg.norm(v1) if np.linalg.norm(v1) > 0 else np.array([1, 0])
            v2_normalized = v2 / np.linalg.norm(v2) if np.linalg.norm(v2) > 0 else np.array([0, 1])

            x_rotation_radians = np.arctan2(v1_normalized[1], v1_normalized[0])
            y_rotation_radians = np.arctan2(v2_normalized[0], v2_normalized[1])

            x_rotation_degrees = np.degrees(x_rotation_radians)
            y_rotation_degrees = np.degrees(y_rotation_radians)

            rotation_degrees = (x_rotation_degrees, y_rotation_degrees)

            translation = -corners[0]

            return (rotation_degrees, translation)

        except (ValueError, ZeroDivisionError):
            return (None, None)

    def decode_qr_from_frame(self, frame):
        qreader_out = QReader().detect_and_decode(image=frame)
        return qreader_out
    
    def detect_degree_information(self, frame):
        detection_results = QReader().detect(image=frame)
        if detection_results:
            for k, v in enumerate(detection_results):
                return self.calculate_rotation_degrees(detection_results[k]["quad_xy"])

def main(args=None):
    rclpy.init(args=args)
    qr_drone_service = QRCodeDroneService()
    rclpy.spin(qr_drone_service)
    qr_drone_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


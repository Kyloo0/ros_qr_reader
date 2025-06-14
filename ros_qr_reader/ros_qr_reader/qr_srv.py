import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import numpy as np
from qreader import QReader
from cv2 import imshow, waitKey

class QRCodeDroneService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')

        self.srv = self.create_service(Trigger, 'start_stop_qr_detection', self.start_stop_callback)
        self.is_detecting = False
        self.current_obj = 0
        self.direction = ''
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Gantilah dengan topik kamera yang sesuai
            self.image_callback,
            10)

    def start_stop_callback(self, request, response):
        """
        Service callback to start or stop QR code detection.
        """
        if self.is_detecting:
            self.is_detecting = False
            response.success = True
            response.message = "QR Code detection stopped."
        else:
            self.is_detecting = True
            response.success = True
            response.message = "QR Code detection started."
        return response

    def image_callback(self, msg: Image):
        """
        Callback function for receiving and processing images.
        """
        if not self.is_detecting:
            return

        try:
            # Convert the ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Decode QR from the frame
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
                self.is_detecting = False  # Stop detection when 'q' is pressed

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def calculate_rotation_degrees(self, corners):
        """
        Calculates the rotation degrees based on the QR code corners.
        """
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
        """
        Decodes QR code from the frame using QReader.
        """
        qreader_out = QReader().detect_and_decode(image=frame)
        return qreader_out
    
    def detect_degree_information(self, frame):
        """
        Detects the degree of rotation of the QR code in the frame.
        """
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

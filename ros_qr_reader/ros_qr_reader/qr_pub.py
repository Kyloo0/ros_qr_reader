import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Point
from vision_direction.srv import VisionDirection
import numpy as np
from qreader import QReader
from cv2 import imshow, waitKey, putText, polylines, circle, FONT_HERSHEY_SIMPLEX, LINE_AA
import re

class QRCodeDroneService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')

        # self.srv = self.create_service(VisionDirection, 'start_stop_qr_detection', self.start_stop_callback)
        # self.is_detecting = False
        self.current_obj = 0
        self.direction = ''
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.image_content,10)
        # self.publisher_direction = self.create_publisher(String, 'qr_direction', 10)
        self.publisher_center = self.create_publisher(Point, 'qr_center_front', 10)
        self.publisher_content = self.create_publisher(String, 'qr_content', 10)


        timer_period = 10
        self.timer = self.create_timer(timer_period, self.image_content)
        self.success = True

    # def start_stop_callback(self, request, response):
    #     "Service callback to start or stop QR code detection."
    #     if request.data< 1 or request.data > 4:
    #         response.success = False
    #         response.message = f"Mission {request.data} error, please check your input!"
    #         return response

    #     self.current_obj = request.data
    #     self.is_detecting = True
    #     response.success = True
    #     response.message = f"QR Code detection {'started' if self.is_detecting else 'stopped'}."
    #     return response

    def image_content(self, msg=Image):
        "Callback function for receiving and processing images."
        content = String()
        # dir = String()
        # target = Int8()

        # if not self.is_detecting:
        #     return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Decode QR from the frame
            qreader_out = self.decode_qr_from_frame(frame)

            cleaned_content = [x for x in qreader_out if x is not None]
            center_info = self.detect_center_information(frame)

            if cleaned_content : 
                content.data = str(cleaned_content)
                self.publisher_content.publish(content)
                self.get_logger().info(f'Content : {cleaned_content}')
                self.publisher_center.publish(center_info)
                self.is_detecting = False


            # for msg in qreader_out:
            #     if not isinstance(msg, str) or not re.match(r'^\s*(?:[NSEW]\s*,\s*)*[NSEW]\s*,\s*\d+\s*$', msg):                    
            #         self.get_logger().info(f"[Mission {self.current_obj}] is not in valid format. Message: {qreader_out}")
            #         self.success = False
            #         break
            #     self.success = True


            # cleaned_list = [x for x in qreader_out if x is not None]
            # if cleaned_list and self.success:
            #     self.get_logger().info(f"QR Code Detected: {cleaned_list}  {center_info}")
            #     for r in cleaned_list:
            #         sequence = r.split(',')
                    
            #         try:
            #             qr_target = int(sequence[-1])
            #             direction = sequence[self.current_obj - 1]
            #             self.direction = direction

            #             # to copter
            #             dir.data = direction
            #             self.get_logger().info(f"[Mission {self.current_obj}] Direction: {self.direction}, Corner: {center_info}")
            #             self.publisher_direction.publish(dir)

            #             # if qr_target == self.current_obj:
            #             #     self.get_logger().info(f"Target {self.current_obj} achieved!")
            #             #     target.data = qr_target
            #             #     self.publisher_target.publish(target)
            #             #     self.get_logger().info(f"Target {target.data}" )
            #             #     self.is_detecting = False

            #         except (ValueError, IndexError) as e:
            #             self.get_logger().warning(f"Error parsing QR data: {e}")

            imshow("QR Code Scanner", frame)
            if waitKey(1) & 0xFF == ord('q'):
                self.is_detecting = False 

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    # def calculate_rotation_degrees(self, corners):
    #     "Calculates the rotation degrees based on the QR code corners."
    #     try:
    #         corners = np.array(corners)
    #         if corners.shape != (4, 2):
    #             return (None, None)

    #         v1 = corners[1] - corners[0]
    #         v2 = corners[3] - corners[0]

    #         v1_normalized = v1 / np.linalg.norm(v1) if np.linalg.norm(v1) > 0 else np.array([1, 0])
    #         v2_normalized = v2 / np.linalg.norm(v2) if np.linalg.norm(v2) > 0 else np.array([0, 1])

    #         x_rotation_degrees = np.degrees(np.arctan2(v1_normalized[1], v1_normalized[0]))
    #         y_rotation_degrees = np.degrees(np.arctan2(v2_normalized[0], v2_normalized[1]))

    #         rotation_degrees = (x_rotation_degrees, y_rotation_degrees)

    #         translation = -corners[0]

    #         return (rotation_degrees, translation)

    #     except (ValueError, ZeroDivisionError):
    #         return (None, None)

    def decode_qr_from_frame(self, frame):
        "Decodes QR code from the frame using QReader."
        qreader_out = QReader().detect_and_decode(image=frame)
        return qreader_out
    
    def detect_center_information(self, frame):
        "Detects the degree of rotation of the QR code in the frame."
        
        detection_results = QReader().detect(image=frame)
        if detection_results:
            for k, v in enumerate(detection_results):
                corner = detection_results[k]["quad_xy"]
                center_x = float(np.mean(corner[:, 0]))
                center_y = float(np.mean(corner[:, 1]))
                center_point = Point(x=center_x, y=center_y, z=0.0)

                polylines(frame, [corner.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)

                for point in corner:
                    x, y = int(point[0]), int(point[1])
                    circle(frame, (x, y), radius=5, color=(255, 0, 0), thickness=-1)

                for i, point in enumerate(corner):
                    x, y = int(point[0]), int(point[1])
                    putText(frame, f"{i}:({x},{y})", (x + 5, y - 5),
                                FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, LINE_AA)

                # self.publisher_center.publish(center_point)

                return center_point

        return None

def main(args=None):
    rclpy.init(args=args)
    qr_drone_service = QRCodeDroneService()
    rclpy.spin(qr_drone_service)
    qr_drone_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

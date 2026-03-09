import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import DetectedObjects, DetectedObject, ObjectClassification, Shape
from geometry_msgs.msg import Pose, Vector3

class LiveObstaclePublisher(Node):
    def __init__(self):
        super().__init__('live_obstacle_publisher')
        self.publisher_ = self.create_publisher(
            DetectedObjects, 
            '/perception/object_recognition/detection/objects', 
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Publishing customized object (Label UNKNOWN) at 10Hz')

    def timer_callback(self):
        msg = DetectedObjects()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # 对应你提供的 header.frame_id

        obj = DetectedObject()
        obj.existence_probability = 1.0

        # --- 1. 分类 (Classification) ---
        # uint8 UNKNOWN = 0
        # uint8 CAR = 1
        # uint8 TRUCK = 2
        # uint8 BUS = 3
        # uint8 TRAILER = 4
        # uint8 MOTORCYCLE = 5
        # uint8 BICYCLE = 6
        # uint8 PEDESTRIAN = 7
        # uint8 ANIMAL = 8
        # uint8 HAZARD = 9 # Defined as an object that can cause danger to autonomous driving
        # uint8 OVER_DRIVABLE = 10 # Defined as an object that can be safely driven over (e.g., leaf)
        # uint8 UNDER_DRIVABLE = 11 # Defined as an object that can be safely driven under (e.g., overpass)

        class_msg = ObjectClassification()
        class_msg.label = 0 
        class_msg.probability = 1.0
        obj.classification.append(class_msg)

        # --- 2. 运动学 (Kinematics) ---

        ###### 单向-单车道
        # obj.kinematics.pose_with_covariance.pose.position.x = 17.029806137084961
        # obj.kinematics.pose_with_covariance.pose.position.y = 186.3141632080078
        # obj.kinematics.pose_with_covariance.pose.position.z = 3.646

        ###### 单向-双车道
        obj.kinematics.pose_with_covariance.pose.position.x = 14.029806137084961
        obj.kinematics.pose_with_covariance.pose.position.y = 180.3141632080078
        obj.kinematics.pose_with_covariance.pose.position.z = 3.646

    
        # 严格复制你提供的四元数
        obj.kinematics.pose_with_covariance.pose.orientation.x = 0.0
        obj.kinematics.pose_with_covariance.pose.orientation.y = 0.0
        obj.kinematics.pose_with_covariance.pose.orientation.z = 0.0
        obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0

        # --- 3. 协方差 (Covariance) ---
        # 这是一个 6x6 的矩阵，展平为 36 个元素的数组
        # 你的数据中主要在对角线有值 (0.0009 和 0.0076)
        # 这一步非常重要，否则 Tracker 可能会认为数据不可靠
        covariance = [0.0] * 36
        covariance[0] = 0.0009  # x 轴方差
        covariance[7] = 0.0009  # y 轴方差
        covariance[14] = 0.0009 # z 轴方差
        covariance[35] = 0.0076 # yaw 角方差 (对应索引 35)
        
        obj.kinematics.pose_with_covariance.covariance = covariance

        # 速度 (Twist) - 你的数据中全是 0.0
        obj.kinematics.twist_with_covariance.twist.linear.x = 0.0
        
        # --- 4. 形状 (Shape) ---
        # Type 1 代表 BOUNDING_BOX
        try:
            obj.shape.type = Shape.BOUNDING_BOX
        except AttributeError:
            obj.shape.type = 1
            
        obj.shape.dimensions.x = 5.0    
        obj.shape.dimensions.y = 3.0
        obj.shape.dimensions.z = 2.0

        msg.objects.append(obj)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LiveObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

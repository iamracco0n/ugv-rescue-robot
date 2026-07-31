import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from rclpy.qos import qos_profile_sensor_data

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # 하드웨어 제원 (6륜 스키드 스티어링)
        self.WHEEL_RADIUS = 0.0420
        self.PHYSICAL_TRACK_WIDTH = 0.1625
        self.SLIP_FACTOR = 1.45
        self.TRACK_WIDTH = self.PHYSICAL_TRACK_WIDTH * self.SLIP_FACTOR
        self.TICKS_PER_REV = 3960.0
        self.M_PER_TICK = (2.0 * math.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REV
        self.INV_L = 1.0
        self.INV_R = -1.0

        # 센서 데이터 구독 (바퀴 엔코더, 포탑 엔코더, 리얼센스 IMU)
        self.sub = self.create_subscription(Int32MultiArray, 'wheel_encoders', self.encoder_callback, qos_profile_sensor_data)
        self.turret_sub = self.create_subscription(Int32MultiArray, 'turret_fb', self.turret_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, 'camera/imu', self.imu_callback, qos_profile_sensor_data)
        
        # 상태 데이터 발행 (로컬 오도메트리 및 관절 상태)
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 50)
        self.joint_pub = self.create_publisher(JointState, 'measured_joint_states', 10)

        # 차체 오도메트리 상태 변수
        self.last_ticks = None
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.chassis_v_th = 0.0
        
        # 포탑 센서 퓨전 상태 변수
        self.turret_yaw = 0.0
        self.turret_pitch = 0.0
        self.fused_turret_yaw = 0.0
        self.last_imu_time = None
        self.COMP_ALPHA = 0.98
        
        # 조인트 상태 발행 타이머
        self.joint_timer = self.create_timer(0.1, self.joint_timer_callback)
        self.get_logger().info('Raw Wheel Odometry & Turret Node Started')

    def turret_callback(self, msg):
        if len(msg.data) >= 2:
            self.turret_yaw = msg.data[0] / 1000.0
            self.turret_pitch = msg.data[1] / 1000.0

    def imu_callback(self, msg):
        current_time = self.get_clock().now()

        if self.last_imu_time is None:
            self.last_imu_time = current_time
            self.fused_turret_yaw = self.turret_yaw
            return

        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        if dt <= 0: return

        self.last_imu_time = current_time

        # 포탑 상대 회전속도 산출 (전체 자이로 회전 - 차체 회전)
        relative_gyro_z = msg.angular_velocity.z - self.chassis_v_th

        # 1D 상보 필터: 고주파(자이로 동적 변화) + 저주파(엔코더 절대 기준점)
        self.fused_turret_yaw = self.COMP_ALPHA * (self.fused_turret_yaw + relative_gyro_z * dt) + (1.0 - self.COMP_ALPHA) * self.turret_yaw

    def encoder_callback(self, msg):
        if len(msg.data) < 6: return
        
        current_time = self.get_clock().now()
        
        if self.last_ticks is None:
            self.last_ticks = list(msg.data)
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return 

        d_l_front = (msg.data[0] - self.last_ticks[0]) * self.M_PER_TICK * self.INV_L
        d_l_mid   = (msg.data[1] - self.last_ticks[1]) * self.M_PER_TICK * self.INV_L
        d_l_rear  = (msg.data[2] - self.last_ticks[2]) * self.M_PER_TICK * self.INV_L
        d_left = (d_l_front + d_l_mid + d_l_rear) / 3.0

        d_r_front = (msg.data[3] - self.last_ticks[3]) * self.M_PER_TICK * self.INV_R
        d_r_mid   = (msg.data[4] - self.last_ticks[4]) * self.M_PER_TICK * self.INV_R
        d_r_rear  = (msg.data[5] - self.last_ticks[5]) * self.M_PER_TICK * self.INV_R
        d_right = (d_r_front + d_r_mid + d_r_rear) / 3.0

        self.last_ticks = list(msg.data)
        self.last_time = current_time

        d_center = (d_left + d_right) / 2.0
        d_th = (d_right - d_left) / self.TRACK_WIDTH

        v_x = d_center / dt
        v_th = d_th / dt
        self.chassis_v_th = v_th 

        self.x += d_center * math.cos(self.th + (d_th / 2.0))
        self.y += d_center * math.sin(self.th + (d_th / 2.0))
        self.th += d_th

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = get_quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.1
        
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_th
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

    def joint_timer_callback(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['front_left_wheel_joint', 'front_right_wheel_joint', 'mid_left_wheel_joint', 
                   'mid_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint', 
                   'turret_yaw_joint', 'turret_pitch_joint']
        # 추정 각도(fused_turret_yaw)를 발행
        js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.fused_turret_yaw, self.turret_pitch]
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

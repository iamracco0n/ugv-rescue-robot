import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

class TeleopJoyNode(Node):
    def __init__(self):
        super().__init__('teleop_joy_node')
        
        self.chassis_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.turret_pub = self.create_publisher(Vector3, 'turret_manual_cmd', 10)
        
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.deadzone = 0.05
        self.get_logger().info('A.R.G.U.S. Rescue Teleop Node Started')

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        twist = Twist()
        linear_x = self.apply_deadzone(msg.axes[1])
        angular_z = self.apply_deadzone(msg.axes[0])
        
        twist.linear.x = linear_x * 0.5
        twist.angular.z = angular_z * 1.0
        self.chassis_pub.publish(twist)
        
        turret_msg = Vector3()
        turret_x = self.apply_deadzone(msg.axes[3])
        turret_y = self.apply_deadzone(msg.axes[4])
        
        turret_msg.x = turret_x * 200.0
        turret_msg.y = turret_y * 150.0
        
        if msg.buttons[5] > 0:
            turret_msg.z = 1.0
        else:
            turret_msg.z = 0.0
            
        self.turret_pub.publish(turret_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

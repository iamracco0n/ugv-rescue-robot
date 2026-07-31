import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
A.R.G.U.S. Rescue Keyboard Teleop
-----------------------------------
Chassis: W A S D
Turret:  I J K L
Center:  O
Stop:    Q
Exit:    CTRL-C
"""

key_states = {
    'w': False, 'a': False, 's': False, 'd': False,
    'i': False, 'j': False, 'k': False, 'l': False,
    'o': False, 'q': False
}

def set_terminal_settings():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old_settings

def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def get_keys():
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    keys = []
    if rlist:
        data = sys.stdin.read(1)
        keys.append(data)
    return keys

def main(args=None):
    old_settings = set_terminal_settings()
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_keyboard_node')

    chassis_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    yaw_pub = node.create_publisher(Float64, '/turret_yaw_cmd', 10)
    pitch_pub = node.create_publisher(Float64, '/turret_pitch_cmd', 10)

    is_stopped = False
    current_linear = 0.0
    current_angular = 0.0

    print(msg)

    try:
        while rclpy.ok():
            keys = get_keys()

            if '\x03' in keys:
                break

            for k in key_states.keys():
                key_states[k] = False

            for key in keys:
                key = key.lower()
                if key in key_states:
                    key_states[key] = True

            if key_states['q']:
                is_stopped = True
            elif any(key_states[k] for k in ['w', 'a', 's', 'd', 'i', 'j', 'k', 'l']):
                is_stopped = False

            if not is_stopped:
                target_linear = 0.0
                target_angular = 0.0
                target_yaw = 0.0
                target_pitch = 0.0

                if key_states['w']: target_linear += 0.5
                if key_states['s']: target_linear -= 0.5
                if key_states['a']: target_angular += 1.0
                if key_states['d']: target_angular -= 1.0

                if key_states['i']: target_pitch += 1.0
                if key_states['k']: target_pitch -= 1.0
                if key_states['j']: target_yaw += 1.0
                if key_states['l']: target_yaw -= 1.0
                if key_states['o']: 
                    target_yaw = 0.0
                    target_pitch = 0.0

                current_linear += (target_linear - current_linear) * 0.5
                current_angular += (target_angular - current_angular) * 0.5

                t_msg = Twist()
                t_msg.linear.x = current_linear
                t_msg.angular.z = current_angular
                chassis_pub.publish(t_msg)

                y_msg = Float64()
                y_msg.data = target_yaw
                yaw_pub.publish(y_msg)

                p_msg = Float64()
                p_msg.data = target_pitch
                pitch_pub.publish(p_msg)

    except Exception as e:
        pass
    finally:
        chassis_pub.publish(Twist())
        yaw_pub.publish(Float64())
        pitch_pub.publish(Float64())
        restore_terminal_settings(old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

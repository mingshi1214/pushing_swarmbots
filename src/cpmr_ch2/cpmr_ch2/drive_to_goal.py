import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def deg_wrap(deg): 
    deg = deg%(2*math.pi)
    return deg 

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_t = 0.0
        self._vel_gain = 50.0
        self._max_vel = 0.4

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._goal_x)
        self.declare_parameter('goal_y', value=self._goal_y)
        self.declare_parameter('goal_t', value=self._goal_t)
        self.declare_parameter('max_vel', value=self._max_vel)
        self.declare_parameter('vel_gain', value=self._vel_gain)

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)


    def _listener_callback(self, msg, max_pos_err=0.05):
        pose = msg.pose.pose

        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        cur_t = yaw
        t_diff = self._goal_t - cur_t 
        
        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        twist = Twist()
        if dist > max_pos_err or t_diff > max_pos_err:
            # x = max(min(x_diff * self._vel_gain, self._max_vel), -self._max_vel)
            # y = max(min(y_diff * self._vel_gain, self._max_vel), -self._max_vel)
            x = x_diff * self._vel_gain
            y = y_diff * self._vel_gain
            t_x = x * math.cos(cur_t) + y * math.sin(cur_t)
            t_y = -x * math.sin(cur_t) + y * math.cos(cur_t)
            
            # don't need to check for max vel now. Will always scale to 0.4 mag
            vel_mag = np.sqrt((t_x)**2 + (t_y)**2)
            vel_p_diff = (self._max_vel - vel_mag)/vel_mag
            if vel_mag > self._max_vel:
                x = t_x * (1 + vel_p_diff)
                y = t_y * (1 + vel_p_diff)
            else:
                x = t_x
                y = t_y

            curr_mag = np.sqrt((x)**2 + (y)**2)
            self.get_logger().info(f"vel mag {vel_mag}, vel dif {vel_p_diff}, x {x}, y {y}, t_x {t_x}, t_y {t_y}, curr mag {curr_mag}")

            t = max(min(t_diff * self._vel_gain, self._max_vel), -self._max_vel)
            twist.linear.x = x 
            twist.linear.y = y 
            twist.angular.z = t
            self.get_logger().info(f"at ({cur_x},{cur_y},{cur_t}) goal ({self._goal_x},{self._goal_y},{self._goal_t})")
        self._publisher.publish(twist)

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_x = param.value
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_y = param.value
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_t = param.value
            elif param.name == 'max_vel' and param.type_ == Parameter.Type.DOUBLE:
                self._max_vel = param.value
            elif param.name == 'vel_gain' and param.type_ == Parameter.Type.DOUBLE:
                self._vel_gain = param.value
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().warn(f"Changing goal {self._goal_x} {self._goal_y} {self._goal_t}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


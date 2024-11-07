import math
import sys
import numpy as np
import json
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from .add_obstacle import make_obstacle

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
        self._vel_gain = 3.0
        self._max_vel = 0.4

        self._rob_radius = 0.1
        self._safety_thresh = 0.15
        self._pose_allowance = 0.01
        self._map = 'ass_map.json'
        self._obs = []
        self._cur_obs = []
        self._t = None
        self._cnt = 0
        
        self.state = 0

        self._obs_recieved = False
        self._way_reached = False
        self._goal_reached = False

        self._cur_x = 0
        self._cur_y = 0
        self._cur_t = 0

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._goal_x)
        self.declare_parameter('goal_y', value=self._goal_y)
        self.declare_parameter('goal_t', value=self._goal_t)
        self.declare_parameter('max_vel', value=self._max_vel)
        self.declare_parameter('vel_gain', value=self._vel_gain)
        self.declare_parameter('map', value=self._map)

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

    def _listener_callback(self, msg):
        pose = msg.pose.pose
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        self._cur_t = yaw
        self.get_logger().info(f'robot at {self._cur_x}, {self._cur_y}, {self._cur_t}')
        
        
        ####################################
        # this is really bad....
        twist = Twist()
        
        if self.state == 1:
            # obs avoidance mode 1:
            # stop and turn 90 degrees and  then follow edge
            if not self._way_reached:
                # calc the angle perpendicular to the vector from the center of the cylinder to the robot
                self.get_logger().info(f'state 1. Turning ')
                if self._t == None:
                #only calc once: 
                    self._t = np.arctan((self._cur_obs[1]-self._cur_y)/(self._cur_obs[0]-self._cur_x)) + np.pi/2
                twist = self.go_to_waypoint([self._cur_x, self._cur_y, self._t])
                # self._publisher.publish(twist)
            else:
                self.get_logger().info(f'state 1. done')
                self.state = 2
                self._way_reached = False
                self.t = None
        elif self.state == 2:
            # obs avoidance mode 2:
            self.get_logger().info(f'state 2. edge following')
            twist = self.follow_edge()
            # self._publisher.publish(twist)
            if not self.check_collision_to_goal():
                self.get_logger().info(f'state 2. done')
                self.state = 0
                self._cnt = 0

        elif self.state == 0:
            # state 0. Default go to goal position
            if self._cnt < 100:
                self._cnt += 1
            else:
                self.check_collision_to_obs()
            self.get_logger().info(f'state 0. going to goal')
            if not self._way_reached:
                twist = self.go_to_waypoint([self._goal_x, self._goal_y, self._goal_t])
                # self._publisher.publish(twist)
            else:
                self._way_reached = False
                self._goal_reached = True
                self.state = 4
                self.get_logger().info(f'current position {self._cur_x}, {self._cur_y}, {self._cur_t} at goal {self._goal_x}, {self._goal_y}, {self._goal_t}')
                self._publisher.publish(twist)
        else:
            self.get_logger().info(f'state 4. goal reached')
        
        return
        
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
            elif param.name == 'map' and param.type_ == Parameter.Type.STRING:
                map_name = param.value
                self._map = map_name
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().warn(f"Changing goal {self._goal_x} {self._goal_y} {self._goal_t}")
            self.get_logger().info(f"map name {self._map}")
        return SetParametersResult(successful=True)
    
    def go_to_waypoint(self, pose, max_pos_err=0.05):
        # return x, y, t for twist command
        x_diff = pose[0] - self._cur_x
        y_diff = pose[1] - self._cur_y
        t_diff = pose[2] - self._cur_t 
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)

        twist = Twist()
        if dist > max_pos_err:
            x = max(min(x_diff * self._vel_gain, self._max_vel), -self._max_vel)
            y = max(min(y_diff * self._vel_gain, self._max_vel), -self._max_vel)
            t_x = x * math.cos(self._cur_t) + y * math.sin(self._cur_t)
            t_y = -x * math.sin(self._cur_t) + y * math.cos(self._cur_t)
            
            # don't need to check for max vel now. Will always scale to 0.4 mag
            vel_mag = np.sqrt((t_x)**2 + (t_y)**2)
            vel_p_diff = (0.4 - vel_mag)/vel_mag
            x = t_x * (1 + vel_p_diff)
            y = t_y * (1 + vel_p_diff)
            curr_mag = np.sqrt((x)**2 + (y)**2)

            twist.linear.x = x 
            twist.linear.y = y 
            self.get_logger().info(f"in dist")
            # self.get_logger().info(f"x {x}, y {y}, curr mag {curr_mag}")

        if abs(t_diff) > max_pos_err/2:
            t = max(min(t_diff * self._vel_gain, self._max_vel), -self._max_vel)
            if twist.linear.x == 0 and twist.linear.y == 0:
                vel_p_diff = (4 - t)/t
            else:
                vel_p_diff = (0.7 - t)/t
            twist.angular.z = t*(1+vel_p_diff)
            self.get_logger().info(f"in z")
        
        self.get_logger().info(f"at ({self._cur_x},{self._cur_y},{self._cur_t}) goal ({pose[0]},{pose[1]},{pose[2]})")
        self.get_logger().info(f"vels: {twist.linear.x}, {twist.linear.y}, {twist.angular.z}")

        self._publisher.publish(twist)

        if abs(dist) <= max_pos_err and abs(t_diff) <= max_pos_err:
            self._way_reached = True
            self.get_logger().info(f'current position {self._cur_x}, {self._cur_y}, {self._cur_t} at goal {pose}')
        
        return twist

    def check_collision_to_goal(self):
        # Ax + By + C = 0 line from current position to goal
        # check for collisions witht the current obstacle
        a = -(self._goal_y-self._cur_y)/(self._goal_x-self._cur_x)
        b = 1
        c = -(self._goal_y + a*self._goal_x)

        d = abs(a*self._cur_obs[0] + b*self._cur_obs[1] + c)/np.sqrt(a**2 + b**2)
        
        if d <= (self._cur_obs[2] + self._rob_radius + self._safety_thresh + self._pose_allowance):
            self.get_logger().info(f'current position {self._cur_x}, {self._cur_y}, {self._cur_t} collides with {self._cur_obs} if going to goal')
            return True
        return False
    
    def dist_to_obs(self, obs):
        d = abs(np.sqrt((obs[0]-self._cur_x)**2 + (obs[1] - self._cur_y)**2)) - (obs[2] + self._rob_radius)
        return d

    def check_collision_to_obs(self):
        for obs in self._obs:
            if self.dist_to_obs(obs) <= (self._safety_thresh + self._pose_allowance):
                self.state = 1
                self._cur_obs = obs
                self.get_logger().info(f'current position {self._cur_x}, {self._cur_y}, {self._cur_t} at obs {obs}')
                return True
        return False
    
    def follow_edge(self):
        twist = Twist()
        d_o = np.sqrt((self._cur_obs[1])**2 + (self._cur_obs[0])**2)
        d_r = np.sqrt((self._cur_y)**2 + (self._cur_x)**2)

        # rel to origin
        if d_o > d_r:
            factor = -1
        else:
            factor = 1
        
        omega = 0.4/(self._cur_obs[2] + self._safety_thresh + self._pose_allowance)
        twist.linear.x = 0.22
        twist.angular.z = factor * omega
        self._publisher.publish(twist)
        return twist

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    package_path = get_package_share_directory('cpmr_ch2')
    try:
        with open(f"{package_path}/{node._map}") as fd:
            map = json.load(fd)
    except Exception as e:
        node.get_logger().info(f"Unable to find/parse map in {package_path}/{map}")
        sys.exit(1)

    for o in map.keys():
        node.get_logger().info(f"Populating map with {map[o]}")
        make_obstacle(node, o, map[o]['x'], map[o]['y'], 1.0, map[o]['r'])
        node._obs.append([map[o]['x'], map[o]['y'], map[o]['r']])
        node._obs_recieved = True

    try:
        # node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


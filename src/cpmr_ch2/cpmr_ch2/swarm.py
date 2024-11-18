import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetEntityState
from enum import Enum
import time

class FSM_STATES(Enum):
    START_TRANS = 'Start Translation',
    PUSHING_TRANS = 'Pushing to translation goal',
    FOLLOWING_TRANS = 'Following the box translation',
    START_ROT = 'Start Rotation',
    PUSHING_ROT = 'Pushing to rotation goal',
    FOLLOWING_ROT = 'Following the box rotation',
    TASK_DONE = 'Task Done'


class pose():
    def __init__(self, x0=0.0, y0=0.0, t0=0.0):
        self.x = x0
        self.y = y0
        self.t = t0

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

def distance(x0, y0, x1, y1):
    x_diff = x1 - x0
    y_diff = y1 - y0
    dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
    return dist

class SwarmRobot(Node):

    Rel_Pose_Pushing_Threshold = 0.06 #m
    Object_Trans_Goal_Threshold = 0.2 #m
    Object_Rot_Goal_Threshold = 0.1 #rads


    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self._goal = pose()
        self._robot_goal = pose()
        self._vel_gain = 50.0
        self._max_vel = 0.4

        self._pose = pose()
        self._pose_rel_box = pose()
        self._cur_state = FSM_STATES.START_TRANS
        self._start_time = self.get_clock().now().nanoseconds * 1e-9

        self._box = pose(2.0, 2.0, 0.0)
        self._box_init = pose(self._box.x, self._box.y, self._box.t)

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._goal.x)
        self.declare_parameter('goal_y', value=self._goal.y)
        self.declare_parameter('goal_t', value=self._goal.t)
        self.declare_parameter('max_vel', value=self._max_vel)
        self.declare_parameter('vel_gain', value=self._vel_gain)

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self._cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self._req = GetEntityState.Request()
        self._client_futures = []

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal.x = param.value
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal.y = param.value
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE:
                self._goal.t = param.value
            elif param.name == 'max_vel' and param.type_ == Parameter.Type.DOUBLE:
                self._max_vel = param.value
            elif param.name == 'vel_gain' and param.type_ == Parameter.Type.DOUBLE:
                self._vel_gain = param.value
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().warn(f"Changing goal {self._goal.x} {self._goal.y} {self._goal.t}")
        return SetParametersResult(successful=True)

    def _listener_callback(self, msg):
        # listen to odom 
        # this is essentially the main loop
        # assigns odom and box pose and goes to state machine
        pose = msg.pose.pose
        # self.get_logger().info(f'in listener')

        self._pose.x = pose.position.x
        self._pose.y = pose.position.y
        o = pose.orientation
        roll, pitch, self._pose.t = euler_from_quaternion(o)

        # weird ros2 service client
        self._req.name = 'unit_box'
        self._client_futures.append(self._cli.call_async(self._req))
        incomplete_futures = []
        resp = None
        for f in self._client_futures:
            if f.done():
                resp = f.result()
            else:
                incomplete_futures.append(f)
        self._client_futures = incomplete_futures

        if resp != None:
            box_pose = resp.state.pose
            self._box.x = box_pose.position.x
            self._box.y = box_pose.position.y
            o = box_pose.orientation
            roll, pitch, self._box.t = euler_from_quaternion(o)

        # self.get_logger().info(f'going to state machine')
        self._state_machine()

    def _state_machine(self):
        if self._cur_state == FSM_STATES.START_TRANS:
            self._do_state_start_trans()
        elif self._cur_state == FSM_STATES.PUSHING_TRANS:
            self._do_state_pushing_trans()
        elif self._cur_state == FSM_STATES.FOLLOWING_TRANS:
            self._do_state_following_trans()
        elif self._cur_state == FSM_STATES.START_ROT:
            self._do_state_start_rot()
        elif self._cur_state == FSM_STATES.PUSHING_ROT:
            self._do_state_pushing_rot()
        elif self._cur_state == FSM_STATES.FOLLOWING_ROT:
            self._do_state_following_rot()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        else:
            self.get_logger().info(f'{self.get_name()} bad state {self._cur_state}')

    def _do_state_start_trans(self):
        # determine if they will push or if they will follow
        # if distance from self to goal > dist of box to goal then we push
        # else we follow
        # sleep for a bit
        time.sleep(5.0)

        # set robot goal pose
        self._robot_goal.x = self._pose.x + self._goal.x
        self._robot_goal.y = self._pose.y + self._goal.y

        d_self2goal = distance(self._pose.x, self._pose.y,  self._robot_goal.x, self._robot_goal.y)
        d_box2goal = distance(self._box.x, self._box.y, self._robot_goal.x, self._robot_goal.y)

        # set initial distance from box and maintain it
        self._pose_rel_box.x = self._box.x - self._pose.x
        self._pose_rel_box.y = self._box.y - self._pose.y

        if d_self2goal>d_box2goal:
            self.get_logger().info(f"I am pushing. me2goal: {d_self2goal}, box2goal: {d_box2goal}")
            self._cur_state = FSM_STATES.PUSHING_TRANS
        else:
            self.get_logger().info(f"I am following. me2goal: {d_self2goal}, box2goal: {d_box2goal}")
            self._cur_state = FSM_STATES.FOLLOWING_TRANS
            time.sleep(1.0) # wait so that the pushing can happen first
        return
    
    def _do_state_pushing_trans(self):
        if ((abs(self._box.x - (self._box_init.x + self._goal.x)) <= SwarmRobot.Object_Trans_Goal_Threshold) and 
            (abs(self._box.y - (self._box_init.y + self._goal.y)) <= SwarmRobot.Object_Trans_Goal_Threshold)):
            self._cur_state = FSM_STATES.START_ROT
            return
        
        # check if we're too far from our initial relative position
        curr_pose_rel_box = pose()
        curr_pose_rel_box.x = self._box.x - self._pose.x
        curr_pose_rel_box.y = self._box.y - self._pose.y

        if ((abs(curr_pose_rel_box.x - self._pose_rel_box.x) > SwarmRobot.Rel_Pose_Pushing_Threshold) or 
            (abs(curr_pose_rel_box.y - self._pose_rel_box.y) > SwarmRobot.Rel_Pose_Pushing_Threshold)):
            #get the correct pose on the box
            maintained_pose = pose()
            maintained_pose.x = self._box.x - self._pose_rel_box.x
            maintained_pose.y = self._box.y - self._pose_rel_box.y

            #ignore the default max pose error because we want to be precise on the box for pushing
            self._drive_to_goal(maintained_pose, 0.0)
        else:
            # push the box
            self._drive_to_goal(self._robot_goal)

        return
    
    def _do_state_following_trans(self):
        if ((abs(self._box.x - (self._box_init.x + self._goal.x)) <= SwarmRobot.Object_Trans_Goal_Threshold) and 
            (abs(self._box.y - (self._box_init.y + self._goal.y)) <= SwarmRobot.Object_Trans_Goal_Threshold)):
            self._cur_state = FSM_STATES.START_ROT
            return
        
        # follow the box
        # set new goals depending on where the box is
        temp_goal = pose()
        # self.get_logger().info(f"I am following. initial goal:{self._goal.x} {self._goal.y}")
        temp_goal.x = self._box.x - self._pose_rel_box.x
        temp_goal.y = self._box.y - self._pose_rel_box.y
        # self.get_logger().info(f"I am following. new goal:{self._goal.x} {self._goal.y}")
        self._drive_to_goal(temp_goal)
        return
    
    def _do_state_start_rot(self):
        # stop motion while we transition to next state
        self._cmd_vel_pub.publish(Twist())

        self.get_logger().info("translation complete, switching to rotation")

        self._cur_state = FSM_STATES.PUSHING_ROT

    def _do_state_pushing_rot(self):
        if(abs(self._box.t - (self._box_init.t + self._goal.t)) <= SwarmRobot.Object_Rot_Goal_Threshold):
            self._cur_state = FSM_STATES.TASK_DONE

        # check if we're too far from our initial relative position
        curr_pose_rel_box = pose()
        curr_pose_rel_box.x = self._box.x - self._pose.x
        curr_pose_rel_box.y = self._box.y - self._pose.y

        # rotate our initial position based on the box's rotation
        rotated_rel_pose = self._rotate_point_about_origin(self._pose_rel_box, self._box.t)

        if ((abs(curr_pose_rel_box.x - rotated_rel_pose.x) > SwarmRobot.Rel_Pose_Pushing_Threshold) or 
            (abs(curr_pose_rel_box.y - rotated_rel_pose.y) > SwarmRobot.Rel_Pose_Pushing_Threshold)):
            #get the correct pose on the box
            maintained_pose = rotated_rel_pose
            maintained_pose.x = self._box.x - maintained_pose.x
            maintained_pose.y = self._box.y - maintained_pose.y

            #ignore the default max pose error because we want to be precise on the box for pushing
            self._drive_to_goal(maintained_pose, 0.0)
        else:
            # push the box
            temp = self._rotate_point_about_origin(self._pose_rel_box, self._goal.t)

            temp.x = self._box.x - temp.x
            temp.y = self._box.y - temp.y

            self._drive_to_goal(temp)

    def _do_state_following_rot(self):
        pass
    
    def _do_state_task_done(self):
        # make sure motion is stopped
        self._cmd_vel_pub.publish(Twist())
        # task is done. shut down
        # self.get_logger().info(f'{self.get_name()} task complete')

        return

    def _drive_to_goal(self, goal, max_pos_err=0.05):
        t_diff = goal.t - self._pose.t 
        
        x_diff = goal.x - self._pose.x
        y_diff = goal.y - self._pose.y
        dist = distance(self._pose.x, self._pose.y, goal.x, goal.y)

        twist = Twist()
        if dist > max_pos_err:
            # always scale to max_vel
            mag = np.sqrt((x_diff)**2 + (y_diff)**2)
            x = x_diff/mag * self._max_vel
            y = y_diff/mag * self._max_vel

            curr_mag = np.sqrt((x)**2 + (y)**2)
            # self.get_logger().info(f"vel mag {mag}, x {x}, y {y}, curr mag {curr_mag}")

            twist.linear.x = x 
            twist.linear.y = y 
            
            # self.get_logger().info(f"at ({self._pose.x},{self._pose.y},{self._pose.t}) goal ({self._goal.x},{self._goal.y},{self._goal.t})")  
        # if abs(t_diff) > 0.1:
        #     twist.angular.z = t_diff/abs(t_diff) * 0.2
        self._cmd_vel_pub.publish(twist)
        return
    
    def _rotate_point_about_origin(self, point, radians):
        adjusted = pose()
        adjusted.x = (point.x * math.cos(radians)) - (point.y * math.sin(radians))
        adjusted.y = (point.x * math.sin(radians)) + (point.y * math.cos(radians))

        return adjusted
    
def main(args=None):
    
    rclpy.init(args=args)
    node = SwarmRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Bool
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelStates
from enum import Enum
import time

class FSM_STATES(Enum):
    START_TRANS = 'Start Translation',
    PUSHING_TRANS = 'Pushing to translation goal',
    FOLLOWING_TRANS = 'Following the box translation',
    START_ROT = 'Start Rotation',
    PUSHING_ROT = 'Pushing to rotation goal',
    FOLLOWING_ROT = 'Following the box rotation',
    WAYPOINT_DONE = 'Waypoint Done',
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

def assign_waypoints(x, y, t):
    # make waypoints a list of pose objects [pose0, pose1, ...]
    waypoints = []
    for i in range(len(x)):
        waypoints.append(pose(x[i], y[i], t[i]))
    return waypoints


class SwarmRobot(Node):

    Rel_Pose_Pushing_Threshold = 0.07 #m
    Rob_Pose_Rot_Pose_Threshold = 0.06 #m
    Object_Trans_Goal_Threshold = 0.1 #m
    Object_Rot_Goal_Threshold = 0.2 #rads


    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self._goal = pose()
        self._driving_goal = pose() # only used for diagnosis for now
        self._robot_goal = pose()
        self._vel_gain = 50.0
        self._max_vel = 0.4

        self._pose = pose()
        self._pose_rel_box = pose()
        self._cur_state = FSM_STATES.START_TRANS
        self._start_time = self.get_clock().now().nanoseconds * 1e-9

        self._box = pose(2.0, 2.0, 0.0)
        self._box_init = pose(self._box.x, self._box.y, self._box.t)

        self._is_adjust = False

        self._all_at_waypoint = False

        self._x_list, self._y_list, self._t_list = None, None, None
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._x_list)
        self.declare_parameter('goal_y', value=self._y_list)
        self.declare_parameter('goal_t', value=self._t_list)
        self.declare_parameter('max_vel', value=self._max_vel)
        self.declare_parameter('vel_gain', value=self._vel_gain)
        self._waypoints = assign_waypoints(self._x_list, self._y_list, self._t_list)
        self._get_next_waypoint()

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._listener_callback, 5)
        self._obj_sub = self.create_subscription(ModelStates, "/gazebo/model_states", self._box_callback, 1)
        self._swarm_waypoints_sub = self.create_subscription(Bool, "/all_same", self._waypoints_callback, 1) # listening to see if everyone is at their positions for the current waypoint
        
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 5)
        self._diagnosis_pub = self.create_publisher(String, "/diagnosis", 5)
        self._waypoints_pub = self.create_publisher(Int32, "/waypoints", 5)

        self._cnt = 0

        self._cmd_vel_pub.publish(Twist())

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                self._x_list = param.value
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                self._y_list = param.value
            elif param.name == 'goal_t' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                self._t_list = param.value
            elif param.name == 'max_vel' and param.type_ == Parameter.Type.DOUBLE:
                self._max_vel = param.value
            elif param.name == 'vel_gain' and param.type_ == Parameter.Type.DOUBLE:
                self._vel_gain = param.value
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().warn(f"Changing goal {self._goal.x} {self._goal.y} {self._goal.t}")
        return SetParametersResult(successful=True)
    
    def _waypoints_callback(self, msg):
        self._all_at_waypoint = msg.data

    def _box_callback(self, msg):
        objs = msg.name
        i = objs.index('unit_box')
        box_pose = msg.pose[i]
        self._box.x = box_pose.position.x
        self._box.y = box_pose.position.y
        o = box_pose.orientation
        roll, pitch, self._box.t = euler_from_quaternion(o)

        # self.get_logger().info(f'box: {round(self._box.x,3)} {round(self._box.y, 3)} {round(self._box.t,3)}')

    def _listener_callback(self, msg):
        # listen to odom 
        # this is essentially the main loop
        # assigns odom and box pose and goes to state machine
        pose = msg.pose.pose

        self._pose.x = pose.position.x
        self._pose.y = pose.position.y
        o = pose.orientation
        roll, pitch, self._pose.t = euler_from_quaternion(o)

        # self.get_logger().info(f'going to state machine')
        way = Int32()
        way.data = len(self._waypoints)
        self._waypoints_pub.publish(way)
        self._diagnosis()
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
        elif self._cur_state == FSM_STATES.WAYPOINT_DONE:
            self._do_state_waypoint_done()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        else:
            self.get_logger().info(f'{self.get_name()} bad state {self._cur_state}')

    def _do_state_start_trans(self):
        # determine if they will push or if they will follow
        # if distance from self to goal > dist of box to goal then we push
        # else we follow
        # sleep for a bit
        self._cmd_vel_pub.publish(Twist())
        time.sleep(5.0)

        # set initial distance from box and maintain it
        self._pose_rel_box.x = self._box.x - self._pose.x
        self._pose_rel_box.y = self._box.y - self._pose.y

        # set robot goal pose
        self._robot_goal.x = self._pose.x + self._goal.x
        self._robot_goal.y = self._pose.y + self._goal.y

        d_self2goal = distance(self._pose.x, self._pose.y,  self._robot_goal.x, self._robot_goal.y)
        d_box2goal = distance(self._box.x, self._box.y, self._robot_goal.x, self._robot_goal.y)

        #TODO maybe worth it to have it be able to transition between pushing and following? 
        # (well if they do it right the first time, not necessary)
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
            self._is_adjust = True

            #ignore the default max pose error because we want to be precise on the box for pushing
            self._drive_to_goal(maintained_pose, 0.0)
        else:
            # push the box
            self._is_adjust = False
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
        # moved here. We want to check if our goal location has been hit for rotation instead of the box.
        # sometimes box is rotated but we are not there and it messes up the next waypoint start position
        temp = self._rotate_point_about_origin(self._pose_rel_box, self._goal.t)
        temp.x = self._box.x - temp.x
        temp.y = self._box.y - temp.y
        
        if ((abs(self._pose.x - temp.x) <= SwarmRobot.Rob_Pose_Rot_Pose_Threshold) and 
            (abs(self._pose.y - temp.y) <= SwarmRobot.Rob_Pose_Rot_Pose_Threshold)):
            # get next waypoint before saying the waypoint is done so we wait for everyone else to be ready for the next one.
            # this lessens length of our waypoint list so that everyone else has to be done their current waypoint before 
            # all_at_waypoint is true
            self.get_logger().info(f"rotation complete, switching to waiting for everyone else: {self._all_at_waypoint}")
            self._cur_state = FSM_STATES.WAYPOINT_DONE
            return

        # check if we're too far from our initial relative position
        curr_pose_rel_box = pose()
        curr_pose_rel_box.x = self._box.x - self._pose.x
        curr_pose_rel_box.y = self._box.y - self._pose.y

        # rotate our initial position based on the box's rotation
        # initial pose ref frame
        rotated_rel_pose = self._rotate_point_about_origin(self._pose_rel_box, self._box.t)

        if ((abs(curr_pose_rel_box.x - rotated_rel_pose.x) > SwarmRobot.Rel_Pose_Pushing_Threshold) or 
            (abs(curr_pose_rel_box.y - rotated_rel_pose.y) > SwarmRobot.Rel_Pose_Pushing_Threshold)):
            #get the correct pose on the box
            # TODO: check if this gets triggered on the 2nd waypoint. it seems like no?
            maintained_pose = rotated_rel_pose
            maintained_pose.x = self._box.x - maintained_pose.x
            maintained_pose.y = self._box.y - maintained_pose.y

            self._is_adjust = True

            #ignore the default max pose error because we want to be precise on the box for pushing
            self._drive_to_goal(maintained_pose, 0.0)
        else:
            # push the box
            # TODO: check if this breaks for big rotations (I think assuming going in a line rn? maybe time to do waypoints?)

            self._is_adjust = False

            self._drive_to_goal(temp)

    def _do_state_following_rot(self):
        pass
    
    def _do_state_waypoint_done(self):
        # make sure motion is stopped 
        # wait for others to get to their curr waypoint
        # start up once their waypoint length is same as yours
        # self.get_logger().info(f"waiting for all same to be true: {self._all_at_waypoint}")
        if self._all_at_waypoint: 
            # start up the process again
            # reset the box init position 
            # TODO: bad for resetting box init I think but might take a while to change it
            self._box_init = pose(self._box.x, self._box.y, self._box.t)
            if len(self._waypoints) == 0:
                self._cur_state = FSM_STATES.TASK_DONE
            else:
                # wait for everyone to get the message
                time.sleep(1)
                self._get_next_waypoint() 
                self._cur_state = FSM_STATES.START_TRANS
        else:
            # wait for everyone else
            self._cmd_vel_pub.publish(Twist())
        return
    
    def _do_state_task_done(self):
        # make sure motion is stopped
        self._cmd_vel_pub.publish(Twist())
        # task is done. shut down
        self.get_logger().info(f'{self.get_name()} task complete')

        return

    def _drive_to_goal(self, goal, max_pos_err=0.05):
        self._driving_goal.x = goal.x
        self._driving_goal.y = goal.y
        
        x_diff = goal.x - self._pose.x
        y_diff = goal.y - self._pose.y
        dist = distance(self._pose.x, self._pose.y, goal.x, goal.y)

        twist = Twist()
        if dist > max_pos_err:
            # always scale to max_vel
            mag = np.sqrt((x_diff)**2 + (y_diff)**2)
            x = x_diff/mag * self._max_vel
            y = y_diff/mag * self._max_vel
            
            g0 = np.array([x, y])
            # need to set in current robot frame (above is in robot init frame which is aligned with world frame in terms of rotation)
            # robot rotates when getting dragged along block. do it in rotated robot frame
            r_01 = np.array([[np.cos(self._pose.t), -np.sin(self._pose.t)],
                            [np.sin(self._pose.t), np.cos(self._pose.t)]])
            
            g1 = r_01.T @ g0
            
            g0 = np.array([x, y])
            # need to set in current robot frame (above is in robot init frame which is aligned with world frame in terms of rotation)
            # robot rotates when getting dragged along block. do it in rotated robot frame
            r_01 = np.array([[np.cos(self._pose.t), -np.sin(self._pose.t)],
                            [np.sin(self._pose.t), np.cos(self._pose.t)]])
            
            g1 = r_01.T @ g0

            curr_mag = np.sqrt((x)**2 + (y)**2)
            # self.get_logger().info(f"vel mag {mag}, x {x}, y {y}, curr mag {curr_mag}")

            twist.linear.x = g1[0] 
            twist.linear.y = g1[1]
            
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
    
    def _diagnosis(self):
        diagnosis = String()
        diagnosis.data = f"pose wrt world: {round(self._pose.x, 3)}, {round(self._pose.y, 3)}, {round(self._pose.t, 3)} pose wrt box: {round(self._box.x - self._pose.x, 3)}, {round(self._box.y - self._pose.y, 3)} init pose wrt box: {round(self._pose_rel_box.x, 3)}, {round(self._pose_rel_box.x, 3)} is adjust: {self._is_adjust} box pose: {round(self._box.x, 3)}, {round(self._box.y, 3)}, {round(self._box.t, 3)} driving goal: {round(self._driving_goal.x, 3)}, {round(self._driving_goal.y, 3)}, {round(self._driving_goal.t, 3)}\n"
        
        self._diagnosis_pub.publish(diagnosis)
        return
    
    def _get_next_waypoint(self):
        # pop first index of waypoints only if there are more waypoints to go to
        self.get_logger().info(f"getting next waypoint: {self._waypoints}")
        if len(self._waypoints) != 0:
            self._goal = self._waypoints.pop(0)
        else:
            self._cur_state = FSM_STATES.TASK_DONE
    
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


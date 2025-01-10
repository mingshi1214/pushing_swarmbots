import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Bool, Int32MultiArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelStates
from enum import Enum
import time
from example_interfaces.srv import AddTwoInts
import random

class FSM_STATES(Enum):
    START_TRANS = 'Start Translation',
    REALLOCATING_BF_TRANS = 'Reallocating before translation',
    PUSHING_TRANS = 'Pushing to translation goal',
    FOLLOWING_TRANS = 'Following the box translation',
    REALLOCATING_BF_ROT = 'Reallocating before rotation',
    START_ROT = 'Start Rotation',
    PUSHING_ROT = 'Pushing to rotation goal',
    FOLLOWING_ROT = 'Following the box rotation',
    WAYPOINT_DONE = 'Waypoint Done',
    TASK_DONE = 'Task Done'


class Pose:
    def __init__(self, x0=0.0, y0=0.0, t0=0.0):
        self.x = x0
        self.y = y0
        self.t = t0


class Object:
    def __init__(self, x_coords, y_coords):
        self.init_points = [Pose(coords[0], coords[1], 0.0) for coords in zip(x_coords, y_coords)]
        self.points = self.init_points.copy()

    # returns < 0 if clockwise, > 0 if counter clockwise, and 0 if colinear
    def _is_counter_clockwise(self, a, b, c):
        val = ((c.y - a.y) * (b.x - a.x)) - ((b.y - a.y) * (c.x - a.x))

        # reduce output into 3 possible values
        if val < 0:
            return -1
        elif val > 0:
            return 1
        else:
            return 0
    
    # only works if points are already proven
    def _is_point_in_colinear_segment(self, s1, s2, p):
        return ((p.x >= min(s1.x, s2.x)) and (p.x <= max(s1.x, s2.x)) and (p.y >= min(s1.y, s2.y)) and (p.y <= max(s1.y, s2.y)))

    def _update_points(self, new_pose):
        updated = []
        for coord in self.init_points:
            # rotate about origin then add the translation
            pt = rotate_point_about_origin(coord, new_pose.t)

            # now translate by new pose
            pt.x = pt.x + new_pose.x
            pt.y = pt.y + new_pose.y

            updated.append(pt)
        self.points = updated


    # based on: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    def does_line_intersect_object(self, p1, p2, new_pose):
        p1 = Pose(p1[0], p1[1], 0.0)
        p2 = Pose(p2[0], p2[1], 0.0)
        self._update_points(new_pose)
        c1_points = self.points
        c2_points = self.points.copy()
        c2_points.append(c2_points.pop(0)) # cycle corners by 1 element

        for i in range(len(c1_points)):
            c1 = c1_points[i]
            c2 = c2_points[i]

            val1 = self._is_counter_clockwise(p1, p2, c1)
            val2 = self._is_counter_clockwise(p1, p2, c2)
            val3 = self._is_counter_clockwise(c1, c2, p1)
            val4 = self._is_counter_clockwise(c1, c2, p2)

            # general case
            if ((val1 != val2) and (val3 != val4)):
                return True
            
            # special case where they're colinear
            if (((val1 == 0) and self._is_point_in_colinear_segment(p1, p2, c1)) or 
                ((val2 == 0) and self._is_point_in_colinear_segment(p1, p2, c2)) or
                ((val3 == 0) and self._is_point_in_colinear_segment(c1, c2, p1)) or
                ((val4 == 0) and self._is_point_in_colinear_segment(c1, c2, p2))):
                return True
            
        return False

            


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
        waypoints.append(Pose(x[i], y[i], t[i]))
    return waypoints

def rotate_point_about_origin(point, radians):
        adjusted = Pose()
        adjusted.x = (point.x * math.cos(radians)) - (point.y * math.sin(radians))
        adjusted.y = (point.x * math.sin(radians)) + (point.y * math.cos(radians))

        return adjusted

class SwarmRobot(Node):

    Rel_Pose_Pushing_Threshold = 0.07 #m
    Rob_Pose_Threshold = 0.06 #m
    Object_Trans_Goal_Threshold = 0.1 #m
    Object_Rot_Goal_Threshold = 0.2 #rads

    Space_To_Box = 0.13

    Valid_Spots = [Pose(-0.8, 1+Space_To_Box), 
                   Pose(-0.5, 1+Space_To_Box), 
                   Pose(-0.2, 1+Space_To_Box), 
                   Pose(0.2, 1+Space_To_Box),
                   Pose(0.5,1+Space_To_Box), 
                   Pose(0.8, 1+Space_To_Box), 
                   Pose(1+Space_To_Box, 0.8), 
                   Pose(1+Space_To_Box, 0.0), 
                   Pose(1+Space_To_Box, -0.8), 
                   Pose(0.8, -1-Space_To_Box), 
                   Pose(0.5, -1-Space_To_Box), 
                   Pose(0.2, -1-Space_To_Box), 
                   Pose(-0.2, -1-Space_To_Box), 
                   Pose(-0.5, -1-Space_To_Box), 
                   Pose(-0.8, -1-Space_To_Box),
                   Pose(-1-Space_To_Box, -0.8), 
                   Pose(-1-Space_To_Box, 0), 
                   Pose(-1-Space_To_Box, 0.8)]


    def __init__(self):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self.number = None # block robot number 0-5

        self._goal = Pose()
        self._driving_goal = Pose() # only used for diagnosis for now
        self._robot_goal = Pose()
        self._vel_gain = 50.0
        self._max_vel = 0.4

        self._pose = Pose()
        self._pose_rel_box = Pose()
        self._cur_state = FSM_STATES.START_TRANS
        self._start_time = self.get_clock().now().nanoseconds * 1e-9

        self._box = Pose(2.0, 2.0, 0.0)
        self._box_init = Pose(self._box.x, self._box.y, self._box.t)

        self._is_adjust = False # micro adjust during movements to keep position
        
        self._reallocate = False # redistributing to dif spots for better pushing
        self._finding_spot = True # finding a spot to reallocate to
        self._pushing = False
        self._spots = []
        self._pushing_spots = []
        self._init_spot = -1

        self._teams_completed_waypoint = -1

        self._x_list, self._y_list, self._t_list, self._x_object, self._y_object = None, None, None, None, None
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._x_list)
        self.declare_parameter('goal_y', value=self._y_list)
        self.declare_parameter('goal_t', value=self._t_list)
        self.declare_parameter('max_vel', value=self._max_vel)
        self.declare_parameter('vel_gain', value=self._vel_gain)
        self.declare_parameter('object_x', value=self._x_object)
        self.declare_parameter('object_y', value=self._y_object)

        self._waypoints = assign_waypoints(self._x_list, self._y_list, self._t_list)
        self._waypoints_index = -1
        self._get_next_waypoint()

        self._object = Object(self._x_object, self._y_object)

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._listener_callback, 5)
        self._obj_sub = self.create_subscription(ModelStates, "/gazebo/model_states", self._box_callback, 1)
        self._swarm_waypoints_sub = self.create_subscription(Int32, "/last_complete_waypoint", self._waypoints_callback, 1) # listening to see if everyone is at their positions for the current waypoint
        self._spots_sub = self.create_subscription(Int32MultiArray, "/taken_spots", self._spots_callback, 5)
        self._pushing_spots_sub = self.create_subscription(Int32MultiArray, "/pushing_spots", self._pushing_spots_callback, 5)

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 5)
        self._diagnosis_pub = self.create_publisher(String, "/diagnosis", 5)
        self._waypoints_pub = self.create_publisher(Int32, "/complete_waypoint", 5)
        self._push_pub = self.create_publisher(Bool, "/pushing", 5)

        self._reallocate_cli = self.create_client(AddTwoInts, '/reallocate')
        while not self._reallocate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self._req = AddTwoInts.Request()

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
            elif param.name == 'object_x' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                self._x_object = param.value
            elif param.name == 'object_y' and param.type_ == Parameter.Type.DOUBLE_ARRAY:
                self._y_object = param.value
            elif param.name == 'reallocate' and param.type_ == Parameter.Type.BOOL:
                self._reallocate = param.value
            elif param.name == 'number' and param.type_ == Parameter.Type.INTEGER:
                self.number = int(param.value)
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().warn(f"Changing goal {self._goal.x} {self._goal.y} {self._goal.t}")
        return SetParametersResult(successful=True)
    
    def _waypoints_callback(self, msg):
        self._teams_completed_waypoint = msg.data

    def _spots_callback(self, msg):
        self._spots = msg.data
        if self._init_spot == -1:
            self._init_spot = self._spots.index(self.number)

    def _pushing_spots_callback(self, msg):
        self._pushing_spots = msg.data

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
        #get index of last completed waypoint from current index
        way = Int32()
        if self._cur_state == FSM_STATES.WAYPOINT_DONE or self._cur_state == FSM_STATES.TASK_DONE:
            way.data = self._waypoints_index
        else:
            way.data = self._waypoints_index - 1 
        self._waypoints_pub.publish(way)
        self._diagnosis()
        self._state_machine()

    def _state_machine(self):
        if self._cur_state == FSM_STATES.START_TRANS:
            self._do_state_start_trans()
        elif self._cur_state == FSM_STATES.REALLOCATING_BF_TRANS:
            self._do_state_reallocation_bf_trans()
        elif self._cur_state == FSM_STATES.PUSHING_TRANS:
            self._do_state_pushing_trans()
        elif self._cur_state == FSM_STATES.FOLLOWING_TRANS:
            self._do_state_following_trans()
        elif self._cur_state == FSM_STATES.START_ROT:
            self._do_state_start_rot()
        elif self._cur_state == FSM_STATES.REALLOCATING_BF_ROT:
            self._do_state_reallocation_bf_rot()
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

    def _send_request(self, a, b):
        self._req.a = a # spot you want
        self._req.b = b # number robot you are
        self.future = self._reallocate_cli.call_async(self._req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def _set_rob_goal(self):
        # set initial distance from box and maintain it
        self._pose_rel_box.x = self._box.x - self._pose.x
        self._pose_rel_box.y = self._box.y - self._pose.y

        # set robot goal pose
        self._robot_goal.x = self._pose.x + self._goal.x
        self._robot_goal.y = self._pose.y + self._goal.y

    def _do_state_start_trans(self):
        # determine if they will push or if they will follow
        # if distance from self to goal > dist of box to goal then we push
        # else we follow
        # sleep for a bit
        self._cmd_vel_pub.publish(Twist())
        time.sleep(5.0)

        self._set_rob_goal()

        # d_self2goal = distance(self._pose.x, self._pose.y,  self._robot_goal.x, self._robot_goal.y)
        # d_box2goal = distance(self._box.x, self._box.y, self._robot_goal.x, self._robot_goal.y)

        #TODO maybe worth it to have it be able to transition between pushing and following? 
        # (well if they do it right the first time, not necessary)
        # if d_self2goal>d_box2goal:
        if self._object.does_line_intersect_object([self._pose.x, self._pose.y], [self._robot_goal.x, self._robot_goal.y], self._box):
            self.get_logger().info(f"I am pushing.")# me2goal: {d_self2goal}, box2goal: {d_box2goal}")

            # only go into reallocating state if we want it
            if self._reallocate == False:
                self._cur_state = FSM_STATES.PUSHING_TRANS
                time.sleep(1.0)
                return
            else:
                self._pushing = True
                self._push_pub.publish(self._pushing)
                self._cur_state = FSM_STATES.REALLOCATING_BF_TRANS
        else:
            self.get_logger().info(f"I am following.")# me2goal: {d_self2goal}, box2goal: {d_box2goal}")
            if self._reallocate == False:
                self._cur_state = FSM_STATES.FOLLOWING_TRANS
                time.sleep(1.0)
                return
            else:
                # reallocate before doing anything
                self._pushing = False
                self._push_pub.publish(self._pushing)
                self._cur_state = FSM_STATES.REALLOCATING_BF_TRANS

        self._push_pub.publish(self._pushing)   
        time.sleep(1.0) # wait so that the pushing can happen first
        self._push_pub.publish(self._pushing)
        return
    
    def _do_state_reallocation_bf_trans(self):
        # do until you get assigned somewhere
        # ie. srv response is true

        if self._finding_spot:
            # need to call srv until its true
            # find all the pushing robot indices
            indices = [i for i, x in enumerate(self._pushing_spots) if x == 1]

            # randomly choose a pushing robot and go either left or right of them lol
            #TODO this fails for sure if there is only pushing robots on the short end of the shape
            left_right = random.choice([-1, 1])
            selected_rob = random.choice(indices)
            spot = selected_rob + left_right
            if spot < 0 or spot > 17:
                # go other way if no spot avail
                spot = selected_rob - left_right

            response = self._send_request(int(spot), int(self.number))
            if response.sum == 1:
                self._finding_spot = False
                # reallocation is correct. now we go to spot
                # rotate spot by box angle
                realloc_pose = SwarmRobot.Valid_Spots[spot]
                realloc_pose = rotate_point_about_origin(realloc_pose, self._box.t)

                # add translation and this is goal location
                self._robot_goal.x = realloc_pose.x + self._box.x
                self._robot_goal.y = realloc_pose.y + self._box.y
        else:
            # go to the spot we chose --> now it's robot_goal
            # need to figure out how to avoid box and not just go in straight line lol
            #TODO avoid box and other bots

            if ((abs(self._pose.x - self._robot_goal.x) <= SwarmRobot.Rob_Pose_Threshold) and 
            (abs(self._pose.y - self._robot_goal.y) <= SwarmRobot.Rob_Pose_Threshold)):
                # we are reallocated
                self._finding_spot = True # set this back to true
                self._set_rob_goal()
                if self._object.does_line_intersect_object([self._pose.x, self._pose.y], [self._robot_goal.x, self._robot_goal.y], self._box):
                    self.get_logger().info(f"I am pushing.")# me2goal: {d_self2goal}, box2goal: {d_box2goal}")
                    self._cur_state = FSM_STATES.PUSHING_TRANS
                else:
                    self.get_logger().info(f"I am following.")# me2goal: {d_self2goal}, box2goal: {d_box2goal}")
                    if self._reallocate == False:
                        self._cur_state = FSM_STATES.FOLLOWING_TRANS

                time.sleep(1.0)
                return

            self._drive_to_goal(self._robot_goal)

        return
    
    def _do_state_pushing_trans(self):
        if (((abs(self._box.x - (self._box_init.x + self._goal.x)) <= SwarmRobot.Object_Trans_Goal_Threshold) and 
            (abs(self._box.y - (self._box_init.y + self._goal.y)) <= SwarmRobot.Object_Trans_Goal_Threshold))
            or ((abs(self._pose.x - self._robot_goal.x) <= SwarmRobot.Rob_Pose_Threshold) and 
            (abs(self._pose.y - self._robot_goal.y) <= SwarmRobot.Rob_Pose_Threshold))):
            # if the box is at goal OR the robot is at goal
            self._cur_state = FSM_STATES.START_ROT
            return
        
        # check if we're too far from our initial relative position
        curr_pose_rel_box = Pose()
        curr_pose_rel_box.x = self._box.x - self._pose.x
        curr_pose_rel_box.y = self._box.y - self._pose.y

        if ((abs(curr_pose_rel_box.x - self._pose_rel_box.x) > SwarmRobot.Rel_Pose_Pushing_Threshold) or 
            (abs(curr_pose_rel_box.y - self._pose_rel_box.y) > SwarmRobot.Rel_Pose_Pushing_Threshold)):
            #get the correct pose on the box
            maintained_pose = Pose()
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
        if (((abs(self._box.x - (self._box_init.x + self._goal.x)) <= SwarmRobot.Object_Trans_Goal_Threshold) and 
            (abs(self._box.y - (self._box_init.y + self._goal.y)) <= SwarmRobot.Object_Trans_Goal_Threshold))
            or ((abs(self._pose.x - self._robot_goal.x) <= SwarmRobot.Rob_Pose_Threshold) and 
            (abs(self._pose.y - self._robot_goal.y) <= SwarmRobot.Rob_Pose_Threshold))):
            self._cur_state = FSM_STATES.START_ROT
            return
        
        # follow the box
        # set new goals depending on where the box is
        temp_goal = Pose()
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

        self._box_init_t = self._box.t

        if self._reallocate == False:
            self._cur_state = FSM_STATES.PUSHING_ROT
        else:
            self._cur_state = FSM_STATES.REALLOCATING_BF_ROT

    def _do_state_reallocation_bf_rot(self):
        #TODO avoid box and other bots
        if ((abs(self._pose.x - self._robot_goal.x) <= SwarmRobot.Rob_Pose_Threshold) and 
            (abs(self._pose.y - self._robot_goal.y) <= SwarmRobot.Rob_Pose_Threshold)):
                # we are reallocated
                self._cur_state = FSM_STATES.PUSHING_ROT
        else:
            realloc_pose = SwarmRobot.Valid_Spots[self._init_spot]
            realloc_pose = rotate_point_about_origin(realloc_pose, self._box.t)

            # add translation and this is goal location
            self._robot_goal.x = realloc_pose.x + self._box.x
            self._robot_goal.y = realloc_pose.y + self._box.y

            self._drive_to_goal(self._robot_goal)
        return

    def _do_state_pushing_rot(self):
        # moved here. We want to check if our goal location has been hit for rotation instead of the box.
        # sometimes box is rotated but we are not there and it messes up the next waypoint start position
        temp = rotate_point_about_origin(self._pose_rel_box, self._goal.t)
        temp.x = self._box.x - temp.x
        temp.y = self._box.y - temp.y
        
        if ((abs(self._pose.x - temp.x) <= SwarmRobot.Rob_Pose_Threshold) and 
            (abs(self._pose.y - temp.y) <= SwarmRobot.Rob_Pose_Threshold)):
            # get next waypoint before saying the waypoint is done so we wait for everyone else to be ready for the next one.
            # this lessens length of our waypoint list so that everyone else has to be done their current waypoint before 
            # all_at_waypoint is true
            self.get_logger().info(f"rotation complete, switching to waiting for everyone else: {self._teams_completed_waypoint}")
            self._cur_state = FSM_STATES.WAYPOINT_DONE
            return

        # check if we're too far from our initial relative position
        curr_pose_rel_box = Pose()
        curr_pose_rel_box.x = self._box.x - self._pose.x
        curr_pose_rel_box.y = self._box.y - self._pose.y

        # rotate our initial position based on the box's rotation since we started rotating
        # initial pose ref frame
        rotated_rel_pose = rotate_point_about_origin(self._pose_rel_box, self._box.t- self._box_init_t)

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

            # Try only rotating in little increments so that we follow an arc around to the correct point instead of driving straight there
            temp = rotate_point_about_origin(curr_pose_rel_box, 0.175) # 10 degs
            temp.x = self._box.x - temp.x
            temp.y = self._box.y - temp.y

            self._drive_to_goal(temp, 0)

    def _do_state_following_rot(self):
        pass
    
    def _do_state_waypoint_done(self):
        # make sure motion is stopped 
        # wait for others to get to their curr waypoint
        # start up once their waypoint length is same as yours
        # self.get_logger().info(f"waiting for all same to be true: {self._teams_completed_waypoint}")
        if self._teams_completed_waypoint == self._waypoints_index: 
            # start up the process again
            # reset the box init position 
            # TODO: bad for resetting box init I think but might take a while to change it
            self._box_init = Pose(self._box.x, self._box.y, self._box.t)
            if self._waypoints_index >= len(self._waypoints):
                self._cur_state = FSM_STATES.TASK_DONE
            else:
                # wait for everyone to get the message
                time.sleep(1)
                self._get_next_waypoint()
                if (self._cur_state != FSM_STATES.TASK_DONE): # get_next_waypoint() can change the state too
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
    
    def _diagnosis(self):
        diagnosis = String()
        diagnosis.data = f"next waypoint: {self._waypoints}, index: {self._waypoints_index}, teams_completed_waypoint: {self._teams_completed_waypoint}, pose wrt world: {round(self._pose.x, 3)}, {round(self._pose.y, 3)}, {round(self._pose.t, 3)} pose wrt box: {round(self._box.x - self._pose.x, 3)}, {round(self._box.y - self._pose.y, 3)} init pose wrt box: {round(self._pose_rel_box.x, 3)}, {round(self._pose_rel_box.x, 3)} is adjust: {self._is_adjust} box pose: {round(self._box.x, 3)}, {round(self._box.y, 3)}, {round(self._box.t, 3)} driving goal: {round(self._driving_goal.x, 3)}, {round(self._driving_goal.y, 3)}, {round(self._driving_goal.t, 3)}\n"
        
        self._diagnosis_pub.publish(diagnosis)
        return
    
    def _get_next_waypoint(self):
        # pop first index of waypoints only if there are more waypoints to go to
        if self._waypoints_index < len(self._waypoints) - 1:
            self._waypoints_index += 1
            self._goal = self._waypoints[self._waypoints_index]
            self.get_logger().info(f"getting next waypoint: {self._waypoints}, index: {self._waypoints_index}")
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


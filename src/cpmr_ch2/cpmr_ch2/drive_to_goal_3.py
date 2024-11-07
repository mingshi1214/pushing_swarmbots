import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

import numpy as np
import PIL.Image as Img
import cv2
import numpy as np
import math
import sys
import argparse
import json
import random

random.seed(4)
def dijkstra_algorithm(graph, start_node):
    unvisited_nodes = list(graph.get_nodes())
 
    # We'll use this dict to save the cost of visiting each node and update it as we move along the graph   
    shortest_path = {}
 
    # We'll use this dict to save the shortest known path to a node found so far
    previous_nodes = {}
 
    # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
    # However, we initialize the starting node's value with 0   
    shortest_path[start_node] = 0
    
    # The algorithm executes until we visit all nodes
    while unvisited_nodes:
        # The code block below finds the node with the lowest score
        current_min_node = None
        for node in unvisited_nodes: # Iterate over the nodes
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
                
        # The code block below retrieves the current node's neighbors and updates their distances
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                # We also update the best path to the current node
                previous_nodes[neighbor] = current_min_node
 
        # After visiting its neighbors, we mark the node as "visited"
        unvisited_nodes.remove(current_min_node)
    
    return previous_nodes, shortest_path

def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node
    
    while node != start_node:
        path.append(node)
        node = previous_nodes[node]
 
    # Add the start node manually
    path.append(start_node)
    
    print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ".join(reversed(path)))
    return reversed(path)

class rrt():
    def __init__(self, objects):
        self.map = np.ones((1234, 1086, 3), dtype=np.uint8)*255
        self.objects = objects
        self.nodes = {}
        self.show_nodes = []
        self.dist_thresh = 100
        self.tree = {}
        self.path = None

    def occupy_map(self):
        img = self.map
        # draw obs
        print(self.objects.shape)
        img[self.objects.T[0], self.objects.T[1]] = (0, 0, 0)
        self.map = img
        return 

    def get_nodes(self):
        return list(self.tree)
   
    def get_outgoing_edges(self, node):
        "Returns the neighbors of a node."
        connections = []
        for out_node in list(self.tree):
            if self.tree[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections
   
    def value(self, node1, node2):
        "Returns the value of an edge between two nodes."
        return self.tree[node1][node2]
   
    def show_img(self):
        img = self.map.astype(np.uint8)
        for node in self.show_nodes:
            # img[node[0], node[1]] = node[2]
            img = cv2.circle(img, (node[1], node[0]), 4, node[2], -1) 

        # draw connections
        tree_nodes = list(self.tree)
        for node in tree_nodes:
            connections = list(self.tree[node])
            for c in connections:
                end = self.nodes[c]
                start = self.nodes[node]
                img = cv2.line(img, (start[1], start[0]), (end[1], end[0]), (255, 0, 0), 3)

        if self.path != None:
            for i in range(0, len(self.path)-1):
                start = self.nodes[self.path[i]]
                end = self.nodes[self.path[i+1]]
                img = cv2.line(img, (start[1], start[0]), (end[1], end[0]), (255, 0, 255), 3)

        cv2.imshow("image", img)
        cv2.waitKey(1)
        return
   
    def gen_rand_node(self, i):
        dim_x = self.map.shape[0]-1
        dim_y = self.map.shape[1]-1
        random.seed(i+500)
        return random.randint(200, 1100), random.randint(92,dim_y)

    def seed(self, startx, starty):
        n = 0
        for i in range(0,4500):
            if i == 0:
                n_x, n_y = startx, starty
            else:
                n_x, n_y = self.gen_rand_node(i)
            if (self.map[n_x, n_y, :] & [255, 255, 255 ]).all():
                self.nodes[f'node_{n}'] = [n_x, n_y]
                n += 1
                self.show_nodes.append([n_x, n_y, (0, 255, 0)])
            else:
                self.show_nodes.append([n_x, n_y, (0, 0, 255)])
        return
   
    def dist(self, x1, y1, x2, y2):
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)
   
    def collision(self, x_1, y_1, x_2, y_2):
        # Ax + By + C = 0 line from current position to goal
        # check for collisions witht the current obstacle
        # this assumes infinite line...
        a = (y_1-y_2)
        b = (x_2-x_1)
        c = (x_1-x_2)*y_1 + x_1*(y_2-y_1)

        min_x = min(x_1, x_2) - 2
        max_x = max(x_1, x_2) + 2
        min_y = min(y_1, y_2) - 2
        max_y = max(y_1, y_2) + 2

        # self.map = cv2.rectangle(self.map, pt1=(min_y,min_x), pt2=(max_y,max_x), color=(0,0,255), thickness=10)
        for obs in self.objects:
            # only check for collisions in the box that the points make
            if obs[0] < max_x and obs[0] > min_x:
                if obs[1] < max_y and obs[1] > min_y:
                    d = abs(a*obs[0] + b*obs[1] + c)/np.sqrt(a**2 + b**2)
                    if d <= 1:
                        return True
        return False
   
    def grow(self, time=1):
        # choose one of points as root
        start_ind = 0
        nodes = list(self.nodes)
        if time == 1:
            self.tree[nodes[start_ind]] = {}

        for node_i in nodes:
            #try to connect to any node in the tree
            smallest = 100000000
            node_closest = None
            tree_nodes = list(self.tree)
            for node_j in tree_nodes:
                if node_i == node_j:
                    continue
                [x_i, y_i] = self.nodes[node_i]
                [x_j, y_j] = self.nodes[node_j]

                # try to connect
                # if dist too far, dont try. not really a good connection
                dist = self.dist(x_i, y_i, x_j, y_j)
                if dist > self.dist_thresh:
                    continue
                if dist < smallest:
                    smallest = dist
                    node_closest = [x_j, y_j]
                    name = node_j
           
            # check for collisions with the closest node
            if node_closest is not None:
                if not self.collision(x_i, y_i, node_closest[0], node_closest[1]):
                    # we can add this as a connection
                    if node_i not in self.tree:
                        self.tree[node_i] = {}
                    if name not in self.tree:
                        self.tree[name] = {}
                    self.tree[node_i][name] = dist
                    self.tree[name][node_i] = dist

            # self.show_img()
            # cv2.waitKey(1)
        return
   
    def attach(self, x, y, name):
        nodes = list(self.tree)
        for node in nodes:
            [x_i, y_i] = self.nodes[node]
            dist = self.dist(x_i, y_i, x, y)
            if dist > self.dist_thresh:
                continue
            if not self.collision(x_i, y_i, x, y):
                    # we can add this as a connection
                if name not in self.tree:
                    self.tree[name] = {}
                    self.nodes[name] = [x, y]
                    self.show_nodes.append([x, y, (0, 255, 0)])
                self.tree[name][node] = dist
                self.tree[node][name] = dist
        return

    def run(self, startx, starty, goalx, goaly):
        # self.show_img()
        self.occupy_map()
        # self.show_img()
        self.seed(startx, starty)
        # self.show_img()
        # input()
        self.grow()
        self.grow(2)
        # self.show_img()
        # input()
        self.attach(startx, starty, 's')
        # self.show_img()
        self.attach(goalx, goaly, 'g')
        # self.show_img()
        # input()
        return

    def world_path(self):
        world_path = []
        map_path = []
        for node in self.path:
            [x, y] = self.nodes[node]
            big_map = transform_to_orig_map([x, y])
            world = transform_to_world(np.array(big_map))
            world_path.append(world)
            map_path.append([x, y])
        print(np.array(world_path))
        print(np.array(map_path))
        return world_path

def transform_to_orig_map(small_coords):
    scale = 1
    return [small_coords[0] * scale, small_coords[1] * scale]

def transform_to_world(map_coords):
    offset = [-0.188, 0.82]
    scale = 1
    map_big = map_coords * scale
    
    map_m_x = map_big.T[0] - 1234//2
    map_m_y = map_big.T[1] - 1086//2
    #every 40 pixels is 1m 
    map_w_x = map_m_x/41 - offset[1]
    map_w_y = map_m_y/41 - offset[0]

    map_w = np.stack((-map_w_x, map_w_y)).T
    return map_w

def transform_to_small_map(world_coords):
    offset = [-0.188, 0.82]
    scale = 1

    w_x = -world_coords.T[0]
    w_y = world_coords.T[1]

    map_x = ((w_x + offset[1])*41 + 1234//2) //scale
    map_y = ((w_y + offset[0])*41 + 1086//2) //scale
    
    return np.stack((map_x, map_y)).T
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
        super().__init__('drive_to_goal_3')
        self.get_logger().info(f'{self.get_name()} created')

        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_t = 0.0
        self._vel_gain = 50.0
        self._max_vel = 0.4
        self.odom = None
        self._cur_x = 0
        self._cur_y = 0
        self._cur_t = 0
        self._max_vel = 0.1
        self._vel_gain = 2.0

        self.waypoints = None
        self.waypoints_assigned = False
        self.point = 0
        self.goal_reached = False

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.declare_parameter('goal_x', value=self._goal_x)
        self.declare_parameter('goal_y', value=self._goal_y)

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)


    def _listener_callback(self, msg, max_pos_err=0.05):
        pose = msg.pose.pose

        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        self._cur_t = yaw
        
        if self.waypoints_assigned == False:
            #run rrt and generate waypoints to go to goal
            small = np.asarray(Img.open('/home/ming/robo_course/IntroToRobotics/working_dir/src/cpmr_ch4/cpmr_ch4/occ_map.png'))
            # gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            gray = small
            black_pixels = np.array(np.where(gray == 0))
            black_pixel_coordinates = np.array(list(zip(black_pixels[0],black_pixels[1])))
            # add a radius of 0.5 to all the "objects"
            objects = np.concatenate((black_pixel_coordinates.T, np.ones(black_pixel_coordinates.shape[0])[:, None].T), axis = 0).T
            print(objects.shape)

            rrt_run = rrt(black_pixel_coordinates)
            start = transform_to_small_map(np.array([[self._cur_y, self._cur_x]]))[0]
            goal = transform_to_small_map(np.array([[self._goal_x, self._goal_y]]))[0]
            self.get_logger().info(f"want to go to {goal}. At {start})")
            rrt_run.run(int(start[0]), int(start[1]), int(goal[0]), int(goal[1]))
            previous_nodes, shortest_path = dijkstra_algorithm(graph=rrt_run, start_node="s")
            path = print_result(previous_nodes, shortest_path, start_node="s", target_node="g")
            rrt_run.path = list(path)
            rrt_run.show_img()
            self.waypoints = np.array(rrt_run.world_path())
            np.savetxt('waypoints.txt', self.waypoints, delimiter=',')
            self.waypoints_assigned = True


        if self.goal_reached == False:
            point = self.waypoints[self.point]
            self.goal_t = np.arctan2(point[1], point[0]) + np.pi
            self._go_to_target([point[1], point[0]], self.goal_t)
        else:
            self.goal_reached = False
            if self.point < self.waypoints.shape[0] - 1:
                self.point += 1


    def _go_to_target(self, pose, goal_t, max_pos_err=0.05):
        # return x, y, t for twist command
        x_diff = pose[0] - self._cur_x
        y_diff = pose[1] - self._cur_y
        t_diff = goal_t - self._cur_t 
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
        
        self.get_logger().info(f"at ({self._cur_x},{self._cur_y},{self._cur_t}) goal ({pose[0]},{pose[1]},{goal_t})")
        self.get_logger().info(f"vels: {twist.linear.x}, {twist.linear.y}, {twist.angular.z}")

        self._publisher.publish(twist)

        if abs(dist) <= max_pos_err:
            self.goal_reached = True
            self.get_logger().info(f'current position {self._cur_x}, {self._cur_y}, {self._cur_t} at goal {pose}')
        
        return twist

    def parameter_callback(self, params):
        self.get_logger().info(f'move_robot_to_goal parameter callback')
        for param in params:
            if param.name == 'goal_x' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_x = param.value
            elif param.name == 'goal_y' and param.type_ == Parameter.Type.DOUBLE:
                self._goal_y = param.value
            else:
                self.get_logger().warn(f'Invalid parameter {param.name}')
                return SetParametersResult(successful=False)
            self.get_logger().warn(f"Changing goal {self._goal_x} {self._goal_y}")
        
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


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Int32MultiArray, Bool
from example_interfaces.srv import AddTwoInts # couldnt figure out how to rospy service in ros 2
import time 

class SwarmBroadcast(Node):
    def __init__(self):
        super().__init__('swarm_broadcast')
        self.get_logger().info(f'{self.get_name()} created')

        time.sleep(2)
        self._waypoint_lens = [1, 2, 3, 4, 5]
        self._all_same = False

        self._sub0 = self.create_subscription(Int32, "block_robot_0/complete_waypoint", self._callback0, 1)
        self._sub1 = self.create_subscription(Int32, "block_robot_1/complete_waypoint", self._callback1, 1)
        self._sub2 = self.create_subscription(Int32, "block_robot_2/complete_waypoint", self._callback2, 1)
        self._sub3 = self.create_subscription(Int32, "block_robot_3/complete_waypoint", self._callback3, 1)
        self._sub4 = self.create_subscription(Int32, "block_robot_4/complete_waypoint", self._callback4, 1)
        self._sub5 = self.create_subscription(Int32, "block_robot_5/complete_waypoint", self._callback5, 1)
        
        self._publisher = self.create_publisher(Int32, "/last_complete_waypoint", 1)

        self._sub_push0 = self.create_subscription(Bool, "block_robot_0/pushing", self._callbackpushing0, 1)
        self._sub_push1 = self.create_subscription(Bool, "block_robot_1/pushing", self._callbackpushing1, 1)
        self._sub_push2 = self.create_subscription(Bool, "block_robot_2/pushing", self._callbackpushing2, 1)
        self._sub_push3 = self.create_subscription(Bool, "block_robot_3/pushing", self._callbackpushing3, 1)
        self._sub_push4 = self.create_subscription(Bool, "block_robot_4/pushing", self._callbackpushing4, 1)
        self._sub_push5 = self.create_subscription(Bool, "block_robot_5/pushing", self._callbackpushing5, 1)

        # spots are where the robots are. numbers are the block robot numbers
        self._spots = [-1, 4, -1, -1, 5, -1, -1, 3, -1, -1, 1, -1, -1, 0, -1, -1, 2, -1]
        self._push_spots = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1]

        self._srv = self.create_service(AddTwoInts, '/reallocate', self._reallocate_callback)
        self._pub_occupancylist = self.create_publisher(Int32MultiArray, "/taken_spots", 1)
        self._pub_pushlist = self.create_publisher(Int32MultiArray, "/pushing_spots", 1)

    def _reallocate_callback(self, request, response):
        # lol sum will just be -1 if false and 1 true
        # request a is slot they want
        # request b is which robot they are
        if self._spots[request.a] == -1:
            # taken. return false
            response.sum = -1
            self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
            self.get_logger().info(f'Cannot. {self._spots}')
        else:
            # return true
            self._spots[self._spots.index(request.b)] = -1
            self._spots[request.a] = request.b
            response.sum = 1
        return response

    def _callback0(self, msg):
        self._waypoint_lens[0] = msg.data
        same = Int32()
        last_way = 9999999
        for way in self._waypoint_lens:
            last_way = min(last_way, way)

        same.data = last_way
        # same.data = all(i == self._waypoint_lens[0] for i in self._waypoint_lens)
        self._publisher.publish(same)
        self._pub_occupancylist.publish(self._spots)
        self._pub_pushlist.publish(self._push_spots)
        # self.get_logger().info(f'allsame: {same.data}. waypoints: {self._waypoint_lens}')

    def _callback1(self, msg):
        self._waypoint_lens[1] = msg.data

    def _callback2(self, msg):
        self._waypoint_lens[2] = msg.data

    def _callback3(self, msg):
        self._waypoint_lens[3] = msg.data

    def _callback4(self, msg):
        self._waypoint_lens[4] = msg.data

    def _callback5(self, msg):
        self._waypoint_lens[5] = msg.data

    def _callbackpushing0(self, msg):
        if msg.data:
            # pushing
            self._push_spots[self._spots.index(0)] = 1
        else:
            self._push_spots[self._spots.index(0)] = -1

    def _callbackpushing1(self, msg):
        if msg.data:
            # pushing
            self._push_spots[self._spots.index(1)] = 1
        else:
            self._push_spots[self._spots.index(1)] = -1
        
    def _callbackpushing2(self, msg):
        if msg.data:
            # pushing
            self._push_spots[self._spots.index(2)] = 1
        else:
            self._push_spots[self._spots.index(2)] = -1

    def _callbackpushing3(self, msg):
        if msg.data:
            # pushing
            self._push_spots[self._spots.index(3)] = 1
        else:
            self._push_spots[self._spots.index(3)] = -1

    def _callbackpushing4(self, msg):
        if msg.data:
            # pushing
            self._push_spots[self._spots.index(4)] = 1
        else:
            self._push_spots[self._spots.index(4)] = -1

    def _callbackpushing5(self, msg):
        if msg.data:
            # pushing
            self._push_spots[self._spots.index(5)] = 1
        else:
            self._push_spots[self._spots.index(5)] = -1


def main():
    rclpy.init()
    node = SwarmBroadcast()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
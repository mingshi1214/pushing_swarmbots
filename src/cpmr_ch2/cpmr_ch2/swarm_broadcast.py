import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
import time 

class SwarmBroadcast(Node):
    def __init__(self):
        super().__init__('swarm_broadcast')
        self.get_logger().info(f'{self.get_name()} created')

        time.sleep(2)
        self._waypoint_lens = [1, 2, 3, 4, 5]
        self._all_same = False

        self._sub0 = self.create_subscription(Int32, "block_robot_0/waypoints", self._callback0, 1)
        self._sub1 = self.create_subscription(Int32, "block_robot_1/waypoints", self._callback1, 1)
        self._sub2 = self.create_subscription(Int32, "block_robot_2/waypoints", self._callback2, 1)
        self._sub3 = self.create_subscription(Int32, "block_robot_3/waypoints", self._callback3, 1)
        self._sub4 = self.create_subscription(Int32, "block_robot_4/waypoints", self._callback4, 1)
        self._sub5 = self.create_subscription(Int32, "block_robot_5/waypoints", self._callback5, 1)
        
        self._publisher = self.create_publisher(Bool, "/all_same", 1)


    def _callback0(self, msg):
        self._waypoint_lens[0] = msg.data
        same = Bool()
        same.data = all(i == self._waypoint_lens[0] for i in self._waypoint_lens)
        self._publisher.publish(same)
        # self.get_logger().info(f'allsame: {same.data}. waypoints: {self._waypoint_lens}')

    def _callback1(self, msg):
        self._waypoint_lens[0] = msg.data

    def _callback2(self, msg):
        self._waypoint_lens[1] = msg.data

    def _callback3(self, msg):
        self._waypoint_lens[2] = msg.data

    def _callback4(self, msg):
        self._waypoint_lens[3] = msg.data

    def _callback5(self, msg):
        self._waypoint_lens[4] = msg.data

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
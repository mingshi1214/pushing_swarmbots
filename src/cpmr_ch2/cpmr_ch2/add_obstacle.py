# A very simple ros node to populate the world
#
import rclpy
import os
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

def make_obstacle(node, id, x0, y0, h, r):
# def make_obstacle(node, id, x0, y0, lx, ly):
  #  BOX_MODEL = """
  #       <sdf version="1.4">
  #         <model name="my_model">
  #           <pose>0 0 0.5 0 0 0</pose>
  #           <static>true</static>
  #           <link name="link">
  #             <inertial>
  #               <mass>1.0</mass>
  #               <inertia> <!-- inertias are tricky to compute -->
  #                 <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
  #                 <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
  #                 <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
  #                 <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
  #                 <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
  #                 <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
  #                 <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
  #               </inertia>
  #             </inertial>
  #             <collision name="collision">
  #               <geometry>
  #                 <box>
  #                   <size>{lx} {ly} 1</size>
  #                 </box>
  #               </geometry>
  #             </collision>
  #             <visual name="visual">
  #               <geometry>
  #                 <box>
  #                   <size>{lx} {ly} 1</size>
  #                 </box>
  #               </geometry>
  #             </visual>
  #           </link>
  #         </model>
  #       </sdf>"""

  #  client = node.create_client(SpawnEntity, "/spawn_entity")
  #  node.get_logger().info("Connecting to /spawn_entity service...")
  #  client.wait_for_service()
  #  node.get_logger().info("...connected")
  #  request = SpawnEntity.Request()
  #  request.name = id
  #  request.initial_pose.position.x = float(x0)
  #  request.initial_pose.position.y = float(y0)
  #  request.initial_pose.position.z = float(0)
  #  dict = {'lx' : lx, 'ly': ly}
  #  request.xml = BOX_MODEL.format(**dict)
  #  node.get_logger().info(f"Making request...")
  #  future = client.call_async(request)
  #  while not future.done():
  #      rclpy.spin_once(node)
  #  node.get_logger().info("...done")
  #  if not future.result().success:
  #      node.get_logger().info(f"Failure {future.result()}")
   CYLINDER_MODEL = """
       <sdf version="1.6"> 				\
         <world name="default">                         \
           <model name="obstacle"> 			\
             <static>false</static> 			\
             <link name="all">                        	\
               <collision name="one">			\
                 <pose>0 0 {o} 0 0 0</pose>    		\
                 <geometry>				\
                   <cylinder>                       	\
                     <radius>{r}</radius>            	\
                     <length>{h}</length>            	\
                   </cylinder>   			\
                  </geometry>				\
               </collision>				\
               <inertial>
                 <mass value="3"/>
                 <inertia ixx="0.76" iyy="0.76" izz="1.5" ixy="0" ixz="0" iyz="0" />
               </inertial>
               <visual name="two">			\
                 <pose>0 0 {o} 0 0 0</pose>    		\
                 <geometry>				\
                   <cylinder>                           \
                     <radius>{r}</radius>               \
                     <length>{h}</length>               \
                   </cylinder>                          \
                 </geometry>				\
               </visual>				\
             </link>                                    \
           </model>					\
           <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"> \
            <ros>\
              <namespace>/gazebo</namespace>\
            </ros>\
            <update_rate>1.0</update_rate>\
          </plugin>
         </world>                                       \
       </sdf>"""

   client = node.create_client(SpawnEntity, "/spawn_entity")
   node.get_logger().info("Connecting to /spawn_entity service...")
   client.wait_for_service()
   node.get_logger().info("...connected")
   request = SpawnEntity.Request()
   request.name = id
   request.initial_pose.position.x = float(x0)
   request.initial_pose.position.y = float(y0)
   request.initial_pose.position.z = float(0)
   dict = {'h' : h, 'r':r, 'o': h/2}
   request.xml = CYLINDER_MODEL.format(**dict)
   node.get_logger().info(f"Making request...")
   future = client.call_async(request)
   while not future.done():
       rclpy.spin_once(node)
   node.get_logger().info("...done")
   if not future.result().success:
       node.get_logger().info(f"Failure {future.result()}")
  
def remove_obstacle(node, id):
   client = node.create_client(DeleteEntity, "/delete_entity")
   node.get_logger().info("Connecting to /delete_entity service...")
   client.wait_for_service()
   node.get_logger().info("...connected")
   request = DeleteEntity.Request()
   request.name = id
   node.get_logger().info("Making request...")
   future = client.call_async(request)
   rclpy.spin_until_future_complete(node, future)
   node.get_logger().info("...done")
   if not future.result().success:
       node.get_logger().info(f"Failure {future.result()}")



def main(args=None):
  rclpy.init(args=args)
  node = rclpy.create_node('demo')
  make_obstacle(node, 'blob', 2, 3, 2, 1)
  
  node.destroy_node()
  rclpy.shutdown()
    
if __name__ == '__main__':
    main()


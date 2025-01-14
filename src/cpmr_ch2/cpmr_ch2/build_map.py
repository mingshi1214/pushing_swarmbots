# Populate the world with a json map file
#
import sys
import json
import rclpy
from ament_index_python.packages import get_package_share_directory
from .add_obstacle import make_obstacle

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('build_map')

    node.declare_parameter('map', 'default.yaml')
    map_name = node.get_parameter('map').get_parameter_value().string_value
    package_path = get_package_share_directory('cpmr_ch2')

    try:
        with open(f"{package_path}/{map_name}") as fd:
            map = json.load(fd)
    except Exception as e:
        node.get_logger().error(f"Unable to find/parse map in {package_path}/{map_name}")
        sys.exit(1)

    for o in map.keys():
        node.get_logger().info(f"Populating map with {map[o]}")
        # make_obstacle(node, o, map[o]['x'], map[o]['y'], map[o]['lx'], map[o]['ly'])
        make_obstacle(node, o, map[o]['x'], map[o]['y'], 0.2, map[o]['r'])
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()


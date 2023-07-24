import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import signal
import yaml

# Global variable to keep track of whether to stop publishing or not
stop_publishing = False

def read_pgm_file(file_path):
    with open(file_path, 'rb') as pgm_file:
        pgm_format = pgm_file.readline().strip().decode('utf-8')
        if pgm_format != 'P5':
            raise ValueError('Invalid PGM format. Expected P5.')

        # Read the width, height, and maximum pixel value from the PGM file header
        header = pgm_file.readline().strip().decode('utf-8').split()
        width, height = int(header[0]), int(header[1])
        max_value = int(pgm_file.readline().strip().decode('utf-8'))

        # Read the pixel data (occupancy values) from the PGM file
        occupancy_data = pgm_file.read()

    # Convert pixel data to a list of occupancy values (0 to 100, where 0 is unknown, 100 is occupied)
    occupancy_values = [int(value * 100 / max_value) for value in occupancy_data]

    return width, height, occupancy_values

def publish_map_data():
    rclpy.init()
    node = Node("map_publisher")
    publisher = node.create_publisher(OccupancyGrid, "map", 10)

    # Read map data from the PGM file
    file_path = "map/map.pgm"
    width, height, occupancy_values = read_pgm_file(file_path)

    def read_yaml_file(yaml_file): 
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        resolution = data['resolution']
        origin_x = data['origin']['position']['x']
        origin_y = data['origin']['position']['y']
        origin_z = data['origin']['position']['z']
        orientation_x = data['origin']['orientation']['x']
        orientation_y = data['origin']['orientation']['y']
        orientation_z = data['origin']['orientation']['z']
        orientation_w = data['origin']['orientation']['w']

        return resolution, origin_x, origin_y, origin_z, orientation_x, orientation_y, orientation_z, orientation_w

    # Read map parameters from the YAML file
    yaml_file_path = "map/output.yaml"
    resolution, origin_x, origin_y, origin_z, orientation_x, orientation_y, orientation_z, orientation_w = read_yaml_file(yaml_file_path)

    # Populate the OccupancyGrid message
    occupancy_grid_msg = OccupancyGrid()
    occupancy_grid_msg.header.frame_id = "map"  # Set the frame ID
    occupancy_grid_msg.info.width = width
    occupancy_grid_msg.info.height = height
    occupancy_grid_msg.info.resolution = resolution
    occupancy_grid_msg.info.origin.position.x = origin_x
    occupancy_grid_msg.info.origin.position.y = origin_y
    occupancy_grid_msg.info.origin.position.z = origin_z
    occupancy_grid_msg.info.origin.orientation.x = orientation_x
    occupancy_grid_msg.info.origin.orientation.y = orientation_y
    occupancy_grid_msg.info.origin.orientation.z = orientation_z
    occupancy_grid_msg.info.origin.orientation.w = orientation_w
    occupancy_grid_msg.data = occupancy_values

    def signal_handler(signum, frame):
        global stop_publishing
        stop_publishing = True

    signal.signal(signal.SIGINT, signal_handler)

    # Publish the map data repeatedly until stop_publishing is True
    # while rclpy.ok() and not stop_publishing:
    #     occupancy_grid_msg.header.stamp = node.get_clock().now().to_msg()
    #     publisher.publish(occupancy_grid_msg)
    #     rclpy.spin_once(node)

    # Publish the map data once
    occupancy_grid_msg.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(occupancy_grid_msg)
    rclpy.spin(node)

    # Shutdown the node and clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    publish_map_data()

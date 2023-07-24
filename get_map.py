import json
import os
import rclpy
from rclpy.node import Node
from your_package_name.msg import MapData

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(MapData, 'map_topic', 10)
        self.load_and_publish_map()

    def load_and_publish_map(self):
        file_path = '/path/to/your/json/file.json'
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return

        with open(file_path, 'r') as file:
            data = json.load(file)

            for obstacle in data.get('obstacles', []):
                map_data = MapData()
                map_data.x = obstacle.get('x', 0)
                map_data.y = obstacle.get('y', 0)
                map_data.width = obstacle.get('width', 0)
                map_data.height = obstacle.get('height', 0)
                self.publisher_.publish(map_data)

            for obstacle in data.get('waypoints', []):
                map_data = MapData()
                map_data.x = obstacle.get('x', 0)
                map_data.y = obstacle.get('y', 0)
                self.publisher_.publish(map_data)

            for obstacle in data.get('charging stations', []):
                map_data = MapData()
                map_data.x = obstacle.get('x', 0)
                map_data.y = obstacle.get('y', 0)
                map_data.width = obstacle.get('width', 0)
                map_data.height = obstacle.get('height', 0)
                self.publisher_.publish(map_data)

            # Similarly, publish charging stations and waypoints data...

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

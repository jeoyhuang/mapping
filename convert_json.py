import json
import yaml
from PIL import Image, ImageDraw

def convert_json_to_rviz(json_file, map_yaml_file, map_pgm_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    dimensions = data.get('dimensions',{}) # this won't be 
    obstacles = data.get('obstacles', [])
    charging_stations = data.get('charging stations', [])
    waypoints = data.get('waypoints', [])

    # Get the maximum x and y values to determine map size, default to 20
    max_x = dimensions['x'] if 'x' in dimensions else 20
    max_y = dimensions['y'] if 'y' in dimensions else 20

    # Convert JSON data to YAML format
    map_data = {
        'image': 'map.pgm',
        'resolution': 0.05,  # Adjust the resolution as needed
        'origin': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
        'negate': False,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    # Save map data as YAML file
    with open(map_yaml_file, 'w') as yaml_file:
        yaml.dump(map_data, yaml_file)

    # Create a blank PGM image
    width = max_x + 1
    height = max_y + 1
    img = Image.new('L', (width, height), color=255)
    draw = ImageDraw.Draw(img)

    # Draw obstacles, charging stations, and waypoints on the image
    for obstacle in obstacles:
        x1, y1 = obstacle['x'], obstacle['y']
        x2, y2 = x1 + obstacle['width'], y1 + obstacle['height']
        draw.rectangle([x1, y1, x2, y2], fill=0)

    for charging_station in charging_stations:
        x1, y1 = charging_station['x'], charging_station['y']
        x2, y2 = x1 + charging_station['width'], y1 + charging_station['height']
        draw.rectangle([x1, y1, x2, y2], fill=128)

    for waypoint in waypoints:
        x, y = waypoint['x'], waypoint['y']
        draw.ellipse([x - 5, y - 5, x + 5, y + 5], fill=192)

    # Save the image as PGM file
    img.save(map_pgm_file)

if __name__ == "__main__":
    json_file_path = "input.json"
    map_yaml_file_path = "map/output.yaml"
    map_pgm_file_path = "map/map.pgm"
    convert_json_to_rviz(json_file_path, map_yaml_file_path, map_pgm_file_path)

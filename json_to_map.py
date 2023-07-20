#!/usr/bin/env python

import os
import json
import yaml
import subprocess

def convert_json_to_yaml(json_file, yaml_file):
    with open(json_file, 'r') as file:
        data = json.load(file)
        with open(yaml_file, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

def save_map_as_pgm_and_yaml(yaml_file, map_name):
    # Save the YAML data as a .yaml file
    with open("{}.yaml".format(map_name), 'w') as outfile:
        outfile.write(yaml_file)

    # Call the map_saver tool from ROS 2 using subprocess
    cmd = ["/opt/ros/galactic/bin/ros2", "run", "nav2_map_server", "map_saver", "-f", map_name] # ros2 run nav2_map_server map_saver -f <map_name>
    subprocess.run(cmd)

if __name__ == "__main__":
    changepath = True
    ros_path = "" if changepath else "/opt/ros/galactic/bin"
    os.environ["PATH"] = ros_path + os.pathsep + os.environ["PATH"]
    os.environ["PYTHONPATH"] = ros_path + os.pathsep + os.environ.get("PYTHONPATH","")

    # Specify the path to your input JSON file
    json_file_path = 'make_map.json' # likely use argparse to get a file, but also provide a default

    # Specify the desired name for your map
    map_name = 'my_map' # this works, maybe we can change it to the json's file name?

    # Convert JSON to YAML
    with open(json_file_path, 'r') as json_file:
        json_data = json.load(json_file)
        yaml_data = yaml.dump(json_data, default_flow_style=False)

    # Save the YAML data as a .yaml file and create PGM and YAML files for the map
    save_map_as_pgm_and_yaml(yaml_data, map_name)

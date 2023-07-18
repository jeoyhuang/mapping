# mapping

Assume the map is written in map.json (map.py or any other) file, what you need to implement are: 
1. Develop an API for reading this kind of map information - let the robot know what the environment is.
2. Develop a localization based on lidar or camera - let the robot know where it is in this predefined environment
3. Write a planner function which should be like list[waypoints] = planner(map, robot_position, tasks), 
   here we implement different planning algorithms.










- Are there any map files I can look at as a reference?
Ideally all the map files should support RVIZ for visualization, so you can use the map format (I remember it contains .pgm 
and .yaml files) in RVIZ as the template. Then, you can write a function to convert it to any format you want for the planner. 

- Am I able to use ros2 related packages for anything I make? I found some helpful packages for both mapping and localization, 
but I wasn't sure if we wanted to use those. I'm also not entirely sure what I need to look for in a map so knowing whether 
or not to use a ros2 package or not would help.
Feel free to modify any code you want.
I suggest you work on some basic map functions first. 
For example, assuming we want to create a map with some obstacles and charging stations, 
you can have your own description about the map, like a json file:
{obstacles: {{x, y, width, height}, {...}, {...}}, charging stations: {{}, {}, {}}, etc...}
Then write a script to automatically generate the map format which RVIZ and the planner support. 
It will save you time when you try to modify the map for different environment testing. 

- For the planner function, does it return a list of waypoints in the order of priority based on our algorithm?
Yes, it will return a list of waypoints. 

- What would a waypoint look like in the code, or can I find out more through the previous team's code?
The waypoints are the lists of global coordinates (x, y) with the agv pose (heading angle).  You can ignore the pose part first. 




To create YAML and PGM files for a map from a JSON file in ROS 2 and RViz, you can follow these steps:

1. Convert JSON to YAML:
   - Install the PyYAML library using `pip install pyyaml`.
   - Write a Python script to convert the JSON file to YAML. Here's an example:

   ```python
   import yaml
   import json

   def convert_json_to_yaml(json_file, yaml_file):
       with open(json_file, 'r') as file:
           data = json.load(file)
           with open(yaml_file, 'w') as outfile:
               yaml.dump(data, outfile, default_flow_style=False)

   # Specify the paths to your input JSON and output YAML files
   json_file_path = 'input.json'
   yaml_file_path = 'output.yaml'

   # Call the conversion function
   convert_json_to_yaml(json_file_path, yaml_file_path)
   ```

   This script reads the JSON file, converts it to a Python object, and then dumps the object to a YAML file using the PyYAML library.

2. Convert YAML to PGM:
   - Install the map_server package in ROS 2 using `sudo apt-get install ros-<distro>-map-server`.
   - Place the generated YAML file (e.g., `output.yaml`) in your ROS workspace, specifically in the `maps/` folder.
   - Open a terminal and navigate to your ROS workspace.
   - Run the `rosrun map_server map_saver -f map_name` command, replacing `map_name` with the desired name for your map. This command generates both the PGM and YAML files for the map.

   For example:
   ```
   ros2 run map_server map_saver -f my_map
   ```

   This command will create `my_map.pgm` and `my_map.yaml` files in the current directory, representing the map converted from the YAML file.

Once you have the PGM and YAML files generated, you can use them in ROS 2 and RViz for mapping and visualization purposes.

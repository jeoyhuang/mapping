# mapping

To visualize the YAML files in RViz2, you'll need to create a ROS2 package and set up the necessary configuration for RViz2 to display the map data. Below are the steps to achieve this:

1. Create a ROS2 package:
   First, create a new ROS2 package to organize your files. Open a terminal and run the following command:

   ```bash
   ros2 pkg create my_map_visualization
   ```

2. Organize the files:
   Move the three YAML files (`obstacles.yaml`, `charging_stations.yaml`, and `waypoints.yaml`) that you created earlier into the `config` folder of your ROS2 package.

3. Create a launch file:
   In the package's `launch` folder, create a new launch file, e.g., `rviz2_map_visualization.launch.py`. This launch file will launch RViz2 and load the YAML files.

   ```python
   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration, Command
   from launch.launch_description_sources import PythonLaunchDescriptionSource

   def generate_launch_description():
       # Get the package path
       pkg_dir = get_package_share_directory('my_map_visualization')

       # Define launch arguments
       obstacles_file = LaunchConfiguration('obstacles_file', default=os.path.join(pkg_dir, 'config', 'obstacles.yaml'))
       charging_stations_file = LaunchConfiguration('charging_stations_file', default=os.path.join(pkg_dir, 'config', 'charging_stations.yaml'))
       waypoints_file = LaunchConfiguration('waypoints_file', default=os.path.join(pkg_dir, 'config', 'waypoints.yaml'))

       # RViz2 launch
       rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
       rviz_node = IncludeLaunchDescription(
           PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rviz2'), 'launch', 'rviz2.launch.py')),
           launch_arguments={
               'use_sim_time': 'false',
               'default_robot_description': Command(['bash', '-c', 'echo $(pwd)/fake_robot_description.urdf']),
               'rviz_config': rviz_config
           }.items(),
       )

       return LaunchDescription([
           DeclareLaunchArgument('obstacles_file', default_value=obstacles_file),
           DeclareLaunchArgument('charging_stations_file', default_value=charging_stations_file),
           DeclareLaunchArgument('waypoints_file', default_value=waypoints_file),

           rviz_node,
       ])

   if __name__ == '__main__':
       generate_launch_description()
   ```

4. Create an RViz2 configuration file:
   In the `config` folder, create a new RViz2 configuration file, e.g., `rviz_config.rviz`. This file will set up the visualization displays for the map data.

   For example, the contents of the `rviz_config.rviz` file may look like this:

   ```yaml
   global_options:
     fixed_frame: map

   map:
     name: Map
     topic: /map
     color_scheme: occupancy
     resolution: 0.1
   ```

5. Build the ROS2 package:
   Go to the root of your workspace and build the package:

   ```bash
   colcon build --packages-select my_map_visualization
   ```

6. Run the launch file:
   Open a terminal and run the following command to launch RViz2 with the map data:

   ```bash
   ros2 launch my_map_visualization rviz2_map_visualization.launch.py
   ```

   RViz2 should open and display the map with obstacles, charging stations, and waypoints based on the data in the YAML files.

Please note that the above steps assume you have already set up your ROS2 workspace, and RViz2 is properly installed. If not, please make sure to install the required packages and set up your workspace accordingly before following the steps above.
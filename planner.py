import json
import yaml
import os.path

def readjson():
  f = open("make_map.json")
  # data = json.load(f)
  return json.load(f)

# create a yaml file from json
def json_to_map(json_file, yaml_file):
  with open(json_file, 'r') as file:
        data = json.load(file)
        with open(yaml_file, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)



# Sorting key function
def priority_key(dictionary):
    return not dictionary.get("priority", False)

# theorectically map is made from the json file
# pos should just be an (x,y) of current position
# tasks include pick up location and deliver location, and priority (likely a boolean?)
# example task: {"pickup" : [x1,y1], "deliver" : [x2,y2], "priority" : true/false}
def waypoints(map, pos, tasks):
  waypoints = []

  # Sort the list based on the priority key
  # tasks = sorted(tasks, key=priority_key)

  # pick out the tasks and list them as waypoints based on algorithm
  # add to waypoints list



  return waypoints
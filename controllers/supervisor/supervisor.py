from controller import Supervisor
import requests

supervisor = Supervisor()
TIME_STEP = 1

robot_def_names = ["EPUCK1", "EPUCK2", "EPUCK3"]
URL = "http://127.0.0.1:4500/"
# Fetch initial positions from the server
# Attempt to fetch initial positions from the server
try:
    response = requests.get(url=URL + "getposition")
    response.raise_for_status()  # Raise an error for bad status codes
    robot_position = response.json()
except requests.exceptions.RequestException as e:
    print(f"Error fetching initial positions: {e}")
    robot_position = [] 

# Set initial positions of robotsa
for robot in robot_position:
    robot_node = supervisor.getFromDef(robot['name'])
    if robot_node:
        position_field = robot_node.getField("translation")
        position = position_field.getSFVec3f()
        _, _, z = position
        new_position = robot['position']
        position_field.setSFVec3f(
            [float(new_position[0]), float(new_position[1]), float(0)]
        )
        rotation_field = robot_node.getField("rotation")
        rotation = rotation_field.setSFRotation([0.0, 0.0, 1.0, float(new_position[2])])
        # Assuming rotation is around z-axis (axis-angle representation)
print("initial position set", robot_position)


while supervisor.step(TIME_STEP) != -1:
    robot_data = []
    for def_name in robot_def_names:
        robot_node = supervisor.getFromDef(def_name)
        if robot_node:
            # Get position
            position_field = robot_node.getField("translation")
            position = position_field.getSFVec3f()
            x, y, _ = position  # Ignoring z-coordinate

            # Get rotation and convert to angle with respect to z-axis
            rotation_field = robot_node.getField("rotation")
            rotation = rotation_field.getSFRotation()
            # Assuming rotation is around z-axis (axis-angle representation)
            _, _, _, angle = rotation  # Extracting angle

            robot_data.append({"x": x, "y": y, "angle": angle})

    # Prepare parameters for the request
    PARAMS = {}
    for i, data in enumerate(robot_data):
        PARAMS[f"EPUCK{i+1}_x"] = data["x"]
        PARAMS[f"EPUCK{i+1}_y"] = data["y"]
        PARAMS[f"EPUCK{i+1}_angle"] = data["angle"]

    # Send request
    if robot_data:
        req = requests.get(url=URL + "updateposition", params=PARAMS)

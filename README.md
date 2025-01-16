# 2D Autonomous Robot

This project simulates a 2D autonomous robot that aims to reach the closest targets in a smooth manner. Once the targets are reached, they are "caught" and killed. The system uses a publisher-subscriber and client-server architecture for interaction between nodes.

### Project Structure

The project consists of three main ROS nodes:

1. **TurtleControllerNode**
   - This node controls the movement of the main robot (turtle1).
   - It subscribes to the pose of the turtle, calculates the closest target turtle, and moves the main robot towards the target using a closed-loop control via Proportional (P) control.
   - Once the target is reached, it sends a request to the `SpawnTurtleNode` via a service to "catch" the turtle.

2. **SpawnTurtleNode**
   - This node is responsible for spawning new turtles at random locations within the simulation environment.
   - It periodically spawns turtles and publishes their status using the `alive_turtles` topic.
   - It also provides a service to "catch" and "kill" a turtle, which is triggered by the `TurtleControllerNode`.

3. **TurtlesimNode**
   - The `turtlesim_node` is the standard ROS 2 simulator node used to visualize the robot's movements. It also provides basic turtle functionalities like spawning, moving, and killing turtles.

### Architecture Overview

- **TurtleControllerNode** interacts with `TurtlesimNode` using a **Publisher-Subscriber** architecture. It subscribes to the pose of the main turtle and publishes velocity commands to control its movement towards the closest turtle.
  
- **TurtleControllerNode** interacts with `SpawnTurtleNode` using both a **Publisher-Subscriber** and **Client-Server** architecture:
  - It subscribes to the `alive_turtles` topic to keep track of the live turtles.
  - It makes requests to the `catch_turtle` service to "catch" a turtle once the main turtle reaches it.

- **SpawnTurtleNode** acts as both a **Server** (handling `catch_turtle` service requests) and a **Client** (sending requests to the `turtlesim` service to spawn and kill turtles).

### Components

1. **Custom Messages:**
   - `Turtle.msg` - Defines the structure for a turtle (its position, name, etc.).
   - `TurtleArray.msg` - Defines an array of `Turtle` messages to represent all alive turtles.
   - `CatchTurtle.srv` - A service that allows the `TurtleControllerNode` to request that a specific turtle be caught and removed from the simulation.

2. **Node Communication:**
   - **TurtleControllerNode** subscribes to:
     - `/turtle1/pose` - The pose of the main turtle (robot).
     - `/alive_turtles` - The list of currently alive turtles.
   - **TurtleControllerNode** publishes to:
     - `/turtle1/cmd_vel` - Velocity commands to move the main turtle.
   - **SpawnTurtleNode**:
     - Subscribes to requests from `TurtleControllerNode` to catch turtles.
     - Publishes updates on alive turtles via the `alive_turtles` topic.
     - Provides the `/catch_turtle` service to allow `TurtleControllerNode` to request catching a specific turtle.

### Launch File

The launch file starts all the necessary nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Start turtlesim_node (the simulation)
    turtlesim_node = Node(
        package="turtlesim",  # Package name
        executable="turtlesim_node",  # Executable name
        name="turtlesim_node",  # Node name
        output="screen",  # Print logs to screen
    )

    # Define the turtle_controller node
    turtle_controller = Node(
        package="my_turtle_controller",  # Package name
        executable="turtle_controller",  # Executable name
        name="turtle_controller",        # Node name
        parameters=[
            {"catch_closest_turtle_first": True},  # Parameter for closest turtle
        ]
    )

    # Define the spawn_turtle node
    spawn_turtle = Node(
        package="my_turtle_controller",  # Package name
        executable="spawn_turtle",       # Executable name
        name="spawn_turtle",             # Node name
        parameters=[
            {"spawn_frequency": 2},  # Set the spawn frequency to 2 seconds
            {"turtle_name_prefix": 'turtle'}  # Prefix for turtle names
        ]
    )

    # Add all actions to the launch description
    ld.add_action(turtlesim_node)  # Launch turtlesim_node
    ld.add_action(turtle_controller)  # Launch turtle_controller
    ld.add_action(spawn_turtle)  # Launch spawn_turtle

    return ld

# How to Run the Project

# 1. Make sure you have ROS 2 (e.g., Foxy, Galactic, or Humble) installed.
# 2. Navigate to your ROS 2 workspace directory:
cd ~/ros2_ws

# 3. Clone the repository containing the project files:
git clone <URL-to-repository>

# 4. Build the workspace:
colcon build --symlink-install

# 5. Source the workspace:
source install/setup.bash

# 6. Launch the system using the launch file:
ros2 launch my_turtle_controller spawn_and_control.launch.py

# This will start:
# - turtlesim_node (the simulation environment)
# - turtle_controller node (to control the main turtle to reach closest targets)
# - spawn_turtle node (to spawn and kill turtles at a specified frequency)

# 7. The robot will automatically start moving toward the closest turtle. When caught, the turtle will be killed.
# You can observe the progress through ROS 2 logs.

# Parameters

# These are the parameters that can be configured for each node:

# 1. turtle_controller node:
#    - catch_closes_turtle_first (default: True)  
#      Determines whether the robot should catch the closest turtle first or the first turtle in the list.

# 2. spawn_turtle node:
#    - spawn_frequency (default: 2)
#      Defines how often new turtles are spawned (in seconds).
#    - turtle_name_prefix (default: "turtle")
#      Prefix for the names of the spawned turtles.

# Example of launching the nodes with specific parameters:

ros2 launch my_turtle_controller spawn_and_control.launch.py catch_closes_turtle_first:=false spawn_frequency:=3 turtle_name_prefix:="my_turtle"

# Services

# The following services are used in this project:

# 1. /catch_turtle (service server in spawn_turtle node):
#    - Request: A custom message with the turtle name.
#    - Response: Confirms whether the turtle was successfully caught and removed.
#    - Used by: The turtle_controller node when a turtle is caught to request its removal.

# 2. /spawn (service client in spawn_turtle node):
#    - Request: The x, y, and theta positions to spawn the turtle in the turtlesim simulation.
#    - Response: Confirmation of the spawned turtle with its assigned name.

# 3. /kill (service client in spawn_turtle node):
#    - Request: The name of the turtle to be killed.
#    - Response: Confirmation that the turtle was killed.

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

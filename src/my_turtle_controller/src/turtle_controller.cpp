#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"  // Include service definition
#include <vector>  // For using std::vector
#include <cmath>   // For atan2, fabs, sqrt

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        // Declare and initialize the catch_closes_turtle_first parameter to true
        this->declare_parameter<bool>("catch_closes_turtle_first", true);
        this->get_parameter("catch_closes_turtle_first", catch_closes_turtle_first);

        // Subscriber to the alive_turtles topic
        turtles_subscriber = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
            "/alive_turtles", 10,
            std::bind(&TurtleControllerNode::AliveTurtlesCallback, this, std::placeholders::_1));

        // Publisher to control turtle movement
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // Subscription to get the pose of the turtle
        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleControllerNode::poseCallback, this, std::placeholders::_1));

        target_reached = false;    // Flag to indicate whether the target is reached
    }

private:
    // Callback for the alive_turtles topic
    void AliveTurtlesCallback(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        // Store the turtles' coordinates in the alive_turtles vector
        alive_turtles.clear(); // Clear previous data
        for (const auto &turtle : msg->turtles) {
            alive_turtles.push_back(turtle);  // Store each turtle in the vector
        }

        // Log the updated list of turtles
        RCLCPP_INFO(this->get_logger(), "Stored %zu turtles", alive_turtles.size());
    }

    // Callback for the pose topic
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        if (alive_turtles.empty()) {
            RCLCPP_WARN(this->get_logger(), "No turtles to catch.");
            return;
        }

        my_robot_interfaces::msg::Turtle target_turtle;

        if (catch_closes_turtle_first) {
            // Find the closest turtle if the parameter is true
            target_turtle = FindClosestTurtle(msg);
        } else {
            // Catch the first turtle in the array if catch_closes_turtle_first is false
            target_turtle = alive_turtles[0];
        }

        // Create and initialize a command message to move the turtle
        auto cmd = geometry_msgs::msg::Twist();

        double delta_x = target_turtle.x - msg->x;
        double delta_y = target_turtle.y - msg->y;
        double theta = atan2(delta_y, delta_x);  // Calculate the angle to the target turtle

        // Calculate the angular error and ensure it's within the range [-pi, pi]
        double angular_error = theta - msg->theta;
        while (angular_error > M_PI) angular_error -= 2 * M_PI;
        while (angular_error < -M_PI) angular_error += 2 * M_PI;

        cmd.angular.z = 5.0 * atan2(sin(angular_error), cos(angular_error));  // Proportional controller for angular movement

        // If the angular error is small, move towards the target
        if (fabs(angular_error) < 0.1) {
            cmd.linear.x = 2.0 * sqrt(delta_x * delta_x + delta_y * delta_y);  // Move forward based on the distance
        }
        else {
            cmd.linear.x = 0.5;  // Stop moving forward if the angle isn't right
        }

        publisher->publish(cmd);

        // Check if the main turtle has reached the closest target
        if (sqrt(delta_x * delta_x + delta_y * delta_y) < 0.2) {
            target_reached = true;  // The target has been reached
            // Remove the target turtle from the alive_turtles vector
            auto it = std::find(alive_turtles.begin(), alive_turtles.end(), target_turtle);
            if (it != alive_turtles.end()) {
                alive_turtles.erase(it);  // Erase the caught turtle from the list
                RCLCPP_INFO(this->get_logger(), "Caught and removed turtle from the list.");
                
                // Send a request to the /catch_turtle service to catch the turtle
                SendCatchTurtleRequest(target_turtle.name);
            }
        }

        // If the current target is reached, reset and prepare for the next target
        if (target_reached) {
            target_reached = false;  // Reset the flag for the next target
        }
    }

    // Function to send the catch_turtle request
    void SendCatchTurtleRequest(const std::string &turtle_name)
    {
        // Create a service client for the /catch_turtle service
        catch_turtle_client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("/catch_turtle");

        // Wait for the service to be available
        while (!catch_turtle_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /catch_turtle service...");
        }

        // Create the service request and set the turtle name
        auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        request->turtle_name = turtle_name;
        
        // Asynchronous service call using a callback
        catch_turtle_client->async_send_request(
            request,
            [this, turtle_name](rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedFuture future)
            {
                try {
                    auto result = future.get(); // Get the result asynchronously
                    RCLCPP_INFO(this->get_logger(), "Successfully caught turtle: %s", turtle_name.c_str());
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }

    // Find the closest turtle
    my_robot_interfaces::msg::Turtle FindClosestTurtle(const turtlesim::msg::Pose::SharedPtr msg)
    {
        my_robot_interfaces::msg::Turtle closest_turtle = alive_turtles[0];
        double min_distance = sqrt(pow(closest_turtle.x - msg->x, 2) + pow(closest_turtle.y - msg->y, 2));

        for (const auto &turtle : alive_turtles) {
            double distance = sqrt(pow(turtle.x - msg->x, 2) + pow(turtle.y - msg->y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest_turtle = turtle;
            }
        }
        return closest_turtle;
    }

    // Declare the catch_closes_turtle_first parameter and its value
    bool catch_closes_turtle_first;

    // Subscriber to alive_turtles topic
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr turtles_subscriber;

    // Vector to store the alive turtles' data (coordinates, names, etc.)
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles;

    // Publisher for controlling turtle
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

    // Subscriber for turtle pose
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;

    // Client for catching turtles
    rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client;

    // Flag to track whether target is reached
    bool target_reached;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp" 
#include <functional>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>  // For std::find_if

class SpawnTurtleNode : public rclcpp::Node
{
public:
    SpawnTurtleNode() : Node("turtle_spawner"), count(2)
    {
        // Declare the parameters and get their values
        this->declare_parameter<int>("spawn_frequency", 2);  // Default is 2 seconds
        this->declare_parameter<std::string>("turtle_name_prefix", "turtle");  // Default prefix is "turtle"
        
        this->get_parameter("spawn_frequency", spawn_frequency);
        this->get_parameter("turtle_name_prefix", turtle_name_prefix);

        // Create service server for catching turtles
        catch_turtle_service = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "/catch_turtle", std::bind(&SpawnTurtleNode::handleCatchTurtle, this, std::placeholders::_1, std::placeholders::_2));

        // Timer for spawning turtles at the specified frequency
        timer = this->create_wall_timer(std::chrono::seconds(spawn_frequency), std::bind(&SpawnTurtleNode::callSpawnService, this));

        // Publisher to notify about alive turtles
        publisher = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10);
    }

private:
    void publishAliveTurtles(const float x, float y, std::string &name)
    {
        auto new_turtle = my_robot_interfaces::msg::Turtle();
        new_turtle.x = x;
        new_turtle.y = y;
        new_turtle.name = name;
        alive_turtles.turtles.push_back(new_turtle); // Add to the alive turtles list
        publisher->publish(alive_turtles);  // Publish the updated list
    }

    // Modified asynchronous kill service call
    void callKillService(const std::string &name)
    {
        auto client = this->create_client<turtlesim::srv::Kill>("kill");

        // Wait for the service to be available
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Kill Service Server to be up...");
        }

        // Create the request for the kill service
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = name;

        // Asynchronous service call using a callback
        client->async_send_request(request, 
            [this, name](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
            {
                try
                {
                    auto response = future.get(); // Wait for the response asynchronously
                    RCLCPP_INFO(this->get_logger(), "Turtle %s killed successfully.", name.c_str());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Kill Service call failed for %s: %s", name.c_str(), e.what());
                }
            });
    }

    void callSpawnService()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 10.0);
        std::uniform_real_distribution<> theta_dis(0.0, 6.28);

        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        if (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Spawn Service Server to be up...");
            return;
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = dis(gen);
        request->y = dis(gen);
        request->theta = theta_dis(gen);
        
        // Use the turtle_name_prefix and increment the count for each new turtle
        std::string turtle_name = turtle_name_prefix + std::to_string(count);
        request->name = turtle_name;

        RCLCPP_INFO(this->get_logger(), "Sending spawn request for turtle: %s", turtle_name.c_str());

        // Asynchronous call to spawn service
        client->async_send_request(request, 
            [this, turtle_name](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
            {
                try
                {
                    auto result = future.get();
                    RCLCPP_INFO(this->get_logger(), "Turtle spawned successfully. Name: %s", result->name.c_str());
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            });

        count++;
        publishAliveTurtles(request->x, request->y, request->name);
    }

    // Handle the catch_turtle service
    void handleCatchTurtle(
        const std::shared_ptr<my_robot_interfaces::srv::CatchTurtle::Request> request,
        std::shared_ptr<my_robot_interfaces::srv::CatchTurtle::Response> response)
    {
        const std::string &turtle_name = request->turtle_name;

        // Search for the turtle in the alive_turtles array
        auto it = std::find_if(alive_turtles.turtles.begin(), alive_turtles.turtles.end(),
                               [&turtle_name](const my_robot_interfaces::msg::Turtle &turtle) {
                                   return turtle.name == turtle_name;
                               });

        if (it != alive_turtles.turtles.end()) {
            RCLCPP_INFO(this->get_logger(), "Turtle %s caught, removing it from the list.", turtle_name.c_str());

            // Remove the caught turtle from the list
            alive_turtles.turtles.erase(it);

            // Call kill service asynchronously
            callKillService(turtle_name);
        } else {
            RCLCPP_WARN(this->get_logger(), "Turtle %s not found.", turtle_name.c_str());
        }
    }

    int count;
    int spawn_frequency;  // Frequency of turtle spawning (in seconds)
    std::string turtle_name_prefix;  // Prefix for turtle names
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr publisher;
    my_robot_interfaces::msg::TurtleArray alive_turtles; // Store alive turtles
    rclcpp::TimerBase::SharedPtr timer;

    // Service server for /catch_turtle
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_service;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnTurtleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf_model/model.h"
#include "urdf_parser/urdf_parser.h"
#include "queue"
#include "fstream"

class TrajectoryRepublisher : public rclcpp::Node
{
public:
    TrajectoryRepublisher() : Node("visualizer_trajectory_republisher")
    {
        publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);

        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter("trajectory_file", "");

        extract_joint_names();

        timer = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                RCLCPP_INFO(this->get_logger(), "Statrting trajectory publishing....");
                this->timer->cancel();
                this->publish_trajectory(this->parse_trajectory());
            });
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    std::vector<std::string> joint_names{};
    rclcpp::Clock clock{RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr timer;

    void extract_joint_names()
    {
        std::string robot_description;
        this->get_parameter("robot_description", robot_description);
        ::urdf::ModelInterfaceSharedPtr const urdfTree = ::urdf::parseURDF(robot_description);
        std::queue<::urdf::LinkConstSharedPtr> link_queue{};

        for (auto const &link : urdfTree->getRoot()->child_links)
        {
            link_queue.push(link);
        }

        while (!link_queue.empty())
        {
            ::urdf::LinkConstSharedPtr const link = link_queue.front();
            link_queue.pop();
            for (auto const &child_link : link->child_links)
            {
                link_queue.push(child_link);
            }
            joint_names.push_back(link->parent_joint->name);
        }
    }

    std::vector<std::pair<double, std::vector<double>>> parse_trajectory()
    {
        std::string trajectory_file_name;
        this->get_parameter("trajectory_file", trajectory_file_name);

        RCLCPP_INFO(this->get_logger(), "Path to trajectory file %s", trajectory_file_name.c_str());

        std::vector<std::pair<double, std::vector<double>>> trajectory;
        std::ifstream file(trajectory_file_name);
        if (!file.is_open())
            throw std::runtime_error("Unable to open trajectory file");

        std::string line;
        while (std::getline(file, line))
        {
            trajectory.push_back(parse_line(line));
        }

        return trajectory;
    }

    std::pair<double, std::vector<double>> parse_line(std::string const &line)
    {
        std::stringstream ss(line);

        std::string timestamp_str, value_str;
        double timestamp;
        std::getline(ss, timestamp_str, ',');
        std::istringstream(timestamp_str) >> timestamp;

        std::vector<double> state;
        double value;
        while (ss >> value)
        {
            state.push_back(value);
        }

        return {timestamp, state};
    }

    void publish_trajectory(std::vector<std::pair<double, std::vector<double>>> const &trajectory)
    {
        if (trajectory.empty())
            return;

        double simulation_time = round(trajectory.back().first);
        double elapsed = 0;
        int idx = 0;
        auto start = std::chrono::high_resolution_clock::now();

        while (elapsed <= simulation_time && idx < trajectory.size())
        {
            auto now = std::chrono::high_resolution_clock::now();
            elapsed = std::chrono::duration<double>(now - start).count();
            double stamp = trajectory[idx].first;
            std::vector<double> state = trajectory[idx].second;

            if (elapsed >= stamp)
            {
                publish_state(state);
                idx++;
            }
        }
    }

    void publish_state(std::vector<double> const &state)
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = clock.now();
        msg.name = std::vector<std::string>(joint_names.begin(), joint_names.begin() + state.size());
        msg.position = state;
        msg.velocity = std::vector<double>(state.size(), 0);
        msg.effort = std::vector<double>(state.size(), 0);
        publisher->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

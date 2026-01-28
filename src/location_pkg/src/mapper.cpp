#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fstream>
#include <iomanip>
#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;
using std::placeholders::_1;

class MapperNode : public rclcpp::Node
{
public:
    MapperNode() : Node("mapper_node")
    {
        this->declare_parameter<std::string>("output_file", "robot_path.csv");
        std::string filename = this->get_parameter("output_file").as_string();
        
        fs::path data_dir = fs::path(std::getenv("HOME")) / "autonomous_robot_localization" / "data";
        if (!fs::exists(data_dir))
        {
            fs::create_directories(data_dir);
            RCLCPP_INFO(this->get_logger(), "Created directory: %s", data_dir.string().c_str());
        }

        fs::path filepath = data_dir / filename;
        if (fs::exists(filepath))
        {
            auto now = std::chrono::system_clock::now();
            auto timestamp = 
                std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            
            std::string old_filename = "old_" + filename.substr(0, filename.find_last_of('.')) 
                                     + "_" + std::to_string(timestamp) + ".csv";
            fs::path old_filepath = data_dir / old_filename;
            
            fs::rename(filepath, old_filepath);
            RCLCPP_INFO(this->get_logger(), "Existing file renamed to: %s", old_filepath.string().c_str());
        }

        csv_file_.open(filepath, std::ios::out);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filepath.c_str());
            rclcpp::shutdown();
            return;
        }

        csv_file_ << "timestamp,x,y,theta" << std::endl;

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "odometry/position", 10, std::bind(&MapperNode::poseCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Mapper node started - Logging to: %s", filepath.c_str());
    }

    ~MapperNode()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed. Total poses logged: %zu", pose_count_);
        }
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::ofstream csv_file_;
    size_t pose_count_{0};
};

void MapperNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Extraer timestamp
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // Extraer x, y
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    // Extraer orientación en Z (yaw) desde el quaternion
    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;

    // Convertir quaternion a yaw (rotación en Z)
    double theta = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));

    // Escribir al CSV
    csv_file_ << std::fixed << std::setprecision(6)
                << timestamp << ","
                << x << ","
                << y << ","
                << theta << std::endl;

    pose_count_++;

    // Log cada 100 poses
    if (pose_count_ % 100 == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Logged %zu poses", pose_count_);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}
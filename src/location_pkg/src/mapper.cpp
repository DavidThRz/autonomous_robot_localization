#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fstream>
#include <iomanip>
#include <cmath>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;
using std::placeholders::_1;

struct Pose 
{
    double timestamp;
    double x;
    double y;
    double theta;
};

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

        file_path_ = data_dir / filename;
        if (fs::exists(file_path_))
        {
            auto now = std::chrono::system_clock::now();
            auto timestamp = 
                std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            
            std::string old_filename = "old_" + filename.substr(0, filename.find_last_of('.')) 
                                     + "_" + std::to_string(timestamp) + ".csv";
            fs::path old_filepath = data_dir / old_filename;
            
            fs::rename(file_path_, old_filepath);
            RCLCPP_INFO(this->get_logger(), "Existing file renamed to: %s", old_filepath.string().c_str());
        }

        fs::path map_path = data_dir / "robot_path_map.png";
        if (fs::exists(map_path))
        {
            fs::remove(map_path);
            RCLCPP_INFO(this->get_logger(), "Removed existing map file: %s", map_path.string().c_str());
        }

        csv_file_.open(file_path_, std::ios::out);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", file_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        csv_file_ << "timestamp,x,y,theta" << std::endl;

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "odometry/position", 10, std::bind(&MapperNode::poseCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Mapper node started - Logging to: %s", file_path_.c_str());
    }

    ~MapperNode()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed. Total poses logged: %zu", pose_count_);

            generateMap();
        }
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void generateMap();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::ofstream csv_file_;
    size_t pose_count_{0};
    fs::path file_path_;
};

void MapperNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;

    double theta = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));

    csv_file_ << std::fixed << std::setprecision(6)
                << timestamp << ","
                << x << ","
                << y << ","
                << theta << std::endl;

    pose_count_++;

    if (pose_count_ % 100 == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Logged %zu poses", pose_count_);
    }
}

void MapperNode::generateMap()
{
    /* Map allignment design */
    #define IMG_HEIGHT      1000
    #define IMG_WIDTH       1000
    #define MARGIN_RIGHT    50
    #define MARGIN_TOP      100
    #define MARGIN_LEFT     110
    #define MARGIN_BOTTOM   65
    const int title_space = MARGIN_TOP - 20;
    const int drawable_x = IMG_WIDTH - MARGIN_LEFT - MARGIN_RIGHT;
    const int drawable_y = IMG_HEIGHT - MARGIN_TOP - MARGIN_BOTTOM;

    /* Read poses from .csv */
    std::ifstream in_file(file_path_.string());
    std::vector<Pose> poses;
    Pose pose;
    std::cout << "Generating map from logged poses..." << std::endl;

    std::string line;
    while (std::getline(in_file, line))
    {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string token;

        try
        {
            std::getline(ss, token, ','); pose.timestamp = std::stod(token);
            std::getline(ss, token, ','); pose.x         = std::stod(token);
            std::getline(ss, token, ','); pose.y         = std::stod(token);
            std::getline(ss, token, ','); pose.theta     = std::stod(token);
            poses.push_back(pose);
        }
        catch (const std::exception& e)
        {
            continue;
        }
    }

    if (poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No poses to generate map.");
        return;
    }

    /* Calculate limits */
    double min_x = poses[0].x, max_x = poses[0].x;
    double min_y = poses[0].y, max_y = poses[0].y;
    double duration = poses.back().timestamp - poses.front().timestamp;
    double total_distance = 0.0;

    for (size_t i = 1; i < poses.size(); ++i)
    {
        min_x = std::min(min_x, poses[i].x);
        max_x = std::max(max_x, poses[i].x);
        min_y = std::min(min_y, poses[i].y);
        max_y = std::max(max_y, poses[i].y);

        double dx = poses[i].x - poses[i-1].x;
        double dy = poses[i].y - poses[i-1].y;
        total_distance += std::hypot(dx, dy);
    }

    const double range_x = max_x - min_x;
    const double range_y = max_y - min_y;

    const double off_x = (range_x > range_y) ? 0.0 : (range_y - range_x) / 2.0;
    const double off_y = (range_x > range_y) ? (range_x - range_y) / 2.0 : 0.0;
    const double scale = (range_x > range_y) ? range_x : range_y;

    std::cout << "======= RESUMEN DE TRAYECTORIA =======" << std::endl;
    std::cout << "  Poses totales   : " << poses.size()                          << std::endl;
    std::cout << "  Duración        : " << duration       << " s"                << std::endl;
    std::cout << "  Distancia total : " << total_distance << " px"                << std::endl;
    std::cout << "  Rango X         : [" << min_x << ", " << max_x << "] px"      << std::endl;
    std::cout << "  Rango Y         : [" << min_y << ", " << max_y << "] px"      << std::endl;
    std::cout << "  Pose final      : (" << poses.back().x  << ", "
                                         << poses.back().y  << ") θ="
                                         << poses.back().theta  << " rad"        << std::endl;
    std::cout << "======================================" << std::endl;

    /* Prepare canvas */
    cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
    
    auto toPixel = [&](double x, double y) -> cv::Point 
    {
        int px = static_cast<int>(MARGIN_LEFT + (x - min_x + off_x) / scale * drawable_x);
        int py = static_cast<int>(IMG_HEIGHT - MARGIN_BOTTOM - (y - min_y + off_y) / scale * drawable_y);
        return {px, py};
    };

    /* Draw grid */
    const int num_divisions = 10;

    cv::Scalar grid_color(220, 220, 220);
    for (int i = 0; i <= num_divisions; ++i) 
    {
        int offset_x = MARGIN_LEFT + i * drawable_x / num_divisions;
        int offset_y = MARGIN_TOP + i * drawable_y / num_divisions;

        cv::line(img, {offset_x, MARGIN_TOP}, {offset_x, IMG_HEIGHT - MARGIN_BOTTOM}, grid_color, 1);
        cv::line(img, {MARGIN_LEFT, offset_y}, {IMG_WIDTH - MARGIN_RIGHT, offset_y}, grid_color, 1);

        double world_x = min_x + (double)i / num_divisions * scale - off_x;
        std::ostringstream label_x;
        label_x << std::fixed << std::setprecision(3) << world_x << "px";
        cv::putText(img, label_x.str(),
                    {offset_x - 18, IMG_HEIGHT - MARGIN_BOTTOM + 20},
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, {80, 80, 80}, 1, cv::LINE_AA);

        double world_y = min_y + (1.0 - (double)i / num_divisions) * scale - off_y;
        std::ostringstream label_y;
        label_y << std::fixed << std::setprecision(3) << world_y << "px";
        cv::putText(img, label_y.str(),
                    {40, offset_y + 4},
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, {80, 80, 80}, 1, cv::LINE_AA);
    }

    int px_0 = static_cast<int>(MARGIN_LEFT - min_x / scale * drawable_x + off_x / scale * drawable_x);
    int py_0 = static_cast<int>(IMG_HEIGHT - MARGIN_BOTTOM + min_y / scale * drawable_y - off_y / scale * drawable_y);
    cv::line(img, {px_0, MARGIN_TOP}, {px_0, IMG_HEIGHT - MARGIN_BOTTOM}, grid_color, 2);
    cv::line(img, {MARGIN_LEFT, py_0}, {IMG_WIDTH - MARGIN_RIGHT, py_0}, grid_color, 2);

    cv::rectangle(img, 
                {MARGIN_LEFT, MARGIN_TOP}, {IMG_WIDTH - MARGIN_RIGHT, IMG_HEIGHT - MARGIN_BOTTOM},
                cv::Scalar(150, 150, 150), 1);

    cv::putText(img, "X (px)", {IMG_WIDTH / 2 - 15, IMG_HEIGHT - 17},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {50, 50, 50}, 1, cv::LINE_AA);

    cv::Mat y_label(30, 60, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::putText(y_label, "Y (px)", {0, 20},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {50, 50, 50}, 1, cv::LINE_AA);
    cv::Mat y_rotated;
    cv::rotate(y_label, y_rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
    y_rotated.copyTo(img(cv::Rect(0, IMG_HEIGHT / 2 - 30, y_rotated.cols, y_rotated.rows)));

    /* Trayectory */
    double t0 = poses.front().timestamp;
    double t1 = poses.back().timestamp;
    double dt = (t1 - t0) < 1e-6 ? 1.0 : (t1 - t0);

    for (size_t i = 1; i < poses.size(); ++i) 
    {
        double t = (poses[i].timestamp - t0) / dt;
        cv::Scalar color(
            255 * (1.0 - t),
            80,
            255 * t
        );
        cv::line(img, toPixel(poses[i-1].x, poses[i-1].y),
                      toPixel(poses[i].x,   poses[i].y),
                      color, 2, cv::LINE_AA);
    }

    /* Orientation */
    const int arrow_step = std::max(1, static_cast<int>(poses.size()) / 80);
    const double arrow_len = scale * 0.03;

    for (size_t i = 0; i < poses.size(); i += arrow_step) 
    {
        cv::Point base = toPixel(poses[i].x, poses[i].y);
        cv::Point tip  = toPixel(poses[i].x + arrow_len * std::cos(poses[i].theta),
                                 poses[i].y + arrow_len * std::sin(poses[i].theta));
        cv::arrowedLine(img, base, tip, cv::Scalar(0, 140, 255), 1, cv::LINE_AA, 0, 0.3);
    }

    /* Init and end poses */
    cv::circle(img, toPixel(poses.front().x, poses.front().y), 8, cv::Scalar(200, 0, 0),  -1, cv::LINE_AA);
    cv::circle(img, toPixel(poses.back().x,  poses.back().y),  8, cv::Scalar(0, 0, 200),  -1, cv::LINE_AA);

    /* Legend */
    const int bar_y = title_space / 2;
    const int legend_init_x = IMG_WIDTH - 220;
    const int legend_end_x = legend_init_x + 80;

    cv::rectangle(img, {0, 0}, {IMG_WIDTH, title_space},
            cv::Scalar(235, 235, 205), -1);
    cv::line(img, {0, title_space}, {IMG_WIDTH, title_space},
            cv::Scalar(200, 200, 200), 1);

    cv::putText(img,
            "Trayectoria (" + std::to_string(poses.size()) + " poses)",
            {140, bar_y + 10},
            cv::FONT_HERSHEY_SIMPLEX, 0.75, {50, 50, 50}, 1, cv::LINE_AA);

    cv::circle(img, {legend_init_x, bar_y}, 7, cv::Scalar(200, 0, 0), -1, cv::LINE_AA);
    cv::putText(img, "Inicio", {legend_init_x + 12, bar_y + 5},
            cv::FONT_HERSHEY_SIMPLEX, 0.45, {50, 50, 50}, 1, cv::LINE_AA);

    cv::circle(img, {legend_end_x, bar_y}, 7, cv::Scalar(0, 0, 200), -1, cv::LINE_AA);
    cv::putText(img, "Fin", {legend_end_x + 12, bar_y + 5},
            cv::FONT_HERSHEY_SIMPLEX, 0.45, {50, 50, 50}, 1, cv::LINE_AA);

    /* Save image */
    std::string output_path = file_path_.parent_path().string() + "/robot_path_map.png";
    cv::imwrite(output_path, img);
    std::cout << "[INFO] Mapa guardado en: " << output_path << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}
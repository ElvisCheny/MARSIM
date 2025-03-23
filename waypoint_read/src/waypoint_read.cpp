// ChenYang
// 2025/02/27
// Read waypoints from a file and publish them as goals
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <waypoint_msgs/WaypointWithVelocity.h>
#include <ros/console.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstddef>
#include <tf/tf.h>

// 定义带速度信息的航点结构体
struct WaypointWithVel {
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Vector3 velocity;
};

// 从文件中读取带速度信息的航点
std::vector<WaypointWithVel> loadWaypointsFromFile(const std::string& filename) {
    std::vector<WaypointWithVel> waypoints;
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        double x, y, z, qx, qy, qz, qw, vx, vy, vz;
        
        // 尝试读取位置、方向和速度
        if (!(ss >> x >> y >> z >> qx >> qy >> qz >> qw >> vx >> vy >> vz)) {
            // 如果读取失败，可能是旧格式，只尝试读取位置和方向
            std::istringstream ss2(line);
            if (!(ss2 >> x >> y >> z >> qx >> qy >> qz >> qw)) {
                ROS_WARN("Invalid line format: %s", line.c_str());
                continue;
            }
            // 旧格式文件中没有速度信息，设为0
            vx = vy = vz = 0.0;
        }
        
        WaypointWithVel waypoint;
        waypoint.pose.pose.position.x = x;
        waypoint.pose.pose.position.y = y;
        waypoint.pose.pose.position.z = z;
        waypoint.pose.pose.orientation.x = qx;
        waypoint.pose.pose.orientation.y = qy;
        waypoint.pose.pose.orientation.z = qz;
        waypoint.pose.pose.orientation.w = qw;
        waypoint.velocity.x = vx;
        waypoint.velocity.y = vy;
        waypoint.velocity.z = vz;
        waypoints.push_back(waypoint);
    }
    file.close();
    return waypoints;
}

// std::vector<geometry_msgs::PoseStamped> loadWaypointsFromFile(const std::string& filename) {
//     std::vector<geometry_msgs::PoseStamped> waypoints;
//     std::ifstream file(filename);
//     if (!file.is_open()) {
//         ROS_ERROR("Failed to open file: %s", filename.c_str());
//         return waypoints;
//     }

//     std::string line;
//     while (std::getline(file, line)) {
//         std::istringstream ss(line);
//         double x, y, z, qx, qy, qz, qw;
//         if (!(ss >> x >> y >> z >> qx >> qy >> qz >> qw)) {
//             ROS_WARN("Invalid line format: %s", line.c_str());
//             continue;
//         }
//         geometry_msgs::PoseStamped waypoint;
//         waypoint.pose.position.x = x;
//         waypoint.pose.position.y = y;
//         waypoint.pose.position.z = z;
//         waypoint.pose.orientation.x = qx;
//         waypoint.pose.orientation.y = qy;
//         waypoint.pose.orientation.z = qz;
//         waypoint.pose.orientation.w = qw;
//         waypoints.push_back(waypoint);
//     }
//     file.close();
//     return waypoints;
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh("~");

    std::string filename;
    nh.param<std::string>("waypoints_file", filename, std::string("default"));

    // std::vector<geometry_msgs::PoseStamped> waypoints = loadWaypointsFromFile(filename);
    std::vector<WaypointWithVel> waypoints = loadWaypointsFromFile(filename);

    // ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);

    // 发布两种主题：兼容原有系统的 /goal 和新增带速度信息的 /waypoint_with_velocity
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);
    ros::Publisher waypoint_vel_pub = nh.advertise<waypoint_msgs::WaypointWithVelocity>("/waypoint_with_velocity", 10);


    ros::Rate rate(0.4); // 发布频率为 1 Hz
    int index = 0;
    while (ros::ok()) {
        if (index < waypoints.size()) {
            const auto& waypoint = waypoints[index];

            // 准备传统 PoseStamped 消息
            geometry_msgs::PoseStamped pose_msg = waypoint.pose;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = "world";
            
            // 准备带速度的自定义消息
            waypoint_msgs::WaypointWithVelocity vel_msg;
            vel_msg.header = pose_msg.header;
            vel_msg.pose = waypoint.pose.pose;
            vel_msg.velocity = waypoint.velocity;
            
            double roll, pitch, yaw;
            tf::Quaternion q(
                waypoint.pose.pose.orientation.x,
                waypoint.pose.pose.orientation.y,
                waypoint.pose.pose.orientation.z,
                waypoint.pose.pose.orientation.w
            );
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            ROS_INFO("Setting goal: Position(%.3f, %.3f, %.3f), Yaw: %.3f, Velocity(%.3f, %.3f, %.3f)",
                     waypoint.pose.pose.position.x, 
                     waypoint.pose.pose.position.y, 
                     waypoint.pose.pose.position.z,
                     yaw,
                     waypoint.velocity.x,
                     waypoint.velocity.y,
                     waypoint.velocity.z);

            // 发布两种消息
            goal_pub.publish(pose_msg);
            waypoint_vel_pub.publish(vel_msg);
            index++;
        } else {
            index = 0; // 重新开始发布
            ROS_INFO("Waypoint cycle completed, restarting from beginning");
        }
        rate.sleep();
    }

    return 0;
}
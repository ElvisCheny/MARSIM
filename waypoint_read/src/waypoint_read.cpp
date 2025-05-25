// ChenYang
// 2025/02/27
// Read waypoints from a file and publish them as goals
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <waypoint_msgs/WaypointWithVelocity.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstddef>
#include <tf/tf.h>
#include <cmath>

// 定义带速度信息的航点结构体
struct WaypointWithVel {
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Vector3 velocity;
};

// 无人机当前位置
geometry_msgs::Point current_position;
bool position_received = false;

// 航点到达阈值（米）
double waypoint_reached_threshold = 0.5;

// 超时时间（秒）
double waypoint_timeout = 10.0;

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

// 里程计回调函数，用于获取无人机当前位置
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position = msg->pose.pose.position;
    if (!position_received) {
        ROS_INFO("First position received: (%.2f, %.2f, %.2f)", 
                current_position.x, current_position.y, current_position.z);
    }
    position_received = true;
}

// 检查是否到达航点
bool hasReachedWaypoint(const geometry_msgs::Point& current, const geometry_msgs::Point& target) {
    double dx = current.x - target.x;
    double dy = current.y - target.y;
    // 不再计算 dz，忽略 Z 坐标
    double distance_2d = std::sqrt(dx*dx + dy*dy);
    return distance_2d < waypoint_reached_threshold;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh("~");

    std::string filename;
    nh.param<std::string>("waypoints_file", filename, std::string("default"));
    nh.param<double>("waypoint_reached_threshold", waypoint_reached_threshold, 0.5);
    nh.param<double>("waypoint_timeout", waypoint_timeout, 10.0);
    
    // 获取里程计话题名称
    std::string odom_topic;
    nh.param<std::string>("odom_topic", odom_topic, "/quad_0/lidar_slam/odom");

    // 加载航点
    std::vector<WaypointWithVel> waypoints = loadWaypointsFromFile(filename);
    if (waypoints.empty()) {
        ROS_ERROR("No valid waypoints loaded. Exiting.");
        return 1;
    }

    // 定义发布器
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);
    ros::Publisher waypoint_vel_pub = nh.advertise<waypoint_msgs::WaypointWithVelocity>("/waypoint_with_velocity", 10);
    
    // 定义里程计订阅器
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, odomCallback);

    // 设置循环频率（检查是否到达航点的频率）
    ros::Rate rate(10.0); // 10Hz，每0.1秒检查一次
    
    int current_waypoint_index = 0;
    bool waypoint_published = false;
    ros::Time waypoint_publish_time; // 记录航点发布的时间
    
    // 等待接收到位置信息
    ROS_INFO("Waiting for position information...");
    ros::Time wait_start_time = ros::Time::now();
    while (ros::ok() && !position_received) {
        // 如果等待超过10秒，跳出循环
        if ((ros::Time::now() - wait_start_time).toSec() > 10.0) {
            ROS_WARN("No position information received after 10 seconds. Continuing anyway.");
            // 设置默认位置以避免崩溃
            current_position.x = 0.0;
            current_position.y = 0.0;
            current_position.z = 0.0;
            position_received = true;
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Position received or timeout. Starting waypoint navigation.");
    ROS_INFO("Waypoint timeout set to %.1f seconds.", waypoint_timeout);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        if (current_waypoint_index >= waypoints.size()) {
            ROS_INFO("All waypoints completed.");
            break;  // 所有航点完成，退出循环
        }
        
        const auto& waypoint = waypoints[current_waypoint_index];
        
        // 判断是否需要发布航点（第一个航点或已到达前一个航点）
        if (!waypoint_published) {
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

            ROS_INFO("Setting goal %d: Position(%.3f, %.3f, %.3f), Yaw: %.3f, Velocity(%.3f, %.3f, %.3f)",
                     current_waypoint_index + 1,
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
            waypoint_published = true;
            waypoint_publish_time = ros::Time::now(); // 记录发布时间
            
            // 计算与当前位置的 XY 平面距离
            double dx = current_position.x - waypoint.pose.pose.position.x;
            double dy = current_position.y - waypoint.pose.pose.position.y;
            double distance_2d = std::sqrt(dx*dx + dy*dy);
            
            ROS_INFO("Distance to waypoint %d: %.2f meters (XY plane only)", current_waypoint_index + 1, distance_2d);
        }
        
        // 检查是否到达当前航点
        bool reached = hasReachedWaypoint(current_position, waypoint.pose.pose.position);
        
        // 检查是否超时
        double elapsed_time = (ros::Time::now() - waypoint_publish_time).toSec();
        bool timeout = elapsed_time > waypoint_timeout;
        
        // 打印当前状态信息（每2秒一次）
        if (waypoint_published) {
            ROS_INFO_THROTTLE(2.0, "Waypoint %d - Time: %.1f/%.1f s, XY Distance: %.2f/%.2f m",
                current_waypoint_index + 1,
                elapsed_time, waypoint_timeout,
                sqrt(pow(current_position.x - waypoint.pose.pose.position.x, 2) + 
                        pow(current_position.y - waypoint.pose.pose.position.y, 2)),
                waypoint_reached_threshold);
        }
        
        // 如果到达或超时，移动到下一个航点
        if (reached || timeout) {
            if (reached) {
                ROS_INFO("Reached waypoint %d. Moving to next waypoint.", current_waypoint_index + 1);
            } else {
                ROS_WARN("Timeout for waypoint %d (%.1f seconds). Skipping to next waypoint.", 
                        current_waypoint_index + 1, elapsed_time);
            }
            
            current_waypoint_index++;
            waypoint_published = false; // 准备发布下一个航点
            
            // 可选：到达航点后等待一段时间
            // ros::Duration(0.5).sleep();
        }
        
        rate.sleep();
    }

    ROS_INFO("Waypoint navigation completed.");
    return 0;
}
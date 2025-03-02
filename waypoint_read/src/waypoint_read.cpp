// ChenYang
// 2025/02/27
// Read waypoints from a file and publish them as goals
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstddef>
#include <tf/tf.h>

std::vector<geometry_msgs::PoseStamped> loadWaypointsFromFile(const std::string& filename) {
    std::vector<geometry_msgs::PoseStamped> waypoints;
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        double x, y, z, qx, qy, qz, qw;
        if (!(ss >> x >> y >> z >> qx >> qy >> qz >> qw)) {
            ROS_WARN("Invalid line format: %s", line.c_str());
            continue;
        }
        geometry_msgs::PoseStamped waypoint;
        waypoint.pose.position.x = x;
        waypoint.pose.position.y = y;
        waypoint.pose.position.z = z;
        waypoint.pose.orientation.x = qx;
        waypoint.pose.orientation.y = qy;
        waypoint.pose.orientation.z = qz;
        waypoint.pose.orientation.w = qw;
        waypoints.push_back(waypoint);
    }
    file.close();
    return waypoints;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh("~");

    std::string filename;
    nh.param<std::string>("waypoints_file", filename, std::string("default"));

    std::vector<geometry_msgs::PoseStamped> waypoints = loadWaypointsFromFile(filename);

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);

    ros::Rate rate(0.5); // 发布频率为 1 Hz
    size_t index = 0;
    while (ros::ok()) {
        if (index < waypoints.size()) {
            const auto& waypoint = waypoints[index];
            double roll, pitch, yaw;
            tf::Quaternion q(
                waypoint.pose.orientation.x,
                waypoint.pose.orientation.y,
                waypoint.pose.orientation.z,
                waypoint.pose.orientation.w
            );
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f",
                     waypoint.header.frame_id.c_str(),
                     waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z,
                     waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w,
                     yaw);

            goal_pub.publish(waypoints[index]);
            index++;
        } else {
            index = 0; // 重新开始发布
        }
        rate.sleep();
    }

    return 0;
}
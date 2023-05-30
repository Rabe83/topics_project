#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Read the range values representing the right side (90 degrees) and left side (-90 degrees)
    float range_right = msg->ranges[0];
    float range_left = msg->ranges[719];

    // Read the range value representing the forward direction (0 degrees)
    float range_forward = msg->ranges[360];

    ROS_INFO("Right Range: %f", range_right);
    ROS_INFO("Forward Range: %f", range_forward);
    ROS_INFO("Left Range: %f", range_left);

    geometry_msgs::Twist cmd;

    if (range_forward < 0.5)
    {
        // Turn fast to the left
        cmd.linear.x = 0.1;
        cmd.angular.z = 1.0;
    }
    else if (range_right > 0.3)
    {
        // Approach the wall
        cmd.linear.x = 0.1;
        cmd.angular.z = -0.1;
    }
    else if (range_right < 0.2)
    {
        // Move away from the wall
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.1;
    }
    else
    {
        // Keep moving forward
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.0;
    }

    pub.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topics_project_node");
    ros::NodeHandle nh;

    ROS_INFO("The node is subscribing to the LaserScan of the robot and publishing the cmd_vel to the robot in a control loop.....");

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("/scan", 10, laserscanCallback);

    ros::spin();

    return 0;
}

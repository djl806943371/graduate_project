#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello");
    ros::NodeHandle nh;

    ROS_INFO("Hello world 1!");

    ROS_INFO("Hello world 2!");

    ROS_INFO("Hello world 3!");

    ROS_INFO("Hello world 4!");

    ros::spin();
}

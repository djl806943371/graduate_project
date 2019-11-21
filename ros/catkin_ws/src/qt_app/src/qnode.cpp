#include <ros/network.h>
#include <sstream>
#include "qnode.h"
#include <QDebug>
#include <signal.h>

QNode::QNode(int argc, char** argv )
    : init_argc(argc), init_argv(argv)
{

}

QNode::~QNode()
{
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    ros::init(init_argc,init_argv,"listen_order_node");
    if (!ros::master::check())
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    ros::NodeHandle nSub;
    // Add your ros communications here.
    chatter_subscriber = nSub.subscribe("navigation/cmd_vel", 100, &QNode::RecvTopicCallback, this);
    start();
    return true;
}

void QNode::RecvTopicCallback(const geometry_msgs::Twist &msg)
{
    double degree = msg.linear.x;
    double angle = msg.angular.z;
//    qDebug() << degree << "||" << angle;
    emit cmd_vel(degree, angle);
}

void QNode::run()
{
    //ros::Rate loop_rate(1);
    m_isCanRun = true;              //标记可以运行
    ros::Duration initDur(0.1);     //循环频率10Hz
    while (ros::ok())
    {
        {
            QMutexLocker locker(&m_lock);
            if(!m_isCanRun)         //在每次循环判断是否可以运行，如果不行就退出循环
            {
                ros::shutdown(); // explicitly needed since we use ros::start();
                ros::waitForShutdown();
                qDebug() << "Ros shutdown, proceeding to close the gui.";
                emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
                return;
            }
        }
        ros::spinOnce();
        //loop_rate.sleep();
        initDur.sleep();
    }
    qDebug() << "Ros shutdown, proceeding to close the gui.";
    emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log(const LogLevel &level, const std::string &msg)
{
    std::stringstream logging_model_msg;
    switch (level)
    {
        case(Debug):
        {
            ROS_DEBUG_STREAM(msg);
            break;
        }
        case(Info):
        {
            ROS_INFO_STREAM(msg);
            break;
        }
        case(Warn):
        {
            ROS_WARN_STREAM(msg);
            break;
        }
        case(Error):
        {
            ROS_ERROR_STREAM(msg);
            break;
        }
        case(Fatal):
        {
            ROS_FATAL_STREAM(msg);
            break;
        }
    }
}

void QNode::stopImmediately()
{
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
}

#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <QThread>
#include <QStringListModel>
#include<geometry_msgs/Twist.h>
#include <QMutex>

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();
    void sig_cancel(int sig);

    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    void log( const LogLevel &level, const std::string &msg);
    void RecvTopicCallback(const geometry_msgs::Twist &msg);

signals:
    void rosShutdown();
    void cmd_vel(double degree, double angle);

public slots:
    void stopImmediately();

private:
    int init_argc;
    char** init_argv;
    ros::Subscriber chatter_subscriber;
    QMutex m_lock;
    bool m_isCanRun;
};

#endif // QNODE_H

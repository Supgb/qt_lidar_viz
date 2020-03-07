#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H

#include <QThread>

#include <ros/ros.h>

#include <lslidar_n301_driver/lslidar_n301_driver.h>
#include "../include/logsys.h"

namespace lidar_driver
{

class LidarDriver final: public QThread, public lidar_base::LogSys
{
    Q_OBJECT
public:
    explicit LidarDriver(ros::NodeHandle& nh, ros::NodeHandle& ph,
                         QStringListModel* lm);
    ~LidarDriver();
    LidarDriver(const LidarDriver&) = delete;
    LidarDriver& operator=(const LidarDriver&) = delete;
    void run(); // Thread entry.

    bool _w_initialize();

    // Log.
    virtual void log_pipe(const LogLevel& level, const QString& msg)
    { log(level, msg); Q_EMIT UPDATE_LOG(); }

Q_SIGNALS:
    void UPDATE_LOG();
    void ROS_EXIT();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;
    lslidar_n301_driver::LslidarN301DriverPtr driver_ptr;
};

}   // namespace lidar_driver

#endif // LIDARDRIVER_H

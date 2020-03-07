#ifndef LIDARDECODER_H
#define LIDARDECODER_H

#include <ros/ros.h>

#include <QThread>

#include "../include/logsys.h"
#include <lslidar_n301_decoder/lslidar_n301_decoder.h>

namespace lidar_driver
{

class LidarDecoder final: public QThread, public lidar_base::LogSys
{
    Q_OBJECT
public:
    explicit LidarDecoder(ros::NodeHandle& nh, ros::NodeHandle& ph,
                          QStringListModel* lm);
    LidarDecoder(const LidarDecoder&) = delete;
    LidarDecoder& operator =(const LidarDecoder&) = delete;
    ~LidarDecoder();
    void run(); // Thread entry.

    bool _w_initialize();

    // Decoder parameters modify.
    void setMaxAngle(double angle_max);
    void setMinAngle(double angle_min);

    // Log.
    virtual void log_pipe(const LogLevel& level, const QString& msg)
    { log(level, msg); Q_EMIT UPDATE_LOG(); }

Q_SIGNALS:
    void UPDATE_LOG();
    void ROS_EXIT();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;
    lslidar_n301_decoder::LslidarN301DecoderPtr decoder_ptr;
};

}   // namespace lidar_driver

#endif // LIDARDECODER_H

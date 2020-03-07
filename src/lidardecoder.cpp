#include "../include/lidardecoder.h"

namespace lidar_driver
{

LidarDecoder::LidarDecoder(ros::NodeHandle& nh, ros::NodeHandle& ph,
                           QStringListModel* lm):
    LogSys(lm),
    m_nh(nh), m_ph(ph),
    decoder_ptr(new lslidar_n301_decoder::LslidarN301Decoder(m_nh, m_ph))
{}

LidarDecoder::~LidarDecoder()
{
    this->wait();
}

void LidarDecoder::run()
{
    log_pipe(Debug, "LidarDecoder thread started...");
    // ROS spin
    ros::waitForShutdown();
    //Q_EMIT ROS_EXIT();
}

bool LidarDecoder::_w_initialize()
{
    if (!decoder_ptr->initialize())
    {
        log_pipe(Debug, "Failed at LslidarN301Decoder::initialize()...");
        return false;
    }
    this->start();
    return true;
}

void LidarDecoder::setMaxAngle(double angle_max)
{
    if(decoder_ptr->setMaxAngle(angle_max))
        log_pipe(Debug, "Successfully update the maximum angle.");
}

void LidarDecoder::setMinAngle(double angle_min)
{
    if(decoder_ptr->setMinAngle(angle_min))
        log_pipe(Debug, "Successfully update the minimum angle");
}

}   // namespace lidar_base

#include "../include/lidardriver.h"

namespace lidar_driver
{

LidarDriver::LidarDriver(ros::NodeHandle& nh, ros::NodeHandle& ph,
                         QStringListModel* lm):
    LogSys(lm),
    m_nh(nh), m_ph(ph),
    driver_ptr(new lslidar_n301_driver::LslidarN301Driver(m_nh, m_ph))
{}


LidarDriver::~LidarDriver()
{
    if( !this->wait(2000))
    {
        this->terminate();
        this->wait();
    }
}

void LidarDriver::run()
{
    log_pipe(Debug, "LidarDriver thread started...");
    while(m_nh.ok() && driver_ptr->polling())
    {
        ;
    }
    //Q_EMIT ROS_EXIT();

}

bool LidarDriver::_w_initialize()
{
    if (!driver_ptr->initialize())
    {
        log_pipe(Error, "Failed at LslidarN301Driver::initialize()");
        return false;
    }
    this->start();
    return true;
}

}   // namespace lidar_driver

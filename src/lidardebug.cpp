#include "../include/lidardebug.h"
#include <boost/thread.hpp>
#include "../include/lidarmdl.h"

namespace lidar_base
{

template class LidarDebug<LidarMDL>;

template <typename T>
void LidarDebug<T>::debug()
{
    _dbg_obj->outputDebug();
}

template <typename T>
boost::thread* LidarDebug<T>::spawn()
{
    return (new boost::thread(&LidarDebug<T>::debug, this));
}

}   // namespace lidar_base

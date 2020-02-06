#include "../include/lidar_assembler.h"

namespace lidar_assembler {

const char* ACT_WS = "source /home/supgb/Documents/Dev/ros_test/devel/setup.sh";
const char* LSLIDAR_SETUP = "roslaunch qt_lidar_viz qt_lidar_viz.launch";

LidarAssembler::LidarAssembler(ros::NodeHandle n):
    _nh(n),
    _laser_sub(_nh, "scan_filtered", 10),
    _laser_notifier(_laser_sub, _listener, "laser_link", 10)
{
    _laser_notifier.registerCallback(
                boost::bind(&LidarAssembler::scanCallback, this, _1));

    _laser_notifier.setTolerance(ros::Duration(0.01));
    _cloud.header.frame_id = "laser_link";
    _cloud_pub = _nh.advertise<sensor_msgs::PointCloud>("lidar_cloud", 100);
}

bool LidarAssembler::init(int init_argc, char** init_argv)
{
    // Start lslidar_n301_decoder
    if ( ! ros::master::check() )
    {
        return false;
    }
    //system(ACT_WS);
    //system(LSLIDAR_SETUP);

    // System params adjust.
    resetIndex(this->_index);
    setDutyRange(this->max_duty_range);
    endUp();
    //setUp();
    ros::start();
    start();
    return true;
}

void LidarAssembler::run()
{
    ros::spin();
}

/*
Auto tune the value of max duty range.
*/
void LidarAssembler::setDutyRange(double &mdr)
{
    ROS_INFO("Max duty range has been updated...");
}

/**/
inline bool LidarAssembler::isEmptyScan()
{
    return cd_buff.points.size() == 0 ? true : false;
}


/**/
bool LidarAssembler::assemble()
{
    _index += 0.01;
    for (auto point : cd_buff.points)
    {
        point.z += _index;
        _cloud.points.push_back(point);
    }
    ROS_INFO("Finished assemble...");

    // Assemble end and publish the whole cloud to the topic.
    _cloud_pub.publish(_cloud);

    return true;
}


/*
Do not process assemble operation,
just store the new in-coming cloud.
*/
inline bool LidarAssembler::store_cd()
{
    _storage_cloud.push_back(cd_buff);
    return true;
}

void LidarAssembler::scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
    // Get point cloud.
    try
    {
        _projector.transformLaserScanToPointCloud(
                    "laser_link", *scan_msg, cd_buff, _listener);
    } catch(const tf::TransformException& e)
    {
        ROS_ERROR(e.what());
        return;
    }

#ifdef SCAN_DEBUG
    if(isEmptyScan() || _db_start_scan)
#else
    if(isEmptyScan())
#endif  // SCAN_DEBUG
    {
        ROS_INFO("No objects detected...");
        return;
    }

    // Mode detection.
    if(_mode == STORE_M)
    {
        // Store the cloud into stock.
        if(!store_cd())
        {
            ROS_WARN("Failed at store_cd()");
            return;
        }
    }
    else if(_mode == ASSEMBLE_M)
    {
        // Assemble begin.
        if(!assemble())
        {
            ROS_WARN("Failed at assemble()");
            return;
        }
    }

}   // end of scanCallback()

}   // namespace lidar_assembler

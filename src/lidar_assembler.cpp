#include "../include/lidar_assembler.h"

#include <QDateTime>
#include <QTextStream>

namespace lidar_base {

const char* ACT_WS = "source /home/supgb/Documents/Dev/ros_test/devel/setup.sh";
const char* LSLIDAR_SETUP = "roslaunch qt_lidar_viz qt_lidar_viz.launch";

LidarAssembler::LidarAssembler(const std::string &sub_topic_name,
                               const std::string& pub_topic_name,
                               const std::string &frame_id,
                               ros::NodeHandle& nh, ros::NodeHandle& ph,
                               QStringListModel* lm):
    LogSys(lm),
    sub_str(sub_topic_name), pub_str(pub_topic_name), _frame_id(frame_id),
    cd_buff(),
    _cloud_shared(new pcl::PointCloud<pcl::PointXYZ>),
    _storage_cloud(),
    _scan(),
    _nh(nh), _ph(ph),
    _laser_sub(_nh, sub_str, 10), _cloud_pub(),
    _projector(), _listener(),
    _laser_notifier(_laser_sub, _listener, _frame_id, 10),
    _mode(LASERSCAN_M), max_duty_range(1.00), _index(0.0), _db_start_scan(0)
{}

bool LidarAssembler::init()
{
    // Start lslidar_n301_decoder
    if ( ! ros::master::check() )
    {
        return false;
    }

    _laser_notifier.registerCallback(
                boost::bind(&LidarAssembler::scanCallback, this, _1));
    _laser_notifier.setTolerance(ros::Duration(0.01));
    _cloud_shared->header.frame_id = _frame_id;
    _cloud_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(pub_str, 100);

    // System params adjust.
    endUp();
    //setUp();
    this->start();

    return true;
}

void LidarAssembler::run()
{
    ros::waitForShutdown();
    //Q_EMIT EXIT_ROS();
}


/*
*** Modify the execution mode.
*/
bool LidarAssembler::setExecMode(const QString& str)
{
    QString str2log;
    if (str == "Storage")
        _mode = STORE_M;
    else if (str == "Assemble")
        _mode = ASSEMBLE_M;
    else if (str == "LaserScan")
        _mode = LASERSCAN_M;
    str2log = "The execute mode is changed to " + str;
    log_pipe(Info, str2log);
    return true;
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
    //return cd_buff.points.size() == 0 ? true : false;
    return cd_buff.points.size() == 0 ? true : false;
}


/**/
bool LidarAssembler::assemble()
{
    _index += 0.01;
    //pcl::fromROSMsg(ros_pcd_buff, cd_buff);
    for (auto point : cd_buff.points)
    {
        point.z += _index;
        _cloud_shared->points.push_back(point);
    }
    ROS_INFO("Finished assemble...");

    // Assemble end and publish the whole cloud to the topic.
    // intraprocess publish.
    _cloud_pub.publish(_cloud_shared);

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
                    _frame_id, *scan_msg, ros_pcd_buff, _listener);
    } catch(const tf::TransformException& e)
    {
        ROS_ERROR(e.what());
        return;
    }
    pcl::fromROSMsg(ros_pcd_buff, cd_buff);
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
    else if(_mode == LASERSCAN_M)
    {
        //pcl::fromROSMsg(ros_pcd_buff, cd_buff);
        //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pd_ptr(&cd_buff);
        resetIndex(0);
        _cloud_shared->clear();
        _cloud_pub.publish(cd_buff);
    }

}   // end of scanCallback()

}   // namespace lidar_assembler

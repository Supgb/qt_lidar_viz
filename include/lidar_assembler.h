#ifndef LIDAR_ASSEMBLER_H
#define LIDAR_ASSEMBLER_H

#include <QThread>
#include <QStringListModel>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <pcl_ros/point_cloud.h>

#include <boost/shared_ptr.hpp>
#include <vector>

#include "../include/logsys.h"

#ifndef SCAN_DEBUG
#define SCAN_DEBUG
#endif // SCAN_DEBUG

namespace lidar_base {

extern const char* ACT_WS;
extern const char* LSLIDAR_SETUP;

class LidarAssembler final: public QThread, public LogSys
{
    Q_OBJECT
public:

    enum ExecMode
    {
        STORE_M,    // Store mode, only accumulate the cloud.
        ASSEMBLE_M,  // Assemble mode, collect the point from the cloud,
                    //  and assemble into _cloud to be published.
        LASERSCAN_M
    };

    typedef boost::shared_ptr<LidarAssembler> LidarAssemblerPtr;
    typedef boost::shared_ptr<const LidarAssembler> LidarAssemblerConstPtr;

    explicit LidarAssembler(const std::string& sub_topic_name,
                            const std::string& pub_topic_name,
                            const std::string& frame_id,
                            ros::NodeHandle& nh, ros::NodeHandle& ph,
                            QStringListModel* lm);
    LidarAssembler(const LidarAssembler&) = delete;
    LidarAssembler& operator=(const LidarAssembler&) = delete;
    ~LidarAssembler()
    {
        this->wait();
    }

    // Qt Routine.
    void run();

    // ROS Routine.
    bool init();

#ifdef SCAN_DEBUG
    inline bool setUp()
        { _db_start_scan = 1; }
    inline bool endUp()
        { _db_start_scan = 0; }
#endif // SCAN_DEBUG

    // Get system details.
    inline const double& getMaxDutyRange() const
        { return max_duty_range; }
    inline void resetIndex(float idx)
        { _index = 0.0; }
    inline int getCloudStorage() const
        { return _storage_cloud.size(); }

    // Log.
    virtual void log_pipe(const LogLevel& level, const QString& msg)
    { log(level, msg); Q_EMIT UPDATE_LOG(); }

    // members access.
    const std::string& getSubTopic() const
    { return sub_str; }
    const std::string& getPubTopic() const
    { return pub_str; }
    const std::string& getFrameId() const
    { return _frame_id; }

    // ros vars access.
    void _unsubscribe()
    { _laser_sub.unsubscribe(); }
    void _resubscribe()
    {
        _laser_sub.subscribe(_nh, sub_str, 10);
    }

Q_SIGNALS:
    void UPDATE_LOG();
    void EXIT_ROS();

public Q_SLOTS:
    bool setExecMode(const QString&);


private:
    // ROS vars.
    std::string sub_str, pub_str, _frame_id;
    sensor_msgs::PointCloud2 ros_pcd_buff;
    pcl::PointCloud<pcl::PointXYZ> cd_buff;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_shared;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> _storage_cloud;   // TODO:Modify the type.
    sensor_msgs::LaserScan _scan;
    ros::NodeHandle _nh;
    ros::NodeHandle _ph;
    message_filters::Subscriber<sensor_msgs::LaserScan> _laser_sub;
    ros::Publisher _cloud_pub;
    laser_geometry::LaserProjection _projector;
    tf::TransformListener _listener;
    tf::MessageFilter<sensor_msgs::LaserScan> _laser_notifier;
    // System vars.
    ExecMode _mode;
    double max_duty_range;
    float _index;

#ifdef SCAN_DEBUG
    bool _db_start_scan;
#endif // SCAN_DEBUG


    // Callback.
    void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);

    // System functions.
    void setDutyRange(double& mdr);
    bool isEmptyScan();


    // Assembler system functions.
    bool assemble();
    bool store_cd();

};

}   // namespace lidar_base
#endif // LIDAR_ASSEMBLER_H

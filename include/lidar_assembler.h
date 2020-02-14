#ifndef LIDAR_ASSEMBLER_H
#define LIDAR_ASSEMBLER_H

#include <QThread>
#include <QStringListModel>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <boost/shared_ptr.hpp>
#include <vector>

#include "../include/logsys.h"

#ifndef SCAN_DEBUG
#define SCAN_DEBUG
#endif // SCAN_DEBUG

namespace lidar_assembler {

extern const char* ACT_WS;
extern const char* LSLIDAR_SETUP;

class LidarAssembler : public QThread, public lidar_log::LogSys
{
    Q_OBJECT
public:

    enum ExecMode
    {
        STORE_M,    // Store mode, only accumulate the cloud.
        ASSEMBLE_M  // Assemble mode, collect the point from the cloud,
                    //  and assemble into _cloud to be published.
    };

//    enum LogLevel
//    {
//        Debug,
//        Info,
//        Warn,
//        Error,
//        Fatal
//    };

    typedef boost::shared_ptr<LidarAssembler> LidarAssemblerPtr;
    typedef boost::shared_ptr<const LidarAssembler> LidarAssemblerConstPtr;

    explicit LidarAssembler(ros::NodeHandle nh);
    LidarAssembler(const LidarAssembler&) = delete;
    LidarAssembler& operator=(const LidarAssembler&) = delete;
    virtual ~LidarAssembler()
    {
        if(ros::isStarted())
        {
            ros::shutdown();
            ros::waitForShutdown();
        }
        wait();
    }

    // Qt Routine.
    void run();

    // ROS Routine.
    bool init(int argc, char** argv);

#ifdef SCAN_DEBUG
    inline bool setUp()
        { _db_start_scan = 1; }
    inline bool endUp()
        { _db_start_scan = 0; }
#endif // SCAN_DEBUG

    // Get system details.
    inline const double& getMaxDutyRange() const
        { return max_duty_range; }
    inline void resetIndex(geometry_msgs::Point32::_z_type& idx)
        { idx = 0.0; }
    inline int getCloudStorage() const
        { return _storage_cloud.size(); }

    // Log.
    virtual void log_pipe(const LogLevel& level, const QString& msg)
    { log(level, msg); Q_EMIT UPDATE_LOG(); }

Q_SIGNALS:
    void UPDATE_LOG();
    void EXIT_ROS();

public Q_SLOTS:
    bool setExecMode(const QString&);



private:
    // ROS vars.
    sensor_msgs::PointCloud _cloud, cd_buff;
    std::vector<sensor_msgs::PointCloud> _storage_cloud;
    sensor_msgs::LaserScan _scan;
    ros::NodeHandle _nh;
    message_filters::Subscriber<sensor_msgs::LaserScan> _laser_sub;
    ros::Publisher _cloud_pub;
    laser_geometry::LaserProjection _projector;
    tf::TransformListener _listener;
    tf::MessageFilter<sensor_msgs::LaserScan> _laser_notifier;
    // Log vars.
//    QStringListModel logging_model;
    // System vars.
    ExecMode _mode;
    double max_duty_range;
    geometry_msgs::Point32::_z_type _index;

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

}   // namespace lidar_assembler
#endif // LIDAR_ASSEMBLER_H

#ifndef LIDARMDL_H
#define LIDARMDL_H

#include <QThread>
#include <QStringListModel>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "../include/logsys.h"

namespace lidar_mdl
{

class LidarMDL : public QThread, public lidar_log::LogSys
{
    Q_OBJECT
public:
    typedef boost::shared_ptr<LidarMDL> LidarMDLPtr;
    typedef boost::shared_ptr<const LidarMDL> LidarMDLConstPtr;

    explicit LidarMDL(ros::NodeHandle& nh, const QString& filename);
    LidarMDL(const LidarMDL&) = delete;
    LidarMDL& operator =(const LidarMDL&) = delete;
    virtual ~LidarMDL()
    {
        wait();
    }

    // Qt Routine.
    void run();

    // Log Routine.
    virtual void log_pipe(const LogLevel& level, const QString& msg)
    { log(level, msg); Q_EMIT UPDATE_LOG(); }

    // ROS Routine.
    bool init(int argc, char** argv);

Q_SIGNALS:
    void UPDATE_LOG();

private:
    //
    const QString m_f_name;

    // PCL related.
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_cloud_with_normals;
    pcl::PointCloud<pcl::Normal>::Ptr m_normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr m_tree;
    pcl::search::KdTree<pcl::PointNormal>::Ptr m_tree_normal;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal>::Ptr m_gp3;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr m_ne;
    pcl::PolygonMesh m_triangles;

    // ROS related.
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;

private:
    void mCloudCallback(const sensor_msgs::PointCloudConstPtr& _cloud_msg);
    void __reconstruct();
    void __outputSTL();

};

} // namespace lidar_mdl
#endif // LIDARMDL_H





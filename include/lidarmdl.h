#ifndef LIDARMDL_H
#define LIDARMDL_H

#include <QThread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/shared_ptr.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <visualization_msgs/Marker.h>

#include "../include/logsys.h"

namespace lidar_base
{
class LidarAssembler;

class LidarMDL final: public QThread, public LogSys
{
    Q_OBJECT
public:
    typedef boost::shared_ptr<LidarMDL> LidarMDLPtr;
    typedef boost::shared_ptr<const LidarMDL> LidarMDLConstPtr;

    explicit LidarMDL(const LidarAssembler& assem,
                      const std::string& _mesh_out_str,
                      const std::string& filename,
                      QStringListModel* lm);
    LidarMDL(const LidarMDL&) = delete;
    LidarMDL& operator =(const LidarMDL&) = delete;
    ~LidarMDL()
    {
        this->wait();
    }

    void __reconstruct();

    visualization_msgs::Marker& getMarker()
    { return marker; }

    // Qt Routine.
    void run();

    // Log Routine.
    virtual void log_pipe(const LogLevel& level, const QString& msg)
    { log(level, msg); Q_EMIT UPDATE_LOG(); }

    // ROS Routine.
    bool init();

    // ROS vars access.
    void _unsubscribe()
    { m_sub.shutdown(); }
    void _resubscribe()
    {
        m_sub = m_nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(sub_cloud_str,
                                                               100, &LidarMDL::mCloudCallback,
                                                               this);
    }

    void outputDebug();

Q_SIGNALS:
    void UPDATE_LOG();
    void EXIT_ROS();

public Q_SLOTS:
    void setValid();

private:
    // Internal access functions.
    void setInvalid() { really_to_go = 0; }
    bool isValid() { return really_to_go; }

    // Control flag.
    bool really_to_go;

    // String
    const std::string m_f_name;
    std::string sub_cloud_str;
    std::string mesh_out_str;
    std::string frame_id_str;
    const std::string path_to_load_mesh_1;

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
    ros::NodeHandle m_ph;
    ros::Subscriber m_sub;
    ros::Publisher m_pub;

    visualization_msgs::Marker marker;

private:
    void mCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& _cloud_msg);
    void __outputSTL();

};

} // namespace lidar_base
#endif // LIDARMDL_H





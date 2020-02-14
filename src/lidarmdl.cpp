#include "../include/lidarmdl.h"

#include <pcl/io/vtk_lib_io.h>

#include <QDateTime>
#include <QTextStream>

namespace lidar_mdl {

LidarMDL::LidarMDL(ros::NodeHandle& nh, const QString& filename):
    m_nh(nh),
    m_sub(),
    m_pub(),
    m_triangles(),
    m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    m_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>),
    m_normals(new pcl::PointCloud<pcl::Normal>),
    m_tree(new pcl::search::KdTree<pcl::PointXYZ>),
    m_tree_normal(new pcl::search::KdTree<pcl::PointNormal>),
    m_gp3(new pcl::GreedyProjectionTriangulation<pcl::PointNormal>),
    m_ne(new pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>),
    m_f_name(filename)
{

}


/**/
bool LidarMDL::init(int argc, char **argv)
{
    // Kd-Tree init.
    m_tree->setInputCloud(m_cloud);

    // Kd-Tree with normal init.
    m_tree_normal->setInputCloud(m_cloud_with_normals);

    // Normal estimation obj init.
    m_ne->setInputCloud(m_cloud);
    m_ne->setSearchMethod(m_tree);
    m_ne->setKSearch(20);

    // Greedy projection obj init.
    m_gp3->setSearchRadius(0.025);  // Set the maximum distance between
                                    // connected points (maximum edge length)
    m_gp3->setMu(2.5);
    m_gp3->setMaximumNearestNeighbors(100);
    m_gp3->setMaximumSurfaceAngle(M_PI/4);  // 45 degrees
    m_gp3->setMinimumAngle(M_PI/18);    // 10 degrees
    m_gp3->setMaximumAngle(2*M_PI/3);    // 120 degrees
    m_gp3->setNormalConsistency(false);
    m_gp3->setInputCloud(m_cloud_with_normals);
    m_gp3->setSearchMethod(m_tree_normal);

    return true;
}

/*
*** Thread entry.
***
*/
void LidarMDL::run()
{

}


/**/
void LidarMDL::mCloudCallback(const sensor_msgs::PointCloudConstPtr &_cloud_msg)
{

}


/**/
void LidarMDL::__reconstruct()
{
    // m_cloud loading.

    // m_ne process.
    m_ne->compute(*m_normals);

    // Concatenate the XYZ and normal fields.
    pcl::concatenateFields(*m_cloud, *m_normals, *m_cloud_with_normals);

    // Reconstruct with greedy projection.
    m_gp3->reconstruct(m_triangles);
}


/**/
inline void LidarMDL::__outputSTL()
{
    pcl::io::savePolygonFileSTL(m_f_name.toStdString(), m_triangles);
}

}   // namespace lidar_mdl

#include "../include/lidarmdl.h"
#include "../include/lidar_assembler.h"
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <iostream>


namespace lidar_base {

LidarMDL::LidarMDL(const LidarAssembler &assem,
                   const std::string& _mesh_out_str,
                   const std::string &filename,
                   QStringListModel* lm):
    LogSys(lm),
    really_to_go(0),
    m_f_name(filename), sub_cloud_str(assem.getPubTopic()), mesh_out_str(_mesh_out_str),
    frame_id_str(assem.getFrameId()),
    path_to_load_mesh_1("/home/supgb/Documents/Dev/ros_test/devel/lib/qt_lidar_viz/mesh.stl"),
    m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    m_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>),
    m_normals(new pcl::PointCloud<pcl::Normal>),
    m_tree(new pcl::search::KdTree<pcl::PointXYZ>),
    m_tree_normal(new pcl::search::KdTree<pcl::PointNormal>),
    m_gp3(new pcl::GreedyProjectionTriangulation<pcl::PointNormal>),
    m_ne(new pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>),
    m_triangles(),
    m_nh(), m_ph("~"),
    m_sub(),
    m_pub(),
    marker()
{}


/**/
bool LidarMDL::init()
{
    if ( !ros::master::check())
    {
        return false;
    }
    // Ros insfrausturction init.
    m_sub = m_nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(sub_cloud_str,
                           100, &LidarMDL::mCloudCallback, this);
    m_pub = m_nh.advertise<visualization_msgs::Marker>(mesh_out_str, 10);

    marker.header.frame_id = frame_id_str;
    marker.header.stamp = ros::Time();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "file://" + path_to_load_mesh_1;    

    this->start();

    return true;
}

/*
*** Thread entry.
***
*/
void LidarMDL::run()
{
    log_pipe(Debug, "LidarMDL thread started..");
    ros::Rate rate(5);
    while (ros::ok())
    {
        m_pub.publish( marker );
        rate.sleep();
    }
    //Q_EMIT EXIT_ROS();
}

/*
*** q_slots
*/
void LidarMDL::setValid()
{
    really_to_go = 1;
    log_pipe(Debug, "LidarMDL is really to reconstruct 3d mesh...");
}

/**/
void LidarMDL::mCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_cloud_msg)
{
    pcl::copyPointCloud(*_cloud_msg, *m_cloud);
    if (!isValid())return;

    setInvalid();

    m_ne->compute(*m_normals);

    // Concatenate the XYZ and normal fields.
    pcl::concatenateFields(*_cloud_msg, *m_normals, *m_cloud_with_normals);
    // Clear m_cloud.
    //m_cloud->clear();

    // Reconstruct with greedy projection.
    m_gp3->reconstruct(m_triangles);

    log_pipe(Debug, "Reconstruct finished...");
}


/**/
void LidarMDL::__reconstruct()
{
    // m_cloud loading.
    m_tree->setInputCloud(m_cloud);

    // m_ne process.
    m_ne->setInputCloud(m_cloud);
    m_ne->setSearchMethod(m_tree);
    m_ne->setKSearch(20);
    m_ne->compute(*m_normals);

    // Concatenate the XYZ and normal fields.
    pcl::concatenateFields(*m_cloud, *m_normals, *m_cloud_with_normals);
    // Clear m_cloud.

    m_tree_normal->setInputCloud(m_cloud_with_normals);

    // Reconstruct with greedy projection.
    m_gp3->setSearchRadius(0.075);  // Set the maximum distance between
                                    // connected points (maximum edge length)
    m_gp3->setMu(2.5);
    m_gp3->setMaximumNearestNeighbors(200);
    m_gp3->setMaximumSurfaceAngle(M_PI/4);  // 45 degrees
    m_gp3->setMinimumAngle(M_PI/18);    // 10 degrees
    m_gp3->setMaximumAngle(2*M_PI/3);    // 120 degrees
    m_gp3->setNormalConsistency(false);
    m_gp3->setInputCloud(m_cloud_with_normals);
    m_gp3->setSearchMethod(m_tree_normal);
    m_gp3->reconstruct(m_triangles);

    __outputSTL();


    //m_pub.publish( marker );    // Publish marker to mesh_out_str.
}


/**/
inline void LidarMDL::__outputSTL()
{
    pcl::io::savePolygonFileSTL(m_f_name, m_triangles);
    pcl::io::saveVTKFile("mesh_in_vtk.vtk", m_triangles);
    log_pipe(Debug, "Output mesh to STL file finished...");
}

void LidarMDL::outputDebug()
{
    fstream csv_handler;
    if ( m_cloud->points.size() == 0)
    {
        log_pipe(Error, "Cannot output the empty cloud!!!");
        return;
    }
    csv_handler.open("qt_lidar_viz_pd_data.csv", ios::in|ios::out|ios::trunc);
    if (!csv_handler.is_open())
    {
        log_pipe(Error, "Failed to open cloud data to disk...");
        return;
    }
    for (auto point : m_cloud->points)
    {
        csv_handler << point.x << "," << point.y << "," << point.z << "\n";
    }
    csv_handler.close();
}

}   // namespace lidar_base

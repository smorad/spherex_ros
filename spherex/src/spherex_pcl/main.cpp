#include <iostream>
#include <ros/ros.h>

//#include <pcl/point_types.h>
//#include <pcl/common/comon.h>
//#include <boost/foreach.hpp>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/poisson.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>

void process_fast_triangulate(const sensor_msgs::PointCloud2& input) {
    
    std::cout<<"Processor got msg"<<std::endl;

    // Convert from ROS to pcl cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
    //pcl_conversions::toPCL(input, cloud)
    pcl::fromROSMsg(input, *cloud);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    pcl::io::saveOBJFile("/home/smorad/mesh.obj", triangles);



    /*
     *     PointCloud<pcl::Normal>::Ptr cloud_normals(new PointCloud<pcl::Normal>());

   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
   ne.setInputCloud(msg->points);
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
   ne.setSearchMethod(tree);
   ne.setRadiusSearch (0.03);
   ne.compute(*cloud_normals);
    

   pcl::Poisson<pcl::PointNormal> poisson;
   poisson.setDepth(9);
   poisson.setInputCloud(cloud_normals);
   pcl::PolygonMesh mesh;
   poisson.reconstruct(mesh);
     */
    //  p->performReconstruction()
    //  BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points) {
    // compute normal
    // screened poisson
    // }
}

int main(int argc, char** argv) {
    std::cout << "Main starting..." << std::endl;
    ros::init(argc, argv, "pcl_pipeline");
    ros::NodeHandle nh;
    ros::Subscriber lidar_reader = nh.subscribe("lidar_stream", 1, process_fast_triangulate);
    ros::spin();
}

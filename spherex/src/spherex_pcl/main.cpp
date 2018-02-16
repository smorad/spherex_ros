#include <iostream>
#include <ros/ros.h>


//#include <pcl/point_types.h>
//#include <pcl/common/comon.h>
//#include <boost/foreach.hpp>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/poisson.h>

#include <sensor_msgs/PointCloud.h>
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
#include <pcl/filters/voxel_grid.h>

void process_fast_triangulate(const sensor_msgs::PointCloud& input) {

    std::cout << "Processor got msg" << std::endl;

    // Convert from ROS to pcl cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> cloud = *cloud_ptr;

    std::cout << "pt 1" << std::flush;
    //pcl::PointCloud<pcl::PointXYZ> cloud = *cloud_ptr;
    // no library fn to convert from PointCloud, only PointCloud2
    //cloud.points = input->points;
    cloud_ptr->points.resize(input.points.size());
    for (int i = 0; i < cloud_ptr->points.size(); ++i) {

        // Filter NaNs and infs
        if (std::isnan(input.points[i].x) || std::isinf(input.points[i].x)) {
            cloud_ptr->points.erase(cloud_ptr->begin() + i);
            continue;
        }
        //    continue;
        //}

        std::cout << "pts" << input.points[i] << std::endl;
        cloud_ptr->points[i].x = input.points[i].x;
        cloud_ptr->points[i].y = input.points[i].y;
        cloud_ptr->points[i].z = input.points[i].z;
        std::cout << "cloud" << cloud_ptr->points[i] << std::endl;

    }
    std::cout << "cloud size" << cloud_ptr->size() << std::endl;

    std::cout << "pt 2" << std::endl;


    //Normals

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_ptr);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*


    // Normal estimation*
    /*pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_ptr);
    n.setInputCloud(cloud_ptr);
    n.setSearchMethod(tree);
    n.setKSearch(1); // was 20
    n.compute(*normals);
    std::cout << "pt 3" << std::flush;*/


    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_ptr, *cloud_normals, *cloud_with_normals);
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
    //ros::Subscriber lidar_reader = nh.subscribe("lidar_stream", 1, process_fast_triangulate);
    ros::Subscriber cloud_reader = nh.subscribe("pointcloud_buffer", 1, process_fast_triangulate);
    ros::spin();
}

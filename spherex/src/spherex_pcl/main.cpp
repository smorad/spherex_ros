#include <iostream>
#include <ros/ros.h>


//#include <pcl/point_types.h>
//#include <pcl/common/comon.h>
//#include <boost/foreach.hpp>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/poisson.h>

#include <sensor_msgs/PointCloud.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/poisson.h>
#include <pcl-1.7/pcl/pcl_base.h>
#include <pcl-1.7/pcl/surface/reconstruction.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/registration/icp.h>
#include <pcl-1.7/pcl/registration/registration.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/src/Geometry/AngleAxis.h>
#include <eigen3/Eigen/src/Core/PlainObjectBase.h>
#include <eigen3/Eigen/src/Geometry/Translation.h>
#include <pcl-1.7/pcl/kdtree/kdtree.h>
//#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/sample_consensus/model_types.h>
#include <pcl-1.7/pcl/sample_consensus/method_types.h>
#include <pcl-1.7/pcl/PointIndices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl-1.7/pcl/features/feature.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <sensor_msgs/Image.h>


gazebo_msgs::ModelState state_buf;
gazebo_msgs::ModelState last_state_buf;
pcl::PolygonMesh partial_surface; // triangles(new pcl::PolygonMesh);

pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud(
        const sensor_msgs::PointCloud& input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr->points.resize(input.points.size());
    for (int i = 0; i < cloud_ptr->points.size(); ++i) {
        // Filter NaNs and infs
        if (std::isnan(input.points[i].x) || std::isinf(input.points[i].x)) {
            cloud_ptr->points.erase(cloud_ptr->begin() + i);
            continue;
        }
        cloud_ptr->points[i].x = input.points[i].x;
        cloud_ptr->points[i].y = input.points[i].y;
        cloud_ptr->points[i].z = input.points[i].z;
    }
    //pcl::io::savePLYFile("/home/smorad/cloud.ply", *cloud_ptr);

    return cloud_ptr;
}

/* Compute normals and insert them into the cloud*/
pcl::PointCloud<pcl::PointNormal>::Ptr compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
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

    // Use all neighbors in a sphere of radius 10cm
    ne.setRadiusSearch(0.1);

    // Compute the features
    ne.compute(*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*



    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_ptr, *cloud_normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    return cloud_with_normals;
}

/* Build a tree of points+normals for faster searching*/
pcl::search::KdTree<pcl::PointNormal>::Ptr build_tree(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals) {
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    return tree2;
}

void process_marching_cubes(const sensor_msgs::PointCloud& input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = build_cloud(input);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = compute_normals(cloud_ptr);
    //pcl::search::KdTree<pcl::PointNormal>::Ptr tree = build_tree(cloud_with_normals);

    pcl::MarchingCubesRBF<pcl::PointNormal> mc;
    pcl::PolygonMesh mesh; // triangles(new pcl::PolygonMesh);
    mc.setInputCloud(cloud_with_normals);
    //mc.setSearchMethod(tree);
    mc.reconstruct(mesh);
    pcl::io::savePLYFile("/home/smorad/marching_cubes.ply", mesh);

}

void process_poisson(const sensor_msgs::PointCloud& input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = build_cloud(input);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = compute_normals(cloud_ptr);

    pcl::Poisson<pcl::PointNormal> poisson;
    pcl::PolygonMesh mesh;
    poisson.setDepth(7);
    poisson.setInputCloud(cloud_with_normals);
    poisson.reconstruct(mesh);
    pcl::io::savePLYFile("/home/smorad/mesh.ply", mesh);
}

void slow_boundary(pcl::PointCloud<pcl::PointXYZ>::ConstPtr world_cloud) {
    // compute normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(world_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(10);
    ne.compute(*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> estimator;
    estimator.setInputCloud(world_cloud);
    estimator.setInputNormals(normals);
    estimator.setRadiusSearch(15);
    estimator.setSearchMethod(tree);
    estimator.compute(boundaries);
    pcl::PointCloud<pcl::PointXYZ> p;
    for (int i = 0; i < boundaries.size(); ++i) {
        if (boundaries.at(i).boundary_point) {
            p.points.push_back(world_cloud->points[i]);
        }
    }
    std::cerr << "found boundaries: " << p.size() << " of total points " << world_cloud->points.size() << std::endl;

    pcl::io::savePLYFile("/home/smorad/boundaries.ply", p);

}

void process_fast_triangulate(const sensor_msgs::PointCloud& input) {

    //std::cout << "Processor got msg" << std::endl;

    // Convert from ROS to pcl cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = build_cloud(input);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = compute_normals(cloud_ptr);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 = build_tree(cloud_with_normals);


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

}


// Build the affine transformation matrix

Eigen::Matrix4f build_trans_mat(const gazebo_msgs::ModelState state) {
    //gazebo_msgs::ModelState state = state_buf;
    Eigen::Quaternionf q(state.pose.orientation.w, state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z);
    Eigen::Matrix3f A = q.toRotationMatrix();
    Eigen::Matrix4f res;
    /* rot rot rot x
     * rot rot rot y
     * rot rot rot z
     * 0   0   0   1
     */
    res << A(0, 0), A(0, 1), A(0, 2), state.pose.position.x,
            A(1, 0), A(1, 1), A(1, 2), state.pose.position.y,
            A(2, 0), A(2, 1), A(2, 2), state.pose.position.z,
            0, 0, 0, 1;

    //std::cerr << res << std::endl;

    return res;
}

Eigen::Matrix4f build_trans_mat2() {
    gazebo_msgs::ModelState state = state_buf;
    Eigen::Quaternionf q(state.pose.orientation.w, state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z);
    Eigen::Matrix3f A = q.toRotationMatrix();
    Eigen::Translation3f T(Eigen::Vector3f(state.pose.position.x, state.pose.position.y, state.pose.position.z));
    Eigen::Matrix4f res; // = T * A;
    /* rot rot rot x
     * rot rot rot y
     * rot rot rot z
     * 0   0   0   1
     */

    //std::cerr << res << std::endl;

    return res;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud;
Eigen::Vector3f previous_pos;

void prune_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f delta_r) {
    for (int i = 0; i < cloud->points.size(); ++i) {

    }
}

void compute_hop_vector_naive(const pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud) {
    //pcl::octree::OctreePointCloudDensity<pcl::PointXYZ> density_tree(50);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(world_cloud);
    int max_density = 0;
    pcl::PointXYZ target_point;
    for (int i = 0; i < world_cloud->points.size(); ++i) {
        pcl::IndicesPtr indices(new std::vector <int>);
        std::vector<float> distances;
        tree.radiusSearch(world_cloud->points[i], 20, *indices, distances);
        int density = indices->size();
        if (density > max_density) {
            max_density = density;
            target_point = world_cloud->points[i];
        }
    }
    std::cerr << "Heading towards point " << target_point << std::endl;
}

void compute_hop_vector_lazy_closest(const pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud) {
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    // Make a copy of world cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_with_pos(world_cloud);
    pcl::PointXYZ p(state_buf.pose.position.x, state_buf.pose.position.y, state_buf.pose.position.z);
    // Add our position as a point in the cloud to aid in searching
    cloud_with_pos->points.push_back(p);

    tree.setInputCloud(cloud_with_pos);
    int min_hole_density = 3;
    int max_hole_density = 6; // per sq meter
    pcl::PointXYZ target_point;
    for (int search_size = 100; search_size < cloud_with_pos->points.size(); search_size++) {
        pcl::IndicesPtr indices(new std::vector <int>);
        std::vector<float> distances;
        tree.nearestKSearch(p, search_size, *indices, distances);

        for (int i = 0; i < indices->size(); ++i) {
            pcl::IndicesPtr indices2(new std::vector <int>);
            std::vector<float> distances2;
            // Search the tree around each potential boundary point in indices
            tree.radiusSearch(world_cloud->points[indices->at(i)], 5, *indices2, distances2);
            int density = indices2->size();
            if (density > min_hole_density && density < max_hole_density) {
                int target_index = indices2->at(i);
                target_point = world_cloud->points[target_index];
                break;
            }
        }
    }
    std::cerr << "Heading towards point " << target_point << std::endl;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> c;
    c.points.push_back(target_point);
    pcl::io::savePLYFile("/home/smorad/cloud_target.ply", c);
}

void compute_hop_vector_lazy(const pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud) {
    //pcl::octree::OctreePointCloudDensity<pcl::PointXYZ> density_tree(50);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(world_cloud);
    int min_hole_density = 3;
    int max_hole_density = 6; // per sq meter
    pcl::PointXYZ target_point;
    for (int i = 0; i < world_cloud->points.size(); ++i) {
        pcl::IndicesPtr indices(new std::vector <int>);
        std::vector<float> distances;
        tree.radiusSearch(world_cloud->points[i], 5, *indices, distances);
        int density = indices->size();
        if (density > min_hole_density && density < max_hole_density) {
            target_point = world_cloud->points[i];
            break;
        }
    }
    std::cerr << "Heading towards point " << target_point << std::endl;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> c;
    c.points.push_back(target_point);
    pcl::io::savePLYFile("/home/smorad/cloud_target.ply", c);

}

void compute_hop_vector(const pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud) {
    // sample cloud, find areas with low point densities
    // voxel grid will not work because the volume density will be 0
    // past a wall
    // pcl::octree::OctreePointCloudDensity<pcl::PointXYZ> tree;

    // for point in cloud, best fit a plane
    //      for r = 1..4
    //          rotate clockwise in steps of 1/4 pi rad
    //              if more than 5pi/4 rads covered, it's not a boundary
    //              else it is a boundary point
    // head towards area with most boundary points

    // compute tree for fast neighbor searches
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(world_cloud);
    pcl::PointXYZ target_point;
    std::vector<pcl::PointXYZ> boundary_points;
    int max_boundary_points = 0;
    int k = 10;

    //std::map<pcl::PointXYZ, int> boundary;
    //for (pcl::PointXYZ p = world_cloud->points.begin(); p != world_cloud->points.end(); ++p) {
    for (int i = 0; i < world_cloud->points.size(); ++i) {
        // radius for hole searching
        pcl::PointXYZ p = world_cloud->points[i];
        // best fit plane
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setOptimizeCoefficients(true);
        seg.setDistanceThreshold(k);
        seg.setEpsAngle(0.5); // in rads
        pcl::IndicesPtr indices(new std::vector <int>);

        pcl::PointIndices::Ptr planar_points(new pcl::PointIndices);

        std::vector<float> distances;
        // compute points within radius 
        tree.radiusSearch(p, k, *indices, distances);
        //std::cerr << indices->size() << std::endl;
        // sometimes radiusSearch returns 5000 (way too many) indices then fails
        // to build a plane around them
        if (indices->size() > 3000) {
            continue;
        } else if (indices->size() < 10) {
            // too few for a plane, this is probably a boundary
            boundary_points.push_back(p);
            continue;
        }

        // build subcloud about point
        pcl::ExtractIndices<pcl::PointXYZ> filter;
        pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
        filter.setInputCloud(world_cloud);
        filter.setIndices(indices);
        filter.filter(*subcloud);
        seg.setInputCloud(subcloud);


        // find points inplane within radius k
        pcl::ModelCoefficients coefficients;
        seg.segment(*planar_points, coefficients);
        // if point density is less than some number, we have a hole
        // let's assume 8 points per sq meter
        int density_threshold = k * k * 8;
        if (planar_points->indices.size() < density_threshold) {
            // hole detected

            boundary_points.push_back(p);
        }
    }
    // Move to densest collection of boundary points per 5*k voxel
    /*pcl::octree::OctreePointCloudDensity<pcl::PointXYZ> density_tree(5 * k);
    int max_density = 0;
    int index = 0;
    for (int i = 0; i < boundary_points.size(); ++i) {
        int density = density_tree.getVoxelDensityAtPoint(boundary_points[i]);
        if (density > max_density) {
            max_density = density;
            index = i;
        }
    }*/
    std::cerr << "Heading towards point " << target_point << std::endl;
}

void remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(30);
    filter.setStddevMulThresh(3.0);
    filter.filter(*cloud_filtered);

    *cloud = *cloud_filtered;
}

void voxel_simplify(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.2f, 0.2f, 0.2f);
    filter.filter(*cloud_filtered);

    *cloud = *cloud_filtered;
}

void resample_simplify(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(3);
    mls.process(*cloud_filtered);


    *cloud = *cloud_filtered;
}

void simplify_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int pts_before = cloud->points.size();

    remove_outliers(cloud);
    voxel_simplify(cloud);

    int pts_after = cloud->points.size();
    std::cerr << "Reduced cloud from " << pts_before << " to " << pts_after << "vertices" << std::endl;

}

void slam(const sensor_msgs::PointCloud& input) {
    // Freeze position first thing, or it may change while we are processing
    gazebo_msgs::ModelState state = state_buf;


    // Convert from ROS to pcl cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = build_cloud(input);


    // Let first iteration be the origin in the inertial frame
    if (!world_cloud) {
        std::cerr << "World was null, initializing" << std::endl;

        // Simplify cloud so we don't run out of memory
        //simplify_cloud(cloud_ptr);
        world_cloud = cloud_ptr;

        previous_pos = Eigen::Vector3f(0, 0, 0);
        pcl::io::savePLYFile("/home/smorad/cloud_origin.ply", *cloud_ptr);
        // TODO: PASS THIS FROM GAZEBO
        float lidar_range = 100; // meters
        return;
    }

    // Simplify cloud so we don't run out of memory
    //simplify_cloud(world_cloud);

    // Grab most recent pos data
    Eigen::Matrix4f estimated_transform = build_trans_mat(state);

    // Transform incoming cloud to inertial frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_ptr, *input_cloud, estimated_transform);

    pcl::io::savePLYFile("/home/smorad/cloud_transformed.ply", *input_cloud);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the max correspondence distance to 25cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.25);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(250);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(0.001);

    
    // find theta1 and theta2 given distance traveled
    // only points in theta1 * r to theta2 * r will be used
    // domain: rdiff < x < r, theta1<theta<theta2
    
    // radial search
    // for each point in search, compute theta
    // circle eq (xc0-x0) ^2 + (yc0-y0) ^2 + (zc0-z0) ^2 = R^2
    // (xc0) ^2 + (yc0) ^2 + (zc0) ^2 = R^2
    // (xc1-x0) ^2 + (yc1-y0) ^2 + (zc1-z0) = (xc0-x0) ^2 + (yc0-y0) ^2 + (zc0-z0)
    
    
    // x^2 + y^2 = r^2
    // (x-x0)^2 + (y-y0)^2 = r^2
    
    // two intersection points should have the same radius
    
    
    // Align cloud_ptr to world_cloud
    icp.setInputSource(input_cloud);
    icp.setInputTarget(world_cloud);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    icp.align(output_cloud);

    if (!icp.hasConverged()) {
        std::cerr << "WARNING: SLAM did not converge" << std::endl;

        return;
    }
    Eigen::Matrix4f transformation_delta = icp.getFinalTransformation();
    std::cerr << "Fitness " << icp.getFitnessScore() << std::endl;

    // TODO remove this to detect slippage
    if (icp.getFitnessScore() > 3) {
        std::cerr << "WARNING: low fitness, discarding scan" << std::endl;
        return;
    }
    // Add new points to the world
    *world_cloud += output_cloud;
    if (world_cloud->points.size() > 20000) {
        //simplify_cloud(world_cloud);
    }
    //std::cerr << "All good, saving world" << std::endl;
    pcl::io::savePLYFile("/home/smorad/world.ply", *world_cloud);
    //compute_hop_vector_naive(world_cloud);
    slow_boundary(world_cloud);
}

void skew_image(const sensor_msgs::Image& input) {
    // Have slam write stamped partial surface
    // Have slam save transformation matrix A for partial surface
    // Apply A to entire pixel array
    // Perform SIFT to stitch images?

    // OR 
    // Take picture
    // Multiply by inv(A) to get t
    // Texture map picture to surface (this handles shearing/skewing/etc)
}

void store_pos(const gazebo_msgs::ModelStates& states) {
    //std::cerr<< state.name. << std::endl;
    for (int i = 0; i < states.name.size(); ++i) {
        if (states.name.at(i) == "SphereX") {
            gazebo_msgs::ModelState s;
            s.model_name = states.name.at(i);
            s.pose = states.pose.at(i);
            last_state_buf = state_buf;
            state_buf = s;

            break;
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "Main starting..." << std::endl;
    ros::init(argc, argv, "pcl_pipeline");
    ros::NodeHandle nh;
    //ros::Subscriber lidar_reader = nh.subscribe("lidar_stream", 1, process_fast_triangulate);
    ros::Subscriber pos_reader = nh.subscribe("gazebo/model_states", 1, store_pos);
    ros::Subscriber cloud_reader = nh.subscribe("pointcloud_buffer", 1, slam);
    ros::Subscriber image_reader = nh.subscribe("cam1_stream", 1, skew_image);
    ros::spin();
}

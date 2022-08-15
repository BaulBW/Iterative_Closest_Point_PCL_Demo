#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using std::cout;
using std::endl;


int ICP_monkey(std::string filepath)
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

    int iterations = 10;

    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile(filepath, *cloud_in) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", filepath);
        return (-1);
    }
    cout << "\nLoaded file " << filepath << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << endl;

    // Defining rotation matrix
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    double theta = M_PI / 8;  // The angle of rotation in radians
    transformation_matrix(0, 0) = std::cos(theta);
    transformation_matrix(0, 1) = -std::sin(theta);
    transformation_matrix(1, 0) = std::sin(theta);
    transformation_matrix(1, 1) = std::cos(theta);

    // Defining translation on Z
    transformation_matrix(2, 3) = 0.4;

    // Display in terminal the transformation matrix
    cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << endl;

    // Executing the transformation
    pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

    // The Iterative Closest Point algorithm
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(0.01); // convergence contidion is that MSE is smaller than threshold

    icp.setInputSource(cloud_icp);
    icp.setInputTarget(cloud_in);
    icp.align(*cloud_icp);
    icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
    cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << endl;

    if (icp.hasConverged())
    {
        cout << "ICP has converged\n";
        cout << "ICP score is: " << icp.getFitnessScore() << endl;
        cout << "transformation matrix is:\n";
        cout << icp.getFinalTransformation();
    }
    else
    {
        PCL_ERROR("\nICP has not converged.\n");
        return (-1);
    }
    return 0;
}
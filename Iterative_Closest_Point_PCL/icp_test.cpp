#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using PointT = pcl::PointXYZ;
using std::cout;
using std::endl;

void icp_test()
{
    // create and initialize PointT shared pointers
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>(5,1));
    pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);

    cloud_in->is_dense = false;

    // Random filling, float between 0 and 1024
    for (auto& pts : *cloud_in)
    {
        pts.x = 1024 * rand() / (RAND_MAX + 1.0f);
        pts.y = 1024 * rand() / (RAND_MAX + 1.0f);
        pts.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    cout << "Total num of points " << cloud_in->size() << endl;

    int ii = 0;
    for (auto& pts : *cloud_in)
    {
        ii++;
        cout << "Point" << ii << pts.x << "," << pts.y << "," << pts.z << endl;
    }
    
    *cloud_out = *cloud_in;

    // displace x coordinate by 0.7
    for (auto& point : *cloud_out)
        point.x += 0.7;

    ii = 0;
    for (auto& pts : *cloud_out)
    {
        ii++;
        cout << "Point" << ii << pts.x << "," << pts.y << "," << pts.z << endl;
    }

    // create icp object
    pcl::IterativeClosestPoint<PointT, PointT> icp;

    icp.setMaxCorrespondenceDistance(0.1);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(0.01); // convergence contidion is that MSE is smaller than threshold
    icp.setMaximumIterations(5);

    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<PointT> Final;
    icp.align(Final);


    if (icp.hasConverged())
    {
        cout << "ICP has converged\n";
        cout << "ICP score is: " << icp.getFitnessScore() << endl;
        cout << "transformation matrix is:\n";
        cout << icp.getFinalTransformation();
    }
    else
    {
        cout << "ICP didn't converge\n";
    }

    return;
}
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <string>

void icp_sample();
void icp_test();
void icp_nl_sample(char* argv);
int ICP_monkey(std::string filepath);

int main()
{
    //icp_sample();
    
    //icp_test();
    
    //icp_nl_sample("capture0001.pcd");

    ICP_monkey("monkey.ply");

    return 0;
}
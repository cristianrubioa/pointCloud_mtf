#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <boost/format.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

ros::Publisher pub1;

float theta = M_PI / 2; // Rotate 90 degress

void input(const sensor_msgs::PointCloud2ConstPtr &scan)
{

    // Msgs to pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan, *cloud); // Convert ROSMsg to pcl

    // Rotation matrix
    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    transform_1.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(*transformed_cloud, cloud_p);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "velodyne";
    pub1.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "input");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, input);
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_rotated", 100);
    ros::spin();
}

#include <iostream>
#include <cmath>
#include <vector>
#include <set>
#include <boost/format.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
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

float offset_pitch = M_PI / 2;        // 90 degress
float offset_yaw = 180 * M_PI / 180;  // 180 degress
float offset_roll = 180 * M_PI / 180; // 180 degress
float offset_x = 0.08;                // 8 centimeters

void angles(const sensor_msgs::ImuConstPtr &msg)
{
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(msg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    ROS_INFO("QUATERNION Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    ROS_INFO("EULER Imu Orientation Roll: [%f], Pitch: [%f], Yaw: [%f]", roll, pitch, yaw);
}

void transform(const sensor_msgs::PointCloud2ConstPtr &scan)
{
    // Msgs to pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*scan, *cloud); // Convert ROSMsg to pcl

    // Rotation matrix
    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    transform_1.rotate(Eigen::AngleAxisf(offset_pitch, Eigen::Vector3f::UnitY())); // Rotate offset_pitch degress
    transform_1.rotate(Eigen::AngleAxisf(offset_roll, Eigen::Vector3f::UnitX()));  // Rotate offset_roll degress
    //transform_1.rotate (Eigen::AngleAxisf (offset_yaw, Eigen::Vector3f::UnitZ()));	// Rotate offset_yaw degress

    // Traslation
    transform_1.translation() << offset_x, 0.0, 0.0;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_1);

    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(*transformed_cloud, cloud_p);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "quanergy";
    pub1.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "input");
    ros::NodeHandle nh;
    //printf("Running ...");
    ros::Subscriber subimu = nh.subscribe<sensor_msgs::Imu>("/vectornav/IMU", 1000, angles);          // /vectornav/IMU  /imu/data
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/quanergy/points", 100, transform); // /quanergy/points
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 100);
    ros::spin();
}

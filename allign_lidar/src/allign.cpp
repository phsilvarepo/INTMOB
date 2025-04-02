#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>  // For pcl_ros::transformPointCloud
#include <pcl_ros/transforms.h>

ros::Publisher transformed_cloud_pub;  // Publisher for transformed point cloud

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, tf2_ros::Buffer& tf_buffer)
{  
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Get the transform from LiDAR 2 to LiDAR 1 (i.e., from frame2 to frame1)
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer.lookupTransform("back_sim_lidar", "front_sim_lidar", ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    // Transform the point cloud from frame2 to frame1
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl_ros::transformPointCloud("front_sim_lidar", cloud, transformed_cloud, tf_buffer);

    // Convert the transformed pcl cloud back to ROS msg format
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(transformed_cloud, output_msg);

    // Set the frame_id to frame1 (this is necessary for proper visualization)
    output_msg.header.frame_id = "front_sim_lidar";

    // Publish the transformed point cloud
    transformed_cloud_pub.publish(output_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_transformer");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Subscribe to the point cloud topic from the second LiDAR
    ros::Subscriber point_cloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/point_cloud_back", 1, boost::bind(pointCloudCallback, _1, boost::ref(tf_buffer)));

    // Advertise the topic to publish the transformed point cloud
    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_back_lidar_topic", 1);

    ros::spin();
    return 0;
}
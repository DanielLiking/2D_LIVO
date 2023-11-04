#include  "r3dio.h"



void R3DIO::image_callback( const sensor_msgs::ImageConstPtr &msg )
{
    ROS_INFO("ffffff");
}

void R3DIO::imu_cbk(const sensor_msgs::Imu::ConstPtr & msg_in)
{
      ROS_INFO("ffffffsssss");
}

void R3DIO::depth_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    ROS_INFO("asdadad");
}

void R3DIO::image_comp_callback( const sensor_msgs::CompressedImageConstPtr &msg )
{
    ROS_INFO("asdadasd");
}
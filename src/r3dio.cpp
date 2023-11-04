#include  "r3dio.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


double vx, vy, vz;

void printf_field_name( sensor_msgs::PointCloud2::ConstPtr &msg )
{
    cout << "Input pointcloud field names: [" << msg->fields.size() << "]: ";
    for ( size_t i = 0; i < msg->fields.size(); i++ )
    {
        cout << msg->fields[ i ].name << ", ";
    }
    cout << endl;
}

void R3DIO::image_callback( const sensor_msgs::ImageConstPtr &msg )
{

}
void R3DIO::imu_cbk(const sensor_msgs::Imu::ConstPtr & msg_in)
{
   
}

void R3DIO::depth_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud< pcl::PointXYZRGB > pl;
    pcl::fromROSMsg( *msg, pl);
    // pcl::PointCloud< PointType > pl_line, pl_plane;
    uint  plsize = pl.size() - 1;
// s
    for ( size_t i = 0; i < msg->fields.size(); i++ )
    {
        cout << msg->fields[ i ].name << ", ";
        cout << pl[i].rgb << ",";
    }
    cout << endl;
}

void R3DIO::image_comp_callback( const sensor_msgs::CompressedImageConstPtr &msg )
{
 
}

void R3DIO::dep_img_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

}
namespace rgbd_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
}
#pragma once
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <common_lib.h>
// #include <kd_tree/ikd_Tree.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Vector3.h>


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "lib_sophus/so3.hpp"
#include "lib_sophus/se3.hpp"

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "tools_thread_pool.hpp"
#include "tools_ros.hpp"


class R3DIO
{
    public:
        ros::Subscriber sub_depth;
        ros::Subscriber sub_img, sub_img_comp;
        ros::Subscriber sub_imu;

        
        std::queue<sensor_msgs::ImuConstPtr> imu_buf;
        
        void depth_points_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg);
        void image_callback(const sensor_msgs::ImageConstPtr &msg);
        void image_comp_callback( const sensor_msgs::CompressedImageConstPtr &msg );
        ros::NodeHandle  nh;
        R3DIO()
        {
        
        std::string Depth_point_topic,  Image_topic, Image_Compressed_topic, Imu_topic;

        get_ros_parameter<std::string>(nh, "/Depth_point_topic", Depth_point_topic, std::string("/camera/depth/color/points") );
        get_ros_parameter<std::string>(nh, "/Image_topic", Image_topic, std::string("/camera/color/image_raw") );
        get_ros_parameter<std::string>(nh, "/IMU_topic", Imu_topic, std::string("/camera/imu") );
        Image_Compressed_topic = std::string(Image_topic).append("/compressed");
        if(1)
        {
            scope_color(ANSI_COLOR_BLUE_BOLD);
            cout << "======= Summary of subscribed topics =======" << endl;
            cout << "Depth Point topic: " << Depth_point_topic << endl;
            cout << "Image topic: " << Image_topic << endl;
            cout << "IMU topic: " <<  Imu_topic << endl;
            cout << "=======        -End-                =======" << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        sub_depth = nh.subscribe(Depth_point_topic.c_str(), 2000000, &R3DIO::depth_points_cbk, this, ros::TransportHints().tcpNoDelay());
        sub_img = nh.subscribe(Image_topic.c_str(), 1000000, &R3DIO::image_callback, this, ros::TransportHints().tcpNoDelay());
        sub_img_comp = nh.subscribe(Image_Compressed_topic.c_str(), 1000000, &R3DIO::image_comp_callback, this, ros::TransportHints().tcpNoDelay());
        sub_imu = nh.subscribe(Imu_topic.c_str(), 2000000, &R3DIO::imu_cbk, this, ros::TransportHints().tcpNoDelay());
        };
        ~R3DIO(){};





};
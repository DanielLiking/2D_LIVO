#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils.h"
#include "param.h"
#include "RGBDProcessingClass.h"

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "tools_openCV_3_to_4.hpp"

#include "r3dio.h"

MeasureGroup Measures;
StatesGroup dio_state;



int main(int argc, char const *argv[])
{
    printf_program("R3DIO START!!!");
    Common_tools::printf_software_version();
    Eigen::initParallel();
    ros::init(argc, argv, "R3DIO");
    R3DIO * Dio = new R3DIO();
    ros::Rate rate(5000);
    bool status = ros::ok();
    ros::spin;
}
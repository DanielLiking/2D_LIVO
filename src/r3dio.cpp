#include  "r3dio.h"

std::mutex mutex_image_callback;

void R3DIO::image_callback( const sensor_msgs::ImageConstPtr &msg )
{
    std::unique_lock< std::mutex > lock( mutex_image_callback );
    if ( sub_image_typed == 2 )
    {
        return; // Avoid subscribe the same image twice.
    }
    sub_image_typed = 1;

    if ( g_flag_if_first_rec_img )
    {
        g_flag_if_first_rec_img = 0;
        m_thread_pool_ptr->commit_task( &R3LIVE::service_process_img_buffer, this );
    }

    cv::Mat temp_img = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
    process_image( temp_img, msg->header.stamp.toSec() );
}







void R3DIO::imu_cbk(const sensor_msgs::Imu::ConstPtr & msg_in)
{
    sensor_msgs::Imu::Ptr msg( new sensor_msgs::Imu( *msg_in ) );
    double                timestamp = msg->header.stamp.toSec();
    g_camera_lidar_queue.imu_in( timestamp );
    mtx_buffer.lock();
    if ( timestamp + 13.0 < last_timestamp_imu )
    {
        ROS_INFO("timestamp is :%f", timestamp);
        ROS_INFO("last_timestamp is :%f", last_timestamp_imu);
        ROS_ERROR( "imu loop back, clear buffer" );
        imu_buffer_lio.clear();
        imu_buffer_vio.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    if ( g_camera_lidar_queue.m_if_acc_mul_G )
    {
        msg->linear_acceleration.x *= G_m_s2;
        msg->linear_acceleration.y *= G_m_s2;
        msg->linear_acceleration.z *= G_m_s2;
    }

    imu_buffer_lio.push_back( msg );
    imu_buffer_vio.push_back( msg );
    // std::cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer_lio.size()<<std::endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
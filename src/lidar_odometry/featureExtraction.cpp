#include "utility.h"
#include "lvi_sam/cloud_info.h"

#define max_scan_count 1500

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer
{
private:
    ros::Subscriber sub_corrected_pointcloud;
    ros::Publisher  pubFullPoints;
    ros::Publisher  pubCornerPoints;
    ros::Publisher  pubLinePoints;



    float edge_threshold_; // 提取角点的阈值

public:
    FeatureExtraction()
    {
         sub_corrected_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>(
            "/corrected_pointcloud", 5, &FeatureExtraction::corrected_pointcloud_Handler, this,
            ros::TransportHints().tcpNoDelay() );
       pubFullPoints = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/full", 5);
       pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("/lidar/feature/corner", 5);
       pubLinePoints = nh.advertise<sensor_msgs::PointCloud2>("/lidar/feature/line", 5);

        edge_threshold_ = 1.0;
    }
    void corrected_pointcloud_Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        std::vector<smoothness_t> scan_smoothness_(max_scan_count); // 存储每个点的曲率与索引
        float *scan_curvature_ = new float[max_scan_count];         // 存储每个点的曲率
        std::map<int, int> map_index;   // 有效点的索引 对应的 scan实际的索引
        int count = 0;                  // 有效点的索引
        
        pcl::PointCloud<pcl::PointXYZ> pl_full, pl_line, pl_corner;
        pcl::PointCloud<pcl::PointXYZ> pl_orig, pl_new;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        if(plsize==0) return;
        pl_full.reserve(plsize);
        pl_line.reserve(plsize);
        pl_corner.reserve(plsize);
        pl_new.reserve(plsize);

        for( int i=0; i < plsize; i++)
        {
            if(!std::isfinite(pl_orig.points[i].x) || !std::isfinite(pl_orig.points[i].y) || !std::isfinite(pl_orig.points[i].z))
            {
                continue;
            }
            // 这点在原始数据中的索引为i，在new_scan中的索引为count
            map_index[count] = i;
            pl_new.points[count] = pl_orig.points[i];
            count ++;
        }

        for( int i = 5; i < count -5; i ++)
        {
            float diffx = pl_new.points[i - 5].x + pl_new.points[i - 4].x 
                        + pl_new.points[i - 3].x + pl_new.points[i - 2].x 
                        + pl_new.points[i - 1].x - 10 * pl_new.points[i].x 
                        + pl_new.points[i + 1].x + pl_new.points[i + 2].x
                        + pl_new.points[i + 3].x + pl_new.points[i + 4].x
                        + pl_new.points[i + 5].x;
            float diffy = pl_new.points[i - 5].y + pl_new.points[i - 4].y 
                        + pl_new.points[i - 3].y + pl_new.points[i - 2].y 
                        + pl_new.points[i - 1].y - 10 * pl_new.points[i].y 
                        + pl_new.points[i + 1].y + pl_new.points[i + 2].y
                        + pl_new.points[i + 3].y + pl_new.points[i + 4].y
                        + pl_new.points[i + 5].y;
            float diffz = pl_new.points[i - 5].z + pl_new.points[i - 4].z 
                        + pl_new.points[i - 3].z + pl_new.points[i - 2].z 
                        + pl_new.points[i - 1].z - 10 * pl_new.points[i].z 
                        + pl_new.points[i + 1].z + pl_new.points[i + 2].z
                        + pl_new.points[i + 3].z + pl_new.points[i + 4].z
                        + pl_new.points[i + 5].z;
        
            scan_curvature_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            scan_smoothness_[i].value = scan_curvature_[i];
            scan_smoothness_[i].index = i;
        }


    }

  

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Lidar Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}
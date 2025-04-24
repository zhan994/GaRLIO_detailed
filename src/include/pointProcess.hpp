#ifndef __POINT_PROCESS_HPP__
#define __POINT_PROCESS_HPP__
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <livox_ros_driver/CustomMsg.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <omp.h>
#include <opencv/cv.h>
#include <tf/transform_broadcaster.h>

#include "common_lib.h"
#include "rio_utils/radar_point_cloud.h"
#include "radar_ego_velocity_estimator.h"


using std::atan2;
using std::cos;
using std::sin;

enum TIME_UNIT {SEC = 0, MS = 1, US = 2, NS = 3};
enum LID_TYPE {AVIA, HEASI};
enum RAR_TYPE {OCULII_1, OCULII_2};
enum Feature {Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround {Prev, Next};
enum E_jump {Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype {
    double range;
    double dista; 
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;

    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

namespace velodyne_ros {
    
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

namespace hesai_ros {
    
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} 
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

namespace ouster_ros {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t  ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

bool isValid(double x);

class PointProcess {
public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointProcess();
    ~PointProcess() = default;

    pcl::PointCloud<PointType> pl_full, pl_surf, pl_corn;
    int lidar_type, radar_type, N_SCANS, point_filter_num, time_uint;
    bool feature_enabled, given_offset_time;
    double blind;
    float time_unit_scale;
    float power_threshold;
    ros::NodeHandle nh;
    ros::Publisher pubFull, pubSurf, pubCorn, radar_raw_pub, pub_twist, pub_inlier_pc2, pub_outlier_pc2, points_pub;
    ros::Subscriber subPoints;
    ros::Subscriber subRadPoints;
    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_removal_filter;

private:
    void radar_handler1(const sensor_msgs::PointCloud::ConstPtr& msg);
    void radar_handler2(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void give_feature(pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, pcl::PointCloud<PointType> &pl_surf,
                      pcl::PointCloud<PointType> &pl_corn);
    void pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct);
    int  plane_judge(const pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
    bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, uint i, Surround nor_dir);
    pcl::PointCloud<RadarPointCloudType>::ConstPtr distance_filter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &cloud) const;
    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
    pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
    static bool time_list_ouster(ouster_ros::Point &point_1, ouster_ros::Point &point_2) {return (point_1.t < point_2.t);}
    static bool time_list_hesai(hesai_ros::Point &point_1, hesai_ros::Point &point_2) {return (point_1.timestamp < point_2.timestamp);}
    static bool time_list_avia(PointType &point_1, PointType &point_2) {return (point_1.curvature < point_2.curvature);}

    
    int group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
    cv::Mat livox_to_RGB;
    cv::Mat RGB_to_livox;
    cv::Mat Thermal_to_RGB;
    cv::Mat Radar_to_Thermal;
    cv::Mat Change_Radarframe;
    cv::Mat Radar_to_livox;
    rio::RadarEgoVelocityEstimator estimator;
    std::vector<double> egovel_time;
    std::vector<Eigen::VectorXi> num_at_dist_vec;

    std::string lidar_topic;
    std::string radar_topic;
    std::string topic_twist;
    std::string topic_inlier_pc2;
    std::string topic_outlier_pc2;
    std::string downsample_method;
    std::string outlier_removal_method;
    double distance_near_thresh;
    double distance_far_thresh;
    double z_low_thresh;
    double z_high_thresh;
    double downsample_resolution;
    double stddev_mul_thresh;
    double radius;
    int mean_k;
    int min_neighbors;
};

#endif

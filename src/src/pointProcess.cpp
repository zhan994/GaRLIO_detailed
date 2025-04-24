#include "pointProcess.hpp"

inline bool isValid(double x)
{
    return std::abs(x) > 1e8 ? true : false;
}

PointProcess::PointProcess()
{

    nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh.param<std::string>("common/radar_topic", radar_topic, "/radar_enhanced_pcl");
    nh.param<std::string>("common/topic_twist", topic_twist, "/eagle_data/twist");
    nh.param<std::string>("common/topic_inlier_pc2", topic_inlier_pc2, "/eagle_data/inlier_pc2");
    nh.param<std::string>("common/topic_outlier_pc2", topic_outlier_pc2, "/eagle_data/outlier_pc2");
    nh.param<float>("threshold/power_threshold", power_threshold, 0);
    nh.param<double>("threshold/distance_near_thresh", distance_near_thresh, 1.0);
    nh.param<double>("threshold/distance_far_thresh", distance_far_thresh, 100.0);
    nh.param<double>("threshold/z_low_thresh", z_low_thresh, -5.0);
    nh.param<double>("threshold/z_high_thresh", z_high_thresh, 20.0);
    nh.param<std::string>("threshold/downsample_method", downsample_method, "VOXELGRID");
    nh.param<double>("threshold/downsample_resolution", downsample_resolution, 0.1);
    nh.param<std::string>("threshold/outlier_removal_method", outlier_removal_method, "STATISTICAL");
    nh.param<int>("threshold/statistical_mean_k", mean_k, 20);
    nh.param<double>("threshold/statistical_stddev", stddev_mul_thresh, 1.0);
    nh.param<double>("threshold/radius_radius", radius, 0.8);
    nh.param<int>("threshold/radius_min_neighbors", min_neighbors, 2);

    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/radar_type", radar_type, OCULII_1);
    nh.param<int>("preprocess/n_scans", N_SCANS, 16);
    nh.param<int>("preprocess/point_filter_num", point_filter_num, 2);
    nh.param<int>("preprocess/timestamp_unit", time_uint, US);
    nh.param<bool>("preprocess/feature_extract_enable", feature_enabled, false);
    nh.param<bool>("preprocess/given_offset_time", given_offset_time, false);
    nh.param<double>("preprocess/blind", blind, 0.1);
    // Some default parameters
    nh.param<int>("preprocess/grounp_size", group_size, 8);
    nh.param<double>("preprocess/disA", disA, 0.01);
    nh.param<double>("preprocess/disB", disB, 0.1);
    nh.param<double>("preprocess/inf_bound", inf_bound, 10.0); // 4 ?
    nh.param<double>("preprocess/limit_maxmid", limit_maxmid, 6.25);
    nh.param<double>("preprocess/limit_midmin", limit_midmin, 6.25);
    nh.param<double>("preprocess/limit_maxmin", limit_maxmin, 3.24);
    nh.param<double>("preprocess/p2l_ratio", p2l_ratio, 225);
    nh.param<double>("preprocess/jump_up_limit", jump_up_limit, 170.0);
    nh.param<double>("preprocess/jump_down_limit", jump_down_limit, 8.0);
    nh.param<double>("preprocess/cos160", cos160, 160.0);
    nh.param<double>("preprocess/edgea", edgea, 2.0);
    nh.param<double>("preprocess/edgeb", edgeb, 0.1);
    nh.param<double>("preprocess/smallp_intersect", smallp_intersect, 172.5);
    nh.param<double>("preprocess/smallp_ratio", smallp_ratio, 1.2);

    jump_up_limit = cos(jump_up_limit / 180 * M_PI);
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);
    cos160 = cos(cos160 / 180 * M_PI);
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);

    switch (lidar_type)
    {
    case AVIA:
        subPoints = nh.subscribe<livox_ros_driver::CustomMsg>(lidar_topic, 1000, &PointProcess::avia_handler, this, ros::TransportHints().tcpNoDelay());
        break;

    case HEASI:
        subPoints = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1000, &PointProcess::hesai_handler, this, ros::TransportHints().tcpNoDelay());
        break;

    default:
        printf("Error LiDAR Type.\n");
        break;
    }

    switch(radar_type){
        case OCULII_1:
            subRadPoints = nh.subscribe<sensor_msgs::PointCloud>(radar_topic, 1000, &PointProcess::radar_handler1, this, ros::TransportHints().tcpNoDelay());
            break;
        case OCULII_2:
            subRadPoints = nh.subscribe<sensor_msgs::PointCloud2>(radar_topic, 1000, &PointProcess::radar_handler2, this, ros::TransportHints().tcpNoDelay());
            break;
        default:
            printf("Error Radar Type.\n");
            break;
    }

    switch (time_uint)
    {
    case SEC:
        time_unit_scale = 1.e3f;
        break;
    case MS:
        time_unit_scale = 1.f;
        break;
    case US:
        time_unit_scale = 1.e-3f;
        break;
    case NS:
        time_unit_scale = 1.e-6f;
        break;
    default:
        time_unit_scale = 1.f;
        break;
    }

    pubFull = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 100);
    pubSurf = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100);
    pubCorn = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    radar_raw_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_raw", 100);
    pub_twist = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(topic_twist, 5);
    pub_inlier_pc2 = nh.advertise<sensor_msgs::PointCloud2>(topic_inlier_pc2, 5);
    pub_outlier_pc2 = nh.advertise<sensor_msgs::PointCloud2>(topic_outlier_pc2, 5);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
}

pcl::PointCloud<RadarPointCloudType>::ConstPtr PointProcess::distance_filter(const pcl::PointCloud<RadarPointCloudType>::ConstPtr &cloud) const
{
    pcl::PointCloud<RadarPointCloudType>::Ptr filtered(new pcl::PointCloud<RadarPointCloudType>());

    filtered->reserve(cloud->size());
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const RadarPointCloudType& p) {
      double d = p.getVector3fMap().norm();
      double z = p.z;
      return d > distance_near_thresh && d < distance_far_thresh && z < z_high_thresh && z > z_low_thresh;
    });


    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
}

pcl::PointCloud<PointT>::ConstPtr PointProcess::outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

pcl::PointCloud<PointT>::ConstPtr PointProcess::downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
        // Remove NaN/Inf points
        pcl::PointCloud<PointT>::Ptr cloudout(new pcl::PointCloud<PointT>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloudout, indices);
        
        return cloudout;
    }

    pcl::RandomSample<PointT> random_sample = new pcl::RandomSample<PointT>(true);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    random_sample.setInputCloud(cloud);
    random_sample.setSample(20);
    random_sample.filter(*filtered);
    pcl::IndicesConstPtr removed_indices = random_sample.getRemovedIndices();

    pcl::IndicesConstPtr all_indices = random_sample.getIndices();
    // Create sets for retained and removed indices
    std::set<int> retained_set(all_indices->begin(), all_indices->end());
    std::set<int> removed_set(removed_indices->begin(), removed_indices->end());

    // Find the difference between the retained and removed sets
    std::set<int> difference;
    std::set_difference(retained_set.begin(), retained_set.end(),
                        removed_set.begin(), removed_set.end(),
                        std::inserter(difference, difference.begin()));
    filtered->header = cloud->header;

    return filtered;
}

void PointProcess::radar_handler1(const sensor_msgs::PointCloud::ConstPtr &msg)
{

    RadarPointCloudType radarpoint_raw;
    PointT radarpoint_xyzi;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw(new pcl::PointCloud<RadarPointCloudType>);
    pcl::PointCloud<PointT>::Ptr radarcloud_xyzi(new pcl::PointCloud<PointT>);

    radarcloud_xyzi->header.frame_id = msg->header.frame_id;
    radarcloud_xyzi->header.seq = msg->header.seq;
    radarcloud_xyzi->header.stamp = msg->header.stamp.toSec() * 1e6;
    for (int i = 0; i < msg->points.size(); i++)
    {
        if (msg->channels[2].values[i] > power_threshold)
        {
            if (msg->points[i].x == NAN || msg->points[i].y == NAN || msg->points[i].z == NAN)
                continue;
            if (msg->points[i].x == INFINITY || msg->points[i].y == INFINITY || msg->points[i].z == INFINITY)
                continue;
            cv::Mat ptMat, dstMat;
            ptMat = (cv::Mat_<double>(4, 1) << msg->points[i].x, msg->points[i].y, msg->points[i].z, 1);
            dstMat = ptMat; 
            radarpoint_raw.x = dstMat.at<double>(0, 0);
            radarpoint_raw.y = dstMat.at<double>(1, 0);
            radarpoint_raw.z = dstMat.at<double>(2, 0);
            radarpoint_raw.intensity = msg->channels[2].values[i];
            radarpoint_raw.doppler = msg->channels[0].values[i];
            radarpoint_xyzi.x = dstMat.at<double>(0, 0);
            radarpoint_xyzi.y = dstMat.at<double>(1, 0);
            radarpoint_xyzi.z = dstMat.at<double>(2, 0);
            radarpoint_xyzi.intensity = msg->channels[2].values[i];

            radarcloud_raw->points.push_back(radarpoint_raw);
            radarcloud_xyzi->points.push_back(radarpoint_xyzi);
        }
    }

    //********** Publish PointCloud2 Format Raw Cloud **********
    sensor_msgs::PointCloud2 pc2_raw_msg;
    pcl::toROSMsg(*radarcloud_raw, pc2_raw_msg);

    pc2_raw_msg.header = msg->header;
    radar_raw_pub.publish(pc2_raw_msg);

    //********** Ego Velocity Estimation **********
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
    clock_t start_ms = clock();
    if (estimator.estimate(pc2_raw_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg))
    {
        clock_t end_ms = clock();
        double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
        egovel_time.push_back(time_used);

        geometry_msgs::TwistWithCovarianceStamped twist;
        twist.header.stamp = pc2_raw_msg.header.stamp;
        twist.twist.twist.linear.x = v_r.x();
        twist.twist.twist.linear.y = v_r.y();
        twist.twist.twist.linear.z = v_r.z();

        twist.twist.covariance.at(0) = std::pow(sigma_v_r.x(), 2);
        twist.twist.covariance.at(7) = std::pow(sigma_v_r.y(), 2);
        twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

        pub_twist.publish(twist);
        pub_inlier_pc2.publish(inlier_radar_msg);
        pub_outlier_pc2.publish(outlier_radar_msg);
    }
    else
    {
        ;
    }

    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_inlier(new pcl::PointCloud<RadarPointCloudType>);
    pcl::fromROSMsg(inlier_radar_msg, *radarcloud_inlier);

    pcl::PointCloud<RadarPointCloudType>::ConstPtr src_cloud;
    src_cloud = radarcloud_inlier;

    if (src_cloud->empty())
    {
        return;
    }


    pcl::PointCloud<RadarPointCloudType>::ConstPtr filtered = distance_filter(src_cloud);
    points_pub.publish(*filtered);
}

void PointProcess::radar_handler2(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

    RadarPointCloudType radarpoint_raw;
    PointT radarpoint_xyzi;
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_raw(new pcl::PointCloud<RadarPointCloudType>);
    pcl::PointCloud<EaglePointXYZIVRAB2>::Ptr radarcloud_msg(new pcl::PointCloud<EaglePointXYZIVRAB2>);

    pcl::fromROSMsg(*msg, *radarcloud_msg);

    pcl::PointCloud<PointT>::Ptr radarcloud_xyzi(new pcl::PointCloud<PointT>);

    radarcloud_xyzi->header.frame_id = msg->header.frame_id;
    radarcloud_xyzi->header.seq = msg->header.seq;
    radarcloud_xyzi->header.stamp = msg->header.stamp.toSec() * 1e6;
    for (int i = 0; i < radarcloud_msg->points.size(); i++)
    {   
    if (radarcloud_msg->points[i].Power > power_threshold)
        {
            if (radarcloud_msg->points[i].x == NAN || radarcloud_msg->points[i].y == NAN || radarcloud_msg->points[i].z == NAN)
                continue;
            if (radarcloud_msg->points[i].x == INFINITY || radarcloud_msg->points[i].y == INFINITY || radarcloud_msg->points[i].z == INFINITY)
                continue;
            cv::Mat ptMat, dstMat;
            ptMat = (cv::Mat_<double>(4, 1) << radarcloud_msg->points[i].x, radarcloud_msg->points[i].y, radarcloud_msg->points[i].z, 1);
            dstMat = ptMat; 
            radarpoint_raw.x = dstMat.at<double>(0, 0);
            radarpoint_raw.y = dstMat.at<double>(1, 0);
            radarpoint_raw.z = dstMat.at<double>(2, 0);
            radarpoint_raw.intensity = radarcloud_msg->points[i].Power;
            radarpoint_raw.doppler = radarcloud_msg->points[i].Doppler;
            radarpoint_xyzi.x = dstMat.at<double>(0, 0);
            radarpoint_xyzi.y = dstMat.at<double>(1, 0);
            radarpoint_xyzi.z = dstMat.at<double>(2, 0);
            radarpoint_xyzi.intensity = radarcloud_msg->points[i].Power;

            radarcloud_raw->points.push_back(radarpoint_raw);
            radarcloud_xyzi->points.push_back(radarpoint_xyzi);
        }
    }

    //********** Publish PointCloud2 Format Raw Cloud **********
    sensor_msgs::PointCloud2 pc2_raw_msg;
    pcl::toROSMsg(*radarcloud_raw, pc2_raw_msg);

    pc2_raw_msg.header = msg->header;
    radar_raw_pub.publish(pc2_raw_msg);

    //********** Ego Velocity Estimation **********
    Eigen::Vector3d v_r, sigma_v_r;
    sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
    clock_t start_ms = clock();
    if (estimator.estimate(pc2_raw_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg))
    {
        clock_t end_ms = clock();
        double time_used = double(end_ms - start_ms) / CLOCKS_PER_SEC;
        egovel_time.push_back(time_used);

        geometry_msgs::TwistWithCovarianceStamped twist;
        twist.header.stamp = pc2_raw_msg.header.stamp;
        twist.twist.twist.linear.x = v_r.x();
        twist.twist.twist.linear.y = v_r.y();
        twist.twist.twist.linear.z = v_r.z();

        twist.twist.covariance.at(0) = std::pow(sigma_v_r.x(), 2);
        twist.twist.covariance.at(7) = std::pow(sigma_v_r.y(), 2);
        twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

        pub_twist.publish(twist);
        pub_inlier_pc2.publish(inlier_radar_msg);
        pub_outlier_pc2.publish(outlier_radar_msg);
    }
    else
    {
        ;
    }
    pcl::PointCloud<RadarPointCloudType>::Ptr radarcloud_inlier(new pcl::PointCloud<RadarPointCloudType>);
    pcl::fromROSMsg(inlier_radar_msg, *radarcloud_inlier);

    pcl::PointCloud<RadarPointCloudType>::ConstPtr src_cloud;
    src_cloud = radarcloud_inlier;
    if (src_cloud->empty())
    {
        return;
    }


    pcl::PointCloud<RadarPointCloudType>::ConstPtr filtered = distance_filter(src_cloud);
    points_pub.publish(*filtered);
}


void PointProcess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    std::vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
    std::vector<std::vector<orgtype>> typess(N_SCANS);

    uint plsize = msg->point_num;

    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    for (int i = 0; i < N_SCANS; i++)
    {
        pl_buff[i].reserve(plsize);
    }
    uint valid_num = 0;

    if (feature_enabled)
    {
        for (uint i = 1; i < plsize; i++)
        {
            if ((msg->points[i].line < N_SCANS) &&
                ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->points[i].offset_time * time_unit_scale; // use curvature as time of each laser points
                

                if ((std::abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (std::abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
                    (std::abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
                {
                    pl_buff[msg->points[i].line].push_back(pl_full[i]);
                }
            }
        }
        static int count = 0;
        static double time = 0.0;
        count++;
        double t0 = omp_get_wtime();
        for (int j = 0; j < N_SCANS; j++)
        {
            if (pl_buff[j].size() <= 5)
                continue;
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            plsize = pl.size();
            std::vector<orgtype> &types = typess[j];
            types.clear();
            types.resize(plsize);
            plsize--;
            for (uint i = 0; i < plsize; i++)
            {
                types[i].range = std::sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = std::sqrt(vx * vx + vy * vy + vz * vz);
            }
            types[plsize].range = std::sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
            give_feature(pl, types, pl_surf, pl_corn);
        }
        time += omp_get_wtime() - t0;
        // printf("Feature extraction time: %lf \n", time / count);
    }
    else
    {
        for (uint i = 1; i < plsize; i++)
        {
            if ((msg->points[i].line < N_SCANS) &&
                ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
            {
                valid_num++;
                if (valid_num % point_filter_num == 0)
                {
                    pl_full[i].x = msg->points[i].x;
                    pl_full[i].y = msg->points[i].y;
                    pl_full[i].z = msg->points[i].z;
                    pl_full[i].intensity = msg->points[i].reflectivity;
                    pl_full[i].curvature = msg->points[i].offset_time * time_unit_scale; // use curvature as time of each laser points, curvature unit: ms

                    if (((std::abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (std::abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
                         (std::abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) &&
                        (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
                    {
                        pl_surf.push_back(pl_full[i]);
                    }
                }
            }
        }
    }
    std::sort(pl_surf.points.begin(), pl_surf.points.end(), time_list_avia);
    pub_func(pl_surf, pubSurf, msg->header.stamp);
}

void PointProcess::hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    std::vector<pcl::PointCloud<PointType>> pl_buff(N_SCANS);
    std::vector<std::vector<orgtype>> typess(N_SCANS);

    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    std::sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_hesai);

    if (feature_enabled) {
        for (int i = 0; i < N_SCANS; i++) {
            pl_buff[i].reserve(plsize);
        }

        for (uint i = 0; i < plsize; i++) {
            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + 
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range < (blind * blind)) continue;

            PointType added_pt;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
            if (yaw_angle >= 180.0) {
                yaw_angle -= 360.0;
            }
            if (yaw_angle <= -180.0) {
                yaw_angle += 360.0;
            }

            added_pt.curvature = pl_orig.points[i].timestamp * time_unit_scale;
            if (pl_orig.points[i].ring < N_SCANS) {
                pl_buff[pl_orig.points[i].ring].push_back(added_pt);
            }
        }

        for (int j = 0; j < N_SCANS; j++) {
            pcl::PointCloud<PointType> &pl = pl_buff[j];
            int linesize = pl.size();
            std::vector<orgtype> &types = typess[j];
            types.clear();
            types.resize(linesize);
            linesize--;
            for (uint i = 0; i < linesize; i++) {
                types[i].range = std::sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[linesize].range = std::sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
            give_feature(pl, types, pl_surf, pl_corn);
        }
    }
    else {
        for (int i = 0; i < pl_orig.points.size(); i++) {
            if (i % point_filter_num != 0) continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + 
                           pl_orig.points[i].z * pl_orig.points[i].z;  
            if (range < (blind * blind)) continue;
            
            PointType added_pt;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.curvature = (pl_orig.points[i].timestamp-msg->header.stamp.toSec()) * time_unit_scale; // curvature unit: ms
            pl_surf.points.push_back(added_pt);
        }
    }
    pub_func(pl_surf, pubSurf, msg->header.stamp);
}

void PointProcess::give_feature(pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, pcl::PointCloud<PointType> &pl_surf,
                                pcl::PointCloud<PointType> &pl_corn)
{
    int plsize = pl.size();
    int plsize2;
    if (plsize == 0) {
        printf("something wrong\n");
        return;
    }

    uint head = 0;
    while (types[head].range < blind) {
        head++;
    }

    // Surf
    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex = 0, i2;
    uint last_i = 0; 
    uint last_i_nex = 0;
    int last_state = 0;
    int plane_type;

    for (uint i = head; i < plsize2; i++) {
        if (types[i].range < blind) {
            continue;
        }

        i2 = i;

        plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
        
        if (plane_type == 1) {
            for (uint j = i; j <= i_nex; j++) { 
                if (j != i && j != i_nex) {
                    types[j].ftype = Real_Plane;
                }
                else {
                    types[j].ftype = Poss_Plane;
                }
            }
            
            if (last_state == 1 && last_direct.norm() > 0.1) {
                double mod = last_direct.transpose() * curr_direct;
                if (mod > -0.707 && mod < 0.707) {
                    types[i].ftype = Edge_Plane;
                }
                else {
                    types[i].ftype = Real_Plane;
                }
            }
            
            i = i_nex - 1;
            last_state = 1;
        }
        else { 
            i = i_nex;
            last_state = 0;
        }

        last_i = i2;
        last_i_nex = i_nex;
        last_direct = curr_direct;
    }

    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for (uint i = head + 3; i < plsize2; i++) {
        if (types[i].range < blind || types[i].ftype >= Real_Plane) {
            continue;
        }

        if (types[i-1].dista < 1e-16 || types[i].dista < 1e-16) {
            continue;
        }

        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
        Eigen::Vector3d vecs[2];

        for (int j = 0; j < 2; j++) {
            int m = -1;
            if (j == 1) {
                m = 1;
            }

            if (types[i + m].range < blind) {
                if (types[i].range > inf_bound) {
                    types[i].edj[j] = Nr_inf;
                }
                else {
                    types[i].edj[j] = Nr_blind;
                }
                continue;
            }

            vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
            vecs[j] = vecs[j] - vec_a;
            
            types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
            if (types[i].angle[j] < jump_up_limit) {
                types[i].edj[j] = Nr_180;
            }
            else if (types[i].angle[j] > jump_down_limit) {
                types[i].edj[j] = Nr_zero;
            }
        }

        types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
        if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && 
            types[i].dista > 4 * types[i - 1].dista) {
            if (types[i].intersect > cos160) {
                if(edge_jump_judge(pl, types, i, Prev)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        }
        else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && 
                 types[i - 1].dista > 4 * types[i].dista) {
            if (types[i].intersect > cos160) {
                if (edge_jump_judge(pl, types, i, Next)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        }
        else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
            if (edge_jump_judge(pl, types, i, Prev)) {
                types[i].ftype = Edge_Jump;
            }
        }
        else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
            if (edge_jump_judge(pl, types, i, Next)) {
                types[i].ftype = Edge_Jump;
            }      
        }
        else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
            if (types[i].ftype == Nor) {
                types[i].ftype = Wire;
            }
        }
    }

    plsize2 = plsize - 1;
    double ratio;
    for (uint i = head + 1; i < plsize2; i++) {
        if(types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) {
            continue;
        }
        
        if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) {
            continue;
        }

        if (types[i].ftype == Nor) {
            if (types[i - 1].dista > types[i].dista) {
                ratio = types[i - 1].dista / types[i].dista;
            }
            else {
                ratio = types[i].dista / types[i - 1].dista;
            }

            if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
                if (types[i - 1].ftype == Nor) {
                    types[i - 1].ftype = Real_Plane;
                }
                if (types[i + 1].ftype == Nor) {
                    types[i + 1].ftype = Real_Plane;
                }
                types[i].ftype = Real_Plane;
            }
        }
    }

    int last_surface = -1;
    for (uint j = head; j < plsize; j++) {
        if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
            if (last_surface == -1) {
                last_surface = j;
            }
            
            if (j == uint(last_surface + point_filter_num - 1)) {
                PointType ap;
                ap.x = pl[j].x;
                ap.y = pl[j].y;
                ap.z = pl[j].z;
                ap.intensity = pl[j].intensity;
                ap.curvature = pl[j].curvature;
                pl_surf.push_back(ap);

                last_surface = -1;
            }
        }
        else {
            if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
                pl_corn.push_back(pl[j]);
            }
            if (last_surface != -1) {
                PointType ap;
                for (uint k = last_surface; k < j; k++) {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.intensity += pl[k].intensity;
                    ap.curvature += pl[k].curvature;
                }
                ap.x /= (j - last_surface);
                ap.y /= (j - last_surface);
                ap.z /= (j - last_surface);
                ap.intensity /= (j - last_surface);
                ap.curvature /= (j - last_surface);
                pl_surf.push_back(ap);
            }
            last_surface = -1;
        }
    }
}

void PointProcess::pub_func(pcl::PointCloud<PointType> &pl, ros::Publisher pub, const ros::Time &ct)
{
    pl.height = 1;
    pl.width = pl.size();
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(pl, output_msg);
    output_msg.header.stamp = ct;
    output_msg.header.frame_id = "livox";
    pub.publish(output_msg);
}

int PointProcess::plane_judge(const pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
    double group_dis = disA * types[i_cur].range + disB;
    group_dis = group_dis * group_dis;
    // i_nex = i_cur;

    double two_dis;
    std::vector<double> disarr;
    disarr.reserve(20);

    for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++) {
        if (types[i_nex].range < blind) {
            curr_direct.setZero();
            return 2;
        }
        disarr.push_back(types[i_nex].dista);
    }
    
    for (;;) {
        if ( (i_cur >= pl.size()) || (i_nex >= pl.size()) ) break;

        if (types[i_nex].range < blind) {
            curr_direct.setZero();
            return 2;
        }
        vx = pl[i_nex].x - pl[i_cur].x;
        vy = pl[i_nex].y - pl[i_cur].y;
        vz = pl[i_nex].z - pl[i_cur].z;
        two_dis = vx * vx + vy * vy + vz * vz;
        if (two_dis >= group_dis) {
            break;
        }
        disarr.push_back(types[i_nex].dista);
        i_nex++;
    }

    double leng_wid = 0;
    double v1[3], v2[3];
    for (uint j = i_cur + 1; j < i_nex; j++) {
        if ( (j >= pl.size()) || (i_cur >= pl.size()) ) break;
        v1[0] = pl[j].x - pl[i_cur].x;
        v1[1] = pl[j].y - pl[i_cur].y;
        v1[2] = pl[j].z - pl[i_cur].z;

        v2[0] = v1[1]*vz - vy*v1[2];
        v2[1] = v1[2]*vx - v1[0]*vz;
        v2[2] = v1[0]*vy - vx*v1[1];

        double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
        if (lw > leng_wid) {
            leng_wid = lw;
        }
    }


    if ((two_dis * two_dis / leng_wid) < p2l_ratio) {
        curr_direct.setZero();
        return 0;
    }

    uint disarrsize = disarr.size();
    for (uint j = 0; j < disarrsize - 1; j++) {
        for (uint k = j + 1; k < disarrsize; k++) {
            if (disarr[j] < disarr[k]) {
                leng_wid = disarr[j];
                disarr[j] = disarr[k];
                disarr[k] = leng_wid;
            }
        }
    }

    if (disarr[disarr.size() - 2] < 1e-16) {
        curr_direct.setZero();
        return 0;
    }

    if (lidar_type == AVIA) {
        double dismax_mid = disarr[0] / disarr[disarrsize / 2];
        double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

        if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
            curr_direct.setZero();
            return 0;
        }
    }
    else {
        double dismax_min = disarr[0] / disarr[disarrsize - 2];
        if (dismax_min >= limit_maxmin) {
            curr_direct.setZero();
            return 0;
        }
    }
    
    curr_direct << vx, vy, vz;
    curr_direct.normalize();
    return 1;
}

bool PointProcess::edge_jump_judge(const pcl::PointCloud<PointType> &pl, std::vector<orgtype> &types, uint i, Surround nor_dir)
{
    if (nor_dir == 0) {
        if (types[i - 1].range < blind || types[i - 2].range < blind) {
            return false;
        }
    }
    else if (nor_dir == 1) {
        if (types[i + 1].range < blind || types[i + 2].range < blind) {
            return false;
        }
    }
    double d1 = types[i + nor_dir - 1].dista;
    double d2 = types[i + 3 * nor_dir - 2].dista;
    double d;

    if (d1 < d2) {
        d = d1;
        d1 = d2;
        d2 = d;
    }

    d1 = std::sqrt(d1);
    d2 = std::sqrt(d2);

    
    if (d1 > edgea * d2 || (d1 - d2) > edgeb) {
        return false;
    }
    
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_process");

    PointProcess *process = new PointProcess();
    ROS_INFO("\033[1;32m---->\033[0m Pointprocess Started.");

    ros::spin();
    return 0;
}
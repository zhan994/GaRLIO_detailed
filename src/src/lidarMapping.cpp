// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Modifier: Pengcheng Shi          pengchengshi1995@gmail.com

//Modifier : Chiyun Noh				gch06208@snu.ac.kr

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <deque>
#include <thread>
#include <condition_variable>
#include <iostream>
#include <string>
#include <omp.h>
#include <opencv/cv.h>
#include <visualization_msgs/Marker.h>



#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rio_utils/radar_point_cloud.h"
#include "lidarFactor.hpp"
#include "imuProcess.hpp"
#include "tools/timer.hpp"
#include "filter_state.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "radar_ego_velocity_estimator.h"

int frameCount = 0;
double whole_map_time = 0, aver_time_consu = 0;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxSyncPolicy;

pcl::PointCloud<PointType>::Ptr featsUndistort(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr featsDownBody(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr featsDownWorld(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr featsFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr featsArray(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr map_save(new pcl::PointCloud<PointType>());

std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lidar_sub;
std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> radar_sub;
std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_RL;

std::deque<double> timeBuf;
std::deque<pcl::PointCloud<PointType>::Ptr> lidarBuf;
std::deque<pcl::PointCloud<RadarPointCloudType>::Ptr> radarBuf;
std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> radar_outlier_Buf;


std::deque<PointBufferType> point_buffer;
std::deque<sensor_msgs::ImuConstPtr> imuBuf;
std::mutex mBuf;
std::condition_variable sigBuf;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
ros::Publisher pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubLaserAfterMappedPath;
ros::Publisher marker_pub, gravity_pub;
Eigen::Vector3d gravity_cal;

nav_msgs::Path laserAfterMappedPath;
Eigen::Quaterniond geoQuat;
std::string result_path, imu_topic;

Eigen::Matrix<double, 4, 4> livox_to_RGB;
Eigen::Matrix<double, 4, 4> RGB_to_livox;
Eigen::Matrix<double, 4, 4> Thermal_to_RGB;
Eigen::Matrix<double, 4, 4> Radar_to_Thermal;
Eigen::Matrix<double, 4, 4> Change_Radarframe;
Eigen::Matrix<double, 4, 4> Radar_to_livox;
double grav_residual;


bool is_first_scan = true, undistortion = false, lidar_pushed = false, EKF_init_flag = false, pcd_save_en = false;
double LASER_POINT_COV = 0.001, VELOCITY_COV = 0.05, gravity_COV = 0.01;
int init_imu_num = 1, feats_num = 0, scan_num = 0;
double first_lidar_time = 0, last_imu_time = -1.0, last_radar_time = 0, last_lidar_time = 0, lidar_end_time = 0, lidar_mean_time = 0;
double gyr_noise = 0.1, gyr_bias_noise = 0.0001, acc_noise = 0.1, acc_bias_noise = 0.0001;

// double loss_threshold = 0.1;
V3D lid_pos;
M3D R_i2l(M3D::Identity());
V3D t_i2l(V3D::Zero());
Eigen::Matrix4d T_r2l(Eigen::Matrix4d::Identity());
Eigen::Matrix4d T_r2i(Eigen::Matrix4d::Identity());

std::shared_ptr<ImuProcess> pImu_(new ImuProcess());
MeasureGroup Measures;
StateGroup lio_state;
StateGroup propagate_state;

V3D last_radar_vel;
V3D current_radar_vel;
M3D last_rot;
V3D current_angular_vel;
double time_max = 0;
int interval = 0;

rio::RadarEgoVelocityEstimator estimator;
Eigen::Vector3d v_r, sigma_v_r;
sensor_msgs::PointCloud2 pc2_raw_msg;

//filtering
std::string downsample_method;
std::string outlier_removal_method;
double downsample_resolution;
double stddev_mul_thresh;
double radius;
int mean_k;
int min_neighbors;
pcl::Filter<PointT>::Ptr downsample_filter;
pcl::Filter<PointT>::Ptr outlier_removal_filter;
double distance_near_thresh;
double distance_far_thresh;
double z_low_thresh;
double z_high_thresh;
int id = 0;

V3D gravity_world;
bool gravity_init = false;

// ikdtree map 
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double cube_len = 0, filter_size_surf = 0, filter_size_map = 0;
int  kdtree_surf_delete_counter = 0, feats_down_size = 0;
std::vector<BoxPointType> cub_needrm;
KD_TREE<PointType> ikdtreeSurf;
std::vector<PointVector> nearestPointsSurf;

// Viravbles for IESKF
bool EKF_converged_flag = false, extrinsic_est_en = true;
int effct_feat_num = 0;
int NUM_MAX_ITERATIONS = 0;
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
std::vector<double> lidar_weight;

void pub_func(pcl::PointCloud<pcl::PointXYZI>::Ptr &pl, ros::Publisher pub, const ros::Time &ct)
{
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*pl, output_msg);
    output_msg.header.stamp = ct;
    output_msg.header.frame_id = "livox";
    pub.publish(output_msg);
}


void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d p_cur(pi->x, pi->y, pi->z);
	// transform from current lidar to current imu frame
	Eigen::Vector3d p_cur_i = lio_state.rot_ex_i2l * p_cur + lio_state.pos_ex_i2l;
	// transform from current imu to world frame
	Eigen::Vector3d p_w = lio_state.rot_end * p_cur_i + lio_state.pos_end; 

	po->x = p_w.x();
	po->y = p_w.y();
	po->z = p_w.z();
	po->intensity = pi->intensity;
}

void pointAssociateToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po)
{
	Eigen::Vector3d p_cur(pi->x, pi->y, pi->z);
	// transform from current lidar to current imu frame
	Eigen::Vector3d p_cur_i = lio_state.rot_ex_i2l * p_cur + lio_state.pos_ex_i2l;
	// transform from current imu to world frame
	Eigen::Vector3d p_w = lio_state.rot_end * p_cur_i + lio_state.pos_end; 

	po->x = p_w.x();
	po->y = p_w.y();
	po->z = p_w.z();
	//po->intensity = 1.0;
}


void pointAssociateToBody(PointType *const p_w, PointType *const p_b)
{
	Eigen::Vector3d p_cur(p_w->x, p_w->y, p_w->z);
	// transform from world frame to current imu frame
	Eigen::Vector3d p_cur_i = lio_state.rot_end.transpose() * (p_cur - lio_state.pos_end);
	// transform from current imu to current lidar 
	Eigen::Vector3d p_l = lio_state.rot_ex_i2l.transpose() * (p_cur_i - lio_state.pos_ex_i2l); 

	p_b->x = p_l.x();
	p_b->y = p_l.y();
	p_b->z = p_l.z();
	p_b->intensity = p_w->intensity;
	//po->intensity = 1.0;
}

void pointAssociateToBody(const pcl::PointXYZ *const p_w, pcl::PointXYZ *const p_b)
{
	Eigen::Vector3d p_cur(p_w->x, p_w->y, p_w->z);
	// transform from world frame to current imu frame
	Eigen::Vector3d p_cur_i = lio_state.rot_end.transpose() * (p_cur - lio_state.pos_end);
	// transform from current imu to current lidar 
	Eigen::Vector3d p_l = lio_state.rot_ex_i2l.transpose() * (p_cur_i - lio_state.pos_ex_i2l); 

	p_b->x = p_l.x();
	p_b->y = p_l.y();
	p_b->z = p_l.z();
}

void radarpointAssociateToBody(const RadarPointCloudType *const p_w, pcl::PointXYZ *const p_b)
{
	Eigen::VectorXd p_cur(4);
	p_cur<<p_w->x, p_w->y, p_w->z, 1;
	// transform from radar frame to current imu frame

	Eigen::VectorXd p_cur_i = T_r2l.inverse()*p_cur;

	p_b->x = p_cur_i(0);
	p_b->y = p_cur_i(1);
	p_b->z = p_cur_i(2);
}

void radarpointAssociateToMap(const RadarPointCloudType *const pi, pcl::PointXYZ *const po)
{
	Eigen::Vector3d p_cur(pi->x, pi->y, pi->z);
	Eigen::MatrixXd R_ri = T_r2i.block<3, 3>(0, 0);
	Eigen::Vector3d p_r_ri = T_r2i.block<3, 1>(0, 3);
	Eigen::Vector3d p_i_ir = R_ri.transpose()*(-p_r_ri);
	// transform from current radar to current imu frame
	Eigen::Vector3d p_cur_i = R_ri.transpose() * p_cur + p_i_ir;
	// transform from current imu to world frame
	Eigen::Vector3d p_w = lio_state.rot_end * p_cur_i + lio_state.pos_end; 

	po->x = p_w.x();
	po->y = p_w.y();
	po->z = p_w.z();
}

void initializeParams() {
    
    if(downsample_method == "VOXELGRID") {
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter.reset(voxelgrid);

    } 
	else if(downsample_method == "APPROX_VOXELGRID") {
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;

    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" << std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    if(outlier_removal_method == "STATISTICAL") {
      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;

    } 
	else if(outlier_removal_method == "RADIUS") {
      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } 
     else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }
}

pcl::PointCloud<PointT>::Ptr downsample(const pcl::PointCloud<PointT>::Ptr& cloud){
    if(!downsample_filter) {
      pcl::PointCloud<PointT>::Ptr cloudout(new pcl::PointCloud<PointT>());
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloudout, indices);
      
      return cloudout;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;
    return filtered;
}

pcl::PointCloud<PointT>::Ptr outlier_removal(const pcl::PointCloud<PointT>::Ptr& cloud){
    if(!outlier_removal_filter) {
      return cloud;
    }
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
}

pcl::PointCloud<PointT>::Ptr distance_filter(const pcl::PointCloud<PointT>::Ptr &cloud)
{
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

    filtered->reserve(cloud->size());
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
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

/* publish gravity marker */
void publish_marker(V3D &vector, V3D &gravity_world, int &id){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "camera_init";
	marker.header.stamp = ros::Time::now();
	marker.ns = "vectors";
	marker.id = id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	// Define the start and end points of the arrow
	geometry_msgs::Point start;
	start.x = lio_state.pos_end.x();
	start.y = lio_state.pos_end.y();
	start.z = lio_state.pos_end.z();
	start.x = 0;
	start.y = 0;
	start.z = 0;

	geometry_msgs::Point end;
	end.x = start.x+vector.x();
	end.y = start.y+vector.y();
	end.z = start.z+vector.z();

	marker.points.push_back(start);
	marker.points.push_back(end);

	// Set the scale of the arrow
	marker.scale.x = 0.1; 
	marker.scale.y = 0.2;
	marker.scale.z = 0.2; 

	// Set the color of the arrow
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	marker_pub.publish(marker);

	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "camera_init";
	marker2.header.stamp = ros::Time::now();
	marker2.ns = "vectors";
	marker2.id = id;
	// id+=1;
	marker2.type = visualization_msgs::Marker::ARROW;
	marker2.action = visualization_msgs::Marker::ADD;

	// Define the start and end points of the arrow
	geometry_msgs::Point start1;
	start1.x = lio_state.pos_end.x();
	start1.y = lio_state.pos_end.y();
	start1.z = lio_state.pos_end.z();
	start1.x = 0;
	start1.y = 0;
	start1.z = 0;

	geometry_msgs::Point end1;
	end1.x = start.x+gravity_world.x();
	end1.y = start.y+gravity_world.y();
	end1.z = start.z+gravity_world.z();

	marker2.points.push_back(start1);
	marker2.points.push_back(end1);

	marker2.scale.x = 0.1; // Shaft diameter
	marker2.scale.y = 0.2; // Head diameter
	marker2.scale.z = 0.2; // Head length

	marker2.color.r = 1.0;
	marker2.color.g = 1.0;
	marker2.color.b = 1.0;
	marker2.color.a = 1.0;
	gravity_pub.publish(marker2);

} 

void laserCloudFeatsHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
	mBuf.lock();
	if (laserCloudMsg->header.stamp.toSec() < last_lidar_time) {
		ROS_ERROR("lidar loop back, clear buffer");
		lidarBuf.clear();
		point_buffer.clear();
	}
	pcl::PointCloud<PointType>::Ptr lidarcloudTemp(new pcl::PointCloud<PointType>());
	pcl::fromROSMsg(*laserCloudMsg, *lidarcloudTemp);
	for(int i = 0; i <lidarcloudTemp->points.size();i++){
		PointBufferType temp;
		temp.x = lidarcloudTemp->points[i].x;
		temp.y = lidarcloudTemp->points[i].y;
		temp.z = lidarcloudTemp->points[i].z;
		temp.intensity = lidarcloudTemp->points[i].intensity;
		temp.start_time = double(laserCloudMsg->header.stamp.toSec());
		temp.timestamp = lidarcloudTemp->points[i].curvature/double(1000) + double(laserCloudMsg->header.stamp.toSec());
		point_buffer.push_back(temp);
	}
	if (last_radar_time == 0) last_radar_time = laserCloudMsg->header.stamp.toSec();


	lidarBuf.push_back(lidarcloudTemp);

	last_lidar_time = laserCloudMsg->header.stamp.toSec();
	mBuf.unlock();
	sigBuf.notify_all();
}

void radarCallback(const sensor_msgs::PointCloud2ConstPtr &radarCloudMsg)
{
	mBuf.lock();
	if (radarCloudMsg->header.stamp.toSec() < last_radar_time) {
		ROS_ERROR("radar loop back, clear buffer");
		radarBuf.clear();
	}
	pcl::PointCloud<RadarPointCloudType>::Ptr radarcloudTemp(new pcl::PointCloud<RadarPointCloudType>());
	pcl::fromROSMsg(*radarCloudMsg, *radarcloudTemp);

	radarBuf.push_back(radarcloudTemp);

	timeBuf.push_back(radarCloudMsg->header.stamp.toSec());
	mBuf.unlock();
	sigBuf.notify_all();
}

void radar_outlier_Callback(const sensor_msgs::PointCloud2ConstPtr &radarCloudMsg)
{
	mBuf.lock();
	if (radarCloudMsg->header.stamp.toSec() < last_radar_time) {
		ROS_ERROR("radar loop back, clear buffer");
		radar_outlier_Buf.clear();
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr radarcloudTemp(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*radarCloudMsg, *radarcloudTemp);

	pcl::PointCloud<pcl::PointXYZ>::Ptr radarcloudTransformed(new pcl::PointCloud<pcl::PointXYZ>());

    // Apply the transformation to the point cloud
	Eigen::Matrix4f T_r2l_float = T_r2l.cast<float>();
    pcl::transformPointCloud(*radarcloudTemp, *radarcloudTransformed, T_r2l_float.inverse());

	radar_outlier_Buf.push_back(radarcloudTransformed);

	mBuf.unlock();
	sigBuf.notify_all();
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg) 
{
	mBuf.lock();
	if (msg->header.stamp.toSec() < last_imu_time) {
		ROS_ERROR("imu loop back, clear buffer");
		imuBuf.clear();
	}
	imuBuf.push_back(msg);
	last_imu_time = msg->header.stamp.toSec();
	mBuf.unlock();
	sigBuf.notify_all();
}

pcl::PointCloud<PointBufferType>::Ptr buildFrame(const std::deque<PointBufferType>& points) {
    pcl::PointCloud<PointBufferType>::Ptr cloud(new pcl::PointCloud<PointBufferType>);
    
    for (const auto& point : points) {
        cloud->points.push_back(point);
    }
    
    cloud->width = points.size();
    cloud->height = 1;  // Unorganized point cloud
    cloud->is_dense = true;
    
    return cloud;
}

bool calc_overlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                              double dis_threshold, int skip_num)
{
	double match_num = 0;
	if (cloud2->points.size()==0) return false;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
		new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kd_tree->setInputCloud(cloud2);
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	cout<<"CLOUD : "<<cloud1->size()<<endl;
	for (size_t i = 0; i < cloud1->size(); i += skip_num)
	{
		pcl::PointXYZ searchPoint = cloud1->points[i];
		if (kd_tree->nearestKSearch(searchPoint, 1, pointIdxNKNSearch,
									pointNKNSquaredDistance) > 0)
		{
			if (pointNKNSquaredDistance[0] < dis_threshold * dis_threshold)
			{
				match_num++;
			}
		}
		
	}

	double overlap =
		(2 * match_num * skip_num) / (cloud1->size() + cloud2->size());
	std::cout<<"Overlap : "<<(match_num / cloud1->size())<<std::endl;
	if ((match_num / cloud1->size()) > 0.7)
	{
		return true;
	}
	else
		return false;
}

// Radar Lidar Scan Synchronization
bool sync_messages(MeasureGroup &meas) {
	
    if (imuBuf.empty() || radarBuf.empty() || timeBuf.empty() || point_buffer.empty()){
		return false;
	}

    if (!(point_buffer.back().timestamp > timeBuf.front()))
    {
        return false;
    }

    if (!(imuBuf.back()->header.stamp.toSec() > timeBuf.front()))
    {
        return false;
    }
    
    while (point_buffer.front().timestamp >= timeBuf.front() || imuBuf.front()->header.stamp.toSec() >= timeBuf.front())
    {
        timeBuf.pop_front();
        radarBuf.pop_front();
		radar_outlier_Buf.pop_front();
        if (imuBuf.empty() || radarBuf.empty() || timeBuf.empty() || point_buffer.empty())
            return false;
    }
	
    meas.imu.clear();
	double imu_time = imuBuf.front()->header.stamp.toSec();
	while (!imuBuf.empty() && imu_time < timeBuf.front()) {
		imu_time = imuBuf.front()->header.stamp.toSec();
		if (imu_time > timeBuf.front()) break;
		meas.imu.push_back(imuBuf.front());
		imuBuf.pop_front();
	}
    meas.point_queue.clear();
	double point_time = point_buffer.front().timestamp;
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	meas.lidar_beg_time = last_radar_time;
	

	while (!point_buffer.empty() && point_time < timeBuf.front()) { 
		point_time = point_buffer.front().timestamp;
		if (point_time > timeBuf.front()) break;
		meas.point_queue.push_back(point_buffer.front());

		pcl::PointXYZINormal pcl_point;
		pcl::PointXYZ point2;
		Eigen::Vector4d point_pose(point_buffer.front().x, point_buffer.front().y, point_buffer.front().z, 1);
		Eigen::Vector4d point_in_radar_Frame = T_r2l*point_pose;
		point2.x = point_in_radar_Frame(0);
        point2.y = point_in_radar_Frame(1);
        point2.z = point_in_radar_Frame(2);
		
		lidar_cloud->points.push_back(point2);

        pcl_point.x = point_buffer.front().x;
        pcl_point.y = point_buffer.front().y;
        pcl_point.z = point_buffer.front().z;
        pcl_point.intensity = point_buffer.front().intensity;
        pcl_point.normal_x = 0.0f; 
        pcl_point.normal_y = 0.0f;
        pcl_point.normal_z = 0.0f;
		pcl_point.curvature = (point_time - last_radar_time)*1000;
		cloud->points.push_back(pcl_point);
		meas.lidar_end_time = point_time;
		lidar_end_time = point_time;
		point_buffer.pop_front();
	}

	last_radar_time = timeBuf.front();
	lidar_cloud->width = static_cast<uint32_t>(meas.point_queue.size());
    lidar_cloud->height = 1;  
    lidar_cloud->is_dense = true;
    cloud->width = static_cast<uint32_t>(meas.point_queue.size());
    cloud->height = 1;  
    cloud->is_dense = true;

	pcl::PointCloud<pcl::PointXYZ>::Ptr radar_xyzI(new pcl::PointCloud<pcl::PointXYZ>);
	for (const auto& point : radarBuf.front()->points)
	{
		pcl::PointXYZ new_point;
		new_point.x = point.x;  
		new_point.y = point.y;  
		new_point.z = point.z;  

		radar_xyzI->points.push_back(new_point);
	}
	radar_xyzI->width = radarBuf.front()->width;
	radar_xyzI->height = radarBuf.front()->height;
	radar_xyzI->is_dense = radarBuf.front()->is_dense;

	meas.lidar = cloud;
	meas.radar = radarBuf.front();
	meas.radar_outlier = radar_outlier_Buf.front();
	
	radarBuf.pop_front();
	radar_outlier_Buf.pop_front();
	timeBuf.pop_front();

	return true;
}

void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v)
{
	V3D un_acc_0 = delta_q * (_acc_0 - linearized_ba);
	V3D un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
	result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
	V3D un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
	V3D un_acc = 0.5 * (un_acc_0 + un_acc_1);
	result_delta_v = delta_v + un_acc * _dt;
}

void calculate_gravity(MeasureGroup &meas, double &sum_dt, V3D &delta_v, Eigen::Quaterniond &delta_q){
    auto v_imu = meas.imu;
    V3D acc_0, acc_1, gyr_0, gyr_1, ba, bg, result_delta_v;
	Eigen::Quaterniond result_delta_q;
    M3D R_imu;

	double dt = 0 ;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);
		dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
		acc_0 = Eigen::Vector3d(head->linear_acceleration.x, head->linear_acceleration.y, head->linear_acceleration.z);
		acc_1 = Eigen::Vector3d(tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z);
		gyr_0 = Eigen::Vector3d(head->angular_velocity.x, head->angular_velocity.y, head->angular_velocity.z);
		gyr_1 = Eigen::Vector3d(tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z);

		midPointIntegration(dt, acc_0, gyr_0, acc_1, gyr_1, 
							delta_q, delta_v, lio_state.bias_a, lio_state.bias_g, result_delta_q, result_delta_v);
		delta_q = result_delta_q;
        delta_v = result_delta_v;
		delta_q.normalize();
		sum_dt += dt;
	}
    
}

void points_cache_collect()
{
	PointVector points_history;
	ikdtreeSurf.acquire_removed_points(points_history);
	for (size_t i = 0; i < points_history.size(); i++) {
		featsArray->push_back(points_history[i]);
	}
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
	kdtree_surf_delete_counter = 0;
    V3D pos_LiD = lid_pos;
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

	if (cub_needrm.size() > 0) {
		kdtree_surf_delete_counter = ikdtreeSurf.Delete_Point_Boxes(cub_needrm);
	}
}

void map_incremental() 
{
	PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointAssociateToMap(&(featsDownBody->points[i]), &(featsDownWorld->points[i]));
        /* decide if need add to map */
        if (!nearestPointsSurf[i].empty() && EKF_init_flag)
        {
            const PointVector &points_near = nearestPointsSurf[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(featsDownWorld->points[i].x / filter_size_map) * filter_size_map + 0.5 * filter_size_map;
            mid_point.y = floor(featsDownWorld->points[i].y / filter_size_map) * filter_size_map + 0.5 * filter_size_map;
            mid_point.z = floor(featsDownWorld->points[i].z / filter_size_map) * filter_size_map + 0.5 * filter_size_map;
            float dist  = calc_dist(featsDownWorld->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map) {
                PointNoNeedDownsample.push_back(featsDownWorld->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < 5; readd_i++)
            {
                if (points_near.size() < 5) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(featsDownWorld->points[i]);
        }
        else
        {
            PointToAdd.push_back(featsDownWorld->points[i]);
        }
    }

    ikdtreeSurf.Add_Points(PointToAdd, true);
    ikdtreeSurf.Add_Points(PointNoNeedDownsample, false); 
}

void processIESKF(bool gravity_update=false, bool vel_update = true) 
{
	/*** ICP and iterated Kalman filter update ***/
	std::vector<double> res_last(feats_down_size, 1000.0); // initial   
	std::vector<bool> point_selected_surf(feats_down_size, true);
	pcl::PointCloud<PointType>::Ptr norm_vec(new pcl::PointCloud<PointType>(*featsDownBody));
	Eigen::MatrixXd R_ri = T_r2i.block<3, 3>(0, 0);
	Eigen::Vector3d p_r_ri = T_r2i.block<3, 1>(0, 3);
	Eigen::Vector3d p_i_ir = R_ri.transpose()*(-p_r_ri);
	std::vector<double> vel_cov;
	V3D vel_r_r2g = R_ri*(propagate_state.rot_end.transpose()*propagate_state.vel_end+skew(pImu_->get_angvel())*p_i_ir);

	Eigen::Matrix3d R_rel = propagate_state.rot_end * last_rot.transpose();

    // Calculate the trace of the relative rotation matrix
    double trace = R_rel.trace();

    // Calculate the rotation angle
    double angle = std::acos((trace - 1.0) / 2.0);
	int rematch_num = 0;
	EKF_converged_flag = true;
	double max_pt_range = 0.0;
	int h_size = 0;
	int h_size_init = 0 ;
	int second_dim = 0 ;
	Eigen::MatrixXd H_x;
	Eigen::VectorXd meas_vec;

	pcl::PointCloud<RadarPointCloudType>::Ptr radarCloudBody(new pcl::PointCloud<RadarPointCloudType>(*(Measures.radar)));

	if(gravity_update) NUM_MAX_ITERATIONS = 1;
	else NUM_MAX_ITERATIONS = 4;
	for (int iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {

		if(!gravity_update){
			laserCloudOri->clear();
			coeffSel->clear();
			/** closest surface search and residual computation **/
			#ifdef MP_EN
				omp_set_num_threads(MP_PROC_NUM);
				#pragma omp parallel for
			#endif

			for (int i = 0; i < feats_down_size; i++) {
				PointType &pointOri_tmpt = featsDownBody->points[i];
				PointType &pointSel_tmpt = featsDownWorld->points[i];
				double ori_pt_dist = sqrt(pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y +
										pointOri_tmpt.z * pointOri_tmpt.z);
				max_pt_range = std::max(max_pt_range, ori_pt_dist);

				/* transform to world frame */
				pointAssociateToMap(&pointOri_tmpt, &pointSel_tmpt);
				std::vector<float> pointSearchSqDis_surf;
				auto &pointsNear = nearestPointsSurf[i];
				
				if (iterCount == 0 || EKF_converged_flag) {
					point_selected_surf[i] = true;
					/** Find the closest surfaces in the map **/
					ikdtreeSurf.Nearest_Search(pointSel_tmpt, NUM_MATCH_POINTS, pointsNear, pointSearchSqDis_surf); 
				
					if (pointsNear.size() < NUM_MATCH_POINTS || pointSearchSqDis_surf[NUM_MATCH_POINTS - 1] > 1.0)
					{
						point_selected_surf[i] = false;
					}
				}

				if (point_selected_surf[i] == false) continue;

				Eigen::Matrix<double, NUM_MATCH_POINTS, 3> matA0;
				Eigen::Matrix<double, NUM_MATCH_POINTS, 1> matB0 = -1 * Eigen::Matrix<double, NUM_MATCH_POINTS, 1>::Ones();
				for (int j = 0; j < NUM_MATCH_POINTS; j++) { 
					matA0(j, 0) = pointsNear[j].x;
					matA0(j, 1) = pointsNear[j].y;
					matA0(j, 2) = pointsNear[j].z;
				}
				// find the norm of plane
				V3D norm = matA0.colPivHouseholderQr().solve(matB0); 
				double negative_OA_dot_norm = 1 / norm.norm();
				norm.normalize();

				bool planeValid = true;
				for (int j = 0; j < NUM_MATCH_POINTS; j++) { 
					// if OX * n > 0.1, then plane is not fit well
					if (fabs(norm(0) * pointsNear[j].x + norm(1) * pointsNear[j].y +
							norm(2) * pointsNear[j].z + negative_OA_dot_norm) > 0.1) {
						planeValid = false;
						point_selected_surf[i] = false;
						break;
					}
				}

				if (planeValid) {
					//loss fuction
					//calculate distance from point to plane
					double pd = norm.x() * pointSel_tmpt.x + norm.y() * pointSel_tmpt.y + norm.z() * pointSel_tmpt.z + negative_OA_dot_norm;
					double s = 1 - 0.9 * fabs(pd) / sqrt(ori_pt_dist);

					if (s > 0.9) { 
						point_selected_surf[i] = true;
						norm_vec->points[i].x = norm.x();
						norm_vec->points[i].y = norm.y();
						norm_vec->points[i].z = norm.z();
						norm_vec->points[i].intensity = pd; 
						res_last[i] = std::abs(pd);
					}
					else {
						point_selected_surf[i] = false;
					}
				}
			}

			effct_feat_num = 0;

			for (int i = 0; i < feats_down_size; i++) {
				if (point_selected_surf[i]) {
					laserCloudOri->push_back(featsDownBody->points[i]);
					coeffSel->push_back(norm_vec->points[i]);
					lidar_weight.push_back(LASER_POINT_COV);
					effct_feat_num ++;
				}
			}
			
			if (effct_feat_num < 50) {
				effct_feat_num = 0;
				// continue;
			} 
			
			/*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
			
			second_dim = radarCloudBody->points.size()/3;			
			if(!vel_update) second_dim = 0;

			/*radar data is poor*/
			if(abs((lio_state.rot_end.transpose()*(lio_state.vel_end)).x() - current_radar_vel.x())>0.5 ||
			   abs((lio_state.rot_end.transpose()*(lio_state.vel_end)).y() - current_radar_vel.y())>0.5 ||
			   abs((lio_state.rot_end.transpose()*(lio_state.vel_end)).z() - current_radar_vel.z())>0.5 )
				second_dim = 0;

			h_size = effct_feat_num + second_dim;


			H_x = Eigen::MatrixXd(h_size, 18);
			meas_vec = Eigen::VectorXd(h_size);
			H_x.setZero();

			//point2plane residual
			for (int i = 0; i < effct_feat_num; i++) {
				const PointType &laser_p  = laserCloudOri->points[i];
				V3D point_this(laser_p.x, laser_p.y, laser_p.z);
				V3D point_imu = lio_state.rot_ex_i2l * point_this + lio_state.pos_ex_i2l;
				M3D point_crossmat = skew(point_imu);
				M3D point_crossmat_2 = skew(lio_state.rot_ex_i2l * point_this);

				/*** get the normal vector of closest surface/corner ***/
				const PointType &norm_p = coeffSel->points[i];
				V3D normvec(norm_p.x, norm_p.y, norm_p.z);

				/*** calculate the Measuremnt Jacobian matrix H ***/
				V3D point_w = lio_state.rot_end * point_imu + lio_state.pos_end; //point in world coordinate
				Eigen::Matrix<double, 1, 3> d_rot(normvec.transpose() * skew(point_w));
				H_x.block<1, 3>(i, 0) = normvec.transpose();
				H_x.block<1, 3>(i, 3) = -d_rot;
				if (extrinsic_est_en) {
					Eigen::Matrix<double, 1, 3> d_rot_ex(normvec.transpose() * lio_state.rot_end * point_crossmat);
					Eigen::Matrix<double, 1, 3> d_rot_ex_2(normvec.transpose() * lio_state.rot_end * point_crossmat_2);
					H_x.block<1, 3>(i, 6) = -d_rot_ex;
					H_x.block<1, 3>(i, 9) = normvec.transpose() * lio_state.rot_end;
				}

				/*** Measuremnt: distance to the closest surface/corner ***/
				meas_vec(i) = -norm_p.intensity;
			}
			
			// pointwise velocity residual
			int idx = 0;
			//sorting residual to avoid outlier 
			Eigen::VectorXd vel_residual(radarCloudBody->points.size());
			std::vector<int> indices(radarCloudBody->points.size());
			vel_cov= std::vector<double>(second_dim, VELOCITY_COV);
			for(int i =0; i<radarCloudBody->points.size(); i++){
				const RadarPointCloudType &laser_p  = radarCloudBody->points[i];
				V3D point_this(laser_p.x, laser_p.y, laser_p.z);
				double norm_p = point_this.norm();

				vel_residual(i) = std::fabs(-((point_this.transpose()*v_r).value() / norm_p + radarCloudBody->points[i].doppler))*exp(10*angle);
				indices[i] = i;
			}

			std::sort(indices.begin(), indices.end(),
				[&vel_residual](size_t i1, size_t i2) { return vel_residual(i1) < vel_residual(i2); });

			//z score calculate
			double median = vel_residual(indices[radarCloudBody->points.size()/2]);
			std::vector<double> absoluteDeviations;
			for (int i =0; i<radarCloudBody->points.size(); i++) {
				absoluteDeviations.push_back(std::fabs(vel_residual(indices[i]) - median));
			}
			std::sort(absoluteDeviations.begin(), absoluteDeviations.end());
			double mad = absoluteDeviations[radarCloudBody->points.size()/2];
			for (int i =0; i<second_dim; i++) {
				if (mad == 0) break;
				double modifiedZ = 0.6745 * (vel_residual(indices[i]) - median) / mad;
				if (abs(modifiedZ)>3.5) vel_cov[i]=vel_cov[i] + VELOCITY_COV/5 * modifiedZ;
			}
			//

			for(int i = 0 ; i <second_dim ; i++){
				idx = indices[i];
				const RadarPointCloudType &laser_p  = radarCloudBody->points[idx];
				V3D point_this(laser_p.x, laser_p.y, laser_p.z);
				double norm_p = point_this.norm();
				/*** calculate the Measuremnt Jacobian matrix H ***/
				H_x.block<1, 3>(i + effct_feat_num, 12) = point_this.transpose()*R_ri*lio_state.rot_end.transpose()/norm_p;
				H_x.block<1, 3>(i + effct_feat_num, 15) = point_this.transpose()*R_ri*skew(p_i_ir)/norm_p;

				meas_vec(i + effct_feat_num) = (-((point_this.transpose()*vel_r_r2g).value() / norm_p + radarCloudBody->points[idx].doppler))/vel_cov[i];
			}
		}
		else{ //Gravity residual
			h_size = 1;

			double sum_dt = 0;
			V3D delta_v = Eigen::Vector3d::Zero();
			Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
			calculate_gravity(Measures, sum_dt, delta_v, delta_q);

			Eigen::Matrix3d rotation_matrix = delta_q.toRotationMatrix(); //IMU delta rotation

			current_radar_vel = lio_state.vel_end;
			gravity_cal = (last_rot*delta_v - current_radar_vel + last_radar_vel)/sum_dt;
			gravity_cal.normalize();
			double gravity_norm = ((last_rot*delta_v - current_radar_vel + last_radar_vel)).norm();

			S2  grav_cal = S2(gravity_cal);
			S2  grav_gt = S2(gravity_world);
			Eigen::Matrix<double, 2, 1> dg;
			grav_gt.minus(dg, grav_cal);
			double gravity_diff = dg.norm();

			grav_residual = 1- gravity_world.dot(gravity_cal);
						
			if(gravity_diff<0.15){
				publish_marker(gravity_cal, gravity_world, id);
				H_x = Eigen::MatrixXd(h_size, 18);
				meas_vec = Eigen::VectorXd(h_size);
				H_x.setZero();
					
				M3D gR = skew(gravity_world)*lio_state.rot_end;
				V3D gravity_crossmat = -1*(gR * (R_ri.transpose()*v_r + skew(p_i_ir)*(current_angular_vel - lio_state.bias_g))).transpose()/gravity_norm;
				// V3D bias_crossmat = -1*gravity_world.transpose()*lio_state.rot_end*skew(p_i_ir)/gravity_norm;

				H_x.block<1, 3>(0, 3) = gravity_crossmat;
				// H_x.block<1, 3>(0, 15) = bias_crossmat;

				meas_vec(0) = -grav_residual;
			}
			else{
				publish_marker(gravity_cal, gravity_world, id);
				return;
			}
		}

		Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, 1> dx;
		
		Eigen::MatrixXd K_gain(StateGroup::DIM_OF_STATE_DOF_, h_size);
		Eigen::MatrixXd K_meas(StateGroup::DIM_OF_STATE_DOF_, 1);
		Eigen::MatrixXd K_x(StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_);
		
		/*** Iterative Kalman Filter Update ***/
		// Prior projection
		dx = lio_state.box_minus(propagate_state); //23*1
		lio_state.P_ = propagate_state.P_;
		M3D jac_rot = Jl(dx.segment<3>(StateGroup::rot_));
		M3D jac_rot_ex = Jl(dx.segment<3>(StateGroup::rot_ext_));
		Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_> J;
		J.setIdentity(); //23 * 23
		J.block<3, 3>(StateGroup::pos_, StateGroup::pos_) = jac_rot;
		J.block<3, 3>(StateGroup::pos_, StateGroup::rot_) = 0.5 * skew(dx.segment<3>(StateGroup::pos_));
		J.block<3, 3>(StateGroup::rot_, StateGroup::rot_) = jac_rot;
		J.block<3, 3>(StateGroup::rot_ext_, StateGroup::rot_ext_) = jac_rot_ex;
		J.block<3, 3>(StateGroup::pos_ext_, StateGroup::rot_ext_) = 0.5 * skew(dx.segment<3>(StateGroup::pos_ext_));
		J.block<3, 3>(StateGroup::pos_ext_, StateGroup::pos_ext_) = jac_rot_ex;
		J.block<3, 3>(StateGroup::vel_, StateGroup::rot_) = 0.5 * skew(dx.segment<3>(StateGroup::vel_));
		J.block<3, 3>(StateGroup::vel_, StateGroup::vel_) = jac_rot;
		Eigen::Matrix<double, 2, 3> Nx;		// for S2 gravity
		Eigen::Matrix<double, 3, 2> Mx;
		lio_state.grav.S2_Nx_yy(Nx);
		propagate_state.grav.S2_Mx(Mx, dx.segment<2>(StateGroup::grav_));
		J.block<2, 2>(StateGroup::grav_, StateGroup::grav_) = Nx * Mx;

		dx = J * dx;
		lio_state.P_ = J * lio_state.P_ * J.transpose();

		if (h_size < StateGroup::DIM_OF_STATE_DOF_) {
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H_cur = 
					Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(h_size, StateGroup::DIM_OF_STATE_DOF_);
			H_cur.topLeftCorner(h_size, 18) = H_x;
			// K = P H^T (H P H^T + R)^-1
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K = lio_state.P_ * H_cur.transpose() * (H_cur * lio_state.P_ * H_cur.transpose() / gravity_COV + 
					Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(h_size, h_size)).inverse() / gravity_COV;
			/* Eigen::MatrixXd S(effct_feat_num, effct_feat_num);
			S.triangularView<Eigen::Upper>() = H_cur * lio_state.P_ * H_cur.transpose() / LASER_POINT_COV;
			S.triangularView<Eigen::Upper>() += Eigen::MatrixXd::Identity(effct_feat_num, effct_feat_num);
			Eigen::MatrixXd S_inv = Eigen::MatrixXd::Identity(effct_feat_num, effct_feat_num);
			S.selfadjointView<Eigen::Upper>().llt().solveInPlace(S_inv);
			Eigen::MatrixXd K = lio_state.P_ * H_cur.transpose() * S_inv.selfadjointView<Eigen::Upper>() / LASER_POINT_COV; */
			K_meas = K * meas_vec;
			K_x = K * H_cur;
		}
		else {
			// K = (H^T R^-1 H + P^-1)^-1 H^T R^-1
			auto &&H_x_T = H_x.transpose();
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H_upper = H_x.block(0, 0, effct_feat_num, 18);
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H_lower = H_x.block(effct_feat_num, 0, h_size-effct_feat_num, 18);

			for (int i= 0 ; i<effct_feat_num;i++){
				H_upper.block(i, 0, 1, 18) = H_upper.block(i, 0, 1, 18)/lidar_weight[i];
			}
			for (int i= 0 ; i<second_dim;i++){  
				H_lower.block(i, 0, 1, 18) = H_lower.block(i, 0, 1, 18)/vel_cov[i]; 
			}
			Eigen::Matrix<double, 18, 18> H_T_H = H_upper.transpose()*H_x.block(0, 0, effct_feat_num, 18)+ H_lower.transpose()*H_x.block(effct_feat_num, 0, h_size-effct_feat_num, 18);

			Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_> P_temp = lio_state.P_.inverse();

			P_temp.block<18, 18>(0, 0) += H_T_H; // H^T R^-1 H + P^-1
			Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_> P_inv = P_temp.inverse(); //(H^T R^-1 H + P^-1)^-1

			Eigen::MatrixXd K_temp(StateGroup::DIM_OF_STATE_DOF_, h_size); //(H^T R^-1 H + P^-1)^-1 H^T
			K_temp = P_inv.block<StateGroup::DIM_OF_STATE_DOF_, 18>(0, 0) * H_x_T;

			// K_gain.leftCols(effct_feat_num) = K_temp.leftCols(effct_feat_num) ;
			K_gain.rightCols(h_size - effct_feat_num) = K_temp.rightCols(h_size - effct_feat_num);

			for (int i= 0 ; i<effct_feat_num;i++){
				K_gain.block(0, i, StateGroup::DIM_OF_STATE_DOF_, 1) = K_temp.block(0, i, StateGroup::DIM_OF_STATE_DOF_, 1)/lidar_weight[i];
			}

			K_meas = K_gain * meas_vec;
			
			K_x.setZero();
			K_x.block<StateGroup::DIM_OF_STATE_DOF_, 18>(0, 0) = K_gain * H_x; // K * H

		}

		Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, 1> dx_new = K_meas + (K_x - 
				Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_>::Identity()) * dx;
		S2 grav_before = lio_state.grav;
		lio_state.box_plus(dx_new);
		EKF_converged_flag = true;
		for (int i = 0; i < StateGroup::DIM_OF_STATE_DOF_; i++) {
			if (std::fabs(dx_new[i]) > 0.001) {
				EKF_converged_flag = false;
				break;
			}
		}

		/*** Rematch Judgement ***/
		if (EKF_converged_flag) {
			rematch_num++;
		}
		if ( (rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2)) ) {
			EKF_converged_flag = true;
		}
		/*** Convergence Judgements and Covariance Update ***/
		if (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1)) {
			if (EKF_init_flag) {
				/*** Covariance Update ***/
				// P = L (I - K H) P L^T
				Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_> L;
				L.setIdentity();
				jac_rot = Jl(dx_new.segment<3>(StateGroup::rot_));
				jac_rot_ex = Jl(dx_new.segment<3>(StateGroup::rot_ext_));
				L.block<3, 3>(StateGroup::pos_, StateGroup::pos_) = jac_rot;
				L.block<3, 3>(StateGroup::pos_, StateGroup::rot_) = 0.5 * skew(dx_new.segment<3>(StateGroup::pos_));
				L.block<3, 3>(StateGroup::rot_, StateGroup::rot_) = jac_rot;
				L.block<3, 3>(StateGroup::rot_ext_, StateGroup::rot_ext_) = jac_rot_ex;
				L.block<3, 3>(StateGroup::pos_ext_, StateGroup::rot_ext_) = 0.5 * skew(dx_new.segment<3>(StateGroup::pos_ext_));
				L.block<3, 3>(StateGroup::pos_ext_, StateGroup::pos_ext_) = jac_rot_ex;
				L.block<3, 3>(StateGroup::vel_, StateGroup::rot_) = 0.5 * skew(dx_new.segment<3>(StateGroup::vel_));
				L.block<3, 3>(StateGroup::vel_, StateGroup::vel_) = jac_rot;
				lio_state.grav.S2_Nx_yy(Nx);
				// grav_before.S2_Mx(Mx, dx_new.segment<2>(StateGroup::grav_));
				propagate_state.grav.S2_Mx(Mx, dx_new.segment<2>(StateGroup::grav_));
				L.block<2, 2>(StateGroup::grav_, StateGroup::grav_) = Nx * Mx;

				K_x.block<StateGroup::DIM_OF_STATE_DOF_, 18>(0, 0) = L * K_x.block<StateGroup::DIM_OF_STATE_DOF_, 18>(0, 0);
				lio_state.P_ = lio_state.P_ * L.transpose();
				L = L * lio_state.P_;
				
				lio_state.P_ = (L - K_x.block<StateGroup::DIM_OF_STATE_DOF_, 18>(0, 0) * lio_state.P_.block<18, StateGroup::DIM_OF_STATE_DOF_>(0, 0)).eval();
				// lio_state.P_.triangularView<Eigen::Upper>() = L - K_x.block<StateGroup::DIM_OF_STATE_DOF_, 12>(0, 0) * lio_state.P_.block<12, StateGroup::DIM_OF_STATE_DOF_>(0, 0);
				// lio_state.P_ = lio_state.P_.selfadjointView<Eigen::Upper>();
			}
			break;
		}
	}
}

float xy2theta( const float & _x, const float & _y )
{
    if ( _x >= 0 & _y >= 0) 
        return (180/M_PI) * atan(_y / _x);

    if ( _x < 0 & _y >= 0) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( _x < 0 & _y < 0) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( _x >= 0 & _y < 0)
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );
}

Eigen::MatrixXd makeScancontext(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    int num_pts_scan_down = cloud->points.size();
    int PC_NUM_RING = 20;
    int PC_NUM_SECTOR = 60;
    double PC_MAX_RADIUS = 80.0;
    double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // main
    const int NO_POINT = -1000;
    Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);
    Eigen::MatrixXd point_num = NO_POINT * Eigen::MatrixXd::Zero(PC_NUM_RING, PC_NUM_SECTOR);

    pcl::PointXYZI pt;
    float azim_angle, azim_range; // within 2D plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++) {
        pt.x = cloud->points[pt_idx].x; 
        pt.y = cloud->points[pt_idx].y;
        pt.z = cloud->points[pt_idx].z;
        // pt.intensity = cloud->points[pt_idx].intensity;
		pt.intensity = 1;

        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if (azim_range > PC_MAX_RADIUS){
			continue;
		}
            

        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

        desc(ring_idx-1, sctor_idx-1) = (desc(ring_idx-1, sctor_idx-1) * point_num(ring_idx-1, sctor_idx-1) + pt.intensity) / (point_num(ring_idx-1, sctor_idx-1) + 1);
        point_num(ring_idx-1, sctor_idx-1) = point_num(ring_idx-1, sctor_idx-1) + 1;
    }

	for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( point_num(row_idx, col_idx) == 0 )
                desc(row_idx, col_idx) = 100;
    
    return desc;
}

void publish_odometry() 
{
	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "camera_init";
	odomAftMapped.child_frame_id = "aft_mapped";
	odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
	odomAftMapped.pose.pose.orientation.x = geoQuat.x();
	odomAftMapped.pose.pose.orientation.y = geoQuat.y();
	odomAftMapped.pose.pose.orientation.z = geoQuat.z();
	odomAftMapped.pose.pose.orientation.w = geoQuat.w();
	odomAftMapped.pose.pose.position.x = lio_state.pos_end.x();
	odomAftMapped.pose.pose.position.y = lio_state.pos_end.y();
	odomAftMapped.pose.pose.position.z = lio_state.pos_end.z();
	pubOdomAftMapped.publish(odomAftMapped);


	std::ofstream lio_path_file(result_path.c_str(), std::ios::app);
	// std::cout<<result_path.c_str()<<std::endl;
	lio_path_file.setf(std::ios::fixed, std::ios::floatfield);
	lio_path_file.precision(10);
	lio_path_file << odomAftMapped.header.stamp.toSec() << " ";
	lio_path_file.precision(10);
	lio_path_file << odomAftMapped.pose.pose.position.x << " "
				  << odomAftMapped.pose.pose.position.y << " "
				  << odomAftMapped.pose.pose.position.z << " "
			      << odomAftMapped.pose.pose.orientation.x << " "
				  << odomAftMapped.pose.pose.orientation.y << " "
				  << odomAftMapped.pose.pose.orientation.z << " "
				  << odomAftMapped.pose.pose.orientation.w << std::endl;

	lio_path_file.close();

	geometry_msgs::PoseStamped laserAfterMappedPose;
	laserAfterMappedPose.header = odomAftMapped.header;
	laserAfterMappedPose.pose = odomAftMapped.pose.pose;
	laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
	laserAfterMappedPath.header.frame_id = "camera_init";
	laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
	pubLaserAfterMappedPath.publish(laserAfterMappedPath); 
	

	static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped" ) );
}

void publish_topics() 
{
	if (pubLaserCloudMap.getNumSubscribers() != 0) {
		sensor_msgs::PointCloud2 laserCloudMsg;
		pcl::toROSMsg(*featsFromMap, laserCloudMsg);
		laserCloudMsg.header.stamp = ros::Time().fromSec(lidar_end_time);
		laserCloudMsg.header.frame_id = "camera_init";
		pubLaserCloudMap.publish(laserCloudMsg);
	}

	if (pubLaserCloudFullRes.getNumSubscribers() != 0) {		
		pcl::PointCloud<PointType>::Ptr laserCloudFullRes(featsUndistort);
		int laserCloudFullResNum = laserCloudFullRes->points.size();
		for (int i = 0; i < laserCloudFullResNum; ++i) {
			pointAssociateToMap(&(laserCloudFullRes->points[i]), &(laserCloudFullRes->points[i]));
		}

		sensor_msgs::PointCloud2 laserCloudMsg;
		pcl::toROSMsg(*laserCloudFullRes, laserCloudMsg);
		laserCloudMsg.header.stamp = ros::Time().fromSec(lidar_end_time);
		laserCloudMsg.header.frame_id = "camera_init";
		pubLaserCloudFullRes.publish(laserCloudMsg);

		if (pcd_save_en) {
			*map_save += *laserCloudFullRes;
		}
	}
}

void publishHeatmap(const Eigen::MatrixXd& matrix, ros::Publisher& pub, int z) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // Adjust frame_id accordingly
    marker.header.stamp = ros::Time::now();
    marker.ns = "heatmap";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;  // Size of each point
    marker.scale.y = 1;

    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            geometry_msgs::Point p;
            p.x = i;
            p.y = j;
            p.z = z;

            std_msgs::ColorRGBA color;
            double intensity = matrix(i, j);
			// cout<<intensity<<endl;
			if (intensity<10 && intensity>-10){
				color.r = 0;
				color.g = 0.0;
				color.b = 0;
				color.a = 1.0;				
			}
			else {
				color.r = 1;
				color.g = 0.0;
				color.b = 0;
				color.a = 1.0;

			}
            

            marker.points.push_back(p);
            marker.colors.push_back(color);
        }
    }

    pub.publish(marker);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidarMapping");
	ros::NodeHandle nh;


	std::vector<double> t_ex(3, 0.0);
	std::vector<double> R_ex(9, 0.0);
	std::vector<double> T_ri_vec(16,0.0);
	std::vector<double> T_rl_vec(16,0.0);

	std::string traj_save_path;

	nh.param<std::string>("common/imu_topic", imu_topic, "/imu/data");
	nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 4);
	nh.param<double>("mapping/cube_side_length", cube_len, 200);
	nh.param<double>("mapping/filter_size_surf", filter_size_surf, 0.5);
    nh.param<double>("mapping/filter_size_map", filter_size_map, 0.5);	
	nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
	nh.param<double>("mapping/acc_noise", acc_noise, 0.1);
	nh.param<double>("mapping/gyr_noise", gyr_noise, 0.1);
	nh.param<double>("mapping/acc_bias_noise", acc_bias_noise, 0.0001);
	nh.param<double>("mapping/gyr_bias_noise", gyr_bias_noise, 0.0001);
	nh.param<vector<double>>("mapping/extrinsic_t_il", t_ex, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R_il", R_ex, vector<double>());
	nh.param<vector<double>>("mapping/extrinsic_T_ri", T_ri_vec, vector<double>());
	nh.param<vector<double>>("mapping/extrinsic_T_rl", T_rl_vec, vector<double>());

	nh.param<std::string>("pcd_save/traj_save_path", traj_save_path, "./output");
	nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
	nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
	nh.param<double>("cov/LASER_POINT_COV", LASER_POINT_COV, 0.001);
	nh.param<double>("cov/GRAVITY_COV", gravity_COV, 0.01);
	nh.param<double>("cov/VELOCITY_COV", VELOCITY_COV, 0.05);

	nh.param<std::string>("threshold/downsample_method", downsample_method, "VOXELGRID");
    nh.param<double>("threshold/downsample_resolution", downsample_resolution, 0.1);
    nh.param<std::string>("threshold/outlier_removal_method", outlier_removal_method, "STATISTICAL");
    nh.param<int>("threshold/statistical_mean_k", mean_k, 20);
    nh.param<double>("threshold/statistical_stddev", stddev_mul_thresh, 1.0);
    nh.param<double>("threshold/radius_radius", radius, 0.8);
    nh.param<int>("threshold/radius_min_neighbors", min_neighbors, 2);
	nh.param<double>("threshold/distance_near_thresh", distance_near_thresh, 1.0);
    nh.param<double>("threshold/distance_far_thresh", distance_far_thresh, 100.0);
    nh.param<double>("threshold/z_low_thresh", z_low_thresh, -5.0);
    nh.param<double>("threshold/z_high_thresh", z_high_thresh, 20.0);

	result_path = traj_save_path + "/garlio_mapped.txt";

	std::ofstream fout(result_path, std::ios::out);
	fout.close();

	downSizeFilterSurf.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);
	initializeParams();

	// Set extrinsic
	t_i2l = vec_from_array(t_ex);
	R_i2l = mat_from_array(R_ex);
	T_r2l <<T_rl_vec[0], T_rl_vec[1], T_rl_vec[2], T_rl_vec[3], T_rl_vec[4], T_rl_vec[5], T_rl_vec[6], T_rl_vec[7],
			T_rl_vec[8], T_rl_vec[9], T_rl_vec[10], T_rl_vec[11], T_rl_vec[12], T_rl_vec[13], T_rl_vec[14], T_rl_vec[15];
	T_r2i <<T_ri_vec[0], T_ri_vec[1], T_ri_vec[2], T_ri_vec[3], T_ri_vec[4], T_ri_vec[5], T_ri_vec[6], T_ri_vec[7],
			T_ri_vec[8], T_ri_vec[9], T_ri_vec[10], T_ri_vec[11], T_ri_vec[12], T_ri_vec[13], T_ri_vec[14], T_ri_vec[15];
	pImu_->set_extrinsic(t_i2l, R_i2l);
	pImu_->set_acc_cov(acc_noise);
	pImu_->set_gyr_cov(gyr_noise);
	pImu_->set_acc_bias_cov(acc_bias_noise);
	pImu_->set_gyr_bias_cov(gyr_bias_noise);


	ros::Subscriber subLaserCloudFeats = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 1000, laserCloudFeatsHandler, ros::TransportHints().tcpNoDelay());
	ros::Subscriber subRadar = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 1000, radarCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10000, imuCallback, ros::TransportHints().tcpNoDelay());
	ros::Subscriber subRadar_outlier = nh.subscribe<sensor_msgs::PointCloud2>("/eagle_data/outlier_pc2", 1000, radar_outlier_Callback, ros::TransportHints().tcpNoDelay());

	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_registered", 100); 
	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 10);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	gravity_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker_gt", 1);


	ROS_INFO("\033[1;32m---->\033[0m Mapping Started.");

	Eigen::MatrixXd R_ri = T_r2i.block<3, 3>(0, 0);
	Eigen::Vector3d p_r_ri = T_r2i.block<3, 1>(0, 3);
	Eigen::Vector3d p_i_ir = R_ri.transpose()*(-p_r_ri);
	ros::Rate loop_rate(5000);
	while (ros::ok()) {
		ros::spinOnce();
		if (sync_messages(Measures)) {
			pcl::toROSMsg(*(Measures.radar), pc2_raw_msg);
			sensor_msgs::PointCloud2 inlier_radar_msg, outlier_radar_msg;
			estimator.estimate(pc2_raw_msg, v_r, sigma_v_r, inlier_radar_msg, outlier_radar_msg);
			
			last_rot = lio_state.rot_end;
			if (is_first_scan) {
				last_radar_vel = R_ri.transpose()*v_r+skew(pImu_->get_angvel())*p_i_ir;;
				lio_state.vel_end = (T_r2i.block<3, 3>(0, 0).transpose() * v_r);
				first_lidar_time = Measures.lidar_beg_time;
				pImu_->first_lidar_time = first_lidar_time; 
				is_first_scan = false;
				continue;
			}

			double t0, t1;
			t0 = omp_get_wtime();
			tools::TicToc t_whole;

			pImu_->Process(Measures, lio_state, featsUndistort); //Propagation and Lidar Point undistortion using FastLIO Forward and Backward
			if(!gravity_init){
				gravity_world = -1*lio_state.grav.vec;
				gravity_world.normalize();
				gravity_init = true;
			}

			current_radar_vel = R_ri.transpose()*v_r+skew(pImu_->get_angvel())*p_i_ir;
			current_angular_vel = pImu_->get_angvel()+lio_state.bias_g;
			
			clock_t start_ms = clock();
			pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>);
			*temp = *featsUndistort;
			for(int i = 0; i < Measures.lidar_temp->size(); i++){
				PointType point;
				pointAssociateToBody(&((Measures.lidar_temp)->points[i]), &(point));
				featsUndistort->points.push_back(point);
			}
			featsUndistort->width = featsUndistort->size();


			/* make full scan end */
			pcl::PointCloud<pcl::PointXYZ>::Ptr radar_xyz(new pcl::PointCloud<pcl::PointXYZ>);
			for (const auto& point : Measures.radar->points)
			{
				pcl::PointXYZ new_point;
				radarpointAssociateToBody(&point, &new_point);

				radar_xyz->points.push_back(new_point);
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr concatenatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
			radar_xyz->width = radar_xyz->points.size();
			radar_xyz->height = Measures.radar->height;
			radar_xyz->is_dense = Measures.radar->is_dense;

			// Create the RadiusOutlierRemoval filter object
			pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
			outrem.setInputCloud(radar_xyz);
			outrem.setRadiusSearch(0.2);  
			outrem.setMinNeighborsInRadius(5);  
			// Apply the filter
			pcl::PointCloud<pcl::PointXYZ>::Ptr radarcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
			outrem.filter(*radarcloud_filtered);

			propagate_state = lio_state;
			lid_pos = propagate_state.rot_end * propagate_state.pos_ex_i2l + propagate_state.pos_end;
			
			if (featsUndistort == NULL){
				ROS_WARN("No point, skip this scan!");
				continue;
			}

			EKF_init_flag = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
			/*** Segment the map in lidar FOV ***/
			lasermap_fov_segment();

			/*** downsample the feature points in a scan ***/
			downSizeFilterSurf.setInputCloud(featsUndistort);
			downSizeFilterSurf.filter(*featsDownBody);

			/*************dynamic remove*************/
			clock_t start, finish;
			double duration;
			pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*featsDownBody, *lidar_cloud);

			pcl::PointCloud<pcl::PointXYZ>::Ptr lidarcloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*featsDownBody, *lidarcloud_2d);

			pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_radarcloud_2d(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::copyPointCloud(*Measures.radar_outlier, *dynamic_radarcloud_2d);

			pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_radarcloud(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::copyPointCloud(*Measures.radar_outlier, *dynamic_radarcloud);
			
			for (size_t i = 0; i < dynamic_radarcloud_2d->points.size(); ++i) {
				dynamic_radarcloud_2d->points[i].z = 0;
			}

			for (size_t i = 0; i < lidarcloud_2d->points.size(); ++i) {
				lidarcloud_2d->points[i].z = 0;
			}

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_dynamic;
			kdtree_dynamic.setInputCloud(lidarcloud_2d);

			int K = 40; // Number of nearest neighbors to find
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			std::vector<int> neighbor_check(lidar_cloud->points.size(), 0);
			std::vector<double> pointNKNMahalnobisDistance(lidar_cloud->points.size(), 100);
			double distance_variance_ = 0.00215;
			double azimuth_variance_ = 0.5 ;
			double elevation_variance_ = 1.0 ;

			int size = 0;
			double radius = 1;
			
			start = clock();
			for (size_t i = 0; i < dynamic_radarcloud_2d->points.size(); ++i) {
				if (kdtree_dynamic.radiusSearch(dynamic_radarcloud_2d->points[i], radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
					int num = 0 ;

					double dist = dynamic_radarcloud->points[i].getVector3fMap().template cast<double>().norm();
					double s_x = dist * distance_variance_ / 400; // 0.00215
					double s_y = dist * sin(azimuth_variance_ / 180 * M_PI); // 0.00873
					double s_z = dist * sin(elevation_variance_ / 180 * M_PI); // 0.01745
					double elevation = atan2(sqrt(dynamic_radarcloud->points[i].x * dynamic_radarcloud->points[i].x + dynamic_radarcloud->points[i].y * dynamic_radarcloud->points[i].y), dynamic_radarcloud->points[i].z);
					double azimuth = atan2(dynamic_radarcloud->points[i].y, dynamic_radarcloud->points[i].x);
					Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(elevation, Eigen::Vector3d::UnitY()));
					Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ()));
					Eigen::Matrix3d R; // Rotation matrix
					R = yawAngle * pitchAngle;
					Eigen::Matrix3d S; // Scaling matix
					S << s_x*10000, 0.0, 0.0,   0.0, s_y*10000, 0.0,   0.0, 0.0, s_z*10000;
					Eigen::Matrix3d A = R * S;
					Eigen::Matrix3d cov_r = A * A.transpose();
					Eigen::Matrix3d cov_inv = cov_r.inverse();

					for (const auto& idx: pointIdxNKNSearch){
						neighbor_check.at(idx)=1;
						Eigen::Vector3f dist_btw_LR_f = dynamic_radarcloud->points[i].getVector3fMap()-lidar_cloud->points[idx].getVector3fMap();
						Eigen::Vector3d dist_btw_LR = dist_btw_LR_f.cast<double>();
						pointNKNMahalnobisDistance.at(idx) = std::min(pointNKNMahalnobisDistance.at(idx),static_cast<double>((dist_btw_LR.transpose() * cov_inv * dist_btw_LR).value()));
					}
				}
			}
			finish = clock();
			double elapsed_secs = double(finish - start) / CLOCKS_PER_SEC;  // Convert clock ticks to seconds

			// Output the elapsed time
			std::cout <<std::fixed<< "Elapsed time: " <<  elapsed_secs << " seconds" << std::endl;

			for(int num : neighbor_check) {
				size += num;
			}

			pcl::PointCloud<PointType>::Ptr lidar_cloud_xyzi(new pcl::PointCloud<PointType>());
			lidar_cloud_xyzi->reserve(lidar_cloud->size());

			// Copy points from pcl::PointXYZ to pcl::PointXYZI
			int num_idx = 0 ;
			for (const auto& point : *featsDownBody) {
				if (pointNKNMahalnobisDistance.at(num_idx)==100){
					lidar_cloud_xyzi->push_back(point);
				}
				num_idx = num_idx+1;
			}
			*featsDownBody = *lidar_cloud_xyzi;

			/*************dynamic remove end*************/

			feats_down_size = featsDownBody->points.size();
			/*** initialize the map kdtree ***/
			if(ikdtreeSurf.Root_Node == nullptr)
			{
				if (feats_down_size > 5) {
					ikdtreeSurf.set_downsample_param(filter_size_map);
					featsDownWorld->resize(feats_down_size);
					for (int i = 0; i < feats_down_size; i++) {
						pointAssociateToMap(&(featsDownBody->points[i]), &(featsDownWorld->points[i]));
					}
					ikdtreeSurf.Build(featsDownWorld->points);
				}
				continue;
			}
			int featsFromMapNum = ikdtreeSurf.validnum();

			featsDownWorld->resize(feats_down_size);

			if (0) {
				if (frameCount % 20 == 0) {
					featsFromMap->clear();
					PointVector().swap(ikdtreeSurf.PCL_Storage);
					ikdtreeSurf.flatten(ikdtreeSurf.Root_Node, ikdtreeSurf.PCL_Storage, NOT_RECORD);
					featsFromMap->points = ikdtreeSurf.PCL_Storage;
				}			
			}

			nearestPointsSurf.resize(feats_down_size);

			/*** two stage iterated state estimation ***/
			Eigen::Vector3d a = propagate_state.vel_end; //prev vel
			processIESKF(); 
			Eigen::Vector3d b = lio_state.vel_end; 
			
			double norm_a = a.norm();
			double norm_b = b.norm();
			
			if(v_r.norm() >3){
				double dot_product = a.dot(b);
				double cosine_theta = dot_product / (norm_a * norm_b);

				// Ensure the value is within the valid range for arccos due to floating-point precision issues
				if (cosine_theta > 1.0) cosine_theta = 1.0;
				else if (cosine_theta < -1.0) cosine_theta = -1.0;
				double angle_radians = std::acos(cosine_theta);

				double angle_degrees = angle_radians * (180.0 / M_PI);
				if (angle_degrees>2){ /*Radar Velocity sometimes fails due to dynamic environment or lidar fail*/
					lio_state = propagate_state;
					LASER_POINT_COV = LASER_POINT_COV / 10;
					processIESKF(false, false);
					Eigen::Vector3d c = lio_state.vel_end;
					cosine_theta = a.dot(c) / (norm_a * c.norm());
					if (cosine_theta > 1.0) cosine_theta = 1.0;
					else if (cosine_theta < -1.0) cosine_theta = -1.0;
					angle_radians = std::acos(cosine_theta);
					angle_degrees = angle_radians * (180.0 / M_PI);
					if(angle_degrees>2) lio_state.vel_end =  propagate_state.rot_end*(R_ri.transpose()*v_r-skew(pImu_->get_angvel())*p_i_ir); //lidar fail due to few features
					LASER_POINT_COV = LASER_POINT_COV * 10;
					ROS_WARN("Angle Refine! \n");
				}
				
			}
			bool gravity_mode = true;
			if (gravity_mode){
				propagate_state = lio_state;
				processIESKF(true, false);
			}

			lio_state.last_update_time = lidar_end_time;
			geoQuat = Eigen::Quaterniond(lio_state.rot_end);
			geoQuat.normalize();

			last_radar_vel = lio_state.vel_end;

			/*** save prev lidar scan into world frame ***/
			Measures.lidar_temp ->clear();
			for(int i = 0 ; i <temp->size();i++){
				PointType point;
				pointAssociateToMap(&(temp->points[i]), &(point));
				Measures.lidar_temp->points.push_back(point);
			}

			/*** save prev radar scan into world frame ***/
			pcl::PointCloud<pcl::PointXYZ>::Ptr radar_save(new pcl::PointCloud<pcl::PointXYZ>);
			for (const auto& point : Measures.radar->points)
			{
				pcl::PointXYZ new_point;
				radarpointAssociateToMap(&point, &new_point);

				radar_save->points.push_back(new_point);
			}
			radar_save->width = Measures.radar->width;
			radar_save->height = Measures.radar->height;
			radar_save->is_dense = Measures.radar->is_dense;

			if(Measures.radar_prev.size()>2) Measures.radar_prev.pop_back();
			Measures.radar_prev.push_front(radar_save);

			/*** add the feature points to map kdtree ***/
			map_incremental();
			t1 = omp_get_wtime();

			frameCount++;
			aver_time_consu = aver_time_consu * (frameCount - 1) / frameCount + (t1 - t0) / frameCount;
			feats_num = feats_num * (frameCount - 1) / frameCount + feats_down_size / frameCount;
			printf("Mapping time %f ms \n", aver_time_consu * 1000);

			publish_odometry();

			publish_topics();

		}

		loop_rate.sleep();
	}

	/*** Save Map ***/
	if (map_save->points.size() > 0 && pcd_save_en) {
		std::string file_name = "scans.pcd";
		std::string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
		pcl::PCDWriter pcd_writer;
		std::cout << "map point save to" << all_points_dir << std::endl;
		pcd_writer.writeBinary(all_points_dir, *map_save);
	}

	return 0;
}
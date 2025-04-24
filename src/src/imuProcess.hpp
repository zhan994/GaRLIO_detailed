#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "common_lib.h"
#include "filter_state.hpp"
#include "tools/color_print.hpp"


/// *************Preconfiguration

inline const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

bool check_state(StateGroup &state_inout) {
    bool is_fail = false;
    for (int idx = 0; idx < 3; ++idx) {
        if (fabs(state_inout.vel_end(idx)) > 10) {
            is_fail = true;
            scope_color(ANSI_COLOR_RED_BG);
            for (int i = 0; i < 10; i++) {
                cout << __FILE__ << ", " << __LINE__ << ", check_state fail !!! " << state_inout.vel_end.transpose() << endl;
            }
            state_inout.vel_end(idx) = 0.0;
        }
    }
    return is_fail;
}

/// *************IMU Process and undistortion
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    typedef Eigen::Matrix<double, StateGroup::DIM_OF_NOISE_, StateGroup::DIM_OF_NOISE_> processNoise;

    ImuProcess();
    ~ImuProcess();
    
    void Reset();
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_gyr_cov(const double &gyr_n);
    void set_acc_cov(const double &acc_n);
    void set_gyr_bias_cov(const double &gyr_w);
    void set_acc_bias_cov(const double &acc_w);
    void Process(const MeasureGroup &meas,  StateGroup &state, pcl::PointCloud<PointType>::Ptr pcl_un_);
    V3D get_angvel();

    V3D acc_cov;
    V3D gyr_cov;
    // V3D acc_cov_init;
    // V3D gyr_cov_init;
    V3D acc_bias_cov;
    V3D gyr_bias_cov;
    processNoise Q;
    double first_lidar_time;

private:
    void IMU_init(const MeasureGroup &meas, StateGroup &state_inout, int &N);
    void undistort_pcl(const MeasureGroup &meas, StateGroup &state_inout, pcl::PointCloud<PointType> &pcl_in_out);
    void predict(StateGroup &state_inout, processNoise &Q, const V3D &gyr, const V3D &acc, double dt);

    pcl::PointCloud<PointType>::Ptr cur_pcl_un_;
    sensor_msgs::ImuConstPtr last_imu_;
    std::deque<sensor_msgs::ImuConstPtr> v_imu_;
    std::vector<Pose6D> IMUpose;
    M3D R_i2l;
    V3D t_i2l;
    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_last;
    double last_lidar_end_time_ = 0.0;
    int    init_iter_num = 1;
    bool   b_first_frame_ = true;
    bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess(): b_first_frame_(true), imu_need_init_(true)
{
    init_iter_num = 1;
    acc_cov       = V3D(0.1, 0.1, 0.1);
    gyr_cov       = V3D(0.01, 0.01, 0.01);
    acc_bias_cov  = V3D(0.0001, 0.0001, 0.0001);
    gyr_bias_cov  = V3D(0.0001, 0.0001, 0.0001);
    mean_acc      = V3D(0, 0, -1.0);
    mean_gyr      = V3D(0, 0, 0);
    Q.setZero();
    angvel_last.setZero();
    t_i2l.setZero();
    R_i2l.setIdentity();
    last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() = default;

void ImuProcess::Reset() 
{
    // ROS_WARN("Reset ImuProcess");
    imu_need_init_ = true;
    b_first_frame_ = true;
    init_iter_num  = 1;
    mean_acc       = V3D(0, 0, -1.0);
    mean_gyr       = V3D(0, 0, 0);
    angvel_last.setZero();
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset(new sensor_msgs::Imu());
    cur_pcl_un_.reset(new pcl::PointCloud<PointType>());
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
    t_i2l = transl;
    R_i2l = rot;
}

void ImuProcess::set_gyr_cov(const double &gyr_n)
{
    gyr_cov = V3D(gyr_n, gyr_n, gyr_n);
}

void ImuProcess::set_acc_cov(const double &acc_n)
{
    acc_cov = V3D(acc_n, acc_n, acc_n);
}

void ImuProcess::set_gyr_bias_cov(const double &gyr_w)
{
    gyr_bias_cov = V3D(gyr_w, gyr_w, gyr_w);
}

void ImuProcess::set_acc_bias_cov(const double &acc_w)
{
    acc_bias_cov = V3D(acc_w, acc_w, acc_w);
}

void ImuProcess::IMU_init(const MeasureGroup &meas, StateGroup &state_inout, int &N)
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/
    
    V3D cur_acc, cur_gyr;
    
    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        first_lidar_time = meas.lidar_beg_time;
    }

    for (const auto &imu : meas.imu) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        //Update of mean based on current frame and mean difference
        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;
        N++;
    }
    //Get the gravity from common_lib.h, and use it with the unit gravity of the acceleration measurement mean to find the gravity acceleration of the rotation matrix type of SO2
    state_inout.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);
    // state_inout.grav = S2(0.0, 0.0, -G_m_s2);
    state_inout.bias_g = mean_gyr;
    state_inout.pos_ex_i2l = t_i2l;
    state_inout.rot_ex_i2l = R_i2l;

    Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_> init_P;
    init_P = Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_>::Identity() * 1e-3;
    init_P.block<3, 3>(StateGroup::rot_ext_, StateGroup::rot_ext_) = M3D::Identity() * 1e-5;    // V3D(1e-5, 1e-5, 1e-5).asDiagonal();
    init_P.block<3, 3>(StateGroup::pos_ext_, StateGroup::pos_ext_) = M3D::Identity() * 1e-5;
    init_P.block<3, 3>(StateGroup::gyr_, StateGroup::gyr_) = M3D::Identity() * 1e-3;
    init_P.block<3, 3>(StateGroup::acc_, StateGroup::acc_) = M3D::Identity() * 1e-2;
    init_P.block<2, 2>(StateGroup::grav_, StateGroup::grav_) = Eigen::Matrix2d::Identity() * 1e-5;
    state_inout.P_ = init_P;

}

void ImuProcess::predict(StateGroup &state_inout, processNoise &Q, const V3D &gyr, const V3D &acc, double dt) 
{
    /* Covariance propagation */
    Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_> F_x;
    Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_NOISE_> F_w;
    F_x.setIdentity();
    F_w.setZero();

    V3D cur_gyr = gyr - state_inout.bias_g;
    V3D cur_acc = acc - state_inout.bias_a;
    V3D cur_acc_global = state_inout.rot_end * cur_acc + state_inout.grav.vec;
    // V3D cur_gyr_dt = cur_gyr * dt;
    M3D tmp = state_inout.rot_end * Exp(cur_gyr, dt) * Jr(cur_gyr * dt) * dt;
    V3D cur_pos = state_inout.pos_end + state_inout.vel_end * dt;
    V3D cur_vel = state_inout.vel_end + cur_acc_global * dt;
    M3D grav_crossmat = skew(state_inout.grav.vec);

    F_x.block<3, 3>(StateGroup::pos_, StateGroup::vel_) = M3D::Identity() * dt;
    F_x.block<3, 3>(StateGroup::pos_, StateGroup::gyr_) = -skew(cur_pos) * tmp;
    F_x.block<3, 3>(StateGroup::rot_, StateGroup::gyr_) = -tmp;
    F_x.block<3, 3>(StateGroup::vel_, StateGroup::rot_) = grav_crossmat * dt;
    F_x.block<3, 3>(StateGroup::vel_, StateGroup::gyr_) = -skew(cur_vel) * tmp;
    F_x.block<3, 3>(StateGroup::vel_, StateGroup::acc_) = -state_inout.rot_end * dt;
    Eigen::Matrix<double, 2, 1> delta = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 3, 2> grav_mat;
    state_inout.grav.S2_Mx(grav_mat, delta);
    F_x.block<3, 2>(StateGroup::vel_, StateGroup::grav_) = grav_mat * dt;

    F_w.block<3, 3>(StateGroup::pos_, 0) = -skew(cur_pos) * tmp;
    F_w.block<3, 3>(StateGroup::rot_, 0) = -tmp;
    F_w.block<3, 3>(StateGroup::vel_, 0) = -skew(cur_vel) * tmp;
    F_w.block<3, 3>(StateGroup::vel_, 3) = -state_inout.rot_end * dt;
    F_w.block<3, 3>(StateGroup::gyr_, 6) = M3D::Identity() * dt;
    F_w.block<3, 3>(StateGroup::acc_, 9) = M3D::Identity() * dt;

    /* State prediction */
    StateGroup state_before = state_inout;
    V3D acc_imu = state_before.rot_end * cur_acc + state_before.grav.vec;
    state_inout.pos_end = state_before.pos_end + state_before.vel_end * dt + 0.5 * acc_imu * dt * dt;
    state_inout.rot_end = state_before.rot_end * Exp(cur_gyr, dt);
    state_inout.vel_end = state_before.vel_end + acc_imu * dt;

    // Update gravity Jacobian in 2-sphere manifold
    Eigen::Matrix<double, 2, 3> Nx;
    Eigen::Matrix<double, 3, 2> Mx;
    state_inout.grav.S2_Nx_yy(Nx);
    state_before.grav.S2_Mx(Mx, delta);
    F_x.block<2, 2>(StateGroup::grav_, StateGroup::grav_) = Nx * M3D::Identity() * Mx;

    state_inout.P_ = F_x * state_inout.P_ * F_x.transpose() + F_w * Q * F_w.transpose();
}

void ImuProcess::undistort_pcl(const MeasureGroup &meas, StateGroup &state_inout, pcl::PointCloud<PointType> &pcl_out)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();
    const double &pcl_beg_time = meas.lidar_beg_time;
    const double &pcl_end_time = meas.lidar_end_time;
    
    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

    /*** Initialize IMU pose ***/
    StateGroup imu_state = state_inout;
    if (0) {
        if (check_state(state_inout)) {
            std::cout << __FILE__ << " " << __LINE__ << std::endl;
            state_inout.display(state_inout, "state_inout");
            imu_state.display(imu_state, "state_in");
        }
    }

    IMUpose.clear();
    IMUpose.push_back(set_pose6d(0.0, imu_state.pos_end, imu_state.vel_end, imu_state.rot_end, acc_last, angvel_last));

    /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;

    double dt = 0;

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);
        
        if (tail->header.stamp.toSec() < last_lidar_end_time_) continue;

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                      0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                      0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr    << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                      0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                      0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
        
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm();

        if(head->header.stamp.toSec() < last_lidar_end_time_) {
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
        }
        else {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }

        /* propagation of IMU state*/
        Q.block<3, 3>(0, 0).diagonal() = gyr_cov;
        Q.block<3, 3>(3, 3).diagonal() = acc_cov;
        Q.block<3, 3>(6, 6).diagonal() = gyr_bias_cov;
        Q.block<3, 3>(9, 9).diagonal() = acc_bias_cov;

        predict(state_inout, Q, angvel_avr, acc_avr, dt);

        /* save the poses at each IMU measurements */
        imu_state = state_inout;
        angvel_last = angvel_avr - imu_state.bias_g;
        acc_last = imu_state.rot_end * (acc_avr - imu_state.bias_a) + imu_state.grav.vec;

        double offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, imu_state.pos_end, imu_state.vel_end, imu_state.rot_end, acc_last, angvel_last));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    if (pcl_end_time > imu_end_time) {
        dt = pcl_end_time - imu_end_time;
    }
    else {
        dt = imu_end_time - pcl_end_time;
    }
    predict(state_inout, Q, angvel_avr, acc_avr, dt);
    
    imu_state = state_inout;
    last_imu_ = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;
    state_inout.last_update_time = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu = mat_from_array(head->rot);
        vel_imu = vec_from_array(head->vel);
        pos_imu = vec_from_array(head->pos);
        acc_imu = vec_from_array(tail->acc);
        angvel_avr = vec_from_array(tail->gyr);
        

        for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
            * Note: Compensation direction is INVERSE of Frame's moving direction
            * So if we want to compensate a point at timestamp-i to the frame-e
            * P_compensate = R_imu_e ^ T * ((R_i * P_i + T_i) - T_e) where T_ei is represented in global frame */
            // V3D angvel_dt = angvel_avr * dt;
            M3D R_i(R_imu * Exp(angvel_avr, dt));
            
            V3D P_l(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D P_i(imu_state.rot_ex_i2l * P_l + imu_state.pos_ex_i2l);
            V3D T_i(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt);
            V3D P_compensate = imu_state.rot_ex_i2l.transpose() * (imu_state.rot_end.transpose() * (R_i * P_i + T_i - imu_state.pos_end) - imu_state.pos_ex_i2l);// not accurate!
            
            // save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin()) break;
        }
    }
}

V3D ImuProcess::get_angvel(){
    return this->angvel_last;
}

void ImuProcess::Process(const MeasureGroup &meas,  StateGroup &state, pcl::PointCloud<PointType>::Ptr cur_pcl_un_)
{
    if (meas.imu.empty()) {
        return;
    }
    
    ROS_ASSERT(meas.point_queue != nullptr);

    if (imu_need_init_) {
        /// The very first lidar frame
        IMU_init(meas, state, init_iter_num);

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();
        
        if (init_iter_num > MAX_INI_CNT) {
            imu_need_init_ = false;

            ROS_INFO("System Initialization Succeeded !!!");
        }
        return;
    }

    undistort_pcl(meas, state, *cur_pcl_un_);

}

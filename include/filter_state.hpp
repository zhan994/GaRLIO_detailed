// This file is part of LINS.

// Modifier: Pengcheng Shi  pengchengshi1995@gmail.com

// Modifier: Chiyun Noh     gch06208@snu.ac.kr

// Copyright (C) 2020 Chao Qin <cscharlesqin@gmail.com>,
// Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
// The Hong Kong University of Science and Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.

#ifndef INCLUDE_FILTER_STATE_HPP_
#define INCLUDE_FILTER_STATE_HPP_

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <fstream>
#include "rio_utils/radar_point_cloud.h"


#include "S2.hpp"

using namespace std;

typedef mf::S2<double, 1, 9810, 1000> S2;

struct Pose6D {
public:
    double offset_time;
    double pos[3];
    double vel[3];
    double rot[9];
    double acc[3];
    double gyr[3];
};

class MeasureGroup {    // Lidar data and imu dates for the curent process
public:
    MeasureGroup() {
        lidar_beg_time = 0.0;
        this->lidar.reset(new pcl::PointCloud<PointType>());
        this->radar.reset(new pcl::PointCloud<RadarPointCloudType>());
        this->lidar_temp.reset(new pcl::PointCloud<PointType>());
    };

    ~MeasureGroup() {};

    double lidar_beg_time; //lidar_begin time
    double lidar_end_time;
    pcl::PointCloud<PointType>::Ptr lidar;
    pcl::PointCloud<PointType>::Ptr lidar_temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr radar_outlier;
    pcl::PointCloud<RadarPointCloudType>::Ptr radar;
    std::deque<sensor_msgs::Imu::ConstPtr> imu;
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> radar_prev;
    std::deque<PointBufferType> point_queue;
};

class StateGroup {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const unsigned int DIM_OF_STATE_ = 24;
    static const unsigned int DIM_OF_STATE_DOF_ = 23;
    static const unsigned int DIM_OF_NOISE_ = 12;       // Dimension of process noise
    static const unsigned int pos_ = 0;
    static const unsigned int rot_ = 3;
    static const unsigned int rot_ext_ = 6;
    static const unsigned int pos_ext_ = 9;
    static const unsigned int vel_ = 12;
    static const unsigned int gyr_ = 15;
    static const unsigned int acc_ = 18;
    static const unsigned int grav_ = 21;

    StateGroup() {
        last_update_time = 0.0;
        pos_end = Eigen::Vector3d::Zero();
        rot_end = Eigen::Matrix3d::Identity();
        rot_ex_i2l = Eigen::Matrix3d::Identity();
        pos_ex_i2l = Eigen::Vector3d::Zero();
        vel_end = Eigen::Vector3d::Zero();
        bias_g = Eigen::Vector3d::Zero();
        bias_a = Eigen::Vector3d::Zero();
        grav = S2(0.0, 0.0, -G_m_s2); 

        P_ = Eigen::Matrix<double, DIM_OF_STATE_DOF_, DIM_OF_STATE_DOF_>::Zero();
        // Q_ = Eigen::Matrix<double, DIM_OF_NOISE_, DIM_OF_NOISE_>::Zero();
    }

    ~StateGroup(){}

    StateGroup& operator=(const StateGroup &x) {
        if (this == &x) return *this;

        this->last_update_time = x.last_update_time;
        this->pos_end = x.pos_end;
        this->rot_end = x.rot_end;
        this->rot_ex_i2l = x.rot_ex_i2l;
        this->pos_ex_i2l = x.pos_ex_i2l;
        this->vel_end = x.vel_end;
        this->bias_g = x.bias_g;
        this->bias_a = x.bias_a;
        this->grav = x.grav;

        this->P_ = x.P_;
        // this->Q_ = x.Q_;
        return *this;
    }

    StateGroup& box_plus(const Eigen::Matrix<double, DIM_OF_STATE_DOF_, 1> &delta_x) {
        M3D dR = Exp(delta_x(rot_), delta_x(rot_ + 1), delta_x(rot_ + 2));
        M3D dR_ex = Exp(delta_x(rot_ext_), delta_x(rot_ext_ + 1), delta_x(rot_ext_ + 2));

        this->pos_end = dR * this->pos_end + delta_x.segment<3>(pos_);
        this->rot_end = dR * this->rot_end;
        this->rot_ex_i2l = dR_ex * this->rot_ex_i2l;
        this->pos_ex_i2l = dR_ex * this->pos_ex_i2l + delta_x.segment<3>(pos_ext_);
        // this->pos_ex_i2l += delta_x.segment<3>(pos_ext_);
        this->vel_end = dR * this->vel_end + delta_x.segment<3>(vel_);
        this->bias_g += delta_x.segment<3>(gyr_);
        this->bias_a += delta_x.segment<3>(acc_);
        this->grav.plus(delta_x.segment<2>(grav_));
        return *this;
    }

    Eigen::Matrix<double, DIM_OF_STATE_DOF_, 1> box_minus(const StateGroup &other) {
        Eigen::Matrix<double, DIM_OF_STATE_DOF_, 1> dx;
        M3D dR(this->rot_end * other.rot_end.transpose());
        M3D dR_ex(this->rot_ex_i2l * other.rot_ex_i2l.transpose());

        dx.block<3, 1>(pos_, 0) = this->pos_end - dR * other.pos_end;
        dx.block<3, 1>(rot_, 0) = Log(dR);
        dx.block<3, 1>(rot_ext_, 0) = Log(dR_ex);
        dx.block<3, 1>(pos_ext_, 0) = this->pos_ex_i2l - dR_ex * other.pos_ex_i2l;
        // dx.block<3, 1>(pos_ext_, 0) = this->pos_ex_i2l - other.pos_ex_i2l;
        dx.block<3, 1>(vel_, 0) = this->vel_end - dR * other.vel_end;
        dx.block<3, 1>(gyr_, 0) = this->bias_g - other.bias_g;
        dx.block<3, 1>(acc_, 0) = this->bias_a - other.bias_a;
        Eigen::Matrix<double, 2, 1> dg;
        this->grav.minus(dg, other.grav);
        dx.block<2, 1>(grav_, 0) = dg;
        return dx;
    }

    static void display(const StateGroup &state, std::string str = std::string("State: "))
    {
        V3D angle_axis = Log(state.rot_end) * 57.296;   // 180 / pi
        printf("%s |", str.c_str());
        printf("[%.5f] | ", state.last_update_time);
        printf("(%.3f, %.3f, %.3f) | ", state.pos_end(0), state.pos_end(1), state.pos_end(2));
        printf("(%.3f, %.3f, %.3f) | ", angle_axis(0), angle_axis(1), angle_axis(2));
        printf("(%.3f, %.3f, %.3f) | ", state.vel_end(0), state.vel_end(1), state.vel_end(2));
        printf("(%.3f, %.3f, %.3f) | ", state.bias_g(0), state.bias_g(1), state.bias_g(2));
        printf("(%.3f, %.3f, %.3f) | ", state.bias_a(0), state.bias_a(1), state.bias_a(2));
        printf("(%.3f, %.3f, %.3f) \r\n", state.grav.vec(0), state.grav.vec(1), state.grav.vec(2));
    }

    double last_update_time;
    V3D pos_end;                // (0-2) Estimated position at the end lidar point in the world frame
    M3D rot_end;                // (3-5) Estimated attitude at the end lidar point in the world frame
    M3D rot_ex_i2l;             // (6-8) Estimated extrinsic between lidar frame to IMU frame on rotation
    V3D pos_ex_i2l;             // (9-11) Estimated extrinsic between lidar frame to IMU frame on position
    V3D vel_end;                // (12-14) Estimated velocity at the end lidar point in the world frame
    V3D bias_g;                 // (15-17) gyroscope bias
    V3D bias_a;                 // (18-20) accelerator bias
    S2  grav;                   // (21-23) Estimated gravity acceleration

    Eigen::Matrix<double, StateGroup::DIM_OF_STATE_DOF_, StateGroup::DIM_OF_STATE_DOF_> P_;
    // Eigen::Matrix<double, StateGroup::DIM_OF_NOISE_, StateGroup::DIM_OF_NOISE_> Q_;
};

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 1> &v,
                const Eigen::Matrix<T, 3, 3> &R, const Eigen::Matrix<T, 3, 1> &a, 
                const Eigen::Matrix<T, 3, 1> &g) {
    Pose6D kp;
    kp.offset_time = t;
    for (int i = 0; i < 3; ++i) {
        kp.pos[i] = p(i);
        kp.vel[i] = v(i);
        kp.acc[i] = a(i);
        kp.gyr[i] = g(i);
        for (int j = 0; j < 3; ++j) {
            kp.rot[i * 3 + j] = R(i, j);
        }
    }

    return std::move(kp);
}

// GlobalState Class contains state variables including position, velocity,
// attitude, acceleration bias, gyroscope bias, and gravity
/* class GlobalState {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr unsigned int DIM_OF_STATE_ = IS_CALIB_EX ? 24 : 18;
    static constexpr unsigned int DIM_OF_NOISE_ = 12;
    static constexpr unsigned int pos_ = 0;
    static constexpr unsigned int vel_ = 3;
    static constexpr unsigned int att_ = 6;
    static constexpr unsigned int acc_ = 9;
    static constexpr unsigned int gyr_ = 12;
    static constexpr unsigned int gra_ = 15;
    static constexpr unsigned int ex_pos_ = 18;
    static constexpr unsigned int ex_att_ = 21;

    GlobalState() { setIdentity(); }

    GlobalState(const V3D& rn, const V3D& vn, const Q4D& qbn, const V3D& ba,
                const V3D& bw) {
        setIdentity();
        rn_ = rn;
        vn_ = vn;
        qbn_ = qbn;
        ba_ = ba;
        bw_ = bw;
    }

    ~GlobalState() {}

    void setIdentity() {
        rn_.setZero();
        vn_.setZero();
        qbn_.setIdentity();
        ba_.setZero();
        bw_.setZero();
        gn_ << 0.0, 0.0, -G_m_s2;
    }

    // boxPlus operator
    void boxPlus(const Eigen::Matrix<double, DIM_OF_STATE_, 1>& xk,
                 GlobalState& stateOut) {
        stateOut.rn_ = rn_ + xk.template segment<3>(pos_);
        stateOut.vn_ = vn_ + xk.template segment<3>(vel_);
        stateOut.ba_ = ba_ + xk.template segment<3>(acc_);
        stateOut.bw_ = bw_ + xk.template segment<3>(gyr_);
        Q4D dq = axis2Quat(xk.template segment<3>(att_));
        stateOut.qbn_ = (qbn_ * dq).normalized();

        stateOut.gn_ = gn_ + xk.template segment<3>(gra_);
    }

    // boxPlus operator for InEKF
    void boxPlusInv(const Eigen::Matrix<double, 18, 1>& xk,
                    GlobalState& stateOut) {
        Q4D dq = axis2Quat(xk.template segment<3>(att_));
        stateOut.qbn_ = (dq * qbn_).normalized();
        stateOut.rn_ = dq * rn_ + xk.template segment<3>(pos_);
        stateOut.vn_ = dq * vn_ + xk.template segment<3>(vel_);
        stateOut.ba_ = ba_ + xk.template segment<3>(acc_);
        stateOut.bw_ = bw_ + xk.template segment<3>(gyr_);
        stateOut.gn_ = gn_ + xk.template segment<3>(gra_);
    }

    // boxMinus operator
    void boxMinus(const GlobalState& stateIn,
                  Eigen::Matrix<double, DIM_OF_STATE_, 1>& xk) {
        xk.template segment<3>(pos_) = rn_ - stateIn.rn_;
        xk.template segment<3>(vel_) = vn_ - stateIn.vn_;
        xk.template segment<3>(acc_) = ba_ - stateIn.ba_;
        xk.template segment<3>(gyr_) = bw_ - stateIn.bw_;
        V3D da = Quat2axis(stateIn.qbn_.inverse() * qbn_);
        xk.template segment<3>(att_) = da;

        xk.template segment<3>(gra_) = gn_ - stateIn.gn_;
    }

    GlobalState& operator=(const GlobalState& other) {
        if (this == &other) return *this;

        this->rn_ = other.rn_;
        this->vn_ = other.vn_;
        this->qbn_ = other.qbn_;
        this->ba_ = other.ba_;
        this->bw_ = other.bw_;
        this->gn_ = other.gn_;
        this->q_ex_ = other.q_ex_;
        this->t_ex_ = other.t_ex_;

        return *this;
    }

    void setExtrinsic(const Q4D &q_ex, const V3D &t_ex) {
        q_ex_ = q_ex;
        t_ex_ = t_ex;
    }

    // !@State
    V3D rn_;   // position in w-frame
    V3D vn_;   // velocity in w-frame
    Q4D qbn_;  // rotation from b-frame to w-frame
    V3D ba_;   // acceleartion bias
    V3D bw_;   // gyroscope bias
    V3D gn_;   // gravity
    Q4D q_ex_;
    V3D t_ex_;
};

class StatePredictor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StatePredictor() = default;

    ~StatePredictor() {}

    bool predict(double dt, const V3D& acc, const V3D& gyr, bool update_jacobian_ = true) {
        if (!isInitialized()) return false;

        if (!flag_init_imu_) {
            flag_init_imu_ = true;
            acc_last = acc;
            gyr_last = gyr;
        }

        // Average acceleration and angular rate
        GlobalState state_tmp = state_;
        V3D un_acc_0 = state_tmp.qbn_ * (acc_last - state_tmp.ba_) + state_tmp.gn_;
        V3D un_gyr = 0.5 * (gyr_last + gyr) - state_tmp.bw_;
        Q4D dq = axis2Quat(un_gyr * dt);
        state_tmp.qbn_ = (dq * state_tmp.qbn_).normalized();
        V3D un_acc_1 = state_tmp.qbn_ * (acc - state_tmp.ba_) + state_tmp.gn_;
        V3D un_acc = 0.5 * (un_acc_0 + un_acc_1);

        // State integral
        state_tmp.rn_ = state_tmp.rn_ + dt * state_tmp.vn_ + 0.5 * dt * dt * un_acc;
        state_tmp.vn_ = state_tmp.vn_ + dt * un_acc;

        if (update_jacobian_) {

            // Calculate F and G of InEKF
            MXD Ft =
                  MXD::Zero(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_);
            MXD Gt =
                  MXD::Zero(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_NOISE_);

            Ft.block<3, 3>(GlobalState::pos_, GlobalState::vel_) = M3D::Identity();
            Ft.block<3, 3>(GlobalState::pos_, GlobalState::gyr_) = -skew(state_tmp.rn_) * state_tmp.qbn_.toRotationMatrix();
            Ft.block<3, 3>(GlobalState::vel_, GlobalState::att_) = skew(state_tmp.gn_);
            Ft.block<3, 3>(GlobalState::vel_, GlobalState::acc_) = -state_tmp.qbn_.toRotationMatrix();
            Ft.block<3, 3>(GlobalState::vel_, GlobalState::gyr_) = -skew(state_tmp.vn_) * state_tmp.qbn_.toRotationMatrix();
            Ft.block<3, 3>(GlobalState::vel_, GlobalState::gra_) = M3D::Identity();
            Ft.block<3, 3>(GlobalState::att_, GlobalState::gyr_) = -state_tmp.qbn_.toRotationMatrix();

            Gt.block<3, 3>(GlobalState::pos_, GlobalState::vel_) = -skew(state_tmp.rn_) * state_tmp.qbn_.toRotationMatrix();
            Gt.block<3, 3>(GlobalState::vel_, GlobalState::pos_) = -state_tmp.qbn_.toRotationMatrix();
            Gt.block<3, 3>(GlobalState::vel_, GlobalState::vel_) = -skew(state_tmp.vn_) * state_tmp.qbn_.toRotationMatrix();
            Gt.block<3, 3>(GlobalState::att_, GlobalState::vel_) = -state_tmp.qbn_.toRotationMatrix();
            Gt.block<3, 3>(GlobalState::acc_, GlobalState::att_) = M3D::Identity();
            Gt.block<3, 3>(GlobalState::gyr_, GlobalState::acc_) = M3D::Identity();
            //Gt = Gt * dt;

            const MXD I =
                MXD::Identity(GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_);
            F_ = I + Ft * dt + 0.5 * Ft * Ft * dt * dt;

            // jacobian_ = F * jacobian_;
            covariance_ =
                F_ * covariance_ * F_.transpose() + dt * Gt * noise_ * Gt.transpose();
            covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval();
        }

        state_ = state_tmp;
        time_ += dt;
        acc_last = acc;
        gyr_last = gyr;
        return true;
    }

    void setState(const GlobalState& state) { state_ = state; }

    void setStateCov(const Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_> &Pk) {
        covariance_.setZero();
        covariance_ = Pk;
    }

    void setNoiseCov(double acc_n, double gyr_n, double acc_w, double gyr_w) {
        double acc_n_2, gyr_n_2, acc_w_2, gyr_w_2;
        acc_n_2 = acc_n * acc_n;
        gyr_n_2 = gyr_n * gyr_n;
        acc_w_2 = acc_w * acc_w;
        gyr_w_2 = gyr_w * gyr_w;
        acc_cov_ = V3D(acc_n_2, acc_n_2, acc_n_2);
        gyr_cov_ = V3D(gyr_n_2, gyr_n_2, gyr_n_2);
        acc_bias_cov_ = V3D(acc_w_2, acc_w_2, acc_w_2);
        gyr_bias_cov_ = V3D(gyr_w_2, gyr_w_2, gyr_w_2);
        noise_.setZero();
        noise_.block<3, 3>(0, 0) = acc_cov_.asDiagonal();
        noise_.block<3, 3>(3, 3) = gyr_cov_.asDiagonal();
        noise_.block<3, 3>(6, 6) = acc_bias_cov_.asDiagonal();
        noise_.block<3, 3>(9, 9) = gyr_bias_cov_.asDiagonal();
    }

    void update(const GlobalState& state,
                const Eigen::Matrix<double, GlobalState::DIM_OF_STATE_,
                GlobalState::DIM_OF_STATE_>& covariance) {
        state_ = state;
        covariance_ = covariance;
    }

    void initialization(double time, const V3D& rn, const V3D& vn, const Q4D& qbn,
                        const V3D& ba, const V3D& bw) {
        state_ = GlobalState(rn, vn, qbn, ba, bw);
        time_ = time;
        flag_init_state_ = true;
    }

    void initialization(double time, const V3D& rn, const V3D& vn, const Q4D& qbn,
                        const V3D& ba, const V3D& bw, const V3D& acc, const V3D& gyr) {
        state_ = GlobalState(rn, vn, qbn, ba, bw);
        time_ = time;
        acc_last = acc;
        gyr_last = gyr;
        flag_init_imu_ = true;
        flag_init_state_ = true;
    }

    inline bool isInitialized() { return flag_init_state_; }

    GlobalState state_;
    double time_;
    Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_>
        F_;
    Eigen::Matrix<double, GlobalState::DIM_OF_STATE_, GlobalState::DIM_OF_STATE_>
        jacobian_, covariance_;
    Eigen::Matrix<double, GlobalState::DIM_OF_NOISE_, GlobalState::DIM_OF_NOISE_>
        noise_;

    V3D acc_last;  // last acceleration measurement
    V3D gyr_last;  // last gyroscope measurement

    V3D acc_cov_;
    V3D gyr_cov_;
    V3D acc_bias_cov_;
    V3D gyr_bias_cov_;

    bool flag_init_state_;
    bool flag_init_imu_;
}; */


#endif  // INCLUDE_FILTER_STATE_HPP_
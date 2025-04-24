// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Modifier: Pengcheng Shi          pengchengshi1995@gmail.com

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

#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <cstdlib>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <deque>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZI PointT;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::VectorXd VXD;
typedef Eigen::MatrixXd MXD;
typedef Eigen::Quaterniond Q4D;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

const double G_m_s2 = 9.81;     // gravity
const int MAX_INI_CNT = 10;
const double INIT_TIME = 0.1;
const int NUM_MATCH_POINTS = 5;

template <typename scalar>
inline scalar tolerance();

template <>
inline float tolerance<float>() { return 1e-5f; }

template <>
inline double tolerance<double>() { return 1e-11; }


inline Eigen::Vector3d vec_from_array(const std::vector<double> &arr) {
    return Eigen::Vector3d{arr[0], arr[1], arr[2]};
}

inline Eigen::Vector3d vec_from_array(const double *arr) {
    return Eigen::Vector3d{arr[0], arr[1], arr[2]};
}

inline Eigen::Matrix3d mat_from_array(const std::vector<double> &arr) {
    return (Eigen::Matrix3d() << arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8]).finished();
}

inline Eigen::Matrix3d mat_from_array(const double *arr) {
    return (Eigen::Matrix3d() << arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8]).finished();
}

inline float calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template <class T>
inline T rad2deg(const T &radians) {
    return radians * 180.0 / M_PI;
}

template <class T>
inline T deg2rad(const T &degrees) {
    return degrees * M_PI / 180.0;
}

template <typename Type>
inline Type wrap_pi(Type x) {
    while (x >= Type(M_PI)) {
      x -= Type(2.0 * M_PI);
    }

    while (x < Type(-M_PI)) {
      x += Type(2.0 * M_PI);
    }
    return x;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1), q(2),
           typename Derived::Scalar(0), -q(0), -q(1), q(0),
           typename Derived::Scalar(0);
    return ans;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d &w_hat) {
    const double EPS = 1e-10;
    assert(fabs(w_hat(2, 1) + w_hat(1, 2)) < EPS);
    assert(fabs(w_hat(0, 2) + w_hat(2, 0)) < EPS);
    assert(fabs(w_hat(1, 0) + w_hat(0, 1)) < EPS);
    return Eigen::Vector3d{w_hat(2, 1), w_hat(0, 2), w_hat(1, 0)};
}

/* template<typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang) {
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> K = skew(ang);
    if (ang_norm >= T(1.745329252e-7)) {
        // Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) / ang_norm * K + (1.0 - std::cos(ang_norm)) / (ang_norm * ang_norm) * K * K;
    }
    else {
        return Eye3 + K + 0.5 * K * K;
    }
} */

/* template<typename T, typename Ts>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    // T r_ang = ang_vel_norm * dt;
    if (ang_vel_norm > T(1.745329252e-7)) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;
        K = skew(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else {
        return Eye3;
    }
} */

/* template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    Eigen::Matrix<T, 3, 1> ang;
    ang << v1, v2, v3;
    Eigen::Matrix<T, 3, 3> K = skew(ang);
    if (norm >= T(1.745329252e-7)) {
        // Roderigous Tranformation
        return Eye3 + std::sin(norm) / norm * K + (1.0 - std::cos(norm)) / (norm * norm) * K * K;
    }
    else {
        return Eye3 + K + 0.5 * K * K;
    }
} */

/* template<typename T>
inline Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R) {
    // -1 <= 0.5 * (R.trace() - 1) <= 1 
    // T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    if (R.trace() > 3.0 || R.trace() < -1.0) {
        return 0.5 * K;
    }
    T theta = std::acos(0.5 * (R.trace() - 1));
    if (std::abs(theta) > 1.745329252e-7) {
        return 0.5 * theta / std::sin(theta) * K;
    }
    else {
        return 0.5 * K;
    }
} */

template<typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang) {
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 1e-7) {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K = skew(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else {
        return Eye3;
    }
}

template<typename T, typename Ts>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 1e-7) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K = skew(r_axis);
        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else {
        return Eye3;
    }
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3) {
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 1e-7) {
        Eigen::Matrix<T, 3, 1> r_ang;
        r_ang << v1 / norm, v2 / norm, v3 / norm;
        Eigen::Matrix<T, 3, 3> K = skew(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template<typename T>
inline Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R) {
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

inline void enforceSymmetry(Eigen::MatrixXd &mat) {
    mat = 0.5 * (mat + mat.transpose()).eval();
}

inline Eigen::Quaterniond axis2Quat(const Eigen::Vector3d &axis, double theta) {
    Eigen::Quaterniond q;

    if (theta < 1e-10) {
      q.w() = 1.0;
      q.x() = q.y() = q.z() = 0;
    }

    double magnitude = sin(theta / 2.0f);

    q.w() = cos(theta / 2.0f);
    q.x() = axis(0) * magnitude;
    q.y() = axis(1) * magnitude;
    q.z() = axis(2) * magnitude;

    return q;
}

inline Eigen::Quaterniond axis2Quat(const Eigen::Vector3d &vec) {
    Eigen::Quaterniond q;
    double theta = vec.norm();

    if (theta < 1e-10) {
      q.w() = 1.0;
      q.x() = q.y() = q.z() = 0;
      return q;
    }

    Eigen::Vector3d tmp = vec / theta;
    return axis2Quat(tmp, theta);
}

inline Eigen::Vector3d Quat2axis(const Eigen::Quaterniond &q) {
    double axis_magnitude = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
    Eigen::Vector3d vec;
    vec(0) = q.x();
    vec(1) = q.y();
    vec(2) = q.z();

    if (axis_magnitude >= 1e-10) {
      vec = vec / axis_magnitude;
      vec = vec * wrap_pi(2.0 * atan2(axis_magnitude, q.w()));
    }

    return vec;
}

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q) {
    // printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
    // printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
    //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta) {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q) {
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(
        1, 0) = qq.vec(),
          ans.template block<3, 3>(1, 1) =
              qq.w() *
                  Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
              skew(qq.vec());
    return ans;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p) {
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(
        1, 0) = pp.vec(),
          ans.template block<3, 3>(1, 1) =
              pp.w() *
                  Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
              skew(pp.vec());
    return ans;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Jl(const Eigen::MatrixBase<Derived> &axis) {
    typedef typename Derived::Scalar Scalar_t;
 
    Eigen::Matrix<Scalar_t, 3, 3> jl;
    Scalar_t theta = axis.norm();
    Eigen::Matrix<Scalar_t, 3, 3> axis_crossmat = skew(axis);
    if (theta < static_cast<Scalar_t>(1.745e-11)) {    // 100*10^-8 degree
        jl = Eigen::Matrix<Scalar_t, 3, 3>::Identity() +
             static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(2.0) * axis_crossmat + 
             static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(6.0) * axis_crossmat * axis_crossmat;
    }
    else {
        jl = Eigen::Matrix<Scalar_t, 3, 3>::Identity() +
             (static_cast<Scalar_t>(1.0) - cos(theta)) / (theta * theta) * axis_crossmat + 
             (theta - sin(theta)) / (theta * theta * theta) * axis_crossmat * axis_crossmat;
    }
    return jl;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Jl_inv(const Eigen::MatrixBase<Derived> &axis) {
    typedef typename Derived::Scalar Scalar_t;
    
    Eigen::Matrix<Scalar_t, 3, 3> jl_inv;
    Scalar_t theta = axis.norm();
    Eigen::Matrix<Scalar_t, 3, 3> axis_crossmat = skew(axis);
    if (theta < static_cast<Scalar_t>(1.745e-11)) {
        jl_inv = Eigen::Matrix<Scalar_t, 3, 3>::Identity() -
                 static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(2.0) * axis_crossmat + 
                 static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(4.0) * axis_crossmat * axis_crossmat;
    }
    else {
        jl_inv = Eigen::Matrix<Scalar_t, 3, 3>::Identity() -
                 static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(2.0) * axis_crossmat + 
                 (static_cast<Scalar_t>(1.0) / (theta * theta) - 
                 (static_cast<Scalar_t>(1.0) + cos(theta)) / (static_cast<Scalar_t>(2.0) * theta * sin(theta))) *
                 axis_crossmat * axis_crossmat;
    }
    return jl_inv;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Jr(const Eigen::MatrixBase<Derived> &axis) {
    typedef typename Derived::Scalar Scalar_t;
 
    Eigen::Matrix<Scalar_t, 3, 3> jr;
    Scalar_t theta = axis.norm();
    Eigen::Matrix<Scalar_t, 3, 3> axis_crossmat = skew(axis);
    if (theta < static_cast<Scalar_t>(1.745e-11)) {    // 100*10^-8 degree
        jr = Eigen::Matrix<Scalar_t, 3, 3>::Identity() -
             static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(2.0) * axis_crossmat + 
             static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(6.0) * axis_crossmat * axis_crossmat;
    }
    else {
        jr = Eigen::Matrix<Scalar_t, 3, 3>::Identity() -
             (static_cast<Scalar_t>(1.0) - cos(theta)) / (theta * theta) * axis_crossmat + 
             (theta - sin(theta)) / (theta * theta * theta) * axis_crossmat * axis_crossmat;
    }
    return jr;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Jr_inv(const Eigen::MatrixBase<Derived> &axis) {
    typedef typename Derived::Scalar Scalar_t;
    
    Eigen::Matrix<Scalar_t, 3, 3> jr_inv;
    Scalar_t theta = axis.norm();
    Eigen::Matrix<Scalar_t, 3, 3> axis_crossmat = skew(axis);
    if (theta < static_cast<Scalar_t>(1.745e-11)) {
        jr_inv = Eigen::Matrix<Scalar_t, 3, 3>::Identity() +
                 static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(2.0) * axis_crossmat + 
                 static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(4.0) * axis_crossmat * axis_crossmat;
    }
    else {
        jr_inv = Eigen::Matrix<Scalar_t, 3, 3>::Identity() +
                 static_cast<Scalar_t>(1.0) / static_cast<Scalar_t>(2.0) * axis_crossmat + 
                 (static_cast<Scalar_t>(1.0) / (theta * theta) - 
                 (static_cast<Scalar_t>(1.0) + cos(theta)) / (static_cast<Scalar_t>(2.0) * theta * sin(theta))) *
                 axis_crossmat * axis_crossmat;
    }
    return jr_inv;
}
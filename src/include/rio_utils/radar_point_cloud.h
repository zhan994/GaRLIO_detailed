// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_macros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace rio
{

// bool pcl2msgToPcl(const sensor_msgs::PointCloud2& pcl_msg, pcl::PointCloud<RadarPointCloudType>& scan);

}  // namespace rio



struct PointBufferType
{
  PCL_ADD_POINT4D      // x,y,z position in [m]
  PCL_ADD_INTENSITY;
  union
    {
      struct
      {
        double start_time;
        double timestamp;
      };
      float data_c[4];
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
} EIGEN_ALIGN16; 
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    PointBufferType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, start_time,start_time)
    (double, timestamp,timestamp)
)

struct RadarPointCloudType
{
  PCL_ADD_POINT4D      // x,y,z position in [m]
  PCL_ADD_INTENSITY;
  union
    {
      struct
      {
        float doppler;
      };
      float data_c[4];
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
} EIGEN_ALIGN16; 
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    RadarPointCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, doppler, doppler)
)

struct EaglePointXYZIVRAB
{
    PCL_ADD_POINT4D 
    float power;
    float doppler;
    float range;
    float alpha;
    float beta;
    uint16_t denoiseFlag;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    EaglePointXYZIVRAB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, power, power)
    (float, doppler, doppler)
    (float, range, range)
    (float, alpha, alpha)
    (float, beta, beta)
    (uint16_t, denoiseFlag, denoiseFlag)
)

struct EaglePointXYZIVRAB2
{
    PCL_ADD_POINT4D;  // This adds the float x, y, z members
    float Doppler;
    float Range;
    float Power;
    float Alpha;
    float Beta;
    float rangeAccu;
    float aziAccu;
    float eleAccu;
    float dopplerAccu;
    float recoveredSpeed;
    float dotFlags;
    float denoiseFlag;
    float historyFrameFlag;
    float dopplerCorrectionFlag;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT
(
    EaglePointXYZIVRAB2,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, Doppler, Doppler)
    (float, Range, Range)
    (float, Power, Power)
    (float, Alpha, Alpha)
    (float, Beta, Beta)
    (float, rangeAccu, rangeAccu)
    (float, aziAccu, aziAccu)
    (float, eleAccu, eleAccu)
    (float, dopplerAccu, dopplerAccu)
    (float, recoveredSpeed, recoveredSpeed)
    (float, dotFlags, dotFlags)
    (float, denoiseFlag, denoiseFlag)
    (float, historyFrameFlag, historyFrameFlag)
    (float, dopplerCorrectionFlag, dopplerCorrectionFlag)
)
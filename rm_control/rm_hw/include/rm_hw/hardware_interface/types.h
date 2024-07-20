/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 1/20/21.
//

#pragma once

#include <string>
#include <rm_common/filters/lp_filter.h>
#include <rm_common/filters/imu_filter_base.h>
#include <unordered_map>

#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define SPEED_SMOOTH_COEF 0.85f
#define DEGREE_2_RAD 0.01745329252f // pi/180
#define CURRENT_SMOOTH_COEF 0.9f

namespace rm_hw
{
struct ActCoeff
{
  double act2pos, act2vel, act2effort, pos2act, vel2act, effort2act, max_out, act2pos_offset, act2vel_offset,
      act2effort_offset, kp2act, kd2act;  // for MIT Cheetah motor
};

struct ActData
{
  std::string name;
  std::string type;
  ros::Time stamp;
  uint16_t state; //暂时达妙
  uint64_t seq;
  bool halted = false, need_calibration = false, calibrated = false, calibration_reading = false;
  uint16_t q_raw;// 原始位置(角度)
  float angle_single_round; // 单圈角度
  int16_t qd_raw;//速度
  int16_t t_int;
  uint8_t temp;//温度

  int64_t q_circle;//圈数
  uint16_t q_last;//上一次位置(角度)数据
  float total_angle;//总角度
  double frequency;//频率数据（执行器操作频率）
  double pos, vel, effort;//存储执行器的 位置 速度 力
  double cmd_pos, cmd_vel, cmd_effort, exe_effort;//存储期望的执行器 位置 速度 命令力  实际力
  double offset;//偏移 处理执行器读数
  // For multiple cycle under absolute encoder (RoboMaster motor)
  LowPassFilter* lp_filter;//低通滤波
};

struct ImuData
{
  ros::Time time_stamp;
  std::string imu_name;
  double ori[4];
  double angular_vel[3], linear_acc[3];
  double angular_vel_offset[3];
  double ori_cov[9], angular_vel_cov[9], linear_acc_cov[9];
  double temperature, angular_vel_coeff, accel_coeff, temp_coeff, temp_offset;
  bool accel_updated, gyro_updated, camera_trigger;
  bool enabled_trigger;
  rm_common::ImuFilterBase* imu_filter;
};

struct TofData
{
  double strength;
  double distance;
};

struct CanDataPtr
{
  std::unordered_map<std::string, ActCoeff>* type2act_coeffs_;
  std::unordered_map<int, ActData>* id2act_data_;
  std::unordered_map<int, ImuData>* id2imu_data_;
  std::unordered_map<int, TofData>* id2tof_data_;
};
}  // namespace rm_hw

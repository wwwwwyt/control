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
// Created by qiayuan on 12/28/20.
//
#include "rm_hw/hardware_interface/can_bus.h"

#include <string>
#include <ros/ros.h>
#include <rm_common/math_utilities.h>
namespace rm_hw
{
CanBus::CanBus(const std::string& bus_name, CanDataPtr data_ptr, int thread_priority)
  : bus_name_(bus_name), data_ptr_(data_ptr)
{
  // Initialize device at can_device, false for no loop back.
if(bus_name =="can1")
{
  socket_tcp_can1.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) && ros::ok();
    ros::Duration(.5).sleep();   
//   ROS_INFO("can1 connected to %s.", bus_name.c_str());
}

if(bus_name == "can0")
{     
  socket_tcp_can0.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) && ros::ok();
    ros::Duration(.5).sleep();
  // ROS_INFO("can2 connected to %s.", bus_name.c_str());  
}
  // ROS_INFO("Successfully connected to %s.", bus_name.c_str());
  // Set up CAN package header
  rm_frame0_.can_id = 0x200;
  rm_frame0_.can_dlc = 8;
  rm_frame1_.can_id = 0x1FF;
  rm_frame1_.can_dlc = 8;
}

void CanBus::write()
{
  // safety first
  bool has_write_frame0 = false, has_write_frame1 = false;
  std::fill(std::begin(rm_frame0_.data), std::end(rm_frame0_.data), 0);
  std::fill(std::begin(rm_frame1_.data), std::end(rm_frame1_.data), 0);

  for (auto& item : *data_ptr_.id2act_data_)
  {
    if (item.second.type.find("rm") != std::string::npos)
    {
      if (item.second.halted)
        continue;
      const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
      int id = item.first - 0x201;
      double cmd =
          minAbs(act_coeff.effort2act * item.second.exe_effort, act_coeff.max_out);  // add max_range to act_data
      if (-1 < id && id < 4)
      {
        rm_frame0_.data[2 * id] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
        rm_frame0_.data[2 * id + 1] = static_cast<uint8_t>(cmd);
        socket_tcp_can0.write(&rm_frame0_);
      }
      else if (3 < id && id < 8)
      {
        rm_frame1_.data[2 * (id - 4)] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
        rm_frame1_.data[2 * (id - 4) + 1] = static_cast<uint8_t>(cmd);
        socket_tcp_can1.write(&rm_frame1_);
      }
    }
    else if (item.second.type.find("dm") != std::string::npos)
    {
      // const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
      rm_frame0_.can_id = item.first + 0x04;

      uint16_t pos_tmp,vel_tmp,kp_tmp{0},kd_tmp{0},tor_tmp;     
      pos_tmp = CanBus::float_to_uint(item.second.cmd_pos,  P_MIN_8009,  P_MAX_8009,  16);
      vel_tmp = CanBus::float_to_uint(item.second.cmd_vel,  V_MIN_8009,  V_MAX_8009,  12);
      // kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12); 
      // kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
      tor_tmp = CanBus::float_to_uint(item.second.cmd_effort, T_MIN_8009,  T_MAX_8009,  12);

      // TODO(wyt) add position vel and effort hardware interface for DM Motor, now we using it as an effort joint.
      if(item.second.cmd_effort <= 20 && item.second.cmd_effort >= -20)
      {
        rm_frame0_.data[0] = pos_tmp >> 8;
        rm_frame0_.data[1] = pos_tmp;
        rm_frame0_.data[2] = vel_tmp >> 4;
        rm_frame0_.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        rm_frame0_.data[4] = kp_tmp ;
        rm_frame0_.data[5] = kd_tmp >> 4;
        rm_frame0_.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        rm_frame0_.data[7] = tor_tmp ;
      }
      else if(item.second.cmd_effort == 21 )
      {
        rm_frame0_.data[0] = 0xFF;//使能帧
        rm_frame0_.data[1] = 0xFF;
        rm_frame0_.data[2] = 0xFF;
        rm_frame0_.data[3] = 0xFF;
        rm_frame0_.data[4] = 0xFF;
        rm_frame0_.data[5] = 0xFF;
        rm_frame0_.data[6] = 0xFF;
        rm_frame0_.data[7] = 0xFC;      
      }
      else
      {
        rm_frame0_.data[0] = 0xFF;//ui能帧
        rm_frame0_.data[1] = 0xFF;
        rm_frame0_.data[2] = 0xFF;
        rm_frame0_.data[3] = 0xFF;
        rm_frame0_.data[4] = 0xFF;
        rm_frame0_.data[5] = 0xFF;
        rm_frame0_.data[6] = 0xFF;
        rm_frame0_.data[7] = 0xFD;            
      }
        // rm_frame0_.data[0] = 0xFF;//使能帧
        // rm_frame0_.data[1] = 0xFF;
        // rm_frame0_.data[2] = 0xFF;
        // rm_frame0_.data[3] = 0xFF;
        // rm_frame0_.data[4] = 0xFF;
        // rm_frame0_.data[5] = 0xFF;
        // rm_frame0_.data[6] = 0xFF;
        // rm_frame0_.data[7] = 0xFC;           
      // rm_frame0_.data[0] = 0xFF;//保存零点 （ 在机械拆装腿时 摆到上限位处理零点 vmc拿机械数据－电机角度
      // rm_frame0_.data[1] = 0xFF;
      // rm_frame0_.data[2] = 0xFF;
      // rm_frame0_.data[3] = 0xFF;
      // rm_frame0_.data[4] = 0xFF;
      // rm_frame0_.data[5] = 0xFF;
      // rm_frame0_.data[6] = 0xFF;
      // rm_frame0_.data[7] = 0xFE;    

      // rm_frame0_.data[0] = 0xFF;//shi neng
      // socket_tcp_can0.write(&rm_frame0_);
      has_write_frame0 = true;
      // std::cout<<"write success !!!"<<std::endl;
    }    
    else if (item.second.type.find("lk") != std::string::npos)
    {
      // const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;

      rm_frame1_.can_id = 0x280;
      int id = item.first - 0x140;
      // uint16_t tau = static_cast<int>(act_coeff.effort2act * (item.second.exe_effort - act_coeff.act2effort_offset)); //期望力矩
      uint16_t tau = CanBus::float_to_uint(item.second.cmd_effort, T_MIN_8009,  T_MAX_8009,  16);
       tau = 0;

      // TODO(wyt) add position vel and effort hardware interface for MIT Cheetah Motor, now we using it as an effort joint.

      // memcpy(rm_frame1_.data + (item.first- 0x280) * 2, &tau, sizeof(uint16_t));
      if(id == 1)
       {
        rm_frame1_.data[0] = *(uint8_t*)(&tau);
        rm_frame1_.data[1] = *((uint8_t*)(&tau)+1);
       }
       if(id == 2)
       {
        rm_frame1_.data[2] = *(uint8_t*)(&tau);
        rm_frame1_.data[3] = *((uint8_t*)(&tau)+1);
       }

      ROS_INFO("RESULT---  %d ", id);
      has_write_frame1 = true;
      // socket_tcp_can1.write(&rm_frame1_);
      // std::cout<<"write success !!!"<<std::endl;
      // socket_can_.write(&frame);
    }    
    else if (item.second.type.find("bulute") != std::string::npos)
    {
      can_frame frame{};
      const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
      frame.can_id = item.first;
      frame.can_dlc = 8;
      uint16_t q_des = static_cast<int>(act_coeff.pos2act * (item.second.cmd_pos - act_coeff.act2pos_offset));
      uint16_t qd_des = static_cast<int>(act_coeff.vel2act * (item.second.cmd_vel - act_coeff.act2vel_offset));
      uint16_t kp = 0.;
      uint16_t kd = 0.;
      uint16_t tau = static_cast<int>(act_coeff.effort2act * (item.second.exe_effort - act_coeff.act2effort_offset));
      // TODO(qiayuan) add position vel and effort hardware interface for MIT Cheetah Motor, now we using it as an effort joint.
      frame.data[0] = q_des >> 8;
      frame.data[1] = q_des & 0xFF;
      frame.data[2] = qd_des >> 4;
      frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
      frame.data[4] = kp & 0xFF;
      frame.data[5] = kd >> 4;
      frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
      frame.data[7] = tau & 0xff;
      socket_can_.write(&frame);
    }
    if (has_write_frame0)
    {
      // socket_can_.write(&rm_frame0_);
      socket_tcp_can0.write(&rm_frame0_);
    }
    if (has_write_frame1)
    {
      // socket_can_.write(&rm_frame1_);
      // ROS_INFO("can1 write  waiting..");
      socket_tcp_can1.write(&rm_frame1_);
      ROS_INFO("can1 write success..");
    }
  }

  // if (has_write_frame0)
  //   {
  //     // socket_can_.write(&rm_frame0_);
  //     socket_tcp_can0.write(&rm_frame0_);
  //   }
  // if (has_write_frame1)
  //   {
  //     // socket_can_.write(&rm_frame1_);
  //     socket_tcp_can1.write(&rm_frame1_);
  //   }
}

void CanBus::read(ros::Time time)
{
  std::lock_guard<std::mutex> guard(mutex_);

  for (auto& imu : *data_ptr_.id2imu_data_)
  {
    imu.second.gyro_updated = false;
    imu.second.accel_updated = false;
  }

  for (const auto& frame_stamp : read_buffer_)
  {
    can_frame frame = frame_stamp.frame;
    // // Check if robomaster motor
    // if (data_ptr_.id2act_data_->find(frame.can_id) != data_ptr_.id2act_data_->end())
    // {
    //   ActData& act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
    //   if ((frame_stamp.stamp - act_data.stamp).toSec() < 0.0005)
    //     continue;
    //   const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
    //   if (act_data.type.find("rm") != std::string::npos)
    //   {
    //     act_data.q_raw = (frame.data[0] << 8u) | frame.data[1];
    //     act_data.qd_raw = (frame.data[2] << 8u) | frame.data[3];
    //     int16_t cur = (frame.data[4] << 8u) | frame.data[5];
    //     act_data.temp = frame.data[6];

    //     // Multiple circle
    //     if (act_data.seq != 0)  // not the first receive
    //     {
    //       if (act_data.q_raw - act_data.q_last > 4096)
    //         act_data.q_circle--;
    //       else if (act_data.q_raw - act_data.q_last < -4096)
    //         act_data.q_circle++;
    //     }
    //     try
    //     {  // Duration will be out of dual 32-bit range while motor failure
    //       act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
    //     }
    //     catch (std::runtime_error& ex)
    //     {
    //     }
    //     act_data.stamp = frame_stamp.stamp;
    //     act_data.seq++;
    //     act_data.q_last = act_data.q_raw;
    //     // Converter raw CAN data to position velocity and effort.
    //     act_data.pos =
    //         act_coeff.act2pos * static_cast<double>(act_data.q_raw + 8191 * act_data.q_circle) + act_data.offset;
    //     act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
    //     act_data.effort = act_coeff.act2effort * static_cast<double>(cur);
    //     // Low pass filter
    //     act_data.lp_filter->input(act_data.vel, frame_stamp.stamp);
    //     // act_data.vel = act_data.lp_filter->output();
    //     continue;
    //   }
    // }
    // // Check MIT Cheetah motor
    // else if (frame.can_id == static_cast<unsigned int>(0x000))
    // {
    //   if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
    //   {
    //     ActData& act_data = data_ptr_.id2act_data_->find(frame.data[0])->second;
    //     const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
    //     if (act_data.type.find("cheetah") != std::string::npos)
    //     {  // MIT Cheetah Motor
    //       act_data.q_raw = (frame.data[1] << 8) | frame.data[2];
    //       uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
    //       uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];
    //       // Multiple cycle
    //       // NOTE: The raw data range is -4pi~4pi
    //       if (act_data.seq != 0)  // not the first receive
    //       {
    //         double pos_new = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
    //                          static_cast<double>(act_data.q_circle) * 8 * M_PI + act_data.offset;
    //         if (pos_new - act_data.pos > 4 * M_PI)
    //           act_data.q_circle--;
    //         else if (pos_new - act_data.pos < -4 * M_PI)
    //           act_data.q_circle++;
    //       }
    //       try
    //       {  // Duration will be out of dual 32-bit range while motor failure
    //         act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
    //       }
    //       catch (std::runtime_error& ex)
    //       {
    //       }
    //       act_data.stamp = frame_stamp.stamp;
    //       act_data.seq++;
    //       act_data.pos = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
    //                      static_cast<double>(act_data.q_circle) * 8 * M_PI + act_data.offset;
    //       // Converter raw CAN data to position velocity and effort.
    //       act_data.vel = act_coeff.act2vel * static_cast<double>(qd) + act_coeff.act2vel_offset;
    //       act_data.effort = act_coeff.act2effort * static_cast<double>(cur) + act_coeff.act2effort_offset;
    //       // Low pass filter
    //       act_data.lp_filter->input(act_data.vel);
    //       act_data.vel = act_data.lp_filter->output();
    //       continue;
    //     }
    //   }
    // }
    // if (data_ptr_.id2act_data_->find(frame.can_id) != data_ptr_.id2act_data_->end()) //dm
    if(frame.can_id == static_cast<unsigned int>(0x01) || 
       frame.can_id == static_cast<unsigned int>(0x02) ||
       frame.can_id == static_cast<unsigned int>(0x03) || 
       frame.can_id == static_cast<unsigned int>(0x04))
    {
      // if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
      // {
        ActData& act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
        // const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
        if (act_data.type.find("dm") != std::string::npos)
        {  
          // motor->para.id = (rx_data[0])&0x0F;
          // motor->para.state = (rx_data[0])>>4;
          act_data.state = (frame.data[0])>>4;
          act_data.q_raw = (frame.data[1]<<8)|frame.data[2];
          act_data.qd_raw = (frame.data[3]<<4)|(frame.data[4]>>4);
          act_data.t_int = ((frame.data[4]&0xF)<<8)|frame.data[5];
          act_data.pos = CanBus::uint_to_float(act_data.q_raw, P_MIN_8009, P_MAX_8009, 16);
          act_data.vel = CanBus::uint_to_float(act_data.qd_raw, V_MIN_8009, V_MAX_8009, 12);
          act_data.effort = CanBus::uint_to_float(act_data.t_int, T_MIN_8009, T_MAX_8009, 12);
          // motor->para.Tmos = (float)(rx_data[6]);
          // motor->para.Tcoil = (float)(rx_data[7]);

          // uint16_t id：这是一个无符号16位整数，用于存储电机的ID或者索引，以便于识别不同的电机。
          // uint16_t state：这同样是一个无符号16位整数，用于表示电机的状态。它可能包含了电机的运行状态、故障代码等信息。
          // int p_int、int v_int、int t_int：这些是有符号整数，分别代表电机的位置、速度和力矩（或扭矩）的整数值。这些值可能是从电机反馈中直接读取的原始数据。
          // int kp_int、int kd_int：这些是有符号整数，代表PID控制器中的比例增益（Kp）和微分增益（Kd）的整数值。这些值用于控制算法中，以调整电机的性能。
          // float pos、float vel、float tor：这些是浮点数，分别代表电机的位置、速度和力矩的浮点数值。这些值可能是经过转换和计算后的实际物理量。
          // float Kp、float Kd：这些是浮点数，代表PID控制器中的比例增益和微分增益。与kp_int和kd_int类似，但这些是浮点数版本，用于更精确的控制。
          // float Tmos、float Tcoil：这两个浮点数可能是与电机温度相关的参数，Tmos可能代表MOSFET（金属氧化物半导体场效应晶体管）的温度，而Tcoil可能代表电机的线圈温度。这些参数用于监测电机的热状态，防止过热。
        try
        {  // Duration will be out of dual 32-bit range while motor failure
          act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
        }
        catch (std::runtime_error& ex)
        {
        }
          continue;
        }
      // }
    }
    else if (frame.can_id == static_cast<unsigned int>(0x141) || frame.can_id == static_cast<unsigned int>(0x142)) //lk
    {
      // if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
      // {
        ActData& act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
        // const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
        if (act_data.type.find("lk") != std::string::npos)
        {  
          act_data.q_last = act_data.pos;
          act_data.pos = (uint16_t)((frame.data[7] << 8) | frame.data[6]);
          act_data.angle_single_round = ECD_ANGLE_COEF_LK * act_data.pos;
          act_data.vel = (1 - SPEED_SMOOTH_COEF) * act_data.vel +
                                DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(frame.data[5] << 8 | frame.data[4]));          
          act_data.effort = (1 - CURRENT_SMOOTH_COEF) * act_data.effort +
                                  CURRENT_SMOOTH_COEF * (float)((int16_t)(frame.data[3] << 8 | frame.data[2]));
          act_data.temp = frame.data[1];
          if (act_data.pos - act_data.q_last > 32768)
              act_data.q_circle--;
          else if (act_data.pos - act_data.q_last < -32768)
              act_data.q_circle++;
          act_data.total_angle = act_data.q_circle * 360 + act_data.angle_single_round;          
        try
        {  // Duration will be out of dual 32-bit range while motor failure
          act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
        }
        catch (std::runtime_error& ex)
        {
        }
          continue;
        }
      // }
    }           
    else if((CAN_PACKET_ID)frame.can_id>>8  == CAN_PACKET_STATUS)
    {
      if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
      {
      ActData& act_data = data_ptr_.id2act_data_->find(frame.data[0])->second;
      // const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
      // if(can_rx_data[0] != 0x07 || can_rx_data[1] != ID){return;}
	    // act_data.pos = (uint32_t)(frame.data[5]<<16 | frame.data[4]<<8 |	frame.data[3]);
      act_data.vel = ((uint32_t) frame.data[0]) << 24 |
					((uint32_t) frame.data[1]) << 16 |
					((uint32_t) frame.data[2]) << 8 |
					((uint32_t) frame.data[3]);
      act_data.lp_filter->input(act_data.vel);
      // act_data.vel = act_data.lp_filter->output();
      continue;
      }
    }
    else if (data_ptr_.id2imu_data_->find(frame.can_id) != data_ptr_.id2imu_data_->end())  // Check if IMU gyro
    {
      ImuData& imu_data = data_ptr_.id2imu_data_->find(frame.can_id)->second;
      imu_data.gyro_updated = true;
      imu_data.angular_vel[0] = (((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.angular_vel_coeff) +
                                imu_data.angular_vel_offset[0];
      imu_data.angular_vel[1] = (((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.angular_vel_coeff) +
                                imu_data.angular_vel_offset[1];
      imu_data.angular_vel[2] = (((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.angular_vel_coeff) +
                                imu_data.angular_vel_offset[2];
      imu_data.time_stamp = frame_stamp.stamp;
      int16_t temp = (int16_t)((frame.data[6] << 3) | (frame.data[7] >> 5));
      if (temp > 1023)
        temp -= 2048;
      imu_data.temperature = temp * imu_data.temp_coeff + imu_data.temp_offset;
      continue;
    }
    else if (data_ptr_.id2imu_data_->find(frame.can_id - 1) != data_ptr_.id2imu_data_->end())  // Check if IMU accel
    {
      ImuData& imu_data = data_ptr_.id2imu_data_->find(frame.can_id - 1)->second;
      imu_data.accel_updated = true;
      imu_data.linear_acc[0] = ((int16_t)((frame.data[1]) << 8) | frame.data[0]) * imu_data.accel_coeff;
      imu_data.linear_acc[1] = ((int16_t)((frame.data[3]) << 8) | frame.data[2]) * imu_data.accel_coeff;
      imu_data.linear_acc[2] = ((int16_t)((frame.data[5]) << 8) | frame.data[4]) * imu_data.accel_coeff;
      imu_data.time_stamp = frame_stamp.stamp;
      imu_data.camera_trigger = frame.data[6] & 1;
      imu_data.enabled_trigger = frame.data[6] & 2;
      imu_data.imu_filter->update(frame_stamp.stamp, imu_data.linear_acc, imu_data.angular_vel, imu_data.ori,
                                  imu_data.linear_acc_cov, imu_data.angular_vel_cov, imu_data.ori_cov,
                                  imu_data.temperature, imu_data.camera_trigger && imu_data.enabled_trigger);
      continue;
    }
    else if (data_ptr_.id2tof_data_->find(frame.can_id) != data_ptr_.id2tof_data_->end())
    {
      TofData& tof_data = data_ptr_.id2tof_data_->find(frame.can_id)->second;
      tof_data.distance = ((int16_t)((frame.data[1]) << 8) | frame.data[0]);
      tof_data.strength = ((int16_t)((frame.data[3]) << 8) | frame.data[2]);
      continue;
    }
    if (frame.can_id != 0x0)
      ROS_ERROR_STREAM_ONCE("Can not find defined device, id: 0x" << std::hex << frame.can_id
                                                                  << " on bus: " << bus_name_);
  }
  read_buffer_.clear();
}

void CanBus::write(can_frame* frame)
{
  // socket_can_.write(frame);
  socket_tcp_can0.write(frame);
  socket_tcp_can1.write(frame);
}

void CanBus::frameCallback(const can_frame& frame)
{
  std::lock_guard<std::mutex> guard(mutex_);
  CanFrameStamp can_frame_stamp{ .frame = frame, .stamp = ros::Time::now() };
  read_buffer_.push_back(can_frame_stamp);
}

float CanBus::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int CanBus::float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
}  // namespace rm_hw

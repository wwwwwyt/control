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

#pragma once

#include "rm_hw/hardware_interface/socketcan.h"
#include "rm_hw/hardware_interface/sockettcp.h"
#include "rm_hw/hardware_interface/types.h"

#include <chrono>
#include <mutex>
#include <thread>


#define P_MIN_8009 -12.5f
#define P_MAX_8009 12.5f
#define V_MIN_8009 -45.0f
#define V_MAX_8009 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN_8009 -50.0f
#define T_MAX_8009 50.0f

namespace rm_hw
{
struct CanFrameStamp
{
  can_frame frame;
  ros::Time stamp;
};

typedef enum {
CAN_PACKET_SET_DUTY = 0,
CAN_PACKET_SET_CURRENT,
CAN_PACKET_SET_CURRENT_BRAKE,
CAN_PACKET_SET_RPM,
CAN_PACKET_SET_POS,
CAN_PACKET_FILL_RX_BUFFER,
CAN_PACKET_FILL_RX_BUFFER_LONG,
CAN_PACKET_PROCESS_RX_BUFFER,
CAN_PACKET_PROCESS_SHORT_BUFFER,
CAN_PACKET_STATUS,
CAN_PACKET_SET_CURRENT_REL,
CAN_PACKET_SET_CURRENT_BRAKE_REL,
CAN_PACKET_SET_CURRENT_HANDBRAKE,
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
CAN_PACKET_STATUS_2,
CAN_PACKET_STATUS_3,
CAN_PACKET_STATUS_4,
CAN_PACKET_PING,
CAN_PACKET_PONG,
CAN_PACKET_DETECT_APPLY_ALL_FOC,
CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
CAN_PACKET_CONF_CURRENT_LIMITS,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
CAN_PACKET_CONF_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_FOC_ERPMS,
CAN_PACKET_CONF_STORE_FOC_ERPMS,
CAN_PACKET_STATUS_5
} CAN_PACKET_ID;

class CanBus
{
public:
  /** \brief
   * Initialize device at can_device, retry if fail. Set up header of CAN frame.
   *
   * \param bus_name Bus's name(example: can0).
   * \param data_ptr Pointer which point to CAN data.
   */
  CanBus(const std::string& bus_name, CanDataPtr data_ptr, int thread_priority);
  /** \brief Read active data from read_buffer_ to data_ptr_, such as position, velocity, torque and so on. Clear
   * read_buffer_ after reading.
   *
   * \param time ROS time, but it doesn't be used.
   */
  void read(ros::Time time);
  /** \brief Write commands to can bus.
   *
   */
  void write();

  void write(can_frame* frame);
  
  /**
  ************************************************************************
  * @brief:      	uint_to_float: 无符号整数转换为浮点数函数
  * @param[in]:   x_int: 待转换的无符号整数
  * @param[in]:   x_min: 范围最小值
  * @param[in]:   x_max: 范围最大值
  * @param[in]:   bits:  无符号整数的位数
  * @retval:     	浮点数结果
  * @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
  ************************************************************************
  **/
  float uint_to_float(int x_int, float x_min, float x_max, int bits);

  /**
  ************************************************************************
  * @brief:      	float_to_uint: 浮点数转换为无符号整数函数
  * @param[in]:   x_float:	待转换的浮点数
  * @param[in]:   x_min:		范围最小值
  * @param[in]:   x_max:		范围最大值
  * @param[in]:   bits: 		目标无符号整数的位数
  * @retval:     	无符号整数结果
  * @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
  ************************************************************************
  **/
  int float_to_uint(float x_float, float x_min, float x_max, int bits);

  const std::string bus_name_;

private:
  /** \brief This function will be called when CAN bus receive message. It push frame which received into a vector: read_buffer_.
   *
   * @param frame The frame which socketcan receive.
   */
  void frameCallback(const can_frame& frame);

  can::SocketCAN socket_can_;
  can::SocketTcp socket_tcp_can0;
  can::SocketTcp socket_tcp_can1;
  CanDataPtr data_ptr_;
  std::vector<CanFrameStamp> read_buffer_;

  can_frame rm_frame0_{};  // for id 0x201~0x204
  can_frame rm_frame1_{};  // for id 0x205~0x208

  mutable std::mutex mutex_;
};

}  // namespace rm_hw

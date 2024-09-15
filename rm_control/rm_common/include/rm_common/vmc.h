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
// Created by qiayuan on 8/13/20.
//

#pragma once

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Core>
#include "ori_tool.h"

typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5;//AE长度 //单位为m
	float	l1;//单位为m
	float l2;//单位为m
	float l3;//单位为m
	float l4;//单位为m
	float Pitch; //右腿机体pitch
    float PithGyro;
	float PitchL;// 左 -右
    float PithGyroL;
    
    float XB,YB;//B点的坐标
	float XD,YD;//D点的坐标
	
	float XC,YC;//C点的直角坐标
	float L0,phi0;//C点的极坐标
	float alpha;
	float d_alpha;	
	
	float	lBD;//BD两点的距离
	
	float d_phi0;//现在C点角度phi0的变换率
	float last_phi0;//上一次C点角度，用于计算角度phi0的变换率d_phi0

	float A0,B0,C0;//中间变量
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;//笛卡尔空间力到关节空间的力的雅可比矩阵系数
	// float torque_set[2];
  float T_back, T_front;

	float F0;//F0为五连杆机构末端沿腿的推力 
	float Tp;
	
	float theta;
	float d_theta;//theta的一阶导数
	float last_d_theta;
	float dd_theta;//theta的二阶导数
	
	float d_L0;//L0的一阶导数
	float dd_L0;//L0的二阶导数
	float last_L0;
	float last_d_L0;
	
	float FN;//支持力
	
	uint8_t first_flag;
	uint8_t leg_flag;//腿长完成标志
} vmc_leg_t;

void VMC_init(vmc_leg_t& vmc, double l1, double l2, double l3, double l4, double l5)  // 给杆长赋值
{
  vmc.l5 = l5;  // AE长度 //单位为m
  vmc.l1 = l1;  // 单位为m
  vmc.l2 = l2;  // 单位为m
  vmc.l3 = l3;  // 单位为m
  vmc.l4 = l4;  // 单位为m
}

void VMC_calc_2( vmc_leg_t& vmc)  // 计算期望的关节输出力矩
{
  float phi12 = vmc.phi1 - vmc.phi2;
  float phi34 = vmc.phi3 - vmc.phi4;
  float phi32 = vmc.phi3 - vmc.phi2;
  float phi03 = vmc.phi0 - vmc.phi3;
  float phi02 = vmc.phi0 - vmc.phi2;
  // float F_m_L = vmc.F0 * vmc.L0;
  // vmc.T_front = (vmc.l1  * sin(phi12) * (F_m_L * sin(phi03) + vmc.Tp * cos(phi03))) / (vmc.L0 * sin(phi32));
  // vmc.T_back = (vmc.l4  * sin(phi34) * (F_m_L * sin(phi02) + vmc.Tp * cos(phi02))) / (vmc.L0 * sin(phi32));

  vmc.j11 = (vmc.l1 * sin(phi03) * sin(phi12)) / sin(phi32);
  vmc.j12 =
      (vmc.l1 * cos(phi03) * sin(phi12)) / (vmc.L0 * sin(phi32));
  vmc.j21 = (vmc.l4 * sin(phi02) * sin(phi34)) / sin(phi32);
  vmc.j22 =
      (vmc.l4 * cos(phi02) * sin(phi34)) / (vmc.L0 * sin(phi32));


  vmc.T_front =
      vmc.j11 * vmc.F0 + vmc.j12 * vmc.Tp;  // 角一  得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力   
  vmc.T_back = vmc.j21 * vmc.F0 + vmc.j22 * vmc.Tp;  //角四  得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩

  // ROS_INFO("test %f" , vmc.j11);
  // ROS_INFO("j12_  : j11: %f | j12: %f | j21: %f  | yy: %f ", vmc.l1 ,vmc.phi0 , vmc.phi3 ,cos(vmc.phi0-vmc.phi3)
  // ); ROS_INFO("j12   : j11: %f | j12: %f | j21: %f  ", vmc.l1*cos(vmc.phi0-vmc.phi3)
  // ,sin(vmc.phi1-vmc.phi2),vmc.L0*sin(vmc.phi3-vmc.phi2)); ROS_INFO("test : j11: %f | j12: %f | j21: %f | j22: %f
  // | f0: %f | fp: %f ", vmc.j11 ,vmc.j12,vmc.j21,vmc.j22,vmc.F0,vmc.Tp);

  // ROS_INFO("j22 : j22: %f | j22_: %f | j22__: %f ", vmc.torque_set[1],vmc.j21*vmc.F0,vmc.j22*vmc.Tp);
}

void VMC_calc_1(vmc_leg_t& vmc, INS_t* ins,
                                         float dt)  // 计算theta和d_theta给lqr用，同时也计算腿长L0
{
  // ROS_INFO("VMC R chassis: PithR %f | PithGyroR %f", PitchR, PithGyroR);
  vmc.YD = vmc.l4 * sin(vmc.phi4);            // D的y坐标
  vmc.YB = vmc.l1 * sin(vmc.phi1);            // B的y坐标
  vmc.XD = vmc.l5 + vmc.l4 * cos(vmc.phi4);  // D的x坐标
  vmc.XB = vmc.l1 * cos(vmc.phi1);            // B的x坐标

  vmc.lBD = sqrt((vmc.XD - vmc.XB) * (vmc.XD - vmc.XB) + (vmc.YD - vmc.YB) * (vmc.YD - vmc.YB));

  vmc.A0 = 2 * vmc.l2 * (vmc.XD - vmc.XB);
  vmc.B0 = 2 * vmc.l2 * (vmc.YD - vmc.YB);
  vmc.C0 = vmc.l2 * vmc.l2 + vmc.lBD * vmc.lBD - vmc.l3 * vmc.l3;
  vmc.phi2 =
      2 * atan2f((vmc.B0 + sqrt(vmc.A0 * vmc.A0 + vmc.B0 * vmc.B0 - vmc.C0 * vmc.C0)), vmc.A0 + vmc.C0);
  vmc.phi3 = atan2f(vmc.YB - vmc.YD + vmc.l2 * sin(vmc.phi2), vmc.XB - vmc.XD + vmc.l2 * cos(vmc.phi2));
  // C点直角坐标
  vmc.XC = vmc.l1 * cos(vmc.phi1) + vmc.l2 * cos(vmc.phi2);
  vmc.YC = vmc.l1 * sin(vmc.phi1) + vmc.l2 * sin(vmc.phi2);
  // C点极坐标
  vmc.L0 = sqrt((vmc.XC - vmc.l5 / 2.0f) * (vmc.XC - vmc.l5 / 2.0f) + vmc.YC * vmc.YC);

  vmc.phi0 = atan2f(vmc.YC, (vmc.XC - vmc.l5 / 2.0f));  // phi0用于计算lqr需要的theta
  vmc.alpha = 3.1415916 / 2.0f - vmc.phi0;

  if (vmc.first_flag == 0)
  {
    vmc.last_phi0 = vmc.phi0;
    vmc.first_flag = 1;
  }
  vmc.d_phi0 = (vmc.phi0 - vmc.last_phi0) / dt;  // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
  vmc.d_alpha = 0.0f - vmc.d_phi0;

  vmc.theta = 3.1415926 / 2.0f - vmc.Pitch - vmc.phi0;  // 得到状态变量1
  vmc.d_theta = (-vmc.PithGyro - vmc.d_phi0);    // 得到状态变量2
// if(vmc.last_phi0 == vmc.phi0)
// {
//   vmc.d_theta = vmc.last_d_theta;
// }
  vmc.last_phi0 = vmc.phi0;

  vmc.d_L0 = (vmc.L0 - vmc.last_L0) / dt;       // 腿长L0的一阶导数
  vmc.dd_L0 = (vmc.d_L0 - vmc.last_d_L0) / dt;  // 腿长L0的二阶导数

  vmc.last_d_L0 = vmc.d_L0;
  vmc.last_L0 = vmc.L0;

  vmc.dd_theta = (vmc.d_theta - vmc.last_d_theta) / dt;
  vmc.last_d_theta = vmc.d_theta;
  // ROS_INFO("VMC R  : L0 %f | theta %f", vmc.L0, vmc.theta);
}

float LQR_K_calc(float* coe, float len)
{
  return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}
//
// Created by yezi on 2022/11/15.
//

#pragma once

#include <rm_common/lqr.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_msgs/BalanceState.h>
// #include <rm_common/VMC_calc.h>
#include <rm_common/ori_tool.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
  typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5;//AE长度 //单位为m
	float	l1;//单位为m
	float l2;//单位为m
	float l3;//单位为m
	float l4;//单位为m
	
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
	float torque_set[2];

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
using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, 
                                             hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
    BLOCK
  };

public:
  BalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void balanceL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,const ros::Duration& period);
  void balanceR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,const ros::Duration& period);
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  geometry_msgs::Twist odometry() override;
  void mySaturate(float *in,float min,float max);


#include <cmath>
// #include <string>
// #include "math_utilities.h"

// #include "ori_tool.h"

#define pi 3.1415926f
#define LEG_PID_KP  350.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  3000.0f
#define LEG_PID_MAX_OUT  90.0f //90牛
#define LEG_PID_MAX_IOUT 0.0f



 void VMC_init(vmc_leg_t *vmc,double l1, double l2, double l3, double l4, double l5);//给杆长赋值

 void VMC_calc_1_right(vmc_leg_t *vmc,INS_t *ins,float dt);//计算theta和d_theta给lqr用，同时也计算腿长L0
 void VMC_calc_1_left(vmc_leg_t *vmc,INS_t *ins,float dt);
 void VMC_calc_2(vmc_leg_t *vmc);//计算期望的关节输出力矩

 uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins);//右腿离地检测
 uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins);//左腿离地检测

 float LQR_K_calc(float *coe,float len);
	





  static const int STATE_DIM = 10;
  static const int CONTROL_DIM = 4;
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;
  double wheel_radius_, wheel_base_;
  double position_des_ = 0;
  double position_offset_ = 0.;
  double position_clear_threshold_ = 0.;
  double yaw_des_ = 0;

  int balance_mode_;
  ros::Time block_time_, last_block_time_;
  double block_angle_, block_duration_, block_velocity_, block_effort_, anti_block_effort_, block_overtime_;
  bool balance_state_changed_ = false, maybe_block_ = false;

  ros::Subscriber imu_sub_;
  geometry_msgs::Vector3 chassis_angle_;
  geometry_msgs::Vector3 chassis_angle_v_;

  // hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle 
  // left_wheel_joint_handle_, right_wheel_joint_handle_,
      left_front_joint_handle_, left_back_joint_handle_, right_front_joint_handle_, right_back_joint_handle_;

  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::BalanceState>> RtpublisherPtr;
  RtpublisherPtr state_pub_;
  geometry_msgs::Vector3 angular_vel_base_;
  double roll_, pitch_, yaw_;

  vmc_leg_t left;
  vmc_leg_t right;

  uint8_t right_flag;
  uint8_t left_flag;

  control_toolbox::Pid pid_l_;
  control_toolbox::Pid pid_r_;
  control_toolbox::Pid Tp_Pid;//防劈叉补偿pd
  control_toolbox::Pid Turn_Pid;//转向pd

  chassis_t chassis_move;

  INS_t ins;
  float LQR_K[12]={ 
   -2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
    2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

  // float Poly_Coefficient[12][4]={	{-88.3079710751263,	68.9068310796955,	-30.0003802287502,	-0.197774178106864},
  //                                 {1.52414598059982	,-1.09343038036609,	-2.82688593867512,	0.0281973842051861},
  //                                 {-21.8700750609220	,12.7421672466682,	-2.58779676995074	,-0.750848242540331},
  //                                 {-29.3271263750692,	17.6067629457167,	-4.23484645974363	,-1.08976980288501},
  //                                 {-147.771748892911,	94.0665615939814,	-22.5139626085997	,2.53224765312440},
  //                                 {-6.72857056332562,	4.46216499907277,	-1.14328671767927	,0.176775242328476},
  //                                 {-43.1495035855057,	35.1427890165576,	-12.7617044245710	,3.36940801739176},
  //                                 {4.14428184617563,	-2.56933858132474,	0.479050092243477	,0.248175261724735},
  //                                 {-229.898177881547	,144.949258291255	,-33.9196587052128,	3.44291788865558},
  //                                 {-329.509693153293,	207.219295206736,	-48.3799707459102	,4.952560575479143},
  //                                 {380.589246401548,	-223.660017597103	,46.1696952431268	,9.82308882692083},
  //                                 {26.1010681824798	,-15.7241310513153	,3.39175554658673	,0.278568898146322}};  
  float Poly_Coefficient[12][4]={	{-613.357621418816 ,542.811474487983 ,-183.842527268893 ,3.41106325464979},
                                  {-6.61498596429207 ,9.55936632666744 ,-8.34575061773464 ,0.335157027001344},
                                  {-50.1572352677957 ,39.8532850703017 ,-11.0426886643061 ,0.408880872397129},
                                  {-100.025682458270 ,79.6803462367037 ,-22.3233075222893 ,0.814559028832847},
                                  {4.08911232373703 ,30.4396227941444 ,-25.1770217968444 ,6.60535781474694},
                                  {5.40535962867781 ,-1.27447513528345 ,-1.19023415582140 ,0.545203817480410},
                                  {1411.73684118300 ,-849.189008070601 ,96.9440722874070 ,25.5773073930053},
                                  {112.904038317453 ,-83.3301921651894 ,18.5265292621191 ,0.313817096067359},
                                  {-21.1136567323741 ,38.9641763103182 ,-21.5338371694114 ,4.55939989237205},
                                  {-37.0955249349109 ,73.8340315937851 ,-41.9908098613963 ,9.07020114467825},
                                  {1440.97171079948 ,-1158.35400660641 ,326.803825498889 ,-15.0388016935310},
                                  {117.942493716830 ,-96.9030008525867 ,28.2489221876323 ,-1.76438009000841}};    
};

}  // namespace rm_chassis_controllers

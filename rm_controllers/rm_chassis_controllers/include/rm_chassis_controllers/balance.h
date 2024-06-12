//
// Created by yezi on 2022/11/15.
//

#pragma once

#include <rm_common/lqr.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_msgs/BalanceState.h>
#include <rm_common/VMC_calc.h>
#include <rm_common/ori_tool.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
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
  void balanceL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K);
  void balanceR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K);
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  geometry_msgs::Twist odometry() override;
  void mySaturate(float *in,float min,float max);


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

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_,
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

  float Poly_Coefficient[12][4]={	{-88.3079710751263,	68.9068310796955,	-30.0003802287502,	-0.197774178106864},
                                  {1.52414598059982	,-1.09343038036609,	-2.82688593867512,	0.0281973842051861},
                                  {-21.8700750609220	,12.7421672466682,	-2.58779676995074	,-0.750848242540331},
                                  {-29.3271263750692,	17.6067629457167,	-4.23484645974363	,-1.08976980288501},
                                  {-147.771748892911,	94.0665615939814,	-22.5139626085997	,2.53224765312440},
                                  {-6.72857056332562,	4.46216499907277,	-1.14328671767927	,0.176775242328476},
                                  {-43.1495035855057,	35.1427890165576,	-12.7617044245710	,3.36940801739176},
                                  {4.14428184617563,	-2.56933858132474,	0.479050092243477	,0.248175261724735},
                                  {-229.898177881547	,144.949258291255	,-33.9196587052128,	3.44291788865558},
                                  {-329.509693153293,	207.219295206736,	-48.3799707459102	,4.952560575479143},
                                  {380.589246401548,	-223.660017597103	,46.1696952431268	,9.82308882692083},
                                  {26.1010681824798	,-15.7241310513153	,3.39175554658673	,0.278568898146322}};  
};

}  // namespace rm_chassis_controllers

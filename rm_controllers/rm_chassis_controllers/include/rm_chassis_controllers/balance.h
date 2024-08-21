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
#include <rm_common/vmc.h>
#include <rm_common/dm_tool.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>

#include <cmath>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{

using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
    BLOCK
  };

public:
  BalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void motor_init();
  void motor_send();
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void balance_init(const ros::Duration& period);
  void balanceL_control_loop(chassis_t* chassis, vmc_leg_t* vmcl, INS_t* ins, float* LQR_K, const ros::Duration& period);
  void balanceR_control_loop(chassis_t* chassis, vmc_leg_t* vmcr, INS_t* ins, float* LQR_K, const ros::Duration& period);
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  void xvEstimateKF_Init(KalmanFilter_t* EstimateKF);
  void observe(const ros::Time& time, const ros::Duration& period);
  geometry_msgs::Twist odometry() override;
  void mySaturate(float* in, float min, float max);

#define pi 3.1415926f
#define LEG_PID_KP 350.0f
#define LEG_PID_KI 0.0f
#define LEG_PID_KD 3000.0f
#define LEG_PID_MAX_OUT 90.0f  // 90牛
#define LEG_PID_MAX_IOUT 0.0f

#define VEL_PROCESS_NOISE 10    // 速度过程噪声
#define VEL_MEASURE_NOISE 2000  // 速度测量噪声

  uint8_t ground_detection(vmc_leg_t* vmc, INS_t* ins);  // 左腿离地检测

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
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_front_joint_handle_,
      left_back_joint_handle_, right_front_joint_handle_, right_back_joint_handle_;

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
  control_toolbox::Pid Tp_Pid;    // 防劈叉补偿pd
  control_toolbox::Pid Turn_Pid;  // 转向pd
  control_toolbox::Pid Roll_Pid;
  chassis_t chassis_move;

  INS_t ins;
  int imu_dis_time{ 0 };  // imu稳定时间 刚上电数据不稳定
  float LQR_K[12] = { -2.1954, -0.2044, -0.8826, -1.3245, 1.2784,  0.1112,
                      2.5538,  0.2718,  1.5728,  2.2893,  12.1973, 0.4578 };

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
  float Poly_Coefficient[12][4] = { { -957.627474314536, 880.506286377135, -357.876255852018, -36.7834942715912 },
                                    { -135.030569678209, 118.091699923961, -60.3614274237114, -12.2212002606395 },
                                    { 49.8430410051520, -34.3689406974576, 6.78173202783122, -10.0253468982792 },
                                    { 289.437783545266, -225.913045146238, 60.8934545072109, -24.0566252619899 },
                                    { -585.516149511807, 585.591783556904, -252.321869742421, 66.4374704298668 },
                                    { -215.517034931957, 191.383282412690, -68.6277845937440, 15.1831152113041 },
                                    { 850.548380184201, -597.474096461842, 101.966428603536, 34.8948149041715 },
                                    { -82.9528879226631, 103.268778196232, -55.5939186137298, 20.0012950152193 },
                                    { 227.315239370022, -160.379400410347, 31.9987429466032, 1.75645339783843 },
                                    { 328.423695129978, -215.423535541665, 33.5659909618445, 6.11788733764742 },
                                    { 524.414659907536, -516.109733495038, 198.131016293426, 53.8592872848996 },
                                    { 14.1727521823463, -33.5333906846945, 21.3370744202405, 6.16232348648967 } };
};

int8_t motor_init_{ 0 };
int8_t Kalman_init{ 0 };
static float wr, wl = 0.0f;
static float vrb, vlb = 0.0f;
static float aver_v = 0.0f;

const float vaEstimateKF_H[4] = { 1.0f, 0.0f, 0.0f, 1.0f };  // 设置矩阵H为常量

float ins_dt = 0.01f;  // imu频率 成品100hz
float gravity[3] = { 0, 0, 9.81f };
}  // namespace rm_chassis_controllers

//
// Created by qiayuan on 2022/11/15.
//
#include "rm_chassis_controllers/balance.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/BalanceState.h>
#include <angles/angles.h>

namespace rm_chassis_controllers
{
bool BalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  // leg init

  // imu
  //  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
  //      getParam(controller_nh, "imu_name", std::string("base_imu")));
  imu_sub_ = controller_nh.subscribe("/imu", 1000, &BalanceController::imuCallback, this);
  //
  std::string left_wheel_joint, right_wheel_joint, left_front_joint, right_front_joint, left_back_joint,
      right_back_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("left/front_joint", left_front_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint) ||
      !controller_nh.getParam("right/front_joint", right_front_joint) ||
      !controller_nh.getParam("left/back_joint", left_back_joint) ||
      !controller_nh.getParam("right/back_joint", right_back_joint))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  // 关节控制器 句柄
  //  左前 左后
  left_front_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_front_joint);
  left_back_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_back_joint);
  // 右
  right_front_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_front_joint);
  right_back_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_back_joint);
  // 俩轮
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);

  // 文件参数获取
  double l1_, l2_, l3_, l4_, l5_;
  // if (!controller_nh.getParam("m", m))
  // {
  //   ROS_ERROR("Params m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
  //   return false;
  // }
  if (!controller_nh.getParam("l1", l1_))
  {
    ROS_ERROR("Params l1 doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l2", l2_))
  {
    ROS_ERROR("Params l2 doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l3", l3_))
  {
    ROS_ERROR("Params l3 doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l4", l4_))
  {
    ROS_ERROR("Params l4 doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l5", l5_))
  {
    ROS_ERROR("Params l5 doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

    if (!controller_nh.getParam("wheel_radius", wheel_radius_))
  {
    ROS_ERROR("Params l5 doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }

  // pid参数获取
  if (controller_nh.hasParam("pid_l"))
    if (!pid_l_.init(ros::NodeHandle(controller_nh, "pid_l")))
      return false;
  if (controller_nh.hasParam("pid_r"))
    if (!pid_r_.init(ros::NodeHandle(controller_nh, "pid_r")))
      return false;
  if (controller_nh.hasParam("Tp_Pid"))
    if (!Tp_Pid.init(ros::NodeHandle(controller_nh, "Tp_Pid")))
      return false;
  if (controller_nh.hasParam("Turn_Pid"))
    if (!Turn_Pid.init(ros::NodeHandle(controller_nh, "Turn_Pid")))
      return false;
  if (controller_nh.hasParam("Roll_Pid"))
    if (!Roll_Pid.init(ros::NodeHandle(controller_nh, "Roll_Pid")))
      return false;
  BalanceController::motor_init();
  VMC_init(&left, l1_, l2_, l3_, l4_, l5_);  // 给杆长赋值
  VMC_init(&right, l1_, l2_, l3_, l4_, l5_);

  state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::BalanceState>(root_nh, "/state", 100));
  // balance_mode_ = BalanceMode::NORMAL;

  return true;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  ROS_INFO("time_dit : %f", period.toSec());
  
 

  // 更新数据
  left.phi1 = 3.8608428383366564 + left_back_joint_handle_.getPosition();  // 41.21 以机械上限位角度为零点
  left.phi4 = left_front_joint_handle_.getPosition() - 0.7192501847468633;
  right.phi1 = 3.8608428383366564 + right_front_joint_handle_.getPosition();
  right.phi4 = right_back_joint_handle_.getPosition() - 0.7192501847468633;

  chassis_move.v_set = vel_cmd_.x;
  chassis_move.x_set += vel_cmd_.x * period.toSec();
  // 更新机体imu数值
  chassis_move.myPith = 0.0f - ins.Pitch;
  chassis_move.myPithGyro = ins.Gyro[0];
  chassis_move.total_yaw = ins.YawTotalAngle;
  chassis_move.roll = ins.Roll;
  chassis_move.theta_err = 0.0f - (right.theta + left.theta);

  chassis_move.roll_lenth = Roll_Pid.computeCommand(ins.Roll, period); //roll 补偿

  // ROS_INFO("test_num || lf: %f | lb: %f | rf: %f | rb: %f ", left_front_joint_handle_.getPosition(),
  //          left_back_joint_handle_.getPosition(), right_front_joint_handle_.getPosition(),
  //          right_back_joint_handle_.getPosition());
  ROS_INFO("init_leg || left_ph1: %f | left_ph4: %f | right_ph1: %f | right_ph4: %f ", left.phi1, left.phi4, right.phi1,
           right.phi4);
    ROS_INFO("init_leg || left_ph1: %f ", right_wheel_joint_handle_.getVelocity());         
  // ROS_INFO("init_chassis || chassis_P: %f | chassis_dp: %f | chassis_total_yaw: %f | chassis_roll: %f ",
  // chassis_move.myPith,chassis_move.myPithGyro,chassis_move.total_yaw,chassis_move.roll);


  BalanceController::observe(time, period);//整车观测

  // if(ins.Pitch<(3.1415926f/6.0f)&&ins.Pitch>(-3.1415926f/6.0f))
  // {//根据pitch角度判断倒地自起是否完成
  // 	chassis_move.recover_flag=0;
  // }

  balanceL_control_loop(&chassis_move, &left, &ins, LQR_K, period);   // 控制计算
  balanceR_control_loop(&chassis_move, &right, &ins, LQR_K, period);  // 控制计算		 //load controller

  // if(chassis_move.start_flag==1)
  // {
  BalanceController::motor_send();
  // }
  // else if(chassis_move.start_flag==0)
  // {
  //     // 左前 左后
  //   left_front_joint_handle_.setCommand(0);
  //   left_back_joint_handle_ .setCommand(0);
  //   //右
  //   right_front_joint_handle_ .setCommand(0);
  //   right_back_joint_handle_.setCommand(0);
  //   // 俩轮
  //   // left_wheel_joint_handle_ .setCommand(0);
  //   // right_wheel_joint_handle_ .setCommand(0);
  //   ROS_INFO("left_front: ");
  // }
  if (!motor_init_)
  {
    BalanceController::motor_init();
    motor_init_ += 1;
  }
}

void BalanceController::balanceL_control_loop(chassis_t* chassis, vmc_leg_t* vmcl, INS_t* ins, float* LQR_K,
                                              const ros::Duration& period)
{
  VMC_calc_1_left(vmcl, ins,
                  period.toSec());  // 计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是  秒

  for (int i = 0; i < 12; i++)
  {
    LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcl->L0);
  }

  // ROS_INFO("k阵: || LQR_K[1]: %f | LQR_K[3]: %f | LQR_K[5]: %f | LQR_K[7]: %f ", LQR_K[1] ,LQR_K[3],LQR_K[5],LQR_K[7]);
  chassis->wheel_motor[1].wheel_T = (
                                     LQR_K[0] * (vmcl->theta - 0.0f) +
                                     LQR_K[1] * (vmcl->d_theta - 0.0f)
                                     // +LQR_K[2]*(chassis->x_set-chassis->x_filter)
                                     // +LQR_K[3]*(chassis->v_set-chassis->v_filter)
                                     + LQR_K[4] * (0.0f - chassis->myPith) + LQR_K[5] * (0.0f - chassis->myPithGyro )
                                    );
  // ROS_INFO("sd%f " ,chassis->wheel_motor[1].wheel_T );
  // //右边髋关节输出力矩
  // vmcl->Tp=(LQR_K[6]*(vmcl->theta-0.0f)
  // 				+LQR_K[7]*(vmcl->d_theta-0.0f)
  // 				// +LQR_K[8]*(chassis->x_set-vel_cmd_.x)
  // 				// +LQR_K[9]*(chassis->v_set-vel_cmd_.y)
  // 				// +LQR_K[10]*(0.0f - chassis->myPith)
  // 				// +LQR_K[11]*(0.0f - chassis->myPithGyro)
  //         );

  chassis->wheel_motor[1].wheel_T = chassis->wheel_motor[1].wheel_T - chassis->turn_T;  // 轮毂电机输出力矩
  BalanceController::mySaturate(&chassis->wheel_motor[1].wheel_T, -10.0f, 10.0f);

  // vmcl->Tp += chassis->leg_tp;//髋关节输出力矩

  // vmcl->F0=13.0f+PID_Calc(leg,vmcl->L0,chassis->leg_set);//前馈+pd
  vmcl->F0 = 0.0f + pid_l_.computeCommand(chassis->leg_set - vmcl->L0, period);
  // 前馈 + pd
  ROS_INFO("LLL  F0: %f | leg_d :%f ", vmcl->F0, chassis->leg_set - vmcl->L0);

  // left_flag=ground_detectionL(vmcl,ins);//左腿离地检测

  //  if(chassis->recover_flag==0)
  //  {//倒地自起不需要检测是否离地
  // 	if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
  // 	{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
  // 		chassis->wheel_motor[1].wheel_T=0.0f;
  // 		vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);

  // 		chassis->x_filter=0.0f;//对位移清零
  // 		chassis->x_set=chassis->x_filter;
  // 		chassis->turn_set=chassis->total_yaw;
  // 		vmcl->Tp=vmcl->Tp+chassis->leg_tp;
  // 	}
  // 	else
  // 	{//没有离地
  // 		vmcl->leg_flag=0;//置为0
  // 	}
  //  }
  //  else if(chassis->recover_flag==1)
  //  {
  // 	 vmcl->Tp=0.0f;
  //  }

  BalanceController::mySaturate(&vmcl->F0, -300.0f, 300.0f);  // 限幅

vmcl->Tp = 0;
  BalanceController::VMC_calc_2(vmcl);  // 计算期望的关节输出力矩

  ROS_INFO("RESULT---  || left_front: %f | left_back: %f ", vmcl->T_front ,vmcl->T_back);

  // 额定扭矩
  BalanceController::mySaturate(&vmcl->T_front, -20.0f, 20.0f);
  BalanceController::mySaturate(&vmcl->T_front, -20.0f, 20.0f);
  // ROS_INFO("RESULT  || torque_set[1]: %f | torque_set[0]: %f ", vmcl->T_front ,vmcl->T_back);
}

void BalanceController::balanceR_control_loop(chassis_t* chassis, vmc_leg_t* vmcr, INS_t* ins, float* LQR_K,
                                              const ros::Duration& period)
{
  VMC_calc_1_right(vmcr, ins,
                  period.toSec());  // 计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是  秒

  for (int i = 0; i < 12; i++)
  {
    LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcr->L0);
  }

  // chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
  //  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点

  // double p_,i_,d_,i_max_,i_min_;
  // Turn_Pid.getGains(p_,i_,d_,i_max_,i_min_);
  // chassis->turn_T=p_*(vel_cmd_.z-chassis->total_yaw)-d_*ins->Gyro[2];//这样计算更稳一点

  // chassis->leg_tp=Tp_Pid.computeCommand( -chassis->theta_err , period);//防劈叉pid计算

  chassis->wheel_motor[0].wheel_T = (
                                     LQR_K[0] * (vmcr->theta - 0.0f) +
                                     LQR_K[1] * (vmcr->d_theta - 0.0f)
                                     // +LQR_K[2]*(chassis->x_filter-chassis->x_set)
                                     // +LQR_K[3]*(chassis->v_filter-chassis->v_set)
                                     + LQR_K[4] * (chassis->myPith - 0.0f) + LQR_K[5] * (chassis->myPithGyro - 0.0f)
                                     );
                                    

  // //右边髋关节输出力矩
  // vmcr->Tp=(LQR_K[6]*(vmcr->theta-0.0f)
  // 				+LQR_K[7]*(vmcr->d_theta-0.0f)
  // 				+LQR_K[8]*(chassis->x_filter-vel_cmd_.x)
  // 				+LQR_K[9]*(chassis->v_filter-vel_cmd_.y)
  // 				+LQR_K[10]*(chassis->myPith-0.0f)
  // 				+LQR_K[11]*(chassis->myPithGyro-0.0f));

  chassis->wheel_motor[0].wheel_T = chassis->wheel_motor[0].wheel_T - chassis->turn_T;  // 轮毂电机输出力矩
  BalanceController::mySaturate(&chassis->wheel_motor[0].wheel_T, -10.0f, 10.0f);

  // vmcr->Tp += chassis->leg_tp;//髋关节输出力矩
  vmcr->Tp = 0;

  // vmcr->F0=13.0f+pid_r_.computeCommand(chassis->leg_set - vmcr->L0 , period);//前馈+pd
  vmcr->F0 = 0.0f + pid_r_.computeCommand(chassis->leg_set - vmcr->L0, period);  // 前馈+pd
  // ROS_INFO("RRR  F0: %f | leg_d :%f ", vmcr->F0, chassis->leg_set - vmcr->L0);
  // right_flag=ground_detectionR(vmcr,ins);//右腿离地检测

  //  if(chassis->recover_flag==0)
  //  {//倒地自起不需要检测是否离地
  // 	if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
  // 	{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
  // 			chassis->wheel_motor[0].wheel_T=0.0f;
  // 			vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

  // 			chassis->x_filter=0.0f;
  // 			chassis->x_set=chassis->x_filter;
  // 			chassis->turn_set=chassis->total_yaw;
  // 			vmcr->Tp=vmcr->Tp+chassis->leg_tp;
  // 	}
  // 	else
  // 	{//没有离地
  // 		vmcr->leg_flag=0;//置为0

  // 	}
  //  }
  //  else if(chassis->recover_flag==1)
  //  {
  // 	 vmcr->Tp=0.0f;
  //  }
  //  ROS_INFO("right '' f0 :%f", vmcr->F0 );
  BalanceController::mySaturate(&vmcr->F0, -300.0f, 300.0f);  // 限幅

  BalanceController::VMC_calc_2(vmcr);  // 计算期望的关节输出力矩
  // ROS_INFO("right '' RESULT---  || right_front %f | right_back: %f ", vmcr->T_front ,vmcr->T_back);
  // 额定扭矩
  BalanceController::mySaturate(&vmcr->T_front, -20.0f, 20.0f);
  BalanceController::mySaturate(&vmcr->T_back, -20.0f, 20.0f);
}

void BalanceController::normal(const ros::Time& time, const ros::Duration& period)
{
  // if (balance_state_changed_)
  // {
  //   ROS_INFO("[balance] Enter NOMAl");
  //   balance_state_changed_ = false;
  // }

  // x_[5] = ((left_wheel_joint_handle_.getVelocity() + right_wheel_joint_handle_.getVelocity()) / 2 -
  //          imu_handle_.getAngularVelocity()[1]) *
  //         wheel_radius_;
  // x_[0] += x_[5] * period.toSec();
  // x_[1] = yaw_;
  // x_[2] = pitch_;
  // x_[3] = left_momentum_block_joint_handle_.getPosition();
  // x_[4] = right_momentum_block_joint_handle_.getPosition();
  // x_[6] = angular_vel_base_.z;
  // x_[7] = angular_vel_base_.y;
  // x_[8] = left_momentum_block_joint_handle_.getVelocity();
  // x_[9] = right_momentum_block_joint_handle_.getVelocity();
  // yaw_des_ += vel_cmd_.z * period.toSec();
  // position_des_ += vel_cmd_.x * period.toSec();
  // Eigen::Matrix<double, CONTROL_DIM, 1> u;
  // auto x = x_;
  // x(0) -= position_des_;
  // x(1) = angles::shortest_angular_distance(yaw_des_, x_(1));
  // if (state_ != RAW)
  //   x(5) -= vel_cmd_.x;
  // x(6) -= vel_cmd_.z;
  // if (std::abs(x(0) + position_offset_) > position_clear_threshold_)
  // {
  //   x_[0] = 0.;
  //   position_des_ = position_offset_;
  // }
  // u = k_ * (-x);
  // if (state_pub_->trylock())
  // {
  //   state_pub_->msg_.header.stamp = time;
  //   state_pub_->msg_.x = x(0);
  //   state_pub_->msg_.phi = x(1);
  //   state_pub_->msg_.theta = x(2);
  //   state_pub_->msg_.x_b_l = x(3);
  //   state_pub_->msg_.x_b_r = x(4);
  //   state_pub_->msg_.x_dot = x(5);
  //   state_pub_->msg_.phi_dot = x(6);
  //   state_pub_->msg_.theta_dot = x(7);
  //   state_pub_->msg_.x_b_l_dot = x(8);
  //   state_pub_->msg_.x_b_r_dot = x(9);
  //   state_pub_->msg_.T_l = u(0);
  //   state_pub_->msg_.T_r = u(1);
  //   state_pub_->msg_.f_b_l = u(2);
  //   state_pub_->msg_.f_b_r = u(3);
  //   state_pub_->unlockAndPublish();
  // }

  // left_wheel_joint_handle_.setCommand(u(0));
  // right_wheel_joint_handle_.setCommand(u(1));
  // left_momentum_block_joint_handle_.setCommand(u(2));
  // right_momentum_block_joint_handle_.setCommand(u(3));
}

void BalanceController::block(const ros::Time& time, const ros::Duration& period)
{
  // if (balance_state_changed_)
  // {
  //   ROS_INFO("[balance] Enter BLOCK");
  //   balance_state_changed_ = false;

  //   last_block_time_ = ros::Time::now();
  // }
  // if ((ros::Time::now() - last_block_time_).toSec() > block_overtime_)
  // {
  //   balance_mode_ = BalanceMode::NORMAL;
  //   balance_state_changed_ = true;
  //   ROS_INFO("[balance] Exit BLOCK");
  // }
  // else
  // {
  //   left_momentum_block_joint_handle_.setCommand(pitch_ > 0 ? -80 : 80);
  //   right_momentum_block_joint_handle_.setCommand(pitch_ > 0 ? -80 : 80);
  //   left_wheel_joint_handle_.setCommand(pitch_ > 0 ? -anti_block_effort_ : anti_block_effort_);
  //   right_wheel_joint_handle_.setCommand(pitch_ > 0 ? -anti_block_effort_ : anti_block_effort_);
  // }
}

void BalanceController::observe(const ros::Time& time, const ros::Duration& period)
{
  float delta_t = period.toSec();
  wr = -right_wheel_joint_handle_.getVelocity() - ins.Gyro[0] +
       right.d_alpha;  // 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
  vrb = wr * wheel_radius_ + right.L0 * right.d_theta * cos(right.theta) + right.d_L0 * sin(right.theta);  // 机体b系的速度

  wl = -left_wheel_joint_handle_.getVelocity() + ins.Gyro[0] +
       left.d_alpha;  // 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
  vlb = wl * wheel_radius_ + left.L0 * left.d_theta * cos(left.theta) + left.d_L0 * sin(left.theta);  // 机体b系的速度
  if(abs(left_wheel_joint_handle_.getVelocity())>0)
  {
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("left_wheel_vel:%f  right_wheel_vel:%f",left_wheel_joint_handle_.getVelocity(),right_wheel_joint_handle_.getVelocity());
ROS_INFO("wl:%f  vlb:%f wr:%f vrb:%f",wl,vlb,wr,vrb );
  }
  aver_v=(vrb-vlb)/2.0f;//取平均
    ins.MotionAccel_n_last[0] = ins.MotionAccel_n[0];
      // 融合加速度计的数据和机体速度
    static float u, k;   // 输入和卡尔曼增益
    static float vel_prior, vel_measure, vel_cov;     // 先验估计、测量、先验协方差
    // 预测
    u = (ins.MotionAccel_n[0] + ins.MotionAccel_n_last[0]) / 2;         // 速度梯形积分
    chassis_move.vel_predict = vel_prior = chassis_move.v_filter + delta_t * u;          // 先验估计
    vel_cov = chassis_move.vel_cov + VEL_PROCESS_NOISE * delta_t;          // 先验协方差

    // 校正
    vel_measure = aver_v;
    k = vel_cov / (vel_cov + VEL_MEASURE_NOISE);            // 卡尔曼增益
    chassis_move.v_filter = vel_prior + k * (vel_measure - vel_prior);    // 后验估计
    chassis_move.vel_cov = (1 - k) * vel_cov;                        // 后验协方差

    BalanceController::mySaturate(&chassis_move.vel_cov, 0.01, 100);       // 协方差限幅

  // 原地自转的过程中v_filter和x_filter应该都是为0

  chassis_move.x_filter += chassis_move.v_filter * delta_t;
  ROS_INFO("observe | v: %f  x: %f", chassis_move.v_filter,chassis_move.x_filter);
}

geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_[5];
  twist.angular.z = x_[6];
  return twist;
}

void BalanceController::mySaturate(float* in, float min, float max)
{
  if (*in < min)
  {
    *in = min;
  }
  else if (*in > max)
  {
    *in = max;
  }
}

void BalanceController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());

// if( imu_dis_time >= 1500)
// {
  tf::Matrix3x3 rotation_matrix;
  tf::Quaternion tf_quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  // 将四元数转换为旋转矩阵
  rotation_matrix.setRotation(tf_quaternion);

  double p_, r_, y_;
  // 从旋转矩阵中提取欧拉角
  rotation_matrix.getRPY(p_, r_, y_);
  ins.Pitch = p_;
  ins.Roll = r_;
  ins.Yaw = y_;

  if (ins.Yaw - ins.YawAngleLast > 3.1415926f)
  {
    ins.YawRoundCount--;
  }
  else if (ins.Yaw - ins.YawAngleLast < -3.1415926f)
  {
    ins.YawRoundCount++;
  }
  ins.YawTotalAngle = 6.283f * ins.YawRoundCount + ins.Yaw;
  ins.YawAngleLast = ins.Yaw;

  ins.Gyro[0] = msg->angular_velocity.x;
  ins.Gyro[1] = msg->angular_velocity.y;
  ins.Gyro[2] = msg->angular_velocity.z;

  ins.Accel[0] = msg->linear_acceleration.y; //imu成品输出为绝对坐标系 加速度
  ins.Accel[0] = msg->linear_acceleration.z;
  ins.Accel[0] = msg->linear_acceleration.x;

  ins.MotionAccel_n[0] = msg->linear_acceleration.y;
  ins.MotionAccel_n[1] = msg->linear_acceleration.z;
  ins.MotionAccel_n[2] = msg->linear_acceleration.x;
  // ins.q[0] = tf_quaternion[0];
  // ins.q[1] = tf_quaternion[1];
  // ins.q[2] = tf_quaternion[2];
  // ins.q[3] = tf_quaternion[3];
  // // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
  // float gravity_b[3];
  // EarthFrameToBodyFrame(gravity, gravity_b, ins.q);
  // for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
  // {
  //   ins.MotionAccel_b[i] = (ins.Accel[i] - gravity_b[i]) * ins_dt / (0.0089f + ins_dt)
  // 												+ ins.MotionAccel_b[i] * 0.0089f / (0.0089f + ins_dt);
  // // 			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt)
  // // 														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
  // }
  // BodyFrameToEarthFrame(ins.MotionAccel_b, ins.MotionAccel_n, ins.q);  // 转换回导航系n

  // 死区处理
  if (fabsf(ins.MotionAccel_n[0]) < 0.05f)
  {
    ins.MotionAccel_n[0] = 0.0f;  // x轴
  }
  if (fabsf(ins.MotionAccel_n[1]) < 0.05f)
  {
    ins.MotionAccel_n[1] = 0.0f;  // y轴
  }
  if (fabsf(ins.MotionAccel_n[2]) < 0.04f)
  {
    ins.MotionAccel_n[2] = 0.0f;  // z轴
  }
// }
// else
// {
//   imu_dis_time++;
// }
  // ROS_INFO("init_imu || ph: %f | yaw: %f | roll: %f | YawRoundCount: %f | ph_d: %f | yaw_d: %f | roll_d: %f |",
  // ins.Pitch ,ins.Yaw,ins.Roll,ins.YawRoundCount,ins.Gyro[0],ins.Gyro[1],ins.Gyro[2]);
}

void BalanceController::VMC_init(vmc_leg_t* vmc, double l1, double l2, double l3, double l4, double l5)  // 给杆长赋值
{
  vmc->l5 = l5;  // AE长度 //单位为m
  vmc->l1 = l1;  // 单位为m
  vmc->l2 = l2;  // 单位为m
  vmc->l3 = l3;  // 单位为m
  vmc->l4 = l4;  // 单位为m
}

void BalanceController::motor_init()
{
  for (int j = 0; j < 10; j++)
  {
    right_front_joint_handle_.setCommand(21);
  }
  for (int j = 0; j < 10; j++)
  {
    right_back_joint_handle_.setCommand(21);
  }
  // ROS_INFO("motor init >> ");
  for (int j = 0; j < 10; j++)
  {
    left_front_joint_handle_.setCommand(21);
  }
  for (int j = 0; j < 10; j++)
  {
    left_back_joint_handle_.setCommand(21);
  }
}

void BalanceController::motor_send()
{
  // ros::Duration d(0.001);
  // if(abs(left.T_front) < 20)
  // left_front_joint_handle_.setCommand(left.T_back);
  // d.sleep();
  // if(abs(left.T_back) < 20)
  // left_back_joint_handle_.setCommand(left.T_front);
  // d.sleep();

  // right_front_joint_handle_.setCommand(-right.T_back);
  // d.sleep();
  // right_back_joint_handle_.setCommand(-right.T_front);
  // d.sleep();

  // left_wheel_joint_handle_.setCommand(chassis_move.wheel_motor[1].wheel_T);
  // d.sleep();
  // right_wheel_joint_handle_.setCommand(-chassis_move.wheel_motor[0].wheel_T);
  ROS_INFO("RESULT :  %f | %f | %f | %f ", left.T_back, left.T_front, right.T_front,
           right.T_back);
  ROS_INFO("RESULT  wheel  :  %f | %f  ", chassis_move.wheel_motor[1].wheel_T, chassis_move.wheel_motor[0].wheel_T);
}

void BalanceController::VMC_calc_1_right(vmc_leg_t* vmc, INS_t* ins,
                                         float dt)  // 计算theta和d_theta给lqr用，同时也计算腿长L0
{
  static float PitchR = 0.0f;
  static float PithGyroR = 0.0f;
  PitchR = chassis_move.myPith;
  PithGyroR = chassis_move.myPithGyro;
  // ROS_INFO("VMC R chassis: PithR %f | PithGyroR %f", PitchR, PithGyroR);
  vmc->YD = vmc->l4 * sin(vmc->phi4);            // D的y坐标
  vmc->YB = vmc->l1 * sin(vmc->phi1);            // B的y坐标
  vmc->XD = vmc->l5 + vmc->l4 * cos(vmc->phi4);  // D的x坐标
  vmc->XB = vmc->l1 * cos(vmc->phi1);            // B的x坐标

  vmc->lBD = sqrt((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

  vmc->A0 = 2 * vmc->l2 * (vmc->XD - vmc->XB);
  vmc->B0 = 2 * vmc->l2 * (vmc->YD - vmc->YB);
  vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;
  vmc->phi2 =
      2 * atan2f((vmc->B0 + sqrt(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), vmc->A0 + vmc->C0);
  vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * sin(vmc->phi2), vmc->XB - vmc->XD + vmc->l2 * cos(vmc->phi2));
  // C点直角坐标
  vmc->XC = vmc->l1 * cos(vmc->phi1) + vmc->l2 * cos(vmc->phi2);
  vmc->YC = vmc->l1 * sin(vmc->phi1) + vmc->l2 * sin(vmc->phi2);
  // C点极坐标
  vmc->L0 = sqrt((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);

  vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f));  // phi0用于计算lqr需要的theta
  vmc->alpha = pi / 2.0f - vmc->phi0;

  if (vmc->first_flag == 0)
  {
    vmc->last_phi0 = vmc->phi0;
    vmc->first_flag = 1;
  }
  vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;  // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
  vmc->d_alpha = 0.0f - vmc->d_phi0;

  vmc->theta = pi / 2.0f - PitchR - vmc->phi0;  // 得到状态变量1
  vmc->d_theta = (-PithGyroR - vmc->d_phi0);    // 得到状态变量2

  vmc->last_phi0 = vmc->phi0;

  vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;       // 腿长L0的一阶导数
  vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;  // 腿长L0的二阶导数

  vmc->last_d_L0 = vmc->d_L0;
  vmc->last_L0 = vmc->L0;

  vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
  vmc->last_d_theta = vmc->d_theta;
  // ROS_INFO("VMC R  : L0 %f | theta %f", vmc->L0, vmc->theta);
}

void BalanceController::VMC_calc_1_left(vmc_leg_t* vmc, INS_t* ins,
                                        float dt)  // 计算theta和d_theta给lqr用，同时也计算腿长L0
{
  static float PitchL = 0.0f;
  static float PithGyroL = 0.0f;
  PitchL = -chassis_move.myPith;  // - 机体  （机体等于右边）
  PithGyroL = -chassis_move.myPithGyro;
  // ROS_INFO("VMC L chassis: PithL %f | PithGyroL %f", PitchL, PithGyroL);
  vmc->YD = vmc->l4 * sin(vmc->phi4);            // D的y坐标
  vmc->YB = vmc->l1 * sin(vmc->phi1);            // B的y坐标
  vmc->XD = vmc->l5 + vmc->l4 * cos(vmc->phi4);  // D的x坐标
  vmc->XB = vmc->l1 * cos(vmc->phi1);            // B的x坐标

  vmc->lBD = sqrt((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

  vmc->A0 = 2 * vmc->l2 * (vmc->XD - vmc->XB);
  vmc->B0 = 2 * vmc->l2 * (vmc->YD - vmc->YB);
  vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;
  vmc->phi2 =
      2 * atan2f((vmc->B0 + sqrt(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), vmc->A0 + vmc->C0);
  vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * sin(vmc->phi2), vmc->XB - vmc->XD + vmc->l2 * cos(vmc->phi2));
  // C点直角坐标
  vmc->XC = vmc->l1 * cos(vmc->phi1) + vmc->l2 * cos(vmc->phi2);
  vmc->YC = vmc->l1 * sin(vmc->phi1) + vmc->l2 * sin(vmc->phi2);
  // C点极坐标
  vmc->L0 = sqrt((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);

  vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f));  // phi0用于计算lqr需要的theta
  vmc->alpha = pi / 2.0f - vmc->phi0;

  if (vmc->first_flag == 0)
  {
    vmc->last_phi0 = vmc->phi0;
    vmc->first_flag = 1;
  }
  vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;  // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
  vmc->d_alpha = 0.0f - vmc->d_phi0;

  vmc->theta = pi / 2.0f - PitchL - vmc->phi0;  // 得到状态变量1
  vmc->d_theta = (-PithGyroL - vmc->d_phi0);    // 得到状态变量2

  vmc->last_phi0 = vmc->phi0;

  vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;       // 腿长L0的一阶导数
  vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;  // 腿长L0的二阶导数

  vmc->last_d_L0 = vmc->d_L0;
  vmc->last_L0 = vmc->L0;

  vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
  vmc->last_d_theta = vmc->d_theta;

  ROS_INFO("VMC L  : ph0 %f | ph1 %f | ph2 %f | ph3 %f  ph4 %f", vmc->phi0, vmc->phi1, vmc->phi2, vmc->phi3, vmc->phi4);
  ROS_INFO("VMC L  : L0 %f | theta %f | theta_d %f", vmc->L0, vmc->theta, vmc->d_theta);
}

void BalanceController::VMC_calc_2(vmc_leg_t* vmc)  // 计算期望的关节输出力矩
{
    // float phi12 = vmc->phi1 - vmc->phi2;
    // float phi34 = vmc->phi3 - vmc->phi4;
    // float phi32 = vmc->phi3 - vmc->phi2;
    // float phi53 = vmc->phi0 - vmc->phi3;
    // float phi52 = vmc->phi0 - vmc->phi2;
    // float F_m_L = vmc->F0 * vmc->L0;
    // vmc->T_front = (vmc->l1  * sin(phi12) * (F_m_L * sin(phi53) + vmc->Tp * cos(phi53))) / (vmc->L0 * sin(phi32));
    // vmc->T_back = (vmc->l4  * sin(phi34) * (F_m_L * sin(phi52) + vmc->Tp * cos(phi52))) / (vmc->L0 * sin(phi32));


  vmc->j11 = (vmc->l1 * sin(vmc->phi0 - vmc->phi3) * sin(vmc->phi1 - vmc->phi2)) / sin(vmc->phi3 - vmc->phi2);
  vmc->j12 =
      (vmc->l1 * cos(vmc->phi0 - vmc->phi3) * sin(vmc->phi1 - vmc->phi2)) / (vmc->L0 * sin(vmc->phi3 - vmc->phi2));
  vmc->j21 = (vmc->l4 * sin(vmc->phi0 - vmc->phi2) * sin(vmc->phi3 - vmc->phi4)) / sin(vmc->phi3 - vmc->phi2);
  vmc->j22 =
      (vmc->l4 * cos(vmc->phi0 - vmc->phi2) * sin(vmc->phi3 - vmc->phi4)) / (vmc->L0 * sin(vmc->phi3 - vmc->phi2));

  vmc->T_front =
      vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp;  // 得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力
  vmc->T_back = vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp;  // 得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩

// ROS_INFO("test %f" , vmc->j11);
  // ROS_INFO("j12_  : j11: %f | j12: %f | j21: %f  | yy: %f ", vmc->l1 ,vmc->phi0 , vmc->phi3 ,cos(vmc->phi0-vmc->phi3)
  // ); ROS_INFO("j12   : j11: %f | j12: %f | j21: %f  ", vmc->l1*cos(vmc->phi0-vmc->phi3)
  // ,sin(vmc->phi1-vmc->phi2),vmc->L0*sin(vmc->phi3-vmc->phi2)); ROS_INFO("test : j11: %f | j12: %f | j21: %f | j22: %f
  // | f0: %f | fp: %f ", vmc->j11 ,vmc->j12,vmc->j21,vmc->j22,vmc->F0,vmc->Tp);

  // ROS_INFO("j22 : j22: %f | j22_: %f | j22__: %f ", vmc->torque_set[1],vmc->j21*vmc->F0,vmc->j22*vmc->Tp);
}

uint8_t BalanceController::ground_detectionR(vmc_leg_t* vmc, INS_t* ins)
{
  vmc->FN = vmc->F0 * acos(vmc->theta) + vmc->Tp * asin(vmc->theta) / vmc->L0 +
            6.0f;  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  //	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
  //+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

  if (vmc->FN < 5.0f)
  {  // 离地了

    return 1;
  }
  else
  {
    return 0;
  }
}

uint8_t BalanceController::ground_detectionL(vmc_leg_t* vmc, INS_t* ins)
{
  vmc->FN = vmc->F0 * cos(vmc->theta) + vmc->Tp * sin(vmc->theta) / vmc->L0 +
            6.0f;  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  //	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
  //+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

  if (vmc->FN < 5.0f)
  {  // 离地了
    return 1;
  }
  else
  {
    return 0;
  }
}

float BalanceController::LQR_K_calc(float* coe, float len)
{
  return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)

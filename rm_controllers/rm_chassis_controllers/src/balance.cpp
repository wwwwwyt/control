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

  debug_pub_ = controller_nh.advertise<rm_msgs::DebugMsg>("/debug_", 1);
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

  VMC_init(left, l1_, l2_, l3_, l4_, l5_);  // 给杆长赋值
  VMC_init(right, l1_, l2_, l3_, l4_, l5_);

  state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::BalanceState>(root_nh, "/state", 100));
  // balance_mode_ = BalanceMode::NORMAL;

  return true;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  ROS_INFO("time_dit : %f", period.toSec());

  BalanceController::balance_init(period);//参数初始化

  VMC_calc_1(left,  &ins,period.toSec());//将五连杆映射成单杆 运动学
  VMC_calc_1(right, &ins,period.toSec()); 

  BalanceController::observe(time, period);//通过卡尔曼滤波估计机体速度

  BalanceController::CalcLQR(&left);//lqr计算
  BalanceController::CalcLQR(&right);

  BalanceController::SynthesizeMotion(period);//转向和抗劈叉

  BalanceController::LegControl(period);//腿长控制和Roll补偿

  VMC_calc_2(left);//VMC映射成关节输出
  VMC_calc_2(right);

  // BalanceController::ground_detection(&left,&ins);//驱动轮支持力解算
  // BalanceController::ground_detection(&right,&ins);

  debug_publier_.acc_n = ins.MotionAccel_n[0];
  debug_publier_.observe_x = chassis_move.x_filter;
  debug_publier_.observe_v = chassis_move.v_filter;
  ROS_INFO("observe : x v %f | %f", chassis_move.x_filter,chassis_move.v_filter );
  debug_pub_.publish(debug_publier_);

  if(cmd_rt_buffer_.readFromRT()->cmd_chassis_.mode == 1)
  {
    if (motor_init_)
    {
      BalanceController::motor_init();
      motor_init_ = 0;
    }
    // BalanceController::motor_send();
    ROS_INFO("------------------------------------------");
  }
  else
  {
    BalanceController::motor_disability();
    motor_init_ = 1;
  }
}

void  BalanceController::balance_init(const ros::Duration& period)
{
  // 更新数据
  left.phi1 = 3.804817886111111 - left_front_joint_handle_.getPosition(); // 38 以机械上限位角度为零点
  left.phi4 = -left_back_joint_handle_.getPosition() - 0.663225136111111;
  right.phi1 = 3.804817886111111 + right_front_joint_handle_.getPosition();
  right.phi4 = right_back_joint_handle_.getPosition() - 0.663225136111111;

  chassis_move.v_set = vel_cmd_.x;
  chassis_move.x_set += vel_cmd_.x * period.toSec();
  // 更新机体imu数值
  chassis_move.myPith = 0.0f - ins.Pitch;
  chassis_move.myPithGyro = -ins.Gyro[0];
  chassis_move.total_yaw = ins.YawTotalAngle;
  chassis_move.roll = ins.Roll;
  chassis_move.theta_err = left.theta - right.theta;
  // if(abs(chassis_move.myPithGyro)<0.05) chassis_move.myPithGyro = 0;
  // if(abs(chassis_move.myPith)<0.1) chassis_move.myPith = 0;
  chassis_move.roll_lenth = Roll_Pid.computeCommand(ins.Roll, period);  // roll 补偿
  left.Pitch = chassis_move.myPith;
  left.PithGyro = chassis_move.myPithGyro;
  right.Pitch = chassis_move.myPith;
  right.PithGyro = chassis_move.myPithGyro;

    // ROS_INFO("test_num || lf: %f | lb: %f | rf: %f | rb: %f ", left_front_joint_handle_.getPosition(),
    //        left_back_joint_handle_.getPosition(), right_front_joint_handle_.getPosition(),
    //        right_back_joint_handle_.getPosition());
  // ROS_INFO("init_leg || left_ph1: %f | left_ph4: %f | right_ph1: %f | right_ph4: %f ", left.phi1, left.phi4, right.phi1,
          //  right.phi4);
  // ROS_INFO("init_leg || left_ph1: %f ", right_wheel_joint_handle_.getVelocity());
  // ROS_INFO("init_chassis || chassis_P: %f", right.Pitch);
    // ROS_INFO("init_leg || pitch : %f pitch_d: %f ", chassis_move.myPith, chassis_move.myPithGyro);
}

void BalanceController::CalcLQR(vmc_leg_t* vmc_)
{
  // for (int i = 0; i < 12; i++)
  // {
  //   LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmc_->L0);
  // }

  chassis_move.wheel_motor[1].wheel_T = (LQR_K[0] * (vmc_->theta - 0.0f) +
                                     LQR_K[1] * (vmc_->d_theta - 0.0f)
                                    //  +LQR_K[2]*(chassis_move->x_filter - chassis_move->x_set)
                                    //  +LQR_K[3]*(chassis_move->v_filter - chassis_move->v_set)
                                     +LQR_K[2]*(0.0f - chassis_move.x_set)
                                     +LQR_K[3]*(0.0f - chassis_move.v_set)
                                     + LQR_K[4] * (0.0f - chassis_move.myPith) + LQR_K[5] * (0.0f - chassis_move.myPithGyro));
  //髋关节输出力矩
  // vmc_->Tp=(LQR_K[6]*(vmc_->theta-0.0f)
  // 				+LQR_K[7]*(vmc_->d_theta-0.0f)
          // +LQR_K[8]*(chassis_move->x_filter - chassis_move->x_set)
          // +LQR_K[9]*(chassis_move->v_filter - chassis_move->v_set)
          // +LQR_K[8]*(0.0f - chassis_move.x_set)
          // +LQR_K[9]*(0.0f - chassis_move.v_set)   
  // 				+LQR_K[10]*(0.0f - chassis_move->myPith)
  // 				+LQR_K[11]*(0.0f - chassis_move->myPithGyro)
  //         );  
}

void BalanceController::SynthesizeMotion(const ros::Duration& period_)
{
  // if (chassis_cmd_recv.chassis_mode == CHASSIS_FREE_DEBUG ||
  //     chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) // 底盘跟随
  // {
  //     float p_ref = PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
  //     PIDCalculate(&steer_v_pid, chassis.wz, p_ref);
  // }
  // else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) // 小陀螺
  // {
  //     PIDCalculate(&steer_v_pid, chassis.wz, (float)chassis_cmd_recv.rotate_w);
  // }
  // else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE_REVERSE)
  // {
  //     PIDCalculate(&steer_v_pid, chassis.wz, (float)chassis_cmd_recv.rotate_w);
  // }
  // double p_,i_,d_,i_max_,i_min_;
  // Turn_Pid.getGains(p_,i_,d_,i_max_,i_min_);
  // chassis_move.turn_T=p_*(vel_cmd_.z-chassis_move.total_yaw)-d_*ins.Gyro[2];//这样计算更稳一点
  // chassis_move.wheel_motor[1].wheel_T -= chassis_move.turn_T;
  // chassis_move.wheel_motor[0].wheel_T += chassis_move.turn_T;

  BalanceController::mySaturate(&chassis_move.wheel_motor[1].wheel_T, -9.6f, 9.6f);
  BalanceController::mySaturate(&chassis_move.wheel_motor[0].wheel_T, -9.6f, 9.6f);
  
  // 抗劈叉
  // static float swerving_speed_ff, ff_coef = 3;
  // swerving_speed_ff = ff_coef * steer_v_pid.Output; // 用于抗劈叉的前馈
  // PIDCalculate(&anti_crash_pid, l_side.phi5 - r_side.phi5, 0);
  chassis_move.leg_tp=Tp_Pid.computeCommand( chassis_move.theta_err , period_);//防劈叉pid计算    
  left.Tp += chassis_move.leg_tp;
  right.Tp -= chassis_move.leg_tp;
}

void BalanceController::LegControl(const ros::Duration& period_)
{
  left.F0 = 0.0f + pid_l_.computeCommand(chassis_move.leg_set - left.L0, period_);
  right.F0 = 0.0f + pid_r_.computeCommand(chassis_move.leg_set - right.L0, period_);

  BalanceController::mySaturate(&left.F0, -600.0f, 600.0f);  // 限幅
  BalanceController::mySaturate(&right.F0, -600.0f, 600.0f);  // 限幅
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
  wr = right_wheel_joint_handle_.getVelocity() - ins.Gyro[0] +
       right.d_alpha;  // 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
  vrb = wr * wheel_radius_ + right.L0 * right.d_theta * cos(right.theta) +
        right.d_L0 * sin(right.theta);  // 机体b系的速度

  wl = -left_wheel_joint_handle_.getVelocity() + ins.Gyro[0] +
       left.d_alpha;  // 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
  vlb = wl * wheel_radius_ + left.L0 * left.d_theta * cos(left.theta) + left.d_L0 * sin(left.theta);  // 机体b系的速度
  // ROS_INFO("wr_speed :%f  wl_speed :%f   %f", right_wheel_joint_handle_.getVelocity(), left_wheel_joint_handle_.getVelocity(), ins.Gyro[0]);
  // ROS_INFO("robo_gyro :%f ",ins.Gyro[0] );  
  // ROS_INFO("wl:  vlb: wr: vrb: %f %f %f %f", wl, vlb, wr, vrb);

  aver_v = (vrb - vlb) / 2.0f;  // 取平均
  // ROS_INFO("aver_v:%f",aver_v);
  // ins.MotionAccel_n[0] *= cos(chassis_move.myPith);
  
  ROS_INFO("acc :%f  acc_last :%f", ins.MotionAccel_n[0], ins.MotionAccel_n_last[0]);
  // 融合加速度计的数据和机体速度
  static float u, k;                             // 输入和卡尔曼增益
  static float vel_prior, vel_measure, vel_cov;  // 先验估计、测量、先验协方差
  // 预测
  u = (ins.MotionAccel_n[0] + ins.MotionAccel_n_last[0]) / 2;                  // 速度梯形积分
  chassis_move.vel_predict = vel_prior = chassis_move.v_filter + delta_t * u;  // 先验估计
  vel_cov = chassis_move.vel_cov + VEL_PROCESS_NOISE * delta_t;                // 先验协方差

  // 校正
  vel_measure = aver_v;
  k = vel_cov / (vel_cov + VEL_MEASURE_NOISE);                        // 卡尔曼增益
  chassis_move.v_filter = vel_prior + k * (vel_measure - vel_prior);  // 后验估计
  chassis_move.vel_cov = (1 - k) * vel_cov;                           // 后验协方差

  BalanceController::mySaturate(&chassis_move.vel_cov, 0.01, 100);  // 协方差限幅

  // 原地自转的过程中v_filter和x_filter应该都是为0



  // 速度和位置分离，有速度输入时不进行位置闭环
  if(abs(chassis_move.v_set) < 0.009)
  {
    chassis_move.x_set = 0;
    chassis_move.x_filter += chassis_move.v_filter * delta_t;
  }
  else
    chassis_move.x_set = chassis_move.x_filter = 0;
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

  ins.Accel[0] = msg->linear_acceleration.y;  // imu成品输出为绝对坐标系 加速度
  ins.Accel[1] = -msg->linear_acceleration.x;
  ins.Accel[2] = msg->linear_acceleration.z;

  ins.MotionAccel_n_last[0] = ins.MotionAccel_n[0];

  ins.MotionAccel_n[0] = msg->linear_acceleration.y;
  ins.MotionAccel_n[1] = msg->linear_acceleration.z;
  ins.MotionAccel_n[2] = msg->linear_acceleration.x;


  tf_quaternion = tf::createQuaternionFromRPY(y_,p_, r_);
    // ROS_INFO("aa : %f , bb: %f , cc:%f",p_, y_,r_); 
  ins.q[0] = tf_quaternion[0];
  ins.q[1] = tf_quaternion[1];
  ins.q[2] = tf_quaternion[2];
  ins.q[3] = tf_quaternion[3];
  // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
  float gravity_b[3];
  EarthFrameToBodyFrame(gravity, gravity_b, ins.q);
  for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
  {
    ins.MotionAccel_b[i] = (ins.Accel[i] - gravity_b[i]) * ins_dt / (0.0089f + ins_dt)
  												+ ins.MotionAccel_b[i] * 0.0089f / (0.0089f + ins_dt);
  // 			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt)
  // 														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
  }
  // ROS_INFO("aa : %f , bb: %f , cc:%f",ins.Accel[0], ins.Accel[1], ins.Accel[2]);  
  // ROS_INFO("a : %f , b: %f , c:%f",gravity_b[0], gravity_b[1], gravity_b[2]);
  BodyFrameToEarthFrame(ins.MotionAccel_b, ins.MotionAccel_n, ins.q);  // 转换回导航系n

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


void BalanceController::motor_init()
{
  for (int j = 0; j < 1000; j++)
  {
    right_front_joint_handle_.setCommand(21);
  }
  for (int j = 0; j < 1000; j++)
  {
    right_back_joint_handle_.setCommand(21);
  }
  // ROS_INFO("motor init >> ");
  for (int j = 0; j < 1000; j++)
  {
    left_front_joint_handle_.setCommand(21);
  }
  for (int j = 0; j < 1000; j++)
  {
    left_back_joint_handle_.setCommand(21);
  }
}

void BalanceController::motor_send()
{
  ros::Duration d(0.001);
  if(abs(left.T_back) < 20)
  left_front_joint_handle_.setCommand(-left.T_front);;
  d.sleep();
  if(abs(left.T_front) < 20)
  left_back_joint_handle_.setCommand(-left.T_back);
  d.sleep();

  right_front_joint_handle_.setCommand(right.T_front);
  d.sleep();
  right_back_joint_handle_.setCommand(right.T_back);
  d.sleep();

  left_wheel_joint_handle_.setCommand(chassis_move.wheel_motor[1].wheel_T);
  d.sleep();
  right_wheel_joint_handle_.setCommand(chassis_move.wheel_motor[0].wheel_T);

  ROS_INFO("RESULT :  %f | %f | %f | %f ", left.T_back, left.T_front, right.T_back, right.T_front);
  ROS_INFO("RESULT wheel: %f | %f", chassis_move.wheel_motor[1].wheel_T, chassis_move.wheel_motor[0].wheel_T);
}

void BalanceController::motor_disability()
{
  ros::Duration d(0.001);
  left_front_joint_handle_.setCommand(-21);;
  d.sleep();
  left_back_joint_handle_.setCommand(-21);
  d.sleep();

  right_front_joint_handle_.setCommand(-21);
  d.sleep();
  right_back_joint_handle_.setCommand(-21);
  d.sleep();

}

uint8_t BalanceController::ground_detection(vmc_leg_t* vmc, INS_t* ins)
{
  vmc->FN = vmc->F0 * cos(vmc->theta) + vmc->Tp * sin(vmc->theta) / vmc->L0 +
            6.0f;  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  //	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
  //+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

  if (vmc->FN < 20.0f)
  {  // 离地了
    return 1;
  }
  else
  {
    return 0;
  }
}


}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)

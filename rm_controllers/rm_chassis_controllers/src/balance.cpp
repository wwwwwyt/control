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
  //leg init

  //imu 
  // imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
  //     getParam(controller_nh, "imu_name", std::string("base_imu")));
  imu_sub_ = controller_nh.subscribe("imu", 1000, &BalanceController::imuCallback, this);
  //    
  std::string left_wheel_joint, right_wheel_joint, left_front_joint, right_front_joint, left_back_joint, right_back_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("left/block_joint", left_front_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint) ||
      !controller_nh.getParam("right/block_joint", right_front_joint) ||
      !controller_nh.getParam("right/wheel_joint", left_back_joint) ||
      !controller_nh.getParam("right/block_joint", right_back_joint)      )
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  
  //关节控制器 句柄 
  // 左前 左后 
  left_front_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_front_joint);
  left_back_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_back_joint);
  //右
  right_front_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_front_joint);
  right_back_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_back_joint);
  // 俩轮
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);


  // const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	// joint_motor_init(&chassis->joint_motor[2],6,MIT_MODE);//发送id为6
	// joint_motor_init(&chassis->joint_motor[3],8,MIT_MODE);//发送id为8
	
	// wheel_motor_init(&chassis->wheel_motor[1],1,MIT_MODE);//发送id为1
	
	VMC_init(&left);//给杆长赋值
	VMC_init(&right);

	// PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

//电机使能帧 4关节加2轮毂
	// for(int j=0;j<10;j++)
	// {
	//   enable_motor_mode(&hfdcan2,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	//   // osDelay(1);
	// }
	// for(int j=0;j<10;j++)
	// {
	//   enable_motor_mode(&hfdcan2,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	//   osDelay(1);
	// }
	// for(int j=0;j<10;j++)
	// {
  //     enable_motor_mode(&hfdcan2,chassis->wheel_motor[1].para.id,chassis->wheel_motor[1].mode);//左边轮毂电机
	//   osDelay(1);
	// }

  state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::BalanceState>(root_nh, "/state", 100));
  balance_mode_ = BalanceMode::NORMAL;

  return true;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  //更新数据
  left.phi1=pi/2.0f+left_front_joint_handle_.getAbsolutePosition();
	left.phi4=pi/2.0f+left_back_joint_handle_.getAbsolutePosition();
  right.phi1=pi/2.0f+right_front_joint_handle_.getAbsolutePosition();
	right.phi4=pi/2.0f+right_back_joint_handle_.getAbsolutePosition();
  //更新机体imu数值	
	chassis_move.myPithL=0.0f-ins.Pitch;
	chassis_move.myPithGyroL=0.0f-ins.Gyro[0];

  balanceL_control_loop(&chassis_move,&left,&ins,LQR_K);//控制计算
			
		if(chassis_move.start_flag==1)	
		{
        // 左前 左后 
      left_front_joint_handle_.setCommand(right.torque_set[1]);
      left_back_joint_handle_ .setCommand(right.torque_set[1]);
      //右
      right_front_joint_handle_ .setCommand(right.torque_set[1]);
      right_back_joint_handle_.setCommand(right.torque_set[0]);
      // 俩轮
      left_wheel_joint_handle_ .setCommand(chassis_move.wheel_motor[1].wheel_T);
      right_wheel_joint_handle_ .setCommand(chassis_move.wheel_motor[0].wheel_T);
		}
		else if(chassis_move.start_flag==0)	
		{
        // 左前 左后 
      left_front_joint_handle_.setCommand(0);
      left_back_joint_handle_ .setCommand(0);
      //右
      right_front_joint_handle_ .setCommand(0);
      right_back_joint_handle_.setCommand(0);
      // 俩轮
      left_wheel_joint_handle_ .setCommand(0);
      right_wheel_joint_handle_ .setCommand(0);
		}

  // geometry_msgs::Vector3 gyro;
  // gyro.x = imu_handle_.getAngularVelocity()[0];
  // gyro.y = imu_handle_.getAngularVelocity()[1];
  // gyro.z = imu_handle_.getAngularVelocity()[2];
  // try
  // {
  //   tf2::doTransform(gyro, angular_vel_base_,
  //                    robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
  // }
  // catch (tf2::TransformException& ex)
  // {
  //   ROS_WARN("%s", ex.what());
  //   return;
  // }
  // tf2::Transform odom2imu, imu2base, odom2base;
  // try
  // {
  //   geometry_msgs::TransformStamped tf_msg;
  //   tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
  //   tf2::fromMsg(tf_msg.transform, imu2base);
  // }
  // catch (tf2::TransformException& ex)
  // {
  //   ROS_WARN("%s", ex.what());
  //   left_wheel_joint_handle_.setCommand(0.);
  //   right_wheel_joint_handle_.setCommand(0.);
  //   left_momentum_block_joint_handle_.setCommand(0.);
  //   right_momentum_block_joint_handle_.setCommand(0.);
  //   return;
  // }
  // tf2::Quaternion odom2imu_quaternion;
  // tf2::Vector3 odom2imu_origin;
  // odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
  //                              imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
  // odom2imu_origin.setValue(0, 0, 0);
  // odom2imu.setOrigin(odom2imu_origin);
  // odom2imu.setRotation(odom2imu_quaternion);
  // odom2base = odom2imu * imu2base;

  // quatToRPY(toMsg(odom2base).rotation, roll_, pitch_, yaw_);

  // // Check block
  // if (balance_mode_ != BalanceMode::BLOCK)
  // {
  //   if (std::abs(pitch_) > block_angle_ &&
  //       (std::abs(left_wheel_joint_handle_.getEffort()) + std::abs(right_wheel_joint_handle_.getEffort())) / 2. >
  //           block_effort_ &&
  //       (left_wheel_joint_handle_.getVelocity() < block_velocity_ ||
  //        right_wheel_joint_handle_.getVelocity() < block_velocity_))
  //   {
  //     if (!maybe_block_)
  //     {
  //       block_time_ = time;
  //       maybe_block_ = true;
  //     }
  //     if ((time - block_time_).toSec() >= block_duration_)
  //     {
  //       balance_mode_ = BalanceMode::BLOCK;
  //       balance_state_changed_ = true;
  //       ROS_INFO("[balance] Exit NOMAl");
  //     }
  //   }
  //   else
  //   {
  //     maybe_block_ = false;
  //   }
  // }

  // switch (balance_mode_)
  // {
  //   case BalanceMode::NORMAL:
  //   {
  //     normal(time, period);
  //     break;
  //   }
  //   case BalanceMode::BLOCK:
  //   {
  //     block(time, period);
  //     break;
  //   }
  // }
}

void BalanceController::balanceL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K)
{
	VMC_calc_1_left(vmcl,ins,((float)1)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
	}
			
	chassis->wheel_motor[1].wheel_T=(LQR_K[0]*(vmcl->theta-0.0f)
																	+LQR_K[1]*(vmcl->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_set-chassis->x_filter)
																	+LQR_K[3]*(chassis->v_set-chassis->v_filter)
																	+LQR_K[4]*(chassis->myPithL-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroL-0.0f));
	
	//右边髋关节输出力矩				
	vmcl->Tp=(LQR_K[6]*(vmcl->theta-0.0f)
					+LQR_K[7]*(vmcl->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_set-chassis->x_filter)
					+LQR_K[9]*(chassis->v_set-chassis->v_filter)
					+LQR_K[10]*(chassis->myPithL-0.0f)
					+LQR_K[11]*(chassis->myPithGyroL-0.0f));
	 		
	chassis->wheel_motor[1].wheel_T= chassis->wheel_motor[1].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
	mySaturate(&chassis->wheel_motor[1].wheel_T,-1.0f,1.0f);	
	
	vmcl->Tp=vmcl->Tp+chassis->leg_tp;//髋关节输出力矩
	
	// vmcl->F0=13.0f+pid_l_.computeCommand(vmcl->L0,chassis->leg_set);//前馈+pd
  	
	left_flag=ground_detectionL(vmcl,ins);//左腿离地检测
	
	 if(chassis->recover_flag==0)	
	 {//倒地自起不需要检测是否离地
		if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			chassis->wheel_motor[1].wheel_T=0.0f;
			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);
			
			chassis->x_filter=0.0f;//对位移清零
			chassis->x_set=chassis->x_filter;
			chassis->turn_set=chassis->total_yaw;
			vmcl->Tp=vmcl->Tp+chassis->leg_tp;		
		}
		else
		{//没有离地
			vmcl->leg_flag=0;//置为0			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcl->Tp=0.0f;
	 }
	
	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅 
	
	VMC_calc_2(vmcl);//计算期望的关节输出力矩
	
  //额定扭矩
  mySaturate(&vmcl->torque_set[1],-3.0f,3.0f);	
	mySaturate(&vmcl->torque_set[0],-3.0f,3.0f);	
			
}

void BalanceController::balanceR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K)
{
	VMC_calc_1_right(vmcr,ins,((float)1)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
	}
		
	//chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
  // double p{0};
  // Turn_Pid.getGains(&p);
  // chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点

	// chassis->leg_tp=Tp_Pid.computeCommand( -chassis->theta_err);//防劈叉pid计算
	
	chassis->wheel_motor[0].wheel_T=(LQR_K[0]*(vmcr->theta-0.0f)
																	+LQR_K[1]*(vmcr->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_filter-chassis->x_set)
																	+LQR_K[3]*(chassis->v_filter-chassis->v_set)
																	+LQR_K[4]*(chassis->myPithR-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroR-0.0f));
	
	//右边髋关节输出力矩				
	vmcr->Tp=(LQR_K[6]*(vmcr->theta-0.0f)
					+LQR_K[7]*(vmcr->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_filter-chassis->x_set)
					+LQR_K[9]*(chassis->v_filter-chassis->v_set)
					+LQR_K[10]*(chassis->myPithR-0.0f)
					+LQR_K[11]*(chassis->myPithGyroR-0.0f));
				
	chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
	mySaturate(&chassis->wheel_motor[0].wheel_T,-1.0f,1.0f);	
	
	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//髋关节输出力矩

	// vmcr->F0=13.0f+pid_r_.computeCommand(chassis->leg_set-vmcr->L0);//前馈+pd
		
	right_flag=ground_detectionR(vmcr,ins);//右腿离地检测
	 
	 if(chassis->recover_flag==0)		
	 {//倒地自起不需要检测是否离地	 
		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set=chassis->x_filter;
				chassis->turn_set=chassis->total_yaw;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;		
		}
		else
		{//没有离地
			vmcr->leg_flag=0;//置为0
			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcr->Tp=0.0f;
	 }	 
	 
	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 
	
	VMC_calc_2(vmcr);//计算期望的关节输出力矩

	//额定扭矩
  mySaturate(&vmcr->torque_set[1],-3.0f,3.0f);	
	mySaturate(&vmcr->torque_set[0],-3.0f,3.0f);		
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

geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_[5];
  twist.angular.z = x_[6];
  return twist;
}

void BalanceController::mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}

void BalanceController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  // imu_msg_ = *msg;

  tf::Matrix3x3 rotation_matrix;
  tf::Quaternion tf_quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  // 将四元数转换为旋转矩阵
  rotation_matrix.setRotation(tf_quaternion);

  // 从旋转矩阵中提取欧拉角
  rotation_matrix.getRPY(ins.Roll, ins.Yaw, ins.Pitch);
  
  ins.Gyro[0] = msg->angular_velocity.y;
  ins.Gyro[1] = msg->angular_velocity.z;
  ins.Gyro[2] = msg->angular_velocity.x;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)


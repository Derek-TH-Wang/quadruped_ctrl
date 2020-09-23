#include <ros/ros.h>
#include <math.h>
#include <iostream> 
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "quadruped_ctrl/commandDes.h"
#include "quadruped_ctrl/QuadrupedCmd.h"

#include "MPC_Ctrl/ConvexMPCLocomotion.h"
#include "Controllers/RobotLegState.h"
#include "Controllers/ControlFSMData.h"
#include "Dynamics/MiniCheetah.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Utilities/IMUTypes.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/ContactEstimator.h"
#include "calculateTool.h"

#include <time.h>

#define freq 400.0

int iter = 0;
int set_robot_mode;
std::vector<double> _gamepadCommand;
std::vector< Vec3<float> > _ini_foot_pos;
float init_joint_pos[12] = {-0.7, -1.0, 2.7, 0.7, -1.0, 2.7, -0.7, -1.0, 2.7, 0.7, -1.0, 2.7};

std::string joint_name[12] = {"abduct_fl", "thigh_fl", "knee_fl", "abduct_hl", "thigh_hl", "knee_hl",
                             "abduct_fr", "thigh_fr", "knee_fr", "abduct_hr", "thigh_hr", "knee_hr"};

ConvexMPCLocomotion *convexMPC;
LegController<float>* _legController;
StateEstimatorContainer<float>* _stateEstimator;
LegData legdata;
LegCommand legcommand;
ControlFSMData<float> control_data;
VectorNavData _vectorNavData;
CheaterState<double>* cheaterState;
StateEstimate<float> _stateEstimate;
RobotControlParameters* controlParameters;
DesiredStateCommand<float>* _desiredStateCommand;

void QuadrupedCtrl(){
  

}

void velCmdCallBack(const geometry_msgs::Twist& msg){
  if(abs(msg.linear.x) < 0.1){
    _gamepadCommand[0] = 0.0;
  }else{
    _gamepadCommand[0] = msg.linear.x;
  }

  if(abs(msg.linear.y) < 0.1){
    _gamepadCommand[1] = 0.0;
  }else{
    _gamepadCommand[1] = msg.linear.y * 0.5;
  }

  if(abs(msg.angular.x) < 0.1){
    _gamepadCommand[2] = 0.0;
  }else{
    _gamepadCommand[2] = msg.angular.x * 2.0;
  }

  if(abs(msg.angular.y) < 0.1){
    _gamepadCommand[3] = 0.0;
  }else{
    _gamepadCommand[3] = msg.angular.y * 2.0;
  }  
  
}

void PoseCmdCallBack(const geometry_msgs::Twist& msg){
  // robotRoll += 0.1 * msg.angular.x;
  // if(robotRoll > PI/4){
  //   robotRoll = PI/4;
  // }else if(robotRoll < -PI/4){
  //   robotRoll = -PI/4;
  // }

  // robotPitch += 0.1 * msg.angular.y;
  // if(robotPitch > PI/4){
  //   robotPitch = PI/4;
  // }else if(robotPitch < -PI/4){
  //   robotPitch = -PI/4;
  // }

  // robotYaw += 0.1 * msg.angular.z;
  // if(robotYaw > PI/4){
  //   robotYaw = PI/4;
  // }else if(robotYaw < -PI/4){
  //   robotYaw = -PI/4;
  // }
  
}

void ImuCmdCallBack(const sensor_msgs::Imu& msg){//IMU数据
  _vectorNavData.accelerometer(0, 0) = msg.linear_acceleration.x;
  _vectorNavData.accelerometer(1, 0) = msg.linear_acceleration.y;
  _vectorNavData.accelerometer(2, 0) = msg.linear_acceleration.z;

  _vectorNavData.quat(0, 0) = msg.orientation.x;
  _vectorNavData.quat(1, 0) = msg.orientation.y;
  _vectorNavData.quat(2, 0) = msg.orientation.z;
  _vectorNavData.quat(3, 0) = msg.orientation.w;

  _vectorNavData.gyro(0, 0) = msg.angular_velocity.x;
  _vectorNavData.gyro(1, 0) = msg.angular_velocity.y;
  _vectorNavData.gyro(2, 0) = msg.angular_velocity.z;
}

void jointStateCmdCallBack(const sensor_msgs::JointState& msg){//反馈的关节角度和速度
  for(int i = 0; i < 4; i++){
    legdata.q_abad[i] = msg.position[i * 3 ];
    legdata.q_hip[i] = msg.position[i * 3 + 1];
    legdata.q_knee[i] = msg.position[i * 3 + 2];
    legdata.qd_abad[i] = msg.velocity[i * 3 ];
    legdata.qd_hip[i] = msg.velocity[i * 3 + 1];
    legdata.qd_knee[i] = msg.velocity[i * 3 + 2];
  }  
}

void RobotComCallBack(const quadruped_ctrl::commandDes& msg){//反馈的关节角度和速度
  _vectorNavData.com_pos(0, 0) = msg.com_position[0];
  _vectorNavData.com_pos(1, 0) = msg.com_position[1];
  _vectorNavData.com_pos(2, 0) = msg.com_position[2];

  _vectorNavData.com_vel(0, 0) = msg.com_velocity[0];
  _vectorNavData.com_vel(1, 0) = msg.com_velocity[1];
  _vectorNavData.com_vel(2, 0) = msg.com_velocity[2];
}

bool setRobotMode(quadruped_ctrl::QuadrupedCmd::Request  &req,
                  quadruped_ctrl::QuadrupedCmd::Response &res)
{
  set_robot_mode = req.cmd;
}


int main(int argc, char **argv) {
  float hMax = 0.25;
  _gamepadCommand.clear();
  _gamepadCommand.resize(4);
  _ini_foot_pos.clear();
  _ini_foot_pos.resize(4);

  clock_t start, finish; 

  Quadruped<float> _quadruped;
  FloatingBaseModel<float> _model;
  convexMPC = new ConvexMPCLocomotion(1/freq, 13);

  std::cout << "ssssssssssss " << 1/freq << std::endl; 
  
  _quadruped = buildMiniCheetah<float>();
  _model = _quadruped.buildModel();

  _legController = new LegController<float>(_quadruped);

  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, &_vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);

  //reset the state estimator
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  //源代码中中对姿态和位置速度的估计，添加到状态估计器中
  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  
  _desiredStateCommand =
    new DesiredStateCommand<float>(1/freq);

  ros::init(argc, argv, "quadruped_robot");
  ros::NodeHandle n;

  ros::Rate loop_rate(freq);
  sensor_msgs::JointState joint_state;

  ros::Publisher pub_joint = n.advertise<sensor_msgs::JointState>("set_js", 1000);  //下发给simulator或者robot的关节控制数据（关节扭矩）
  ros::ServiceClient jointCtrlMode = n.serviceClient<quadruped_ctrl::QuadrupedCmd>("set_jm");
  ros::ServiceServer robotMode = n.advertiseService("robot_mode", setRobotMode);
  ros::Subscriber sub_vel = n.subscribe("cmd_vel", 1000, velCmdCallBack);           //接收的手柄速度信息
  ros::Subscriber sub_imu = n.subscribe("imu_body", 1000, ImuCmdCallBack);          // imu反馈的身体位置和姿态
  ros::Subscriber sub_jointstate = n.subscribe("get_js", 1000, jointStateCmdCallBack);   //simulator或者robot反馈的关节信息（位置、速度等）
  ros::Subscriber sub_com_state = n.subscribe("get_com", 1000, RobotComCallBack);

  usleep(1000 * 1000);

  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();



  while (ros::ok()){
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.effort.resize(12);

    
    _stateEstimator->run();//Run the state estimator step
    
    // Update the data from the robot
    _legController->updateData(&legdata); 

    // Setup the leg controller for a new iteration
    _legController->zeroCommand();
    _legController->setEnabled(true);
    _legController->setMaxTorqueCheetah3(208.5);

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands(_gamepadCommand);
   
    convexMPC->run(_quadruped, *_legController, *_stateEstimator, *_desiredStateCommand, _gamepadCommand, set_robot_mode);
    
    // start = clock();
    // finish = clock();
    // double duration;
    // duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // printf( "%f seconds\n", duration );
    // for(int i = 0; i < 3; i++){
    //   std::cout << "get com position" << legdata.robot_com_position[i] << std::endl;
    // }

    _legController->updateCommand(&legcommand);

    for(int i = 0; i < 12; i++){
      joint_state.name[i] = joint_name[i];
    }

    for(int i = 0; i < 4; i++){
      joint_state.effort[i * 3] = legcommand.tau_abad_ff[i];
      joint_state.effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
      joint_state.effort[i * 3 + 2] = legcommand.tau_knee_ff[i];  
    }

    for(int i = 0; i < 3; i++){
      joint_state.position[i] = _legController->commands[0].qDes(i);
      joint_state.position[i + 3] = _legController->commands[1].qDes(i);
      joint_state.position[i + 6] = _legController->commands[2].qDes(i);
      joint_state.position[i + 9] = _legController->commands[3].qDes(i);
    }  

    iter++;
    
    if(iter > 2){
      pub_joint.publish(joint_state);  //第一次的值有问题，1和2腿的commandPes的值为0
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
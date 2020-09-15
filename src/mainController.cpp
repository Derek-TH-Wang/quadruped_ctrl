#include <ros/ros.h>
#include <math.h>
#include <iostream> 
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "ConvexMPC/commandDes.h"
#include "ConvexMPC/QuadrupedCmd.h"

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

#define freq 500

int iter = 0;
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

void RobotComCallBack(const ConvexMPC::commandDes& msg){//反馈的关节角度和速度
  _vectorNavData.com_pos(0, 0) = msg.com_position[0];
  _vectorNavData.com_pos(1, 0) = msg.com_position[1];
  _vectorNavData.com_pos(2, 0) = msg.com_position[2];

  _vectorNavData.com_vel(0, 0) = msg.com_velocity[0];
  _vectorNavData.com_vel(1, 0) = msg.com_velocity[1];
  _vectorNavData.com_vel(2, 0) = msg.com_velocity[2];
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
  convexMPC = new ConvexMPCLocomotion(0.002, 13);
  
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
    new DesiredStateCommand<float>(0.002);

  ros::init(argc, argv, "quadruped_robot");
  ros::NodeHandle n;

  ros::Rate loop_rate(freq);
  sensor_msgs::JointState joint_state;

  ros::Publisher pub_joint = n.advertise<sensor_msgs::JointState>("set_js", 1000);  //下发给simulator或者robot的关节控制数据（关节扭矩）
  // ros::Publisher pub_command = n.advertise<ConvexMPC::commandDes>("set_command", 1000);  //下发给simulator的关节期望位置
  ros::ServiceClient jointCtrlMode = n.serviceClient<ConvexMPC::QuadrupedCmd>("set_jm");
  ros::Subscriber sub_vel = n.subscribe("cmd_vel", 1000, velCmdCallBack);           //接收的手柄速度信息
  // ros::Subscriber sub_pos = n.subscribe("cmd_pose", 1000, PoseCmdCallBack);         //接收的手柄位置信息
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

    // if(iter == 0){
    //   for(int i = 0; i < 4; i++){
    //     legdata.q_abad[i] = init_joint_pos[i * 3 ];
    //     legdata.q_hip[i] = init_joint_pos[i * 3 + 1];
    //     legdata.q_knee[i] = init_joint_pos[i * 3 + 2];
    //   }
    //   _legController->updateData(&legdata);
    //   for(size_t leg(0); leg<4; ++leg){
    //     _ini_foot_pos[leg] = _legController->datas[leg].p;
    //   }
    // }
    
    _stateEstimator->run();//Run the state estimator step

    // if(iter < 10){
    //   std::cout << "the need value is : "<< legdata.q_abad[0]  
    //             << legdata.q_hip[0] << legdata.q_knee[0] << std::endl;
    // }
    
    // Update the data from the robot
    _legController->updateData(&legdata); 

    // std::cout << "position: " << legdata.q_abad[0] << std::endl;


    // Setup the leg controller for a new iteration
    _legController->zeroCommand();
    _legController->setEnabled(true);
    _legController->setMaxTorqueCheetah3(208.5);

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands(_gamepadCommand);

    // std::cout << "*******************************aaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
    // start = clock();
    convexMPC->run(_quadruped, *_legController, *_stateEstimator, *_desiredStateCommand, _gamepadCommand);
    
    // finish = clock();
    // double duration;
    // duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // printf( "%f seconds\n", duration );
    // for(int i = 0; i < 3; i++){
    //   std::cout << "get com position" << legdata.robot_com_position[i] << std::endl;
    // }

    
    // float progress = 2 * iter * 0.002;

    // if (progress > 1.){ progress = 1.; }

    // for(int i = 0; i < 4; i++) {
    //   _legController->commands[i].kpCartesian = Vec3<float>(300, 300, 300).asDiagonal();
    //   _legController->commands[i].kdCartesian = Vec3<float>(5, 5, 5).asDiagonal();

    //   _legController->commands[i].pDes = _ini_foot_pos[i];
    //   // std::cout << "the " << i << " leg is" << _ini_foot_pos[i] << std::endl;
    //   _legController->commands[i].pDes[2] = 
    //     progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    //   // std::cout << "the z value is " << _legController->commands[i].pDes[2] << std::endl;
    // }

    // if(iter == 0){
    //   usleep(1000 * 500);
    //   std::cout << "dddddddddddddddddddd" << std::endl;
    // }

    _legController->updateCommand(&legcommand);
    
    // for(int i = 0; i < 4; i++){
    //   std::cout << "the force value is " << i << ":" << legcommand.tau_knee_ff[i] << std::endl;
    // }

    for(int i = 0; i < 12; i++){
      joint_state.name[i] = joint_name[i];
    }
    for(int i = 0; i < 4; i++){
      joint_state.effort[i * 3] = legcommand.tau_abad_ff[i];
      joint_state.effort[i * 3 + 1] = legcommand.tau_hip_ff[i];
      joint_state.effort[i * 3 + 2] = legcommand.tau_knee_ff[i];
      // if(iter < 10){
      //   std::cout << "the need value is : " << i << "  " << legcommand.tau_abad_ff[i] << ", " 
      //             << legcommand.tau_hip_ff[i] << ", " << legcommand.tau_knee_ff[i] << std::endl;
      // }
      
    }
    for(int i = 0; i < 3; i++){
      joint_state.position[i] = _legController->commands[0].pDes(i);
      joint_state.position[i + 3] = _legController->commands[1].pDes(i);
      joint_state.position[i + 6] = _legController->commands[2].pDes(i);
      joint_state.position[i + 9] = _legController->commands[3].pDes(i);
    }

    // for(int i = 0; i < 3; i++){
    //   joint_state.velocity[i] = _legController->commands[0].vDes(i);
    //   joint_state.velocity[i + 3] = _legController->commands[1].vDes(i);
    //   joint_state.velocity[i + 6] = _legController->commands[2].vDes(i);
    //   joint_state.velocity[i + 9] = _legController->commands[3].vDes(i);
    // }

    // if(iter < 2){
    //   std::cout << "the need value is : " << _legController->commands[1].pDes << ", " 
    //               << _legController->commands[2].pDes << ", " << _legController->commands[0].pDes << std::endl;
    // }

    // joint_state.position[0] = 0.0;
    // joint_state.position[1] = -0.0950483;
    // joint_state.position[2] = -0.285717;
    // joint_state.position[3] = -0.0099;
    // joint_state.position[4] = 0.0959878;
    // joint_state.position[5] = -0.281347;
    // joint_state.position[6] = -0.0100371;
    // joint_state.position[7] = -0.095976;
    // joint_state.position[8] = -0.281307;
    // joint_state.position[9] = 0.0;
    // joint_state.position[10] = 0.0949517;
    // joint_state.position[11] = -0.285748;


    // std::cout << "the value is :" << _legController->commands[0].pDes(2) << std::endl;
    
    

    iter++;
    
    
    if(iter > 2){
      pub_joint.publish(joint_state);  //第一次的值有问题，1和2腿的commandPes的值为0
    }

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
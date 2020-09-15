/*
* this file give some useful tools to calculate
*/
#include "calculateTool.h"

CalculateTool::CalculateTool(){
  firstOrnot = true;
  lastTime = 0.0;
  lastErr = 0.0;
}

CalculateTool::~CalculateTool(){

}


int CalculateTool::factorial(int n){
  unsigned long long factor = 1;
  for(int i = 1; i <= n; i++){
    factor *= i;
  }
  return factor;
}

double CalculateTool::power(double a, int k){
  double result = 1.0;
  for(int i = 1; i <= k; i++){
    result = result * a;
  }
  return result;
}

double CalculateTool::getCurrentTime(){//get time: s
  double currentTime;
  struct timeval tv;
  gettimeofday(&tv, NULL);

  currentTime = double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;

  return currentTime;
}

void CalculateTool::getRx(double roll, Matrix4d &rotationX){
  rotationX(0,0) = 1.0;
  rotationX(0,1) = 0.0;
  rotationX(0,2) = 0.0;
  rotationX(0,3) = 0.0;
  rotationX(1,0) = 0.0;
  rotationX(1,1) = cos(roll);
  rotationX(1,2) = -sin(roll);
  rotationX(1,3) = 0.0;
  rotationX(2,0) = 0.0;
  rotationX(2,1) = sin(roll);
  rotationX(2,2) = cos(roll);
  rotationX(2,3) = 0.0;
  rotationX(3,0) = 0.0;
  rotationX(3,1) = 0.0;
  rotationX(3,2) = 0.0;
  rotationX(3,3) = 1.0;
}

void CalculateTool::getRy(double pitch, Matrix4d &rotationY){
  rotationY(0,0) = cos(pitch);
  rotationY(0,1) = 0.0;
  rotationY(0,2) = sin(pitch);
  rotationY(0,3) = 0.0;
  rotationY(1,0) = 0.0;
  rotationY(1,1) = 1.0;
  rotationY(1,2) = 0.0;
  rotationY(1,3) = 0.0;
  rotationY(2,0) = -sin(pitch);
  rotationY(2,1) = 0.0;
  rotationY(2,2) = cos(pitch);
  rotationY(2,3) = 0.0;
  rotationY(3,0) = 0.0;
  rotationY(3,1) = 0.0;
  rotationY(3,2) = 0.0;
  rotationY(3,3) = 1.0;
}

void CalculateTool::getRz(double yaw, Matrix4d &rotationZ){
  rotationZ(0,0) = cos(yaw);
  rotationZ(0,1) = -sin(yaw);
  rotationZ(0,2) = 0.0;
  rotationZ(0,3) = 0.0;
  rotationZ(1,0) = sin(yaw);
  rotationZ(1,1) = cos(yaw);
  rotationZ(1,2) = 0.0;
  rotationZ(1,3) = 0.0;
  rotationZ(2,0) = 0.0;
  rotationZ(2,1) = 0.0;
  rotationZ(2,2) = 1.0;
  rotationZ(2,3) = 0.0;
  rotationZ(3,0) = 0.0;
  rotationZ(3,1) = 0.0;
  rotationZ(3,2) = 0.0;
  rotationZ(3,3) = 1.0;
}

void CalculateTool::RTmatrix(std::vector<double> orientation, std::vector<double> position, Matrix4d &translateMatrix){
  Matrix4d rotationX;
  Matrix4d rotationY;
  Matrix4d rotationZ;
  Matrix4d translation;

  getRx(orientation[0], rotationX);
  getRy(orientation[1], rotationY);
  getRz(orientation[2], rotationZ);

  translation << 1.0, 0.0, 0.0, position[0],
                 0.0, 1.0, 0.0, position[1],
                 0.0, 0.0, 1.0, position[2],
                 0.0, 0.0, 0.0, 1.0;

  translateMatrix = rotationX * rotationY * rotationZ * translation;
}

void CalculateTool::transform(std::vector<double> coord, std::vector<double> orientation, 
                              std::vector<double> position, std::vector<double> &transformVector){
  Matrix4d translateMatrix;
  MatrixXd myVector(4,1);
  MatrixXd transform(4,1);
  myVector(0,0) = coord[0];
  myVector(1,0) = coord[1];
  myVector(2,0) = coord[2];
  myVector(3,0) = 1.0;

  RTmatrix(orientation, position, translateMatrix);
  transform = translateMatrix * myVector;
  transformVector[0] = transform(0,0);
  transformVector[1] = transform(1,0);
  transformVector[2] = transform(2,0);
}

void CalculateTool::NormVector(int n, float *A, float *B) {
  float sum = 0;
  int i;
  for (i = 0; i < n; i++) {
    sum += (*(A + i)) * (*(A + i));
  }
  *B = sqrt(sum);
}

void CalculateTool::Quat2T(float quat[4], float *T) {
  float x = quat[0];
  float y = quat[1];
  float z = quat[2];
  float s = quat[3];
  float norm;

  NormVector(4, quat, &norm);
  x = x / norm;
  y = y / norm;
  z = z / norm;
  s = s / norm;
  T[0] = 1.0 - 2.0 * (y * y + z * z);
  T[1] = 2.0 * (x * y - s * z);
  T[2] = 2.0 * (x * z + s * y);

  T[3] = 2.0 * (x * y + s * z);
  T[4] = 1.0 - 2.0 * (x * x + z * z);
  T[5] = 2.0 * (y * z - s * x);

  T[6] = 2.0 * (x * z - s * y);
  T[7] = 2.0 * (y * z + s * x);
  T[8] = 1.0 - 2.0 * (x * x + y * y);

}

double CalculateTool::PIDController(double Kp, double Ki, double Kd, double Setpoint, double Input){
  double currentTime;
  double timeChange;
  double error, errSum, dErr;
  double Output;

  if(firstOrnot){
    lastTime = getCurrentTime();
    firstOrnot = false;

  }
  currentTime = getCurrentTime();
  // timeChange = currentTime - lastTime;
  // std::cout << "time is " << currentTime << ", " << lastTime << ", " << timeChange << std::endl;
  timeChange = 0.005;
  error = Setpoint - Input;
  errSum += (error * timeChange);

  if(timeChange < 0.00001){
    dErr = 0.0;
  }else{
    dErr = (error - lastErr) / timeChange;
  }
  

  Output = Kp * error + Ki * errSum + Kd * dErr;

  lastErr = error;
  lastTime = currentTime;

  return Output;
}



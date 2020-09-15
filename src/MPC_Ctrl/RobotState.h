#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "Utilities/common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w; //定义了机器人的在世界坐标系下的，位置p，速度p˙，以及机身坐标系下的旋转角ω,均为3×1矩阵
        Matrix<fpt,3,4> r_feet; //机身参考系下的足端位置，3×4矩阵
        Matrix<fpt,3,3> R;       //机身坐标系到世界坐标系的旋转矩阵
        Matrix<fpt,3,3> R_yaw;   //偏航角旋转矩阵
        Matrix<fpt,3,3> I_body;  //机身坐标系下的惯量矩阵
        Quaternionf q;           //四元素表示的世界坐标系下的旋转
        fpt yaw;                 //偏航角
        fpt m = 9;               //机器人质量
        //fpt m = 50.236; //DH
    //private:
};
#endif

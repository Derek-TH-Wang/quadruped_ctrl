#!/usr/bin/env python3

import os
import numpy
import rospy
import rospkg
import time
import threading
import random
import ctypes
import pybullet as p
import pybullet_data
from geometry_msgs.msg import Twist
from quadruped_ctrl.srv import QuadrupedCmd, QuadrupedCmdResponse

getMode = 10
get_position = []
get_effort = []
get_velocity = []
get_last_vel = [0.0, 0.0, 0.0]
myflags = 0
firstflags = 0
jointConfig = numpy.array(
    [-0.7, -1.0, 2.7, 0.7, -1.0, 2.7, -0.7, -1.0, 2.7, 0.7, -1.0, 2.7])
# init_new_pos = [-0.0, -1.4, 2.7, 0.0, -1.4, 2.7, -0.0, -1.4, 2.7, 0.0, -1.4, 2.7]
init_new_pos = [0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class StructPointer(ctypes.Structure):
    _fields_ = [("eff", ctypes.c_double * 12)]


def convert_type(input):
    ctypes_map = {int: ctypes.c_int,
                  float: ctypes.c_double,
                  str: ctypes.c_char_p
                  }
    input_type = type(input)
    if input_type is list:
        length = len(input)
        if length == 0:
            rospy.logerr("convert type failed...input is "+input)
            return 0
        else:
            arr = (ctypes_map[type(input[0])] * length)()
            for i in range(length):
                arr[i] = bytes(
                    input[i], encoding="utf-8") if (type(input[0]) is str) else input[i]
            return arr
    else:
        if input_type in ctypes_map:
            return ctypes_map[input_type](bytes(input, encoding="utf-8") if type(input) is str else input)
        else:
            rospy.logerr("convert type failed...input is "+input)
            return 0


def thread_job():
    rospy.spin()


def callback_gait(req):
    cpp_gait_ctrller.set_gait_type(convert_type(req.cmd))
    return QuadrupedCmdResponse(0, "get the mode")


def callback_body_vel(msg):
    vel = [msg.linear.x, msg.linear.y, msg.angular.x]
    cpp_gait_ctrller.set_robot_vel(convert_type(vel))


def acc_filter(value, last_accValue):
    a = 1.0
    filter_value = a * value + (1 - a) * last_accValue
    return filter_value


def init_simulator():
    global boxId, motor_id_list, compensateReal, get_last_euler
    get_last_euler = [0.0, 0.0, 0.0]
    p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    # motor_id_list = [4, 5, 6, 12, 13, 14, 0, 1, 2, 8, 9, 10]
    compensateReal = [-1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1]
    p.setGravity(0, 0, -9.8)
    cubeStartPos = [0, 0, 0.41]
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    # p.setPhysicsEngineParameter(numSolverIterations=30)
    # p.setPhysicsEngineParameter(enableConeFriction=0)
    # planeId = p.loadURDF("plane.urdf")
    # p.changeDynamics(planeId, -1, lateralFriction=1.0)
    # boxId = p.loadURDF("/home/wgx/Workspace_Ros/src/cloudrobot/src/quadruped_robot.urdf", cubeStartPos,
    #                    useFixedBase=FixedBase)

    heightPerturbationRange = 0.06
    plane = True
    if plane:
        planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
        ground_id = p.createMultiBody(0, planeShape)
    else:
        numHeightfieldRows = 256
        numHeightfieldColumns = 256
        heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns
        for j in range(int(numHeightfieldColumns/2)):
            for i in range(int(numHeightfieldRows/2)):
                height = random.uniform(0, heightPerturbationRange)
                heightfieldData[2*i+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+1+2*j*numHeightfieldRows] = height
                heightfieldData[2*i+(2*j+1)*numHeightfieldRows] = height
                heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows] = height

        terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.05, .05, 1], heightfieldTextureScaling=(
            numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
        ground_id = p.createMultiBody(0, terrainShape)

    p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
    p.changeDynamics(ground_id, -1, lateralFriction=1.0)

    boxId = p.loadURDF("mini_cheetah/mini_cheetah.urdf", cubeStartPos,
                       useFixedBase=False)

    jointIds = []
    for j in range(p.getNumJoints(boxId)):
        p.getJointInfo(boxId, j)
        jointIds.append(j)

    for j in range(12):
        p.setJointMotorControl2(
            boxId, motor_id_list[j], p.POSITION_CONTROL, init_new_pos[j], force=5.0)

    # slope terrain
    # colSphereId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.001])
    # colSphereId1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.5, 0.06])
    # BoxId = p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
    # BoxId = p.createMultiBody(100, colSphereId1, basePosition=[1.6, 1.0, 0.0])

    # stairs
    # colSphereId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.02])
    # colSphereId1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.04])
    # colSphereId2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.06])
    # colSphereId3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.08])
    # colSphereId4 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.0, 0.4, 0.1])
    # BoxId = p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
    # BoxId = p.createMultiBody(100, colSphereId1, basePosition=[1.2, 1.0, 0.0])
    # BoxId = p.createMultiBody(100, colSphereId2, basePosition=[1.4, 1.0, 0.0])
    # BoxId = p.createMultiBody(100, colSphereId3, basePosition=[1.6, 1.0, 0.0])
    # BoxId = p.createMultiBody(100, colSphereId4, basePosition=[2.7, 1.0, 0.0])

    # many box
    colSphereId = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId1 = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId2 = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId3 = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId4 = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
    p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
    p.createMultiBody(100, colSphereId1, basePosition=[1.2, 1.0, 0.0])
    p.createMultiBody(100, colSphereId2, basePosition=[1.4, 1.0, 0.0])
    p.createMultiBody(100, colSphereId3, basePosition=[1.6, 1.0, 0.0])
    p.createMultiBody(10, colSphereId4, basePosition=[2.7, 1.0, 0.0])

    for i in range(1000):
        p.stepSimulation()
        # time.sleep(0.001)


def main():
    global getMode, get_last_vel, myflags, firstflags
    get_orientation = []
    get_matrix = []
    get_velocity = []
    get_invert = []
    freq = 400
    rate = rospy.Rate(freq)  # hz

    imu_data = [0] * 10
    leg_data = [0] * 24

    freq = rospy.get_param('/simulation/freq')
    stand_kp = rospy.get_param('/simulation/stand_kp')
    stand_kd = rospy.get_param('/simulation/stand_kd')
    joint_kp = rospy.get_param('/simulation/joint_kp')
    joint_kd = rospy.get_param('/simulation/joint_kd')
    rospy.loginfo("freq = " + str(freq) + " PID = " +
                  str([stand_kp, stand_kd, joint_kp, joint_kd]))
    cpp_gait_ctrller.init_controller(convert_type(
        freq), convert_type([stand_kp, stand_kd, joint_kp, joint_kd]))
    while not rospy.is_shutdown():

        get_orientation = []
        pose_orn = p.getBasePositionAndOrientation(boxId)
        for i in range(4):
            get_orientation.append(pose_orn[1][i])
        # get_euler = p.getEulerFromQuaternion(get_orientation)
        get_velocity = p.getBaseVelocity(boxId)
        get_invert = p.invertTransform(pose_orn[0], pose_orn[1])
        get_matrix = p.getMatrixFromQuaternion(get_invert[1])

        # IMU data
        imu_data[3] = pose_orn[1][0]
        imu_data[4] = pose_orn[1][1]
        imu_data[5] = pose_orn[1][2]
        imu_data[6] = pose_orn[1][3]

        imu_data[7] = get_matrix[0] * get_velocity[1][0] + get_matrix[1] * \
            get_velocity[1][1] + get_matrix[2] * get_velocity[1][2]
        imu_data[8] = get_matrix[3] * get_velocity[1][0] + get_matrix[4] * \
            get_velocity[1][1] + get_matrix[5] * get_velocity[1][2]
        imu_data[9] = get_matrix[6] * get_velocity[1][0] + get_matrix[7] * \
            get_velocity[1][1] + get_matrix[8] * get_velocity[1][2]

        # calculate the acceleration of the robot
        linear_X = (get_velocity[0][0] - get_last_vel[0]) * freq
        linear_Y = (get_velocity[0][1] - get_last_vel[1]) * freq
        linear_Z = 9.8 + (get_velocity[0][2] - get_last_vel[2]) * freq
        imu_data[0] = get_matrix[0] * linear_X + \
            get_matrix[1] * linear_Y + get_matrix[2] * linear_Z
        imu_data[1] = get_matrix[3] * linear_X + \
            get_matrix[4] * linear_Y + get_matrix[5] * linear_Z
        imu_data[2] = get_matrix[6] * linear_X + \
            get_matrix[7] * linear_Y + get_matrix[8] * linear_Z

        # joint data
        joint_state = p.getJointStates(boxId, motor_id_list)
        leg_data[0:12] = [joint_state[0][0], joint_state[1][0], joint_state[2][0],
                          joint_state[3][0], joint_state[4][0], joint_state[5][0],
                          joint_state[6][0], joint_state[7][0], joint_state[8][0],
                          joint_state[9][0], joint_state[10][0], joint_state[11][0]]

        leg_data[12:24] = [joint_state[0][1], joint_state[1][1], joint_state[2][1],
                           joint_state[3][1], joint_state[4][1], joint_state[5][1],
                           joint_state[6][1], joint_state[7][1], joint_state[8][1],
                           joint_state[9][1], joint_state[10][1], joint_state[11][1]]
        com_velocity = [get_velocity[0][0],
                        get_velocity[0][1], get_velocity[0][2]]
        get_last_vel.clear()
        get_last_vel = com_velocity

        if myflags < 40:
            cpp_gait_ctrller.pre_work(convert_type(
                imu_data), convert_type(leg_data))
        else:
            tau = cpp_gait_ctrller.toque_calculator(convert_type(
                imu_data), convert_type(leg_data))

            get_effort = [tau.contents.eff[i] for i in range(12)]
            if len(get_effort):
                if firstflags < 1:
                    for j in range(16):
                        force = 0
                        p.setJointMotorControl2(
                            boxId, j, p.VELOCITY_CONTROL, force=force)
                p.setJointMotorControlArray(bodyUniqueId=boxId,
                                            jointIndices=motor_id_list,
                                            controlMode=p.TORQUE_CONTROL,
                                            forces=get_effort)
                firstflags = firstflags + 1

        myflags = myflags + 1
        p.setTimeStep(0.0015)
        p.stepSimulation()
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('quadruped_simulator', anonymous=True)
    init_simulator()
    rospack = rospkg.RosPack()
    path = rospack.get_path('quadruped_ctrl')
    so_file = path.replace('src/quadruped_ctrl',
                           'devel/lib/libquadruped_ctrl.so')
    if(not os.path.exists(so_file)):
        so_file = path.replace('src/quadruped_ctrl',
                               'build/lib/libquadruped_ctrl.so')
    if(not os.path.exists(so_file)):
        rospy.logerr("cannot find cpp.so file")
    cpp_gait_ctrller = ctypes.cdll.LoadLibrary(so_file)
    cpp_gait_ctrller.toque_calculator.restype = ctypes.POINTER(StructPointer)
    rospy.loginfo("find so file = " + so_file)
    s = rospy.Service('gait_type', QuadrupedCmd, callback_gait)
    rospy.Subscriber("cmd_vel", Twist, callback_body_vel, buff_size=10000)
    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    main()

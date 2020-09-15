#!/usr/bin/env python3

import pybullet as p
import numpy
import rospy
import time
import threading
import pybullet_data
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from quadruped_ctrl.srv import QuadrupedCmd, QuadrupedCmdResponse
from quadruped_ctrl.msg import commandDes

getMode = 10
get_position = []
get_effort = []
get_velocity = []
get_last_vel = [0.0, 0.0, 0.0]
flags = 0
myflags = 0

def init_simulation():
    global boxId, motor_id_list, compensateReal, get_last_euler
    get_last_euler = [0.0, 0.0, 0.0]
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    # motor_id_list = [4, 5, 6, 12, 13, 14, 0, 1, 2, 8, 9, 10]
    compensateReal = [-1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1]
    p.setGravity(0, 0, -9.8)
    cubeStartPos = [0, 0, 0.8]
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    FixedBase = False  # if fixed no plane is imported
    if (FixedBase == False):
        planeId = p.loadURDF("plane.urdf")
    p.changeDynamics(planeId, -1, lateralFriction=0.3)
    # print(p.getDynamicsInfo(planeId, -1))
    # boxId = p.loadURDF("/home/wgx/Workspace_Ros/src/cloudrobot/src/quadruped_robot.urdf", cubeStartPos,
    #                    useFixedBase=FixedBase)
    boxId = p.loadURDF("mini_cheetah/mini_cheetah.urdf", cubeStartPos,
                       useFixedBase=False)

    jointIds = []
    for j in range(p.getNumJoints(boxId)):
        #    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(boxId, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        jointIds.append(j)

    footFR_index = 3
    footFL_index = 7
    footBR_index = 11
    footBL_index = 15
    # init_new_pos = [0.02, -0.78, 1.74, -0.02, -0.78, 1.74, 0.02, -0.78, 1.74, -0.02, -0.78, 1.74]
    jointConfig = numpy.array([-0.7, -1.0, 2.7, 0.7, -1.0, 2.7, -0.7, -1.0, 2.7, 0.7, -1.0, 2.7])
    # init_new_pos = [-0.7, -1.0, 2.7, 0.7, -1.0, 2.7, -0.7, -1.0, 2.7, 0.7, -1.0, 2.7]
    init_new_pos = [-0.0, -1.4, 2.7, 0.0, -1.4, 2.7, -0.0, -1.4, 2.7, 0.0, -1.4, 2.7]
    for j in range(12):
        p.setJointMotorControl2(boxId, motor_id_list[j], p.POSITION_CONTROL, init_new_pos[j], force=500.0)

    # slope terrain
    # colSphereId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.001])
    # colSphereId1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.5, 0.06])
    # BoxId = p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
    # BoxId = p.createMultiBody(100, colSphereId1, basePosition=[1.6, 1.0, 0.0])

    # stairs
    colSphereId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.02])
    colSphereId1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.04])
    colSphereId2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.06])
    colSphereId3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.08])
    colSphereId4 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.0, 0.4, 0.1])
    BoxId = p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
    BoxId = p.createMultiBody(100, colSphereId1, basePosition=[1.2, 1.0, 0.0])
    BoxId = p.createMultiBody(100, colSphereId2, basePosition=[1.4, 1.0, 0.0])
    BoxId = p.createMultiBody(100, colSphereId3, basePosition=[1.6, 1.0, 0.0])
    BoxId = p.createMultiBody(100, colSphereId4, basePosition=[2.7, 1.0, 0.0])


def thread_job():
    rospy.spin()


def callback_state(msg):
    global getMode, flags, get_position, get_effort
    coxa = 0.066
    femur = 0.209
    tibia = 0.195
    getMode = 2
    _FRcoord = []
    _FLcoord = []
    _BRcoord = []
    _BLcoord = []
    get_position.clear()
    get_effort.clear()

    for i in range(3):
        _FRcoord.append(msg.position[i])
        _FLcoord.append(msg.position[i + 3])
        _BRcoord.append(msg.position[i + 6])
        _BLcoord.append(msg.position[i + 9])

    FR_angles = solve_R(_FRcoord, coxa, femur, tibia)
    FL_angles = solve_L(_FLcoord, coxa, femur, tibia)
    BR_angles = solve_R(_BRcoord, coxa, femur, tibia)
    BL_angles = solve_L(_BLcoord, coxa, femur, tibia)

    # if flags < 10:
    #     print(_FRcoord, _FLcoord, _BRcoord, _BLcoord)

    get_position.append(FR_angles[0])
    get_position.append(-FR_angles[1])
    get_position.append(-FR_angles[2])
    get_position.append(FL_angles[0])
    get_position.append(-FL_angles[1])
    get_position.append(-FL_angles[2])
    get_position.append(BR_angles[0])
    get_position.append(-BR_angles[1])
    get_position.append(-BR_angles[2])
    get_position.append(BL_angles[0])
    get_position.append(-BL_angles[1])
    get_position.append(-BL_angles[2])

    get_effort.append(msg.effort[0])
    get_effort.append(msg.effort[1])
    get_effort.append(msg.effort[2])
    get_effort.append(msg.effort[3])
    get_effort.append(msg.effort[4])
    get_effort.append(msg.effort[5])
    get_effort.append(msg.effort[6])
    get_effort.append(msg.effort[7])
    get_effort.append(msg.effort[8])
    get_effort.append(msg.effort[9])
    get_effort.append(msg.effort[10])
    get_effort.append(msg.effort[11])

    # for i in range(3):
    #     get_position.append(FR_angles[i])
    # for i in range(3):
    #     get_position.append(FL_angles[i])
    # for i in range(3):
    #     get_position.append(BR_angles[i])
    # for i in range(3):
    #     get_position.append(BL_angles[i])

    # for i in range(12):
    #     get_position.append(msg.position[i])

    # print(get_position)

    flags = flags + 1


def callback_mode(req):
    global getMode
    getMode = req.cmd
    if getMode == 0:
        p.resetBasePositionAndOrientation(boxId, [0, 0, 0.2], [0, 0, 0, 1])
        time.sleep(1)
        for j in range(16):
            force = 0
            p.setJointMotorControl2(boxId, j, p.VELOCITY_CONTROL, force=force)  # , positionGain=10, velocityGain=10)
            # p.changeDynamics(quadruped, j, spinningFriction=0.01, rollingFriction=0.01, jointDamping=1.0)
            # p.changeDynamics(quadruped, j, jointDamping=0.5)

    return QuadrupedCmdResponse(0, "get the mode")


def thread_job():
    rospy.spin()


def checkdomain(D):
    if D > 1.00001 or D < -1.00001:
        print("____OUT OF DOMAIN____")
        if D > 1.00001:
            D = 0.99999
            return D
        elif D < -1.00001:
            D = -0.99999
            return D
    else:
        return D


def solve_R(coord, coxa, femur, tibia):
    D = (coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2 + (-coord[0]) ** 2 - femur ** 2 - tibia ** 2) / (
                2 * tibia * femur)  # siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1 - D ** 2), D)
    tetta = -numpy.arctan2(coord[2], coord[1]) - numpy.arctan2(numpy.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2),
                                                               -coxa)
    alpha = numpy.arctan2(-coord[0], numpy.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2)) - numpy.arctan2(
        tibia * numpy.sin(gamma), femur + tibia * numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles


def solve_L(coord, coxa, femur, tibia):
    D = (coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2 + (-coord[0]) ** 2 - femur ** 2 - tibia ** 2) / (
                2 * tibia * femur)  # siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1 - D ** 2), D)
    tetta = -numpy.arctan2(coord[2], coord[1]) - numpy.arctan2(numpy.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2),
                                                               coxa)
    alpha = numpy.arctan2(-coord[0], numpy.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2)) - numpy.arctan2(
        tibia * numpy.sin(gamma), femur + tibia * numpy.cos(gamma))
    angles = numpy.array([-tetta, alpha, gamma])
    return angles


def talker():
    global getMode, get_last_vel, myflags
    iter = 0
    iter1 = 0
    get_orientation = []
    get_euler = []
    get_matrix = []
    get_velocity = []
    get_invert = []
    jointTorques = []
    jointTorques1 = []
    init_pos = []
    middle_target = []
    joint_Kp = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]
    joint_Kd = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    # stand_target = [-0.204585, -0.78432, 1.88128,
    #                0.205284, -0.784348, 1.88175,
    #                -0.278591, -0.805554, 1.95002,
    #                0.27894, -0.805785, 1.95044]
    # stand_target = [0.02, -0.78, 1.74, -0.02, -0.78, 1.74, 0.02, -0.78, 1.74, -0.02, -0.78, 1.74]
    stand_target = [0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6]
    pub1 = rospy.Publisher('/get_js', JointState, queue_size=100)
    pub2 = rospy.Publisher('/imu_body', Imu, queue_size=100)
    pub3 = rospy.Publisher('/get_com', commandDes, queue_size=100)
    imu_msg = Imu()
    setJSMsg = JointState()
    com_msg = commandDes()
    freq = 500
    rate = rospy.Rate(freq)  # hz
    while not rospy.is_shutdown():
        # p.stepSimulation()
        get_orientation = []
        pose_orn = p.getBasePositionAndOrientation(boxId)
        for i in range(4):
            get_orientation.append(pose_orn[1][i])
        get_euler = p.getEulerFromQuaternion(get_orientation)
        get_matrix = p.getMatrixFromQuaternion(pose_orn[1])
        get_velocity = p.getBaseVelocity(boxId)
        get_invert = p.invertTransform(pose_orn[0], pose_orn[1])

        # IMU data
        imu_msg.orientation.x = pose_orn[1][0]
        imu_msg.orientation.y = pose_orn[1][1]
        imu_msg.orientation.z = pose_orn[1][2]
        imu_msg.orientation.w = pose_orn[1][3]

        imu_msg.angular_velocity.x = get_velocity[1][0]
        imu_msg.angular_velocity.y = get_velocity[1][1]
        imu_msg.angular_velocity.z = get_velocity[1][2]

        # calculate the acceleration of the robot
        imu_msg.linear_acceleration.x = (get_velocity[0][0] - get_last_vel[0]) * 200
        imu_msg.linear_acceleration.y = (get_velocity[0][1] - get_last_vel[1]) * 200
        imu_msg.linear_acceleration.z = 9.8 - (get_velocity[0][2] - get_last_vel[2]) * 200
        # print(get_euler)


        # joint data
        joint_state = p.getJointStates(boxId, motor_id_list)
        setJSMsg.position = [joint_state[0][0], joint_state[1][0], joint_state[2][0],
                             joint_state[3][0], joint_state[4][0], joint_state[5][0],
                             joint_state[6][0], joint_state[7][0], joint_state[8][0],
                             joint_state[9][0], joint_state[10][0], joint_state[11][0]]

        setJSMsg.velocity = [joint_state[0][1], joint_state[1][1], joint_state[2][1],
                             joint_state[3][1], joint_state[4][1], joint_state[5][1],
                             joint_state[6][1], joint_state[7][1], joint_state[8][1],
                             joint_state[9][1], joint_state[10][1], joint_state[11][1]]
        # if iter1 < 5:
        #     print(setJSMsg.position)

        # com data
        com_msg.com_position = [pose_orn[0][0], pose_orn[0][1], pose_orn[0][2]]
        com_msg.com_velocity = [get_velocity[0][0], get_velocity[0][1], get_velocity[0][2]]
        get_last_vel.clear()
        get_last_vel = com_msg.com_velocity
        # print(get_euler)

        # get init pos
        if getMode == 0:
            init_pos = setJSMsg.position

        # stand up control
        if getMode == 2 and len(get_position) == 0:
        # if getMode == 2:
            if iter >= 500:
                iter = 500
            jointTorques.clear()
            middle_target.clear()
            for j in range(12):
                middle_target.append((stand_target[j] - init_pos[j]) * iter / 500 + init_pos[j])
                jointTorques.append(100.0 * (middle_target[j] - joint_state[j][0]) - 1.0 * joint_state[j][1])

            # print(jointTorques)
            p.setJointMotorControlArray(bodyUniqueId=boxId,
                                        jointIndices=motor_id_list,
                                        controlMode=p.TORQUE_CONTROL,
                                        forces=jointTorques)
            # p.setJointMotorControlArray(bodyUniqueId=boxId,
            #                             jointIndices=motor_id_list,
            #                             controlMode=p.POSITION_CONTROL,
            #                             targetPositions=middle_target)
            iter = iter + 1

        if getMode == 2 and len(get_position):
            jointTorques1.clear()
            for j in range(12):
                jointTorques1.append(joint_Kp[j] * (get_position[j] - joint_state[j][0]) - joint_Kd[j] * joint_state[j][1] + get_effort[j])

            # for i in range(6):
            #     jointTorques1.append(50 * (get_position[i+6] - joint_state[i+6][0]) - 0.25 * joint_state[i+6][1] + get_effort[i+6])

            p.setJointMotorControlArray(boxId,
                                        jointIndices=motor_id_list,
                                        controlMode=p.TORQUE_CONTROL,
                                        forces=jointTorques1)
            # if iter1 < 10:
            #     print(jointTorques1)
            # pos_gain = [0, 0, 0, 0,  0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            # vel_gain = [0, 0, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            # p.setJointMotorControlArray(bodyUniqueId=boxId,
            #                             jointIndices=motor_id_list,
            #                             controlMode=p.POSITION_CONTROL,
            #                             targetPositions=get_position)
                                        # positionGains=pos_gain,
                                        # velocityGains=vel_gain)

            iter1 = iter1 + 1
            print(jointTorques1[8])

        setJSMsg.header.stamp = rospy.Time.now()
        setJSMsg.name = ["abduct_fr", "thigh_fr", "knee_fr", "abduct_fl", "thigh_fl", "knee_fl",
                         "abduct_hr", "thigh_hr", "knee_hr", "abduct_hl", "thigh_hl", "knee_hl"]

        pub1.publish(setJSMsg)
        pub2.publish(imu_msg)
        pub3.publish(com_msg)

        myflags = myflags + 1
        # p.setRealTimeSimulation(1)
        p.setTimeStep(0.002)
        p.stepSimulation()
        rate.sleep()


if __name__ == '__main__':
    init_simulation()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("set_js", JointState, callback_state, buff_size=10000)
    s = rospy.Service('set_jm', QuadrupedCmd, callback_mode)
    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    talker()
    # rospy.spin()

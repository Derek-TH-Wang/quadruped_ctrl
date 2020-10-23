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
import random

getMode = 10
get_position = []
get_effort = []
get_velocity = []
get_last_vel = [0.0, 0.0, 0.0]
myflags = 0
firstflags = 0

def init_simulation():
    global boxId, motor_id_list, compensateReal, get_last_euler
    get_last_euler = [0.0, 0.0, 0.0]
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
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
      planeShape = p.createCollisionShape(shapeType = p.GEOM_PLANE)
      ground_id  = p.createMultiBody(0, planeShape)
    else:
      numHeightfieldRows = 256
      numHeightfieldColumns = 256
      heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
      for j in range (int(numHeightfieldColumns/2)):
        for i in range (int(numHeightfieldRows/2) ):
          height = random.uniform(0,heightPerturbationRange)
          heightfieldData[2*i+2*j*numHeightfieldRows]=height
          heightfieldData[2*i+1+2*j*numHeightfieldRows]=height
          heightfieldData[2*i+(2*j+1)*numHeightfieldRows]=height
          heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows]=height
    
      terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.05,.05,1], heightfieldTextureScaling=(numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
      ground_id  = p.createMultiBody(0, terrainShape)

    p.resetBasePositionAndOrientation(ground_id,[0,0,0], [0,0,0,1])
    p.changeDynamics(ground_id, -1, lateralFriction=1.0)


    boxId = p.loadURDF("mini_cheetah/mini_cheetah.urdf", cubeStartPos,
                       useFixedBase=False)

    jointIds = []
    for j in range(p.getNumJoints(boxId)):
        info = p.getJointInfo(boxId, j)
        jointIds.append(j)

    jointConfig = numpy.array([-0.7, -1.0, 2.7, 0.7, -1.0, 2.7, -0.7, -1.0, 2.7, 0.7, -1.0, 2.7])
    # init_new_pos = [-0.0, -1.4, 2.7, 0.0, -1.4, 2.7, -0.0, -1.4, 2.7, 0.0, -1.4, 2.7]
    init_new_pos = [0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6]
    for j in range(12):
        p.setJointMotorControl2(boxId, motor_id_list[j], p.POSITION_CONTROL, init_new_pos[j], force=5.0)


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

    #many box
    colSphereId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
    colSphereId4 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
    BoxId = p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
    BoxId = p.createMultiBody(100, colSphereId1, basePosition=[1.2, 1.0, 0.0])
    BoxId = p.createMultiBody(100, colSphereId2, basePosition=[1.4, 1.0, 0.0])
    BoxId = p.createMultiBody(100, colSphereId3, basePosition=[1.6, 1.0, 0.0])
    BoxId = p.createMultiBody(10, colSphereId4, basePosition=[2.7, 1.0, 0.0])



def thread_job():
    rospy.spin()


def callback_state(msg):
    global getMode, get_position, get_effort
    get_position.clear()
    get_effort.clear()

    get_position.append(msg.position[0])
    get_position.append(msg.position[1])
    get_position.append(msg.position[2])
    get_position.append(msg.position[3])
    get_position.append(msg.position[4])
    get_position.append(msg.position[5])
    get_position.append(msg.position[6])
    get_position.append(msg.position[7])
    get_position.append(msg.position[8])
    get_position.append(msg.position[9])
    get_position.append(msg.position[10])
    get_position.append(msg.position[11])

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


def callback_mode(req):
    # global getMode
    # getMode = req.cmd
    # print(getMode)
    # print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
    # if getMode == 0:
    #     p.resetBasePositionAndOrientation(boxId, [0, 0, 0.2], [0, 0, 0, 1])
    #     time.sleep(1)
    #     for j in range(16):
    #         force = 0
    #         p.setJointMotorControl2(boxId, j, p.VELOCITY_CONTROL, force=force)  # , positionGain=10, velocityGain=10)
    #         # p.changeDynamics(quadruped, j, spinningFriction=0.01, rollingFriction=0.01, jointDamping=1.0)
    #         # p.changeDynamics(quadruped, j, jointDamping=0.5)

    return QuadrupedCmdResponse(0, "get the mode")


def thread_job():
    rospy.spin()


def acc_filter(value, last_accValue):
    a = 1.0
    filter_value = a * value + (1 - a) * last_accValue
    return filter_value


def talker():
    global getMode, get_last_vel, myflags, firstflags
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
    stand_target = [0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6]
    pub1 = rospy.Publisher('/get_js', JointState, queue_size=100)
    pub2 = rospy.Publisher('/imu_body', Imu, queue_size=100)
    pub3 = rospy.Publisher('/get_com', commandDes, queue_size=100)
    imu_msg = Imu()
    setJSMsg = JointState()
    com_msg = commandDes()
    freq = 400
    rate = rospy.Rate(freq)  # hz

    


    while not rospy.is_shutdown():

        # if myflags == 100:
        #     p.resetBasePositionAndOrientation(boxId, [0, 0, 0.3], [0, 0, 0, 1])
        #     time.sleep(1)
        #     for j in range(16):
        #         force = 0
        #         p.setJointMotorControl2(boxId, j, p.VELOCITY_CONTROL, force=force)

        get_orientation = []
        pose_orn = p.getBasePositionAndOrientation(boxId)
        for i in range(4):
            get_orientation.append(pose_orn[1][i])
        get_euler = p.getEulerFromQuaternion(get_orientation)
        get_velocity = p.getBaseVelocity(boxId)
        get_invert = p.invertTransform(pose_orn[0], pose_orn[1])
        get_matrix = p.getMatrixFromQuaternion(get_invert[1])

        # IMU data
        imu_msg.orientation.x = pose_orn[1][0]
        imu_msg.orientation.y = pose_orn[1][1]
        imu_msg.orientation.z = pose_orn[1][2]
        imu_msg.orientation.w = pose_orn[1][3]

        imu_msg.angular_velocity.x = get_matrix[0] * get_velocity[1][0] + get_matrix[1] * get_velocity[1][1] + get_matrix[2] * get_velocity[1][2]
        imu_msg.angular_velocity.y = get_matrix[3] * get_velocity[1][0] + get_matrix[4] * get_velocity[1][1] + get_matrix[5] * get_velocity[1][2]
        imu_msg.angular_velocity.z = get_matrix[6] * get_velocity[1][0] + get_matrix[7] * get_velocity[1][1] + get_matrix[8] * get_velocity[1][2]

        # calculate the acceleration of the robot
        linear_X = (get_velocity[0][0] - get_last_vel[0]) * freq
        linear_Y = (get_velocity[0][1] - get_last_vel[1]) * freq
        linear_Z = 9.8 + (get_velocity[0][2] - get_last_vel[2]) * freq
        imu_msg.linear_acceleration.x = get_matrix[0] * linear_X + get_matrix[1] * linear_Y + get_matrix[2] * linear_Z
        imu_msg.linear_acceleration.y = get_matrix[3] * linear_X + get_matrix[4] * linear_Y + get_matrix[5] * linear_Z
        imu_msg.linear_acceleration.z = get_matrix[6] * linear_X + get_matrix[7] * linear_Y + get_matrix[8] * linear_Z


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

        # com data
        com_msg.com_position = [pose_orn[0][0], pose_orn[0][1], pose_orn[0][2]]
        com_msg.com_velocity = [get_velocity[0][0], get_velocity[0][1], get_velocity[0][2]]
        get_last_vel.clear()
        get_last_vel = com_msg.com_velocity

        # if myflags < 1000:
        #   for j in range(12):
        #     force = 0
        #     p.setJointMotorControl2(boxId, j, p.VELOCITY_CONTROL, force=force)
          
        #   for m in range(12):
        #     p.resetJointState(boxId, motor_id_list[m], stand_target[m], targetVelocity=0)

        # stand up control
        if len(get_effort):
            # print(get_effort)
            if firstflags < 1:
              for j in range(16):
                force = 0
                p.setJointMotorControl2(boxId, j, p.VELOCITY_CONTROL, force=force)
            p.setJointMotorControlArray(bodyUniqueId=boxId,
                                        jointIndices=motor_id_list,
                                        controlMode=p.TORQUE_CONTROL,
                                        forces=get_effort)
            firstflags = firstflags + 1

        setJSMsg.header.stamp = rospy.Time.now()
        setJSMsg.name = ["abduct_fr", "thigh_fr", "knee_fr", "abduct_fl", "thigh_fl", "knee_fl",
                         "abduct_hr", "thigh_hr", "knee_hr", "abduct_hl", "thigh_hl", "knee_hl"]
        if myflags % 2 == 0:
            pub2.publish(imu_msg)

        pub1.publish(setJSMsg)# p.changeDynamics(planeId, -1, lateralFriction=1.0)
        pub3.publish(com_msg)
        myflags = myflags + 1
        p.setTimeStep(0.0015)
        p.stepSimulation()
        rate.sleep()


if __name__ == '__main__':
    init_simulation()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("set_js", JointState, callback_state, buff_size=10000)
    s = rospy.Service('robot_mode', QuadrupedCmd, callback_mode)
    add_thread = threading.Thread(target=thread_job)
    add_thread.start()
    talker()

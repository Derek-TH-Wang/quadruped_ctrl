#!/usr/bin/env python

import os
import numpy
import pyquaternion
import pcl
import tf2_ros
import rospy
import rospkg
import threading
import random
import ctypes
from PIL import Image as pil
import pybullet as p
import pybullet_data
from pybullet_utils import gazebo_world_parser
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from quadruped_ctrl.srv import QuadrupedCmd, QuadrupedCmdResponse
from whole_body_state_msgs.msg import WholeBodyState
from whole_body_state_msgs.msg import JointState as WBJointState
from whole_body_state_msgs.msg import ContactState as WBContactState


get_last_vel = [0] * 3
robot_height = 0.30
motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
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
    return QuadrupedCmdResponse(0, "get the gait")


def callback_mode(req):
    cpp_gait_ctrller.set_robot_mode(convert_type(req.cmd))
    return QuadrupedCmdResponse(0, "get the mode")


def callback_body_vel(msg):
    vel = [msg.linear.x, msg.linear.y, msg.angular.x]
    cpp_gait_ctrller.set_robot_vel(convert_type(vel))


def acc_filter(value, last_accValue):
    a = 1
    filter_value = a * value + (1 - a) * last_accValue
    return filter_value


def fill_tf_message(parent_frame, child_frame, translation, rotation):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t


def pub_nav_msg(base_pos, imu_data):
    pub_odom = rospy.Publisher("/robot_odom", Odometry, queue_size=30)
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "world"
    odom.child_frame_id = "body"
    odom.pose.pose.position.x = base_pos[0]
    odom.pose.pose.position.y = base_pos[1]
    odom.pose.pose.position.z = base_pos[2]
    odom.pose.pose.orientation.x = imu_data[3]
    odom.pose.pose.orientation.y = imu_data[4]
    odom.pose.pose.orientation.z = imu_data[5]
    odom.pose.pose.orientation.w = imu_data[6]

    pub_odom.publish(odom)

    t = fill_tf_message(odom.header.frame_id, odom.child_frame_id, base_pos[0:3], imu_data[3:7])
    robot_tf.sendTransform(t)


def pub_imu_msg(imu_data):
    pub_imu = rospy.Publisher("/imu0", Imu, queue_size=30)
    imu_msg = Imu()
    imu_msg.linear_acceleration.x = imu_data[0]
    imu_msg.linear_acceleration.y = imu_data[1]
    imu_msg.linear_acceleration.z = imu_data[2]
    imu_msg.angular_velocity.x = imu_data[7]
    imu_msg.angular_velocity.y = imu_data[8]
    imu_msg.angular_velocity.z = imu_data[9]
    imu_msg.orientation.x = imu_data[3]
    imu_msg.orientation.y = imu_data[4]
    imu_msg.orientation.z = imu_data[5]
    imu_msg.orientation.w = imu_data[6]
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "body"
    pub_imu.publish(imu_msg)

def pub_joint_states(joint_states):
    pub_js = rospy.Publisher("joint_states", JointState, queue_size=30)
    js_msg = JointState()
    js_msg.name = []
    js_msg.position = []
    js_msg.velocity = []
    i = 0
    for _ in joint_states["index"]:
        js_msg.name.append(joint_states["name"][i].decode('utf-8'))
        js_msg.position.append(joint_states["state"][i])
        js_msg.velocity.append(joint_states["state"][12+i])
        i += 1
    js_msg.header.stamp = rospy.Time.now()
    js_msg.header.frame_id = "body"
    pub_js.publish(js_msg)

def pub_whole_body_state(imu_data, leg_data, base_pos, contact_points):
    wbs_pub = rospy.Publisher("wb_state", WholeBodyState, queue_size=10)
    wbs = WholeBodyState()
    wbs.header.stamp = rospy.Time.now()
    wbs.header.frame_id = "world"
    wbs.time = wbs.header.stamp.secs
    # This represents the base state (CoM motion, angular motion and centroidal momenta)
    wbs.centroidal.com_position.x = base_pos[0]
    wbs.centroidal.com_position.y = base_pos[1]
    wbs.centroidal.com_position.z = base_pos[2]
    wbs.centroidal.base_orientation.x = imu_data[3]
    wbs.centroidal.base_orientation.y = imu_data[4]
    wbs.centroidal.base_orientation.z = imu_data[5]
    wbs.centroidal.base_orientation.w = imu_data[6]
    wbs.centroidal.base_angular_velocity.x = imu_data[7]
    wbs.centroidal.base_angular_velocity.y = imu_data[8]
    wbs.centroidal.base_angular_velocity.z = imu_data[9]
    # This represents the joint state (position, velocity, acceleration and effort)
    wbs.joints = []
    i = 0
    for _ in leg_data["index"]:
        js_msg = WBJointState()
        js_msg.name = leg_data["name"][i].decode('utf-8')
        js_msg.position = leg_data["state"][i]
        js_msg.velocity = leg_data["state"][12+i]
        wbs.joints.append(js_msg)
        i += 1
    # This represents the end-effector state (cartesian position and contact forces)
    wbs.contacts = []
    for contact_point in contact_points:
        contact_msg = WBContactState()
        contact_msg.name = "body"
        contact_msg.type = WBContactState.UNKNOWN
        contact_msg.pose.position.x = contact_point[5][0]
        contact_msg.pose.position.y = contact_point[5][1]
        contact_msg.pose.position.z = contact_point[5][2]
        contact_msg.wrench.force.z = contact_point[9]
        contact_msg.surface_normal.x = contact_point[7][0]
        contact_msg.surface_normal.y = contact_point[7][1]
        contact_msg.surface_normal.z = contact_point[7][2]
        contact_msg.friction_coefficient = 1.0
        wbs.contacts.append(contact_msg)
    wbs_pub.publish(wbs)

def get_motor_joint_states(robot):
    joint_number_range = range(p.getNumJoints(robot))
    joint_states = p.getJointStates(robot, joint_number_range)
    joint_infos = [p.getJointInfo(robot, i) for i in joint_number_range]
    joint_states, joint_name, joint_index = zip(*[(j, i[1], i[0]) for j, i in zip(joint_states, joint_infos) if i[2] != p.JOINT_FIXED])
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques, joint_name, joint_index

def get_data_from_sim():
    global get_last_vel
    get_matrix = []
    get_velocity = []
    get_invert = []
    imu_data = [0] * 10
    leg_data = {}
    leg_data["state"] = [0] * 24
    leg_data["name"] = [""] * 12
    leg_data["index"] = [0] * 12

    base_pose = p.getBasePositionAndOrientation(boxId)

    get_velocity = p.getBaseVelocity(boxId)
    get_invert = p.invertTransform(base_pose[0], base_pose[1])
    get_matrix = p.getMatrixFromQuaternion(get_invert[1])

    # IMU data
    imu_data[3] = base_pose[1][0]
    imu_data[4] = base_pose[1][1]
    imu_data[5] = base_pose[1][2]
    imu_data[6] = base_pose[1][3]

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
    joint_positions, joint_velocities, _, joint_names, joint_index = get_motor_joint_states(boxId)
    leg_data["state"][0:12] = joint_positions
    leg_data["state"][12:24] = joint_velocities
    leg_data["name"] = joint_names
    leg_data["index"] = joint_index

    com_velocity = [get_velocity[0][0],
                    get_velocity[0][1], get_velocity[0][2]]

    get_last_vel = []
    get_last_vel = com_velocity

    # Contacts
    contact_points = p.getContactPoints(boxId)

    return imu_data, leg_data, base_pose[0], contact_points


def reset_robot():
    if terrain == "racetrack":
        robot_z = 0.4
    else:
        robot_z = robot_height
    p.resetBasePositionAndOrientation(
        boxId, [0, 0, robot_z], [0, 0, 0, 1])
    p.resetBaseVelocity(boxId, [0, 0, 0], [0, 0, 0])
    for j in range(12):
        p.resetJointState(boxId, motor_id_list[j], init_new_pos[j], init_new_pos[j+12])
    cpp_gait_ctrller.init_controller(convert_type(
        freq), convert_type([stand_kp, stand_kd, joint_kp, joint_kd]))

    for _ in range(10):
        p.stepSimulation()
        imu_data, leg_data, _, _ = get_data_from_sim()
        cpp_gait_ctrller.pre_work(convert_type(
            imu_data), convert_type(leg_data["state"]))

    for j in range(16):
        force = 0
        p.setJointMotorControl2(
            boxId, j, p.VELOCITY_CONTROL, force=force)

    cpp_gait_ctrller.set_robot_mode(convert_type(1))
    for _ in range(200):
        run()
        p.stepSimulation
    cpp_gait_ctrller.set_robot_mode(convert_type(0))


def init_simulator():
    global boxId, reset, low_energy_mode, high_performance_mode, terrain, p
    robot_start_pos = [0, 0, 0.42]
    p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.resetSimulation()
    p.setTimeStep(1.0/freq)
    p.setGravity(0, 0, -9.8)
    reset = p.addUserDebugParameter("reset", 1, 0, 0)
    low_energy_mode = p.addUserDebugParameter("low_energy_mode", 1, 0, 0)
    high_performance_mode = p.addUserDebugParameter("high_performance_mode", 1, 0, 0)
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])

    heightPerturbationRange = 0.06
    numHeightfieldRows = 256
    numHeightfieldColumns = 256
    if terrain == "plane":
        planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
        ground_id = p.createMultiBody(0, planeShape)
        p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        p.changeDynamics(ground_id, -1, lateralFriction=lateralFriction)
    elif terrain == "random1":
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
        p.changeDynamics(ground_id, -1, lateralFriction=lateralFriction)
    elif terrain == "random2":
        terrain_shape = p.createCollisionShape(
            shapeType=p.GEOM_HEIGHTFIELD,
            meshScale=[.5, .5, .5],
            fileName="heightmaps/ground0.txt",
            heightfieldTextureScaling=128)
        ground_id = p.createMultiBody(0, terrain_shape)
        textureId = p.loadTexture(path+"/models/grass.png")
        p.changeVisualShape(ground_id, -1, textureUniqueId=textureId)
        p.resetBasePositionAndOrientation(ground_id, [1, 0, 0.2], [0, 0, 0, 1])
        p.changeDynamics(ground_id, -1, lateralFriction=lateralFriction)
    elif terrain == "stairs":
        planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
        ground_id = p.createMultiBody(0, planeShape)
        # p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0.0872, 0, 0.9962])
        p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
        # many box
        colSphereId = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
        colSphereId1 = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.02])
        colSphereId2 = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.03])
        colSphereId3 = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.04])
        # colSphereId4 = p.createCollisionShape(
        #     p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
        p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
        p.changeDynamics(colSphereId, -1, lateralFriction=lateralFriction)
        p.createMultiBody(100, colSphereId1, basePosition=[1.2, 1.0, 0.0])
        p.changeDynamics(colSphereId1, -1, lateralFriction=lateralFriction)
        p.createMultiBody(100, colSphereId2, basePosition=[1.4, 1.0, 0.0])
        p.changeDynamics(colSphereId2, -1, lateralFriction=lateralFriction)
        p.createMultiBody(100, colSphereId3, basePosition=[1.6, 1.0, 0.0])
        p.changeDynamics(colSphereId3, -1, lateralFriction=lateralFriction)
        # p.createMultiBody(10, colSphereId4, basePosition=[2.7, 1.0, 0.0])
        # p.changeDynamics(colSphereId4, -1, lateralFriction=0.5)
        p.changeDynamics(ground_id, -1, lateralFriction=lateralFriction)
    elif terrain == "racetrack":
        os.chdir(path)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        gazebo_world_parser.parseWorld(p, filepath = "worlds/racetrack_day.world")
        p.configureDebugVisualizer(shadowMapResolution = 8192)
        p.configureDebugVisualizer(shadowMapWorldSize = 25)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # TODO: Get the URDF from robot_description parameter (or URDF file in the repo)
    boxId = p.loadURDF("mini_cheetah/mini_cheetah.urdf", robot_start_pos, useFixedBase=False)
    p.changeDynamics(boxId, 3, spinningFriction=spinningFriction)
    p.changeDynamics(boxId, 7, spinningFriction=spinningFriction)
    p.changeDynamics(boxId, 11, spinningFriction=spinningFriction)
    p.changeDynamics(boxId, 15, spinningFriction=spinningFriction)

    reset_robot()


def run():
    # get data from simulator
    imu_data, leg_data, base_pos, contact_points = get_data_from_sim()

    # pub msg
    pub_nav_msg(base_pos, imu_data)
    pub_imu_msg(imu_data)
    pub_joint_states(leg_data)
    pub_whole_body_state(imu_data, leg_data, base_pos, contact_points)

    # call cpp function to calculate mpc tau
    tau = cpp_gait_ctrller.torque_calculator(convert_type(
        imu_data), convert_type(leg_data["state"]))

    # set tau to simulator
    p.setJointMotorControlArray(bodyUniqueId=boxId,
                                jointIndices=motor_id_list,
                                controlMode=p.TORQUE_CONTROL,
                                forces=tau.contents.eff)

    p.stepSimulation()

    return


def camera_update():
    rate_1 = rospy.Rate(20)
    near = 0.1
    far = 1000
    step_index = 4
    pixelWidth = int(320/step_index)
    pixelHeight = int(240/step_index)
    cameraEyePosition = [0.3, 0, 0.26436384367425125]
    cameraTargetPosition = [1.0, 0, 0]
    cameraUpVector = [45, 45, 0]
    pub_pointcloud = PointCloud2()
    pub_image = Image()
    pointcloud_publisher = rospy.Publisher("/generated_pc", PointCloud2, queue_size=10)
    image_publisher = rospy.Publisher("/cam0/image_raw", Image, queue_size=10)

    while not rospy.is_shutdown():
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        get_matrix = p.getMatrixFromQuaternion(cubeOrn)

        T1 = numpy.mat([[0, -1.0/2.0, numpy.sqrt(3.0)/2.0, 0.25], [-1, 0, 0, 0],
                        [0, -numpy.sqrt(3.0)/2.0, -1.0/2.0, 0], [0, 0, 0, 1]])

        T2 = numpy.mat([[get_matrix[0], get_matrix[1], get_matrix[2], cubePos[0]],
                        [get_matrix[3], get_matrix[4], get_matrix[5], cubePos[1]],
                        [get_matrix[6], get_matrix[7], get_matrix[8], cubePos[2]],
                        [0, 0, 0, 1]])

        T2_ = (T2.I)
        T3_ = numpy.array(T2*T1)

        cameraEyePosition[0] = T3_[0][3]
        cameraEyePosition[1] = T3_[1][3]
        cameraEyePosition[2] = T3_[2][3]
        cameraTargetPosition = (numpy.mat(T3_)*numpy.array([[0],[0],[1],[1]]))[0:3]

        q = pyquaternion.Quaternion(matrix=T3_)
        cameraQuat = [q[1], q[2], q[3], q[0]]

        robot_tf.sendTransform(fill_tf_message("world", "robot", cubePos, cubeOrn))
        robot_tf.sendTransform(fill_tf_message("world", "cam", cameraEyePosition, cameraQuat))
        robot_tf.sendTransform(fill_tf_message("world", "tar", cameraTargetPosition, cubeOrn))

        cameraUpVector = [0, 0, 1]
        viewMatrix = p.computeViewMatrix(
            cameraEyePosition, cameraTargetPosition, cameraUpVector)
        aspect = float(pixelWidth) / float(pixelHeight)
        projectionMatrix = p.computeProjectionMatrixFOV(60, aspect, near, far)
        width, height, rgbImg, depthImg, _ = p.getCameraImage(pixelWidth,
                                   pixelHeight,
                                   viewMatrix=viewMatrix,
                                   projectionMatrix=projectionMatrix,
                                   shadow=1,
                                   lightDirection=[1, 1, 1],
                                   renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # point cloud mehted
        pc_list = []
        pcl_data = pcl.PointCloud()
        fx = (pixelWidth*projectionMatrix[0])/2.0
        fy = (pixelHeight*projectionMatrix[5])/2.0
        cx = (1-projectionMatrix[2])*pixelWidth/2.0
        cy = (1+projectionMatrix[6])*pixelHeight/2.0
        cloud_point = [0]*pixelWidth*pixelHeight*3
        depthBuffer = numpy.reshape(depthImg,[pixelHeight,pixelWidth])
        depth = depthBuffer
        for h in range(0, pixelHeight):
            for w in range(0, pixelWidth):
                depth[h][w] =float(depthBuffer[h,w])
                depth[h][w] = far * near / (far - (far - near) * depthBuffer[h][w])
                Z= float(depth[h][w])
                if (Z >4):
                    continue
                if (Z< 0.01):
                    continue
                X=(w-cx)*Z/fx
                Y=(h-cy)*Z/fy
                XYZ_= numpy.mat([[X],[Y],[Z],[1]])
                XYZ =numpy.array(T3_*XYZ_)
                # XYZ = numpy.array(XYZ_)
                X= float(XYZ[0])
                Y= float(XYZ[1])
                Z= float(XYZ[2])
                cloud_point[h*pixelWidth*3+w*3+0] = float(X)
                cloud_point[h*pixelWidth*3+w*3+1] = float(Y)
                cloud_point[h*pixelWidth*3+w*3+2] = float(Z)
                pc_list.append([X,Y,Z])

        pcl_data.from_list(pc_list)
        pub_pointcloud.header.stamp = rospy.Time().now()
        pub_pointcloud.header.frame_id = "body"
        pub_pointcloud.height = 1
        pub_pointcloud.width = len(pc_list)
        pub_pointcloud.point_step = 12
        pub_pointcloud.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        pub_pointcloud.data = numpy.asarray(pc_list, numpy.float32).tostring()
        pointcloud_publisher.publish(pub_pointcloud)

        # grey image
        pub_image.header.stamp = rospy.Time().now()
        pub_image.header.frame_id = "cam"
        pub_image.width = width
        pub_image.height = height
        pub_image.encoding = "mono8"
        pub_image.step = width
        grey = pil.fromarray(rgbImg)
        pub_image.data = numpy.asarray(grey.convert('L')).reshape([1,-1]).tolist()[0]
        image_publisher.publish(pub_image)

        rate_1.sleep()


def main():
    cnt = 0
    rate = rospy.Rate(freq)  # hz
    reset_flag = p.readUserDebugParameter(reset)
    low_energy_flag = p.readUserDebugParameter(low_energy_mode)
    high_performance_flag = p.readUserDebugParameter(high_performance_mode)
    while not rospy.is_shutdown():
        # check reset button state
        if(reset_flag < p.readUserDebugParameter(reset)):
            reset_flag = p.readUserDebugParameter(reset)
            rospy.logwarn("reset the robot")
            cnt = 0
            reset_robot()
        if(low_energy_flag < p.readUserDebugParameter(low_energy_mode)):
            low_energy_flag = p.readUserDebugParameter(low_energy_mode)
            rospy.logwarn("set robot to low energy mode")
            cpp_gait_ctrller.set_robot_mode(convert_type(1))
        if(high_performance_flag < p.readUserDebugParameter(high_performance_mode)):
            high_performance_flag = p.readUserDebugParameter(high_performance_mode)
            rospy.logwarn("set robot to high performance mode")
            cpp_gait_ctrller.set_robot_mode(convert_type(0))

        run()

        cnt += 1
        if cnt > 99999999:
            cnt = 99999999
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('quadruped_simulator', anonymous=True)

    terrain = rospy.get_param('/simulation/terrain')
    camera = rospy.get_param('/simulation/camera')
    lateralFriction = rospy.get_param('/simulation/lateralFriction')
    spinningFriction = rospy.get_param('/simulation/spinningFriction')
    freq = rospy.get_param('/simulation/freq')
    stand_kp = rospy.get_param('/simulation/stand_kp')
    stand_kd = rospy.get_param('/simulation/stand_kd')
    joint_kp = rospy.get_param('/simulation/joint_kp')
    joint_kd = rospy.get_param('/simulation/joint_kd')
    rospy.loginfo("lateralFriction = " + str(lateralFriction) + " spinningFriction = " + str(spinningFriction))
    rospy.loginfo(" freq = " + str(freq) + " PID = " + str([stand_kp, stand_kd, joint_kp, joint_kd]))

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
    cpp_gait_ctrller.torque_calculator.restype = ctypes.POINTER(StructPointer)
    rospy.loginfo("find so file = " + so_file)

    s = rospy.Service('gait_type', QuadrupedCmd, callback_gait)
    s1 = rospy.Service('robot_mode', QuadrupedCmd, callback_mode)
    rospy.Subscriber("cmd_vel", Twist, callback_body_vel, buff_size=10000)

    global robot_tf
    robot_tf = tf2_ros.TransformBroadcaster()

    init_simulator()

    add_thread = threading.Thread(target=thread_job)
    add_thread.start()

    if camera:
        add_thread_1 = threading.Thread(target=camera_update)
        add_thread_1.start()

    main()

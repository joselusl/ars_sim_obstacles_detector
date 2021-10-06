#!/usr/bin/env python

import numpy as np
from numpy import *

import os

import copy




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header


import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



import tf_conversions

import tf2_ros




#
import ars_lib_helpers





class ArsSimObstaclesDetectorRos:

  #######

  # Robot frame
  robot_frame = None

  # Detector range
  detector_range = None

  # Covariance on measurement of position
  cov_meas_pos = None

  # Covariance on measurement of sizes
  cov_meas_siz = None

  # Robot pose subscriber
  robot_pose_sub = None

  # Obstacles static sub
  obstacles_static_sub = None
  
  # Obstacles dynamic sub
  obstacles_dynamic_sub = None

  # Obstacles detected pub
  flag_pub_obstacles_detected_world = False
  obstacles_detected_world_pub = None
  obstacles_detected_robot_pub = None


  # Robot Pose
  flag_robot_pose_set = None
  robot_posi = None
  robot_atti_quat_simp = None

  # Obstacles static
  obstacles_static_msg = None

  # Obstacles dynamic
  obstacles_dynamic_msg = None

  # Obstacles detected
  obstacles_detected_world_msg = None
  obstacles_detected_robot_msg = None


  # Obstacle Detection loop
  # freq
  obstacle_detect_loop_freq = None
  # Timer
  obstacle_detect_loop_timer = None
  


  #########

  def __init__(self):

    # Robot frame
    self.robot_frame = 'robot_base_link'

    # Robot size radius
    self.detector_range = 2.0

    # Covariance on measurement of position
    self.cov_meas_pos = {'x': 0.0001, 'y': 0.0001, 'z': 0.000001}

    # Covariance on measurement of sizes
    self.cov_meas_siz = {'R': 0.0001, 'h': 0.000001}

    #
    self.flag_robot_pose_set = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    #
    self.obstacles_static_msg = MarkerArray()

    #
    self.obstacles_dynamic_msg = MarkerArray()

    #
    self.obstacles_detected_world_msg = MarkerArray()
    self.obstacles_detected_robot_msg = MarkerArray()


    # Obstacle Detection loop
    # freq
    self.obstacle_detect_loop_freq = 10.0
    # Timer
    self.obstacle_detect_loop_timer = None


    # end
    return


  def init(self, node_name='ars_sim_obstacles_detector_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_obstacles_detector')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback)
    
    # 
    self.obstacles_static_sub = rospy.Subscriber('obstacles_static', MarkerArray, self.obstaclesStaticCallback)
    #
    self.obstacles_dynamic_sub = rospy.Subscriber('obstacles_dynamic', MarkerArray, self.obstaclesDynamicCallback)


    # Publishers

    # 
    if(self.flag_pub_obstacles_detected_world):
      self.obstacles_detected_world_pub = rospy.Publisher('obstacles_detected_world', MarkerArray, queue_size=1)
    # 
    self.obstacles_detected_robot_pub = rospy.Publisher('obstacles_detected_robot', MarkerArray, queue_size=1)



    # Timers
    #
    self.obstacle_detect_loop_timer = rospy.Timer(rospy.Duration(1.0/self.obstacle_detect_loop_freq), self.obstacleDetectorLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotPoseCallback(self, robot_pose_msg):

    #
    self.flag_robot_pose_set = True

    # Position
    self.robot_posi[0] = robot_pose_msg.pose.position.x
    self.robot_posi[1] = robot_pose_msg.pose.position.y
    self.robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.detectObstacles()
    
    #
    return



  def obstaclesStaticCallback(self, obstacles_static_msg):

    self.obstacles_static_msg = obstacles_static_msg

    #
    return



  def obstaclesDynamicCallback(self, obstacles_dynamic_msg):

    self.obstacles_dynamic_msg = obstacles_dynamic_msg

    #
    return



  def detectObstacles(self):

    #
    self.obstacles_detected_world_msg = MarkerArray()
    self.obstacles_detected_world_msg.markers = []

    #
    self.obstacles_detected_robot_msg = MarkerArray()
    self.obstacles_detected_robot_msg.markers = []

    #
    robot_atti_rot_mat = ars_lib_helpers.Quaternion.rotMat3dFromQuatSimp(self.robot_atti_quat_simp)


    # Check
    if(self.flag_robot_pose_set):

      # Obstacles static
      for obst_i_msg in self.obstacles_static_msg.markers:

        if(obst_i_msg.action == 0):

          # Check distance
          if(obst_i_msg.type == 3):

            obst_i_posi_world = np.zeros((3,), dtype=float)
            obst_i_posi_world[0] = obst_i_msg.pose.position.x
            obst_i_posi_world[1] = obst_i_msg.pose.position.y
            obst_i_posi_world[2] = obst_i_msg.pose.position.z

            obst_i_rad = obst_i_msg.scale.x/2.0

            distance = ars_lib_helpers.distancePointCircle(self.robot_posi[0:2], obst_i_posi_world[0:2], obst_i_rad)

            #
            if(distance <= self.detector_range):

              
              # Noises
              #
              posi_noise = np.zeros((3,), dtype=float)
              posi_noise[0] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['x']))
              posi_noise[1] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['y']))
              posi_noise[2] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['z']))
              #
              radius_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_siz['R']))
              height_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_siz['h']))
          


              ############
              # obstacle wrt World 
              obst_i_world_msg = []
              obst_i_world_msg = copy.deepcopy(obst_i_msg)

              # Change color
              obst_i_world_msg.color.r = 0.0
              obst_i_world_msg.color.g = 0.0
              obst_i_world_msg.color.b = 1.0
              obst_i_world_msg.color.a = 0.6

              # Lifetime
              obst_i_world_msg.lifetime = rospy.Duration(2.0*1.0/self.obstacle_detect_loop_freq)

              #
              obst_i_world_msg.pose.position.x = obst_i_posi_world[0] + posi_noise[0]
              obst_i_world_msg.pose.position.y = obst_i_posi_world[1] + posi_noise[1]
              obst_i_world_msg.pose.position.z = obst_i_posi_world[2] + posi_noise[2]

              # Sizes with noise
              obst_i_world_msg.scale.x += 2.0*radius_noise
              obst_i_world_msg.scale.y += 2.0*radius_noise
              obst_i_world_msg.scale.z += height_noise



              ##############
              # obstacle wrt Robot 
              obst_i_robot_msg = []
              obst_i_robot_msg = copy.deepcopy(obst_i_msg)

              # Change color
              obst_i_robot_msg.color.r = 0.0
              obst_i_robot_msg.color.g = 0.0
              obst_i_robot_msg.color.b = 1.0
              obst_i_robot_msg.color.a = 0.6

              # Lifetime
              obst_i_robot_msg.lifetime = rospy.Duration(2.0*1.0/self.obstacle_detect_loop_freq)
              
              # Change to robot coordinates
              # t_O_R = (R_R_W)^T * (t_O_W - t_R_W)
              # R_O_R = (R_R_W)^T * R_O_W
              #
              obst_i_posi_robot = np.matmul(robot_atti_rot_mat.T, (obst_i_posi_world-self.robot_posi))

              #
              obst_i_robot_msg.pose.position.x = obst_i_posi_robot[0] + posi_noise[0]
              obst_i_robot_msg.pose.position.y = obst_i_posi_robot[1] + posi_noise[1]
              obst_i_robot_msg.pose.position.z = obst_i_posi_robot[2] + posi_noise[2]


              # Change frame
              obst_i_robot_msg.header.frame_id = self.robot_frame

              # Sizes with noise
              obst_i_robot_msg.scale.x += 2.0*radius_noise
              obst_i_robot_msg.scale.y += 2.0*radius_noise
              obst_i_robot_msg.scale.z += height_noise


              

              # Append world
              self.obstacles_detected_world_msg.markers.append(obst_i_world_msg)

              # Append robot
              self.obstacles_detected_robot_msg.markers.append(obst_i_robot_msg)


          else:
            print("Unknown obstacle type:"+obst_i_msg.type)



      # Obstacles dynamic
      for obst_i_msg in self.obstacles_dynamic_msg.markers:

        if(obst_i_msg.action == 0):

          # Check distance
          if(obst_i_msg.type == 3):

            # TODO: Change to robot coordinates
            # t_O_R = (R_R_W)^T * (t_O_R - t_R_W)
            # R_O_R = (R_R_W)^T * R_O_W

            obst_i_posi_world = np.zeros((3,), dtype=float)
            obst_i_posi_world[0] = obst_i_msg.pose.position.x
            obst_i_posi_world[1] = obst_i_msg.pose.position.y
            obst_i_posi_world[2] = obst_i_msg.pose.position.z

            obst_i_rad = obst_i_msg.scale.x/2.0

            distance = ars_lib_helpers.distancePointCircle(self.robot_posi[0:2], obst_i_posi_world[0:2], obst_i_rad)
            #distance = np.linalg.norm(obst_i_posi_world-self.robot_posi)

            if(distance <= self.detector_range):

              # Noises
              #
              posi_noise = np.zeros((3,), dtype=float)
              posi_noise[0] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['x']))
              posi_noise[1] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['y']))
              posi_noise[2] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_pos['z']))
              #
              radius_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_siz['R']))
              height_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_meas_siz['h']))
          


              ############
              # obstacle wrt World 
              obst_i_world_msg = []
              obst_i_world_msg = copy.deepcopy(obst_i_msg)

              # Change color
              obst_i_world_msg.color.r = 0.0
              obst_i_world_msg.color.g = 0.0
              obst_i_world_msg.color.b = 1.0
              obst_i_world_msg.color.a = 0.6

              # Lifetime
              obst_i_world_msg.lifetime = rospy.Duration(2.0*1.0/self.obstacle_detect_loop_freq)

              #
              obst_i_world_msg.pose.position.x = obst_i_posi_world[0] + posi_noise[0]
              obst_i_world_msg.pose.position.y = obst_i_posi_world[1] + posi_noise[1]
              obst_i_world_msg.pose.position.z = obst_i_posi_world[2] + posi_noise[2]

              # Sizes with noise
              obst_i_world_msg.scale.x += 2.0*radius_noise
              obst_i_world_msg.scale.y += 2.0*radius_noise
              obst_i_world_msg.scale.z += height_noise



              ##############
              # obstacle wrt Robot 
              obst_i_robot_msg = []
              obst_i_robot_msg = copy.deepcopy(obst_i_msg)

              # Change color
              obst_i_robot_msg.color.r = 0.0
              obst_i_robot_msg.color.g = 0.0
              obst_i_robot_msg.color.b = 1.0
              obst_i_robot_msg.color.a = 0.6

              # Lifetime
              obst_i_robot_msg.lifetime = rospy.Duration(2.0*1.0/self.obstacle_detect_loop_freq)
              
              # Change to robot coordinates
              # t_O_R = (R_R_W)^T * (t_O_W - t_R_W)
              # R_O_R = (R_R_W)^T * R_O_W
              #
              obst_i_posi_robot = np.matmul(robot_atti_rot_mat.T, (obst_i_posi_world-self.robot_posi))

              #
              obst_i_robot_msg.pose.position.x = obst_i_posi_robot[0] + posi_noise[0]
              obst_i_robot_msg.pose.position.y = obst_i_posi_robot[1] + posi_noise[1]
              obst_i_robot_msg.pose.position.z = obst_i_posi_robot[2] + posi_noise[2]


              # Change frame
              obst_i_robot_msg.header.frame_id = self.robot_frame

              # Sizes with noise
              obst_i_robot_msg.scale.x += 2.0*radius_noise
              obst_i_robot_msg.scale.y += 2.0*radius_noise
              obst_i_robot_msg.scale.z += height_noise


              

              # Append world
              self.obstacles_detected_world_msg.markers.append(obst_i_world_msg)

              # Append robot
              self.obstacles_detected_robot_msg.markers.append(obst_i_robot_msg)
              

          else:
            print("Unknown obstacle type!!")


    # Publish
    if(self.flag_pub_obstacles_detected_world):
      self.obstacles_detected_world_pub.publish(self.obstacles_detected_world_msg)
    self.obstacles_detected_robot_pub.publish(self.obstacles_detected_robot_msg)

    #
    return


  def obstacleDetectorLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.detectObstacles()

    #
    return

#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header


import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import visualization_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker



import tf_conversions

import tf2_ros




#
import ars_lib_helpers





class ArsSimObstaclesDetectorRos:

  #######


  # Robot frame
  robot_frame = 'robot_base_link'


  # Detector range
  detector_range = 2.0


  # Robot pose subscriber
  robot_pose_sub = None

  # Obstacles static sub
  obstacles_static_sub = None
  
  # Obstacles dynamic sub
  obstacles_dynamic_sub = None


  # Obstacles detected pub
  obstacles_detected_pub = None



  

  # Robot Pose
  flag_robot_pose_set = False
  robot_posi = None
  robot_atti_quat_simp = None

  # Obstacles static
  obstacles_static_msg = None

  # Obstacles dynamic
  obstacles_dynamic_msg = None

  # Obstacles detected
  obstacles_detected_msg = None
  


  #########

  def __init__(self):


    # Robot size radius
    self.detector_range = 2.0

    #
    self.flag_robot_pose_set = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    #
    self.obstacles_static_msg = MarkerArray()

    #
    self.obstacles_dynamic_msg = MarkerArray()


    #
    self.obstacles_detected_msg = MarkerArray()


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
    self.obstacles_detected_pub = rospy.Publisher('obstacles_detected', MarkerArray, queue_size=1)


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
    self.obstacles_detected_msg = MarkerArray()
    self.obstacles_detected_msg.markers = []


    # Check
    if(self.flag_robot_pose_set):

      # Obstacles static
      for obst_i_msg in self.obstacles_static_msg.markers:

        if(obst_i_msg.action == 0):

          # Check distance
          if(obst_i_msg.type == 3):

            obst_i_posi = np.zeros((3,), dtype=float)
            obst_i_posi[0] = obst_i_msg.pose.position.x
            obst_i_posi[1] = obst_i_msg.pose.position.y
            obst_i_posi[2] = obst_i_msg.pose.position.z

            obst_i_rad = obst_i_msg.scale.x/2.0

            distance = ars_lib_helpers.distancePointCircle(self.robot_posi[0:2], obst_i_posi[0:2], obst_i_rad)
            #distance = np.linalg.norm(obst_i_posi-self.robot_posi)

            if(distance <= self.detector_range):
              
              # TODO: Change to robot coordinates
              # t_O_R = (R_R_W)^T * (t_O_R - t_R_W)
              # R_O_R = (R_R_W)^T * R_O_W

              # Change color
              obst_i_msg.color.r = 0.0
              obst_i_msg.color.g = 0.0
              obst_i_msg.color.b = 1.0
              obst_i_msg.color.a = 0.6

              # Lifetime
              obst_i_msg.lifetime = rospy.Duration(0.1)

              #
              self.obstacles_detected_msg.markers.append(obst_i_msg)

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

            obst_i_posi = np.zeros((3,), dtype=float)
            obst_i_posi[0] = obst_i_msg.pose.position.x
            obst_i_posi[1] = obst_i_msg.pose.position.y
            obst_i_posi[2] = obst_i_msg.pose.position.z

            obst_i_rad = obst_i_msg.scale.x/2.0

            distance = ars_lib_helpers.distancePointCircle(self.robot_posi[0:2], obst_i_posi[0:2], obst_i_rad)
            #distance = np.linalg.norm(obst_i_posi-self.robot_posi)

            if(distance <= self.detector_range):

              # Change color
              obst_i_msg.color.r = 0.0
              obst_i_msg.color.g = 0.0
              obst_i_msg.color.b = 1.0
              obst_i_msg.color.a = 0.6

              # Lifetime
              obst_i_msg.lifetime = rospy.Duration(0.1)

              #
              self.obstacles_detected_msg.markers.append(obst_i_msg)

          else:
            print("Unknown obstacle type!!")



    # Publish

    self.obstacles_detected_pub.publish(self.obstacles_detected_msg)



    #
    return
import numpy as np
from numpy import *

import os

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader

import copy


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from builtin_interfaces.msg import Duration


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers



class ArsSimObstaclesDetectorRos(Node):

  #######

  #
  sim_obstacles_detector_params_yaml_file_name = None

  #
  sim_obstacles_detector = None

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

  def __init__(self, node_name='ars_sim_obstacles_detector_node'):
    # Init ROS
    super().__init__(node_name)
    
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


    #
    self.__init(node_name)

    # end
    return


  def __init(self, node_name='ars_sim_obstacles_detector_node'):
       
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_sim_obstacles_detector')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")


    #### READING PARAMETERS ###
    
    # Obstacle detector params
    default_sim_obstacles_detector_params_yaml_file_name = os.path.join(pkg_path,'config','config_sim_obstacles_detector.yaml')
    # Declare the parameter with a default value
    self.declare_parameter('sim_obstacles_detector_params_yaml_file', default_sim_obstacles_detector_params_yaml_file_name)
    # Get the parameter value
    sim_obstacles_detector_params_yaml_file_name_str = self.get_parameter('sim_obstacles_detector_params_yaml_file').get_parameter_value().string_value
    self.get_logger().info(sim_obstacles_detector_params_yaml_file_name_str)
    #
    self.sim_obstacles_detector_params_yaml_file_name = os.path.abspath(sim_obstacles_detector_params_yaml_file_name_str)

    ###


    # Load Obstacle detector params
    with open(self.sim_obstacles_detector_params_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.sim_obstacles_detector = yaml.load(file, Loader=SafeLoader)['sim_obstacles_detector']

    if(self.sim_obstacles_detector is None):
      self.get_logger().info("Error loading sim obstacles detector param")
    else:
      self.get_logger().info("Sim obstacles detector parameters:")
      self.get_logger().info(str(self.sim_obstacles_detector))

    # Set params
    self.obstacle_detect_loop_freq = self.sim_obstacles_detector['obstacle_detect_loop_freq']
    self.robot_frame = self.sim_obstacles_detector['robot_frame']
    self.detector_range = self.sim_obstacles_detector['detector_range']
    self.cov_meas_pos = self.sim_obstacles_detector['cov_meas_pos']
    self.cov_meas_siz = self.sim_obstacles_detector['cov_meas_siz']


    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.robotPoseCallback, qos_profile=10)
    
    # 
    obstacles_static_qos_profile = rclpy.qos.QoSProfile(depth=1)
    obstacles_static_qos_profile.history=rclpy.qos.HistoryPolicy.KEEP_LAST
    obstacles_static_qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    obstacles_static_qos_profile.reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
    self.obstacles_static_sub = self.create_subscription(MarkerArray, 'obstacles_static', self.obstaclesStaticCallback, obstacles_static_qos_profile)
    #
    self.obstacles_dynamic_sub = self.create_subscription(MarkerArray, 'obstacles_dynamic', self.obstaclesDynamicCallback, qos_profile=10)


    # Publishers

    # 
    if(self.flag_pub_obstacles_detected_world):
      self.obstacles_detected_world_pub = self.create_publisher(MarkerArray, 'obstacles_detected_world', qos_profile=10)
    # 
    self.obstacles_detected_robot_pub = self.create_publisher(MarkerArray, 'obstacles_detected_robot', qos_profile=10)



    # Timers
    #
    self.obstacle_detect_loop_timer = self.create_timer(1.0/self.obstacle_detect_loop_freq, self.obstacleDetectorLoopTimerCallback)


    # End
    return


  def run(self):

    rclpy.spin(self)

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
    return



  def obstaclesStaticCallback(self, obstacles_static_msg):

    self.obstacles_static_msg = obstacles_static_msg

    #
    return



  def obstaclesDynamicCallback(self, obstacles_dynamic_msg):

    self.obstacles_dynamic_msg = obstacles_dynamic_msg

    #
    return


  def getObstacleDetectI(self, obst_i_msg, robot_posi, robot_atti_rot_mat, time_stamp_current):

    # Init outputs
    obst_i_robot_msg = None
    obst_i_world_msg = None
    
    #
    obst_i_posi_world = np.zeros((3,), dtype=float)
    obst_i_posi_world[0] = obst_i_msg.pose.position.x
    obst_i_posi_world[1] = obst_i_msg.pose.position.y
    obst_i_posi_world[2] = obst_i_msg.pose.position.z

    obst_i_rad = obst_i_msg.scale.x/2.0

    distance = ars_lib_helpers.distancePointCircle(robot_posi[0:2], obst_i_posi_world[0:2], obst_i_rad)

    # distance
    if(distance <= self.detector_range):

      # Noises
      #
      posi_noise = np.zeros((3,), dtype=float)
      posi_noise[0] = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_pos['x']))
      posi_noise[1] = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_pos['y']))
      posi_noise[2] = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_pos['z']))
      #
      radius_noise = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_siz['R']))
      height_noise = np.random.normal(loc = 0.0, scale = np.math.sqrt(self.cov_meas_siz['h']))
  

      ############
      # obstacle wrt World
      if(self.flag_pub_obstacles_detected_world):
        obst_i_world_msg = []
        obst_i_world_msg = copy.deepcopy(obst_i_msg)

        # Header
        obst_i_world_msg.header.stamp=time_stamp_current.to_msg()
        # obst_i_world_msg.header.frame_id = 

        # Change color
        obst_i_world_msg.color.r = 0.0
        obst_i_world_msg.color.g = 0.0
        obst_i_world_msg.color.b = 1.0
        obst_i_world_msg.color.a = 0.6

        # Lifetime
        duration_in_sec = 2.0*1.0/self.obstacle_detect_loop_freq
        obst_i_world_msg.lifetime = Duration(sec=int(duration_in_sec), nanosec=int((duration_in_sec-int(duration_in_sec))*1e9))

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

      # Header
      obst_i_robot_msg.header.stamp=time_stamp_current.to_msg()
      # obst_i_robot_msg.header.frame_id = 

      # Change color
      obst_i_robot_msg.color.r = 0.0
      obst_i_robot_msg.color.g = 0.0
      obst_i_robot_msg.color.b = 1.0
      obst_i_robot_msg.color.a = 0.6

      # Lifetime
      duration_in_sec = 2.0*1.0/self.obstacle_detect_loop_freq
      obst_i_robot_msg.lifetime = Duration(sec=int(duration_in_sec), nanosec=int((duration_in_sec-int(duration_in_sec))*1e9))

      # Change to robot coordinates
      # t_O_R = (R_R_W)^T * (t_O_W - t_R_W)
      # R_O_R = (R_R_W)^T * R_O_W
      #
      obst_i_posi_robot = np.matmul(robot_atti_rot_mat.T, (obst_i_posi_world-robot_posi))

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

    # End
    return obst_i_robot_msg, obst_i_world_msg



  def detectObstacles(self, time_stamp_current):

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

          if(obst_i_msg.type == 3):

            obst_i_robot_msg, obst_i_world_msg = self.getObstacleDetectI(obst_i_msg, self.robot_posi, robot_atti_rot_mat, time_stamp_current)

            if(obst_i_robot_msg is not None):
              # Append robot
              self.obstacles_detected_robot_msg.markers.append(obst_i_robot_msg)

            if(obst_i_world_msg is not None):
              # Append world
              self.obstacles_detected_world_msg.markers.append(obst_i_world_msg)

          else:
            self.get_logger().info("Unknown obstacle type:"+obst_i_msg.type)


      # Obstacles dynamic
      for obst_i_msg in self.obstacles_dynamic_msg.markers:

        if(obst_i_msg.action == 0):

          if(obst_i_msg.type == 3):

            obst_i_robot_msg, obst_i_world_msg = self.getObstacleDetectI(obst_i_msg, self.robot_posi, robot_atti_rot_mat, time_stamp_current)

            if(obst_i_robot_msg is not None):
              # Append robot
              self.obstacles_detected_robot_msg.markers.append(obst_i_robot_msg)

            if(obst_i_world_msg is not None):
              # Append world
              self.obstacles_detected_world_msg.markers.append(obst_i_world_msg)

          else:
            self.get_logger().info("Unknown obstacle type!!")


    # Publish
    if(self.flag_pub_obstacles_detected_world):
      self.obstacles_detected_world_pub.publish(self.obstacles_detected_world_msg)
    self.obstacles_detected_robot_pub.publish(self.obstacles_detected_robot_msg)

    #
    return


  def obstacleDetectorLoopTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

    #
    self.detectObstacles(time_stamp_current)

    #
    return

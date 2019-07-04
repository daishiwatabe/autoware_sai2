# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/autoware/Autoware_1.11.0_edit/ros/install/ymc;/home/autoware/Autoware_1.11.0_edit/ros/install/xsens_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/lattice_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/waypoint_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/waypoint_maker;/home/autoware/Autoware_1.11.0_edit/ros/install/way_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/trafficlight_recognizer;/home/autoware/Autoware_1.11.0_edit/ros/install/op_utilities;/home/autoware/Autoware_1.11.0_edit/ros/install/op_simulation_package;/home/autoware/Autoware_1.11.0_edit/ros/install/op_local_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/op_global_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_kf_contour_track;/home/autoware/Autoware_1.11.0_edit/ros/install/op_ros_helpers;/home/autoware/Autoware_1.11.0_edit/ros/install/lane_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/ff_waypoint_follower;/home/autoware/Autoware_1.11.0_edit/ros/install/dp_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/waypoint_follower;/home/autoware/Autoware_1.11.0_edit/ros/install/vlg22c_cam;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_ssd_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_segment_enet_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_lane_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_dpm_ttic_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_darknet_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_beyond_track;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_model;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_gazebo_simulation_launcher;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_gazebo_simulation_interface;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_description;/home/autoware/Autoware_1.11.0_edit/ros/install/op_simu;/home/autoware/Autoware_1.11.0_edit/ros/install/op_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/op_utility;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_euclidean_cluster_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vector_map_server;/home/autoware/Autoware_1.11.0_edit/ros/install/road_occupancy_processor;/home/autoware/Autoware_1.11.0_edit/ros/install/costmap_generator;/home/autoware/Autoware_1.11.0_edit/ros/install/object_map;/home/autoware/Autoware_1.11.0_edit/ros/install/naive_motion_predict;/home/autoware/Autoware_1.11.0_edit/ros/install/map_file;/home/autoware/Autoware_1.11.0_edit/ros/install/libvectormap;/home/autoware/Autoware_1.11.0_edit/ros/install/imm_ukf_pda_track;/home/autoware/Autoware_1.11.0_edit/ros/install/decision_maker;/home/autoware/Autoware_1.11.0_edit/ros/install/vector_map;/home/autoware/Autoware_1.11.0_edit/ros/install/vector_map_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/vectacam;/home/autoware/Autoware_1.11.0_edit/ros/install/udon_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/tablet_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/runtime_manager;/home/autoware/Autoware_1.11.0_edit/ros/install/mqtt_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/tablet_socket_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/state_machine_lib;/home/autoware/Autoware_1.11.0_edit/ros/install/sound_player;/home/autoware/Autoware_1.11.0_edit/ros/install/signal_creator;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_lms5xx;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_description;/home/autoware/Autoware_1.11.0_edit/ros/install/rslidar_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/rslidar_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/points2image;/home/autoware/Autoware_1.11.0_edit/ros/install/rosinterface;/home/autoware/Autoware_1.11.0_edit/ros/install/rosbag_controller;/home/autoware/Autoware_1.11.0_edit/ros/install/roi_object_filter;/home/autoware/Autoware_1.11.0_edit/ros/install/range_vision_fusion;/home/autoware/Autoware_1.11.0_edit/ros/install/pos_db;/home/autoware/Autoware_1.11.0_edit/ros/install/points_preprocessor;/home/autoware/Autoware_1.11.0_edit/ros/install/points_downsampler;/home/autoware/Autoware_1.11.0_edit/ros/install/pixel_cloud_fusion;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_localizer;/home/autoware/Autoware_1.11.0_edit/ros/install/pcl_omp_registration;/home/autoware/Autoware_1.11.0_edit/ros/install/pc2_downsampler;/home/autoware/Autoware_1.11.0_edit/ros/install/ouster_ros;/home/autoware/Autoware_1.11.0_edit/ros/install/oculus_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/obj_db;/home/autoware/Autoware_1.11.0_edit/ros/install/novatel_oem7;/home/autoware/Autoware_1.11.0_edit/ros/install/nmea_navsat;/home/autoware/Autoware_1.11.0_edit/ros/install/ndt_tku;/home/autoware/Autoware_1.11.0_edit/ros/install/ndt_gpu;/home/autoware/Autoware_1.11.0_edit/ros/install/ndt_cpu;/home/autoware/Autoware_1.11.0_edit/ros/install/multi_lidar_calibrator;/home/autoware/Autoware_1.11.0_edit/ros/install/microstrain_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/memsic_imu;/home/autoware/Autoware_1.11.0_edit/ros/install/marker_downsampler;/home/autoware/Autoware_1.11.0_edit/ros/install/map_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/map_tf_generator;/home/autoware/Autoware_1.11.0_edit/ros/install/log_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_shape_estimation;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_point_pillars;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_naive_l_shape_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_fake_perception;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_apollo_cnn_seg_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/libdpm_ttic;/home/autoware/Autoware_1.11.0_edit/ros/install/lgsvl_simulator_bridge;/home/autoware/Autoware_1.11.0_edit/ros/install/lgsvl_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/kvaser;/home/autoware/Autoware_1.11.0_edit/ros/install/kitti_launch;/home/autoware/Autoware_1.11.0_edit/ros/install/kitti_player;/home/autoware/Autoware_1.11.0_edit/ros/install/kitti_box_publisher;/home/autoware/Autoware_1.11.0_edit/ros/install/javad_navsat_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/integrated_viewer;/home/autoware/Autoware_1.11.0_edit/ros/install/image_processor;/home/autoware/Autoware_1.11.0_edit/ros/install/hokuyo;/home/autoware/Autoware_1.11.0_edit/ros/install/graph_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/gnss_localizer;/home/autoware/Autoware_1.11.0_edit/ros/install/gnss;/home/autoware/Autoware_1.11.0_edit/ros/install/glviewer;/home/autoware/Autoware_1.11.0_edit/ros/install/gazebo_world_description;/home/autoware/Autoware_1.11.0_edit/ros/install/gazebo_imu_description;/home/autoware/Autoware_1.11.0_edit/ros/install/gazebo_camera_description;/home/autoware/Autoware_1.11.0_edit/ros/install/garmin;/home/autoware/Autoware_1.11.0_edit/ros/install/freespace_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/fastvirtualscan;/home/autoware/Autoware_1.11.0_edit/ros/install/detected_objects_visualizer;/home/autoware/Autoware_1.11.0_edit/ros/install/decision_maker_panel;/home/autoware/Autoware_1.11.0_edit/ros/install/dbw_mkz_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/data_preprocessor;/home/autoware/Autoware_1.11.0_edit/ros/install/custom_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/calibration_publisher;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_health_checker;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_system_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_rviz_plugins;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_pointgrey_drivers;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_driveworks_interface;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_connector;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_camera_lidar_calibrator;/home/autoware/Autoware_1.11.0_edit/ros/install/astar_search;/home/autoware/Autoware_1.11.0_edit/ros/install/as;/home/autoware/Autoware_1.11.0_edit/ros/install/amathutils_lib;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_launcher_rviz;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_launcher;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_driveworks_gmsl_interface;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_config_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_can_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_build_flags;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_bag_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/adi_driver;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/autoware/Autoware_1.11.0_edit/build/waypoint_planner/devel/env.sh')

output_filename = '/home/autoware/Autoware_1.11.0_edit/build/waypoint_planner/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)

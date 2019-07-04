#!/usr/bin/python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

'''This file generates shell code for the setup.SHELL scripts to set environment variables'''

from __future__ import print_function
import argparse
import copy
import errno
import os
import platform
import sys

CATKIN_MARKER_FILE = '.catkin'

system = platform.system()
IS_DARWIN = (system == 'Darwin')
IS_WINDOWS = (system == 'Windows')

# subfolder of workspace prepended to CMAKE_PREFIX_PATH
ENV_VAR_SUBFOLDERS = {
    'CMAKE_PREFIX_PATH': '',
    'LD_LIBRARY_PATH' if not IS_DARWIN else 'DYLD_LIBRARY_PATH': ['lib', os.path.join('lib', 'x86_64-linux-gnu')],
    'PATH': 'bin',
    'PKG_CONFIG_PATH': [os.path.join('lib', 'pkgconfig'), os.path.join('lib', 'x86_64-linux-gnu', 'pkgconfig')],
    'PYTHONPATH': 'lib/python2.7/dist-packages',
}


def rollback_env_variables(environ, env_var_subfolders):
    '''
    Generate shell code to reset environment variables
    by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH.
    This does not cover modifications performed by environment hooks.
    '''
    lines = []
    unmodified_environ = copy.copy(environ)
    for key in sorted(env_var_subfolders.keys()):
        subfolders = env_var_subfolders[key]
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        value = _rollback_env_variable(unmodified_environ, key, subfolders)
        if value is not None:
            environ[key] = value
            lines.append(assignment(key, value))
    if lines:
        lines.insert(0, comment('reset environment variables by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH'))
    return lines


def _rollback_env_variable(environ, name, subfolders):
    '''
    For each catkin workspace in CMAKE_PREFIX_PATH remove the first entry from env[NAME] matching workspace + subfolder.

    :param subfolders: list of str '' or subfoldername that may start with '/'
    :returns: the updated value of the environment variable.
    '''
    value = environ[name] if name in environ else ''
    env_paths = [path for path in value.split(os.pathsep) if path]
    value_modified = False
    for subfolder in subfolders:
        if subfolder:
            if subfolder.startswith(os.path.sep) or (os.path.altsep and subfolder.startswith(os.path.altsep)):
                subfolder = subfolder[1:]
            if subfolder.endswith(os.path.sep) or (os.path.altsep and subfolder.endswith(os.path.altsep)):
                subfolder = subfolder[:-1]
        for ws_path in _get_workspaces(environ, include_fuerte=True, include_non_existing=True):
            path_to_find = os.path.join(ws_path, subfolder) if subfolder else ws_path
            path_to_remove = None
            for env_path in env_paths:
                env_path_clean = env_path[:-1] if env_path and env_path[-1] in [os.path.sep, os.path.altsep] else env_path
                if env_path_clean == path_to_find:
                    path_to_remove = env_path
                    break
            if path_to_remove:
                env_paths.remove(path_to_remove)
                value_modified = True
    new_value = os.pathsep.join(env_paths)
    return new_value if value_modified else None


def _get_workspaces(environ, include_fuerte=False, include_non_existing=False):
    '''
    Based on CMAKE_PREFIX_PATH return all catkin workspaces.

    :param include_fuerte: The flag if paths starting with '/opt/ros/fuerte' should be considered workspaces, ``bool``
    '''
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    value = environ[env_name] if env_name in environ else ''
    paths = [path for path in value.split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE)) or (include_fuerte and path.startswith('/opt/ros/fuerte')) or (include_non_existing and not os.path.exists(path))]
    return workspaces


def prepend_env_variables(environ, env_var_subfolders, workspaces):
    '''
    Generate shell code to prepend environment variables
    for the all workspaces.
    '''
    lines = []
    lines.append(comment('prepend folders of workspaces to environment variables'))

    paths = [path for path in workspaces.split(os.pathsep) if path]

    prefix = _prefix_env_variable(environ, 'CMAKE_PREFIX_PATH', paths, '')
    lines.append(prepend(environ, 'CMAKE_PREFIX_PATH', prefix))

    for key in sorted([key for key in env_var_subfolders.keys() if key != 'CMAKE_PREFIX_PATH']):
        subfolder = env_var_subfolders[key]
        prefix = _prefix_env_variable(environ, key, paths, subfolder)
        lines.append(prepend(environ, key, prefix))
    return lines


def _prefix_env_variable(environ, name, paths, subfolders):
    '''
    Return the prefix to prepend to the environment variable NAME, adding any path in NEW_PATHS_STR without creating duplicate or empty items.
    '''
    value = environ[name] if name in environ else ''
    environ_paths = [path for path in value.split(os.pathsep) if path]
    checked_paths = []
    for path in paths:
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        for subfolder in subfolders:
            path_tmp = path
            if subfolder:
                path_tmp = os.path.join(path_tmp, subfolder)
            # skip nonexistent paths
            if not os.path.exists(path_tmp):
                continue
            # exclude any path already in env and any path we already added
            if path_tmp not in environ_paths and path_tmp not in checked_paths:
                checked_paths.append(path_tmp)
    prefix_str = os.pathsep.join(checked_paths)
    if prefix_str != '' and environ_paths:
        prefix_str += os.pathsep
    return prefix_str


def assignment(key, value):
    if not IS_WINDOWS:
        return 'export %s="%s"' % (key, value)
    else:
        return 'set %s=%s' % (key, value)


def comment(msg):
    if not IS_WINDOWS:
        return '# %s' % msg
    else:
        return 'REM %s' % msg


def prepend(environ, key, prefix):
    if key not in environ or not environ[key]:
        return assignment(key, prefix)
    if not IS_WINDOWS:
        return 'export %s="%s$%s"' % (key, prefix, key)
    else:
        return 'set %s=%s%%%s%%' % (key, prefix, key)


def find_env_hooks(environ, cmake_prefix_path):
    '''
    Generate shell code with found environment hooks
    for the all workspaces.
    '''
    lines = []
    lines.append(comment('found environment hooks in workspaces'))

    generic_env_hooks = []
    generic_env_hooks_workspace = []
    specific_env_hooks = []
    specific_env_hooks_workspace = []
    generic_env_hooks_by_filename = {}
    specific_env_hooks_by_filename = {}
    generic_env_hook_ext = 'bat' if IS_WINDOWS else 'sh'
    specific_env_hook_ext = environ['CATKIN_SHELL'] if not IS_WINDOWS and 'CATKIN_SHELL' in environ and environ['CATKIN_SHELL'] else None
    # remove non-workspace paths
    workspaces = [path for path in cmake_prefix_path.split(os.pathsep) if path and os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE))]
    for workspace in reversed(workspaces):
        env_hook_dir = os.path.join(workspace, 'etc', 'catkin', 'profile.d')
        if os.path.isdir(env_hook_dir):
            for filename in sorted(os.listdir(env_hook_dir)):
                if filename.endswith('.%s' % generic_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in generic_env_hooks_by_filename:
                        i = generic_env_hooks.index(generic_env_hooks_by_filename[filename])
                        generic_env_hooks.pop(i)
                        generic_env_hooks_workspace.pop(i)
                    # append env hook
                    generic_env_hooks.append(os.path.join(env_hook_dir, filename))
                    generic_env_hooks_workspace.append(workspace)
                    generic_env_hooks_by_filename[filename] = generic_env_hooks[-1]
                elif specific_env_hook_ext is not None and filename.endswith('.%s' % specific_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in specific_env_hooks_by_filename:
                        i = specific_env_hooks.index(specific_env_hooks_by_filename[filename])
                        specific_env_hooks.pop(i)
                        specific_env_hooks_workspace.pop(i)
                    # append env hook
                    specific_env_hooks.append(os.path.join(env_hook_dir, filename))
                    specific_env_hooks_workspace.append(workspace)
                    specific_env_hooks_by_filename[filename] = specific_env_hooks[-1]
    env_hooks = generic_env_hooks + specific_env_hooks
    env_hooks_workspace = generic_env_hooks_workspace + specific_env_hooks_workspace
    count = len(env_hooks)
    lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_COUNT', count))
    for i in range(count):
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d' % i, env_hooks[i]))
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d_WORKSPACE' % i, env_hooks_workspace[i]))
    return lines


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    parser.add_argument('--extend', action='store_true', help='Skip unsetting previous environment variables to extend context')
    return parser.parse_known_args(args=args)[0]


if __name__ == '__main__':
    try:
        try:
            args = _parse_arguments()
        except Exception as e:
            print(e, file=sys.stderr)
            sys.exit(1)

        # environment at generation time
        CMAKE_PREFIX_PATH = '/home/autoware/Autoware_1.11.0_edit/ros/install/ymc;/home/autoware/Autoware_1.11.0_edit/ros/install/xsens_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/lattice_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/waypoint_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/waypoint_maker;/home/autoware/Autoware_1.11.0_edit/ros/install/way_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/trafficlight_recognizer;/home/autoware/Autoware_1.11.0_edit/ros/install/op_utilities;/home/autoware/Autoware_1.11.0_edit/ros/install/op_simulation_package;/home/autoware/Autoware_1.11.0_edit/ros/install/op_local_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/op_global_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_kf_contour_track;/home/autoware/Autoware_1.11.0_edit/ros/install/op_ros_helpers;/home/autoware/Autoware_1.11.0_edit/ros/install/lane_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/ff_waypoint_follower;/home/autoware/Autoware_1.11.0_edit/ros/install/dp_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/waypoint_follower;/home/autoware/Autoware_1.11.0_edit/ros/install/vlg22c_cam;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_ssd_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_segment_enet_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_lane_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_dpm_ttic_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_darknet_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vision_beyond_track;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_model;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_gazebo_simulation_launcher;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_gazebo_simulation_interface;/home/autoware/Autoware_1.11.0_edit/ros/install/vehicle_description;/home/autoware/Autoware_1.11.0_edit/ros/install/op_simu;/home/autoware/Autoware_1.11.0_edit/ros/install/op_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/op_utility;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_euclidean_cluster_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/vector_map_server;/home/autoware/Autoware_1.11.0_edit/ros/install/road_occupancy_processor;/home/autoware/Autoware_1.11.0_edit/ros/install/costmap_generator;/home/autoware/Autoware_1.11.0_edit/ros/install/object_map;/home/autoware/Autoware_1.11.0_edit/ros/install/naive_motion_predict;/home/autoware/Autoware_1.11.0_edit/ros/install/map_file;/home/autoware/Autoware_1.11.0_edit/ros/install/libvectormap;/home/autoware/Autoware_1.11.0_edit/ros/install/imm_ukf_pda_track;/home/autoware/Autoware_1.11.0_edit/ros/install/decision_maker;/home/autoware/Autoware_1.11.0_edit/ros/install/vector_map;/home/autoware/Autoware_1.11.0_edit/ros/install/vector_map_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/vectacam;/home/autoware/Autoware_1.11.0_edit/ros/install/udon_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/tablet_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/runtime_manager;/home/autoware/Autoware_1.11.0_edit/ros/install/mqtt_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/tablet_socket_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/state_machine_lib;/home/autoware/Autoware_1.11.0_edit/ros/install/sound_player;/home/autoware/Autoware_1.11.0_edit/ros/install/signal_creator;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_lms5xx;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/sick_ldmrs_description;/home/autoware/Autoware_1.11.0_edit/ros/install/rslidar_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/rslidar_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/points2image;/home/autoware/Autoware_1.11.0_edit/ros/install/rosinterface;/home/autoware/Autoware_1.11.0_edit/ros/install/rosbag_controller;/home/autoware/Autoware_1.11.0_edit/ros/install/roi_object_filter;/home/autoware/Autoware_1.11.0_edit/ros/install/range_vision_fusion;/home/autoware/Autoware_1.11.0_edit/ros/install/pos_db;/home/autoware/Autoware_1.11.0_edit/ros/install/points_preprocessor;/home/autoware/Autoware_1.11.0_edit/ros/install/points_downsampler;/home/autoware/Autoware_1.11.0_edit/ros/install/pixel_cloud_fusion;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_localizer;/home/autoware/Autoware_1.11.0_edit/ros/install/pcl_omp_registration;/home/autoware/Autoware_1.11.0_edit/ros/install/pc2_downsampler;/home/autoware/Autoware_1.11.0_edit/ros/install/ouster_ros;/home/autoware/Autoware_1.11.0_edit/ros/install/oculus_socket;/home/autoware/Autoware_1.11.0_edit/ros/install/obj_db;/home/autoware/Autoware_1.11.0_edit/ros/install/novatel_oem7;/home/autoware/Autoware_1.11.0_edit/ros/install/nmea_navsat;/home/autoware/Autoware_1.11.0_edit/ros/install/ndt_tku;/home/autoware/Autoware_1.11.0_edit/ros/install/ndt_gpu;/home/autoware/Autoware_1.11.0_edit/ros/install/ndt_cpu;/home/autoware/Autoware_1.11.0_edit/ros/install/multi_lidar_calibrator;/home/autoware/Autoware_1.11.0_edit/ros/install/microstrain_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/memsic_imu;/home/autoware/Autoware_1.11.0_edit/ros/install/marker_downsampler;/home/autoware/Autoware_1.11.0_edit/ros/install/map_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/map_tf_generator;/home/autoware/Autoware_1.11.0_edit/ros/install/log_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_shape_estimation;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_point_pillars;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_naive_l_shape_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_fake_perception;/home/autoware/Autoware_1.11.0_edit/ros/install/lidar_apollo_cnn_seg_detect;/home/autoware/Autoware_1.11.0_edit/ros/install/libdpm_ttic;/home/autoware/Autoware_1.11.0_edit/ros/install/lgsvl_simulator_bridge;/home/autoware/Autoware_1.11.0_edit/ros/install/lgsvl_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/kvaser;/home/autoware/Autoware_1.11.0_edit/ros/install/kitti_launch;/home/autoware/Autoware_1.11.0_edit/ros/install/kitti_player;/home/autoware/Autoware_1.11.0_edit/ros/install/kitti_box_publisher;/home/autoware/Autoware_1.11.0_edit/ros/install/javad_navsat_driver;/home/autoware/Autoware_1.11.0_edit/ros/install/integrated_viewer;/home/autoware/Autoware_1.11.0_edit/ros/install/image_processor;/home/autoware/Autoware_1.11.0_edit/ros/install/hokuyo;/home/autoware/Autoware_1.11.0_edit/ros/install/graph_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/gnss_localizer;/home/autoware/Autoware_1.11.0_edit/ros/install/gnss;/home/autoware/Autoware_1.11.0_edit/ros/install/glviewer;/home/autoware/Autoware_1.11.0_edit/ros/install/gazebo_world_description;/home/autoware/Autoware_1.11.0_edit/ros/install/gazebo_imu_description;/home/autoware/Autoware_1.11.0_edit/ros/install/gazebo_camera_description;/home/autoware/Autoware_1.11.0_edit/ros/install/garmin;/home/autoware/Autoware_1.11.0_edit/ros/install/freespace_planner;/home/autoware/Autoware_1.11.0_edit/ros/install/fastvirtualscan;/home/autoware/Autoware_1.11.0_edit/ros/install/detected_objects_visualizer;/home/autoware/Autoware_1.11.0_edit/ros/install/decision_maker_panel;/home/autoware/Autoware_1.11.0_edit/ros/install/dbw_mkz_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/data_preprocessor;/home/autoware/Autoware_1.11.0_edit/ros/install/custom_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/calibration_publisher;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_health_checker;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_system_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_rviz_plugins;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_pointgrey_drivers;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_driveworks_interface;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_connector;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_camera_lidar_calibrator;/home/autoware/Autoware_1.11.0_edit/ros/install/astar_search;/home/autoware/Autoware_1.11.0_edit/ros/install/as;/home/autoware/Autoware_1.11.0_edit/ros/install/amathutils_lib;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_launcher_rviz;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_launcher;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_driveworks_gmsl_interface;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_config_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_can_msgs;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_build_flags;/home/autoware/Autoware_1.11.0_edit/ros/install/autoware_bag_tools;/home/autoware/Autoware_1.11.0_edit/ros/install/adi_driver;/opt/ros/kinetic'.split(';')
        # prepend current workspace if not already part of CPP
        base_path = os.path.dirname(__file__)
        if base_path not in CMAKE_PREFIX_PATH:
            CMAKE_PREFIX_PATH.insert(0, base_path)
        CMAKE_PREFIX_PATH = os.pathsep.join(CMAKE_PREFIX_PATH)

        environ = dict(os.environ)
        lines = []
        if not args.extend:
            lines += rollback_env_variables(environ, ENV_VAR_SUBFOLDERS)
        lines += prepend_env_variables(environ, ENV_VAR_SUBFOLDERS, CMAKE_PREFIX_PATH)
        lines += find_env_hooks(environ, CMAKE_PREFIX_PATH)
        print('\n'.join(lines))

        # need to explicitly flush the output
        sys.stdout.flush()
    except IOError as e:
        # and catch potential "broken pipe" if stdout is not writable
        # which can happen when piping the output to a file but the disk is full
        if e.errno == errno.EPIPE:
            print(e, file=sys.stderr)
            sys.exit(2)
        raise

    sys.exit(0)

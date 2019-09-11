/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include "autoware_msgs/VehicleCmd.h"
#include <autoware_can_msgs/CANInfo.h>
#include <autoware_msgs/PositionChecker.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_config_msgs/ConfigManualDriveStroke.h>

#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

struct CommandData {
  double linear_x;
  double angular_z;
  int modeValue;
  int gearValue;
  int lampValue;
  int accellValue;
  int brakeValue;
  int steerValue;
  double linear_velocity;
  double steering_angle;

  void reset();
};

void CommandData::reset()
{
  linear_x      = 0;
  angular_z     = 0;
  modeValue     = 0;
  gearValue     = 0;
  lampValue     = 0;
  accellValue   = 0;
  brakeValue    = 0;
  steerValue    = 0;
  linear_velocity = -1;
  steering_angle = 0;
}

static bool run_manual_drive_stroke = false;
static double manual_drive_stroke=0;
static double manual_brake_stroke=0;
static void manualDriveStrokeCallback(const autoware_config_msgs::ConfigManualDriveStroke& msg)
{
	run_manual_drive_stroke = msg.run;
	manual_drive_stroke = msg.drive_stroke;
	manual_brake_stroke = msg.brake_stroke;
}

static CommandData command_data;
static int stopFlag = 1;

static int steer_gain_ki = 40;
static void canCallback(const autoware_can_msgs::CANInfo& msg)
{
    double speed = msg.speed;
    if(speed < 2.0)
    {
        steer_gain_ki = 0;
    }
    else
    {
        steer_gain_ki = 4*(speed-20)+80;
        if(steer_gain_ki > 120) steer_gain_ki = 120;
        if(steer_gain_ki < 20) steer_gain_ki = 20;
    }
}

static void positionCheckerCallback(const autoware_msgs::PositionChecker& msg)
{
    if(msg.stop_flag == true) {stopFlag = 1; std::cout << "aaa\n";}
    else stopFlag = 0;
}

static int blinker=-1;
static int velocity_KPPlus=-1;//8000;
static int velocity_KPMinus=-1;//3500;
static int velocity_punchPlus=-1;
static int velocity_punchMinus=-1;
static int velocity_windowPlus=-1;
static int velocity_windowMinus=-1;
static double drive_stroke=-1;
static double brake_stroke=-1;
static int pause_val=-1;
static void wayparamCallback(const autoware_msgs::WaypointParam& msg)
{
    blinker = msg.blinker;
    velocity_KPPlus = msg.velocity_KPPlus;
    velocity_KPMinus = msg.velocity_KPMinus;
    velocity_punchPlus = msg.velocity_punchPlus;
    velocity_punchMinus = msg.velocity_punchMinus;
    velocity_windowPlus = msg.velocity_windowPlus;
    velocity_windowMinus = msg.velocity_windowMinus;
	drive_stroke = msg.drive_stroke;
	brake_stroke = msg.brake_stroke;
    pause_val = msg.pause;
}

static void vehicleCmdCallback(const autoware_msgs::VehicleCmd& msg)
{
  command_data.linear_x = msg.twist_cmd.twist.linear.x;
  command_data.angular_z = msg.twist_cmd.twist.angular.z;
  command_data.modeValue = msg.mode;
  command_data.gearValue = msg.gear;
  if(msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 0) {
    command_data.lampValue = 0;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 0) {
    command_data.lampValue = 1;
  }
  else if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 1) {
    command_data.lampValue = 2;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 1) {
    command_data.lampValue = 3;
  }
  command_data.accellValue = msg.accel_cmd.accel;
  command_data.steerValue = msg.steer_cmd.steer;
  command_data.brakeValue = msg.brake_cmd.brake;
  command_data.linear_velocity = msg.ctrl_cmd.linear_velocity;
  command_data.steering_angle = msg.ctrl_cmd.steering_angle;
}

static void *sendCommand(void *arg)
{
  int *client_sockp = static_cast<int*>(arg);
  int client_sock = *client_sockp;
  delete client_sockp;

  std::ostringstream oss;
  oss << command_data.linear_x << ",";
  oss << command_data.angular_z << ",";
  oss << command_data.modeValue << ",";
  oss << command_data.gearValue << ",";
  oss << command_data.accellValue << ",";
  oss << command_data.steerValue << ",";
  oss << command_data.brakeValue << ",";
  oss << command_data.linear_velocity << ",";
  oss << command_data.steering_angle << ",";
  //oss << command_data.lampValue;
  oss << stopFlag << ",";
  oss << blinker << ",";
  oss << pause_val << ",";
  oss << steer_gain_ki << ",";
  oss << velocity_KPPlus << ",";
  oss << velocity_KPMinus << ",";
  oss << velocity_punchPlus << ",";
  oss << velocity_punchMinus << ",";
  oss << velocity_windowPlus << ",";
  oss << velocity_windowMinus << ",";
  oss << manual_drive_stroke << ",";
  oss << manual_brake_stroke << ",";

  std::string cmd(oss.str());
  ssize_t n = write(client_sock, cmd.c_str(), cmd.size());
  if(n < 0){
    std::perror("write");
    return nullptr;
  }

  if(close(client_sock) == -1){
    std::perror("close");
    return nullptr;
  }

  std::cout << "cmd: " << cmd << ", size: " << cmd.size() << std::endl;
  return nullptr;
}

static void* receiverCaller(void *unused)
{
  constexpr int listen_port = 10001;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if(sock == -1){
    std::perror("socket");
    return nullptr;
  }

  sockaddr_in addr;
  sockaddr_in client;
  socklen_t len = sizeof(client);

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = PF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = INADDR_ANY;

  int ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if(ret == -1){
    std::perror("bind");
    goto error;
  }

  ret = listen(sock, 20);
  if(ret == -1){
    std::perror("listen");
    goto error;
  }

  while(true){
    //get connect to android
    std::cout << "Waiting access..." << std::endl;

    int *client_sock = new int();
    *client_sock = accept(sock, reinterpret_cast<sockaddr*>(&client), &len);
    if(*client_sock == -1){
      std::perror("accept");
      break;
    }

    std::cout << "get connect." << std::endl;

    pthread_t th;
    if(pthread_create(&th, nullptr, sendCommand, static_cast<void*>(client_sock)) != 0){
      std::perror("pthread_create");
      break;
    }

    if(pthread_detach(th) != 0){
      std::perror("pthread_detach");
      break;
    }
  }

error:
  close(sock);
  return nullptr;
}

int main(int argc, char **argv)
{
  ros::init(argc ,argv, "vehicle_sender") ;
  ros::NodeHandle nh;

  std::cout << "vehicle sender" << std::endl;
  ros::Subscriber sub = nh.subscribe("/vehicle_cmd", 1, vehicleCmdCallback);
  ros::Subscriber sub_position_checker = nh.subscribe("/position_checker", 1, positionCheckerCallback);
  ros::Subscriber waypoint_param_sub = nh.subscribe("/waypoint_param", 1, wayparamCallback);
  ros::Subscriber can_sub = nh.subscribe("/can_info", 1, canCallback);
  ros::Subscriber manual_drive_stroke_sub = nh.subscribe("/manual_drive_stroke", 1, manualDriveStrokeCallback);

  command_data.reset();

  pthread_t th;
  if(pthread_create(&th, nullptr, receiverCaller, nullptr) != 0){
    std::perror("pthread_create");
    std::exit(1);
  }

  if (pthread_detach(th) != 0){
    std::perror("pthread_detach");
    std::exit(1);
  }

  ros::spin();
  return 0;
}

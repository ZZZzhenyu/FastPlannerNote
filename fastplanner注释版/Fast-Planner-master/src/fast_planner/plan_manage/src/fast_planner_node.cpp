/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

//此文件为节点定义文件

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");

  int planner;
  nh.param("planner_node/planner", planner, -1);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;//实例化实例化kinoReplanFSM类对象

  if (planner == 1) {//执行基于动力学的路径规划
    kino_replan.init(nh);//初始化kinoReplanFSM类对象
  } else if (planner == 2) {//执行基于拓扑的路径规划
    topo_replan.init(nh);
  }

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

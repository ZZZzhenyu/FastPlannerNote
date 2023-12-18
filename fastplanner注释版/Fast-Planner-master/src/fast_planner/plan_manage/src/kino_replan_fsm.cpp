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




#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;//无人机位置
  
  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules *///对规划管理类的智能指针对象planner_manager_进行了初始化
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);//设置nh参数 ；将kino_path_finder以及Bspline_optimizers进行初始化。
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  //定时器
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);//每0.01s进行一次execFSMCallback(主要函数：按不同状态来调用不同函数执行任务)
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);//用来判断目标点是否有障碍物或者是轨迹执行过程中是否有障碍物的
  //订阅器
  waypoint_sub_ =nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);//此处的waypoint是指目标点
      // have_odom 和 trigger 这两个bool变量分别在KinoReplanFSM::odometryCallback()和KinoReplanFSM::waypointCallback() 这两个消息订阅的回调函数中被改变。
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);
  //发布器，此三发布的信息被traj_server接收
  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);//发布bspline轨迹
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  //获取目标点end_pt_
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));//目标点可视化
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;//调用次数
  if (fsm_num == 100) {//fsm_num == 100：0.01*100=1S；1s打印一次当前状态
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {//判断有没有收到odometry 和 plan的 ttrigger是否被触发，都通过，改变当前状态为WAIT_TARGET。
      if (!have_odom_) {//odometryCallback
        return;
      }
      if (!trigger_) {//waypointCallback
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {//判断是否有target，如果有，则改变状态为GEN_NEW_TRAJ
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {//用callKinodynamicReplan()函数进行规划。成功则改变执行状态为EXEC_TRAJ,失败则改变执行状态为GEN_NEW_TRAJ
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);//egine函数：toRotationMatrix()：将目前的n维对象转换为n*n旋转矩阵
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();//核心！！！！！！！  规划接口
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {//判断是否重规划
    /*果当前时间距离起始时间已经超过当前执行轨迹的时长，则将have_target置于false，将执行状态变为WAIT_TARGET。
    如果当前距离与终点距离小于不需要重规划的阈值，或者当前距离与起点距离小于horizon，那么直接返回（fsm最终出口）。反之，则进入重规划阶段。*/
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {//t_cur当前时间距离起始时间，info->duration_轨迹执行时间
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {//重规划阶段
    /*则利用当前的位置，速度，状态，调用callKinodynamicReplan重规划，
    如果成功，则将执行状态更改为EXEC_TRAJ,否则将执行状态更改为GEN_NEW_TRAJ.*/
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();//当前的相对时间

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  //用来判断目标点是否有障碍物或者是轨迹执行过程中是否有障碍物的
  /*  如果目标点有障碍物，就在目标点周围通过离散的半径及角度循环来寻找新的安全的目标点。
  如果找到了，就直接改变状态进入REPLAN_TRAJ。
  如果目标点周围没有障碍物且目前状态是EXEX_TRAJ,则利用planner_manager的checkTrajCollision函数进行轨迹检查，
  如果轨迹不发生碰撞，则无事发生，如果轨迹碰撞，则状态变为REPLAN_TRAJ,进行重规划
*/
//检查目标点是否在障碍物区
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);//目标点end_pt_离周围最近障碍物距离

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;//循环步长
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {//半径循环
        for (double theta = -90; theta <= 270; theta += dtheta) {//角度循环
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {//z方向循环

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);//新目标点
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);////新目标点离周围最近障碍物距离

            if (dist > max_dist) {//原目标点周围不断找离障碍物更远的目标点，存储在goal中
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {//最优目标点goal满足要求，则将其作为目标点：end_pt_
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  //终点碰撞检查完毕后进行 轨迹碰撞检查，碰撞了则会进行重规划
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {//如果目标点周围没有障碍物且目前状态是EXEX_TRAJ,则利用planner_manager的checkTrajCollision函数进行轨迹检查，
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);//轨迹碰撞检查函数

    if (!safe) {//如果轨迹碰撞，则状态变为REPLAN_TRAJ,进行重规划
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {//调用kinodynamicReplan，进行规划，如果规划成功的话，就通过Bspline_pub把相应的B样条轨迹发布出去，并进行可视化，否则返回FALSE。
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);
      //最终得到的优化轨迹为：local_data_.position_traj_ 

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);//kinodynamicReplan确定的是无人机的位置轨迹即xyz，依据微分平坦理论，还应该确定yaw

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    //发布的轨迹：bspline
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);//bspline.pos_pts：轨迹控制点的位置信息
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));//bspline.knots：轨迹节点
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);//控制点的yaw信息
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);//通过Bspline_pub把相应的B样条轨迹发布出去

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner

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

#include <path_searching/kinodynamic_astar.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
  KinodynamicAstar::~KinodynamicAstar()
  {
    for (int i = 0; i < allocate_num_; i++)
    {
      delete path_node_pool_[i];
    }
  }

  int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                               Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
  {

//step1:初始化
    //？？？？？？？？？？？？？？？？？？？？？？？init_search是什么标识
    start_vel_ = start_v;
    start_acc_ = start_a;

    PathNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state.head(3) = start_pt;
    cur_node->state.tail(3) = start_v;
    cur_node->index = posToIndex(start_pt); // 起点位置转化为栅格地图中的index
    cur_node->g_score = 0.0;

    Eigen::VectorXd end_state(6);
    Eigen::Vector3i end_index;
    double time_to_goal;

    end_state.head(3) = end_pt;
    end_state.tail(3) = end_v;
    end_index = posToIndex(end_pt);
    cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal); // lambda_heu_ > 1 : 权重A*,牺牲最优性提高速度
    // 节点添加入open列表
    cur_node->node_state = IN_OPEN_SET; // 表征节点是在open列表还是close列表
    open_set_.push(cur_node);           // 1、open列表：open_set_
    use_node_num_ += 1;
    // 节点加入expanded_nodes_
    if (dynamic)
    {
      time_origin_ = time_start;
      cur_node->time = time_start;
      cur_node->time_idx = timeToIndex(time_start);
      expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
      // cout << "time start: " << time_start << endl;
    }
    else
      expanded_nodes_.insert(cur_node->index, cur_node);

    PathNodePtr neighbor = NULL;
    PathNodePtr terminate_node = NULL;           // 终止节点
    bool init_search = init;                     ////判断hybrid A*有没有初始化成功???????
    const int tolerance = ceil(1 / resolution_); // 误差范围：在终止节点一定范围内则认为到达// ceil () 函数用于求不小于 x 的最小整数（向上取整） floor：向下取整
//step2：路径终止判断
    while (!open_set_.empty()) // 在open_list中寻找节点，直到open_list为空
    {
      /* --- 1）从open_list优先级队列中取出f(n) = g(n) + h(n)代价值最小的节点 get lowest f_score node ---------- */
      cur_node = open_set_.top(); // 取出栈顶节点，栈顶节点就是代价值最小的节点
      /* --- 2）判断当前节点是否超出horizon或是离终点较近了，并计算一条直达曲线，检查这条曲线上是否存在。若存在，则搜索完成，返回路径点即可 ---------- */
      // Terminate?
      bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
      bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                      abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                      abs(cur_node->index(2) - end_index(2)) <= tolerance;

      if (reach_horizon || near_end)
      {
        terminate_node = cur_node;
        retrievePath(terminate_node);
        if (near_end)
        {
          // Check whether shot traj exist
          estimateHeuristic(cur_node->state, end_state, time_to_goal);
          computeShotTraj(cur_node->state, end_state, time_to_goal); // 计算一条直达终点的曲线
          if (init_search)
            ROS_ERROR("Shot in first search loop!");
        }
      }
      if (reach_horizon)
      {
        if (is_shot_succ_)
        {
          std::cout << "reach end" << std::endl;
          return REACH_END;
        }
        else
        {
          std::cout << "reach horizon" << std::endl;
          return REACH_HORIZON;
        }
      }

      if (near_end)
      {
        if (is_shot_succ_)
        {
          std::cout << "reach end" << std::endl;
          return REACH_END;
        }
        else if (cur_node->parent != NULL)
        {
          std::cout << "near end" << std::endl;
          return NEAR_END;
        }
        else
        {
          std::cout << "no path" << std::endl;
          return NO_PATH;
        }
      }
//step3：扩展节点      
      /* ---3）若当前节点没有抵达终点，就要进行节点扩张 ---------- */

      /*以上代码在当前节点的基础上，根据对输入、时间的离散进行扩展得到临时节点tmp
      首先判断节点是否已经被扩展过，即是否与当前节点在同一个节点
      检查速度约束，检查碰撞，都通过的话，就计算当前节点的g_score以及f_score.
      其中的state transit函数即通过前向积分得到扩展节点的位置和速度。接下来，就要进行节点剪枝*/

      // 1、在open_list中删除节点，并在close_list中添加节点
      open_set_.pop();//pop() 从堆栈中删除顶部元素并将堆栈大小减少一
      cur_node->node_state = IN_CLOSE_SET;
      iter_num_ += 1;

      //2、初始化状态传递
      double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;//决定节点扩展的密度
      Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
      Eigen::Matrix<double, 6, 1> pro_state;
      vector<PathNodePtr> tmp_expand_nodes;
      Eigen::Vector3d um;
      double pro_t;
      vector<Eigen::Vector3d> inputs;
      vector<double> durations;
      //3、节点扩展采样
      if (init_search)//针对初次扩展
      //最开始会以start_acc作为起始点输入进行查找，如果找不到，则再进行一轮离散化输入的寻找，若都找不到，则路径寻找失败。
      {
        inputs.push_back(start_acc_);
        for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
             tau += time_res_init * init_max_tau_)
          durations.push_back(tau);
        init_search = false;
      }
      else  
      {
        for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
          for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
            for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
            {
              um << ax, ay, az;
              inputs.push_back(um);//扩展的不同方向和大小的加速度
            }
        for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
          durations.push_back(tau);//扩展时间
      }

      //4、状态传递循环迭代，检查速度约束，检查碰撞，都通过的话，就计算当前节点的g_score以及f_score.并且对重复的扩展节点进行剪枝

      // cout << "cur state:" << cur_state.head(3).transpose() << endl;
      for (int i = 0; i < inputs.size(); ++i)
        for (int j = 0; j < durations.size(); ++j)
        {
          //进行扩展状态计算
          um = inputs[i];//um：扩展的加速度，采样
          double tau = durations[j];//tau:扩展的时间，采样
          stateTransit(cur_state, pro_state, um, tau);//状态传递，通过前向积分得到扩展节点的位置和速度//返回pro_state

          pro_t = cur_node->time + tau;//扩展节点的时间
          Eigen::Vector3d pro_pos = pro_state.head(3);//扩展节点的状态P

          // Check if in close set检查扩展节点是否在close列表（是否已经被扩展）
          Eigen::Vector3i pro_id = posToIndex(pro_pos);
          int pro_t_id = timeToIndex(pro_t);
          PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
          {
            if (init_search)
              std::cout << "close" << std::endl;
            continue;//跳过此扩展节点，进行下一节点循环
          }

          // Check maximal velocity判断节点最大速度
          Eigen::Vector3d pro_v = pro_state.tail(3);
          if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
          {
            if (init_search)
              std::cout << "vel" << std::endl;
            continue;
          }

          // Check not in the same voxel 判断节点不在同样的网格中 
          Eigen::Vector3i diff = pro_id - cur_node->index;
          int diff_time = pro_t_id - cur_node->time_idx;
          if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
          {
            if (init_search)
              std::cout << "same" << std::endl;
            continue;
          }

          // Check safety检查碰撞
          Eigen::Vector3d pos;
          Eigen::Matrix<double, 6, 1> xt;
          bool is_occ = false;
          for (int k = 1; k <= check_num_; ++k)//离散检查
          {
            double dt = tau * double(k) / double(check_num_);
            stateTransit(cur_state, xt, um, dt);
            pos = xt.head(3);
            if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1)
            {
              is_occ = true;
              break;
            }
          }
          if (is_occ)
          {
            if (init_search)
              std::cout << "safe" << std::endl;
            continue;
          }
          //计算当前扩展节点的真实代价g和fcost并记录
          double time_to_goal, tmp_g_score, tmp_f_score;
          tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
          tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

          // Compare nodes expanded from the same parent//剪枝
          bool prune = false;
          for (int j = 0; j < tmp_expand_nodes.size(); ++j)
          {
            PathNodePtr expand_node = tmp_expand_nodes[j];//tmp_expand_nodes零时扩展节点
            //首先判断当前临时扩展节点与current node的其他临时扩展节点是否在同一个voxel中，如果是的话，就要进行剪枝。
            if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))//pro_id：当前临时扩展节点在栅格地图中的index
            {
              prune = true;//进行剪枝
              //要判断当前临时扩展节点的fscore是否比同一个voxel的对比fscore小，如果是的话，则更新这一Voxel节点为当前临时扩展节点。
              if (tmp_f_score < expand_node->f_score)
              {
                expand_node->f_score = tmp_f_score;
                expand_node->g_score = tmp_g_score;
                expand_node->state = pro_state;
                expand_node->input = um;
                expand_node->duration = tau;
                if (dynamic)
                  expand_node->time = cur_node->time + tau;
              }//expand node存储扩展循环中cost最小的扩展节点
              break;
            }
          }

          // This node end up in a voxel different from others//向openset压入扩展节点
          //如果不剪枝的话，则首先判断当前临时扩展节点pro_node是否出现在open集中，
          //如果是不是的话，则可以直接将pro_node加入open集中。如果存在于open集但还未扩展的话，则比较当前临时扩展节点与对应VOXEL节点的fscore,若更小，则更新voxel中的节点。
          
          if (!prune)//不剪枝
          {
            if (pro_node == NULL)//没有扩展
            {
              
              pro_node = path_node_pool_[use_node_num_];
              pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              pro_node->node_state = IN_OPEN_SET;
              if (dynamic)
              {
                pro_node->time = cur_node->time + tau;
                pro_node->time_idx = timeToIndex(pro_node->time);
              }
              open_set_.push(pro_node);//加入openset

              if (dynamic)
                expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
              else
                expanded_nodes_.insert(pro_id, pro_node);

              tmp_expand_nodes.push_back(pro_node);

              use_node_num_ += 1;
              if (use_node_num_ == allocate_num_)
              {
                cout << "run out of memory." << endl;
                return NO_PATH;
              }
            }
            else if (pro_node->node_state == IN_OPEN_SET)// 如果不用剪枝的话，则首先判断当前临时扩展节点pro_node是否出现在open集中
            {
              if (tmp_g_score < pro_node->g_score)// 如果是存在于open集但还未扩展的话，则比较当前临时扩展节点与对应VOXEL节点的fscore,若更小，则更新voxel中的节为当前临时扩展节点。
              {
                // pro_node->index = pro_id;
                pro_node->state = pro_state;
                pro_node->f_score = tmp_f_score;
                pro_node->g_score = tmp_g_score;
                pro_node->input = um;
                pro_node->duration = tau;
                pro_node->parent = cur_node;
                if (dynamic)
                  pro_node->time = cur_node->time + tau;
              }
            }
            else
            {
              cout << "error type in searching: " << pro_node->node_state << endl;
            }
          }
        }
      // init_search = false;
    }

    cout << "open set empty, no path!" << endl;
    cout << "use node num: " << use_node_num_ << endl;
    cout << "iter num: " << iter_num_ << endl;
    return NO_PATH;
  }

  void KinodynamicAstar::setParam(ros::NodeHandle &nh)
  {
    nh.param("search/max_tau", max_tau_, -1.0);//如果考虑对时间维度进行划分才设置，这里未设置
    nh.param("search/init_max_tau", init_max_tau_, -1.0);
    nh.param("search/max_vel", max_vel_, -1.0);// 速度限制
    nh.param("search/max_acc", max_acc_, -1.0);//加速度限制
    nh.param("search/w_time", w_time_, -1.0);//g中的时间权重
    nh.param("search/horizon", horizon_, -1.0);//限制全局规划的距离，保证实时性
    nh.param("search/resolution_astar", resolution_, -1.0);//空间分辨率
    nh.param("search/time_resolution", time_resolution_, -1.0);// 时间维度分辨率
    nh.param("search/lambda_heu", lambda_heu_, -1.0);// 启发函数权重
    nh.param("search/allocate_num", allocate_num_, -1);//最大节点数目
    nh.param("search/check_num", check_num_, -1);//对中间状态安全检查
    nh.param("search/optimistic", optimistic_, true);
    tie_breaker_ = 1.0 + 1.0 / 10000;

    double vel_margin;
    nh.param("search/vel_margin", vel_margin, 0.0);//检测碰撞
    max_vel_ += vel_margin;
  }

  void KinodynamicAstar::retrievePath(PathNodePtr end_node) // 从后往前回溯路径
  {
    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
      cur_node = cur_node->parent;
      path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());//reversr：翻转数组
  }

  double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
  {

    // 算法原理：对应文章中的III.B小节

    // 主要原理:
    // 利用庞特里亚金原理解决两点边值问题，得到最优解后用最优解的控制代价作为启发函数。

    // 具体步骤：
    //  首先通过设置启发函数对时间求导等于0，得到启发函数关于时间T的四次方程，再通过求解该四次方程，
    //  得到一系列实根，通过比较这些实根所对应的cost大小，得到最优时间。

    // input ：起点x1、终点x2
    // return :optimal_time最优控制时间、启发值代价
    // head：p；tail：v
    const Vector3d dp = x2.head(3) - x1.head(3);
    const Vector3d v0 = x1.segment(3, 3); // segment（i,n）取向量第i到第i+n个元素//此处等同于tail（3）
    const Vector3d v1 = x2.segment(3, 3);
    //(1)首先通过设置启发函数对时间求导等于0，得到启发函数关于时间T的四次方程
    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
    double c5 = w_time_; // 对应rou
    //(2)求解一元四次方程，得到一系列实根，通过比较这些实根所对应的cost大小，得到最优时间。
    // 关于时间的一元四次方程是通过费拉里方法求解的，需要嵌套一个元三次方程进行求解，也就是代码中应的cubic（）函数。
    // 一元四次方程的求解过程参见wikipedia中的费拉里方法：https://zh.wikipedia.org/wiki/%E5%9B%9B%E6%AC%A1%E6%96%B9%E7%A8%8B
    // 一元三次方程的求见过程参见wikipedia中的求根系数法。需要加以说明的是，代码中的判别式大于0及等于0的情况利用了求根公式，判别式小于0的情况则是使用了三角函数解法
    // https://zh.wikipedia.org/wiki/%E4%B8%89%E6%AC%A1%E6%96%B9%E7%A8%8B

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1); // ts：代价函数求导为0 求解得到的一系列时间

    double v_max = max_vel_ * 0.5;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
    // 如果想使用其他元素级的范数，使用lpNorm<p>()方法，当求无穷范数时，模板参数p可以取特殊值Infinity，得到的是所有元素的最大绝对值。
    ts.push_back(t_bar);

    double cost = 100000000;
    double t_d = t_bar;

    for (auto t : ts)
    {
      if (t < t_bar)
        continue;
      double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
      if (c < cost)
      {
        cost = c; // cost记录ts中时间对应的最小的代价
        t_d = t;
      }
    }

    optimal_time = t_d; // 最优化时间Th，此时的cost也对应的是Th的代价

    return 1.0 * (1 + tie_breaker_) * cost; // 轻微放大代价函数打破对称性：见tie breaker
  }

  bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
  {
    // 计算一条直达终点的轨迹
    // 【原理】利用庞特里亚金原理解一个两点边值问题。
    // 因为最优控制时间time_to_goal已经在estimateHeuristic中计算过了所以这里只要引入该时间进行多项式计算即可

    /* ---------- 获取系数get coefficient ---------- */
    const Vector3d p0 = state1.head(3);
    const Vector3d dp = state2.head(3) - p0;
    const Vector3d v0 = state1.segment(3, 3);
    const Vector3d v1 = state2.segment(3, 3);
    const Vector3d dv = v1 - v0;
    double t_d = time_to_goal; // 轨迹时间，已给定
    MatrixXd coef(3, 4);       // 多项式轨迹系数矩阵
    end_vel_ = v1;

    Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
    Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
    Vector3d c = v0;
    Vector3d d = p0;

    // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
    // a*t^3 + b*t^2 + v0*t + p0
    coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d; // col：列

    Vector3d coord, vel, acc;
    VectorXd poly1d, t, polyv, polya;
    Vector3i index;

    Eigen::MatrixXd Tm(4, 4);
    Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;
    /*Tm<<
    0100
    0020
    0003
    0000
  */
    /* ----------前进轨迹检查 forward checking of trajectory ---------- */

    double t_delta = t_d / 10;
    for (double time = t_delta; time <= t_d; time += t_delta) // 离散时间点检查
    {
      t = VectorXd::Zero(4);
      for (int j = 0; j < 4; j++)
        t(j) = pow(time, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef.row(dim);               // row（）：行
        coord(dim) = poly1d.dot(t);           // p，即轨迹
        vel(dim) = (Tm * poly1d).dot(t);      // v
        acc(dim) = (Tm * Tm * poly1d).dot(t); // a

        if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_) // 轨迹是否满足va限制
        {
          // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
          // return false;
        }
      }

      if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
          coord(2) < origin_(2) || coord(2) >= map_size_3d_(2)) // 轨迹是否超出规划区域
      {
        return false;
      }

      // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
      //   return false;
      // }
      if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1) // 通过栅格占据查询，来判断轨迹是否碰到障碍物//此处是膨胀后的障碍物
      {
        return false;
      }
    }
    coef_shot_ = coef;
    t_shot_ = t_d;
    is_shot_succ_ = true; // 是否找到直达路径的标识
    return true;
  }

  vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
  {
    // 求解一元三次方程
    // ax^3 + bx^2 + cx + d = 0
    vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0)
    {
      double S = std::cbrt(R + sqrt(D));
      double T = std::cbrt(R - sqrt(D));
      dts.push_back(-a2 / 3 + (S + T));
      return dts;
    }
    else if (D == 0)
    {
      double S = std::cbrt(R);
      dts.push_back(-a2 / 3 + S + S);
      dts.push_back(-a2 / 3 - S);
      return dts;
    }
    else
    {
      double theta = acos(R / sqrt(-Q * Q * Q));
      dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
      dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
      dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
      return dts;
    }
  }

  vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
  { // 关于时间的一元四次方程是通过费拉里方法求解的，需要嵌套一个元三次方程进行求解，也就是代码中应的cubic（）函数
    // ax^4 + bx^3 + cx^2 + dx + e = 0
    vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0); // 求解一元三次方程
    double y1 = ys.front();
    double r = a3 * a3 / 4 - a2 + y1;
    if (r < 0)
      return dts;

    double R = sqrt(r);
    double D, E;
    if (R != 0)
    {
      D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
      E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    }
    else
    {
      D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
      E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D))
    {
      dts.push_back(-a3 / 4 + R / 2 + D / 2);
      dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E))
    {
      dts.push_back(-a3 / 4 - R / 2 + E / 2);
      dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
  }

  void KinodynamicAstar::init()
  {
    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    inv_time_resolution_ = 1.0 / time_resolution_;
    edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

    cout << "origin_: " << origin_.transpose() << endl;
    cout << "map size: " << map_size_3d_.transpose() << endl;

    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
      path_node_pool_[i] = new PathNode;
    }

    phi_ = Eigen::MatrixXd::Identity(6, 6);
    use_node_num_ = 0;
    iter_num_ = 0;
  }

  void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr &env)
  {
    this->edt_environment_ = env;
  }

  void KinodynamicAstar::reset()
  {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);//swap:交换值

    for (int i = 0; i < use_node_num_; i++)
    {
      PathNodePtr node = path_node_pool_[i];
      node->parent = NULL;
      node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
    has_path_ = false;
  }

  std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)//获取规划得到的路径点：state_list
  //完成路径搜索后，按照预设的时间分辨率delta_t， 通过节点回溯和状态前向积分得到分辨率更高的路径点。 
  //如果最后的shot trajectory存在的话，则还要加上最后一段shot trajectory(即通过computeshottraj)算出来得到的
  {
    vector<Vector3d> state_list;

    /* ---------- get traj of searching ---------- *///输出为：state_list
    PathNodePtr node = path_nodes_.back();//back：访问vector最后一个函数（终止节点）//不一定是终点，可能还要加上short轨迹
    Matrix<double, 6, 1> x0, xt;

    while (node->parent != NULL)//往前回溯， 直到第一个节点
    {
      Vector3d ut = node->input;//扩展的加速度
      double duration = node->duration;//扩展的时间
      x0 = node->parent->state;//父节点状态

      for (double t = duration; t >= -1e-5; t -= delta_t)//所谓分辨率更高是指扩展的加速度的方向的和大小不变，只是时间点更多，即轨迹不变，路径点更多
      {
        stateTransit(x0, xt, ut, t);//前向积分，输出扩展节点状态xt
        state_list.push_back(xt.head(3));
      }
      node = node->parent;
    }
    reverse(state_list.begin(), state_list.end());
    /* ---------- get traj of one shot ---------- */
    if (is_shot_succ_)
    {
      Vector3d coord;
      VectorXd poly1d, time(4);

      for (double t = delta_t; t <= t_shot_; t += delta_t)//使得short轨迹的分辨率更高
      {
        for (int j = 0; j < 4; j++)
          time(j) = pow(t, j);

        for (int dim = 0; dim < 3; dim++)
        {
          poly1d = coef_shot_.row(dim);
          coord(dim) = poly1d.dot(time);
        }
        state_list.push_back(coord);
      }
    }

    return state_list;
  }

  void KinodynamicAstar::getSamples(double &ts, vector<Eigen::Vector3d> &point_set,
                                    vector<Eigen::Vector3d> &start_end_derivatives)
  //将一条连续的多项式轨迹离散，获得一些轨迹点以及起始点的速度与加速度。   start_end_derivatives：  tart v   end v    start a  end a      
  //获得离散点：point_set      
  //此时的轨迹还是多项式轨迹                   
  {
    /* ---------- path duration ---------- *///得到整段路径时间T_sum
    double T_sum = 0.0;
    if (is_shot_succ_)
      T_sum += t_shot_;
    PathNodePtr node = path_nodes_.back();
    while (node->parent != NULL)
    {
      T_sum += node->duration;
      node = node->parent;
    }
    // cout << "duration:" << T_sum << endl;

    // Calculate boundary vel and acc//得到末尾速度和加速end_vel  end_acc//并初始化t为每段轨迹的时间
    Eigen::Vector3d end_vel, end_acc;
    double t;
    if (is_shot_succ_)
    {
      t = t_shot_;
      end_vel = end_vel_;
      for (int dim = 0; dim < 3; ++dim)
      {
        Vector4d coe = coef_shot_.row(dim);
        end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
      }
    }
    else
    {
      t = path_nodes_.back()->duration;
      end_vel = node->state.tail(3);
      end_acc = path_nodes_.back()->input;
    }

    // Get point samples
    int seg_num = floor(T_sum / ts);//floor（）向下取整数   ceil（）向上取整数   round（）四舍五入
    seg_num = max(8, seg_num);//轨迹至少分八段
    ts = T_sum / double(seg_num);//离散轨迹的时间
    bool sample_shot_traj = is_shot_succ_;
    node = path_nodes_.back();

    for (double ti = T_sum; ti > -1e-5; ti -= ts)
    {
      if (sample_shot_traj)
      {
        // samples on shot traj
        Vector3d coord;
        Vector4d poly1d, time;

        for (int j = 0; j < 4; j++)
          time(j) = pow(t, j);

        for (int dim = 0; dim < 3; dim++)
        {
          poly1d = coef_shot_.row(dim);
          coord(dim) = poly1d.dot(time);
        }

        point_set.push_back(coord);
        t -= ts;

        /* end of segment */
        if (t < -1e-5)
        {
          sample_shot_traj = false;
          if (node->parent != NULL)
            t += node->duration;
        }
      }
      else
      {
        // samples on searched traj
        Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
        Eigen::Matrix<double, 6, 1> xt;
        Vector3d ut = node->input;

        stateTransit(x0, xt, ut, t);

        point_set.push_back(xt.head(3));
        t -= ts;

        // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
        if (t < -1e-5 && node->parent->parent != NULL)
        {
          node = node->parent;
          t += node->duration;
        }
      }
    }
    reverse(point_set.begin(), point_set.end());

    // calculate start acc
    Eigen::Vector3d start_acc;
    if (path_nodes_.back()->parent == NULL)
    {
      // no searched traj, calculate by shot traj
      start_acc = 2 * coef_shot_.col(2);
    }
    else
    {
      // input of searched traj
      start_acc = node->input;
    }

    start_end_derivatives.push_back(start_vel_);
    start_end_derivatives.push_back(end_vel);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(end_acc);
  }

  std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
  {
    vector<PathNodePtr> visited;
    visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);//assign（）将容器一个区间的值复制到另一个区间
    return visited;
  }

  Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt) // 轨迹位置转换为栅格地图的index
  {
    Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

    // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
    // origin_(1)) * inv_resolution_),
    //     floor((pt(2) - origin_(2)) * inv_resolution_);

    return idx;
  }

  int KinodynamicAstar::timeToIndex(double time)//轨迹时间转化栅格地图为index
  {
    int idx = floor((time - time_origin_) * inv_time_resolution_);
    return idx;
  }

  void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1,
                                      Eigen::Vector3d um, double tau)
  {
    for (int i = 0; i < 3; ++i)
      phi_(i, i + 3) = tau;//tau扩展的时间//um扩展的加速度

    Eigen::Matrix<double, 6, 1> integral;//增量integral
    integral.head(3) = 0.5 * pow(tau, 2) * um;
    integral.tail(3) = tau * um;

    state1 = phi_ * state0 + integral;//前向状态state1
  }

} // namespace fast_planner

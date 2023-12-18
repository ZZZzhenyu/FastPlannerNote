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



#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/lambda4", lambda4_, -1.0);
  nh.param("optimization/lambda5", lambda5_, -1.0);
  nh.param("optimization/lambda6", lambda6_, -1.0);
  nh.param("optimization/lambda7", lambda7_, -1.0);
  nh.param("optimization/lambda8", lambda8_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/visib_min", visib_min_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {//设置栅格边界
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";//jerk平滑项
  if (cost_function_ & DISTANCE) cost_str += " dist  |";//障碍物距离项
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";//速度、加速度可达项
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";//在当前轨迹只是达到感知距离以外并未达到目标点时，目标函数需要加上ENDPOINT优化项，
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {//接口函数，负责设置以及调用optimize进行优化
/*首先是BsplineOptimizeTraj()函数，将优化的控制点，均匀B样条的时间间隔，cost Function包含的优化项，以及终止条件（最大优化次数及最长优化时间）都设置好以后，
利用BsplineOptimizer的optimize()函数进行优化*/
  setControlPoints(points);//返回控制点control_points_ 和维度dim_:3
  setBsplineInterval(ts);
  setCostFunction(cost_function);//会记录iter_num_（优化次数）、min_cost_（最小代价）、best_variable_（与之对应的优化变量）
  setTerminateCond(max_num_id, max_time_id);//终止条件（最大优化次数及最长优化时间

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
     //根据pt_num确定各部分优化项梯度vector的大小
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();//控制点个数
  g_q_.resize(pt_num);//优化过程中的控制点
  g_smoothness_.resize(pt_num);//smooth梯度
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);
     //根据是否有终值点约束确定优化变量的个数.variable_num_：优化变量个数*dim_
  if (cost_function_ & ENDPOINT) {//有endpoint优化项：优化变量x要考虑最后pb个控制点
    variable_num_ = dim_ * (pt_num - order_);//前pb个控制点不用优化
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /* do optimization using NLopt slover */
      //实例化Nlopt::opt类对象 opt。并设定目标函数。设定最大优化次数与最长优化时间，设定目标函数的最小值（这三者都是设定终止条件）
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);//给优化器设置使用什么优化算法，并设置参数个数
  opt.set_min_objective(BsplineOptimizer::costFunction, this);//设置优化目标
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);
      //根据线性拟合得到的控制点设置优化变量的初值，并设置每个优化变量的上下界（初始值±10）。
  vector<double> q(variable_num_);//优化变量q
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;//上下界参数
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }
      //利用Nlopt::opt类对行opt的optimize函数进行迭代优化求解，在求解结束后，通过costFunction中保留的best_Variable对control_point进行赋值
  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost);//执行优化//最小cost存储在final_cost//对应变量为q

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}//
//优化项计算
/*利用均匀B样条进行优化，首先要看三部分的优化项及相应梯度是如何表达的。三项优化项分别是jerk平顺项，障碍物距离项，以及速度、加速度可达项。

其中，平顺项，速度、加速度项都是利用均匀B样条的微分公式，求出速度控制点，加速度控制点，jerk控制点，从而构造与相应控制点相关的优化项。
梯度则是利用链式法则，用cost对影响这一cost的控制点求导，然后在循环中对cost及各控制点梯度进行累加（一个控制点会影响多个控制点的优化项）。

这里需要注意的是，速度与加速度的优化项及梯度都考虑了时间项Δ t，但jerk项省略了时间项-或许对应文中说的后续纯几何构造法，便于后续进行时间调整。但是，平顺项的实现方式与原论文中的elastic band的方法也是不一样的。

至于障碍物项。控制点距离最近障碍物的距离及这一距离关于控制点的梯度是通过SDF地图实现的，关于SDF地图留待以后讨论。

*/


//计算平滑项代价fs：jerk平顺项
void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);//fill函数可以为数组或者vector中的每个元素赋以相同的值，通常用于初始化！//begin()返回向量头指针，指向第一个元素 //end()返回向量尾指针，指向向量最后一个元素的下一个位置
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {//order_为3对应jerk
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];//不考虑时间项Δ t//速度、加速度、jerk控制点：vi=1/ts（Qi+1-Qi）；ai=1/ts/ts(Qi+2-Qi+1-(Qi+1-Qi));jerk=1/ts(ai+1-ai)
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient *///梯度是cost对影响其的控制点q[i+3]～q[i]分别求导//一段轨迹，只受pb+1个控制点影响
    gradient[i + 0] += -temp_j;//qi片导
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;//对q（i+3）的片导
  }
}
//障碍物距离项
void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;//有endpoint 则考虑最后pb个控制点，没有则不考虑
  /*在当前轨迹只是达到感知距离以外并未达到目标点时，目标函数需要加上ENDPOINT优化项，
  此时的优化变量应该包含最后pb​控制点。但当前端寻找的路径的状态已经是REACH_END时，
  由于拟合最后pb​个控制点已经能保证位置点约束，因此优化项中不再包含EDNPOINT，
  优化变量也不再包含最后pb​个控制点*/

  for (int i = order_; i < end_idx; i++) {//Pb~N-pb累加
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);//计算控制点q（i）与最近障碍物的距离dist，以及对q（i）的梯度dist_grad
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();//dist_grad变换为梯度方向

//https://blog.csdn.net/lemonxiaoxiao/article/details/121996477
//norm（）向量长度，normalized：向量方向
    if (dist < dist0_) {//dist0_：对应课程中dthr
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}
//va可达项
void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;//每段轨迹的时间
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {//外循环：针对控制点累加，pb-1～N-pb
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; j++) {//内循环：xyz
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;//v,a cost都考虑了deltat
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);//对qi的梯度
        gradient[i + 1](j) += temp_v * vi(j);//对q+1的梯度
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {//外循环：针对控制点累加，pb-2～N-pb
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {//约束bspline求得的终点中间点位置参数逼近多项式轨迹的位置参数
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;// 1 / 6.0 * (q_3 + 4 * q_2 + q_1) :bspline求得的轨迹点的p  //p=sMQ,针对p，第一行
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {//约束bspline求得的轨迹中间点位置参数逼近多项式轨迹的位置参数
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;//位置约束
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;//???????????????????????????????????

  for (int i = order_; i < end_idx; i++) {//一条bspline定义域：t[pb,M-pb+1]
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
/*结合三种优化项的函数是combineCost()函数，函数有三个参数，第一个std::vector& x即是Nlopt优化的变量，应该与三个维度的控制点对应。
第二个参数是std::vector& grad，即总的优化项关于每个优化变量的梯度信息。
第三个参数是double& f_combine 。即结合后的Cost。
*/
/*首先是给g_q赋值，g_q是用来计算每次优化循环三个优化项的控制点。值得注意的是，前 p b 个控制点和最后p b个控制点是不进行优化的，
始终保持线性拟合得到控制点原值。中间的控制点则是因为每一次迭代优化后都不同，因此用x来赋值。

这里的x是通过Nlopt的opt 对象在set_min_objective中进行初始化的，具体的大小在构造 Nlopt optimizer对象时就通过variable_num的大小确定了。
而初始值则是在Nlopt求解时 .optimize函数中进行赋值。

接下来就是利用CalcSmoothneesCost, CalcDistanceCost, CalcFeasibilityCost 三个函数计算每一部分的cost和grad,
并乘上权重后累加至grad 和 f_combine中，唯一需要注意的是grad和各部分优化项梯度之间维度上差了两个B样条次数。

最后，构建符合Nlopt要求的目标函数，即costFunction函数，该函数通过combinecost返回总的cost，并在参数中提供梯度信息。

*/
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.


  //1：首先是给g_q赋值，g_q是用来计算每次优化循环三个优化项的控制点。
      //前 p b 个控制点和最后p b个控制点是不进行优化的，始终保持线性拟合得到控制点原值。
  for (int i = 0; i < order_; i++) {//前pb个控制点不优化，保持为control_points_原值
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {//针对一维和二维bspline
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {//variable_num_ / dim_:优化变量个数
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {//?????????????????????????????cost_function_ & ENDPOINT的具体作用
    for (int i = 0; i < order_; i++) {//最后p b个控制点是不进行优化的

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }
//结合
  f_combine = 0.0;//结合后的cost
  grad.resize(variable_num_);//结合后的梯度信息
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);//？？？？？？？？？唯一需要注意的是grad和各部分优化项梯度之间维度上差了两个B样条次数
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;//iter_num:优化次数

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner
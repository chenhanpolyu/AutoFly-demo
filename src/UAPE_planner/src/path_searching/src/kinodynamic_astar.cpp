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

#include <path_searching/include/path_searching/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

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
  // Eigen::Vector3d start_vel_;
  // Eigen::Vector3d start_acc_;
  // cout<<"mark"<<start_v<<"\n"<<start_vel_<<endl;
  start_vel_ = start_v;
  // cout<<"mark"<<endl;
  start_acc_ = start_a;

  goal_ = end_pt;
  call_search_num++;
  // cout<<"expanded nodes size: "<<expanded_nodes_.size()<<endl;
  if (!checkSafety(goal_, -5.0))
  {
    // std::cout << "goal is occupied!" << std::endl;
    return GOAL_OCC;
  }
  // cout<<"mark"<<endl;
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

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
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

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
        computeShotTraj(cur_node->state, end_state, time_to_goal);
        if (init_search)
          ROS_ERROR("[Hybrid A*] Shot in first search loop!");
      }
    }
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "[Hybrid A*] reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout << "[Hybrid A*] reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "[Hybrid A*] reach end" << std::endl;
        return REACH_END;
      }
      else if (cur_node->parent != NULL)
      {
        std::cout << "[Hybrid A*] near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "[Hybrid A*] no path" << std::endl;
        return NO_PATH;
      }
    }
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    // cout<<"close node: "<<cur_node<<endl;
    iter_num_ += 1;

    double res = 1 / acc_sample_num_, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
    }
    else
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    // cout << "cur state:" << cur_state.head(3).transpose() << endl << cur_state.tail(3).transpose()<<endl<<allocate_num_<<endl<<inputs.size()<<endl<<init_search<<endl;
    for (size_t i = 0; i < inputs.size(); ++i)
      for (size_t j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // Check if in close set
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          // std::cout << "close" << std::endl;
          // if (init_search)

          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (!init_search)
          //   {std::cout << "vel: " << pro_v<<"  "<<max_vel_<<std::endl;}
          // else
          {
            continue;
          }
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          // if (init_search)
          //   std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        double current_time;
        Eigen::Vector3d obs_pos;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          current_time = cur_node->time + dt;
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          // if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 )

          if (!checkSafety(pos, current_time))
          {
            // cout << "check safe fail:" <<pos<<endl;
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          // if (init_search)
          //   std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // cout << "score of the node: "<<tmp_g_score<<"  "<< tmp_f_score <<endl;
        // Compare nodes expanded from the same parent
        bool prune = false;
        for (size_t j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            if (pro_node->node_state == IN_CLOSE_SET)
            {
              cout << "error caught: " << pro_node->node_state << "  state: " << pro_node->state << endl;
            }
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
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "[Hybrid A*] run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
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
            cout << "[Hybrid A*] error type in searching: " << pro_node->node_state << "  state: " << pro_node->state << "  call_search_num: " << call_search_num << endl;
            if (pro_node->parent != NULL)
              cout << " parent state: " << pro_node->parent->node_state << "  " << pro_node->parent->state << endl;
            cout << "expanded close node: " << pro_node << endl;
          }
        }
      }
    init_search = false;
  }

  cout << "[Hybrid A*] open set empty, no path!" << endl;
  cout << "[Hybrid A*] use node num: " << use_node_num_ << endl;
  cout << "[Hybrid A*] iter num: " << iter_num_ << endl;
  cout << "[Hybrid A*] kd-tree quiry times: " << quiry_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(ros::NodeHandle &nh)
{
  nh.getParam("search/max_tau", max_tau_);
  nh.getParam("search/init_max_tau", init_max_tau_);
  nh.getParam("VelMax", max_vel_);
  nh.getParam("PathAccMax", max_acc_);
  nh.getParam("search/w_time", w_time_);
  nh.getParam("search/horizon", horizon_);
  nh.getParam("search/resolution_astar", resolution_);
  nh.getParam("search/time_resolution", time_resolution_);
  nh.getParam("search/lambda_heu", lambda_heu_);
  nh.getParam("search/allocate_num", allocate_num_);
  nh.getParam("search/check_num", check_num_);
  nh.getParam("search/optimistic", optimistic_);
  nh.getParam("search/safety_radius", S_r);
  nh.getParam("search/acc_sample_num_", acc_sample_num_);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.getParam("search/vel_margin", vel_margin);
  max_vel_ += vel_margin;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
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
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;
  if ((x1.head(3) - camera_vertex_.col(0)).dot((camera_vertex_.col(0) - camera_vertex_.col(1)).cross(camera_vertex_.col(0) - camera_vertex_.col(2))) < 0 || (x1.head(3) - camera_vertex_.col(0)).dot((camera_vertex_.col(0) - camera_vertex_.col(2)).cross(camera_vertex_.col(0) - camera_vertex_.col(3))) < 0 || (x1.head(3) - camera_vertex_.col(0)).dot((camera_vertex_.col(0) - camera_vertex_.col(3)).cross(camera_vertex_.col(0) - camera_vertex_.col(4))) < 0 || (x1.head(3) - camera_vertex_.col(0)).dot((camera_vertex_.col(0) - camera_vertex_.col(4)).cross(camera_vertex_.col(0) - camera_vertex_.col(1))) < 0 || (x1.head(3) - camera_vertex_.col(1)).dot((camera_vertex_.col(1) - camera_vertex_.col(4)).cross(camera_vertex_.col(1) - camera_vertex_.col(2))) < 0)
  {
    cost *= 1.5;
    //  cout << "sample out of FOV"<<endl;
  }
  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (size_t j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (size_t dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    // if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)
    // current_time = cur_node->time + dt;
    if (!checkSafety(coord, -1.0))
    {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
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
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
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
  // edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "[Hybrid A*] origin_: " << origin_.transpose() << endl;
  cout << "[Hybrid A*] map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
  call_search_num = 0;
}

// template <typename num_t>
void KinodynamicAstar::setEnvironment(vec_Vec3f *cloud, dynobs_tmp *dynamic_obs, Eigen::Matrix<double, 3, 5> &camera_vertex, vector<double> glbox_o, vector<double> glbox_l)
{
  // this->st_cloud_ = cloud；
  this->dynobs_pointer_ = dynamic_obs;
  pcPtr_ = std::make_shared<Obs>();
  // The adaptor
  // cout << "get dyn"<<endl;
  pcPtr_->pts = cloud;
  // cout << "get cloud:\n"<<cloud->size()<<endl;
  kdPtr_ = std::make_shared<my_kd_tree_t>(3, *pcPtr_);
  // cout << "build index"<<endl;
  kdPtr_->buildIndex();
  // construct a kd-tree index:
  camera_vertex_ = camera_vertex;
  // my_kd_tree_t index(3 /*dim*/, pc2kd_, nanoflann::KDTreeSingleIndexAdaptorParams(20 /* max leaf */));
  // index.buildIndex();
  time_offset = ros::Time::now().toSec() - dynamic_obs->time_stamp;
  // max_height_ = max_height;
  glbox_o_ = glbox_o;
  glbox_l_ = glbox_l;
}
inline bool KinodynamicAstar::checkSafety(const Eigen::Vector3d &query_pt, const double current_time)
{
  // do a knn search
  if (pcPtr_->pts->size() == 0)
  {
    return true;
  }
  quiry_num_ += 1;
  size_t ret = kdPtr_->knnSearch(query_pt.data(), 1, &nearest_index, &dist_sqr);
  bool global_range_safe = (query_pt(2) + S_r < glbox_o_[2] + glbox_l_[2]) && (query_pt(2) - S_r > glbox_o_[2]) && (query_pt(1) + S_r < glbox_o_[1] + glbox_l_[1]) && (query_pt(1) - S_r > glbox_o_[1]) && (query_pt(0) + S_r < glbox_o_[0] + glbox_l_[0]) && (query_pt(0) - S_r > glbox_o_[0]);
  // cout << "kd tree result:"<<dist_sqr<<"---"<<nearest_index<<"---"<<S_r<<"\n"<<pcPtr_->pts->at(nearest_index) <<endl<<query_pt<<endl;
  if (current_time < -2)
  {
    return (dist_sqr > 1 * S_r) && global_range_safe;
  }
  else if (current_time < 0 || dynobs_pointer_->dyn_number == 0)
  {
    return (dist_sqr > 0.7*S_r) && global_range_safe;
  }

  double time = current_time + time_offset;
  bool dyn_safe = true;
  // cout << "times for dyn check: (kino)" << time << "---"<< time_offset<<endl;
  for (int i = 0; i < dynobs_pointer_->dyn_number; i++)
  {
    Eigen::Vector3d check_vec = ((dynobs_pointer_->centers[i] + (time * dynobs_pointer_->vels[i]) - query_pt).cwiseAbs() - dynobs_pointer_->obs_sizes[i] * 0.5);
    if ((check_vec.array() < 1.5*S_r).all())

    {
      dyn_safe = false;
      break;
    }
  }
  // nnpt = pcPtr_->pts[nearest_index];
  return (dist_sqr > S_r) && dyn_safe && global_range_safe;
}
void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();
  // NodeHashTable expanded_nodes_new;
  // expanded_nodes_ = expanded_nodes_new;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    // PathNodePtr node = ;
    path_node_pool_[i]->parent = NULL;
    path_node_pool_[i]->node_state = NOT_EXPAND;
    // path_node_pool_[i] = new PathNode;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
  quiry_num_ = 0;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
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

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (size_t j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (size_t dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

bool KinodynamicAstar::checkOldPath(vector<Eigen::Vector3d> &point_set, Eigen::Vector3d ct_pos)
{
  bool if_safe = true;
  int del_id = -1;
  if (point_set.size() < 2)
  {
    return false;
  }
  for (size_t i = 0; i < point_set.size() - 1; i++)
  {
    if (line_collide(point_set[i], point_set[i + 1]))
    {
      if_safe = false;
      break;
    }
    if ((point_set[i] - ct_pos).squaredNorm() + (point_set[i] - point_set.back()).squaredNorm() > (point_set.back() - ct_pos).squaredNorm())
      // && (point_set[i + 1] - ct_pos).squaredNorm() + (point_set[i + 1] - point_set.back()).squaredNorm() < (point_set.back() - ct_pos).squaredNorm())
      del_id = i;
  }
  // ct_pos -
  if (if_safe)
  {
    del_id = max(1, del_id + 1);
    double  rate = (ct_pos - point_set[del_id - 1]).dot(point_set[del_id] - point_set[del_id - 1])/ (point_set[del_id] - point_set[del_id - 1]).squaredNorm();
    Eigen::Vector3d new_start = point_set[del_id - 1] + (point_set[del_id] - point_set[del_id - 1]) *rate;
    point_set.erase(point_set.begin(), point_set.begin() + del_id);
    if (point_set.size() == 1)
      point_set.insert(point_set.begin(), ct_pos);
    else
      point_set.insert(point_set.begin(), new_start);
  }
  if (!if_safe)
    cout << "A* old path is not safe! " << endl;
  return if_safe;
}
bool KinodynamicAstar::line_collide(Eigen::Vector3d &p1, Eigen::Vector3d &p2)
{
  double dist = (p1 - p2).norm();
  int num = int(dist / 0.2);
  for (auto i = 1; i <= num; i++)
  {
    if (!checkSafety(p1 + (p2 - p1) * i / num, -1))
      return true;
  }
  return false;
}
void KinodynamicAstar::getSamples(double ts, vector<Eigen::Vector3d> &point_set,
                                  vector<Eigen::Vector3d> &start_end_derivatives)
{
  /* ---------- path duration ---------- */
  point_set.clear();
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

  // Calculate boundary vel and acc
  Eigen::Vector3d end_vel, end_acc, last_pos;
  double t;
  if (is_shot_succ_)
  {
    t = t_shot_;
    end_vel = end_vel_;
    for (size_t dim = 0; dim < 3; ++dim)
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
  int seg_num = floor(T_sum / ts);
  seg_num = max(8, seg_num);
  ts = T_sum / double(seg_num);
  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();
  point_set.push_back(goal_);
  Vector3d coord;
  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    if (sample_shot_traj)
    {
      // samples on shot traj

      Vector4d poly1d, time;

      for (size_t j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (size_t dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      // cout<<"coord and last wpt:\n"<<coord<<"\n"<<point_set.back()<<endl;
      if (ti<T_sum - ts-1e-3 && line_collide(coord, point_set.back()))
        point_set.push_back(last_pos);
      else if (ti < ts && (point_set.back() - coord).norm() > 0.3)
        point_set.push_back(coord);
      // cout<<"last_pos:\n"<<last_pos<<"\n"<<point_set.back()<<endl;}
      last_pos = coord;
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
      coord = xt.head(3);
      // cout << "return node:"<<xt.head(3)<<endl;
      //  cout<<"coord and last wpt:\n"<<coord<<"\n"<<point_set.back()<<endl;
      if (ti<T_sum - ts-1e-3 && line_collide(coord, point_set.back()))
        point_set.push_back(last_pos);
      else if (ti < ts && (point_set.back() - coord).norm() > 0.3)
        point_set.push_back(coord);
      // cout<<"last_pos:\n"<<last_pos<<"\n"<<point_set.back()<<endl;}
      last_pos = coord;
      // point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  //   if ((point_set.back() - coord).norm() > 0.5)
  //   {
  // cout<<"coord and last wpt:\n"<<coord<<"\n"<<point_set.back()<<endl;
  //    point_set.push_back(coord);
  //   }
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
  start_end_derivatives.clear();
  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1,
                                    Eigen::Vector3d um, double tau)
{
  for (size_t i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

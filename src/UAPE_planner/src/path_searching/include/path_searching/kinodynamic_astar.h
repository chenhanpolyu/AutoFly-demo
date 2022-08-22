#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// #include <path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud.h>
#include <path_searching/include/path_searching/nanoflann.hpp>
#include <call_states/ros_communicate.h>
#include <traj_opt/se3_planner.h>
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30
// using namespace nanoflann;
class PathNode
{
public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time; // dyn
  int time_idx;
  PathNode *parent;
  char node_state;

  /* -------------------- */
  PathNode()
  {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode *PathNodePtr;

class NodeComparator
{
public:
  bool operator()(PathNodePtr node1, PathNodePtr node2)
  {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node)
  {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node)
  {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx)
  {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
    data_4d_.clear();
  }
  int size()
  {
    return data_3d_.size() + data_4d_.size();
  }
};

class KinodynamicAstar
{
private:
  /* ---------- main data structure ---------- */
  struct Obs
  {

    // sensor_msgs::PointCloud *pts;
    vec_Vec3f *pts;

    // Must return the number of data points
    // inline size_t kdtree_get_point_count() const { return pts->points.size(); }
    inline size_t kdtree_get_point_count() const { return pts->size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    // inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    // {
    // 	if (dim == 0) return pts->points[idx].x;
    // 	else if (dim == 1) return pts->points[idx].y;
    // 	else return pts->points[idx].z;
    // }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
      if (dim == 0)
        return pts->at(idx)(0);
      else if (dim == 1)
        return pts->at(idx)(1);
      else
        return pts->at(idx)(2);
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
  };

  // Obs pc2kd_;
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, Obs>,
      Obs, 3 /* dim */>
      my_kd_tree_t;
  // my_kd_tree_t index(3 /*dim*/, pc2kd_, nanoflann::KDTreeSingleIndexAdaptorParams(20 /* max leaf */));
  std::shared_ptr<Obs> pcPtr_;
  std::shared_ptr<my_kd_tree_t> kdPtr_;
  // my_kd_tree_t index;
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Vector3d goal_;
  double max_height_;
  vector<double> glbox_o_, glbox_l_;
  Eigen::Matrix<double, 6, 6> phi_; // state transit matrix
  // shared_ptr<SDFMap> sdf_map;
  sensor_msgs::PointCloud st_cloud_;
  dynobs_tmp *dynobs_pointer_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_, S_r;
  double max_vel_, max_acc_, acc_sample_num_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_ = {500.0, 500.0, 500.0};
  Eigen::Vector3d map_size_3d_ = {-500.0, -500.0, -500.0};
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double &optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1> &state0,
                    Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um,
                    double tau);
  inline bool checkSafety(const Eigen::Vector3d &query_pt, const double current_time);
  inline bool line_collide(Eigen::Vector3d &p1, Eigen::Vector3d &p2);
  double dist_sqr;
  size_t nearest_index = 0;
  double time_offset;
  int quiry_num_;
  Eigen::Matrix<double, 3, 5> camera_vertex_;
  double v_camera_fov = 0;
  int call_search_num;

public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3,
    NEAR_END = 4,
    GOAL_OCC = 5
  };

  /* main API */
  void setParam(ros::NodeHandle &nh);
  void init();
  void reset();
  bool checkOldPath(vector<Eigen::Vector3d> &point_set, Eigen::Vector3d ct_pos);
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);

  void setEnvironment(vec_Vec3f *cloud, dynobs_tmp *dynamic_obs, Eigen::Matrix<double, 3, 5> &camera_vertex, vector<double> glbox_o, vector<double> glbox_l);
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  void getSamples(double ts, vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
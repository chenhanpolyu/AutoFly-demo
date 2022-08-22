#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
#include <Tools.h>
#include <ros/ros.h>
#include <cstdlib>
#include <ros_missn.h>
//#include <controllers/backstepping.h>
#include <planner_fast.h>
#include <Tools.h>
#include <path_searching/include/path_searching/kinodynamic_astar.h>
//#include <logs/log_flight.h>

using namespace std;
using namespace Eigen;

// #define CtrlFreq 50
// #define MaxVel 2.0
double sign(double x)
{
  return (x > 0) - (x < 0);
}
unique_ptr<KinodynamicAstar> kino_path_finder_;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "traj_planner");
  // Create a handle to this process node.
  ros::NodeHandle nh("~");

  Vector3d ct_pos, ct_vel, ct_acc, home_pos;
  ct_pos.setZero();
  Vector3d p_d, v_d, a_d, p_d_yaw;
  a_d.setZero();
  MatrixXd sp_pos, sp_vel, sp_acc, waypoints_m;
  // double last_path_t = 0;

  Eigen::Vector3d end_state = Eigen::Vector3d::Zero(3);
  vector<double> goalp, gbbox_o, gbbox_l, timecosts;
  Matrix<double, 3, 5> camera_vertex, camera_vertex_b, camera_vertex_bv;
  vector<Eigen::Vector3d> waypoints, start_end_derivatives;
  int status;
  double d2r = 3.14159265 / 180;
  double cam_depth = 10.0;
  double cam_depth_v = 2.0;
  double h_fov = 87; // in degree
  double v_fov = 58;
  double dis_goal, tem_dis_goal;
  // double sfck_t;
  bool ifMove, if_rand;
  int CtrlFreq;
  bool if_debug;
  bool path_replan;
  bool if_reach = false;
  bool last_if_reach = false;
  double gap;
  double singlestep_time;
  bool rc_goal = false;
  int return_home = 0;
  bool if_raw_pcl = false; // if directly use the raw point cloud from sensor
  bool if_depth_img = false;
  nh.getParam("goal", goalp);
  nh.getParam("search/horizon", dis_goal);
  nh.getParam("ifMove", ifMove);
  nh.getParam("cam_depth", cam_depth);
  nh.getParam("h_fov", h_fov);
  nh.getParam("v_fov", v_fov);
  nh.getParam("if_RandomGoal", if_rand);
  nh.getParam("GlobalBox_min", gbbox_o);
  nh.getParam("GlobalBox_size", gbbox_l);
  nh.getParam("CtrlFreq", CtrlFreq);
  nh.getParam("if_debug", if_debug);
  nh.getParam("UseRawPcl", if_raw_pcl);
  nh.getParam("UseRawDepth", if_depth_img);

  nh.getParam("ReturnHome", return_home);
  nh.getParam("UseRcGuide", rc_goal);

  ros::Rate loop_rate(CtrlFreq);
  camera_vertex_b.col(0) << 0, 0, 0;
  camera_vertex_b.col(1) << cam_depth, tan(h_fov / 2 * d2r) * cam_depth, tan(v_fov / 2 * d2r) * cam_depth;
  camera_vertex_b.col(2) << cam_depth, -tan(h_fov / 2 * d2r) * cam_depth, tan(v_fov / 2 * d2r) * cam_depth;
  camera_vertex_b.col(3) << cam_depth, -tan(h_fov / 2 * d2r) * cam_depth, -tan(v_fov / 2 * d2r) * cam_depth;
  camera_vertex_b.col(4) << cam_depth, tan(h_fov / 2 * d2r) * cam_depth, -tan(v_fov / 2 * d2r) * cam_depth;

  camera_vertex_bv.col(0) << 0, 0, 0;
  camera_vertex_bv.col(1) << cam_depth_v, tan(h_fov / 2 * d2r) * cam_depth_v, tan(v_fov / 2 * d2r) * cam_depth_v;
  camera_vertex_bv.col(2) << cam_depth_v, -tan(h_fov / 2 * d2r) * cam_depth_v, tan(v_fov / 2 * d2r) * cam_depth_v;
  camera_vertex_bv.col(3) << cam_depth_v, -tan(h_fov / 2 * d2r) * cam_depth_v, -tan(v_fov / 2 * d2r) * cam_depth_v;
  camera_vertex_bv.col(4) << cam_depth_v, tan(h_fov / 2 * d2r) * cam_depth_v, -tan(v_fov / 2 * d2r) * cam_depth_v;
  TrajectoryGenerator_fast reference(camera_vertex_b);
  // dis_goal = dis_goal_ini-0.5;
  Eigen::Vector3d g_goal = {goalp[0], goalp[1], goalp[2]};
  Eigen::Vector3d goal = g_goal;
  Eigen::Vector3d local_goal = {0, 0, 0};
  Eigen::Vector3d initial_goal = g_goal;
  bool if_initial = true;
  bool if_end = false;
  bool if_safe = true;
  bool ball_time_out = false;
  double ball_pass_time = 2.0;
  double min_dist2dynobs = 1e6;
  double tmp_dist, t_gap_ball;
  double timee = 0;
  ros::Time traj_last_t = ros::Time::now();
  chrono::high_resolution_clock::time_point last_traj_tic = chrono::high_resolution_clock::now();
  int rand_num = -1, rand_num_tmp;
  reference.read_param(&nh);
  States state;
  cout << "Traj node initialized!" << endl;
  RosClass flying(&nh, CtrlFreq, if_raw_pcl, if_depth_img);
  do
  {
    state = flying.get_state();
    ct_pos = state.P_E;
    ros::Duration(0.1).sleep();
  } while (ct_pos.norm() < 1e-3);
  cout << "UAV message received!" << ct_pos << endl;
  reference.last_check_pos = ct_pos;
  home_pos = ct_pos;
  // flying.dynobs_pointer->ball_number = 0;
  while (!flying.pcl_update)
  {
    ros::Duration(0.05).sleep();
    state = flying.get_state();
  }
  cout << "Point cloud received!\n"
       << endl;

  // ------- start the safety check and re-planning loop--------------------------------------------------------------
  kino_path_finder_.reset(new KinodynamicAstar);
  kino_path_finder_->setParam(nh);
  kino_path_finder_->init();
  while (nh.ok()) // main loop
  {
    // dis_goal = dis_goal_ini-0.5;

    path_replan = false;
    if ((return_home > 0 && if_end) || (if_rand && (if_end || (rand_num < 0)))) // choose goal randomly at the global bounding box boundary
    {
      if (if_rand)
      {
        if (rand_num < 0)
          rand_num = rand() % 4;
        else
        {
          do
            rand_num_tmp = rand() % 4;
          while (rand_num_tmp == rand_num);
          rand_num = rand_num_tmp;
        }
        if (rand_num == 0)
        {
          g_goal(0) = gbbox_o[0] + gbbox_l[0] - 0.8;
          g_goal(1) = gbbox_o[1] + rand() % int(gbbox_l[1]) + 0.8;
        }
        else if (rand_num == 1)
        {
          g_goal(0) = gbbox_o[0] + rand() % int(gbbox_l[0]) + 0.8;
          g_goal(1) = gbbox_o[1] + gbbox_l[1] - 0.8;
        }
        else if (rand_num == 2)
        {
          g_goal(0) = gbbox_o[0] + 0.8;
          g_goal(1) = gbbox_o[1] + rand() % int(gbbox_l[1]) + 0.8;
        }
        else
        {
          g_goal(0) = gbbox_o[0] + rand() % int(gbbox_l[0]) + 0.8;
          g_goal(1) = gbbox_o[1] + 0.8;
        }
        g_goal(2) = 1.0;
        goal = g_goal;
      }
      else if (return_home > 1 && if_end)
      {
        if ((home_pos - ct_pos).norm() > 1)
          g_goal = home_pos;
        else
          g_goal = initial_goal;
        goal = g_goal;
        cout << "one goal: " << g_goal << endl;
        return_home--;
      }
      // else if (return_home <=1)
      // {
      //   cout<<"Reach the maximal trip number, exit!"<<endl;
      //   break;
      // }
      // cout<<"goal:"<<goal<<" "<<g_goal<<endl;
      if_end = false;
      if_initial = true;
      if_reach = false;
      waypoints.clear();
      flying.dynobs_pointer->dyn_number = 0;
      Vector2d v2 = (g_goal - state.P_E).head(2);
      Vector2d v1;
      v1 << 1.0, 0.0;
      double desire_yaw = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
      if (v2(1) < 0)
      {
        desire_yaw = -desire_yaw;
      }
      ct_pos = state.P_E;
      double yaw_rate = abs(desire_yaw - state.Euler(2)) > 3.14159 ? -sign(desire_yaw - state.Euler(2)) * 0.6 : sign(desire_yaw - state.Euler(2)) * 0.6;
      ros::Duration(0.5).sleep();
      do
      {
        state = flying.step(state.Euler(2) + yaw_rate * 0.5, yaw_rate, ct_pos, Vector3d::Zero(3), Vector3d::Zero(3), "pos_vel_acc_yaw_c");
        // cout<<"turn to goal: "<< state.Euler(2) + yaw_rate * 0.5 << endl;
        ros::Duration(0.05).sleep();
      } while (abs(state.Euler(2) - desire_yaw) > 0.3);
      ros::Duration(0.5).sleep();
    }
    else if (rc_goal)
    {
      // state = flying.get_state();
      local_goal = {flying.rc_data.ch[1], flying.rc_data.ch[0], clip(flying.rc_data.ch[3], -0.3, 0.3)};
      ct_pos = state.P_E;
      double yaw_fix = state.Euler(2);
      while ((local_goal * dis_goal).norm() < 0.5)
      {
        state = flying.step(yaw_fix, 0, ct_pos, Vector3d::Zero(3), Vector3d::Zero(3), "pos_vel_acc_yaw_c");
        local_goal = {flying.rc_data.ch[1], flying.rc_data.ch[0], 0};
        ros::Duration(0.1).sleep();
        if_end = false;
        if_initial = true;
        if_reach = false;
        waypoints.clear();
        flying.dynobs_pointer->dyn_number = 0;
        cout << "RC hover" << endl;
      }

      while ((local_goal * dis_goal).norm() < 2.0)
      {
        state = flying.step(yaw_fix, 0, state.P_E + local_goal, Vector3d::Zero(3), Vector3d::Zero(3), "pos_vel_acc_yaw_c");
        local_goal = {flying.rc_data.ch[1], flying.rc_data.ch[0], clip(flying.rc_data.ch[3], -0.3, 0.3)};
        ros::Duration(0.1).sleep();
        if_end = false;
        if_initial = true;
        if_reach = false;
        waypoints.clear();
        flying.dynobs_pointer->dyn_number = 0;
        cout << "RC move" << endl;
      }

      g_goal = state.P_E + local_goal * dis_goal;
      goal = g_goal;
      Vector2d v2 = (g_goal - state.P_E).head(2);
      Vector2d v1;
      v1 << 1.0, 0.0;
      double desire_yaw = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
      if (v2(1) < 0)
      {
        desire_yaw = -desire_yaw;
      }
      ct_pos = state.P_E;
      double yaw_rate = abs(desire_yaw - state.Euler(2)) > 3.14159 ? -sign(desire_yaw - state.Euler(2)) * 0.6 : sign(desire_yaw - state.Euler(2)) * 0.6;
      while (abs(state.Euler(2) - desire_yaw) > 0.3 && if_initial)
        ;
      {
        state = flying.step(state.Euler(2) + yaw_rate * 0.5, yaw_rate, ct_pos, Vector3d::Zero(3), Vector3d::Zero(3), "pos_vel_acc_yaw_c");
        ros::Duration(0.05).sleep();
      }
      reference.config.velMax = clip(flying.rc_data.ch[2] + 1, 0.3, 2.0);
    }

    // ros::Time t1 = ros::Time::now();
    chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
    if (flying.pcl_update || flying.dyn_update)
    {

      state = flying.get_state();
      // flying.set_cod_update(false);
      if (flying.pcl_update && flying.obs_pointer->size() > 0)
        kino_path_finder_->setEnvironment(flying.obs_pointer, flying.dynobs_pointer, camera_vertex, gbbox_o, gbbox_l);
      if (flying.dynobs_pointer->ball_number > 0 && ros::Time::now().toSec() - flying.dynobs_pointer->ball_time_stamp > ball_pass_time) // for bag sim// flying.dynobs_pointer->ball_number>0 && (flying.dynobs_pointer->ballvel[0](0) > -0.2)||
      {
        if (if_debug)
          cout << "dyn ball time out: " << flying.dynobs_pointer->ballvel[0] << endl;
        flying.dynobs_pointer->ball_number = 0;
        ball_time_out = true;
        // double p_gap = flying.dynobs_pointer->ballpos[0](0) - ct_pos(0);
        // double ball_pass_time1 = (-flying.dynobs_pointer->ballvel[0](0) + sqrt(pow(flying.dynobs_pointer->ballvel[0](0),2)-2*flying.dynobs_pointer->ballacc[0](0)*p_gap))/(2*p_gap);
        // double ball_pass_time2 = (-flying.dynobs_pointer->ballvel[0](0) - sqrt(pow(flying.dynobs_pointer->ballvel[0](0),2)-2*flying.dynobs_pointer->ballacc[0](0)*p_gap))/(2*p_gap);
        // if (ball_pass_time1>0)
        // {ball_pass_time = ball_pass_time1;}
        // else if (ball_pass_time2>0)
        // {ball_pass_time = ball_pass_time2;}
        // else{ball_pass_time = 2;}
        // cout<<"ball_pass_time: "<<ball_pass_time<<endl;
      }
      if (flying.dynobs_pointer->dyn_number > 0 && ros::Time::now().toSec() - flying.dynobs_pointer->time_stamp > ball_pass_time) // for bag sim// flying.dynobs_pointer->ball_number>0 && (flying.dynobs_pointer->ballvel[0](0) > -0.2)||
      {
        flying.dynobs_pointer->dyn_number = 0;
        cout << "dyn time out! " << endl;
      }
      min_dist2dynobs = 1e6;
      for (int bi = 0; bi < flying.dynobs_pointer->dyn_number; bi++)

      {
        t_gap_ball = ros::Time::now().toSec() - flying.dynobs_pointer->time_stamp;
        tmp_dist = (state.P_E - flying.dynobs_pointer->centers[bi] - t_gap_ball * flying.dynobs_pointer->vels[bi]).norm();
        if (tmp_dist < min_dist2dynobs)
        {
          min_dist2dynobs = tmp_dist;
          if (if_debug)
            cout << "min distance from objects to drone:" << min_dist2dynobs << endl
                 << state.P_E << endl
                 << flying.dynobs_pointer->centers[bi] + t_gap_ball * flying.dynobs_pointer->vels[bi];
        }
      }
      min_dist2dynobs = 1e6;
      for (int di = 0; di < flying.dynobs_pointer->ball_number && flying.dynobs_pointer->ballvel[0](0) < -0.4; di++)
      {
        t_gap_ball = ros::Time::now().toSec() - flying.dynobs_pointer->ball_time_stamp;
        tmp_dist = (state.P_E - (flying.dynobs_pointer->ballpos[di] + t_gap_ball * flying.dynobs_pointer->ballvel[di] + 0.5 * t_gap_ball * t_gap_ball * flying.dynobs_pointer->ballacc[di])).norm();
        if (tmp_dist < min_dist2dynobs)
        {
          min_dist2dynobs = tmp_dist;
          if (if_debug)
            cout << "min distance from ball to drone:" << min_dist2dynobs << endl;
        }
      }
      if (if_initial || !ifMove)
      {
        ct_pos = state.P_E;
        // ct_vel = state.V_E;
        // ct_acc = state.A_E;
        ct_vel.setZero();
        ct_acc.setZero();
      }
      else
      {
        ct_pos = p_d;
        ct_vel = v_d;
        ct_acc = a_d;
      }
      last_if_reach = if_reach;
      if (if_debug)
        cout << "track goal dist: " << (state.P_E - p_d).norm() << endl;
      // cout<<"if initial: "<<if_initial<<endl;
      camera_vertex = (state.Rota * camera_vertex_b).array().colwise() + state.P_E.array();
      double dis2goal = (g_goal - ct_pos).norm();

      if (dis2goal > 1.0 && flying.obs_pointer->size() > 0) // && !if_initial
      {
        // cout << "The obs pointer:\n" << flying.obs_pointer << "---pcl size: "<< flying.obs_pointer->size()<< endl;
        // cout<<"goal:"<<goal<<" "<<g_goal<<endl;
        chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
        // cout << "The obs pointer:\n" << flying.obs_pointer << "---pcl size: "<< flying.obs_pointer->size()<< endl;
        if (!kino_path_finder_->checkOldPath(waypoints, ct_pos) || (reference.last_jointPolyH_check(ct_pos) && !if_reach))
        {
          cout << "Ready to search" << endl;

          if (dis2goal > dis_goal)
          {
            goal = ct_pos + (g_goal - ct_pos) / dis2goal * dis_goal;
            if_reach = false;
            tem_dis_goal = dis_goal;
          }
          else
          {
            goal = g_goal;
            if_reach = true;
            tem_dis_goal = dis2goal;
          }
          kino_path_finder_->reset();
          status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, true);
          // last_path_t = ros::Time::now().toSec();
          while (status == KinodynamicAstar::GOAL_OCC)
          {
            cout << "[kino replan]: Goal occluded:\n"
                 << goal << endl;
            tem_dis_goal -= 0.3;
            goal = ct_pos + (g_goal - ct_pos).normalized() * tem_dis_goal;
            if (if_reach)
              g_goal = goal;
            if ((goal - state.P_E).norm() < 0.5 || tem_dis_goal < 0.5)
            {
              g_goal = goal;
              if_reach = false;
              break;
            }
            kino_path_finder_->reset();
            status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, true);
            // if_reach = false;
          }
          if (status == KinodynamicAstar::NO_PATH)
          {
            cout << "[kino replan]: kinodynamic search fail!" << endl;

            // retry searching with discontinuous initial state
            kino_path_finder_->reset();
            status = kino_path_finder_->search(ct_pos, ct_vel, ct_acc, goal, end_state, false);
            // last_path_t = ros::Time::now().toSec();
            if (status == KinodynamicAstar::NO_PATH)
            {
              if_reach = false;
              cout << "[kino replan]: Can't find path. Please restart" << endl;
              // return 0;
            }
            else
            {
              cout << "[kino replan]: retry search success." << endl;
            }
          }
          // else
          // {
          //   last_path_t = ros::Time::now().toSec();
          // }
          double compTime = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
          cout << "kino path planning finished! time cost (ms)： " << compTime << endl;
          if (status != KinodynamicAstar::NO_PATH && status != KinodynamicAstar::GOAL_OCC)
          {
            kino_path_finder_->getSamples(0.2, waypoints, start_end_derivatives);
          }
          path_replan = true;
        }
        else
        {
          // cout << "[kino replan]: Old kino path is safe" << endl;
        }
      }
      else
      {
        waypoints.clear();
        waypoints.emplace_back(ct_pos);
        waypoints.emplace_back(goal);
      }
      MatrixXd waypoints_m = Map<MatrixXd>(waypoints[0].data(), 3, waypoints.size());
      // cout<<"waypoints: \n"<<waypoints_m<<endl;
      if (!if_initial)
      {
        if (ifMove)
          if_safe = reference.check_polyH_safe(traj_last_t.toSec(), waypoints_m, ct_pos, flying.obs_pointer, flying.dynobs_pointer, ros::Time::now().toSec(), path_replan, flying.pcl_update);
        else
          if_safe = reference.check_polyH_safe(ros::Time::now().toSec(), waypoints_m, ct_pos, flying.obs_pointer, flying.dynobs_pointer, ros::Time::now().toSec(), path_replan, flying.pcl_update);
      }
      flying.set_pcl_update(false);
      // cout << "corridor update, check safety result:" << if_safe <<endl;
      if (if_debug)
        cout << "point cloud update, check safety result:" << if_safe << endl;
      if (if_safe && !if_initial && !((!last_if_reach) && if_reach) && !ball_time_out) //&& (timee+sfck_t < reference.total_t || reference.total_t <sfck_t)
      {
        gap = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - last_traj_tic).count() * 1.0e-6;
        timee = gap ; //+ 1 / CtrlFreq / 4;
      }

      else
      {
        cout << "Dynamic obj number: " << flying.dynobs_pointer->dyn_number << endl;
        reference.replan_traj(ct_pos, ct_vel, ct_acc, waypoints_m, start_end_derivatives, flying.obs_pointer, flying.dynobs_pointer, ros::Time::now().toSec(), camera_vertex_b, if_initial, if_reach);

        if (ball_time_out)
          ball_time_out = false;

        traj_last_t = ros::Time::now();
        gap = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - last_traj_tic).count() * 1.0e-6 - timee;
        last_traj_tic = chrono::high_resolution_clock::now();
        if (if_initial)
          timee = 0; //1 / CtrlFreq / 4;
        else
          timee = gap;
        // cout<<"if reach: "<<last_if_reach<<"  "<<if_reach<<endl;
        if (if_initial)
          if_initial = false;
        if (if_debug)
          cout << "old traj is not safe, get new traj!" << endl;
      }
      singlestep_time = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - t1).count() * 1.0e-3;
      // cout << "Total time cost (ms): " << singlestep_time << endl;
      timecosts.push_back(singlestep_time);
      if (timecosts.size() > 1000)
      {
        cout << "Average Total time cost (ms): " << std::accumulate(std::begin(timecosts), std::end(timecosts), 0.0) / timecosts.size() << endl;
        timecosts.clear();
      }
      //  <<ros::Time::now().toSec()<<endl<<t1<<endl<<traj_last_t<<endl<<flying.dynobs_pointer->time_stamp<<endl;
      if (ifMove)
        reference.get_traj_samples(sp_pos, sp_vel, sp_acc, (ros::Time::now() - traj_last_t).toSec());
      else
        reference.get_traj_samples(sp_pos, sp_vel, sp_acc, 0.0);
      flying.pub_traj(sp_pos, sp_vel, sp_acc, reference.fail_pt);
      // flying.pub_fovlist (sp_pos, sp_vel, sp_acc,camera_vertex_bv, reference.yaw_plan);
      flying.pub_path(waypoints);
      flying.pub_polyh(reference.decompPolys);
    }

    else // if pcl has not been updated
    {
      //  ros::Duration(1/CtrlFreq).sleep();
      //  timee = (ros::Time::now() - traj_last_t).toSec()+ 1/CtrlFreq;
      gap = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - last_traj_tic).count() * 1.0e-6;
      timee = gap; // + 1 / CtrlFreq / 4;
      // cout<<"control sample time gap (no pcl updated): "<<gap<<endl;
      state = flying.get_state();
      //  if ( timee > reference.total_t - 0.1)
      //  {
      //   if(if_debug) cout << "reach goal, wait for new path!" << endl;
      //   ros::Duration(0.5).sleep();
      //   continue;
      //  }
    }
    Vector2d desire_psi;
    //  cout<< "distance to goal: "<<(g_goal-state.P_E).norm()<<endl;
    // if ((g_goal - state.P_E).norm() > 0.1)
    // {
    if (timee < reference.total_t - 0.01)
    {
      reference.get_desire(timee, p_d, v_d, a_d, p_d_yaw);
      // Vector2d v2 = (p_d_yaw - state.P_E).head(2);
      // Vector2d v1;
      // v1<<1.0,0.0;
      // desire_psi=acos(v1.dot(v2) /(v1.norm()*v2.norm()));

      // if (v2(1)<0)
      // {desire_psi = -desire_psi;}
      desire_psi = reference.getYaw(timee + traj_last_t.toSec());
    }
    else
    {
      reference.get_desire(reference.total_t, p_d, v_d, a_d, p_d_yaw);
      cout << "goal reached: " << g_goal << endl;
      // p_d = g_goal;
      // v_d.setZero();
      // a_d.setZero();
      if_end = true;
      // for (int counter = 0;counter<20;counter++)
      // {
      //   state = flying.step(desire_psi[0], desire_psi[1], p_d, v_d, a_d, "pos_vel_acc_yaw_c");
      //   ros::Duration(0.05).sleep();
      // }
    }

    if (ifMove)
    {
      state = flying.step(desire_psi[0], desire_psi[1], p_d, v_d, a_d, "pos_vel_acc_yaw_c");
      // reference.get_traj_samples(sp_pos, sp_vel,sp_acc, (ros::Time::now() - traj_last_t).toSec());
    }
    // else{reference.get_traj_samples(sp_pos, sp_vel,sp_acc, 0.0);}

    flying.pub_fovshape(camera_vertex);
    if (flying.dynobs_pointer->ball_number > 0)
    {
      flying.pub_ballstates();
    }

    // ros::Duration(1/CtrlFreq).sleep();
    loop_rate.sleep();
    if (if_end && !if_rand && return_home <= 1 && !rc_goal)
    {
      break;
    }
  }

  return 0;
}

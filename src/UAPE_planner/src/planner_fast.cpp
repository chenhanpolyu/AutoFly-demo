#include <planner_fast.h>

void TrajectoryGenerator_fast::read_param(const ros::NodeHandle *nh_priv)
{
  // { nh_(*nh_priv);
  // cout << "mk2" << endl;
  config.loadParameters(*nh_priv);
}
// generate the trajectory
void TrajectoryGenerator_fast::replan_traj(Vector3d &start, Vector3d &vi, Vector3d &ai, MatrixXd &waypoints, vector<Vector3d> &start_end_divs, vec_Vec3f *obs_pointer, dynobs_tmp *dynobs, double plan_t, Matrix<double, 3, 5> camera_vertex, bool full_trip, bool if_reach)
{
  dynobs_pointer = dynobs;

  //  cout << "1" << endl;

  //  cout << "waypoints:\n" <<waypoints << endl;
  //  cout << "vi, ai: \n" << vi << endl << ai <<endl;
  if (full_trip)
  {
    wPs.clear();
    if (waypoints.cols() < 3)
    {
      wPs.emplace_back(start.transpose());
      wPs.emplace_back(((start.transpose() + waypoints.col(1).transpose()) / 2));
      wPs.emplace_back(waypoints.col(1).transpose());
    }
    else
    {
      wPs.emplace_back(start.transpose());
      for (uint i = 1; i < waypoints.cols(); i++)
      {
        wPs.emplace_back(waypoints.col(i).transpose());
      }
    }
    //  cout << "received obs points:" << obs_pointer->size()<<endl << obs_pointer <<endl;

    gen_polyhedrons(obs_pointer);
    //  if_config = false;
  }
  Matrix3d iniState, finState;
  //  Vector3d final_va = Vector3d::Zero(3);
  iniState.col(0) = start;
  iniState.col(1) = vi;
  iniState.col(2) = ai;
  finState.col(0) = wPs.back();
  finState.col(1) = Vector3d::Zero(3);
  //  if (if_reach || full_trip)
  //  finState.col(1) = Vector3d::Zero(3);
  //  else {
  //   // cout<< "end vel: "<<start_end_divs[1]<<" "<<start_end_divs[1].norm()<<endl;
  //    finState.col(1) =  (finState.col(0)-iniState.col(0)).normalized()*config.velMax/4;//(waypoints.col(waypoints.cols()-1)-waypoints.col(waypoints.cols()-2)).normalized()*config.velMax/4;
  //   //  cout<< "end vel: "<<finState.col(1)<<endl;
  //    }//start_end_divs[1].normalized()*config.velMax; }//the end velocity of the kinoA* path //(finState.col(0)-iniState.col(0)).normalized()*config.velMax;
  finState.col(2) = Vector3d::Zero(3);
  // check_wps_in_polyH();
  Traj_opt(iniState, finState, plan_t);
  if (config.yawplan && dynobs_pointer->dyn_number > 0)
  {
    yaw_plan_tm = ros::Time::now().toSec();
  try
  {
    Yaw_plan(yaw_plan_tm);
  }
  catch(const std::exception& e)
  {
    std::cerr << "caught error: "<<e.what() << '\n';
    yaw_plan.clear();
  }
  
    yaw_timeout = false;
  }
}

void TrajectoryGenerator_fast::Yaw_plan(double plan_t)
{
  // total_t = traj.getTotalDuration();
  Matrix<double, 3, 5> camera_vertex;
  Vector3d sp_pos, sp_acc, sp_vel;
  Matrix3d Rota;
  Vector3d ct_center;
  double thrust, t_base, score;
  Vector2d v1, v2;
  int rows = (min(total_t, plan_t - plan_tm + total_t / 2) - (plan_t - plan_tm)) / delta_t_yaw + 1;
  int cols = config.max_yaw_range / 0.175 + 1;
  double v_psi;
  double M_yaw[rows][cols], M_vis_score[rows][cols], M_score[rows][cols];

  // vector<vector<Matrix<double, 3, 5>>> M_camera_vertex;
  // vector<Matrix<double, 3, 5>> fov_plan;
  int M_parent[rows][cols];
  double sp_yaw1, sp_theta, sp_phi;
  yaw_plan.clear();
  // Matrix<double, rows, cols> M_yaw, M_yaw1, M_vis_score, M_score;
  // Matrix<int, rows, cols> M_parent;
  v1 << 1.0, 0.0;
  int row = 0, col = 0;
  cout << "yaw plan begin" << endl;

  for (double ti = plan_t - plan_tm; ti < min(total_t, plan_t - plan_tm + total_t / 2); ti += delta_t_yaw)
  {

    t_base = plan_tm - dynobs_pointer->time_stamp + ti;
    // yaw_plan_t.push_back(ti);
    sp_pos = traj.getPos(ti);
    sp_acc = traj.getAcc(ti);
    sp_vel = traj.getPos(total_t) - traj.getPos(ti); // traj.getVel(ti+0.1); //
    sp_acc(2) += G;
    thrust = sp_acc.norm();
    v2 = sp_vel.head(2);
    v_psi = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
    if (v2(1) < 0)
    {
      v_psi = -v_psi;
    }
    if (dynobs_pointer->dyn_number == 0)
    {
      yaw_plan.push_back(v_psi);
      // if (config.if_debug) std::cout<<"row,col:"<<row<<" "<<col<<" "<<rows<<" "<<cols<<std::endl;
      if (row == rows - 1)
      {
        // std::cout<<"No dynamic obs, use velocity yaw "<<std::endl;
        return;
      }
      row++;
      continue;
    }
    for (double sp_yaw = v_psi - config.max_yaw_range / 2; sp_yaw < v_psi + config.max_yaw_range / 2; sp_yaw += 0.175)
    {
      if (row == 0)
      {
        M_vis_score[row][col] = 1;
        M_yaw[row][col] = v_psi; //
        M_score[row][col] = 3;
        continue;
      }

      if (sp_yaw > M_PI)
        sp_yaw1 = sp_yaw - 2 * M_PI;
      else if (sp_yaw < -M_PI)
        sp_yaw1 = sp_yaw + 2 * M_PI;
      else
        sp_yaw1 = sp_yaw;
      sp_theta = atan((sp_acc(0) + sp_acc(1) * tan(sp_yaw1)) / (sp_acc(2) * (cos(sp_yaw1) + sin(sp_yaw1) * tan(sp_yaw1))));
      sp_phi = acos(sp_acc(2) / thrust / cos(sp_theta));
      // AngleAxisd rollAngle(AngleAxisd(sp_phi, Vector3d::UnitX()));
      // AngleAxisd pitchAngle(AngleAxisd(sp_theta, Vector3d::UnitY()));
      // AngleAxisd yawAngle(AngleAxisd(sp_yaw1, Vector3d::UnitZ()));
      // Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
      // Rota = Quaternion2Rota(quaternion.normalized());
      Rota = Quaternion2Rota(Euler2Quaternion(Vector3d(sp_phi, sp_theta, sp_yaw1)));
      camera_vertex = (Rota * camera_vertex_b).array().colwise() + sp_pos.array();
      double score_vis = 0;
      bool traj_vel_vis = inFOV(camera_vertex, (traj.getVel(ti).normalized() * 0.5 + sp_pos));
      //   score_traj_vel_vis = 0.0;
      // else
      //   score_traj_vel_vis = -5.0;
      // std::cout<<"Got the sample Rota: "<<Rota<<"row,col:"<<row<<" "<<col<<" "<<rows<<" "<<cols<<std::endl;
      for (int i = 0; i < dynobs_pointer->dyn_number; i++)
      {
        ct_center = dynobs_pointer->centers[i] + t_base * dynobs_pointer->vels[i];
        if (inFOV(camera_vertex, ct_center) && traj_vel_vis)
        {
          score_vis += 2 * max(0.0, double(5 - (ct_center - sp_pos).norm()));
        }
      }
      // score_vis += score_traj_vel_vis;
      M_vis_score[row][col] = score_vis;
      M_yaw[row][col] = sp_yaw1; //
      // M_camera_vertex[row][col] = camera_vertex;
      col += 1;
    }
    col = 0;
    row += 1;
  }
  // cout << "yaw plan begin" << endl;
  // std::cout<<"Got the vis score matrix: "<<M_vis_score<<std::endl;
  double max_total_score = 0;
  int choosed_col;
  for (row = 1; row < rows; row++)
  {
    for (col = 0; col < cols; col++)
    {
      int parent=0;
      double tmp_score;
      score = 0;

      for (int col0 = 0; col0 < cols; col0++)
      {

        double yaw_gap_raw = abs(M_yaw[row][col] - M_yaw[row - 1][col0]);
        double yaw_gap = (yaw_gap_raw > M_PI) ? (2 * M_PI - yaw_gap_raw) : yaw_gap_raw;
        if (config.yaw_gap_max - yaw_gap < 0 && row > 1)
          continue;

        if ((M_vis_score[row][col]) == 0)
        {
          yaw_gap_raw = abs(M_yaw[row][col] - v_psi);
          yaw_gap = (yaw_gap_raw > M_PI) ? (2 * M_PI - yaw_gap_raw) : yaw_gap_raw;
        }
        // cout << "yaw gap:" << yaw_gap << " "<<yaw_gap_raw<< endl;

        tmp_score = M_vis_score[row][col] + M_score[row - 1][col0] + (config.yaw_gap_max - yaw_gap) * config.yaw_w;
        if (tmp_score > score)
        {
          parent = col0;
          score = tmp_score;
        }
        if (row == 1)
          break;
      }
      // cout << "parent:" <<parent<< endl;
      M_parent[row][col] = parent;
      M_score[row][col] = score;

      if (row == rows - 1 && score > max_total_score)
      {
        max_total_score = score;
        choosed_col = col;
      }
    }
  }
  for (row = rows - 1; row >= 0; row--)
  {
    yaw_plan.push_back(M_yaw[row][choosed_col]);
    choosed_col = M_parent[row][choosed_col];
    if (config.if_debug)
      std::cout << "yaw planned: " << yaw_plan.back() << " row:" << row << " choosed_col: " << choosed_col << " rows: " << rows << std::endl;
  }
  std::reverse(std::begin(yaw_plan), std::end(yaw_plan));
  cout << "yaw plan finish" << endl;
}

inline bool TrajectoryGenerator_fast::inFOV(Matrix<double, 3, 5> camera_vertex, Vector3d ct_center)
{
  return !((ct_center.head(3) - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(1)).cross(camera_vertex.col(0) - camera_vertex.col(2))) < 0 || (ct_center.head(3) - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(2)).cross(camera_vertex.col(0) - camera_vertex.col(3))) < 0 || (ct_center.head(3) - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(3)).cross(camera_vertex.col(0) - camera_vertex.col(4))) < 0 || (ct_center.head(3) - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(4)).cross(camera_vertex.col(0) - camera_vertex.col(1))) < 0 || (ct_center.head(3) - camera_vertex.col(1)).dot((camera_vertex.col(1) - camera_vertex.col(4)).cross(camera_vertex.col(1) - camera_vertex.col(2))) < 0);
}
Vector2d TrajectoryGenerator_fast::getYaw(double t)
{

  uint indx = (t - yaw_plan_tm) / delta_t_yaw;
  Vector2d desire_yaw; // yaw,yaw_rate
  double yaw_gap;
  Vector2d v1, v2;
  v1 << 1.0, 0.0;
  //  v2 = traj.getVel(min(t+1,total_t)).head(2);

  v2 = (traj.getPos(total_t) - traj.getPos(min((t - plan_tm), total_t - 0.2))).head(2);
  // std::cout<<"v2: "<<v2<<std::endl;
  double v_psi = acos(v1.dot(v2) / (v2.norm()));
  if (v2(1) < 0)
  {
    v_psi = -v_psi;
  }
  // std::cout<<"Set yaw begin "<< indx<<" "<<yaw_plan.size()<<"\n"<<((yaw_plan.size()>0)?yaw_plan.back():0)<<" "<<dynobs_pointer->dyn_number<< " "<<(yaw_plan.size() - 1)<<std::endl;
  if ((indx + 2) > yaw_plan.size() || !config.yawplan || dynobs_pointer->dyn_number == 0)
  {
    yaw_timeout = true;
    desire_yaw(0) = v_psi;
    desire_yaw(1) = 0;
  }
  else
  {
    try {

    yaw_gap = (yaw_plan[indx + 1] - yaw_plan[indx]);
    if (abs(yaw_gap) > M_PI)
    {
      yaw_gap = copysign((2 * M_PI - abs(yaw_gap)), desire_yaw(0));
    }
    desire_yaw(0) = yaw_plan[indx] + ((t - yaw_plan_tm) - indx * delta_t_yaw) / delta_t_yaw * yaw_gap;
    desire_yaw(1) = 0;
    if (abs(desire_yaw(0)) > M_PI)
      cout << "yaw out range! " << desire_yaw(0) << endl;
    std::cout << "Set yaw-2:\n"
              << desire_yaw(0) << " " << yaw_plan[indx + 1] << " " << yaw_plan[indx] << std::endl;
    } 
    catch (...) { // exception should be caught by reference
        cout << "get yaw exception " << "\n";
    yaw_timeout = true;
    desire_yaw(0) = v_psi;
    desire_yaw(1) = 0;
    }
    
  }
return desire_yaw;
}

void TrajectoryGenerator_fast::check_wps_in_polyH(void)
{

  Vector3d pos;
  Polyhedron3D poly;
  int i = 0;
  for (uint j = 0; j < 2; j++) // check if neccessary to update corridor
  {
    // cout<<"(in func) check traj safe:\n"<<j<<endl<<total_t<<endl;
    pos = wPs[i];
    if (i == 0)
    {
      poly = decompPolys[0];
    }
    else
    {
      poly = decompPolys.back();
    }
    Vec3f pt;
    pt[0] = pos(0);
    pt[1] = pos(1);
    pt[2] = pos(2);

    bool if_inside = poly.inside(pt);
    if (!if_inside)
    {
      cout << "The " << i << " th waypoint is not in SFC" << endl;
    }
    else
    {
      cout << "The " << i << " th waypoint is in SFC" << endl;
    }
    i = wPs.size() - 1;
  }
}

bool TrajectoryGenerator_fast::check_polyH_safe(const double plan_t, const MatrixXd &waypoints, Vector3d &start, vec_Vec3f *obs_pointer, dynobs_tmp *dynobs, double start_t, bool path_replan, bool pcl_update)
{
  //  if_config =false;
  dynobs_pointer = dynobs;
  plan_tm = plan_t;
  double start_t1 = start_t - plan_t;
  //  check_sfc_ind = 0;
  Vector3d pos;
  double dt = 0.1;
  Polyhedron3D poly;
  pt[0] = start(0);
  pt[1] = start(1);
  pt[2] = start(2);
  // total_t = traj.getTotalDuration();

  wPs.clear();
  if (waypoints.cols() < 3)
  {
    wPs.emplace_back(start.transpose());
    wPs.emplace_back(((start.transpose() + waypoints.col(1).transpose()) / 2));
    wPs.emplace_back(waypoints.col(1).transpose());
  }
  else
  {
    wPs.emplace_back(start.transpose());
    for (uint i = 1; i < waypoints.cols(); i++)
    {
      wPs.emplace_back(waypoints.col(i).transpose());
    }
  }

  // if (old_plhd_safe)
  // { cout<<"old corridor is safe for new waypoints!"<<endl;
  //   // return true;
  //   }
  //   else{
  //   cout<<"old corridor is not safe for new waypoints!"<<endl;
  //   }
  bool in_first_polyh = decompPolys.front().inside(pt, 2 * config.safeMargin);
  if (pcl_update || path_replan || !in_first_polyh || (last_check_pos - start).norm() > config.sfck_td || (ros::Time::now() - last_check_time).toSec() > config.sfck_td)
  {
    gen_polyhedrons(obs_pointer);
    last_check_pos = start;
    last_check_time = ros::Time::now();
    last_traj_polyH_check = true;
  }
  // if_config = true;

  if (config.yawplan && dynobs_pointer->dyn_number > 0 && yaw_timeout)
  {
    yaw_plan_tm = ros::Time::now().toSec();
    Yaw_plan(yaw_plan_tm);
    yaw_timeout = false;
  }
  if (path_replan && (traj.getPos(total_t) - wPs.back()).norm() > config.horizon * 0.3)
  {
    cout << "Path replanned, so traj replan. Traj end and goal distance:" << (traj.getPos(total_t) - wPs.back()).norm() << endl;
    return false;
  }
  // if ((path_replan && start_t1 > 0.15*total_t) || start_t1 > 0.4*total_t)
  // return false;
  // if (!in_first_polyh || (traj.getPos(total_t)-wPs.back()).norm() > config.horizon*0.5)
  //   return false;
  if (start_t1 + dt > total_t)
  {
    cout << "Safety check timestamp is close to traj end!" << endl;
    return true;
  }
  //  cout<< "mk1"<<endl;
  //  check_sfc_ind = 0;
  //  poly = decompPolys[check_sfc_ind];

  //  cout<< "mk2"<<endl;
  for (double j = start_t1 + dt; j < total_t; j += dt)
  {
    // cout<<"(in func) check traj safe:\n"<<j<<endl<<total_t<<endl;
    pos = traj.getPos(j);
    if (j < total_t*0.8 && !dyn_safe_check(pos, j + start_t - start_t1))
    {
      cout << "dyn check fail!  " << dynobs_pointer->dyn_number << endl; // dynobs_pointer->ballvel[0]<<endl
      return false;
    }
    // if (old_plhd_safe)
    // {continue;}
    // Vec3f pt;
    pt[0] = pos(0);
    pt[1] = pos(1);
    pt[2] = pos(2);
    if (j == start_t1 + dt)
    {
      for (uint kk = 0; kk < decompPolys.size(); kk++)
      {
        check_sfc_ind = kk;
        poly = decompPolys[check_sfc_ind];
        if (poly.inside(pt, config.safeMargin))
          break;
      }
    }
    while (j > start_t1 + dt && !poly.inside(pt, 0.5*config.safeMargin) && (last_traj_polyH_check || !poly.inside(pt, 0)))
    {
      check_sfc_ind += 1;
      if (check_sfc_ind > decompPolys.size() - 1)
      {
        cout << "old traj SFC check fail! Traj time:" << j << "\ncheck pos:\n"
             << pt << "\nindex:" << check_sfc_ind << "\nlast_traj_polyH_check:"<<last_traj_polyH_check<<"\n inside poly: "<<poly.inside(pt, 0)<<endl;
        last_traj_polyH_check = false;
        fail_pt = pt;
        return false;
      }
      poly = decompPolys[check_sfc_ind];
      // if(!poly.inside(pt)){
      // cout<< "old traj SFC check fail!--2: "<<check_sfc_ind<<"  "<<hPolys.size()<<endl;
      // return false;}
    }
  }
  
  // cout << "old traj is safe for dyn obs, and all in new corridor! " << dynobs_pointer->dyn_number << " " << ros::Time::now().toSec() - dynobs_pointer->time_stamp << endl;
  return true;
}

bool TrajectoryGenerator_fast::last_jointPolyH_check(Vector3d ct_pos)
{
  //  cout << "wPs size: " << wPs.size()<<endl;
  double dis2goal = (wPs.back() - ct_pos).norm();
  if ((decompPolys.back().inside(ct_pos, 0) && dis2goal < config.horizon * 0.7) || dis2goal < 0.4 * config.horizon) // if the initial   && !decompPolys[decompPolys.size()-2].inside(wPs[0])
  {
    // cout << "pos in the last polyH: " << decompPolys.back().inside(ct_pos, 0) << "  " << dis2goal << endl;
    return true;
  }
  else
    return false;
}
inline bool TrajectoryGenerator_fast::dyn_safe_check(Vector3d pt, double check_t)
// { return true; // for rosbag tests!!!
{
  double t_base;
  Vector3d ct_center;
  Vector3d conv_vec;
  double obj_prop_conv;
  if (dynobs_pointer->dyn_number > 0)
  {
    t_base = check_t - dynobs_pointer->time_stamp;

    for (int j = 0; j < dynobs_pointer->dyn_number; j++)
    {
      ct_center = dynobs_pointer->centers[j] + t_base * dynobs_pointer->vels[j];
      obj_prop_conv = pow(dynobs_pointer->max_accs[j](1) + dynobs_pointer->max_accs[j](2) * t_base * t_base, 0.5);
      obj_prop_conv = obj_prop_conv > 0.15 ? 0.15 : obj_prop_conv;
      conv_vec = {obj_prop_conv, obj_prop_conv, 0.0};
      // cout << "ct_center:\n"<<ct_center <<endl<<"pt: \n"<<pt<<endl<<"obs size: \n"<<dynobs_pointer->obs_sizes[j]*0.5<<"\ntime gap: "<<t_base<<"\n pos gap:"<<(ct_center - pt).cwiseAbs()<<endl<<(((ct_center - pt).cwiseAbs() - dynobs_pointer->obs_sizes[j]*0.5).array()<0)<<endl;
      if ((((ct_center - pt).cwiseAbs() - dynobs_pointer->obs_sizes[j] * 0.5 - conv_vec).array() < config.safeMargin).all())
      {
        // cout<<"1111"<<endl;
        return false;
      }
    }
  }
  if (dynobs_pointer->ball_number > 0)
  {
    t_base = check_t - dynobs_pointer->ball_time_stamp;
    for (int j = 0; j < dynobs_pointer->ball_number; j++)
    {
      ct_center = dynobs_pointer->ballpos[j] + t_base * dynobs_pointer->ballvel[j] + 0.5 * t_base * t_base * dynobs_pointer->ballacc[j];
      if ((((ct_center - pt).cwiseAbs() - dynobs_pointer->ball_sizes[j] * 0.5).array() < config.safeMargin).all())
      {
        return false;
      }
    }
  }
  return true;
}
void TrajectoryGenerator_fast::Traj_opt(const MatrixXd &iniState, const MatrixXd &finState, double plan_t)

{
  chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
  // Trajectory traj;
  //  cout << "received dynamic obs number:" << dynobs_pointer->dyn_number << endl << dynobs_pointer<<endl<<"dyn_timestamp:"<<dynobs_pointer->time_stamp<<endl<<plan_t<<endl;
  ROS_INFO("Begin to optimize the traj~");
  // double vel_max;
  // cout << "final state in use:" << endl << finState.col(0) << endl;
  // if (dynobs_pointer->dyn_number >0 && config.velMax<2.0)
  // {
  //   vel_max
  // }
  if (!nonlinOpt.setup(config.rho, config.totalT, iniState, finState, hPolys, INFINITY,
                       config.qdIntervals, config.horizHalfLen, config.vertHalfLen,
                       config.safeMargin, (dynobs_pointer->dyn_number > 0 && config.velMax < 2.0) ? 2.0 : config.velMax, config.thrustAccMin, config.thrustAccMax,
                       config.bodyRateMax, config.gravAcc, config.penaltyPVTB, config.useC2Diffeo, plan_t, dynobs_pointer))
  {
    ROS_INFO("gcopter initialize fail!");
    return;
  }
  double finalObj = nonlinOpt.optimize(traj, config.optRelTol);

  chrono::high_resolution_clock::time_point toc = chrono::high_resolution_clock::now();
  double compTime = chrono::duration_cast<chrono::microseconds>(toc - tic).count() * 1.0e-3;

  printf("finished!!!\n");
  cout << "Optimization time usage: " << compTime << " ms" << endl;
  cout << "Final jerk cost: " << finalObj << endl;
  cout << "Maximum Vel: " << traj.getMaxVelRate() << endl;
  cout << "Maximum Acc: " << traj.getMaxAccRate() << endl;
  cout << "Total traj Duration: " << traj.getTotalDuration() << endl;
  total_t = traj.getTotalDuration();
  plan_tm = ros::Time::now().toSec();
}

void TrajectoryGenerator_fast::gen_polyhedrons(vec_Vec3f *obs_pointer)
{
  chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
  // cout << "gen polyhedrons, wPs: " <<wPs.size()<< endl;
  EllipsoidDecomp3D decomp_util(config.global_min, config.global_size);
  // cout << "mk00:" <<obs_pointer->size()<<endl;
  decomp_util.set_obs(*obs_pointer);
  // cout << "mk0" <<endl;
  decomp_util.set_local_bbox(Eigen::Vector3d(config.polyhedronBox(0),
                                             config.polyhedronBox(1),
                                             config.polyhedronBox(2)));
  decompPolys.clear();
  // vec_E<Ellipsoid3D> ellips;
  //  cout << "mk1" <<endl;
  for (uint i = 0; i < wPs.size() - 1;)
  {
    // find the farest unblocked point
    vec_Vec3f line;
    line.push_back(wPs[i]);
    line.push_back(wPs[i + 1]);
    decomp_util.dilate(line);
    Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
    // Ellipsoid3D ellip = decomp_util.get_ellipsoids()[0];
    decompPolys.push_back(poly);
    if (i >= wPs.size() - 2)
    {
      break;
    }
    // ellips.push_back(ellip);
    // find the nearest one to the boundry of poly.
    // cout << "mk22 " << i<<endl<<wPs.size()<<endl;
    if (i + 2 < wPs.size())
    {
      uint j;
      for (j = i + 2; j < wPs.size(); j++)
      {
        Vec3f pt;
        pt[0] = wPs[j](0);
        pt[1] = wPs[j](1);
        pt[2] = wPs[j](2);
        if (!poly.inside(pt, config.safeMargin))
        {
          // if (j == wPs.size() - 1)
          j--;
          break;
        }
      }
      // j--;
      if (j >= wPs.size() - 1)
      {
        if (decompPolys.size() == 1)
        {
          j = wPs.size() - 2;
        }
        else
        {
          break;
        }
        break;
      }
      i = j;

      // int wp;
      // wp = round((1*i+4*j)/5);
      // i = wp;
    }
    // else{
    //   // i+=1;
    // i = wPs.size()-2;}
  }
  if (config.if_debug)
    cout << "Number of polyhedrons: " << decompPolys.size() << endl;
  if (decompPolys.size() < 2)
  {
    vec_Vec3f line;
    Vector3d midpt = (wPs[0] + wPs.back()) / 2;
    line.push_back(midpt);
    line.push_back(wPs.back());
    decomp_util.dilate(line);
    Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
    // Ellipsoid3D ellip = decomp_util.get_ellipsoids()[0];
    decompPolys.push_back(poly);
    // line[0] = wPs.back();
    // decomp_util.dilate(line);
    // Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
    // //Ellipsoid3D ellip = decomp_util.get_ellipsoids()[0];
    // decompPolys.push_back(poly);
  }
  // for (size_t i = 0; i < decompPolys.size(); i++)
  // {
  //     decompPolys[i].add(Plane3D(Eigen::Vector3d(0.0, 0.0, config.mapHeight),
  //                                       Eigen::Vector3d(0.0, 0.0, 1.0)));
  //     decompPolys[i].add(Plane3D(Eigen::Vector3d(0.0, 0.0, 0.0),
  //                                       Eigen::Vector3d(0.0, 0.0, -1.0)));
  // }
  // visualization.visualizePolyH(decompPolys);
  hPolys.clear();
  Eigen::MatrixXd current_poly;
  for (uint i = 0; i < decompPolys.size(); i++)
  {
    vec_E<Plane3D> current_hyperplanes = decompPolys[i].hyperplanes();
    current_poly.resize(6, current_hyperplanes.size());
    for (uint j = 0; j < current_hyperplanes.size(); j++)
    {
      current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
      // outside
    }
    hPolys.push_back(current_poly);
  }

  double compTime = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  if (config.if_debug)
    cout << "polyhedrons finished! time cost (ms)： " << compTime << endl;
}

bool TrajectoryGenerator_fast::get_new_wps(Trajectory traj, const MatrixXd &cd_c, const VectorXd &cd_r)
{
  // total_t = traj.getTotalDuration();
  if (total_t > 20)
  {
    return false;
  }
  if (wPs.size() < 3)
  {
    return true;
  }
  check_sfc_ind = 0;
  Vector3d pos;
  double dt = 0.1;
  vector<Vector3d> out_points;
  vector<Vector3d> out_centers;
  int count = 0, count_tt = 0;
  for (int j = 1; j * dt < total_t; j++)
  {

    // if (check_sfc_ind > cd_c.rows()-1)
    //   {if (count !=0)
    //    { MatrixXd Moutpts = Map<MatrixXd>(out_points[0].data(),out_points.size(),3);
    //  out_centers.emplace_back(Moutpts.colwise().mean());
    //  vector<Vector3d> out_points;
    //    }
    //   break;}
    pos = traj.getPos(j * dt);

    // cout<<"j:"<<j<<endl;
    if (total_t > dt + j * dt && safe_check(pos, cd_c, cd_r))
    {
      count += 1;
      count_tt += 1;
      out_points.emplace_back(pos);
      cout << "unsafe pos,time:" << pos << endl
           << j * dt << endl
           << total_t << endl;
    }
    else if (count != 0)
    {
      count = 0;
      // int osize = out_points.size();
      MatrixXd Moutpts = Map<MatrixXd>(out_points[0].data(), 3, out_points.size());
      out_centers.emplace_back(Moutpts.rowwise().mean());
      out_points.clear();
      cout << "found unsafe segment!" << endl
           << out_centers[out_centers.size() - 1] << endl;
      // break;
    }
    // cout << "safe_check returned!:"<< check_sfc_ind << endl;
  }
  if (count_tt == 0)
  {
    return true;
  }
  else
  {
    update_wps(out_centers, cd_c, cd_r);
    return false;
  }
}

bool TrajectoryGenerator_fast::check_traj_safe(const MatrixXd &cd_c, const VectorXd &cd_r, const double start_t)
{
  // total_t = traj.getTotalDuration();
  check_sfc_ind = 0;
  Vector3d pos;
  double dt = 0.1;
  vector<Vector3d> out_points;
  vector<Vector3d> out_centers;
  if (start_t + dt > total_t)
  {
    return false;
  }
  for (double j = start_t + dt; j < total_t; j += dt)
  {
    // cout<<"(in func) check traj safe:\n"<<j<<endl<<total_t<<endl;
    pos = traj.getPos(j);
    if (check_sfc_ind > cd_c.rows() - 1)
    {
      return false;
    }
    if (safe_check(pos, cd_c, cd_r))
    {

      return false;
      //  out_points.emplace_back(pos);
    }
  }

  return true;
}

// void TrajectoryGenerator_fast::update_wps( const vector<Vector3d> &out_centers, const MatrixXd &cd_c, const VectorXd &cd_r)
// {
//  vector<Vector3d> wPs_copy = wPs;
//  for (int k=0; k<out_centers.size(); k++)
//  {
//  //MatrixXd centers = VectorXd::Map(&waypoints[0],waypoints.size());
//  VectorXd dists = (cd_c.rowwise() - out_centers[k].transpose()).rowwise().norm();
//  VectorXd gap = dists - cd_r;
//  cout << "update_wps!" << gap << endl << wPs.size()<< endl;
//  ptrdiff_t min_ind;
//  double min_gap = gap.minCoeff(&min_ind);
//  Vector3d min_row = cd_c.row(min_ind);
//  Vector3d new_wp = min_row + (out_centers[k] - min_row).normalized()*(cd_r(min_ind)*0.5);
//
//  wPs.insert(wPs.begin()+get_insert_ind(new_wp),new_wp);
//  }
//  }

void TrajectoryGenerator_fast::update_wps(const vector<Vector3d> &out_centers, const MatrixXd &cd_c, const VectorXd &cd_r)
{
  vector<Vector3d> wPs_copy = wPs;
  Vector3d new_wp;
  double a, b, c, s;
  for (uint k = 0; k < out_centers.size(); k++)
  {
    // MatrixXd centers = VectorXd::Map(&waypoints[0],waypoints.size());
    int location = get_insert_ind_1(out_centers[k], cd_c, cd_r);
    // MatrixXd ck = out_centers[k];
    //   a= (cd_c.row(location) - ck).norm();
    //   b= (cd_c.row(location+1) - ck).norm();
    a = cd_r(location);
    b = cd_r(location + 1);
    c = (cd_c.row(location) - cd_c.row(location + 1)).norm();
    s = (a + b + c) / 2;
    cout << "s of triangle: " << s << endl
         << wPs.size() << endl
         << a << endl
         << b << endl
         << c << endl;
    if (out_centers[k](2) < 0.2)
    {
      new_wp << out_centers[k](0), out_centers[k](1), 0.3;
    }
    else if (c < a + b)
    {
      double joint_r = sqrt(s * (s - a) * (s - b) * (s - c)) * 2 / c;
      Vector3d ancor = cd_c.row(location) + (cd_c.row(location + 1) - cd_c.row(location)).normalized() * sqrt(a * a - joint_r * joint_r);
      new_wp = ancor + (out_centers[k] - ancor).normalized() * joint_r * 0.6;
    }
    else
    {
      new_wp = (cd_c.row(location) + cd_c.row(location + 1)) / 2;
    }
    cout << "going to insert!" << new_wp << endl;
    bool if_insert = true;
    for (uint i = 0; i < wPs.size(); i++)
    {
      if ((wPs[i] - new_wp).norm() < 0.3)
      {
        if_insert = false;
        break;
      }
    }

    int insert_pos = get_insert_ind(new_wp);
    if (if_insert)
    {
      wPs.insert(wPs.begin() + insert_pos, new_wp);
    }
  }
}

// inline int TrajectoryGenerator_fast::get_insert_ind(const Vector3d &check_c)
// {
//  for (int i=0; i<wPs.size()-1; i++)
//  {
//   if (((check_c-wPs[i]).norm() + (check_c-wPs[i+1]).norm())/(wPs[i]-wPs[i+1]).norm() < 1.01)
//   {return i+1;
//   }
//  }
//
// }

inline int TrajectoryGenerator_fast::get_insert_ind_1(const Vector3d &check_c, const MatrixXd &cd_c, const VectorXd &cd_r)
{
  double base;
  for (uint i = 0; i < cd_c.rows() - 1; i++)
  {
    base = (cd_c.row(i) - cd_c.row(i + 1)).squaredNorm();
    cout << "mark" << endl
         << cd_c.row(i) << endl
         << cd_c.row(i + 1) << endl
         << check_c.transpose() << endl
         << base << endl;
    if (((check_c.transpose() - cd_c.row(i)).squaredNorm() < (check_c.transpose() - cd_c.row(i + 1)).squaredNorm() + base) && ((check_c.transpose() - cd_c.row(i + 1)).squaredNorm() < (check_c.transpose() - cd_c.row(i)).squaredNorm() + base))
    {
      cout << "insert position (cd_c):" << i << endl;
      return i;
    }
  }
  return 0;
}

inline int TrajectoryGenerator_fast::get_insert_ind(const Vector3d &check_c)
{
  double base;
  for (uint i = 0; i < wPs.size() - 1; i++)
  {
    base = (wPs[i] - wPs[i + 1]).squaredNorm();
    cout << "mark1" << endl;
    if ((check_c - wPs[i]).squaredNorm() < (check_c - wPs[i + 1]).squaredNorm() + base && (check_c - wPs[i + 1]).squaredNorm() < (check_c - wPs[i]).squaredNorm() + base)
    {
      cout << "insert position:" << i << endl;
      return i + 1;
    }
  }
  return 0;
}

// inline bool TrajectoryGenerator_fast::safe_check(const Vector3d pos, const MatrixXd &cd_c, const VectorXd &cd_r)
// {
//  // cout<<"safe_check begin:"<< check_sfc_ind <<"\n pos:"<< pos <<"\n cd_c:"<<cd_c<<"\n cd_r:"<<cd_r<<endl;
//  Vector3d checkrow = cd_c.row(check_sfc_ind);
//  bool safe = ((pos - checkrow).norm() > cd_r(check_sfc_ind));
//  if (safe)  //safe = true for collision (not in SFC)
//  {
// //  cout<<"mark3"<<endl;
//   check_sfc_ind +=1;
//  if (check_sfc_ind > cd_c.rows()-1)
//   {return safe;}
//  checkrow = cd_c.row(check_sfc_ind);
//  safe = ((pos - checkrow).norm() > cd_r(check_sfc_ind));
//  //cout<<"mark2"<<endl;
//   if (safe and check_sfc_ind+1 < cd_c.rows())
//   {
// //   cout<<"mmm:\n"<<(cd_c.block(check_sfc_ind+1, 0, cd_c.rows()-check_sfc_ind-1, 3).rowwise() - pos)<<endl;
//   VectorXd gaps = (cd_c.block(check_sfc_ind+1, 0, cd_c.rows()-check_sfc_ind-1, 3).rowwise() - pos.transpose()).rowwise().norm() - cd_r.tail(cd_c.rows()-check_sfc_ind-1);
//  //  cout<<"mark1"<<endl;
//    for (int k=0;k<gaps.size();k++)
//    {
//    if (gaps(k) < 0)
//    {
//   if (k != gaps.size()-1)
//    {
//    safe = false;
//    check_sfc_ind = k+check_sfc_ind+1;}
//
//   else{check_sfc_ind -=1;}
//   break;
//   }
//    }}
//
//   }
//  // cout<<"safe_check end:"<< check_sfc_ind <<endl;
//  return safe;
// }

inline bool TrajectoryGenerator_fast::safe_check(const Vector3d pos, const MatrixXd &cd_c, const VectorXd &cd_r)
{
  double min_dis = ((cd_c.rowwise() - pos.transpose()).rowwise().norm() - cd_r).minCoeff();
  // cout << "minimal distance: " << min_dis <<endl;
  bool safe = (min_dis > 0 || pos(2) < 0.3);
  return safe;
}

void TrajectoryGenerator_fast::get_wPs(const MatrixXd &waypoints, const MatrixXd &cd_c, const VectorXd &cd_r, const Vector3d &start)
{

  // vector<Vector3d> wPs;
  vector<double> dists;
  wPs.emplace_back(start.transpose());
  if (waypoints.rows() > 2)
  {
    double base = (waypoints.row(0) - waypoints.row(waypoints.rows() - 1)).norm();
    //  cout << "mark2" << endl;
    Vector3d vec2, vec1;
    double dis;
    for (uint k = 1; k < waypoints.rows() - 1; k++)
    {
      vec2 = waypoints.row(k) - waypoints.row(0);
      vec1 = waypoints.row(k) - waypoints.row(waypoints.rows() - 1);
      dis = vec1.cross(vec2).norm() / base;
      dists.push_back(dis);
      cout << "mark3" << endl
           << k << endl;
    }
    //  cout << "mark1" << endl;
    VectorXi ind;
    VectorXd sorted_vec;
    VectorXd re = VectorXd::Map(&dists[0], dists.size());
    sort_vec(re, sorted_vec, ind);
    wPs.emplace_back(waypoints.row(ind(0) + 1));
  }

  cout << "mark2  " << waypoints.row(waypoints.rows() - 1) << endl;

  wPs.emplace_back(waypoints.row(waypoints.rows() - 1));
  // return wPs
}

void TrajectoryGenerator_fast::sort_vec(const VectorXd &vec, VectorXd &sorted_vec, VectorXi &ind)
{
  ind = VectorXi::LinSpaced(vec.size(), 0, vec.size() - 1); //[0 1 2 3 ... N-1]
  auto rule = [vec](int i, int j) -> bool
  {
    return vec(i) > vec(j);
  };
  sort(ind.data(), ind.data() + ind.size(), rule);

  sorted_vec.resize(vec.size());
  for (uint i = 0; i < vec.size(); i++)
  {
    sorted_vec(i) = vec(ind(i));
  }
}

// allocate time for waypoints
VectorXd TrajectoryGenerator_fast::allocateTime(const MatrixXd &wayPs,
                                                double vel,
                                                double acc)
{
  int N = (int)(wayPs.cols()) - 1;
  VectorXd durations(N);
  if (N > 0)
  {
    Eigen::Vector3d p0, p1;
    double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;

    for (int k = 0; k < N; k++)
    {
      p0 = wayPs.col(k);
      p1 = wayPs.col(k + 1);
      D = (p1 - p0).norm(); // distance

      acct = vel / acc;               // accelerate time
      accd = (acc * acct * acct / 2); // accelerate distance
      dcct = vel / acc;               // de-accelerate time
      dccd = acc * dcct * dcct / 2;   // de-accelerate distance

      if (D < accd + dccd)
      {
        t1 = sqrt(acc * D) / acc;
        t2 = (acc * t1) / acc;
        dtxyz = t1 + t2;
      }
      else
      {
        t1 = acct;
        t2 = (D - accd - dccd) / vel;
        t3 = dcct;
        dtxyz = t1 + t2 + t3;
      }

      durations(k) = dtxyz;
    }
  }

  return durations;
}

// double TrajectoryGenerator_fast::generate()
// {
//     // generate&get trajectory
//     jerkOpt.generate(waypoints.block(0, 1, 3, waypGen.N - 2), ts);
//     jerkOpt.getTraj(minJerkTraj);
//
//     // check maximum velocity
//     while ((!minJerkTraj.checkMaxVelRate(MaxVel)) || (!minJerkTraj.checkMaxAccRate(5.0)))
//     {
//         printf("maximum velocity %.2f\n", minJerkTraj.getMaxVelRate());
//
//         // re-allocate time
//         MaxVelCal = MaxVelCal - 0.2;
//         ts = allocateTime(waypoints, MaxVelCal, 5.0);
//
//         // generate&get trajectory
//         jerkOpt.generate(waypoints.block(0, 1, 3, waypGen.N - 2), ts);
//         jerkOpt.getTraj(minJerkTraj);
//     }
//     printf("maximum velocity %.2f\n", minJerkTraj.getMaxVelRate());
//     int maxRow;
//
//     do
//     {   double max_time = ts.maxCoeff(&maxRow);
//          ts (maxRow) = max_time-0.2;
//         //  VectorXd grad_t = jerkOpt.getGradT();
//         // grad_t.head(1) << 0;
//         // grad_t.tail(1) << 0;
//         // ts = ts - grad_t*0.02;
//  cout<<"time locations:   "<<ts<<"\n grad_t: \n"<<endl; //grad_t<<endl;
//         // generate&get trajectory
//         jerkOpt.generate(waypoints.block(0, 1, 3, waypGen.N - 2), ts);
//         jerkOpt.getTraj(minJerkTraj);
//         printf("maximum velocity222:  %.2f\n", minJerkTraj.getMaxVelRate());
//     }
//     while  ((minJerkTraj.checkMaxVelRate(MaxVel)) && (minJerkTraj.checkMaxAccRate(5.0)));
//
//     return minJerkTraj.getTotalDuration();
// }

void TrajectoryGenerator_fast::get_desire(double timee, Vector3d &p_d, Vector3d &v_d, Vector3d &a_d, Vector3d &p_d_yaw)
{
  p_d = traj.getPos(timee);
  v_d = traj.getVel(timee);
  a_d = traj.getAcc(timee);
  p_d_yaw = traj.getPos(clip(timee + 5.0, 0.0, total_t - 0.01));
  // p_d_yaw = traj.getPos(total_t-0.1);
  // cout<<"pd,vd,ad,p_d_yaw: \n"<<p_d<<"\n"<<v_d<<"\n"<<a_d<<"\n"<<p_d_yaw<<endl;
}

void TrajectoryGenerator_fast::get_traj_samples(MatrixXd &sp_pos, MatrixXd &sp_vel, MatrixXd &sp_acc, double start_t)
{
  // total_t = traj.getTotalDuration();
  // cout << "pub traj:" << total_t <<endl;
  start_t = min(start_t, total_t);
  double delta_t = 0.3;
  int num = (total_t - start_t) / delta_t;
  num = max(num, 1);
  sp_pos.resize(num + 1, 3);
  sp_vel.resize(num + 1, 3);
  sp_acc.resize(num + 1, 3);
  for (int i = 0; i < num; i++)
  {
    sp_pos.row(i) = traj.getPos(start_t + i * delta_t);
    sp_vel.row(i) = traj.getVel(min(start_t + i * delta_t + 0.1, total_t));
    sp_acc.row(i) = traj.getAcc(start_t + i * delta_t);
  }
  sp_pos.row(num) = traj.getPos(total_t);
  sp_vel.row(num) = traj.getVel(total_t);
  sp_acc.row(num) = traj.getAcc(total_t);
}

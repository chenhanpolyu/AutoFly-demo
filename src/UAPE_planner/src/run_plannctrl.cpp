#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
#include <Tools.h>
#include <ros/ros.h>

#include <ros_missn.h>
//#include <controllers/backstepping.h>
#include <planner_fast.h>
#include <Tools.h>
//#include <logs/log_flight.h>

using namespace std;
using namespace Eigen;

#define CtrlFreq 40
#define MaxVel 2.0

// struct Config
// {


//     std::string odomFrame;

//     // Params
//     double scaleSI;
//     double mapHeight;
//     Eigen::Vector3d polyhedronBox;
//     double rho;
//     double totalT;
//     int qdIntervals;
//     double horizHalfLen;
//     double vertHalfLen;
//     double safeMargin;
//     double velMax;
//     double thrustAccMin;
//     double thrustAccMax;
//     double bodyRateMax;
//     double gravAcc;
//     Eigen::Vector4d penaltyPVTB;
//     bool useC2Diffeo;
//     double optRelTol;
//     double trajVizWidth;
//     Eigen::Vector3d trajVizRGB;
//     std::string routeStoragePath;
//     std::string ellipsoidPath;
//     Eigen::Vector4d ellipsoidVizRGBA;} config;
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv,"traj_planner");   
            // Create a handle to this process node.
    ros::NodeHandle nh("~");
    RosClass flying(&nh, CtrlFreq);
    Vector3d ct_pos,ct_vel,ct_acc;
    ct_pos.setZero();
    Vector3d p_d, v_d, a_d, p_d_yaw;
    MatrixXd sp_pos, sp_vel, sp_acc;
    TrajectoryGenerator_fast reference;
    // cout << "mk1" << endl;
        
    reference.read_param(&nh);
    // ct_pos<<0,0,4;
    // Intialize Planner
    // cout << "ctpos:\n"<<ct_pos<<endl;
    States state;
    cout << "Traj node initialized!" << endl;
    do{state = flying.get_state();
       ct_pos = state.P_E;
            ros::Duration(0.1).sleep();}
    while (ct_pos.norm() < 1e-3);
    cout << "UAV message received!" << ct_pos << endl;
    while (flying.waypoints.rows() < 2)
    { ros::Duration(0.05).sleep();
      state = flying.get_state();
    }
    cout << "Waypoints received!\n" << flying.waypoints << endl;
    state = flying.get_state();
    ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;
    reference.replan_traj(MaxVel,ct_pos,ct_vel,ct_acc,flying.waypoints,flying.cd_c,flying.cd_r,flying.obs_pointer, flying.dynobs_pointer,ros::Time::now().toSec());
    double timee, traj_last_t;
    traj_last_t =ros::Time::now().toSec();
    
    while (nh.ok())
    {
    double t1 = ros::Time::now().toSec();
    if (!flying.pcl_update) // || !flying.waypoint_update)
    {
     ros::Duration(1/CtrlFreq).sleep();
     timee = ros::Time::now().toSec() - traj_last_t + 1/CtrlFreq;
     state = flying.get_state();
     if (reference.total_t < timee+0.2)
     {cout << "reach goal, wait for new path!" << endl;
      
      ros::Duration(0.5).sleep();
      continue;
     }
       
       // cout << "(corridor not update)" << endl;
    }
    else{
    flying.set_cod_update(false);
    flying.set_pcl_update(false);
    state = flying.get_state();
    ct_pos = state.P_E;
    ct_vel = state.V_E;
    ct_acc = state.A_E;
    // cout << "the updated waypoints:" << flying.waypoints << endl;
    bool if_safe = reference.check_polyH_safe(traj_last_t, flying.waypoints,ct_pos, flying.obs_pointer, flying.dynobs_pointer,ros::Time::now().toSec());
    
    // cout << "corridor update, check safety result:" << if_safe <<endl;
    cout << "point cloud update, check safety result:" << if_safe <<endl;
    if (if_safe && (timee < 0.8))
    {
    //TrajectoryGenerator_fast reference(MaxVel,ct_pos,ct_vel,ct_acc,flying.waypoints,flying.cd_c,flying.cd_r);
   // double FlyTime = reference.get_duration();
//     Vector2d v2 = (reference.waypoints.col(1).head(2) - reference.waypoints.col(0).head(2));
//     Vector2d v1;
//     v1<<1.0,0.0;
//     double desire_psi=acos(v1.dot(v2) /(v1.norm()*v2.norm())); 
//     
//     if (v2(1)<0)
//     {desire_psi = -desire_psi;}
 
     timee = ros::Time::now().toSec() - traj_last_t + 1/CtrlFreq;
        
        // cout<<"44"<<endl;
        // calculate control inputs
        
        }
    
    else{
    reference.replan_traj(MaxVel,ct_pos,ct_vel,ct_acc,flying.waypoints,flying.cd_c,flying.cd_r,flying.obs_pointer, flying.dynobs_pointer,ros::Time::now().toSec());
    traj_last_t =ros::Time::now().toSec();
    timee = 1/CtrlFreq;
    cout << "old traj is not safe, get new traj!" << endl;
    
    }
     cout << "traj time cost: " << ros::Time::now().toSec()-t1 <<endl<<ros::Time::now().toSec()<<endl<<t1<<endl<<traj_last_t<<endl<<flying.dynobs_pointer->time_stamp<<endl;  
       }
      // cout << "timee:"<<timee<<endl;
        reference.get_desire(timee, p_d, v_d, a_d,p_d_yaw);
    //    bsc.controller(state, p_d, v_d, a_d,p_d_yaw,next_goal);
        // cout<<"55"<<endl;
        // step forward
        Vector2d v2 = (p_d_yaw - state.P_E).head(2);
        Vector2d v1;
        v1<<1.0,0.0;
        double desire_psi=acos(v1.dot(v2) /(v1.norm()*v2.norm())); 
        
        if (v2(1)<0)
        {desire_psi = -desire_psi;}
        desire_psi = state.Euler(2) + clip(desire_psi - state.Euler(2),-0.4,0.4);
        state = flying.step(desire_psi, p_d, v_d, a_d, "pos_vel_acc_yaw"); 
      //  cout << "state setpoint:" << desire_psi <<"\n"<< p_d<<"\n"<<v_d<<"\n"<< a_d << endl;
        // state = flying.step(0.0, bsc.Vc, "yaw_n_velocity"); 
        // cout<<"flying.step: \n"<<endl;
        // log
        // logger.desires(p_d, v_d, a_d);
        // logger.states(state);
        // logger.actors(bsc.forceCtrl, bsc.eulerCtrl);

        // break if crashed
//         if (flying.done)
//         {
//             break;
//         }
       // cout << "pub traj (out):" << reference.total_t <<endl;
        reference.get_traj_samples(sp_pos, sp_vel,sp_acc, ros::Time::now().toSec() - traj_last_t);
        flying.pub_traj (sp_pos, sp_vel,sp_acc);
        flying.pub_polyh (reference.decompPolys);
    
    ros::Duration(1/CtrlFreq).sleep();
    
    }

  
    return 0;}



#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <sensor_msgs/Imu.h>
// Useful customized headers
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

// joy stick state, zxzxzxzx
    enum joyState_t{START_PUB_TRAJ = 0, DONT_PUB_TRAJ}  joyState = START_PUB_TRAJ;

// Param from launch file
    double _vis_traj_width;
    double _Vel, _Acc;
    int    _dev_order, _min_order;

// ros related
    ros::Subscriber _way_pts_sub, _odom_pts_sub, _joy_pts_sub,_imu_sub;
    ros::Publisher  _wp_traj_vis_pub, _wp_path_vis_pub;
    ros::Publisher  _vis_pos_pub, _vis_vel_pub, _vis_acc_pub;
    ros::Publisher  _traj_pub;

// **** global variable *** //
// for planning
    int _poly_num1D;
    MatrixXd _polyCoeff;
    VectorXd _polyTime;
    Vector3d _startPos(0,0,0.5);
    Vector3d _startVel(0,0,0);
    Vector3d A_E,A_B;
    Quaterniond Quat;
    Matrix3d Rota;
    ros::Time _traj_time_start, _traj_time_final;
    bool _has_odom = false;
// for visualization
    visualization_msgs::Marker _vis_pos, _vis_vel, _vis_acc;

// declare
    void visWayPointTraj( MatrixXd polyCoeff, VectorXd time);
    void visWayPointPath(MatrixXd path);

    Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t );
    Vector3d getVelPoly( MatrixXd polyCoeff, int k, double t );
    Vector3d getAccPoly( MatrixXd polyCoeff, int k, double t );
    Matrix3d Quaternion2Rota(Quaterniond q);
    void trajGeneration(Eigen::MatrixXd path);
    void rcvWaypointsCallBack(const nav_msgs::Path & wp);
    void rcvOdometryCallBack(const nav_msgs::Odometry & odom);
    void imuCallBack(const sensor_msgs::Imu & imu);
    void trajPublish( MatrixXd polyCoeff, VectorXd time); //zxzxzxzx

void rcvOdometryCallBack(const nav_msgs::Odometry & odom)
{   
    _has_odom = true;
    _startPos(0)  = odom.pose.pose.position.x;
    _startPos(1)  = odom.pose.pose.position.y;
    _startPos(2)  = odom.pose.pose.position.z;    

    _startVel(0)  = odom.twist.twist.linear.x;
    _startVel(1)  = odom.twist.twist.linear.y;
    _startVel(2)  = odom.twist.twist.linear.z;    
    Quat.x() = odom.pose.pose.orientation.x;
    Quat.y() = odom.pose.pose.orientation.y;
    Quat.z() = odom.pose.pose.orientation.z;
    Quat.w() = odom.pose.pose.orientation.w;
    Quat = Quat.normalized();
    Rota = Quaternion2Rota(Quat);    //from body to earth
}

void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) = _startPos;
    
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    trajGeneration(waypoints);

    trajPublish( _polyCoeff, _polyTime);
}

Matrix3d Quaternion2Rota(Quaterniond q)
{
    Eigen::Matrix3d rota;
    double r, i, j, k;
    r = q.w();
    i = q.x();
    j = q.y();
    k = q.z();

    // convert to rota
    rota(0, 0) = 1 - 2 * (j * j + k * k);
    rota(0, 1) = 2 * (i * j - k * r);
    rota(0, 2) = 2 * (i * k + j * r);
    //
    rota(1, 0) = 2 * (i * j + k * r);
    rota(1, 1) = 1 - 2 * (i * i + k * k);
    rota(1, 2) = 2 * (j * k - i * r);
    //
    rota(2, 0) = 2 * (i * k - j * r);
    rota(2, 1) = 2 * (j * k + i * r);
    rota(2, 2) = 1 - 2 * (i * i + j * j);

    return rota;
}
void rcvJoyCallBack(const sensor_msgs::Joy &joy) //zxzxzxzx
{   
    static double last_joy_sw_ch = 0;
    #define SWITCH_CHANNEL 5
    double joy_sw_ch = joy.axes.at(SWITCH_CHANNEL);

    if(joy_sw_ch > -0.75 && last_joy_sw_ch < -0.75) // switch moved
    {
        //joyState = START_PUB_TRAJ;
        trajPublish( _polyCoeff, _polyTime);
    }
    // else if(joy_sw_ch < -0.75)
    // {
    //     joyState = DONT_PUB_TRAJ;
    // }

    last_joy_sw_ch = joy_sw_ch;
}

VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);

    for (int k = 0; k < (Path.rows() - 1); k++)
    {
        double dtxyz;

        Vector3d p0   = Path.row(k);        
        Vector3d p1   = Path.row(k + 1);    
        double D    = (p1 - p0).norm();             

        double acct = (_Vel) / _Acc;
        double accd = (_Acc * acct * acct / 2);
        double dcct = _Vel / _Acc;                                  
        double dccd = _Acc * dcct * dcct / 2;                           

        if (D < accd + dccd)
        {   
            double t1 = sqrt( _Acc * D ) / _Acc;
            double t2 = (_Acc * t1) / _Acc;
            dtxyz     = t1 + t2;    
        }
        else
        {                                        
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
        }

	if(k!=0 && k!=(Path.rows() - 2)) // First and last. zxzx
	{
		dtxyz /= 1.5;
	}
	else
	{
		dtxyz *= 1.5;
	}

        time(k) = dtxyz;
    }

    return time;
}

void trajPublish( MatrixXd polyCoeff, VectorXd time) //zxzxzxzx
{    
    if(polyCoeff.size() == 0 || time.size() == 0)
    {
        ROS_WARN("[trajectory_generator_waypoint] empty trajectory, nothing to publish.");
        return;
    }

    unsigned int poly_number;

    static int count = 1; // The first trajectory_id must be greater than 0. zxzxzxzx

    quadrotor_msgs::PolynomialTrajectory traj_msg;


    traj_msg.header.seq = count;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = std::string("/map");
    traj_msg.trajectory_id = count;
    traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

    traj_msg.num_order = 2 * _dev_order - 1; // the order of polynomial
    traj_msg.num_segment = time.size();

    traj_msg.start_yaw = 0;
    traj_msg.final_yaw = 0;

    poly_number = traj_msg.num_order + 1;
    cout << "p_order:" << poly_number << endl;
    cout << "traj_msg.num_order:" << traj_msg.num_order << endl;
    cout << "traj_msg.num_segment:" << traj_msg.num_segment << endl;
    //cout << "time:" << time << endl;
    //cout << "polyCoeff:" << polyCoeff << endl;
    for(unsigned int i=0; i<traj_msg.num_segment; i++)
    {
        for (unsigned int j = 0; j < poly_number; j++)
        {
          traj_msg.coef_x.push_back(polyCoeff(i,j) * pow(time(i),j));
          traj_msg.coef_y.push_back(polyCoeff(i, poly_number + j) * pow(time(i),j));
          traj_msg.coef_z.push_back(polyCoeff(i, 2*poly_number + j) * pow(time(i),j));
        }
        traj_msg.time.push_back(time(i));
        traj_msg.order.push_back(traj_msg.num_order);
    }
    traj_msg.mag_coeff = 1;

    count++;
    ROS_WARN("[traj..gen...node] traj_msg publish");
    _traj_pub.publish(traj_msg);
}

void imuCallBack(const sensor_msgs::Imu & imu)
{
    A_B(0) = imu.linear_acceleration.x;
    A_B(1) = imu.linear_acceleration.y;
    A_B(2) = imu.linear_acceleration.z;     //  -9.8066;
    A_E = Rota * A_B;
    A_E(2) = A_E(2) - 9.8066;
}

void trajGeneration(Eigen::MatrixXd path)
{   
    //if( !_has_odom ) return;

    

    TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
    
    MatrixXd vel = MatrixXd::Zero(2, 3); 
    MatrixXd acc = MatrixXd::Zero(2, 3);

    vel.row(0) = _startVel;
    acc.row(0) = A_E;
    cout<<"acc:"<<acc<<endl;
    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    _polyTime  = timeAllocation(path);

    // generate a minimum-jerk piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);   

    //std::cout << "The matrix _polyTime is of size " << _polyTime.rows() << "x" << _polyTime.cols() << std::endl;
    //std::cout << "The matrix _polyCoeff is of size " << _polyCoeff.rows() << "x" << _polyCoeff.cols() << std::endl;

    visWayPointPath(path);
    visWayPointTraj( _polyCoeff, _polyTime);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel",   _Vel,   1.0 );
    nh.param("planning/acc",   _Acc,   1.0 );
    nh.param("planning/dev_order", _dev_order,  3 );
    nh.param("planning/min_order", _min_order,  3 );

    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    _poly_num1D = 2 * _dev_order;
    
    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );
    _odom_pts_sub    = nh.subscribe( "odom",  1, rcvOdometryCallBack );
    _joy_pts_sub     = nh.subscribe( "joy",  1, rcvJoyCallBack );
    _imu_sub     = nh.subscribe( "/mavros/imu/data", 1, imuCallBack);
    _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);


    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

/*    {
        // define the visualization information of the published velocity and acceleration commands
        _vis_pos_pub     = nh.advertise<visualization_msgs::Marker>("desired_position", 50);    
        _vis_vel_pub     = nh.advertise<visualization_msgs::Marker>("desired_velocity", 50);    
        _vis_acc_pub     = nh.advertise<visualization_msgs::Marker>("desired_acceleration", 50);

        _vis_pos.id = _vis_vel.id = _vis_acc.id = 0;
        _vis_pos.header.frame_id = _vis_vel.header.frame_id = _vis_acc.header.frame_id = "/map";
        
        _vis_pos.ns = "pos";
        _vis_pos.type   = visualization_msgs::Marker::SPHERE;
        _vis_pos.action = visualization_msgs::Marker::ADD;
        _vis_pos.color.a = 1.0; _vis_pos.color.r = 0.0; _vis_pos.color.g = 0.0; _vis_pos.color.b = 0.0;
        _vis_pos.scale.x = 0.2; _vis_pos.scale.y = 0.2; _vis_pos.scale.z = 0.2;

        _vis_vel.ns = "vel";
        _vis_vel.type = visualization_msgs::Marker::ARROW;
        _vis_vel.action = visualization_msgs::Marker::ADD;
        _vis_vel.color.a = 1.0; _vis_vel.color.r = 0.0; _vis_vel.color.g = 1.0; _vis_vel.color.b = 0.0;
        _vis_vel.scale.x = 0.2; _vis_vel.scale.y = 0.4; _vis_vel.scale.z = 0.4;

        _vis_acc.ns = "acc";
        _vis_acc.type = visualization_msgs::Marker::ARROW;
        _vis_acc.action = visualization_msgs::Marker::ADD;
        _vis_acc.color.a = 1.0; _vis_acc.color.r = 1.0; _vis_acc.color.g = 1.0; _vis_acc.color.b = 0.0;
        _vis_acc.scale.x = 0.2; _vis_acc.scale.y = 0.4; _vis_acc.scale.z = 0.4;
    }
*/
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();  
        rate.sleep();
    }

    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{        
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "/map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }

    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = _vis_traj_width;
    points.scale.y = _vis_traj_width;
    points.scale.z = _vis_traj_width;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = _vis_traj_width;
    line_list.scale.y = _vis_traj_width;
    line_list.scale.z = _vis_traj_width;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

Vector3d getVelPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
            if(j==0)
                time(j) = 0.0;
            else
                time(j) = j * pow(t, j-1);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Vector3d getAccPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );

        for(int j = 0; j < _poly_num1D; j ++)
            if( j==0 || j==1 )
                time(j) = 0.0;
            else
                time(j) = j * (j - 1) * pow(t, j-2);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

/*
This code includes all communication classes to MAVROS.
*/
#include <call_states/ros_communicate.h>
#include <cv_bridge/cv_bridge.h>
void Listener::init()
{
        // camera_mt = {0.12,0,0};  //for sim
        camera_mt = {0.06,0,0.046};   //for hardware
        Cam_mt<<0,0,1,-1,0,0,0,-1,0;
        fx = 382.7880554199219; cx = 319.20953369140625; fy = 382.7880554199219; cy = 242.66583251953125;
}
// void Listener::rcCb(const mavros_msgs::RCInConstPtr & inMsg)
// {
  
// }
void Listener::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    flight_state = *msg;
}
void Listener::estateCb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    flight_estate = *msg;
}

void Listener::posCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose = *msg;
    // position
    P_E(0) = pose.pose.position.x;
    P_E(1) = pose.pose.position.y;
    P_E(2) = pose.pose.position.z;
//     cout<<P_E<<endl;
    // orientation
    Quat.x() = pose.pose.orientation.x;
    Quat.y() = pose.pose.orientation.y;
    Quat.z() = pose.pose.orientation.z;
    Quat.w() = pose.pose.orientation.w;
    Quat = Quat.normalized();
    Rota = Quaternion2Rota(Quat);    //from body to earth
}
void Listener::triggerCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    trigger = true;
}
void Listener::wptsCb(const nav_msgs::Path::ConstPtr& msg)
{
 waypoint_update = true;
 nav_msgs::Path wpts = *msg;
 int len = wpts.poses.size();
 //MatrixXd waypoints_ins(len,3);
 waypoints.resize(len,3);
 for (int i =0;i<len;i++)
 {
 waypoints.row(i) << wpts.poses[i].pose.position.x, wpts.poses[i].pose.position.y, wpts.poses[i].pose.position.z;
 }
//  cout<< "wpts callback:" << waypoints << endl << len <<endl;
// waypoints = waypoints_ins;
}
// void Listener::posCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     geometry_msgs::PoseStamped pose = *msg;
//     // position
//     P_E(0) = pose.pose.position.x;
//     P_E(1) = pose.pose.position.y;
//     P_E(2) = pose.pose.position.z;
//     // orientation
//     Quat.x() = pose.pose.orientation.x;
//     Quat.y() = pose.pose.orientation.y;
//     Quat.z() = pose.pose.orientation.z;
//     Quat.w() = pose.pose.orientation.w;
//     Rota = Quat.normalized().toRotationMatrix();    //from body to earth
//     Rota_EB = Rota.transpose(); //from earth to body
//     //Euler = Rota.eulerAngles(0, 1, 2);  // roll pitch yaw
//     Euler = Quaternion2Euler(Quat);
// }

void Listener::velCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    geometry_msgs::TwistStamped velocity = *msg;
    // linear velocity
    V_E(0) = velocity.twist.linear.x;
    V_E(1) = velocity.twist.linear.y;
    V_E(2) = velocity.twist.linear.z;
}

// void Listener::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     sensor_msgs::Imu imu = *msg;
//     // accerleration
//     A_B(0) = imu.linear_acceleration.x;
//     A_B(1) = imu.linear_acceleration.y;
//     A_B(2) = imu.linear_acceleration.z;//-9.8066;
//     A_E = Rota * A_B;
//     A_E(2) = A_E(2) - 9.8066;
//     // angular velocity
//     Rate_B(0) = imu.angular_velocity.x;
//     Rate_B(1) = imu.angular_velocity.y;
//     Rate_B(2) = imu.angular_velocity.z;
//     Rate_E = Rota * Rate_B;
// }

void Listener::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu = *msg;
    // accerleration
    A_B(0) = imu.linear_acceleration.x;
    A_B(1) = imu.linear_acceleration.y;
    A_B(2) = imu.linear_acceleration.z;     //  -9.8066;
    A_E = Rota * A_B;
    A_E(2) = A_E(2) - 9.8066;
    // angular velocity
    Rate_B(0) = imu.angular_velocity.x;
    Rate_B(1) = imu.angular_velocity.y;
    Rate_B(2) = imu.angular_velocity.z;
}

void Listener::crdCb(const visualization_msgs::MarkerArray::ConstPtr& msg)
{ visualization_msgs::MarkerArray corridor = *msg;
  
  
  cout<< "corridor updated!" <<endl;
  int len = corridor.markers.size();
//   len = (len-2)/5+2;
//   len = ball.pose.orientation.w=1.0
  cout<< "corridor updated!" << len <<endl;
  int i = 0;
   while (i < len)
  { visualization_msgs::Marker ball =  corridor.markers[i];
    
    if (ball.color.b < 0.9)
    {cout << "keep cords num:"<< i << endl;
    break;}
    i++;}
//   for (int i =0; i<len;i++)
   cd_c.resize(i,3);
   cd_r.resize(i);
 int j = 0;
   while (j < i)
  { visualization_msgs::Marker ball =  corridor.markers[j];
    
    cd_c.row(j) << ball.pose.position.x, ball.pose.position.y, ball.pose.position.z;
    cd_r(j) = ball.scale.x/2;
    j++;
    
  }
 // cd_c = cd_c_ins;
 // cd_r = cd_r_ins;
}

void Listener::odomCb(const nav_msgs::Odometry::ConstPtr & msg)
    {
        V_E(0) = msg->twist.twist.linear.x;
        V_E(1) = msg->twist.twist.linear.y;
        V_E(2) = msg->twist.twist.linear.z;

            // position
        P_E(0) = msg->pose.pose.position.x;
        P_E(1) = msg->pose.pose.position.y;
        P_E(2) = msg->pose.pose.position.z;
        // cout<<P_E<<endl;
        // orientation
        Quat.x() = msg->pose.pose.orientation.x;
        Quat.y() = msg->pose.pose.orientation.y;
        Quat.z() = msg->pose.pose.orientation.z;
        Quat.w() = msg->pose.pose.orientation.w;
        Quat = Quat.normalized();
        Rota = Quaternion2Rota(Quat);    //from body to earth
    }

void Listener::static_pcl_Cb(const sensor_msgs::PointCloud2::ConstPtr & msg)
{   sensor_msgs::PointCloud cloud;
//    {cout<<"convert!"<<endl;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg,cloud);
    pcl_update = true;
    obs.clear();
    // obs.resize(cloud.points.size());
    // obs.resize(cloud.points.size);
    if (cloud.points.size() == 0)
    return;
    for (auto &point:cloud.points)
    {
        obs.emplace_back(point.x,point.y,point.z);
    }
    
}

void  Listener::objCb(const obj_state_msgs::ObjectsStates::ConstPtr & msg)
{   
    cout << "obj received: "<<msg->states.size()<<endl;
    dyn_update = true;
    dynobs.dyn_number = msg->states.size();
    dynobs.time_stamp =msg->header.stamp.toSec();
    dynobs.centers.clear();
    dynobs.vels.clear();
    dynobs.max_accs.clear();
    dynobs.obs_sizes.clear();
    cout<<"time gap from obj to now: "<<(ros::Time::now().toSec() - dynobs.time_stamp)<<endl;
    // dynobs.centers.resize(dynobs.dyn_number);
    // dynobs.vels.resize(dynobs.dyn_number);
    // dynobs.obs_sizes.resize(dynobs.dyn_number);
    // dynobs.max_accs.resize(dynobs.dyn_number);
    for (auto state:msg->states)
    {
        dynobs.centers.emplace_back(state.position.x, state.position.y, state.position.z);
        dynobs.vels.emplace_back(state.velocity.x, state.velocity.y, state.velocity.z);
        dynobs.obs_sizes.emplace_back(state.size.x, state.size.y, state.size.z);
        dynobs.max_accs.emplace_back(state.acceleration.x, state.acceleration.y, state.acceleration.z);
    }
}

void Listener::obsCb(const sensor_msgs::PointCloud2::ConstPtr & msg)
{   sensor_msgs::PointCloud cloud;
//    {cout<<"convert!"<<endl;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg,cloud);
    pcl_update = true;
    obs.clear();
    // obs.resize(cloud.points.size());
    dynobs.time_stamp = cloud.header.stamp.toSec();
    // dynobs.time_stamp = ros::Time::now().toSec() - 0.03;
    if (abs(cloud.points.back().y) + abs(cloud.points.back().z)  > 1e-5)
    {dynobs.dyn_number = 0;
    }
    else
    {
    dynobs.dyn_number = cloud.points.back().x;}
    dynobs.centers.clear();
    dynobs.vels.clear();
    dynobs.max_accs.clear();
    dynobs.obs_sizes.clear();
    dynobs.centers.resize(dynobs.dyn_number);
    dynobs.vels.resize(dynobs.dyn_number);
    dynobs.obs_sizes.resize(dynobs.dyn_number);
    dynobs.max_accs.resize(dynobs.dyn_number);
    obs.resize(cloud.points.size()-dynobs.dyn_number*4-1);
    // cout << "point cloud received, dynamic number: " <<  cloud.points.back().x << "pcl size:" <<cloud.points.size()<<endl;
    for (size_t i = 0; i < cloud.points.size()-1; i++)
    {
    if (i < obs.size())
    {
    obs[i](0) = cloud.points[i].x;
    obs[i](1) = cloud.points[i].y;
    obs[i](2) = cloud.points[i].z;
    }
    else if (i < obs.size() + dynobs.dyn_number)
{   dynobs.centers[i-obs.size()](0) = cloud.points[i].x;
    dynobs.centers[i-obs.size()](1) = cloud.points[i].y;
    dynobs.centers[i-obs.size()](2) = cloud.points[i].z;}
    else if (i < obs.size() + 2*dynobs.dyn_number)
{   dynobs.vels[i-obs.size()-dynobs.dyn_number](0) = cloud.points[i].x;
    dynobs.vels[i-obs.size()-dynobs.dyn_number](1) = cloud.points[i].y;
    dynobs.vels[i-obs.size()-dynobs.dyn_number](2) = cloud.points[i].z;
    }
    else if (i < obs.size() + 3*dynobs.dyn_number)
{   dynobs.max_accs[i-obs.size()-2*dynobs.dyn_number](0) = cloud.points[i].x;
    dynobs.max_accs[i-obs.size()-2*dynobs.dyn_number](1) = cloud.points[i].y;
    dynobs.max_accs[i-obs.size()-2*dynobs.dyn_number](2) = cloud.points[i].z;

    }
    else
{   dynobs.obs_sizes[i-obs.size()-3*dynobs.dyn_number](0) = cloud.points[i].x;
    dynobs.obs_sizes[i-obs.size()-3*dynobs.dyn_number](1) = cloud.points[i].y;
    dynobs.obs_sizes[i-obs.size()-3*dynobs.dyn_number](2) = cloud.points[i].z;}

    }
    // if (dynobs.dyn_number>0)
//    cout << "point cloud received, dynamic number: " <<  cloud.points.back().x << "pcl size:" <<cloud.points.size()<<"accs:\n"<<dynobs.max_accs[0]<<endl;
}
void Listener::depthCb(const sensor_msgs::Image::ConstPtr & msg)   //for onboard computer with a CPU too weak to generate pointcloud
{
    int w,h,d;
    double x, y, z;
    cv::Mat depthImgCopy_;
    ros::spinOnce();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      w = cv_ptr->image.cols;
      h = cv_ptr->image.rows;
      depthImgCopy_ = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // cout<<"1111"<<endl;
    obs.clear();
    // obs.resize(cloud.points.size());
    dynobs.dyn_number = 0;
    if (Rota.sum() ==0)
    return;
   
    for (int i=0;i<w;i+=10)
    {
    for (int j=0;j<h;j+=10)
    {
    d = depthImgCopy_.at<ushort>(j, i);
    // cout<<"d"<<d<<endl;
	 if(d>300 && d<8000)
	  {
            z = d * 0.001;
	    x = (i - cx) * z /fx;
	    y = (j - cy) * z /fy;
	    
    obs.emplace_back(x, y, z);
    obs.back() =  Rota*(Cam_mt*obs.back()+camera_mt) + P_E;

	  }

    }
    }
    if (obs.size()>0)
        pcl_update = true;
//   cout<<"end!!!!!"<<obs.size()<<endl;  
}
void Listener::pclCb(const sensor_msgs::PointCloud2::ConstPtr & msg)
{   

    ros::spinOnce();
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg,cloud);
    // cout<<"convert!\n"<<cloud.points.size()<<"\n"<<P_E<< endl;

    // cout<<"convert!\n"<<cloud.points.size()<<endl;
    obs.clear();
    // obs.resize(cloud.points.size());
    dynobs.dyn_number = 0;
    if (cloud.points.size() == 0 || Rota.sum() ==0)
    return;
    pcl_update = true;
    int step = (1,cloud.points.size()/2000);
    step = (step<1)?1:step;
    // cout<<"Rota:\n"<<Rota<<endl<<Cam_mt<<endl;
    for (int i=0; i<cloud.points.size();i+=step)
    {
    if (isnan(cloud.points[i].z))
    continue;
    obs.emplace_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    // obs[i](1) = point.y;
    // obs[i](2) = point.z;
    obs.back() =  Rota*(Cam_mt*obs.back()+camera_mt) + P_E;
    // cout<<"point:\n"<<obs.size()<<endl<<obs.back()<<endl;
    }
}
void  Listener::ballCb(const obj_state_msgs::ObjectsStates::ConstPtr & msg)
{   
    // cout << "ball received!"<<endl;
    dynobs.ball_number = msg->states.size();
    dyn_update = true;
    dynobs.ballpos.clear();
    dynobs.ballpos.resize(dynobs.ball_number);
    dynobs.ballvel.clear();
    dynobs.ballvel.resize(dynobs.ball_number);
    dynobs.ballacc.clear();
    dynobs.ballacc.resize(dynobs.ball_number);
    dynobs.ball_sizes.clear();
    dynobs.ball_sizes.resize(dynobs.ball_number);
    dynobs.ball_time_stamp = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;
    // dynobs.ball_time_stamp = ros::Time::now().toSec() - 0.05; // for bag sim
    for (size_t i = 0; i < dynobs.ball_number; i++)
    {
    dynobs.ballpos[i](0) = msg->states[i].position.x;
    dynobs.ballpos[i](1) = msg->states[i].position.y;
    dynobs.ballpos[i](2) = msg->states[i].position.z;
    dynobs.ballpos[i] = dynobs.ballpos[i];  //+P_E, for bag sim.;
    dynobs.ballvel[i](0) = msg->states[i].velocity.x;
    dynobs.ballvel[i](1) = msg->states[i].velocity.y;
    dynobs.ballvel[i](2) = msg->states[i].velocity.z;
    dynobs.ballvel[i] = dynobs.ballvel[i]; //+V_E;
    dynobs.ballacc[i](0) = msg->states[i].acceleration.x;
    dynobs.ballacc[i](1) = msg->states[i].acceleration.y;
    dynobs.ballacc[i](2) = msg->states[i].acceleration.z;
    dynobs.ballacc[i] = dynobs.ballacc[i];//+A_E;
    dynobs.ball_sizes[i](0) = 0.6;
    dynobs.ball_sizes[i](1) = 0.6;
    dynobs.ball_sizes[i](2) = 0.6;
    // cout << "ball received,  number: " <<  dynobs.ball_number << "   ball pos:  " <<dynobs.ballpos[i]<<endl;
    }
}
/*
FCU Modes Requests
*/
/*
class fcuModes
{
    private:
        // ros
        ros::NodeHandle nh_;
        ros::ServiceClient arming_client_, land_client_, set_mode_client_;
    
    public:
        // init clients
        fcuModes(ros::NodeHandle *nodehandle);

        // fcu modes
        bool setArm_(void);
        bool setLand_(void);
        bool setMode_Offboard_(void);
};

fcuModes::fcuModes(ros::NodeHandle* nodehandle):nh_(*nodehandle) 
{   
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

bool fcuModes::setArm_()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}

bool fcuModes::setLand_()
{
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;

    if (land_client_.call(land_cmd) && land_cmd.response.success)
    {
        ROS_INFO("Vehicle landed");
        return true;
    }
    return false;
}

bool fcuModes::setMode_Offboard_()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
                ROS_INFO("Offboard enabled");
        return true;
    }
    return false;
}
*/
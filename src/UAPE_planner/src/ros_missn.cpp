#include <ros_missn.h>

RosClass::RosClass(ros::NodeHandle *nodehandle, int FREQ, bool if_raw_pcl, bool if_raw_dpmap) : nh_(*nodehandle),
                                                                                                rate(FREQ)
{
    // subscribers
    //// mavros states
    rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, &RC_Data_t::feed, &rc_data);
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &Listener::stateCb, &listener_);
    exstate_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &Listener::estateCb, &listener_);
    //// control states
    pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &Listener::posCb, &listener_);
    vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, &Listener::velCb, &listener_);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &Listener::imuCb, &listener_);
    corridor_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/corridor", 1, &Listener::crdCb, &listener_);
    wpts_sub_ = nh_.subscribe<nav_msgs::Path>("/wpts_path", 1, &Listener::wptsCb, &listener_);
    obs_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_global_all", 1, &Listener::obsCb, &listener_);
    static_pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_global_static", 1, &Listener::static_pcl_Cb, &listener_);
    if (if_raw_pcl)
    {
        raw_pcl_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, &Listener::pclCb, &listener_);
    }
    if (if_raw_dpmap)
    {
        depth_sub_ = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw1111", 1, &Listener::depthCb, &listener_);
    }
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/vicon_imu_ekf_odom", 1, &Listener::odomCb, &listener_);
    traj_start_trigger_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger", 10, &Listener::triggerCb, &listener_);
    ball_sub_ = nh_.subscribe<obj_state_msgs::ObjectsStates>("/objects_states", 1, &Listener::ballCb, &listener_);
    obj_sub_ = nh_.subscribe<obj_state_msgs::ObjectsStates>("/obj_states", 1, &Listener::objCb, &listener_);
    // publishers
    pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    raw_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    actuCtrl_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);

    traj_pub_ = nh_.advertise<nav_msgs::Path>("/optimal_trajectory", 2);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/kino_path", 2);
    detail_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detailed_optimal_trajectory", 2);
    polyh_pub_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("/polyhedra", 1);
    poscmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    ball_vispub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ball_vis_states", 10);
    cam_vispub_ = nh_.advertise<visualization_msgs::Marker>("/camera_fov", 10);
    cam_listpub_ = nh_.advertise<visualization_msgs::MarkerArray>("/camera_fov_list", 10);
    // fcu modes
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // control frequency
    Freq_ = FREQ;
    h_ = 1.0 / FREQ;
    listener_.init();
}

Vector3d RosClass::get_position()
{
    cout << "current position: \n"
         << listener_.P_E << endl;
    return listener_.P_E;
}

void RosClass::init(double HGT, double yaw, double THR_C, double TOR_C, double MASS, double Ix, double Iy, double Iz)
{

    // drone parameters
    mass_ = MASS;
    Inertia_.diagonal() << Ix, Iy, Iz;
    Yaw = yaw;
    // control coefficients
    k_thr_ = THR_C / mass_ / gravity; // / gravity;   k_th_ shold be 1/max thrust force(N)
    k_tor_xy_ = TOR_C;
    k_tor_z_ = TOR_C;

    Start << 0.0, 0.0, HGT;
    End << 0.0, 0.0, 1.0;
}

States RosClass::launch(void)
{
    mavros_msgs::PositionTarget takeoff;
    takeoff.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048;
    takeoff.position.x = Start(0);
    takeoff.position.y = Start(1);
    takeoff.position.z = Start(2);
    takeoff.yaw = Yaw;

    // wait for arm and offboard mode.
    printf("Waiting for arm and OFFBOARD mode.\n");
    while (!listener_.flight_state.armed && ros::ok())
    {
        setArm_();
        ros::spinOnce();
        rate.sleep();
    }
    while (listener_.flight_state.mode != "OFFBOARD" && ros::ok())
    {
        pos_pub_.publish(takeoff);
        setMode_Offboard_();
        ros::spinOnce();
        rate.sleep();
    }
    printf("Armed. OFFBOARD mode.\n");

    // take off
    Start(0) = listener_.P_E(0);
    Start(1) = listener_.P_E(1);
    takeoff.position.x = Start(0);
    takeoff.position.y = Start(1);
    while (ros::ok())
    {
        Vector3d distance = Start - listener_.P_E;
        if (distance.norm() < 0.2)
        {
            break;
        }
        pos_pub_.publish(takeoff);
        ros::spinOnce();
        rate.sleep();
    }

    get_state();

    return state;
}

States RosClass::step(double double_n, double yaw_rate, Vector3d pos, Vector3d vel, Vector3d acc, string mode)
{
    // step
    if (mode == "thrust_n_euler")
    {
        double throttle = k_thr_ * double_n; // double_n is total thrust force here
        Quaterniond quat = Euler2Quaternion(pos);

        mavros_msgs::AttitudeTarget ctrl_sp;
        ctrl_sp.type_mask = 7;
        ctrl_sp.thrust = clip(throttle, 0, 1);
        ctrl_sp.orientation.x = quat.x();
        ctrl_sp.orientation.y = quat.y();
        ctrl_sp.orientation.z = quat.z();
        ctrl_sp.orientation.w = quat.w();

        raw_pub_.publish(ctrl_sp);
    }
    else if (mode == "thrust_n_rate")
    {
        double throttle = k_thr_ * double_n;

        mavros_msgs::AttitudeTarget ctrl_sp;
        ctrl_sp.type_mask = 128;
        ctrl_sp.thrust = clip(throttle, 0, 1);
        ctrl_sp.body_rate.x = pos(0);
        ctrl_sp.body_rate.y = pos(1);
        ctrl_sp.body_rate.z = pos(2);

        raw_pub_.publish(ctrl_sp);
    }
    else if (mode == "yaw_n_position")
    {
        mavros_msgs::PositionTarget p_sp;
        p_sp.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048;
        p_sp.position.x = pos(0);
        p_sp.position.y = pos(1);
        p_sp.position.z = pos(2);
        p_sp.yaw = double_n;

        pos_pub_.publish(p_sp);
    }
    else if (mode == "yaw_n_velocity")
    {
        mavros_msgs::PositionTarget v_sp;
        v_sp.coordinate_frame = 1;
        v_sp.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 2048;
        v_sp.velocity.x = vel(0);
        v_sp.velocity.y = vel(1);
        v_sp.velocity.z = vel(2);
        v_sp.yaw = double_n;

        pos_pub_.publish(v_sp);
    }
    else if (mode == "pos_vel_acc_yaw")
    {
        mavros_msgs::PositionTarget pos_target;

        // pos_target.header = Header()
        pos_target.header.frame_id = "map";
        pos_target.header.stamp = ros::Time::now();
        pos_target.coordinate_frame = 1;
        pos_target.type_mask = 512 + 2048;
        pos_target.yaw = double_n;
        // geometry_msgs::Point pos;
        pos_target.position.x = pos(0);
        pos_target.position.y = pos(1);
        pos_target.position.z = pos(2);
        pos_target.velocity.x = vel(0);
        pos_target.velocity.y = vel(1);
        pos_target.velocity.z = vel(2);
        pos_target.acceleration_or_force.x = acc(0);
        pos_target.acceleration_or_force.y = acc(1);
        pos_target.acceleration_or_force.z = acc(2);

        pos_pub_.publish(pos_target);
    }
    else if (mode == "pos_vel_acc_yaw_c")
    {
        quadrotor_msgs::PositionCommand pos_target;

        // pos_target.header = Header()
        pos_target.header.frame_id = "map";
        pos_target.header.stamp = ros::Time::now();

        pos_target.yaw = double_n;
        // pos_target.yaw_dot = yaw_rate;
        geometry_msgs::Point pos1;
        pos1.x = pos(0);
        pos1.y = pos(1);
        pos1.z = pos(2);
        geometry_msgs::Vector3 vel1;
        vel1.x = vel(0);
        vel1.y = vel(1);
        vel1.z = vel(2);
        geometry_msgs::Vector3 accel;
        accel.x = acc(0);
        accel.y = acc(1);
        accel.z = acc(2);
        pos_target.position = pos1;
        pos_target.velocity = vel1;
        pos_target.acceleration = accel;

        poscmd_pub_.publish(pos_target);
    }
    ros::spinOnce();
    // rate.sleep();

    get_state();

    crashed_();

    return state;
}
void RosClass::pub_path(vector<Eigen::Vector3d> &waypoints)
{
    nav_msgs::Path traj;
    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    for (int i = 0; i < waypoints.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        //    pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = waypoints[i](0);
        pose.pose.position.y = waypoints[i](1);
        pose.pose.position.z = waypoints[i](2);

        traj.poses.push_back(pose);
    }
    path_pub_.publish(traj);
}
void RosClass::pub_traj(MatrixXd pos, MatrixXd vel, MatrixXd acc, Vec3f fail_pt)
{
    nav_msgs::Path traj;
    visualization_msgs::MarkerArray detail_traj;
    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    //    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = fail_pt[0];
    pose.pose.position.y = fail_pt[1];
    pose.pose.position.z = fail_pt[2];
    visualization_msgs::Marker pos_sample;
    pos_sample.header.frame_id = "map";
    pos_sample.header.stamp = ros::Time::now();
    pos_sample.id = 0;
    pos_sample.type = visualization_msgs::Marker::SPHERE;
    pos_sample.pose = pose.pose;
    pos_sample.pose.orientation.w = 1.0;
    pos_sample.scale.x = 0.2;
    pos_sample.scale.y = 0.2;
    pos_sample.scale.z = 0.2;
    pos_sample.color.r = 1.0;
    pos_sample.color.a = 1.0;
    pos_sample.lifetime = ros::Duration(1);
    detail_traj.markers.push_back(pos_sample);

    for (int i = 0; i < pos.rows(); i++)
    {
        visualization_msgs::Marker pos_sample;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        //    pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = pos(i, 0);
        pose.pose.position.y = pos(i, 1);
        pose.pose.position.z = pos(i, 2);
        traj.poses.push_back(pose);
        // if (i < 9)
        // {
        //     pos_sample.header.frame_id = "map";
        //     pos_sample.header.stamp = ros::Time::now();
        //     pos_sample.id = i;
        //     pos_sample.type = visualization_msgs::Marker::SPHERE;
        //     pos_sample.pose = pose.pose;
        //     pos_sample.pose.orientation.w = 1.0;
        //     pos_sample.scale.x = 0.2;
        //     pos_sample.scale.y = 0.2;
        //     pos_sample.scale.z = 0.2;
        //     pos_sample.color.b = 1.0;
        //     pos_sample.color.a = 1.0;
        //     pos_sample.lifetime = ros::Duration(0.5);
        //     detail_traj.markers.push_back(pos_sample);
        // }
    }

    traj_pub_.publish(traj);
    detail_traj_pub_.publish(detail_traj);
}

void RosClass::pub_fovlist(MatrixXd pos, MatrixXd vel, MatrixXd acc, Eigen::Matrix<double, 3, 5> camera_vertex_b, vector<double> yaw_plan)
{
    visualization_msgs::MarkerArray fovList;
    acc.col(2).array() += 9.8016;
    for (int j = 0; j < yaw_plan.size(); j += 1)
    {
        double sp_yaw1 = yaw_plan[j];
        // if (j==0) sp_yaw1 += 0.01;
        double thrust = acc.row(j).norm();
        double sp_theta = atan((acc.row(j)(0) + acc.row(j)(1) * tan(sp_yaw1)) / (acc.row(j)(2) * (cos(sp_yaw1) + sin(sp_yaw1) * tan(sp_yaw1))));
        double sp_phi = acos(acc.row(j)(2) / thrust / cos(sp_theta));
        AngleAxisd rollAngle(AngleAxisd(sp_phi, Vector3d::UnitX()));
        AngleAxisd pitchAngle(AngleAxisd(sp_theta, Vector3d::UnitY()));
        AngleAxisd yawAngle(AngleAxisd(sp_yaw1, Vector3d::UnitZ()));
        Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
        Matrix3d Rota = Quaternion2Rota(quaternion.normalized());
        //  cout<<"Rota:"<<Rota<<" pos: "<<pos.col(j).array()<<endl;
        Matrix<double, 3, 5> camera_vertex = (Rota * camera_vertex_b).array().colwise() + pos.row(j).transpose().array();
        visualization_msgs::Marker fov;
        fov.type = fov.LINE_LIST;
        fov.header.frame_id = "map";
        fov.header.stamp = ros::Time::now();

        fov.color.a = 1.0;
        fov.color.r = 1.0;
        fov.color.g = 0.0;
        fov.color.b = 0.0;
        fov.scale.x = 0.03;
        geometry_msgs::Point p3;
        p3.x = camera_vertex.col(0)(0);
        p3.y = camera_vertex.col(0)(1);
        p3.z = camera_vertex.col(0)(2);
        // p3.x,p3.y,p3.z = uav_pos
        // dots=np.matmul(self.b2e,self.dots.T).T+uav_pos
        for (int i = 1; i < 5; i++)
        {
            geometry_msgs::Point p1, p2;
            p1.x = camera_vertex.col(i)(0);
            p1.y = camera_vertex.col(i)(1);
            p1.z = camera_vertex.col(i)(2);
            if (i < 4)
            {
                p2.x = camera_vertex.col(i + 1)(0);
                p2.y = camera_vertex.col(i + 1)(1);
                p2.z = camera_vertex.col(i + 1)(2);
            }
            else
            {
                p2.x = camera_vertex.col(1)(0);
                p2.y = camera_vertex.col(1)(1);
                p2.z = camera_vertex.col(1)(2);
            }
            fov.points.emplace_back(p1);
            fov.points.emplace_back(p2);
            fov.points.emplace_back(p3);
            fov.points.emplace_back(p1);
        }
        fov.id = 2 * j;
        fov.pose.orientation.w = 1.0;
        fovList.markers.push_back(fov);
        //////
        // Vector2d v1, v2;
        // v1 << 1.0, 0.0;
        // v2 = vel.row(j).head(2);
        // sp_yaw1 = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
        // if (v2(1) < 0)
        // {
        //     sp_yaw1 = -sp_yaw1;
        // }
        // sp_theta = atan((acc.row(j)(0) + acc.row(j)(1) * tan(sp_yaw1)) / (acc.row(j)(2) * (cos(sp_yaw1) + sin(sp_yaw1) * tan(sp_yaw1))));
        // sp_phi = acos(acc.row(j)(2) / thrust / cos(sp_theta));
        // AngleAxisd rollAngle1(AngleAxisd(sp_phi, Vector3d::UnitX()));
        // AngleAxisd pitchAngle1(AngleAxisd(sp_theta, Vector3d::UnitY()));
        // AngleAxisd yawAngle1(AngleAxisd(sp_yaw1, Vector3d::UnitZ()));
        // quaternion = yawAngle1 * pitchAngle1 * rollAngle1;
        // Rota = Quaternion2Rota(quaternion.normalized());
        // cout << "Rota:" << Rota << endl;
        // camera_vertex = (Rota * camera_vertex_b).array().colwise() + pos.row(j).transpose().array();
        // if (j == 0)
        //     fov.color.a = 0.5;
        // else
        //     fov.color.a = 1.0;
        // fov.color.r = 0.0;
        // fov.color.b = 1.0;
        // fov.color.g = 0.2;
        // fov.scale.x = 0.03;
        // // geometry_msgs::Point p3;
        // p3.x = camera_vertex.col(0)(0);
        // p3.y = camera_vertex.col(0)(1);
        // p3.z = camera_vertex.col(0)(2);
        // // p3.x,p3.y,p3.z = uav_pos
        // // dots=np.matmul(self.b2e,self.dots.T).T+uav_pos
        // fov.points.clear();
        // for (int i = 1; i < 5; i++)
        // {
        //     geometry_msgs::Point p1, p2;
        //     p1.x = camera_vertex.col(i)(0);
        //     p1.y = camera_vertex.col(i)(1);
        //     p1.z = camera_vertex.col(i)(2);
        //     if (i < 4)
        //     {
        //         p2.x = camera_vertex.col(i + 1)(0);
        //         p2.y = camera_vertex.col(i + 1)(1);
        //         p2.z = camera_vertex.col(i + 1)(2);
        //     }
        //     else
        //     {
        //         p2.x = camera_vertex.col(1)(0);
        //         p2.y = camera_vertex.col(1)(1);
        //         p2.z = camera_vertex.col(1)(2);
        //     }

        //     fov.points.emplace_back(p1);
        //     fov.points.emplace_back(p2);
        //     fov.points.emplace_back(p3);
        //     fov.points.emplace_back(p1);
        // }
        // fov.id = 2 * j + 1;
        // fov.pose.orientation.w = 1.0;
        // fovList.markers.push_back(fov);
    }
    cam_listpub_.publish(fovList);
}

void RosClass::pub_polyh(vec_E<Polyhedron3D> &polyhedra)
{
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = ros::Time::now();
    polyh_pub_.publish(poly_msg);
}

void RosClass::land(Vector3d endp)
{
    // fly to land point
    mavros_msgs::PositionTarget land;
    land.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 1024 + 2048;
    land.position.x = endp(0);
    land.position.y = endp(1);
    land.position.z = endp(2);
    // land.yaw = 0;

    pos_pub_.publish(land);
    while (ros::ok())
    {
        Vector3d distance = End - listener_.P_E;
        if (distance.norm() < 0.4)
        {
            break;
        }
        pos_pub_.publish(land);
        ros::spinOnce();
        rate.sleep();
    }

    printf("Setting Land Mode.\n");

    // set land mode
    bool landed = setLand_();
    while (ros::ok() && !landed && listener_.flight_state.armed)
    {
        landed = setLand_();
        ros::spinOnce();
        rate.sleep();
    }

    printf("Landed.\n");
}

// States RosClass::get_state_(void)
// {
//     // raw signals
//     state.P_E = listener_.P_E;
//     state.V_E = listener_.V_E;
//     //// attitude
//     state.Quat = listener_.Quat;
//     state.Rota = listener_.Rota;            //state.Quat.normalized().toRotationMatrix();    //from body to earth
//     state.Rota_EB = state.Rota.transpose(); //from earth to body
//     state.Euler = Quaternion2Euler(state.Quat);
//     //Euler = Rota.eulerAngles(0, 1, 2);  // roll pitch yaw
//     //// imu
//     state.Rate_Braw = listener_.Rate_B;
//     state.A_Eraw = listener_.A_E;

//     // first-order filter for ctrl
//     //// accel
//     state.A_E = state.A_Eraw;
//     // state.A_E = filter_accel.filter(state.A_Eraw);
//     //// rate
//     state.Rate_B = state.Rate_Braw;
//     // state.Rate_B = filter_rate.filter(state.Rate_Braw);

//     state.Rate_E = state.Rota * state.Rate_Braw;
//     cd_c = listener_.cd_c;
//     cd_r = listener_.cd_r;
//     waypoints = listener_.waypoints;
//     pcl_update = listener_.pcl_update;
//     waypoint_update = listener_.waypoint_update;
//     obs_pointer = &listener_.obs;
//     dynobs_pointer = &listener_.dynobs;
//     pcl_pointer = &listener_.cloud;
//     trigger = listener_.trigger;
// //     cout << "state:" << state.P_E << state.Euler << listener_.flight_state <<endl;
//     return state;
// }

// void RosClass::pub_fov_list(Eigen::Matrix<double, 3, 5>& camera_vertex)
// {

// }

void RosClass::pub_fovshape(Eigen::Matrix<double, 3, 5> &camera_vertex)
{
    visualization_msgs::Marker fov;
    // uav_pos=pose[0:3]
    fov.type = fov.LINE_LIST;
    fov.header.frame_id = "map";
    fov.header.stamp = ros::Time::now();

    fov.color.a = 1.0;
    fov.color.r = 1.0;
    fov.color.g = 0.2;
    fov.scale.x = 0.03;
    geometry_msgs::Point p3;
    p3.x = camera_vertex.col(0)(0);
    p3.y = camera_vertex.col(0)(1);
    p3.z = camera_vertex.col(0)(2);
    // p3.x,p3.y,p3.z = uav_pos
    // dots=np.matmul(self.b2e,self.dots.T).T+uav_pos
    for (int i = 1; i < 5; i++)
    {
        geometry_msgs::Point p1, p2;
        p1.x = camera_vertex.col(i)(0);
        p1.y = camera_vertex.col(i)(1);
        p1.z = camera_vertex.col(i)(2);
        if (i < 4)
        {
            p2.x = camera_vertex.col(i + 1)(0);
            p2.y = camera_vertex.col(i + 1)(1);
            p2.z = camera_vertex.col(i + 1)(2);
        }
        else
        {
            p2.x = camera_vertex.col(1)(0);
            p2.y = camera_vertex.col(1)(1);
            p2.z = camera_vertex.col(1)(2);
        }
        fov.points.emplace_back(p1);
        fov.points.emplace_back(p2);
        fov.points.emplace_back(p3);
        fov.points.emplace_back(p1);
    }
    fov.id = 6;
    fov.pose.orientation.w = 1.0;
    cam_vispub_.publish(fov);
}
void RosClass::pub_ballstates()
{
    visualization_msgs::MarkerArray balls;
    double t_gap = ros::Time::now().toSec() - dynobs_pointer->ball_time_stamp;
    //   balls.header.frame_id = "map";
    //   balls.header.stamp = ros::Time();
    //   cout<<"pub vised ball:"<<endl;
    for (int i = 0; i < dynobs_pointer->ball_number; i++)
    {
        for (float j = t_gap; j < 1.2 + t_gap; j += 0.2)
        {
            visualization_msgs::Marker ball_pos;
            visualization_msgs::Marker ball_vel;

            ball_pos.header.frame_id = "map";
            ball_pos.header.stamp = ros::Time::now();
            ball_pos.id = 30 * i + (int)(j * 5);
            ball_pos.type = visualization_msgs::Marker::SPHERE;
            ball_pos.action = visualization_msgs::Marker::ADD;
            ball_pos.pose.position.x = dynobs_pointer->ballpos[i](0) + dynobs_pointer->ballvel[i](0) * j + dynobs_pointer->ballacc[i](0) * j * j * 0.5;
            ball_pos.pose.position.y = dynobs_pointer->ballpos[i](1) + dynobs_pointer->ballvel[i](1) * j + dynobs_pointer->ballacc[i](1) * j * j * 0.5;
            ball_pos.pose.position.z = dynobs_pointer->ballpos[i](2) + dynobs_pointer->ballvel[i](2) * j + dynobs_pointer->ballacc[i](2) * j * j * 0.5;
            ball_pos.pose.orientation.w = 1.0;
            ball_pos.scale.x = dynobs_pointer->ball_sizes[i](0);
            ball_pos.scale.y = dynobs_pointer->ball_sizes[i](1);
            ball_pos.scale.z = dynobs_pointer->ball_sizes[i](2);
            ball_pos.color.a = 0.2;
            ball_pos.color.r = 1.0;
            geometry_msgs::Point p1, p2;
            ball_vel.header.frame_id = "map";
            ball_vel.header.stamp = ros::Time::now();
            ball_vel.type = visualization_msgs::Marker::ARROW;
            ball_vel.id = 30 * i + 10 + (int)(5 * j);
            p1.x = ball_pos.pose.position.x;
            p1.y = ball_pos.pose.position.y;
            p1.z = ball_pos.pose.position.z;
            p2.x = p1.x + dynobs_pointer->ballvel[i](0) + dynobs_pointer->ballacc[i](0) * j;
            p2.y = p1.y + dynobs_pointer->ballvel[i](1) + dynobs_pointer->ballacc[i](1) * j;
            p2.z = p1.z + dynobs_pointer->ballvel[i](2) + dynobs_pointer->ballacc[i](2) * j;
            ball_vel.points.push_back(p1);
            ball_vel.points.push_back(p2);

            ball_vel.scale.x = 0.1;
            ball_vel.scale.y = 0.2;
            ball_vel.scale.z = 0.2;
            ball_vel.color.a = 1;
            ball_vel.color.r = 0.1;
            ball_vel.color.g = 0.8;
            ball_vel.color.b = 0.1;
            ball_pos.lifetime = ros::Duration(0.1);
            ball_vel.lifetime = ros::Duration(0.1);
            balls.markers.push_back(ball_pos);
            balls.markers.push_back(ball_vel);
        }
    }
    ball_vispub_.publish(balls);
    //   cout<<"pub vised ball finished"<<endl;
}
States RosClass::get_state()
{
    // raw signals
    ros::spinOnce();
    state.P_E = listener_.P_E;
    state.V_E = listener_.V_E;
    //// attitude
    state.Quat = listener_.Quat;
    state.Rota = listener_.Rota;            // state.Quat.normalized().toRotationMatrix();    //from body to earth
    state.Rota_EB = state.Rota.transpose(); // from earth to body
    state.Euler = Quaternion2Euler(state.Quat);
    // Euler = Rota.eulerAngles(0, 1, 2);  // roll pitch yaw
    //// imu
    state.Rate_Braw = listener_.Rate_B;
    state.A_Eraw = listener_.A_E;

    // first-order filter for ctrl
    //// accel
    state.A_E = state.A_Eraw;
    // state.A_E = filter_accel.filter(state.A_Eraw);
    //// rate
    state.Rate_B = state.Rate_Braw;
    // state.Rate_B = filter_rate.filter(state.Rate_Braw);

    state.Rate_E = state.Rota * state.Rate_Braw;
    cd_c = listener_.cd_c;
    cd_r = listener_.cd_r;
    waypoint_update = listener_.waypoint_update;
    pcl_update = listener_.pcl_update;
    dyn_update = listener_.dyn_update;
    waypoints = listener_.waypoints;
    obs_pointer = &listener_.obs;
    dynobs_pointer = &listener_.dynobs;
    // cout << "11"<<endl;
    pcl_pointer = &listener_.cloud;
    trigger = listener_.trigger;
    // cout << "(rosmission)received obs points:" << obs_pointer->size()<<endl << obs_pointer <<endl;
    // cout << "(rosmission)dynamic obs number:" << dynobs_pointer->dyn_number <<endl;
    //   cout<< "wpts callback 1:" << waypoints << endl;
    //     cout << "state:" << state.P_E << state.Euler << listener_.flight_state <<endl;

    return state;
}

void RosClass::crashed_()
{
    done = false;
    // double z = state.P_E(2);
    // if (z>8.0 || z<0.3)
    // {
    //     printf("Crashed at height %.2f.\n", z);
    //     done =  true;
    // }
    // else
    // {
    //     done = false;
    // }
}

/*
fcu modes
*/
void RosClass::set_cod_update(bool cod_update)
{
    listener_.waypoint_update = cod_update;
}

void RosClass::set_pcl_update(bool pcl_update)
{
    listener_.pcl_update = pcl_update;
    listener_.dyn_update = pcl_update;
}
bool RosClass::setArm_()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
    {
        // ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}

bool RosClass::setLand_()
{
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;

    if (land_client_.call(land_cmd) && land_cmd.response.success)
    {
        // ROS_INFO("Vehicle landed");
        return true;
    }
    return false;
}

bool RosClass::setMode_Offboard_()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        // ROS_INFO("Offboard enabled");
        return true;
    }
    return false;
}

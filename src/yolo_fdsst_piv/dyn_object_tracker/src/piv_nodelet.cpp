#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <math.h>
#include "fftw3.h"
#include <Eigen/Eigen>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加引用

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include "pcl_ros/point_cloud.h"
#include <obj_state_msgs/ObjectsStates.h>
#include <target_ros_msgs/BoundingBoxes.h>
#include <target_ros_msgs/BoundingBox.h>

#include "obj_state_msgs/State.h"
#include "Tools.h"
#include "kalman.hpp"
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>

#include <string>
#include <vector>
#include <queue>
#include <deque>
#include <list>
#include <unordered_map>
// #include "include/dbscan.h"
namespace pcl_filter_ns
{
    using namespace std;
    struct HashFunc_t
    {
        size_t operator()(const vector<int> &key) const
        {
            std::hash<int> hasher;
            size_t seed = 0;
            for (int i : key)
            {
                seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };
    class PIV : public nodelet::Nodelet
    {
    public:
        PIV() { ; }
        ~PIV() { ; }

    private:
        // 定义点云类型
        ros::NodeHandle nh;
        typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
        Eigen::Vector3i map_o = {200, 200, 20};
        const static int x_size = 601, y_size = 401, z_size = 51;
        int local_map[x_size][y_size][z_size];
        double time_map[x_size][y_size][z_size];
        Eigen::Vector4d bbox_size;
        Eigen::Vector3i origin_v;
        double extd_bb_1[16][16][10][3];
        double extd_bb_2[16][16][10][3];
        double in3[16][16][10][3];
        // int *extd_bb_1;
        // int *extd_bb_2;
        // int *in3;
        fftw_complex *out1 = NULL, *out2 = NULL, *out3 = NULL; // fftwf_complex --> 即为float版本
        fftw_plan p, p_inv;
        const int peak_num = 4;
        double imu_delay = 0.015;
        PointCloud pcl_sta;
        ros::Publisher pcl_pub, obj_vis_pub, obj_state_pub;
        ros::Subscriber dynobj_sub;
        // queue<std::shared_ptr<PointCloud>> pcl_list;
        queue<PointCloud::Ptr> pcl_list;
        queue<ros::Time> time_list;
        queue<Eigen::Matrix3d> Rota_list;
        queue<Eigen::Vector3d> Pos_list;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub;
        message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> *ApproSync_;
        typedef std::vector<Eigen::Vector3d> vectors3d;
        // std::vector<Eigen::Matrix<double, 3, Dynamic>> xyz_clusters1, xyz_clusters2, rgb_clusters1,rgb_clusters2;
        boost::shared_ptr<sensor_msgs::CameraInfo const> cam_info_msg;
        std::vector<Eigen::Matrix<double, 3, Dynamic>> xyz_clusters2_M;
        std::vector<vectors3d> xyz_clusters1, xyz_clusters2, rgb_clusters1, rgb_clusters2;
        std::vector<Eigen::Matrix<double, 3, 2>> extended_3dbb;
        std::vector<Eigen::Matrix<int, 3, 2>> extended_3dbb_map;
        double inflate_bb, min_dt;
        Eigen::Vector3d inflate_vector;
        double cx = 422.04, cy = 240.698, fx = 611.633, fy = 610.954; // 848*480,hardware
        Eigen::Vector2d img_p1, img_p2;
        Eigen::Vector3d camera_mt, depth2rgb_extrinsics;
        Eigen::Matrix3d Cam_mt;
        Eigen::Vector3d Rate_B1, Euler, Pos, Pos1, Vel1;
        Eigen::Quaterniond Quat, Quat1;
        Eigen::Matrix3d Rota, Rota1;
        std::vector<int> obj_ids;
        std::vector<Eigen::Vector3d> ori_var_list;
        Eigen::Vector3d ori_var;
        std::deque<std::vector<int>> voxel_id_list;
        std::unordered_map<std::vector<int>, bool, HashFunc_t> voxel_id_map;
        // ros::Time rostime;
        std::vector<double> camera_m, camera_ext;
        bool debug, sim;
        target_ros_msgs::BoundingBoxes obj;
        int obj_num;
        int pcl_sta_len;
        double resolution;
        double obj_timegap_max;
        bool obj_updated = false;
        double max_vel;
        std::vector<Eigen::Vector2d> obj_shrink;
        double shrink_factor, coe_atte_init, coe_atte, sta_thres_init, sta_thres;
        // KF
        const int states_n = 2; // Number of states
        const int obsv_n = 2;   // Number of measurements

        // double kf_dt; // Time step
        // ros::Time last_kf_t;
        Eigen::MatrixXd A; // System dynamics matrix
        Eigen::MatrixXd C; // State observation matrix
        Eigen::MatrixXd Q; // Process noise covariance
        Eigen::MatrixXd R; // Measurement noise covariance
        Eigen::MatrixXd P; // Estimate error covariance
        std::vector<KalmanFilter> KF_list;
        std::vector<Eigen::Matrix<double, 2, 3>> state_list;
        double kf_lifetime;
        virtual void onInit()
        {

            ros::NodeHandle &nh = getPrivateNodeHandle();
            nh.getParam("/debug", debug);
            nh.getParam("/CameraMount", camera_m);
            nh.getParam("/Camera2RgbExtincs", camera_ext);
            nh.getParam("/InflateSize", inflate_bb);
            nh.getParam("/MinDt", min_dt);
            nh.getParam("/StaticPclLength", pcl_sta_len);
            nh.getParam("/KFLifeTime", kf_lifetime);
            nh.getParam("/MaxVel", max_vel);
            nh.getParam("/MapResolution", resolution);
            //  inflate_bb = 0.4;
            //  min_dt = 0.2;
            //  kf_lifetime = 0.5;
            inflate_vector = {inflate_bb, inflate_bb, 0};
            this->bbox_size = {16, 16, 10, 3};
            camera_mt = {camera_m[0], camera_m[1], camera_m[2]};
            depth2rgb_extrinsics = {camera_ext[0], camera_ext[1], camera_ext[2]};
            Cam_mt << 0, 0, 1, -1, 0, 0, 0, -1, 0;
            //   pcl_sta_len = 500;
            sta_thres_init = 50;
            coe_atte_init = 80;
            //   resolution = 0.2;
            shrink_factor = 0.1;
            obj_timegap_max = 0.08; // max latency to receive obj message
            // voxel_id_list.clear();
            voxel_id_map.clear();
            voxel_id_map.reserve(pcl_sta_len + 10);
            //   max_vel = 3;
            //    fx = 343.4963684082031;
            //    fy = 343.4963684082031;
            //    cx = 320.0;
            //    cy = 180.0; // 0,4,2,5

            memset(local_map, 0, sizeof(local_map));
            memset(time_map, 0, sizeof(local_map));
            A.resize(states_n, states_n);
            C.resize(obsv_n, states_n);
            Q.resize(states_n, states_n);
            R.resize(obsv_n, obsv_n);
            P.resize(states_n, states_n);

            pcl_sub.subscribe(nh, "/filtered_RadiusOutlierRemoval", 1);
            imu_sub.subscribe(nh, "/mavros/imu/data", 50);
            odom_sub.subscribe(nh, "/vicon_imu_ekf_odom", 50);
            ApproSync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(30), pcl_sub, imu_sub, odom_sub);
            ApproSync_->registerCallback(boost::bind(&PIV::input_cb, this, _1, _2, _3));
            dynobj_sub = nh.subscribe<target_ros_msgs::BoundingBoxes>("/objects", 3, &PIV::obj_cb, this);

            // Create a ROS publisher for the output point cloud 输出
            pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/points_global_static", 3);
            obj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/dyn", 3);
            obj_state_pub = nh.advertise<obj_state_msgs::ObjectsStates>("/obj_states", 3);
            // Spin
            // ros::spin ();

            out1 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * bbox_size.prod());
            out2 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * bbox_size.prod());
            out3 = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * bbox_size.prod());
            int n[4] = {bbox_size(0), bbox_size(1), bbox_size(2), bbox_size(3)};

            // int* n = &bbox_size;
            p = fftw_plan_dft_r2c(4, (int *)&n, (double *)&extd_bb_1, out2, FFTW_MEASURE);

            p_inv = fftw_plan_dft_c2r(4, (int *)&n, out3, (double *)in3, FFTW_MEASURE);
            // cout<<"Start PIV nodelet!"<<endl;
            // last_kf_t = ros::Time::now();
            C << 1, 0, 0, 1;
            P << 2, 0, 0, 2;

            cam_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info", ros::Duration(2));
            cout << "wait camera intrinsics!" << endl;
            if (cam_info_msg)
            {
                cout << "Got camera intrinsics!" << endl;
                fx = cam_info_msg->K[0];
                fy = cam_info_msg->K[4];
                cx = cam_info_msg->K[2];
                cy = cam_info_msg->K[5];
            }
            cout << "Init PIV nodelet!" << endl;
        }

        void obj_state_pb()
        {
            obj_state_msgs::ObjectsStates states;

            states.header.frame_id = "map";
            states.header.stamp = ros::Time().fromSec(KF_list[0].last_update_t); // ros::Time::now(); // // time_list.back();

            for (auto &kf : KF_list)
            {
                obj_state_msgs::State state;
                state.position.x = kf.state()(0, 0);
                state.position.y = kf.state()(0, 1);
                state.position.z = kf.state()(0, 2);
                state.velocity.x = kf.state()(1, 0);
                state.velocity.y = kf.state()(1, 1);
                state.velocity.z = kf.state()(1, 2);
                state.size.x = kf.size(0);
                state.size.y = kf.size(1);
                state.size.z = kf.size(2);
                state.acceleration.x = 1.1;
                state.acceleration.y = kf.P(0,0);
                state.acceleration.z = kf.P(1,1);
                states.states.push_back(state);
            }
            obj_state_pub.publish(states);
        }
        void dyn_pb()
        {
            visualization_msgs::MarkerArray dyn;
            double life_t = 0.1;
            int id = 0;
            for (auto &kf : KF_list)
            {
                visualization_msgs::Marker dynobs;
                dynobs.header.frame_id = "map";
                dynobs.header.stamp = ros::Time::now();
                dynobs.type = visualization_msgs::Marker::CUBE;
                dynobs.pose.position.x = kf.state()(0, 0);
                dynobs.pose.position.y = kf.state()(0, 1);
                dynobs.pose.position.z = kf.state()(0, 2);
                dynobs.id = id++;
                dynobs.scale.x = kf.size(0);
                dynobs.scale.y = kf.size(1);
                dynobs.scale.z = kf.size(2);

                dynobs.color.a = 0.4;
                dynobs.color.r = 0.0;
                dynobs.color.g = 0.9;
                dynobs.color.b = 0.1;
                dynobs.pose.orientation.x = 0;
                dynobs.pose.orientation.y = 0;
                dynobs.pose.orientation.z = 0;
                dynobs.pose.orientation.w = 1.0;

                // dynbbox = Marker()
                // dynbbox.header.frame_id = "map"
                // dynbbox.header.stamp = rospy.Time.now()
                // dynbbox.type = Marker.CUBE
                // dynbbox.pose.position.x = c_dyn1[m][0]
                // dynbbox.pose.position.y = c_dyn1[m][1]

                // dynbbox.id=3*m+1
                // dynbbox.scale.x = obsd[m][0]+2*max_displ
                // dynbbox.scale.y = obsd[m][1]+2*max_displ
                // if c_dyn1[m][2] - 0.5*obsd[m][2] - max_displ <0:
                //     dynbbox.scale.z = 0.5*obsd[m][2]+max_displ+c_dyn1[m][2]
                //     dynbbox.pose.position.z = dynbbox.scale.z*0.5
                // else:
                //     dynbbox.pose.position.z = c_dyn1[m][2]
                //     dynbbox.scale.z = obsd[m][2]+2*max_displ

                // dynbbox.color.a = 0.1
                // dynbbox.color.r = 1.0
                // dynbbox.color.g = 0.6
                // dynbbox.color.b = 0.0

                // dynbbox.pose.orientation.x = 0
                // dynbbox.pose.orientation.y = 0
                // dynbbox.pose.orientation.z = 0
                // dynbbox.pose.orientation.w = 1.0

                visualization_msgs::Marker dynv;
                geometry_msgs::Point p1, p2;
                dynv.header.frame_id = "map";
                dynv.header.stamp = dynobs.header.stamp;
                dynv.type = visualization_msgs::Marker::ARROW;
                dynv.id = id++;
                p1.x = dynobs.pose.position.x;
                p1.y = dynobs.pose.position.y;
                p1.z = dynobs.pose.position.z;
                p2.x = p1.x + kf.state()(1, 0);
                p2.y = p1.y + kf.state()(1, 1);
                p2.z = p1.z + kf.state()(1, 2);
                dynv.points.push_back(p1);
                dynv.points.push_back(p2);
                dynv.pose.orientation.w = 1;
                dynv.scale.x = 0.2; // diameter of arrow shaft
                dynv.scale.y = 0.4; // diameter of arrow head
                dynv.scale.z = 0.6;
                dynv.color.a = 1; // transparency
                dynv.color.r = 0.8;
                dynv.color.g = 0.1;
                dynv.color.b = 0.1;

                visualization_msgs::Marker dynid;
                dynid.header.frame_id = "map";
                dynid.header.stamp = ros::Time::now();
                dynid.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                dynid.pose.position.x = kf.state()(0, 0);
                dynid.pose.position.y = kf.state()(0, 1);
                dynid.pose.position.z = kf.state()(0, 2) + 1;
                dynid.id = id++;
                dynid.pose.orientation.w = 1;
                dynid.scale.z = 0.5;
                dynid.text = to_string(kf.id);
                dynid.color.a = 1;
                dynid.color.r = 1.0;
                dynobs.lifetime = ros::Duration(life_t);
                dynv.lifetime = dynobs.lifetime;
                dynid.lifetime = dynobs.lifetime;
                // dynbbox.lifetime = rospy.Duration(life_t)
                dyn.markers.push_back(dynobs);
                dyn.markers.push_back(dynv);
                dyn.markers.push_back(dynid);
                // dyn.markers.append(dynbbox)
                obj_vis_pub.publish(dyn);
            }
        }

        void pcl_static_pb()
        {
            pcl_sta.header.frame_id = "map";
            pcl_conversions::toPCL(ros::Time::now(), pcl_sta.header.stamp);
            pcl_sta.height = 1;
            pcl_sta.width = pcl_sta.size();
            pcl_sta.is_dense = true;
            pcl_pub.publish(pcl_sta);
        }
        void
        obj_cb(const target_ros_msgs::BoundingBoxesConstPtr &obj_msg)
        {
            if (debug)
                cout << "obj received!" << endl;
            obj_shrink.clear();
            obj_ids.clear();
            obj = *obj_msg;
            obj_updated = true;
            obj_num = obj.bounding_boxes.size();
            for (auto &box : obj.bounding_boxes)
            {
                obj_ids.emplace_back(box.id);
                obj_shrink.emplace_back((box.xmax - box.xmin) * shrink_factor, (box.ymax - box.ymin) * shrink_factor);
            }
            ros::spinOnce();
        }

        int getObjRelaSeq(auto &point)
        {
            double img_x, img_y;
            img_x = (point.x + depth2rgb_extrinsics(0)) / (point.z + depth2rgb_extrinsics(2)) * fx + cx;
            img_y = (point.y + depth2rgb_extrinsics(1)) / (point.z + depth2rgb_extrinsics(2)) * fy + cy;
            // cout<<"img coord: "<<img_x<<"  "<<img_y<<endl;
            for (auto i = 0; i < obj.bounding_boxes.size(); i++)
            {
                if (img_x < obj.bounding_boxes[i].xmax - obj_shrink[i](0) && img_y < obj.bounding_boxes[i].ymax - obj_shrink[i](1) && img_x > obj.bounding_boxes[i].xmin + obj_shrink[i](0) && img_y > obj.bounding_boxes[i].ymin + obj_shrink[i](1))
                {
                    return i;
                }
            }
            return -1;
        }

        inline double get_Point2ObjCenter_dis(Vector3d &point, int obj_id)
        {
            double img_x, img_y;
            img_x = point(0) / point(2) * fx + cx;
            img_y = point(1) / point(2) * fy + cy;
            img_p1 = img_p2;
            img_p2 << img_x, img_y;
            double obj_cx = (obj.bounding_boxes[obj_id].xmax + obj.bounding_boxes[obj_id].xmin) * 0.5;
            double obj_cy = (obj.bounding_boxes[obj_id].ymax + obj.bounding_boxes[obj_id].ymin) * 0.5;
            double res = Eigen::Vector2d(img_x - obj_cx, img_y - obj_cy).norm();
            cout << "dis to obj center: " << res << endl;
            return res;
        }

        inline Eigen::Vector2d point2img(Vector3d &point)
        {
            double img_x, img_y;
            img_x = point(0) / point(2) * fx + cx;
            img_y = point(1) / point(2) * fy + cy;
            return Eigen::Vector2d(img_x, img_y);
        }

        inline Eigen::Vector2d obj_center(int obj_id)
        {
            double obj_cx = (obj.bounding_boxes[obj_id].xmax + obj.bounding_boxes[obj_id].xmin) * 0.5;
            double obj_cy = (obj.bounding_boxes[obj_id].ymax + obj.bounding_boxes[obj_id].ymin) * 0.5;
            return Eigen::Vector2d(obj_cx, obj_cy);
        }

        inline void point2index(std::vector<int> &point_index, Vector3d &point)
        {
            point_index = {int(point(0) / resolution) + map_o(0),
                           int(point(1) / resolution) + map_o(1),
                           int(point(2) / resolution) + map_o(2)};
        }
        inline void point2index(std::vector<int> &point_index, pcl::PointXYZRGB &point)
        {
            point_index = {int(point.x / resolution) + map_o(0),
                           int(point.y / resolution) + map_o(1),
                           int(point.z / resolution) + map_o(2)};
        }

        void
        input_cb(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                 const sensor_msgs::Imu::ConstPtr &imu_msg, const nav_msgs::Odometry::ConstPtr &odom_msg)
        {
            // read raw point cloud data
            // and convert to raw_cloud
            if (debug)
                cout << "pcl received! seq number: " << cloud_msg->header.seq << endl;
            chrono::high_resolution_clock::time_point tic1 = chrono::high_resolution_clock::now();
            PointCloud::Ptr cloud(new PointCloud);
            // auto cloud = make_shared<PointCloud>;
            // output = *cloud_msg;

            pcl::fromROSMsg(*cloud_msg, *cloud); //
                                                 //    {cout<<"convert!"<<endl;
                                                 // sensor_msgs::convertPointCloud2ToPointCloud(*msg,cloud);
            Rate_B1(0) = imu_msg->angular_velocity.x;
            Rate_B1(1) = imu_msg->angular_velocity.y;
            Rate_B1(2) = imu_msg->angular_velocity.z;
            Quat1.x() = odom_msg->pose.pose.orientation.x;
            Quat1.y() = odom_msg->pose.pose.orientation.y;
            Quat1.z() = odom_msg->pose.pose.orientation.z;
            Quat1.w() = odom_msg->pose.pose.orientation.w;
            Vel1 << odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z;
            Pos1(0) = odom_msg->pose.pose.position.x;
            Pos1(1) = odom_msg->pose.pose.position.y;
            Pos1(2) = odom_msg->pose.pose.position.z;
            Rota1 = Quaternion2Rota(Quat1.normalized());
            double time_odom2pcl = (odom_msg->header.stamp - cloud_msg->header.stamp).toSec();
            Euler = Quaternion2Euler(Quat1.normalized()) - Rota1 * Rate_B1 * time_odom2pcl;
            Pos = Pos1 - Vel1 * time_odom2pcl;
            Rota = Quaternion2Rota(Euler2Quaternion(Euler));
            // ori_var_list.emplace_back(Euler);
            ori_var_list.insert(ori_var_list.begin(), Euler);
            Eigen::Vector3d var_ori;
            double speed = Vel1.norm();
            sta_thres = sta_thres_init / (speed * speed + 1);
            coe_atte = coe_atte_init / (1 + speed * speed); // adap to speed. IF the drone flies fast, the latency to build the map is small
            coe_atte = coe_atte > 40 ? coe_atte : 40.0;
            if (ori_var_list.size() > 50)
            {
                // ori_var_list.resize(50);
                ori_var_list.pop_back();
                Eigen::MatrixXd ori_var_mat = Eigen::Map<Eigen::MatrixXd>(ori_var_list.front().data(), 3, ori_var_list.size());
                Eigen::MatrixXd ori_mean = ori_var_mat.colwise() - ori_var_mat.rowwise().mean();
                var_ori = (ori_mean.array() * ori_mean.array()).rowwise().sum() / (ori_var_list.size() - 1);
                // for (auto elem:ori_var_list)
                // cout<<elem<<endl;
                // cout<<"var_ori:\n"<<var_ori<<"\nlist size:\n"<<ori_var_list.size() <<"\n ori_var_mat:\n"<<ori_var_mat<<"\nEuler:\n"<<Euler<<endl;
            }
            // Eigen::MatrixXd ori_var_mat =  Map<Eigen::MatrixXd>(ori_var_list.front().data(),3,ori_var_list.size());

            Eigen::Vector3d acc_var = {imu_msg->linear_acceleration_covariance[0], imu_msg->linear_acceleration_covariance[4], imu_msg->linear_acceleration_covariance[8]};
            Eigen::Vector3d ang_vel_var = {imu_msg->angular_velocity_covariance[0], imu_msg->angular_velocity_covariance[4], imu_msg->angular_velocity_covariance[8]};

            if (ori_var_list.size() < 50) //((var_ori.array() > 1).any() || std::isnan(var_ori(0)) || std::isnan(var_ori(1)) || std::isnan(var_ori(2)))
                var_ori = (ang_vel_var * pow(time_odom2pcl, 2)).array() + 0.001;
            else
                var_ori = var_ori + ang_vel_var * pow(time_odom2pcl, 2);

            Eigen::Vector3d var_pos = acc_var * pow(time_odom2pcl, 4) / 4 + acc_var * pow(time_odom2pcl, 2);

            // cloud_msg->header.stamp = cloud_msg->header.stamp - ros::Duration(imu_delay);
            time_list.emplace(cloud_msg->header.stamp);
            pcl_list.emplace(cloud);
            Rota_list.emplace(Rota);
            Pos_list.emplace(Pos);
            double dt = (time_list.back() - time_list.front()).toSec();
            if (debug)
                cout << "dt: " << dt << "t gap between pcl and odom: " << time_odom2pcl << endl;

            double map_update_time = cloud_msg->header.stamp.toSec();

            // bool solid_before = false;
            if (debug)
                cout << "voxel_id_list size: " << voxel_id_list.size() << "  " << pcl_sta.points.size() << endl;
            // auto itr = voxel_id_map.cbegin();
            std::vector<int> index_out;
            for (int idd = 0; idd < pcl_sta.points.size(); idd++)
            {
                // if (local_map[index_out[0]][index_out[1]][index_out[2]] > sta_thres)
                //     solid_before = true;
                // auto index_out = itr->first;
                point2index(index_out, pcl_sta.points[idd]);
                if (local_map[index_out[0]][index_out[1]][index_out[2]] >= 0)
                {
                    local_map[index_out[0]][index_out[1]][index_out[2]] -= int((map_update_time - time_map[index_out[0]][index_out[1]][index_out[2]]) * coe_atte);
                    time_map[index_out[0]][index_out[1]][index_out[2]] = map_update_time;
                }
                // cout<<"idd: "<<idd<<endl;
                bool if_in_dynamic = false;
                Eigen::Vector3d sta_pt = {pcl_sta.points[idd].x, pcl_sta.points[idd].y, pcl_sta.points[idd].z};
                for (auto bbox : extended_3dbb)
                {
                    if ((sta_pt.array() > bbox.col(0).array()).all() && (sta_pt.array() < bbox.col(1).array()).all())
                    {
                        if_in_dynamic = true;
                        break;
                    }
                }
                if ((local_map[index_out[0]][index_out[1]][index_out[2]] < 0 || if_in_dynamic) && pcl_sta.points.size() > 0 && voxel_id_map.size() > 0)
                {
                    auto it2 = pcl_sta.points.erase(pcl_sta.points.begin() + idd);
                    voxel_id_map.erase(index_out);
                    idd--;
                    // itr--;
                }
                // else
                // {
                //     itr++;
                // }
            }
            if (debug)
                cout << "map erased! " << voxel_id_map.size() << "  " << pcl_sta.points.size() << endl;
            if (pcl_sta.points.size() == 0)
            {
                voxel_id_map.clear();
            }
            if (dt < 0)
            {
                time_list = {};
                pcl_list = {};
                Rota_list = {};
                Pos_list = {};
                memset(local_map, 0, sizeof(local_map));
                memset(time_map, 0, sizeof(time_map));
                return;
            }

            for (auto i = 0; i < KF_list.size(); i++)
            {
                if (KF_list[i].duration(cloud_msg->header.stamp.toSec()) > kf_lifetime)
                {
                    if (debug)
                        cout << "remove timeout KF: " << KF_list[i].duration(cloud_msg->header.stamp.toSec()) << endl;
                    auto iter = KF_list.erase(KF_list.begin() + i);
                    i--;
                }
            }

            int temp_ind, temp_amp;
            std::vector<int> voxel_index(3);
            vectors3d static_pcl;

            xyz_clusters1.clear();
            xyz_clusters2.clear();
            rgb_clusters1.clear();
            rgb_clusters2.clear();
            extended_3dbb.clear();
            xyz_clusters2_M.clear();
            extended_3dbb_map.clear();
            xyz_clusters1.resize(obj_num);
            xyz_clusters2.resize(obj_num);
            rgb_clusters1.resize(obj_num);
            rgb_clusters2.resize(obj_num);
            xyz_clusters2_M.resize(obj_num);
            extended_3dbb.resize(obj_num);
            extended_3dbb_map.resize(obj_num);
            // cout << "dynamic obj num: " << obj_num << endl;
            // compensate the UAV state

            if (dt > min_dt && pcl_list.back()->points.size() != 0 && pcl_list.front()->points.size() != 0)
            {
                int cnt = 0;
                bool obj_timeout = abs((time_list.back() - obj.header.stamp).toSec()) > obj_timegap_max;
                std::vector<int> cluster_begin_idx, cluster_size;
                std::vector<Eigen::Vector3d> cluster_1st;
                cluster_begin_idx.emplace_back(0);
                if (debug)
                {
                    cout << "PCL width: " << cloud->width << "  PCL size: " << cloud->points.size() << endl;
                    cout << "cluster number: " << cloud->points.back().x << "  " << cloud->points[cloud->width - 1].y << endl;
                    cout << "time gap to obj: " << abs((time_list.back() - obj.header.stamp).toSec()) << "  timeout? " << obj_timeout << endl;
                }
                for (int id = cloud->width - 1 - cloud->points.back().x; (cloud->points[id].y == 0 && cloud->points[id].z == 0); id++)
                {
                    cluster_1st.emplace_back(cloud->points[cluster_begin_idx.back()].x,
                                             cloud->points[cluster_begin_idx.back()].y, cloud->points[cluster_begin_idx.back()].z);
                    cluster_begin_idx.emplace_back(cluster_begin_idx.back() + cloud->points[id].x);
                    cluster_size.emplace_back(cloud->points[id].x);
                }

                std::vector<int> index(cluster_1st.size());
                iota(index.begin(), index.end(), 0);
                sort(index.begin(), index.end(),
                     [&](const int &a, const int &b)
                     {
                         return (cluster_1st[a](2) < cluster_1st[b](2));
                     });

                // for (auto cluster:xyz_clusters2)
                // {
                //     if (cluster.size()!=0 && (Eigen::Vector2d(img_x,img_y) - point2img(cluster.front())).norm()<80 && point.z > cluster.front()(2))
                //     return -1;
                // }
                int cluster_id = 0;
                int temp_size = 1;

                for (auto id : index) //(auto &point:cloud->points)
                {
                    // cout<<" cnt: "<<cnt<<"  "<<cloud->points.size();
                    // cout<<"depth: "<<point.z <<endl;
                    for (auto pt_id = cluster_begin_idx[id]; pt_id < cluster_begin_idx[id] + cluster_size[id]; pt_id++)
                    {
                        auto point = cloud->points[pt_id];
                        if (obj_timeout || !obj_updated || id == cluster_1st.size() - 1)
                            temp_ind = -1;
                        else if (pt_id == cluster_begin_idx[id])
                        {
                            temp_ind = getObjRelaSeq(point);
                            if (temp_ind >= 0)
                            {
                                for (auto id2 : index)
                                {
                                    if (id2 == id)
                                        break;
                                    // Eigen::Vector3d pt2 = {cloud->points[cluster_begin_idx[id2]].x, cloud->points[cluster_begin_idx[id2]].y,cloud->points[cluster_begin_idx[id2]].z};
                                    if ((point2img(cluster_1st[id2]) - point2img(cluster_1st[id])).norm() < 100) // depth of id2 always smaller (closer) than id
                                    {
                                        temp_ind = -1;
                                        // cout << "found occluded object!" << endl;
                                        break;
                                    }
                                }
                            }
                            if (temp_ind >= 0)
                                temp_size = xyz_clusters2[temp_ind].size();
                            // cluster_id++;
                        }
                        // cout<<"temp_ind: "<<temp_ind<<endl;
                        cnt++;
                        if (temp_ind < 0)
                        {
                            // pcl::PointXYZRGB pt;
                            // static_pcl.emplace_back(point.x, point.y, point.z);
                            // Eigen::MatrixXd static_pcl_M =  Map<Eigen::MatrixXd>(static_pcl.front().data(),3,static_pcl.size());
                            Eigen::Vector3d static_pt = (Rota * ((Cam_mt * Eigen::Vector3d(point.x, point.y, point.z)) + camera_mt)) + Pos;
                            // cout<<"static point in earth:\n "<<static_pt<<endl;
                            point2index(voxel_index, static_pt);
                            if (voxel_index[0] >= x_size || voxel_index[1] >= y_size || voxel_index[2] >= z_size || voxel_index[0] < 0 || voxel_index[1] < 0 || voxel_index[2] < 0)
                            {
                                // cout << "invalid voxel index in map:  [" << voxel_index[0] << ", " << voxel_index[1] << ", " << voxel_index[2] << "]" << endl;
                                continue;
                            }
                            // auto it = find(voxel_id_list.begin(), voxel_id_list.end(), voxel_index);
                            if (local_map[voxel_index[0]][voxel_index[1]][voxel_index[2]] <= 2 * sta_thres)
                            {
                                if (local_map[voxel_index[0]][voxel_index[1]][voxel_index[2]] > 0)
                                    local_map[voxel_index[0]][voxel_index[1]][voxel_index[2]] -= int((map_update_time - time_map[voxel_index[0]][voxel_index[1]][voxel_index[2]]) * coe_atte);
                                local_map[voxel_index[0]][voxel_index[1]][voxel_index[2]] += 10;
                                time_map[voxel_index[0]][voxel_index[1]][voxel_index[2]] = map_update_time;
                                // cout<<"map index score: "<<voxel_index[0]<<"  "<<voxel_index[1]<<"  "<<voxel_index[2]<<"  "<<local_map[voxel_index[0]][voxel_index[1]][voxel_index[2]]<<endl;
                            }

                            if (local_map[voxel_index[0]][voxel_index[1]][voxel_index[2]] > sta_thres && voxel_id_map.find(voxel_index) == voxel_id_map.end())
                            {
                                point.x = static_pt(0);
                                point.y = static_pt(1);
                                point.z = static_pt(2);
                                pcl_sta.points.insert(pcl_sta.points.begin(), point);
                                // voxel_id_list.emplace_front(voxel_index);
                                voxel_id_map[voxel_index] = true;
                                // cout<<"pcl_sta size: "<<pcl_sta.points.size()<<endl;
                                // if (voxel_id_map.size() > pcl_sta_len)
                                // {
                                //     // cout<<"old voxel id: "<<voxel_id_list[pcl_sta_len][0]<<" "<<voxel_id_list[pcl_sta_len][1]<<" "<<voxel_id_list[pcl_sta_len][2]<<endl;
                                //     cout << "map erased-0! " << voxel_id_map.size() << endl;
                                //     static_pt = {pcl_sta.points[pcl_sta_len].x, pcl_sta.points[pcl_sta_len].y, pcl_sta.points[pcl_sta_len].z};
                                //     point2index(voxel_index, static_pt);
                                //     auto end_itr = voxel_id_map.find(voxel_index);
                                //     if (end_itr == voxel_id_map.end())
                                //         cout << "the oldest point does not exist " << endl;
                                //     local_map[end_itr->first[0]][end_itr->first[1]][end_itr->first[2]] = sta_thres - 10; // sta_thres-1;
                                // }
                            }
                            //  else if (pcl_sta.points.size()>10)
                            //  {
                            //      if (it != voxel_id_list.end() && local_map[voxel_index[0]][voxel_index[1]][voxel_index[2]] <= sta_thres)
                            //      {
                            //          int id_pcl = distance(voxel_id_list.begin(),it);
                            //          auto it2 = pcl_sta.points.erase(pcl_sta.points.begin()+id_pcl);
                            //          auto it3 = voxel_id_list.erase(voxel_id_list.begin()+id_pcl);
                            //         //  cout<<"delete map point! "<<endl;
                            //      }
                            //  }
                        }
                        else
                        {
                            // xyz_clusters2[temp_ind].col(cnt)<<point.x,point.y,point.z;
                            // rgb_clusters2[temp_ind].col(cnt++)<<point.r,point.g,point.b;
                            // cout<<"depth: "<<point.z <<"  id: "<<temp_ind<<endl;

                            if (temp_size == 0 || abs(xyz_clusters2[temp_ind].front()(2) - point.z) < 0.3)
                            {
                                xyz_clusters2[temp_ind].emplace_back(point.x, point.y, point.z);
                                //   cout<<"this point: \n"<<xyz_clusters2[temp_ind].back()<<endl;
                                rgb_clusters2[temp_ind].emplace_back(point.r, point.g, point.b);
                            }

                            // if (xyz_clusters2[temp_ind].size()!=0  && cnt-1 == cluster_begin_idx[cluster_id] && point.z < xyz_clusters2[temp_ind].front()(2))
                            // {
                            //     xyz_clusters2[temp_ind].clear();
                            //     rgb_clusters2[temp_ind].clear();
                            // }
                            // if (cnt-1< cluster_begin_idx[cluster_id] + cluster_size[cluster_id])
                            // {
                            // xyz_clusters2[temp_ind].emplace_back(point.x,point.y,point.z);
                            // rgb_clusters2[temp_ind].emplace_back(point.r,point.g,point.b);
                            // }

                            // if (xyz_clusters2[temp_ind].size()==0 || abs(point.z -xyz_clusters2[temp_ind].front()(2)) <0.7)
                            // // {
                            // // Eigen::Vector3d temp_point = {point.x,point.y,point.z};

                            // // // if (get_Point2ObjCenter_dis(temp_point,temp_ind) < get_Point2ObjCenter_dis(xyz_clusters2[temp_ind].back(),temp_ind))
                            // // // {
                            // //     if (point.z -xyz_clusters2[temp_ind].back()(2) < 0)
                            // //     {
                            // //     // cout<<"img point gap: "<<abs(img_p2(0)-img_p1(0))<<endl;
                            // //     // cout<<"img point coord: \n"<<img_p2<<endl<<img_p1<<endl<<"depth: "<<point.z <<"  "<<xyz_clusters2[temp_ind].back()(2)<<endl;

                            // //     // if(abs(img_p2(0)-img_p1(0))<10) //if the current point and former point is in the same line in image
                            // //     // {
                            // //     xyz_clusters2[temp_ind].clear();
                            // //     rgb_clusters2[temp_ind].clear();
                            // //     // cout<<"depth: "<<point.z <<endl;
                            // // xyz_clusters2[temp_ind].emplace_back(point.x,point.y,point.z);
                            // // rgb_clusters2[temp_ind].emplace_back(point.r,point.g,point.b);

                            // //     }
                            // // // }
                            // // }
                            // // else
                            // {
                            // // cout<<"depth: "<<point.z <<"  "<<xyz_clusters2[temp_ind].back()(2)<<endl;
                            // xyz_clusters2[temp_ind].emplace_back(point.x,point.y,point.z);
                            // rgb_clusters2[temp_ind].emplace_back(point.r,point.g,point.b);

                            // }
                            // cout<<"update cluster2\n";
                        }
                    }
                }
                if (debug)
                    cout << "map update finished! " << endl;
                if (pcl_sta.points.size() > pcl_sta_len)
                {
                    for (auto jj = pcl_sta.points.size() - 1; jj >= pcl_sta_len; jj--)
                    {
                        // cout << "map erased-0! " << voxel_id_map.size() << endl;
                        Eigen::Vector3d static_pt = {pcl_sta.points[jj].x, pcl_sta.points[jj].y, pcl_sta.points[jj].z};
                        point2index(voxel_index, static_pt);
                        auto end_itr = voxel_id_map.find(voxel_index);
                        if (end_itr == voxel_id_map.end())
                            cout << "the oldest point does not exist " << static_pt << endl;
                        else
                        {
                            local_map[end_itr->first[0]][end_itr->first[1]][end_itr->first[2]] = sta_thres - 10;
                            voxel_id_map.erase(end_itr);
                        }
                        // pcl_sta.points.pop_back();
                        // cout << "map erased-1! " << voxel_id_map.size() << endl;
                    }
                    pcl_sta.points.resize(pcl_sta_len);
                    // voxel_id_list.resize(pcl_sta_len);
                    // voxel_id_map.erase(itr+pcl_sta_len,voxel_id_map.end());
                    // cout << "pcl_sta size: " << pcl_sta.points.size() << endl;
                }
                //   if (static_pcl.size()>0)
                //   {

                //   }
                int i = 0;
                bool dynamic = false;
                //   cout<<"loop end\n";
                for (auto &cluster_o : xyz_clusters2)
                {
                    // convert points from body to earth frame
                    //  cout<<"cluster_o size:  "<<cluster_o.size()<<endl;
                    if (cluster_o.size() > 0)
                    {
                        Eigen::MatrixXd cluster = Map<Eigen::MatrixXd>(cluster_o.front().data(), 3, cluster_o.size());

                        cluster = (Rota * ((Cam_mt * cluster).colwise() + camera_mt)).colwise() + Pos;
                        xyz_clusters2_M[i] = cluster;
                        //    cout<<"mk1: "<<cluster.col(0)<<endl;
                        extended_3dbb[i].col(0) = cluster.rowwise().minCoeff().array() - inflate_vector.array(); // get min value of each row
                        extended_3dbb[i].col(1) = cluster.rowwise().maxCoeff().array() + inflate_vector.array(); // get max value of each row
                        // extended_3dbb_map[i].col(0) = (extended_3dbb[i].col(0).array() / resolution).cast<int>() + map_o;
                        // extended_3dbb_map[i].col(1) = (extended_3dbb[i].col(1).array() / resolution).cast<int>() + map_o;
                        dynamic = true;
                    }
                    //    cout<<"i-1:  "<<i<<endl;
                    i++;
                }
                // cout << "loop end-1\n";
                if (dynamic)
                {

                    Eigen::Vector3d pt;
                    for (auto &point : pcl_list.front()->points) // point cloud frame 1
                    {
                        int bb_id;
                        pt = {point.x, point.y, point.z};
                        pt = Rota_list.front() * (Cam_mt * pt + camera_mt) + Pos_list.front(); // from B to E
                        for (bb_id = 0; bb_id < extended_3dbb.size(); bb_id++)
                        {
                            if (xyz_clusters2[bb_id].size() > 0 && pt(0) > extended_3dbb[bb_id].col(0)(0) && pt(1) > extended_3dbb[bb_id].col(0)(1) && pt(2) > extended_3dbb[bb_id].col(0)(2) && pt(0) < extended_3dbb[bb_id].col(1)(0) && pt(1) < extended_3dbb[bb_id].col(1)(1) && pt(2) < extended_3dbb[bb_id].col(1)(2))
                            {
                                xyz_clusters1[bb_id].emplace_back(pt);
                                rgb_clusters1[bb_id].emplace_back(point.r, point.g, point.b);
                                // break;
                            }
                        }
                    }
                    Eigen::Vector3d dynamic_vsize;
                    memset(extd_bb_1, 0, sizeof(extd_bb_1));
                    memset(extd_bb_2, 0, sizeof(extd_bb_2));
                    // rostime = ros::Time::now();
                    // kf_dt = (rostime- last_kf_t).toSec();
                    Eigen::Array3d size = bbox_size.head(3).array();

                    for (auto j = 0; j < xyz_clusters1.size(); j++)
                    {
                        if (debug)
                            cout << "size of cluser of frame 1 and 2: " << xyz_clusters1[j].size() << "  " << xyz_clusters2[j].size() << endl;
                        if (xyz_clusters1[j].size() > 5 && xyz_clusters2[j].size() > 5 && abs(1 - double(xyz_clusters1[j].size() / xyz_clusters2[j].size())) < 0.3)
                        {
                            dynamic_vsize = ((extended_3dbb[j].col(1) - extended_3dbb[j].col(0)).array() / size).matrix();
                            origin_v = (inflate_vector.array() / dynamic_vsize.array()).cast<int>();
                            // origin_v(2) = 0;
                            Eigen::Matrix<int, 3, Dynamic> ids_cluster1, ids_cluster2;
                            ids_cluster1.resize(3, xyz_clusters1[j].size());
                            ids_cluster2.resize(3, xyz_clusters2[j].size());
                            Eigen::MatrixXd cluster1j = Map<Eigen::MatrixXd>(xyz_clusters1[j].front().data(), 3, xyz_clusters1[j].size());
                            ids_cluster1.row(0) = ((cluster1j.row(0).array() - extended_3dbb[j](0, 0)) / dynamic_vsize(0)).cast<int>().matrix();
                            ids_cluster1.row(1) = ((cluster1j.row(1).array() - extended_3dbb[j](1, 0)) / dynamic_vsize(1)).cast<int>().matrix();
                            ids_cluster1.row(2) = ((cluster1j.row(2).array() - extended_3dbb[j](2, 0)) / dynamic_vsize(2)).cast<int>().matrix();
                            ids_cluster2.row(0) = ((xyz_clusters2_M[j].row(0).array() - extended_3dbb[j](0, 0)) / dynamic_vsize(0)).cast<int>().matrix();
                            ids_cluster2.row(1) = ((xyz_clusters2_M[j].row(1).array() - extended_3dbb[j](1, 0)) / dynamic_vsize(1)).cast<int>().matrix();
                            ids_cluster2.row(2) = ((xyz_clusters2_M[j].row(2).array() - extended_3dbb[j](2, 0)) / dynamic_vsize(2)).cast<int>().matrix();
                            // cout<<"size: "<<ids_cluster2.rows()<<"  "<<ids_cluster2.cols()<<"  "<<xyz_clusters2_M[j].row(0).size()<<"  "<<(((xyz_clusters2_M[j].row(0).array() - extended_3dbb[j](0,0))/dynamic_vsize(0)).cast<int>()).size()<<endl;
                            for (auto k = 0; k < ids_cluster1.cols(); k++)
                            {
                                // extd_bb_1[ids_cluster1(0)(k)][ids_cluster1(1)(k)][ids_cluster1(2)(k)][0] ++;
                                extd_bb_1[ids_cluster1(0, k)][ids_cluster1(1, k)][ids_cluster1(2, k)][0] += 1 + rgb_clusters1[j][k](0) / 255; // add the red value
                                extd_bb_1[ids_cluster1(0, k)][ids_cluster1(1, k)][ids_cluster1(2, k)][1] += 1 + rgb_clusters1[j][k](1) / 255; // add the green value
                                extd_bb_1[ids_cluster1(0, k)][ids_cluster1(1, k)][ids_cluster1(2, k)][2] += 1 + rgb_clusters1[j][k](2) / 255; // add the blue value

                                // extd_bb_1[ids_cluster1(0)(k)][ids_cluster1(1)(k)][ids_cluster1(2)(k)][1] /= extd_bb_1[ids_cluster1(0)(k)][ids_cluster1(1)(k)][ids_cluster1(2)(k)][0]
                            }
                            for (auto k = 0; k < ids_cluster2.cols(); k++)
                            {
                                extd_bb_2[ids_cluster2(0, k)][ids_cluster2(1, k)][ids_cluster2(2, k)][0] += 1 + rgb_clusters2[j][k](0) / 255; // add the red value
                                extd_bb_2[ids_cluster2(0, k)][ids_cluster2(1, k)][ids_cluster2(2, k)][1] += 1 + rgb_clusters2[j][k](1) / 255; // add the green value
                                extd_bb_2[ids_cluster2(0, k)][ids_cluster2(1, k)][ids_cluster2(2, k)][2] += 1 + rgb_clusters2[j][k](2) / 255; // add the blue value
                                // cout<<"rgb-2: "<< rgb_clusters2[j][k](0) << " " <<rgb_clusters2[j][k](1) <<" "<<rgb_clusters2[j][k](2)<<endl;
                            }

                            Eigen::Vector3d p_j = (extended_3dbb[j].col(1) + extended_3dbb[j].col(0)) / 2;

                            Eigen::Vector3d target_v = {0, 0, 0};
                            for (auto &kfi : KF_list)
                            {
                                if (((p_j.transpose() - kfi.forward_state(cloud_msg->header.stamp.toSec()).row(0)).head(2).norm() < 0.5))
                                {
                                    target_v = kfi.forward_state(cloud_msg->header.stamp.toSec()).row(1).transpose().array() * dt / dynamic_vsize.array();
                                    break;
                                }
                            }
                            Eigen::Vector3d v_j = getDisp(target_v, max_vel * dt / dynamic_vsize.array()).array() * dynamic_vsize.array() / dt;
                            Eigen::Vector3d s_j = (extended_3dbb[j].col(1) - extended_3dbb[j].col(0)) - 2 * inflate_vector;
                            s_j(0) = max(s_j(0), 0.5);
                            s_j(1) = max(s_j(1), 0.5);
                            s_j(2) = max(s_j(2), 1.6);
                            s_j(0) = min(s_j(0), 1.0);
                            s_j(1) = min(s_j(1), 1.0);
                            s_j(2) = min(s_j(2), 2.0);
                            // cout << "observed pos:\n"
                            //      << p_j << "\n vel:\n"
                            //      << v_j << "\nobstacle size: \n"
                            //      << s_j << endl;
                            if (v_j.norm() > 10)
                            {
                                cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                                continue;
                            }
                            Eigen::Matrix<double, 2, 3> state;
                            state.row(0) = p_j.transpose();
                            state.row(1) = v_j.transpose();
                            // state_list.emplace_back(state);
                            double var_p = 0.00375 * (p_j - Pos).squaredNorm();
                            double var_pos_pcl = (var_B2E((Rota.inverse() * (p_j - Pos)), Euler) * var_ori).mean();
                            var_p += (var_pos_pcl + var_pos.mean() + 0.1);
                            double var_v = 2 * var_p / pow(dt, 2) - 2 * var_p + pow(dynamic_vsize.mean(), 2) / 3;
                            R(0, 0) = var_p;
                            R(0, 1) = sqrt(var_p * var_v);
                            R(1, 0) = R(0, 1);
                            R(1, 1) = var_v;
                            // cout<<"R mat: \n"<<R<<"var_ori:\n"<<var_ori<<"var_pos_pcl:\n"<<var_pos_pcl<<"var_pos:\n"<<var_pos.mean()<<endl;
                            bool matched = false;
                            bool id_matched = false;
                            for (auto &kfi : KF_list)
                            {
                                if (obj_ids[j] == kfi.id)
                                    id_matched = true; 
                                if (((p_j.transpose() - kfi.forward_state(cloud_msg->header.stamp.toSec()).row(0)).head(2).norm() < 0.5 && (v_j.transpose() - kfi.forward_state(cloud_msg->header.stamp.toSec()).row(1)).head(2).norm() < 1.0) || (obj_ids[j] == kfi.id &&
                                 ((p_j.transpose() - kfi.forward_state(cloud_msg->header.stamp.toSec()).row(0)).head(2).norm() < 0.8 &&
                                (v_j.transpose() - kfi.forward_state(cloud_msg->header.stamp.toSec()).row(1)).head(2).norm() < 1.8))) //  &&
                                {
                                    kfi.update(cloud_msg->header.stamp.toSec() - imu_delay, R, state);
                                    kfi.id = obj_ids[j];
                                    // last_kf_t = ros::Time::now();
                                    matched = true;
                                    cout << "update KF with observe, id: " << kfi.id << endl;
                                    break;
                                }
                                //    int seq_id = std::find(obj_ids.begin(), obj_ids.end(), kfi.id);
                            }
                            if (!matched && v_j.norm() < max_vel && !id_matched)
                            {
                                KalmanFilter kf(A, C, Q, R, P, obj_ids[j]);
                                // kf.init(cloud_msg->header.stamp.toSec(),state);  //for real-time
                                kf.init(cloud_msg->header.stamp.toSec() - imu_delay, state, s_j); // for rosbag test
                                KF_list.emplace_back(kf);
                                // last_kf_t = ros::Time::now();
                                cout << "initialize KF: " << KF_list.size() << "\nstate:\n"
                                     << state << endl;
                            }
                        }
                    }
                }
                //    int kf_number =  KF_list.size();
            } // if dt>min_dt
            while ((time_list.back() - time_list.front()).toSec() > min_dt)
            {
                time_list.pop();
                pcl_list.pop();
                Pos_list.pop();
                Rota_list.pop();
            }
            // std::cout << "Output states:" << KF_list.size() << "\n";
            for (auto i = 0; i < KF_list.size() && debug; i++)
            {
                std::cout << "KF state size: \n"
                          << KF_list[i].state().rows() << " " << KF_list[i].state().cols() << "\n\n";
                std::cout << KF_list[i].state() << "\n\n";
            }
            if (KF_list.size() > 0)
            {
                for (auto &kfi : KF_list) // forward the KFs if no observation is available
                {
                    //    int seq_id = std::find(obj_ids.begin(), obj_ids.end(), kfi.id);
                    //    int id;
                    //    for (id =0; id<obj_ids.size();id++)
                    //    {
                    //        if (obj_ids[id] == kfi.id)
                    //        break;
                    //    }
                    // if (cloud_msg->header.stamp.toSec() - kfi.last_observe_t > 0.001) // (!(id<obj_ids.size()))
                    // {
                    // A << 1,kf_dt,0,1;
                    // last_kf_t = ros::Time::now();
                    kfi.update(ros::Time::now().toSec(), R, Eigen::MatrixXd::Zero(2, 3), false); // update the KF without observation
                    cout << "update KF with current time: " << kfi.id << "\nstate:\n"
                         << kfi.state() << endl;
                }
                dyn_pb();
                cout << "output time gap to now: " << (ros::Time::now() - cloud_msg->header.stamp).toSec() << endl;
                obj_state_pb();
            }

            chrono::high_resolution_clock::time_point toc1 = chrono::high_resolution_clock::now();
            double compTime = chrono::duration_cast<chrono::microseconds>(toc1 - tic1).count() * 1.0e-3;
            if (debug)
                std::cout << "time cost for one frame (ms)： " << compTime << std::endl;

            pcl_static_pb();
            ros::spinOnce();
        }
        Eigen::Vector3d getDisp(Eigen::Vector3d target_disp, Eigen::Vector3d max_disp) //(int in1, int in2)
        {
            // cout << "call fft!" << endl;
            double peaks[peak_num];
            memset(peaks, 0, sizeof(peaks));
            vector<Eigen::Vector3d> index(peak_num);

            chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();

            fftw_execute(p);
            fftw_execute_dft_r2c(p, (double *)&extd_bb_2, out1); // swap tenser 1 and 2, because the point cloud origin of cluster2 is known

            for (int i = 0; i < bbox_size.prod(); i++)
            {
                out3[i][0] = out1[i][0] * out2[i][0] + out1[i][1] * out2[i][1]; // real part: ac - bd, conjugate the out2(out1 and out2 has exchanged): d = -d,-> ac+bd
                out3[i][1] = out1[i][1] * out2[i][0] - out1[i][0] * out2[i][1]; // imaginary part, bc + ad,->bc-ad
                // printf("{%.2f ,  %.2f}", out3[i][0],out3[i][1]);
            }
            fftw_execute(p_inv);
            // double max = 0;
            // int ii,jj,kk;
            for (int i = 0; i < bbox_size(0); i++)
            {
                for (int j = 0; j < bbox_size(1); j++)
                {
                    for (int k = 0; k < 2 || (k > bbox_size(2) - 2 && k < bbox_size(2)); k++)
                    {
                        double value = 0;
                        for (int m = 0; m < bbox_size(3); m++)
                        {
                            value += in3[i][j][k][m];
                            // cout<<"value:  "<<value<<endl;
                        }
                        for (int id = 0; id < peak_num; id++)
                        {
                            if (value > peaks[id])
                            {
                                for (int id2 = peak_num - 1; id2 > id; id2--)
                                {
                                    peaks[id2] = peaks[id2 - 1];
                                    index[id2] = index[id2 - 1];
                                }
                                peaks[id] = value;
                                index[id] = Eigen::Vector3d(i, j, k);
                                //  cout<<"i,j,k:\n"<<index[id]<<endl;
                                break;
                            }
                        }
                        //    std::cout << in3[i][j][k][m] << "  ";
                    }
                }
            }
            if (debug)
                cout << "origin_v: \n"
                     << origin_v << endl;
            for (auto &ind : index)
            {
                if (debug)
                    cout << "raw peak index: \n"
                         << ind << endl;
                for (auto dim = 0; dim < 3; dim++)
                {
                    if (ind(dim) + origin_v[dim] >= bbox_size(dim) - 1)
                    {
                        ind(dim) = ind(dim) - bbox_size(dim);
                    }
                }
            }
            Eigen::Vector3d dp = {1000, 1000, 1000};
            bool leagal = false;
            for (auto &ind : index)
            {
                if (ind.norm() < max_disp.norm() && (target_disp.sum() == 0 || (ind - target_disp).norm() < 3))
                {
                    dp = ind;
                    leagal = true;
                    break;
                }
            }
            // Eigen::Vector3d dp = index[0].cast<double>();
            if (!leagal)
                return dp;
            if (debug)
                cout << "peak index: \n"
                     << dp << endl;
            int count = 1;
            for (auto i = 1; i < peak_num; i++)
            {
                if (((index[0] - index[i]).array().abs() < 2).all() && abs(peaks[0] - peaks[i]) / peaks[0] < 0.05)
                {
                    dp = dp + index[i];
                    count++;
                }
            }
            chrono::high_resolution_clock::time_point toc = chrono::high_resolution_clock::now();
            double compTime = chrono::duration_cast<chrono::microseconds>(toc - tic).count() * 1.0e-3;
            std::cout << "time cost for one PIV (ms)： " << compTime << std::endl;
            return dp / count; // inverse the displacement because input 1 and 2 are swapped
        }
    };
}
PLUGINLIB_EXPORT_CLASS(pcl_filter_ns::PIV, nodelet::Nodelet)

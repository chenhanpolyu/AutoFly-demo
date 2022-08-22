#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <chrono>
// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加引用

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <iostream>
#include <string>
#include "DBSCAN_kdtree.h"
namespace pcl_filter_ns
{
  using namespace std;
  class PclFilter : public nodelet::Nodelet
  {
  public:
    PclFilter() { ; }
    ~PclFilter() { ; }

  private:
    // 定义点云类型
    ros::NodeHandle nh;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<pcl::Normal> Normal;

    ros::Publisher pub;
    ros::Subscriber sub;
    string input;
    double cut_dis;
    double voxel_size;
    double n_r, r_cluster;
    int n_n, min_n_cluster;
    int MK;
    double stdthr;
    bool use_time;
    bool if_rgb;
    double delay = 0;
    int cluster_1st_ind = 0;
    // output = *cloud_msg;

    // create temp point cloud ptr
    //  PointCloud::Ptr    cloud_f  (new PointCloud);
    sensor_msgs::PointCloud2 output;

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sta;

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    DBSCANKdtreeCluster<pcl::PointXYZRGB> ec;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    // pcl::PCDWriter writer;

    virtual void onInit()
    {
      // Initialize ROS
      // ros::init (argc, argv, "chen_filter");
      ros::NodeHandle &nh = getPrivateNodeHandle();
      nh.getParam("input", input);
      nh.getParam("cut_dis", cut_dis);
      nh.getParam("voxel_size", voxel_size);
      nh.getParam("n_r", n_r);
      nh.getParam("n_n", n_n);
      nh.getParam("radius_cluster", r_cluster);
      nh.getParam("min_number_cluster", min_n_cluster);
      nh.getParam("MK", MK);
      nh.getParam("std", stdthr);
      nh.getParam("use_current_time", use_time);
      nh.getParam("if_rgb_pcl", if_rgb);
      nh.getParam("img_delay", delay);
      //  cout<<""
      // Create a ROS subscriber for the input point cloud 输入
      //  cout<<"input topic:"<<input<<endl<<cut_dis<<endl;
      sub = nh.subscribe<sensor_msgs::PointCloud2>(input, 3, &PclFilter::cloud_cb, this);

      // Create a ROS publisher for the output point cloud 输出
      pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/filtered_RadiusOutlierRemoval", 3);

      // Spin
      // ros::spin ();
      // cout << "Init pointcloud filter nodelet!-1" << endl;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE); //分割模型　平面模型
      seg.setMethodType(pcl::SAC_RANSAC);    //随机采样一致性　参数估计方法
      seg.setMaxIterations(50);              //最大的迭代的次数
      seg.setDistanceThreshold(0.15);        //设置符合模型的内点　阀值
      cout << "Init pointcloud filter nodelet!" << endl;
    }
    void
    cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
      // read raw point cloud data
      // and convert to raw_cloud
      PointCloud::Ptr raw_cloud{new PointCloud};
      PointCloud::Ptr cloud_filtered{new PointCloud};
      PointCloud::Ptr voxel_filtered{new PointCloud};
      PointCloud::Ptr r_filtered{new PointCloud};
      PointCloud::Ptr sta_filtered{new PointCloud};
      PointCloud::Ptr ground{new PointCloud};
      PointCloud::Ptr cloud_clustered{new PointCloud};
      pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
      pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients}; //系数
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree{new pcl::search::KdTree<pcl::PointXYZRGB>};

      chrono::high_resolution_clock::time_point tic1 = chrono::high_resolution_clock::now();
      output = *cloud_msg;
      if (if_rgb)
      {

        // int i=0, nr_points = (int) ;//下采样前点云数量

        // cout<<cloud_filtered->points.size ()<<endl;
        pcl::fromROSMsg(output, *raw_cloud); //

        //  raw_cloud->points.resize(10000);
        // Perform the actual filtering-1
        if (cut_dis < 0)
        {
          cloud_filtered = raw_cloud;
        }
        else
        {
          pass.setInputCloud(raw_cloud);
          pass.setFilterFieldName("z");
          pass.setFilterLimits(0, cut_dis);
          pass.filter(*cloud_filtered);
        }

        // Perform the actual filtering-2
        if (voxel_size < 0)
        {
          voxel_filtered = cloud_filtered;
        }
        else
        {

          sor.setInputCloud(cloud_filtered);
          sor.setLeafSize(voxel_size, voxel_size, voxel_size);
          sor.filter(*voxel_filtered);
        }

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (voxel_filtered);
        seg.segment (*inliers, *coefficients);//分割
        if (inliers->indices.size () != 0)
        {
          // std::cout << "Estimated a planar model for the given dataset." << inliers->indices.size() <<std::endl;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;//按索引提取点云
        extract.setInputCloud (voxel_filtered);
        extract.setIndices (inliers);//提取符合平面模型的内点
        // // 移去平面局内点，提取剩余点云
        extract.setNegative (false);
        extract.filter (*ground);
        extract.setNegative (true);
        extract.filter (*voxel_filtered);

        // = *cloud_f;//剩余点云
        }

        // Perform the actual filtering-3
        if (n_n < 0 || n_r < 0.0)
        {
          r_filtered = voxel_filtered;
        }
        else
        {

          outrem.setInputCloud(voxel_filtered);
          outrem.setRadiusSearch(n_r);
          outrem.setMinNeighborsInRadius(n_n);
          // cout<<"before radius filter"<<endl;
          outrem.filter(*r_filtered);
        }
        // cout<<"after radius filter"<<endl;
        if (MK < 0 || stdthr < 0)
        {
          sta_filtered = r_filtered;
        }
        else
        {

          sta.setInputCloud(r_filtered);
          sta.setMeanK(MK);
          sta.setStddevMulThresh(stdthr);
          sta.filter(*sta_filtered);
        }
        chrono::high_resolution_clock::time_point toc_bf = chrono::high_resolution_clock::now();
        chrono::high_resolution_clock::time_point toc_ext;
        if (r_cluster < 0 || min_n_cluster < 0 || sta_filtered->points.size() == 0)
        {
          cloud_clustered = sta_filtered;
        }
        else
        {
          // std::cout<< min_n_cluster<<"  "<<r_cluster<<std::endl;

          tree->setInputCloud(sta_filtered);

          // Use defaut Euclidean clustering
          //    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;// 欧式聚类对象
          //  ec.setClusterTolerance (r_cluster);                    // 设置近邻搜索的搜索半径
          //  ec.setMinClusterSize (min_n_cluster);                       // 设置一个聚类需要的最少的点数目
          //  ec.setMaxClusterSize (25000);                     // 设置一个聚类需要的最大点数目为25000
          //  ec.setSearchMethod (tree);                        // 设置点云的搜索机制
          //  ec.setInputCloud (sta_filtered);
          //  ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中

          // Use DBSCAN:
            // cout<<"dbscan pcl size: "<<sta_filtered->points.size()<<endl;
          std::vector<pcl::PointIndices> cluster_indices; // 点云团索引
          ec.setCorePointMinPts(min_n_cluster);
          ec.setClusterTolerance(r_cluster);
          ec.setMinClusterSize(min_n_cluster + 10);
          ec.setMaxClusterSize(25000);
          ec.setInputCloud(sta_filtered);
          // ec.setSearchMethod(tree);
          // ec.extract(cluster_indices);
          ec.setSearchMethod();
          ec.extractNano(cluster_indices);

          //迭代访问点云索引cluster_indices,直到分割处所有聚类
          int pt_sum = 0;
          toc_ext = chrono::high_resolution_clock::now();
          // std::cout<<"clusters num: " <<cluster_indices.size()<<std::endl;

          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
          {

            double x_sum = 0, y_sum = 0, z_sum = 0;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
              cloud_clustered->points.push_back(sta_filtered->points[*pit]); //获取每一个点云团　的　点
              x_sum += sta_filtered->points[*pit].x;
              y_sum += sta_filtered->points[*pit].y;
              z_sum += sta_filtered->points[*pit].z;
              if (pit == it->indices.begin())
              {
                cluster_1st_ind = cloud_clustered->points.size() - 1;
                cloud_clustered->points.back().r = 255;
                cloud_clustered->points.back().g = 0;
                cloud_clustered->points.back().b = 0;
              }
              // std::cout<< sta_filtered->points[*pit].z<<std::endl;
            }
            cloud_clustered->points[cluster_1st_ind].x = x_sum / it->indices.size();
            cloud_clustered->points[cluster_1st_ind].y = y_sum / it->indices.size();
            cloud_clustered->points[cluster_1st_ind].z = z_sum / it->indices.size();
            // std::cout<<"one cluster end!"<<std::endl;
          }
          // std::cout<<"one frame end!"<<std::endl;
          // for (auto &pt:ground->points)
          // {
          //  pt.r = 0; pt.g = 0; pt.b = 255; 
          // }
          // ground->points.front().r = 0; ground->points.front().g = 0; ground->points.front().b = 255;
          cloud_clustered->points.insert(cloud_clustered->points.end(), ground->points.begin(),ground->points.end());
          pcl::PointXYZRGB pt;
          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
          {
            pt.x = it->indices.size();
            pt.y = 0;
            pt.z = 0;
            // pt_sum += it->indices.size();
            cloud_clustered->points.push_back(pt);
          }
          //  std::cout<<"ground pcl size: "<<ground->points.size()<<std::endl;
            pt.x = ground->points.size();
            pt.y = 0;
            pt.z = 0;
            cloud_clustered->points.push_back(pt);
          pt.x = cluster_indices.size()+1;
          pt.y = 1000;
          pt.z = 1000;
          cloud_clustered->points.push_back(pt);
          // Convert to ROS data type
          //    sensor_msgs::PointCloud2 cloud_pt;
          //    pcl_conversions::moveFromPCL(r_filtered, cloud_pt);
          //      pcl_conversions::toROSMsg(r_filtered, cloud_pt)
          //    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
          //    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
          // Publish the data
          cloud_clustered->header.frame_id = cloud_msg->header.frame_id;

          // cloud_clustered->header.stamp = cloud_msg->header.stamp - ros::Duration(delay);

          cloud_clustered->header.seq = cloud_msg->header.seq;
          cloud_clustered->width = cloud_clustered->points.size();
          cloud_clustered->height = 1;
        }
        if (use_time)
        {
          pcl_conversions::toPCL(ros::Time::now(), cloud_clustered->header.stamp);
        }
        else
        {
          pcl_conversions::toPCL(cloud_msg->header.stamp - ros::Duration(delay), cloud_clustered->header.stamp);
        }
        // cout<<"pcl filter pub timestamp gap: "<<(cloud_clustered->header.stamp - pcl_conversions::toPCL(cloud_msg->header.stamp))<<endl;
        chrono::high_resolution_clock::time_point toc1 = chrono::high_resolution_clock::now();
        double compTime = chrono::duration_cast<chrono::microseconds>(toc1 - tic1).count() * 1.0e-3;
        double compTime_bf = chrono::duration_cast<chrono::microseconds>(toc_bf - tic1).count() * 1.0e-3;
        double compTime_ext = chrono::duration_cast<chrono::microseconds>(toc_ext - tic1).count() * 1.0e-3;
        if (compTime > 15)
          cout << "filter time cost longer than 15 (ms):  " << compTime << " before dbscan: " << compTime_bf << " after clustering: " << compTime_ext << endl;
        pub.publish(cloud_clustered);
      }

      else
      {

        Cloud::Ptr raw_cloud(new Cloud);
        // output = *cloud_msg;
        pcl::fromROSMsg(output, *raw_cloud); //

        // create temp point cloud ptr
        //  Cloud::Ptr    cloud_f  (new Cloud);
        Cloud::Ptr cloud_filtered(new Cloud);
        Cloud::Ptr voxel_filtered(new Cloud);
        Cloud::Ptr r_filtered(new Cloud);
        Cloud::Ptr sta_filtered(new Cloud);
        Cloud::Ptr cloud_clustered(new Cloud);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sta;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); //系数
        // pcl::Cloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::Cloud<pcl::PointXYZ> ());
        // pcl::PCDWriter writer;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE); //分割模型　平面模型
        seg.setMethodType(pcl::SAC_RANSAC);    //随机采样一致性　参数估计方法
        seg.setMaxIterations(100);             //最大的迭代的次数
        seg.setDistanceThreshold(0.05);        //设置符合模型的内点　阀值

        // int i=0, nr_points = (int) cloud_filtered->points.size ();//下采样前点云数量

        // Perform the actual filtering-1
        if (cut_dis < 0)
        {
          cloud_filtered = raw_cloud;
        }
        else
        {
          pass.setInputCloud(raw_cloud);
          pass.setFilterFieldName("z");
          pass.setFilterLimits(0, cut_dis);
          pass.filter(*cloud_filtered);
        }

        // Perform the actual filtering-2
        if (voxel_size < 0)
        {
          voxel_filtered = cloud_filtered;
        }
        else
        {

          sor.setInputCloud(cloud_filtered);
          sor.setLeafSize(voxel_size, voxel_size, voxel_size);
          sor.filter(*voxel_filtered);
        }

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(voxel_filtered);
        seg.segment(*inliers, *coefficients); //分割
        if (inliers->indices.size() != 0)
        {
          // std::cout << "Estimated a planar model for the given dataset." << std::endl;
          pcl::ExtractIndices<pcl::PointXYZ> extract; //按索引提取点云
          extract.setInputCloud(voxel_filtered);
          extract.setIndices(inliers); //提取符合平面模型的内点
          // extract.setNegative (false);
          // // 平面模型内点
          // extract.filter (*cloud_plane);
          // std::cout << "Cloud representing the planar component: " <<
          //   cloud_plane->points.size () <<
          //   " data points." << std::endl;
          // // 移去平面局内点，提取剩余点云
          extract.setNegative(true);
          extract.filter(*voxel_filtered);
          // = *cloud_f;//剩余点云
        }

        // Perform the actual filtering-3
        if (n_n < 0 || n_r < 0.0)
        {
          r_filtered = voxel_filtered;
        }
        else
        {

          outrem.setInputCloud(voxel_filtered);
          outrem.setRadiusSearch(n_r);
          outrem.setMinNeighborsInRadius(n_n);
          // cout<<"before radius filter"<<endl;
          outrem.filter(*r_filtered);
        }
        // cout<<"after radius filter"<<endl;
        if (MK < 0 || stdthr < 0)
        {
          sta_filtered = r_filtered;
        }
        else
        {

          sta.setInputCloud(r_filtered);
          sta.setMeanK(MK);
          sta.setStddevMulThresh(stdthr);
          sta.filter(*sta_filtered);
        }

        if (r_cluster < 0 || min_n_cluster < 0 || sta_filtered->points.size() == 0)
        {
          cloud_clustered = sta_filtered;
        }
        else
        {
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
          std::vector<pcl::PointIndices> cluster_indices; // 点云团索引
          tree->setInputCloud(sta_filtered);

          // Use defaut Euclidean clustering
          //    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;// 欧式聚类对象
          //  ec.setClusterTolerance (r_cluster);                    // 设置近邻搜索的搜索半径
          //  ec.setMinClusterSize (min_n_cluster);                       // 设置一个聚类需要的最少的点数目
          //  ec.setMaxClusterSize (25000);                     // 设置一个聚类需要的最大点数目为25000
          //  ec.setSearchMethod (tree);                        // 设置点云的搜索机制
          //  ec.setInputCloud (sta_filtered);
          //  ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中

          // Use DBSCAN:
          DBSCANKdtreeCluster<pcl::PointXYZ> ec;
          ec.setCorePointMinPts(min_n_cluster);
          ec.setClusterTolerance(r_cluster);
          ec.setMinClusterSize(min_n_cluster + 10);
          ec.setMaxClusterSize(25000);
          ec.setInputCloud(sta_filtered);

          // ec.setSearchMethod(tree);
          // ec.extract(cluster_indices);
          ec.setSearchMethod();
          ec.extractNano(cluster_indices);

          //迭代访问点云索引cluster_indices,直到分割处所有聚类
          int pt_sum = 0;
          // std::cout<< cluster_indices.size()<<std::endl;
          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
          {
            // int cluster_1st_ind;
            double x_sum = 0, y_sum = 0, z_sum = 0;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
              cloud_clustered->points.push_back(sta_filtered->points[*pit]); //获取每一个点云团　的　点
              x_sum += sta_filtered->points[*pit].x;
              y_sum += sta_filtered->points[*pit].y;
              z_sum += sta_filtered->points[*pit].z;
              if (pit == it->indices.begin())
              {
                cluster_1st_ind = cloud_clustered->points.size() - 1;
                // cloud_clustered->points.back().r = 255;cloud_clustered->points.back().g = 0;cloud_clustered->points.back().b = 0;
              }
              // std::cout<< sta_filtered->points[*pit].z<<std::endl;
            }
            cloud_clustered->points[cluster_1st_ind].x = x_sum / it->indices.size();
            cloud_clustered->points[cluster_1st_ind].y = y_sum / it->indices.size();
            cloud_clustered->points[cluster_1st_ind].z = z_sum / it->indices.size();
            // std::cout<<"one cluster end!"<<std::endl;
          }

          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
          {
            pcl::PointXYZ pt;
            pt.x = it->indices.size();
            pt.y = 0;
            pt.z = 0;
            pt_sum += it->indices.size();
            cloud_clustered->points.push_back(pt);
          }
          // Convert to ROS data type
          //    sensor_msgs::Cloud2 cloud_pt;
          //    pcl_conversions::moveFromPCL(r_filtered, cloud_pt);
          //      pcl_conversions::toROSMsg(r_filtered, cloud_pt)
          //    pcl::Cloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::Cloud<pcl::PointXYZ>);
          //    pcl::fromPCLCloud2(pcl_pc2,*temp_cloud);
          // Publish the data
          cloud_clustered->header.frame_id = cloud_msg->header.frame_id;

          cloud_clustered->header.seq = cloud_msg->header.seq;
          cloud_clustered->width = cloud_clustered->points.size();
          cloud_clustered->height = 1;
        }
        if (use_time)
        {
          pcl_conversions::toPCL(ros::Time::now(), cloud_clustered->header.stamp);
        }
        else
        {
          pcl_conversions::toPCL(cloud_msg->header.stamp - ros::Duration(delay), cloud_clustered->header.stamp);
        }
        pub.publish(cloud_clustered);
      }
    }
  };
}
PLUGINLIB_EXPORT_CLASS(pcl_filter_ns::PclFilter, nodelet::Nodelet)

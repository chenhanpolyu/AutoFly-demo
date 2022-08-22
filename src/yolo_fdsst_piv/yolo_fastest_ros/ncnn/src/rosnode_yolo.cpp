#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "yolo-fastestv2.h"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <algorithm>
#include <iomanip>
#include <numeric>
//ROS
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

//OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
// #include<opencv2/opencv.hpp>
// #include </home/chen/opencv_build/opencv_contrib/modules/tracking/include/opencv2/tracking/tracking.hpp>
// #include "opencv2/tracking/tracking.hpp"

#include <target_ros_msgs/BoundingBox.h>
#include <target_ros_msgs/BoundingBoxes.h>
#include <target_ros_msgs/CheckForObjectsAction.h>
#include <target_ros_msgs/ObjectCount.h>

//fDSST tracker
#include<string>
#include<fstream>
#include<time.h>
#include "include/fdssttracker.hpp"

// static const char* class_names[] = {
//   "sports_ball"
// };




namespace yolo_ns
{
  static const char* class_names[] = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush"};
class YoloDetector : public nodelet::Nodelet
{
// using namespace cv;

  // YoloDetector() : it_(nh_)
  // {
  //   //read yaml parameters
  //   std::string cameraTopicName, depthTopicName;
  //   nh_.getParam("subscribers/camera_reading/img_topic", cameraTopicName);
  //   nh_.getParam("subscribers/camera_reading/depth_topic", depthTopicName);

  //   std::string ModelPath, ParaPath;
  //   nh_.getParam("yolo/model_reading/model_path", ModelPath);
  //   nh_.getParam("yolo/model_reading/para_path", ParaPath);
    
  //   nh_.getParam("yolo/interested_objects_id", interested_obj);
  //   const char *modelpath = ModelPath.data();
  //   const char *parapath = ParaPath.data();
  //   api.loadModel(parapath, modelpath);

  //   bool gpu;
  //   int input_size;
  //   nh_.getParam("yolo/detection_parameters/use_gpu", gpu);
  //   nh_.getParam("yolo/detection_parameters/input_size", input_size);
  //   float thresh1;
  //   float thresh2;
  //   float h2wratio;
  //   nh_.getParam("yolo/detection_parameters/nmsThresh", thresh1);
  //   nh_.getParam("yolo/detection_parameters/confThresh", thresh2);
  //   nh_.getParam("yolo/detection_parameters/relaxTime", relax_time);
  //   nh_.getParam("yolo/detection_parameters/recoverTime", recover_time);
  //   nh_.getParam("yolo/detection_parameters/h2wRatio", h2wratio);
  //   nh_.getParam("yolo/detection_parameters/if_debug", if_debug);
  //   api.loadParams(gpu, input_size, thresh1, thresh2, h2wratio);
  //   nh_.getParam("track/track_confidence", track_conf);
  //   nh_.getParam("track/init_box_scale", scale);
  //   nh_.getParam("track/if_multi_scale", MULTISCALE);
  //    nh_.getParam("track/iou_threshold",iou_thre);
    
  //   // Subscrive to input video feed and publish output video feed
  //   image_sub_ = it_.subscribe(cameraTopicName, 1, &YoloDetector::imageCb, this);
  //   depth_sub_ = it_.subscribe(depthTopicName, 1, &YoloDetector::depthCb, this);
  //   image_pub_ = it_.advertise("/image_converter/output_video", 1);
  //   objects_pub_ = nh_.advertise<target_ros_msgs::BoundingBoxes>("/objects",10);
    
  //   // cv::namedWindow(OPENCV_WINDOW,0);
  // }

private:
  // static const char* class_names[];
  ros::NodeHandle nh_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;
  // ros::Subscriber image_sub_;
  // ros::Subscriber depth_sub_;
  // ros::Publisher image_pub_;
  ros::Publisher objects_pub_;
  
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv::Mat depthImgCopy_;
    bool if_depth = false;
    double last_detect_time=0;
    ros::Time last_pop_time = ros::Time::now();
    double relax_time,recover_time;
    bool HOG = true;
    bool FIXEDWINDOW = true;
    bool MULTISCALE =true;
    bool SILENT = true;
    bool LAB = false;
    cv::Rect t_bbox;
    double track_conf;
    double scale;
    double rgb_time_ = 0, depth_time_ = 0;
    bool if_debug = true;
    int matched_track_id,matched_track_id_fail;
    double max_iou;
    int tracker_id = 0;
    float iou_thre = 0.45;
//   static const char* class_names;
  yoloFastestv2 api;
  double  depths[10] = {0};
  // memset(depths, 0, 10);
  // std::string OPENCV_WINDOW="tracking";
  std::vector<TargetBox> boxes;
  std::vector<cv::Rect> boxes_track,boxes_trackfail;
  std::vector<int> boxes_cate_id;
  std::vector<FDSSTTracker> trackers;
  std::deque<FDSSTTracker> trackers_fail;
  std::deque<int> fail_num;
  std::vector<int> fail_id;
  std::vector<int> interested_obj;
  float color_vec[8][3] = {{1,0.5,0.5},{0.5,1,0.5},{1,1,0},{1,0,1},{0,1,1},{1,0,0},{0,1,0},{0,0,1}};
  cv::Mat dst;

  public:
 YoloDetector():it_(nh_) {;}
  ~YoloDetector() {;}

  private:
  virtual void onInit()
  {
  std::cout<<"Start object detector thread"<<std::endl;

    //ros::NodeHandle& nh_ = getMTPrivateNodeHandle();
      // ros::NodeHandle nh_(getMTPrivateNodeHandle());
      ros::NodeHandle& nh_= getPrivateNodeHandle();

   //read yaml parameters
    std::string cameraTopicName, depthTopicName;
    nh_.getParam("/subscribers/camera_reading/img_topic", cameraTopicName);
    nh_.getParam("/subscribers/camera_reading/depth_topic", depthTopicName);
    
    std::string ModelPath, ParaPath;
    nh_.getParam("/yolo/model_reading/model_path", ModelPath);
    nh_.getParam("/yolo/model_reading/para_path", ParaPath);
    
    nh_.getParam("/yolo/interested_objects_id", interested_obj);
    const char *modelpath = ModelPath.data();
    const char *parapath = ParaPath.data();
    // std::cout<<"path:  "<<ModelPath<<std::endl;
    api.loadModel(parapath, modelpath);

    bool gpu;
    int input_size;
    nh_.getParam("/yolo/detection_parameters/use_gpu", gpu);
    nh_.getParam("/yolo/detection_parameters/input_size", input_size);
    float thresh1;
    float thresh2;
    float h2wratio;
    nh_.getParam("/yolo/detection_parameters/nmsThresh", thresh1);
    nh_.getParam("/yolo/detection_parameters/confThresh", thresh2);
    nh_.getParam("/yolo/detection_parameters/relaxTime", relax_time);
    nh_.getParam("/yolo/detection_parameters/recoverTime", recover_time);
    nh_.getParam("/yolo/detection_parameters/h2wRatio", h2wratio);
    nh_.getParam("/yolo/detection_parameters/if_debug", if_debug);
    api.loadParams(gpu, input_size, thresh1, thresh2, h2wratio);
    nh_.getParam("/track/track_confidence", track_conf);
    nh_.getParam("/track/init_box_scale", scale);
    nh_.getParam("/track/if_multi_scale", MULTISCALE);
     nh_.getParam("/track/iou_threshold",iou_thre);
    
    // Subscrive to input video feed and publish output video feed
    // std::cout<<"sub:\n"<<cameraTopicName<< "\n"<<depthTopicName<<std::endl;
    // std::cout<<"Cate:\n"<<class_names[3]<<std::endl;
    image_sub_ = it_.subscribe(cameraTopicName, 1, &YoloDetector::imageCb, this);
    depth_sub_ = it_.subscribe(depthTopicName, 1, &YoloDetector::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    objects_pub_ = nh_.advertise<target_ros_msgs::BoundingBoxes>("/objects",10);
    
    // cv::namedWindow(OPENCV_WINDOW,0);
    
  }

  void get_depth ()
  {
  //  depths.clear();
   ushort dp;
   for (size_t i = 0; i < boxes_track.size(); i++) {
    // std::cout<< boxes_track[i].x<<"\n"<< boxes_track[i].y<< "\n"<<boxes_track[i].x + boxes_track[i].width <<"\n"<< boxes_track[i].y+ boxes_track[i].height <<std::endl;
    cv::Mat reg = cv_depth_ptr->image(boxes_track[i]);
    if(if_debug) std::cout << "time gap (pub depth):"<< (depth_time_ - rgb_time_)<< std::endl;
    int count = 0;
    double depth = 1e4;
    // std::cout<< reg.cols<< " "<<reg.rows<<std::endl;
    for (int ii =  reg.cols/4; ii <  reg.cols*3/4; ii++ ) 
        for (int jj =  reg.rows/4; jj < reg.rows*3/4; jj++) {
            dp = reg.ptr<ushort>(jj)[ii];
            if (dp > 0 && dp<1e4 && dp<depth) {
                count ++;  
                depth = dp;   // Do your operations
              //  std::cout<<dp*1e-3<<"---"<<count<<" "<<jj<<" "<<ii<<std::endl;
            }
        }
    depths[i] = depth;
    if(if_debug) std::cout<<"depth:"<<depths[i]<<std::endl;
    }
  }

  int clip (int x, int x_min, int x_max)
  {
    if (x<x_min)
    {x=x_min;}
    else if (x>x_max)
    {x=x_max;}
    return x;
  }

  void pub_objects (const sensor_msgs::ImageConstPtr& msg)
  {
      target_ros_msgs::BoundingBoxes bboxes_track;
      for (size_t i = 0; i < boxes_track.size(); i++)
      {
         target_ros_msgs::BoundingBox bbox;
        //  bbox.probability =  boxes_track[i].score;
         bbox.xmin = boxes_track[i].x;
         bbox.ymin = boxes_track[i].y;
         bbox.xmax = boxes_track[i].x + boxes_track[i].width;
         bbox.ymax = boxes_track[i].y + boxes_track[i].height;
         bbox.distance = depths[i];
         bbox.id = i;
         bbox.Class = class_names[boxes_cate_id[i]];
         bboxes_track.bounding_boxes.push_back(bbox);
      }
      bboxes_track.header = msg->header;
      bboxes_track.image_header = msg->header;
      objects_pub_.publish(bboxes_track);
  }
  float intersection_area(const TargetBox &a, const cv::Rect &bb)
{
     TargetBox b;
     b.x1 = bb.x; b.y1 = bb.y; b.x2 = bb.x+bb.width; b.y2 = bb.y+bb.height;
    if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1)
    {
        // no intersection
        return 0.f;
    }

    float inter_width = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
    float inter_height = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);
    // std::cout << "inter_area:  "<<inter_width * inter_height<<std::endl;
    return inter_width * inter_height;
}

float getIOU (TargetBox &target)
{ //get the max IOU with the existed tracked boxes
  float iou=0.01;
  float tmp_iou,in_area;
  for (int i = 0;i<boxes_track.size(); i++)
  { in_area = intersection_area(target, boxes_track[i]);
    tmp_iou =in_area/(target.area() + boxes_track[i].width*boxes_track[i].height - in_area);
    // iou = std::max(iou,tmp_iou );
    if (iou< tmp_iou)
    {
    iou = tmp_iou;
    matched_track_id = i;
    }
  }
  std::cout << "iou:  "<<iou<<std::endl;
  return iou;
}

float getIOU_fail (TargetBox &target)
{ //get the max IOU with the existed tracked boxes
  float iou=0.01;
  float tmp_iou,in_area;
  for (int i = 0;i<trackers_fail.size(); i++)
  { in_area = intersection_area(target, trackers_fail[i].Roi);
    tmp_iou =in_area/(target.area() + trackers_fail[i].Roi.width*trackers_fail[i].Roi.height - in_area);
    // iou = std::max(iou,tmp_iou );
    if (iou< tmp_iou)
    {
    iou = tmp_iou;
    matched_track_id_fail = i;
    }
  }
  std::cout << "iou with failed trackers:  "<<iou<<std::endl;
  return iou;
}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  { 
    // std::cout << "Cate: "<<class_names[3]<<std::endl;
    rgb_time_ = msg->header.stamp.toSec();
    while( (rgb_time_ - depth_time_) > 0.004 && (rgb_time_ - depth_time_) < 1) {sleep(0.001); ros::spinOnce();}
    if(if_debug) std::cout << "time gap (rgb):"<< (depth_time_ - rgb_time_)<< std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // std::cout<<"raw img type: "<<cv_ptr->image.type()<<std::endl;
      cv::cvtColor(cv_ptr->image, dst, cv::COLOR_BGR2GRAY);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // clear screen
    if(if_debug)
    {printf("\033[2J");
    printf("\033[1;1H");}
    bool track_fail = false;
    fail_id.clear();
    boxes.clear();
     if(if_debug) std::cout << "trackers size:  " << boxes_track.size()<< " "<<ros::Time::now().toSec()<<std::endl;
    int fail_num_frame =0;
    for (int j = 0; j<trackers.size();j++ )
    {
      t_bbox = trackers[j].update(dst,track_conf); //confidence
      t_bbox.x = clip( t_bbox.x,0,  dst.cols );
      t_bbox.y = clip( t_bbox.y, 0,  dst.rows );
      t_bbox.height = clip( t_bbox.height, 1,  dst.rows - t_bbox.y-2 );
      t_bbox.width = clip( t_bbox.width, 0,  dst.cols - t_bbox.x-2);
      
      if (!trackers[j].suc) 
      { if(if_debug) std::cout << "track fail!" << std::endl;
         track_fail = true;
        //  fail_id.emplace_back(j);
         trackers_fail.push_back(trackers[j]);
         auto iter = trackers.erase(trackers.begin() + j);
         auto iter1 = boxes_track.erase(boxes_track.begin() + j);
         auto iter2 =  boxes_cate_id.erase( boxes_cate_id.begin()+j);
         fail_num_frame++;
         j--;
      }
      else
      {
      if(if_debug) std::cout << "tracked!" << std::endl;
      boxes_track[j] = t_bbox;}

    }
    fail_num.push_back(fail_num_frame);
    if (fail_num.size()>3) fail_num.pop_front();
    int fail_sum = accumulate(fail_num.begin(),fail_num.end(),0);
    //  std::cout << "fail_num_frame:"<<fail_num_frame<<"  fail_num_sum: " <<fail_sum<< std::endl;
    while (trackers_fail.size() >fail_sum)
      trackers_fail.pop_front();
    if ((ros::Time::now() - last_pop_time).toSec() > 0.2 && trackers_fail.size() > 0)
    {
        last_pop_time = ros::Time::now();
        trackers_fail.pop_front();
    }
    if ((boxes_track.size()==0 && (ros::Time::now().toSec()-last_detect_time) >relax_time) || track_fail ||  ( (ros::Time::now().toSec()-last_detect_time) >recover_time && (ros::Time::now().toSec()-last_detect_time) < 1e5 ))
    {
    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
    api.detection(cv_ptr->image, boxes);
    last_detect_time = ros::Time::now().toSec();
    double compTime = std::chrono::duration_cast<std::chrono::microseconds>
    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    std::cout << "detection time cost (ms)： " << compTime <<"  "<<boxes.size()<<std::endl;
    }
    for (int i = 0; i < boxes.size(); i++) {
        boxes[i].x1 = clip( boxes[i].x1, 0,  cv_ptr->image.cols );
        boxes[i].y1 = clip( boxes[i].y1, 0,  cv_ptr->image.rows );
        boxes[i].x2 = clip( boxes[i].x2, 0,  cv_ptr->image.cols-2 );
        boxes[i].y2 = clip( boxes[i].y2, 0,  cv_ptr->image.rows-2 );
        std::cout<<"box cate: "<<boxes[i].cate<<"  "<<boxes_track.size()<<std::endl;
        if (std::find(interested_obj.begin(), interested_obj.end(), boxes[i].cate ) == interested_obj.end())
        {break;}
        std::cout<<boxes[i].x1<<" "<<boxes[i].y1<<" "<<boxes[i].x2<<" "<<boxes[i].y2
                 <<" "<<boxes[i].score<<" "<< class_names[boxes[i].cate] <<std::endl;
        max_iou = getIOU(boxes[i]);
        float max_iou_fail = getIOU_fail(boxes[i]);
        if (max_iou < 0.93 && max_iou > iou_thre) //tracking box is not accurate anymore
        {
         if (if_debug) std::cout<<"update the box pos:  "<<" "<<matched_track_id<<"   "<<trackers[matched_track_id].tracker_id<<" "<<max_iou<<std::endl;
        FDSSTTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB,trackers[matched_track_id].tracker_id);
        auto iter = trackers.erase(trackers.begin() +matched_track_id);
        auto iter1 = boxes_track.erase(boxes_track.begin() + matched_track_id);
        auto iter2 =  boxes_cate_id.erase( boxes_cate_id.begin()+matched_track_id);
        t_bbox.x =  boxes[i].x1;  t_bbox.y = boxes[i].y1;  t_bbox.height = boxes[i].getHeight();  t_bbox.width =  boxes[i].getWidth();
        tracker.init(t_bbox,dst,scale);
        //  std::cout<<"1:  "<<boxes_track.size()<<std::endl;
         trackers.emplace_back(tracker);
        //  std::cout<<"2:  "<<boxes_track.size()<<std::endl;
         boxes_track.emplace_back(t_bbox);
         boxes_cate_id.emplace_back(boxes[i].cate);
        }
        else if (  max_iou_fail> iou_thre)
        {
         if (if_debug) std::cout<<"recover from failed trackers:  "<<" "<<matched_track_id_fail<<"   "<<trackers_fail[matched_track_id_fail].tracker_id<<" "<<max_iou<<std::endl;
        FDSSTTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB,trackers_fail[matched_track_id_fail].tracker_id);
        t_bbox.x =  boxes[i].x1;  t_bbox.y = boxes[i].y1;  t_bbox.height = boxes[i].getHeight();  t_bbox.width =  boxes[i].getWidth();
        tracker.init(t_bbox,dst,scale);
        //  std::cout<<"1:  "<<boxes_track.size()<<std::endl;
         trackers.emplace_back(tracker);
        //  std::cout<<"2:  "<<boxes_track.size()<<std::endl;
         boxes_track.emplace_back(t_bbox);
         boxes_cate_id.emplace_back(boxes[i].cate);
        }
        else if  ((max_iou<iou_thre ) &&  (  max_iou_fail < iou_thre))  //new object detected
        {
         tracker_id += 1;
         FDSSTTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB,tracker_id);
          if (if_debug) std::cout<<"new obj:  "<<tracker_id<<std::endl;
         t_bbox.x =  boxes[i].x1;  t_bbox.y = boxes[i].y1;  t_bbox.height = boxes[i].getHeight();  t_bbox.width =  boxes[i].getWidth();
         tracker.init(t_bbox,dst,scale);
        //  std::cout<<"1:  "<<boxes_track.size()<<std::endl;
         trackers.emplace_back(tracker);
        //  std::cout<<"2:  "<<boxes_track.size()<<std::endl;
         boxes_track.emplace_back(t_bbox);
         boxes_cate_id.emplace_back(boxes[i].cate);
        }
        // double distance = depthImgCopy_.at<ushort>(int((boxes[i].y1 + boxes[i].y2)/2), int((boxes[i].x1 + boxes[i].x2)/2));
        // std::cout << "distance :" << distance << std::endl;
       if (if_debug)
       {
        char text[256];
        sprintf(text, "%s %.1f%%", class_names[boxes[i].cate], boxes[i].score * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = boxes[i].x1;
        int y = boxes[i].y1 - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > cv_ptr->image.cols)
            x = cv_ptr->image.cols - label_size.width;

        cv::rectangle(cv_ptr->image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), -1);

        cv::putText(cv_ptr->image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

        cv::rectangle (cv_ptr->image, cv::Point(boxes[i].x1, boxes[i].y1), 
                       cv::Point(boxes[i].x2, boxes[i].y2), cv::Scalar(255, 255, 0), 2, 2, 0);
        }
  }
  if (if_debug)
   {
  for (int k =0;k<boxes_track.size();k++)
  {
     char track_id[256];
   sprintf(track_id, "%s %d","id-",trackers[k].tracker_id);
   int baseLine1 = 0;
   cv::Size label_size1 = cv::getTextSize(track_id, cv::FONT_HERSHEY_COMPLEX, 0.6, 1,&baseLine1);
        int x1 = boxes_track[k].x;
        int y1 = boxes_track[k].y - label_size1.height-baseLine1;
        if (y1 < 0)
            y1 = 0;
        if (x1 + label_size1.width > cv_ptr->image.cols)
            x1 = cv_ptr->image.cols - label_size1.width;

  cv::rectangle (cv_ptr->image, cv::Point(boxes_track[k].x, boxes_track[k].y), 
                cv::Point(boxes_track[k].x+boxes_track[k].width, boxes_track[k].y+boxes_track[k].height),cv::Scalar(color_vec[trackers[k].tracker_id%8][0]*255,color_vec[trackers[k].tracker_id%8][1]*255,
                color_vec[trackers[k].tracker_id%8][2]*255), 2, 2, 0);
  cv::putText(cv_ptr->image, track_id, cv::Point(x1, y1 + label_size1.height),
          cv::FONT_HERSHEY_COMPLEX, 0.9, cv::Scalar(0, 105, 255), 1);
   }
 image_pub_.publish(cv_ptr->toImageMsg());
   }

  if (if_depth)
  {
  get_depth();}
  // std::cout<<"depths:"<<depths[0]<<std::endl;
  if (boxes_track.size())
      pub_objects(msg);
  ros::spinOnce();
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    depth_time_ = msg->header.stamp.toSec();
    // std::cout<<"depth receive! \n"<<depth_time_<<std::endl;
    while ((depth_time_  - rgb_time_) > 0.004){sleep(0.001);ros::spinOnce();}
    if(if_debug) std::cout << "time gap (depth):"<< (depth_time_ - rgb_time_)<< std::endl;
    if_depth = true;
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(msg, msg->encoding); //16UC1
      depthImgCopy_ = cv_depth_ptr->image.clone();
      if(if_debug) std::cout<<"depth receive! \n"<<cv_depth_ptr->image.size()<<std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ros::spinOnce();
  }

};
}
// PLUGINLIB_EXPORT_CLASS(yolo_ns, YoloDetector, yolo_ns::YoloDetector, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(yolo_ns::YoloDetector, nodelet::Nodelet)

// int main(int argc, char** argv)
// {   

//   ros::init(argc, argv, "object_detector");
//   ros::NodeHandle nh;
//   YoloDetector ic;

//   ros::spin();

//     // cv::Mat cvImg = cv::imread("download.jpeg");
//     // cv::imwrite("output.png", cvImg);

//   return 0;
// }


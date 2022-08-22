#include<iostream>
#include<opencv2/opencv.hpp>
#include</home/chen/opencv_build/opencv_contrib/modules/tracking/include/opencv2/tracking/tracking.hpp>
#include<string>
#include<fstream>
#include<time.h>
#include "fdssttracker.hpp"

using namespace std;
using namespace cv;

#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

Point previousPoint, currentPoint;
Rect bbox(1,1,1,1);
Point center;
//动态数组存储坐标点
vector<Point2d> points;

void draw_rectangle(int event, int x, int y, int flags, void*);
Mat tmp,frame,dst;


int main(int argc,char *argv[])
{
	VideoCapture cap;
	int count = 1;
	double total_t = 0;
	//按照视频实际路径更改
	//若是图片序列，自行修改其文件名遍历格式
	//例如：图像路径imgPath
	//char name[10];
	//sprintf_s(name, "%04d", count);
	//std::string imgPath = "F:\\Code\\trackingAlgorithm\\fDSST_cpp-master\\src\\Car1\\img\\";
	//std::string imgFinalPath = imgPath + std::string(name) + ".jpg";

	string filename = "videoplayback.mp4";	
	cap.open(filename);
	if (!cap.isOpened())
	{
		cout << "###############视频打开失败###############" << endl;
		return -1;
	}
	cap.set(cv::CAP_PROP_POS_FRAMES,900);
	cap.read(frame);
	cvtColor(frame,dst, COLOR_BGR2GRAY);
	if (!frame.empty())
	{
		namedWindow("output", 0);
		imshow("output", dst);
		setMouseCallback("output", draw_rectangle, 0);
		waitKey();
	}

	/*********************Opencv目标追踪算法模板函数***************************/
	//Ptr<TrackerMIL> tracker=TrackerMIL::create();
	//Ptr<TrackerTLD> tracker=TrackerTLD::create();
	//Ptr<TrackerMedianFlow> tracker=TrackerMedianFlow::create();
	// Ptr<TrackerKCF> tracker=TrackerKCF::create();
	//Ptr<TrackerBoosting> tracker=TrackerBoosting::create();
	//Ptr<TrackerCSRT> tracker = TrackerCSRT::create();
	//Ptr<TrackerGOTURN> tracker = TrackerGOTURN::create();
	/***********************************************************************/

	/************************FDSST目标跟踪算法*********************************/
	/************************************************************************/

	bool HOG = true;
	bool FIXEDWINDOW = true;
	bool MULTISCALE =true;
	bool SILENT = true;
	bool LAB = false;
	// Create DSSTTracker tracker object
	FDSSTTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	//ofstream infile;
	//infile.open("./coordxy.txt");
	double duration = 0;
	for(;;)
	{	
		cap.read(frame);
		if (!cap.read(frame))
		{
			break;
		}
		cvtColor(frame, dst, COLOR_BGR2GRAY);//fdsst源程序fhog只能传入灰度图片，因此对图片做一个灰度转化

		//计时打点开始
		auto t_start = clock();
		//字符数组存储坐标点输出名称
		//char target[30];
		if (frame.empty())
		{
			break;
		}
		double timer = (double)getTickCount();
		if (count==1)
		{
			//传入首帧鼠标框选初始跟踪框bbox
		
			/*opencv自带算法*/
			// tracker->init(frame, bbox);
			/*改进算法*/
			//FDSST
			tracker.init(bbox, dst);
		}
		else {
			/*opencv自带算法*/
			// tracker->update(frame, bbox);
			/*改进算法*/
			//FDSST
			bbox = tracker.update(dst);
			
		}
		//计时函数计时打点结束
        
		//求FPS:
		 float fps = getTickFrequency() / ((double)getTickCount() - timer);
		// cout << "FPS: " << 1 / duration << "\n"<<"time cost:\n"<<duration<<endl;
		
		  total_t += 1/fps;
		cout << "FPS: " << fps <<"     "<<count/total_t<<endl;
		count++;
		putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
		rectangle(frame, bbox, Scalar(255, 255, 0), 2, 1);
		center.x = bbox.x + bbox.width / 2;
		center.y = bbox.y + bbox.height / 2;
		points.push_back(center);
		//绘制轨迹跟踪框中心点图线
		for (int j = 1; j < points.size(); j++)
		{
			Point pre, last;
			if (points.size() < 3)
			{
				pre = points[j];
				last = points[j];
			}
			else
			{
				pre = points[j - 1];
				last = points[j];
				line(frame, pre, last, Scalar(0, 255, 0), 1, 8);
			}
		}
		circle(frame, center, 4, Scalar(0, 255, 0), -1);
		/*显示输出检测中心点位置坐标*/
		//sprintf(target, "检测框中心位置(%d,%d)", center.x, center.y);
		//cout << target << endl;
		/********************将检测框中心写入txt文件中*************************/
		/*
		infile << center.x << "\t" << center.y << endl;
		*/

		namedWindow("tracking", 0);
		imshow("tracking", frame);
		int delayms = 20;
		if (waitKey(delayms) == 27)
			break;
		

	}
	//infile.close();
	return 0;
}
void draw_rectangle(int event, int x, int y, int flags, void*)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		previousPoint = Point(x, y);
		cout << "(" << previousPoint.x << "," << previousPoint.y << ")" << endl;
	}
	else if (event == EVENT_MOUSEMOVE && (flags&EVENT_FLAG_LBUTTON))
	{
		Mat tmp;
		char str[20];
		frame.copyTo(tmp);
		currentPoint = Point(x, y);
		sprintf(str, "(%d,%d)", currentPoint.x, currentPoint.y);
		putText(tmp, str, currentPoint, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 2, 8);
		rectangle(tmp, previousPoint, currentPoint, Scalar(0, 255, 0),2, 1, 0);
		imshow("output", tmp);
	}
	else if (event == EVENT_LBUTTONUP)
	{
		bbox.x = previousPoint.x;
		bbox.y = previousPoint.y;
		bbox.width = abs(previousPoint.x - currentPoint.x);
		bbox.height = abs(previousPoint.y - currentPoint.y);

	}
	else if (event == EVENT_RBUTTONUP)
		destroyWindow("output");
}

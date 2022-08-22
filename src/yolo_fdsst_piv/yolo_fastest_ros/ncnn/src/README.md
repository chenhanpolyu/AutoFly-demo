# trackingTarget
##  c++测试fDsst和Opencv中跟踪算法API效果
#### 软件与库环境：opencv3.4+opencv_contrib3.4
#### 程序效果：鼠标框选初始跟踪框，运行后显示跟踪效果以及跟踪框中心坐标轨迹。
#### 🎯如下图： 

<table>
<td><center><img src=https://github.com/TakumiWzy/trackingTarget/blob/master/img/pic1.PNG width="200px" height="200px" />
	
	Figure 1. 鼠标框选初始跟踪框
</center>
</td>
<td><center><img src=https://github.com/TakumiWzy/trackingTarget/blob/master/img/pic2.PNG width="360px" height="200px" />
	
	Figure 2. 跟踪轨迹效果图	
</center>
</td>
</table>

#### 🎯跟踪算法使用方法：
程序下载之后主函数：`trackingTarget.cpp`。自行修改视频路径：filename.若为图片序列：将视频路径上面路径解除注释。

#### 🎯步骤总结为：
##### Step1.修改输入路径
##### Step2.编译运行，鼠标框选初始跟踪框跟踪，默认跟踪算法为fdsst。自行修改体验不同跟踪算法。
##### Step3.opencv跟踪API使用（需要自行编译opencv_contrib模块）。
###### ①将跟踪API模板函数取消注释即可。
###### ②例如：Ptr<TrackerKCF> tracker=TrackerKCF::create(),再将fdsst的算法注释。			
			/*opencv自带算法*/
			tracker->init(frame, bbox);
			/*改进算法*/
			//FDSST
			//tracker.init(bbox, dst); 

##### Step4.运行并查看目标跟踪和跟踪轨迹效果。

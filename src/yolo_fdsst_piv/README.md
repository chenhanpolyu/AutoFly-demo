# yolo_fdsst_piv
A universal object detecting, tracking, and motion estimating framework

Demo:

RGB image based object tracker:

![image](https://github.com/arclab-hku/yolo_fdsst_piv/blob/main/gif/ezgif.com-video-to-gif%20(1).gif)

Object tracking and motion estimation in 3D space:

![image](https://github.com/arclab-hku/yolo_fdsst_piv/blob/main/gif/yolofdsst.gif)

If we use PIV method only to track and classify the objects into dynamic or static by velocity, the classification is often wrong due to the error in velocity estimation:

![image](https://github.com/arclab-hku/yolo_fdsst_piv/blob/main/gif/tracking_3d_noyolo.gif)
![image](https://github.com/arclab-hku/yolo_fdsst_piv/blob/main/gif/tracking_3d_noyolo_1.gif)

## launch

Build this repo in your /src folder, source the workspace.

launch the object detector and tracker on RGB image:
`roslaunch yolo_detector yolo_ros.launch`
Please modify the related parameters in the config/yolo_ros.yaml. Set if_debug=true if you want to check the tracking bounding box in the image output.

Then, launch the object velocity estimator based on point cloud:
`roslaunch dyn_object_tracker piv_filter_start.launch`

It is developed from 2d PIV method, I make it applicable for 3d point cloud and design two simple local features to make the velocity estimation more accurate.
It can classify the dynamic and static objects with the help of the image-based object detector, and only output the velocity of the dynamic ones.

#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField,Imu,Image
from sensor_msgs import point_cloud2
import tf
from geometry_msgs.msg import PoseStamped,TwistStamped,Point
import numpy as np
from utils import earth_to_body_frame,body_to_earth_frame
import threading
import time
import sklearn.cluster as skc
from visualization_msgs.msg import Marker,MarkerArray
import math,copy
from message_filters import TimeSynchronizer, Subscriber,ApproximateTimeSynchronizer
#from cv_bridge import CvBridge, CvBridgeError
# import cv2
# from track_test import track
class convert_pcl():
    def callback(self,data):
        #y.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.pcl = [list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)),list(point_cloud2.read_points(data, field_names=("rgb")))]
        self.pcl_time =data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        
    def pos_pcl(self,pcl,imu,odom):
        # self.pos=pos.pose
        self.pos=odom.pose
        self.pos_time = odom.header.stamp.secs + odom.header.stamp.nsecs * 1e-9
#        assert isinstance(pcl, PointCloud2)
        # global point2,pos,pub
        self.pcl = [list(point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True)),list(point_cloud2.read_points(pcl, field_names=("rgb")))]
        self.pcl_time =pcl.header.stamp.secs + pcl.header.stamp.nsecs * 1e-9
        self.pcl_timestamp = pcl.header.stamp
        self.ang_vel = np.array([imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z])
        self.line_vel = np.array([odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z])
        self.vel_time = imu.header.stamp.secs + imu.header.stamp.nsecs * 1e-9
        # print("alighed",self.vel_time,self.pcl_time)
        self.if_align = 1
    
    # def talker(point2):
    
    #     # while not rospy.is_shutdown():
    #     pub.publish(hello_str)
    #     rate.sleep()      
    def thread_job():
        rospy.spin()
        
    def local_position_callback(self,data):
        self.pos=data
        # parse_local_position(data)
        
    def velocity_callback(self, data):
        self.ang_vel = np.array([data.twist.angular.x,data.twist.angular.y,data.twist.angular.z])
        self.line_vel = np.array([data.twist.linear.x,data.twist.linear.y,data.twist.linear.z])
    def octo_callback(self,data):
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.octo_pcl = list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    def listener(self):
    

        self.if_align = 0
       # self.CvImg = CvBridge()
        rospy.init_node('point_transfrer', anonymous=True)
        self.clust_pub = rospy.Publisher('/cluster_center',Marker,queue_size=1)
        self.sta_pub = rospy.Publisher('/points_global_sta', PointCloud2, queue_size=1)  #static points
#        self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity_local',
#                                       TwistStamped,
#                                       self.velocity_callback,queue_size=1,buff_size=52428800)
#        self.local_vel_sub1 = rospy.Subscriber('mavros/local_position/velocity',
#                                       TwistStamped,
#                                       self.velocity_callback,queue_size=1,buff_size=52428800)
        self.alpub = rospy.Publisher('/points_global_all', PointCloud2, queue_size=1)    #all points
        self.globalpub = rospy.Publisher('/points_global', PointCloud2, queue_size=1)
        self.octo_pub = rospy.Publisher('/octomap_point_cloud_centers_local', PointCloud2, queue_size=1)
#        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
#                                                  PoseStamped,
#                                                  self.local_position_callback,queue_size=1)
#        self.pcl_sub = rospy.Subscriber('/filtered_RadiusOutlierRemoval',
#                                      PointCloud2,
#                                      self.callback,queue_size=1,buff_size=52428800) #/filtered_RadiusOutlierRemoval',
        self.octo_pcl_sub = rospy.Subscriber('/octomap_point_cloud_centers',
                                          PointCloud2,
                                          self.octo_callback,queue_size=1,buff_size=52428800) #/octomap_point_cloud_centers
        # self.dynobs_publisher = rospy.Publisher("dyn_obs", Marker, queue_size=1)
        # self.dynv_publisher = rospy.Publisher("dyn_v", Marker, queue_size=1)
        self.dyn_publisher = rospy.Publisher("/dyn", MarkerArray, queue_size=1)
        self.potential_dyn_publisher = rospy.Publisher("/potential_dyn", MarkerArray, queue_size=1)
        self.target_publisher = rospy.Publisher("/target", MarkerArray, queue_size=1)
        self.tss = ApproximateTimeSynchronizer([
                                                Subscriber('/filtered_RadiusOutlierRemoval',PointCloud2),
                                                Subscriber('/mavros/imu/data', Imu),
                                                Subscriber('/vicon_imu_ekf_odom', Odometry)],
                                               # Subscriber('/points_global_all',PointCloud2)
                                                # Subscriber('/mavros/local_position/velocity',TwistStamped)],
        5,0.03, allow_headerless=True)
        self.tss.registerCallback(self.pos_pcl)
        self.tss_img = ApproximateTimeSynchronizer([
                                                     Subscriber('/camera/depth/image_rect_raw',Image),
                                                    # Subscriber('/camera/depth_aligned_to_color_and_infra1/image_raw',Image),
                                                     Subscriber('/camera/color/image_raw',Image)],
                                                     5,0.03, allow_headerless=True)
        self.tss_img.registerCallback(self.img_cb)
        # add_thread = threading.Thread(target = thread_job)
        # add_thread.start()
        # rospy.spin()
    
    def img_cb(self,depth,color):
        self.color_img = color #self.CvImg.imgmsg_to_cv2(color, "bgr8")
        self.depth_img = depth #self.CvImg.imgmsg_to_cv2(depth, "16UC1")
        self.if_color_depth_align = True

    def cluster_pub(self,cluster_center):
        CLU = Marker()
        CLU.header.frame_id = "map"
        CLU.header.stamp = rospy.Time.now()
        CLU.type = Marker.POINTS
        CLU.id=0
        CLU.scale.x = 0.3
        CLU.scale.y = 0.3
        CLU.color.a = 0.75
        CLU.color.b = 1.0
        for m in cluster_center:

            p = Point()
            p.x = m[0]
            p.y = m[1]
            p.z = m[2]
            CLU.points.append(p)
        self.clust_pub.publish(CLU)
    def parse_local_position(self,local_position, mode="e"):
        # global rx,ry,rz,rpy_eular
        rx=local_position.pose.position.x
        ry=local_position.pose.position.y
        rz=local_position.pose.position.z
        
        qx=local_position.pose.orientation.x
        qy=local_position.pose.orientation.y
        qz=local_position.pose.orientation.z
        qw=local_position.pose.orientation.w
        
#        print(time)
        # print(rx,ry,rz)
        if mode == "q":
            return (rx,ry,rz,qx,qy,qz,qw)
    
        elif mode == "e":
            rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
            return (rx,ry,rz)+ rpy_eular
        else:
            raise UnboundLocalError
            
    def xyz_array_to_pointcloud2(self,points, frame_id=None, stamp=None):
        '''
        Create a sensor_msgs.PointCloud2 from an array
        of points.
        '''
        msg = PointCloud2()
        if stamp:
            msg.header.stamp = stamp
        if frame_id:
            msg.header.frame_id = frame_id
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = np.asarray(points, np.float32).tostring()
    
        return msg
    def distance_filter(self,pcl,dis):
        # filtered_pcl=[]
        # for point in pcl:
        #     point=list(point)
        #     d_point=np.linalg.norm(point)
        #     if d_point<dis:
        #         filtered_pcl.append(point+[d_point])
        # filtered_pcl=np.array(filtered_pcl)
        
        d_point=np.linalg.norm(pcl,axis=1)
        filtered_pcl=np.c_[pcl,d_point]
        filtered_pcl=filtered_pcl[d_point<dis]
        if len(filtered_pcl)>0:
            filtered_pcl=filtered_pcl[np.lexsort(filtered_pcl.T)]
            # filtered_pcl=filtered_pcl[:,0:3]
        return filtered_pcl[:,0:3]

    # def parse_velocity(self,vel):
    #     wx=vel.twist.angular.x
    #     wy=vel.twist.angular.y
    #     wz=vel.twist.angular.z
    #     vx=vel.twist.linear.x
    #     vy=vel.twist.linear.y
    #     vz=vel.twist.linear.z
    #     return [vx,vy,vz,wx,wy,wz]
    def publish_dyn_obs(self,c_dyn1,v_dyn1,obsd,max_displ,if_track):
        dyn=MarkerArray()
        
        for m in range(len(c_dyn1)):
#            if np.linalg.norm(np.array(pos[0:3])-c_dyn1[m])>8 :#or abs(math.atan2(-pos[1]+c_dyn1[m][1],-pos[0]+c_dyn1[m][0])-pos[3])>math.pi/3:
#                break
            dynobs = Marker()
            dynobs.header.frame_id = "map"
            dynobs.header.stamp = rospy.Time.now()
            dynobs.type = Marker.CUBE
            dynobs.pose.position.x = c_dyn1[m][0]
            dynobs.pose.position.y = c_dyn1[m][1]
            dynobs.pose.position.z = c_dyn1[m][2]
            dynobs.id=3*m
            dynobs.scale.x = obsd[m][0]
            dynobs.scale.y = obsd[m][1]
            dynobs.scale.z = obsd[m][2]
            if if_track[m]:
                dynobs.color.a = 0.4
                dynobs.color.r = 0.0
                dynobs.color.g = 0.9
                dynobs.color.b = 0.1  
            else:
                dynobs.color.a = 0.3
                dynobs.color.r = 255/255
                dynobs.color.g = 20/255
                dynobs.color.b = 147/255
            
            dynobs.pose.orientation.x = 0
            dynobs.pose.orientation.y = 0
            dynobs.pose.orientation.z = 0
            dynobs.pose.orientation.w = 1.0
            
            dynbbox = Marker()
            dynbbox.header.frame_id = "map"
            dynbbox.header.stamp = rospy.Time.now()
            dynbbox.type = Marker.CUBE
            dynbbox.pose.position.x = c_dyn1[m][0]
            dynbbox.pose.position.y = c_dyn1[m][1]
            
            dynbbox.id=3*m+1
            dynbbox.scale.x = obsd[m][0]+2*max_displ
            dynbbox.scale.y = obsd[m][1]+2*max_displ
            if c_dyn1[m][2] - 0.5*obsd[m][2] - max_displ <0:
                dynbbox.scale.z = 0.5*obsd[m][2]+max_displ+c_dyn1[m][2]
                dynbbox.pose.position.z = dynbbox.scale.z*0.5
            else:
                dynbbox.pose.position.z = c_dyn1[m][2]
                dynbbox.scale.z = obsd[m][2]+2*max_displ
       
            dynbbox.color.a = 0.1
            dynbbox.color.r = 1.0
            dynbbox.color.g = 0.6
            dynbbox.color.b = 0.0
            
            dynbbox.pose.orientation.x = 0
            dynbbox.pose.orientation.y = 0
            dynbbox.pose.orientation.z = 0
            dynbbox.pose.orientation.w = 1.0
            
            dynv = Marker()
            p1=Point()
            p2=Point()
            dynv.header.frame_id = "map"
            dynv.header.stamp = rospy.Time.now()
            dynv.type = Marker.ARROW
            dynv.id=3*m+2
            p1.x,p1.y,p1.z=c_dyn1[m][0],c_dyn1[m][1],c_dyn1[m][2]
            # p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0]+np.sign(v_dyn1[m][0])*obsd[m]/2,c_dyn1[m][1]+v_dyn1[m][1]+np.sign(v_dyn1[m][1])*obsd[m]/2,v_dyn1[m][2]+c_dyn1[m][2]+np.sign(v_dyn1[m][2])*obsd[m]/2
            p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0],c_dyn1[m][1]+v_dyn1[m][1],v_dyn1[m][2]+c_dyn1[m][2]
            # dynv.points.push_back(p1)
            # dynv.points.push_back(p2)
            dynv.points.append(p1)
            dynv.points.append(p2)
            # dynv.pose.position.x = self.velo_goal[0]
            # dynv.pose.position.y = self.velo_goal[1]
            # dynv.pose.position.z = self.velo_goal[2]
    
            dynv.scale.x = 0.2 #diameter of arrow shaft
            dynv.scale.y = 0.4 #diameter of arrow head
            dynv.scale.z = 0.6
            dynv.color.a = 1  #transparency
            dynv.color.r = 0.8
            dynv.color.g = 0.1
            dynv.color.b = 0.1
            
            
            dyn.markers.append(dynobs)
            dyn.markers.append(dynv)
            dyn.markers.append(dynbbox)
        # dynv.pose.orientation.x = 0
        # dynv.pose.orientation.y = 0
        # dynv.pose.orientation.z = 0
        # dynv.pose.orientation.w = 0
        # Publish the MarkerArray
        # self.dynobs_publisher.publish(dynobs)
        # self.dynv_publisher.publish(dynv)
        self.dyn_publisher.publish(dyn)
        
    def publish_dyn_obs_pyramids(self,c_dyn1,v_dyn1,obsd,max_displ,if_track,local_pos):
        dyn=MarkerArray()
        life_t =0.1
        for m in range(len(c_dyn1)):
#            if np.linalg.norm(np.array(pos[0:3])-c_dyn1[m])>8 :#or abs(math.atan2(-pos[1]+c_dyn1[m][1],-pos[0]+c_dyn1[m][0])-pos[3])>math.pi/3:
#                break
            box_w = np.linalg.norm(obsd[m][0:2])
            box_h = obsd[m][2]
            rela_p = c_dyn1[m] - local_pos
            dy = ((box_w**2/4)/(1+(rela_p[1]/rela_p[0])**2))**0.5
            dx = -rela_p[1]/rela_p[0]*dy
            dots = np.array([[c_dyn1[m][0]-dx,c_dyn1[m][1]-dy,c_dyn1[m][2]-0.5*obsd[m][2]],
                             [c_dyn1[m][0]-dx,c_dyn1[m][1]-dy,c_dyn1[m][2]+0.5*obsd[m][2]],
                             [c_dyn1[m][0]+dx,c_dyn1[m][1]+dy,c_dyn1[m][2]+0.5*obsd[m][2]],
                             [c_dyn1[m][0]+dx,c_dyn1[m][1]+dy,c_dyn1[m][2]-0.5*obsd[m][2]],
                             [c_dyn1[m][0]-dx,c_dyn1[m][1]-dy,c_dyn1[m][2]-0.5*obsd[m][2]]])
            print("box_w:",box_w,"box_h:",box_h,"dy",dy,"dx",dx,(box_w**2/4)/(1+(obsd[m][1]/obsd[m][0])**2),(box_w**2/4))
            print("obsd:",obsd[m],"cluster pos:",c_dyn1[m])
            dynobs = Marker()
            dynobs.header.frame_id = "map"
            dynobs.header.stamp = rospy.Time.now()
            dynobs.type = Marker.LINE_STRIP
            for i in range(5):
                p1=Point()
                p1.x,p1.y,p1.z = dots[i]
                dynobs.points.append(p1)
            dynobs.id=3*m
            dynobs.scale.x = 0.1
          
            dynobs.color.a = 1.0
            dynobs.color.b = 1.0  
            
            
            pyramid = Marker()
            pyramid.header.frame_id = "map"
            pyramid.header.stamp = rospy.Time.now()
            pyramid.type = Marker.LINE_LIST
            pn=Point()
            pn.x,pn.y,pn.z = local_pos
            for i in range(4):
                p1=Point()
                p1.x,p1.y,p1.z = dots[i]
                pyramid.points.append(pn)
                pyramid.points.append(p1)
            pyramid.id=3*m+1
            pyramid.scale.x = 0.03
            pyramid.color.a = 1.0
            pyramid.color.r = 1.0  
            
            
            dynv = Marker()
            p1=Point()
            p2=Point()
            dynv.header.frame_id = "map"
            dynv.header.stamp = rospy.Time.now()
            dynv.type = Marker.ARROW
            dynv.id=3*m+2
            p1.x,p1.y,p1.z=c_dyn1[m][0],c_dyn1[m][1],c_dyn1[m][2]
            # p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0]+np.sign(v_dyn1[m][0])*obsd[m]/2,c_dyn1[m][1]+v_dyn1[m][1]+np.sign(v_dyn1[m][1])*obsd[m]/2,v_dyn1[m][2]+c_dyn1[m][2]+np.sign(v_dyn1[m][2])*obsd[m]/2
            p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0],c_dyn1[m][1]+v_dyn1[m][1],v_dyn1[m][2]+c_dyn1[m][2]
            # dynv.points.push_back(p1)
            # dynv.points.push_back(p2)
            dynv.points.append(p1)
            dynv.points.append(p2)
            # dynv.pose.position.x = self.velo_goal[0]
            # dynv.pose.position.y = self.velo_goal[1]
            # dynv.pose.position.z = self.velo_goal[2]
    
            dynv.scale.x = 0.2 #diameter of arrow shaft
            dynv.scale.y = 0.4 #diameter of arrow head
            dynv.scale.z = 0.6
            dynv.color.a = 1  #transparency
            dynv.color.r = 0.8
            dynv.color.g = 0.1
            dynv.color.b = 0.1
            
            
            dynobs.lifetime = rospy.Duration(life_t)
            dynv.lifetime = rospy.Duration(life_t)
            pyramid.lifetime = rospy.Duration(life_t)
            dyn.markers.append(dynobs)
            dyn.markers.append(dynv)
            dyn.markers.append(pyramid)
        # dynv.pose.orientation.x = 0
        # dynv.pose.orientation.y = 0
        # dynv.pose.orientation.z = 0
        # dynv.pose.orientation.w = 0
        # Publish the MarkerArray
        # self.dynobs_publisher.publish(dynobs)
        # self.dynv_publisher.publish(dynv)
        self.dyn_publisher.publish(dyn)
    def publish_potential_dyn_obs(self,c_dyn1,v_dyn1,obsd):
        dyn=MarkerArray()
        
        for m in range(len(c_dyn1)):
            dynobs = Marker()
            dynobs.header.frame_id = "map"
            dynobs.header.stamp = rospy.Time.now()
            dynobs.type = Marker.SPHERE
            dynobs.pose.position.x = c_dyn1[m][0]
            dynobs.pose.position.y = c_dyn1[m][1]
            dynobs.pose.position.z = c_dyn1[m][2]
            dynobs.id=2*m
            dynobs.scale.x = obsd[m]
            dynobs.scale.y = obsd[m]
            dynobs.scale.z = obsd[m]
            dynobs.color.a = 0.5
            dynobs.color.r = 0.6
            dynobs.color.g = 0.1
            dynobs.color.b = 0.6
            
            dynobs.pose.orientation.x = 0
            dynobs.pose.orientation.y = 0
            dynobs.pose.orientation.z = 0
            dynobs.pose.orientation.w = 0
            
            dynv = Marker()
            p1=Point()
            p2=Point()
            dynv.header.frame_id = "map"
            dynv.header.stamp = rospy.Time.now()
            dynv.type = Marker.ARROW
            dynv.id=2*m+1
            p1.x,p1.y,p1.z=c_dyn1[m][0],c_dyn1[m][1],c_dyn1[m][2]
            # p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0]+np.sign(v_dyn1[m][0])*obsd[m]/2,c_dyn1[m][1]+v_dyn1[m][1]+np.sign(v_dyn1[m][1])*obsd[m]/2,v_dyn1[m][2]+c_dyn1[m][2]+np.sign(v_dyn1[m][2])*obsd[m]/2
            p2.x,p2.y,p2.z=v_dyn1[m][0]+c_dyn1[m][0],c_dyn1[m][1]+v_dyn1[m][1],v_dyn1[m][2]+c_dyn1[m][2]
            # dynv.points.push_back(p1)
            # dynv.points.push_back(p2)
            dynv.points.append(p1)
            dynv.points.append(p2)
            # dynv.pose.position.x = self.velo_goal[0]
            # dynv.pose.position.y = self.velo_goal[1]
            # dynv.pose.position.z = self.velo_goal[2]
    
            dynv.scale.x = 0.1 #diameter of arrow shaft
            dynv.scale.y = 0.2 #diameter of arrow head
            dynv.scale.z = 0.6
            dynv.color.a = 1  #transparency
            dynv.color.r = 0.5
            dynv.color.g = 0.5
            dynv.color.b = 0.1
            
            
            dyn.markers.append(dynobs)
            dyn.markers.append(dynv)
    
        # dynv.pose.orientation.x = 0
        # dynv.pose.orientation.y = 0
        # dynv.pose.orientation.z = 0
        # dynv.pose.orientation.w = 0
        # Publish the MarkerArray
        # self.dynobs_publisher.publish(dynobs)
        # self.dynv_publisher.publish(dynv)
        self.potential_dyn_publisher.publish(dyn)
    def pub_target (self,targets,matched):
        tgs = MarkerArray()
        for i in range(len(targets)):
            tg = Marker()
            tg.header.frame_id = "map"
            tg.header.stamp = rospy.Time.now()
            tg.type = Marker.SPHERE
            tg.pose.position.x = targets[i][0]
            tg.pose.position.y = targets[i][1]
            tg.pose.position.z = targets[i][2]
            tg.id=i
            tg.scale.x = 0.2
            tg.scale.y = 0.2
            tg.scale.z = 0.2
            tg.color.a = 1.0
            if matched[i]:
                tg.color.g = 1.0
            else:
                tg.color.r = 1.0
            tg.pose.orientation.w = 1.0
            tg.lifetime = rospy.Duration(0.1)
            tgs.markers.append(tg)
        self.target_publisher.publish(tgs)
    def ang_cos(self,b,a,c=[]): #c-> the angle we need
        if len(c) ==0:
            c = np.zeros(len(b))
        cb=np.linalg.norm(b-c)
        ca=np.linalg.norm(a-c)
        ab=np.linalg.norm(a-b)
        ang_cc=np.clip((cb**2+ca**2-ab**2)/(2*cb*ca),-1,1)
        print("cos of the angle:",ang_cc)
        return ang_cc
    def correct_c_dyn(self,c_dyn,p_dyn,local_pos): #modify the center of the detected obstacle
        c_orin=np.array(c_dyn)
        p_edge=p_dyn[np.argmax(np.linalg.norm(np.array(p_dyn)-c_orin,axis=1))]
        L=np.linalg.norm(np.array(p_edge)-local_pos)
        alpha=math.acos(self.ang_cos(local_pos, p_edge, c_orin))
        r_obs=L*alpha
        beta=math.pi/2-alpha
        OG=math.sin(beta)/beta*r_obs
        AG=np.linalg.norm(c_orin-local_pos)
        AO=OG+AG
        c_c_dyn=((c_orin-local_pos)*AO/AG*2+(c_orin-local_pos)*1)/3+local_pos
        # print("AO,AG!!!!!!!!!!!",AO,AG)
        return c_c_dyn,r_obs
    
    def f_vector(self,c_dyn,r_obs,p_dyn,rgb_dyn):  # return the feature vector
        num=len(p_dyn)
        vd=(4/3*math.pi*r_obs**3)
        va=np.var(p_dyn)
        rgb=np.mean(rgb_dyn)*1e39
        rgb_va=np.var(rgb_dyn*1e39)
        return [c_dyn[0],c_dyn[1],1*c_dyn[2],2*vd,0.5*va,0.7*num,25*rgb,15*rgb_va]
#        return [c_dyn[0],c_dyn[1],c_dyn[2],num,dens]
        
    def get_dplm (self,fm1,fm2,pos1):
        # pks=[]
        fft1 = np.fft.rfftn(fm1)
        fft2 = np.fft.rfftn(fm2)
        ifft = np.mean(np.fft.irfftn(np.conj(fft1)*fft2),axis=3)
        # pk1 = np.max(ifft)
     
                
        pk1 = np.max(ifft)
        disp  = np.rint(np.mean((np.where(ifft==pk1)),axis=1)).astype(int) #np.where(ifft==pk1)
        disp  = np.where(ifft==pk1)
        ifft[disp] = 0
        # print(disp)
        # pks = ifft[max(0,disp[0]-1):disp[0]+2,max(0,disp[1]-1):disp[1]+2,max(0,disp[2]-1):disp[2]+2]
        # pks_index = np.where(abs(pks-pk1)/pk1 < 0.02)
        # print(pks_index)
        # err = np.mean(pks_index,axis=1)
        # disp = disp + (err-1)
        
        pk2 = np.max(ifft)
        disp2 = np.where(ifft==pk2)
        print("pos1",pos1)
        if abs(pk2-pk1)/pk1 < 0.02:
            # disp = np.array([max(np.r_[disp[0],disp2[0]],key=abs),max(np.r_[disp[1],disp2[1]],key=abs),max(np.r_[disp[2],disp2[2]],key=abs)])
            disp = np.array([np.mean(np.r_[disp[0],disp2[0]]),np.mean(np.r_[disp[1],disp2[1]]),np.mean(np.r_[disp[2],disp2[2]])])
           
            conv_dms = np.where(np.array(pos1)+disp>=np.array(np.shape(fm1)[0:3])-1)
            print("conv_dms,disp,space shape:",conv_dms,disp,np.array(np.shape(fm1)[0:3]))
            disp[conv_dms] = disp[conv_dms] - np.array(np.shape(fm1)[0:3])[conv_dms]
        
        else:
            # disp = np.array([max(disp[0],key=abs),max(disp[1],key=abs),max(disp[2],key=abs)])
            disp = np.array([np.mean(disp[0]),np.mean(disp[1]),np.mean(disp[2])])
            conv_dms = np.where(np.array(pos1)+disp>=np.array(np.shape(fm1)[0:3])-1)
            print("conv_dms,disp,space shape:",conv_dms,disp,np.array(np.shape(fm1)[0:3]))
            disp[conv_dms] = disp[conv_dms] - np.array(np.shape(fm1)[0:3])[conv_dms]
        
        # disp = np.array([np.mean(disp[0]),np.mean(disp[1]),np.mean(disp[2])])
        # conv_dms = np.where(np.array(pos1)+disp>=np.array(np.shape(fm1)[0:3]))
        # # print("conv_dms,disp,space shape:",conv_dms,disp,np.array(np.shape(fm1)[0:3]))
        # disp[conv_dms] = disp[conv_dms] - np.array(np.shape(fm1)[0:3])[conv_dms]
        
        return disp
    def romove_occluded(self,clusters,centers,local_pos):
        angs = np.zeros(len(centers))
        i=0
        index = []
        centers = centers - local_pos
        for ct in centers:
            angs[i] = math.atan2(ct[0],ct[1])
            i+=1
        for j in range(i):
            angs_other = np.delete(angs,j)
            centers_other = np.delete(centers,j,axis=0)
            if len(angs_other) ==0:
                return []
            ind = np.argmin(abs(angs[j]-angs_other))
            if abs(angs[j]-angs_other[ind]) < 0.10 and np.linalg.norm(centers[j]) > np.linalg.norm(centers_other[ind]):
                index.append(j)
        return index
if __name__ == '__main__':
    # global pub,rate,point2,pos
    print("begin!")
    convert=convert_pcl()
   # c_track = track()
    convert.listener()
    # convert.vel=None
    convert.if_color_depth_align = False
    convert.depth_img = None
    convert.pos=None
    convert.octo_pcl=None
    convert.pcl=None
    convert.f=None
    convert.ang_vel=None
    rosrate=100
    point22=None
    c_dyn11=None
    dyn_v11=None
    local_pos=0
    octo_pcl=[]
    r0=0
    p0=0
    y0=0
    vel_infl = 0.5
    map_reso=0.2  #resolution of the voxel
    fac_dv=1.0    #dynamic obstacle velocity factor
    v_size = 0.15 #voxel size of the bounding box
    center_r = 0.3 # center ragion of the obstacle has higher weight, because the rotation is weaker
    rate = rospy.Rate(rosrate) 
    dt=0.25       # time gap bewteen two neighbor point cloud frames
    max_obs_size = 3.5 # max size of obstacle BB, x-y-z
    max_obs_speed = 2.0 # Prior Knowledge of the maximal object speed
    n_p=18        #params for DBSCAN
    r_p = 0.3
    mask_thr = 0.3 #the thredshold for the static block mask ratio to abandon the current cluster. If the masked blockes/current cluster's total blockes is greater than mask_thr, this cluster is abandoned.
    last_kftime = 0
    wt_pnum = 1.0    # weight for the number of the points in one voxel
    wt_color = 4*1e39  # weight for the color information of the colorful point cloud
    ft_vet_len = 2   # feature vector length for each voxel. Here only the mean of color and number of the points in the voxel are considered.
    xb,yb,zb = 0.12,0,0.00  # Mounting matrix of the camera.
    static_pcl_len = 1000   #Maximal memorized static point cloud (i.e., the map) size
    kf_predtime = 0.5   #Maximal predicting time in a KF for an object if no more observation can be matched
    pub_pcl_size = 500
    pcl_h=[]
    pcl_h_l=[]
    rgb_h=[]
    t_pcl=[]
    t_pcl_l=[]
    pos_h=[]
    t_dh=[]     #time for the moving obj found
    f_dyn=[]
    c_dyn=[]    #center of the moving object
    dyn_v=[]    #speed of the moving object
    dyn_d=[]
    p_dyn=[]

    c_dynkf=[]
    v_dynkf=[]
    octo_pcl1=[]
    time22=0
    time_cap=0
    pre_ct=0
    dyn_obs_reset=1
    c_dyn_l,v_dyn_l,d_dyn_l=[],[],[]
    pcl_sta,pcl_sta_h=[],[]
    sta_centers = []
    t_obs = []
    p_mat_list,p_mat_l = [],[]
    inv_flag = 0
    targets = []
    while not rospy.is_shutdown():
        starttime1 = time.time()
        t_track = 0
        pcl1=[]
        clu_p1=[]
        clu_p2=[]
        clu_c1=[]
        clu_c2=[]
        clu_c3=[] #ocro_map point cluster center
        rgb_p1=[]
        rgb_p2=[]
        obsd=[]
        c_dyn1=[]    #center of the moving object,remove in rviz simulation
        v_dyn1=[]    #speed of the moving object
        obsd2=[]
        dyn_v2=[]
        c_dyn2=[]    
        not_dyn_pt = []
        fea_1=[]   #feature array
        fea_2=[]
        if_inherit = 0
        if_stah_clear=0
        pcl_c1,pcl_c2 = [],[]
        time_itv_if = 0
        # track_flag=0
        origin=[]
        bb_lst1 = []
        bb_lst2 = []
        c_dynkf=[]
        v_dynkf=[]
        obsdkf=[]
        clts_center = []
        pub_points, pub_points_center = [],[]
                       
        # print(pos)
        if convert.pos is not None and (convert.octo_pcl is not None) and convert.if_align == 1:
            
            convert.if_align = 0
            px,py,pz,r,p,y=convert.parse_local_position(convert.pos)
            
            local_pos1=np.array([px,py,pz])
            octo_pcl=np.array(convert.octo_pcl)
            local_pos1 = (convert.pcl_time - convert.pos_time)*convert.line_vel + local_pos1
            # print('num-obs',len(octo_pcl))
            if len(octo_pcl)>0:
                octo_pcl1=octo_pcl-local_pos1

                octo_pcl1=convert.distance_filter(octo_pcl1,4)+local_pos1
            
                point22=convert.xyz_array_to_pointcloud2(octo_pcl1,'map',rospy.Time.now())
        #print((convert.pos is not None),(convert.pcl is not None),convert.ang_vel,convert.if_align)   (convert.depth_img is not None) and 
        if convert.pos is not None and (convert.pcl is not None) and (convert.ang_vel is not None) and convert.if_align: #and len(convert.pcl[0])>n_p  and convert.if_align == 1
            convert.if_align = 0
            px,py,pz,r,p,y=convert.parse_local_position(convert.pos)
            # convert.pcl_pt=convert.pcl[0]
            # convert.pcl_rgb=convert.pcl[1]
            pcl_rgb=np.array(convert.pcl[1])
            pcl=np.array(convert.pcl[0])
            
            pcl_time = convert.pcl_time
            pos_time = convert.pos_time
            vel_time = convert.vel_time
           # print(np.matmul(np.array([[1,math.sin(p)*math.tan(r),math.cos(p)*math.tan(r)],\
           # [0,math.cos(p),-math.sin(p)],[0,math.sin(p)/math.cos(r),math.cos(p)/math.cos(r)]]),convert.ang_vel.T))
            r,p,y = np.array([r,p,y])+(pcl_time - vel_time)*np.matmul(np.array([
            [0,math.cos(p),-math.sin(p)],[1,math.sin(p)*math.tan(r),math.cos(p)*math.tan(r)],[0,math.sin(p)/math.cos(r),math.cos(p)/math.cos(r)]]),convert.ang_vel.T)
            line_vel = convert.line_vel
            b2e = body_to_earth_frame(r,p,y)
            

            len_pcl_pt=len(pcl)
            len_pcl_rgb=len(pcl_rgb)

            
            


            pcl_c=pcl.copy()
#        
            if np.linalg.norm(convert.ang_vel[0:3]) < 1.5*math.pi and len(pcl)>0:
                if local_pos is 0 or ((local_pos is not 0) and np.linalg.norm(local_pos-np.array([px,py,pz]))<0.3):
                    pcl_c[:,0]=pcl[:,2]+xb  #distance from camera to uav center, xb yb zb are offset
                    pcl_c[:,1]=-pcl[:,0]+yb
                    pcl_c[:,2]=-pcl[:,1]+zb
    #                pcl_c=np.array(pcl_c)
                    local_pos1=np.array([px,py,pz])
                    local_pos1 = (pcl_time - pos_time)*line_vel + local_pos1
                    pcl_c=np.matmul(b2e,pcl_c.T).T+np.tile(local_pos1,(len(pcl_c),1))  # convert coordinate from B to E
                    pcl_c[:,2] -= np.min(pcl_c[::5][:,2])
                else:
                    pcl_c=[]
            elif len(pcl)>0:
                print('Rotating too fast!!!',convert.ang_vel)
                pcl_c=[]
                # continue
            else:
                print('empty pcl!!')
                pcl_c=[]
#            pcl1=np.array(pcl1)
            local_pos=np.array([px,py,pz])

            if len(pcl_c)>0:
                local_pos=local_pos1
                pcl1=pcl_c[pcl_c[:,2]>0.3]
                if len_pcl_pt==len_pcl_rgb:
                    pcl_rgb=pcl_rgb[pcl_c[:,2]>0.3]

            if len(pcl_sta) > static_pcl_len:
                pcl_sta_h = pcl_sta
            elif len(pcl_sta_h) + len(pcl_sta) > static_pcl_len:
                pcl_sta_h = pcl_sta_h + pcl_sta
                pcl_sta_h = pcl_sta_h[len(pcl_sta_h)-static_pcl_len::]
            else:
                pcl_sta_h = pcl_sta_h + pcl_sta
            pcl_sta=[]  #static points
            # print("mk1")
            if len(pcl1)>n_p and len_pcl_pt==len_pcl_rgb and (len(t_pcl) == 0 or pcl_time != t_pcl[-1]):
                
                if (len(t_pcl)>1 and t_pcl[-1]-t_pcl[-2]>10) or (len(t_pcl)>1 and t_pcl[-1]-t_pcl[0]<0): #in case the time is reversed
                    pcl_h=[]
                    pcl_h_l = []
                    t_pcl=[]
                    t_pcl_l=[]
                    pos_h=[]
                    rgb_h=[]
                    dyn_obs_reset=1

                    print('refresh!!!')
                pcl_h.append(pcl1)
                
                rgb_h.append(pcl_rgb)
                pos_h.append(local_pos)
                t_pcl.append(pcl_time)
                # print("mk2",len_pcl_pt==len_pcl_rgb,t_pcl[-1]-t_pcl[0],len(t_pcl))
            if len(t_pcl)>1 and len_pcl_pt==len_pcl_rgb and t_pcl[-1]-t_pcl[0]>=dt:
                # print("mk3")
                for j in range(0,len(t_pcl)-1):
                    # print("mk4")
                    if j<len(t_pcl) and t_pcl[-1]-t_pcl[j+1]<dt:  #t_pcl[-1]-t_pcl[j]>=dt and 
                        print('pos est time intevel:',t_pcl[-1]-t_pcl[j])
                        time_itv_if = 1
                        break

            if time_itv_if ==1:
                time_itv_if = 0
                pcl_c2=pcl_h[-1]
                pcl_c1=pcl_h[j]
                rgb_c2=rgb_h[-1]
                rgb_c1=rgb_h[j]
                

                
                t_c2=t_pcl[-1]
                t_c1=t_pcl[j]
                pos_c2=pos_h[-1]
                pos_c1=pos_h[j]
                
                pcl_h=pcl_h[j+1::]
                rgb_h=rgb_h[j+1::]
                t_pcl=t_pcl[j+1::]
                pos_h=pos_h[j+1::]

    
            if len(pcl_c2) and len(pcl_c1):# and len(index_c1)==len(pcl_c1) and len(index_c2)==len(pcl_c2) and index_c1.any() and index_c2.any():

                db2 = skc.DBSCAN(eps=r_p, min_samples=n_p).fit(pcl_c2)

                labels2 = db2.labels_

                n_clusters2_ = len(set(labels2)) - (1 if -1 in labels2 else 0)
                if n_clusters2_== 0:
                    continue

                i=0
                for i in range(n_clusters2_):
                    one_cluster = pcl_c2[labels2 == i]
                    rgb_cluster = rgb_c2[labels2 == i]
                    clu_p2.append(one_cluster)
                    
                    rgb_p2.append(rgb_cluster)
                    clts_center.append(np.mean(one_cluster,axis=0))

                pcl_clust = copy.copy(clu_p2)
                clu_c1=np.array(clu_c1)
                clu_c2=np.array(clu_c2)
                fea_1=np.array(fea_1)
                fea_2=np.array(fea_2)

                # if clu_c1[0].shape
                remove_index = convert.romove_occluded(clu_p2,clts_center,local_pos)
                print('number of obstacles this frame & removed:',n_clusters2_,len(remove_index))

                # time11=time.time()
                no_cdyn0=2
                i=0

                jj = 0
                not_dyn_pt = list(range(0,n_clusters2_))
                max_displ = max((t_c2-t_c1)*max_obs_speed*vel_infl,v_size)
                t_track1 = time.time()
                inv_flag = abs(inv_flag-1)
                if_sta = 0
                if convert.if_color_depth_align and len(clts_center):
                    convert.if_color_depth_align = False
                    convert.color_img1 = convert.CvImg.imgmsg_to_cv2(convert.color_img, "bgr8")

                    convert.depth_img1 = convert.CvImg.imgmsg_to_cv2(convert.depth_img, "16UC1")
                    cams = c_track.get_target_xyz (convert.color_img1,convert.depth_img1)
                    targets = []
                    matched = []
                    print("returned target len:",len(cams),cams)
                    for i in range(len(cams)):
                        if sum(cams[0])==0:
                            break
                        body_x = cams[i][2]+xb  #distance from camera to uav center, xb yb zb are offset
                        body_y = -cams[i][0]+yb
                        body_z = -cams[i][1]+zb
                        target =np.matmul(b2e,np.array([body_x,body_y,body_z]).T).T+local_pos1# convert coordinate from B to E
                        print("found target!",target)
                        targets.append(target)
                        matched.append(False)

                for i in range(n_clusters2_):
                    if i in remove_index :
                        # inv_flag = abs(inv_flag-1)
                       
                        continue
                    pos_obs2 = np.array([min(clu_p2[i][:,0]),min(clu_p2[i][:,1]),min(clu_p2[i][:,2])])
                    pos_obs2_up = np.array([max(clu_p2[i][:,0]),max(clu_p2[i][:,1]),max(clu_p2[i][:,2])])
                    bb_x = [pos_obs2[0]-max_displ, pos_obs2_up[0]+max_displ]
                    bb_y = [pos_obs2[1]-max_displ, pos_obs2_up[1]+max_displ]
                    bb_z = [max(0,pos_obs2[2]-max_displ),pos_obs2_up[2]+max_displ]
                    index_pts_f1 = np.where((bb_x[1]>pcl_c1[:,0]) & (pcl_c1[:,0]>bb_x[0]) & (bb_y[1]>pcl_c1[:,1])&(pcl_c1[:,1]>bb_y[0])&(bb_z[1]>pcl_c1[:,2])&(pcl_c1[:,2]>bb_z[0]))
                    points_f1=pcl_c1[index_pts_f1]
                    pts_sta_bb=[]
                    if len(pcl_sta_h) > 0:
                        pts_sta = np.array(pcl_sta_h)
                        index_pts_sta = np.where((bb_x[1]>pts_sta[:,0]) & (pts_sta[:,0]>bb_x[0]) & (bb_y[1]>pts_sta[:,1])&(pts_sta[:,1]>bb_y[0])&(bb_z[1]>pts_sta[:,2])&(pts_sta[:,2]>bb_z[0]))
                        pts_sta_bb = pts_sta[index_pts_sta]
                        if len(pts_sta_bb):
                            sta_index1 = np.rint((pts_sta_bb-np.array([[bb_x[0],bb_y[0],bb_z[0]]]))/v_size)
                            sta_index = np.unique(sta_index1,axis=0)
                            
                    if len(points_f1) >n_p: #np.linalg.norm(convert.ang_vel)<math.pi*0.2 and 
                        origin.append([bb_x[0],bb_y[0],bb_z[0]])
                        pos_obs2_bb = np.rint((pos_obs2-np.array([bb_x[0],bb_y[0],bb_z[0]]))/v_size)
                        vx_n = int((bb_x[1]-bb_x[0])/v_size)+2
                        vy_n = int((bb_y[1]-bb_y[0])/v_size)+2
                        vz_n = int((bb_z[1]-bb_z[0])/v_size)+2
                        # print("size:",[vx_n,vy_n,vz_n,ft_vet_len])
                        if max([vx_n,vy_n,vz_n]) > int(max_obs_size/v_size):
                            # inv_flag = abs(inv_flag-1)
                            continue
                        bb_space = np.zeros((vx_n,vy_n,vz_n,ft_vet_len))
                        bb_space1 = bb_space.copy()
                        obs_index2 = np.rint((clu_p2[i]-np.array([[bb_x[0],bb_y[0],bb_z[0]]]))/v_size)
                        index2, id2, num2 = np.unique(obs_index2,return_index = True,return_counts=True,axis=0)
                        if len(pts_sta_bb):
                            # sta_mask = (index2[:, None] != sta_index).any(-1).all(1)
                            # sta_mask = (np.sum(abs(index2[:, None] - sta_index),axis=2)>1).all(1)
                            sta_mask = (np.max(abs(index2[:, None] - sta_index),axis=2)>1).all(1)
                            if sum(sta_mask==False) > min(mask_thr*n_p,mask_thr*len(index2)) or sum(sta_mask)<0.5*n_p: #sum(sta_mask)<0.5*n_p:
                                print("remained dyn points too few",sum(sta_mask),clts_center[i])
                                # inv_flag = abs(inv_flag-1)
                                
                                continue
                            index2 = index2[sta_mask]
                            num2 = num2[sta_mask]
                            id2 = id2[sta_mask]
                       #     print("static masked!",sum(sta_mask==False),len(pts_sta_bb),index2,sta_index)
                            

                        colors2=np.zeros(len(index2))
                        for ind in range(len(index2)):
                          
                            color_index = np.where((obs_index2==index2[ind]).all(1))
      
                            colors2[ind] = np.mean(rgb_p2[i][color_index])
                        
                        index2 = index2.astype(int)
                        bb_space[index2[:,0],index2[:,1],index2[:,2],:] = np.clip(np.tile((center_r/v_size)/np.linalg.norm(index2-np.mean(index2,axis=0),axis=1),(ft_vet_len,1)).T,0.6,10)*np.array([wt_pnum*num2,colors2*wt_color]).T
                        bb_lst2.append(bb_space)
                    
                        obs_index1 = np.rint((points_f1-np.array([[bb_x[0],bb_y[0],bb_z[0]]]))/v_size)
                        index1, id1, num1 = np.unique(obs_index1,return_index = True, return_counts=True,axis=0)
                        
                        if len(pts_sta_bb):
                            # sta_mask = (index1[:, None] != sta_index).any(-1).all(1)
                            # sta_mask = (np.sum(abs(index1[:, None] - sta_index),axis=2)>1).all(1)
                            sta_mask = (np.max(abs(index1[:, None] - sta_index),axis=2)>1).all(1)
                            if sum(sta_mask==False) > min(mask_thr*n_p,mask_thr*len(index1)) or sum(sta_mask)<0.5*n_p:#sum(sta_mask) < min(mask_thr*n_p,mask_thr*len(index1)):
                                print("remained dyn points too few",sum(sta_mask),clts_center[i])
                                # inv_flag = abs(inv_flag-1)
                               
                                continue
                            index1 = index1[sta_mask]
                            num1 = num1[sta_mask]
                            id1 = id1[sta_mask]
                           # print("static masked!",sum(sta_mask==False))
                        
                        rgb_p1 = rgb_c1[index_pts_f1]
                        # colors1 = rgb_p1[id1].flatten()
                        
                        colors1=np.zeros(len(num1))
                        for ind in range(len(index1)):
                            color_index = np.where((obs_index1==index1[ind]).all(1))
                            # if len(color_index[0])>1:
                            #     print("more than one points:",len(color_index[0]))
                            colors1[ind] = np.mean(rgb_p1[color_index])
                        index1 = index1.astype(int)
                        bb_space1[index1[:,0],index1[:,1],index1[:,2],:] = np.clip(np.tile((center_r/v_size)/np.linalg.norm(index1-np.mean(index1,axis=0),axis=1),(ft_vet_len,1)).T,0.6,10)*np.array([wt_pnum*num1,colors1*wt_color]).T
                        # bb_lst1.append(bb_space1)
                        
                        disp_i = -convert.get_dplm(bb_space, bb_space1, pos_obs2_bb) * v_size
                        v_i = disp_i/(t_c2-t_c1)
                        
                        # c_dyn1.append(np.mean(clu_p2[i][:,0],axis=0))
                        out_velo_index = np.where(abs(v_i)>max_obs_speed*1.8)
    
                        v_i[out_velo_index] = 0.1
                        # if_in_xy = (clts_center[i][0:2] > np.array([0,1])).all() and (clts_center[i][0:2] < np.array([5.0,3.3])).all() and v_i[0]<0 and clts_center[i][2] > 0.4 and local_pos1[0]>-1 and local_pos1[0]<0  # 3
                       # if_in_xy = (clts_center[i][0:2] > np.array([0,-1.5])).all() and (clts_center[i][0:2] < np.array([5.5,0.3])).all() and v_i[0]<0 and clts_center[i][2] > 0.4 and local_pos1[0]>-0.5
                        if_in_xy = 1
                        if_target_in = False
                        for ii in range(len(targets)):
                            if_target_in = (pos_obs2-np.ones(3)*0.8<targets[ii]).all() and (pos_obs2_up+np.array([0.8,0.8,1.8])>targets[ii]).all()
                            print("try match",clts_center[i],targets[ii])
                            if if_target_in:
                                print("matched!",clts_center[i],targets[ii])
                                matched[ii] = True
                                break
                        if (not if_target_in) and np.linalg.norm(v_i) < 1.0*v_size/(t_c2-t_c1) and inv_flag and (len(sta_centers)==0 or (np.linalg.norm((np.array(sta_centers)-clts_center[i])[:,0:2],axis = 1) > 0.3).all()):
                            print("static obs!",clts_center[i],disp_i,v_i,t_c2-t_c1,"static obs num:",len(sta_centers))
                            if_sta = 1
                            sta_centers.append(clts_center[i])
                         
                        elif (not if_target_in) and np.linalg.norm(v_i) < 1.0*v_size/(t_c2-t_c1) and (not inv_flag) and (len(sta_centers)==0 or (np.linalg.norm((np.array(sta_centers)-clts_center[i])[:,0:2],axis = 1) < 1).any()):
                            pcl_sta=pcl_sta+clu_p2[i].tolist()
                            if_sta = 1
                            
                        elif if_in_xy and (if_target_in or (np.linalg.norm(v_i) < max_obs_speed*1.5 and len(out_velo_index[0])==0 and (np.linalg.norm(v_i[0:2])>0.1 and abs(v_i[2])/np.linalg.norm(v_i[0:2]) < 1.0))and pos_obs2[2] <0.7 and (-pos_obs2+pos_obs2_up)[2]>0.6 and (-pos_obs2+pos_obs2_up)[2]/max((-pos_obs2+pos_obs2_up)[0:2])>0.9): # 
                            
                            #c_dyn1.append((clts_center[i]+(pos_obs2+pos_obs2_up)/2)/2)
                            c_dyn1.append(clts_center[i])
                            v_dyn1.append(v_i)
                            obsd.append(-pos_obs2+pos_obs2_up)
                            # inv_flag = abs(inv_flag-1)
                        else:
                     	    if_sta = 1
                    	    sta_centers.append(clts_center[i])
                            print("estimated velocity out of constrain!",clts_center[i],v_i,-disp_i/v_size)
                            # inv_flag = abs(inv_flag-1)
                           
                    else:
                        if_sta = 1
                        sta_centers.append(clts_center[i])
                        print("no enough points in former frame!",len(points_f1))
                        # inv_flag = abs(inv_flag-1)
                
                if not if_sta:
                    inv_flag = abs(inv_flag-1)
                if len(c_dyn1):
                    t_track = time.time()-t_track1

                if len(c_dyn1):   
                    c_dynkf=c_dyn1
                    v_dynkf=v_dyn1
                    obsdkf=obsd
                    time_cap = time.time()
 
                        
            lastkf_c = copy.copy(c_dyn_l)
            lastkf_v = copy.copy(v_dyn_l)
            lastkf_d = copy.copy(d_dyn_l)
            p_mat_list = copy.copy(p_mat_l)
            c_dyn_l,v_dyn_l,d_dyn_l,if_track,p_mat_l=[],[],[],[],[]
            c_dyn_tk,v_dyn_tk,d_dyn_tk=[],[],[]
            x_mat=[]
            observe_tk, kf_tk = [],[]
           
            if_initial = 0
            if len(lastkf_c) or len(c_dynkf):

                if last_kftime:
                    kf_dt = time.time()-last_kftime
                else:
                    kf_dt = protime
                last_kftime = time.time()
                if len(lastkf_c)==0:
                    lastkf_c = np.array(c_dynkf)
                    lastkf_v = np.array(v_dynkf)
                    lastkf_d = np.array(obsdkf)
                    if_initial = 1
                    if_track += [0]*len(c_dynkf)
                    t_obs = np.r_[t_obs,np.ones(len(c_dynkf))*time.time()]
                    for i in range(len(c_dynkf)):
                        p_mat_list.append(np.mat([[2,0],[0,2]]))
                else:
                    lastkf_c = np.array(lastkf_c)+kf_dt*np.array(lastkf_v)
                    lastkf_v = np.array(lastkf_v)
                    lastkf_d = np.array(lastkf_d)
                if len(t_obs)==0:
                    t_obs = np.zeros(len(lastkf_c))
                if len(c_dynkf) and if_initial==0:
                    for i in range(len(lastkf_c)):
                        dis_gaps = np.linalg.norm((np.array(c_dynkf)-lastkf_c[i])[0:2],axis=1)
                        mingap_kf = min(dis_gaps)
                        if mingap_kf < 1.0:
                            index_kf = np.where(dis_gaps==mingap_kf)[0][0]
                          
                            print("position matched!",mingap_kf,np.linalg.norm(v_dynkf[index_kf]-lastkf_v[i]))
                            if  math.acos(convert.ang_cos(v_dynkf[index_kf],lastkf_v[i]))< 3.5 or (v_dynkf[index_kf] -lastkf_v[i]==0).all(): #np.linalg.norm(v_dynkf[index_kf]-lastkf_v[i])
                                
                                observe_tk.append(index_kf)
                                kf_tk.append(i)
                              
                                t_obs[i]=time.time()
                                lastkf_d[i] = obsdkf[index_kf]
                                print("kf tracking!",mingap_kf)
                                # track_flag = 1
                                if_track.append(1)
                            else:
                                if_track.append(0)
                        else:
                            if_track.append(0)
                    
                        
                    new_obs_index = list(set(np.arange(len(c_dynkf)))-set(observe_tk))  
                    if len(new_obs_index):
                       # print("kfnew obs added!",np.array(c_dynkf)[new_obs_index],np.array(v_dynkf)[new_obs_index])
                        lastkf_c = np.r_[lastkf_c,np.array(c_dynkf)[new_obs_index]]
                        lastkf_v = np.r_[lastkf_v,np.array(v_dynkf)[new_obs_index]]
                        lastkf_d = np.r_[lastkf_d,np.array(obsdkf)[new_obs_index]]
                        if_track += [0]*len(new_obs_index)
                        t_obs = np.r_[t_obs,np.ones(len(new_obs_index))*time.time()]
                        for i in range(len(new_obs_index)):
                            p_mat_list.append(np.mat([[2,0],[0,2]]))
                else:
                    if_track = len(lastkf_c)*[0]
            
           # print("lastkf_c",lastkf_c,t_obs)
            if not (len(t_obs) == len(lastkf_c) == len(if_track) == len(lastkf_d) == len(lastkf_v) == len(p_mat_list)):
                print("shut down!", len(t_obs) , len(lastkf_c) ,len(if_track), len(lastkf_d), len(lastkf_v),len(p_mat_list))
                break
            for kk in range(len(lastkf_c)):
                del_which = kk-(len(lastkf_c)-len(if_track))
                if time.time()-t_obs[del_which] > kf_predtime:
                    
                    print("kftimeout obstacle!",len(lastkf_c),kk,time.time(),time.time()-t_obs[del_which],lastkf_c[kk],lastkf_v[kk])
                    del if_track[del_which]
                    t_obs = np.delete(t_obs,del_which)
                    continue
                # set initial value
                if kk in kf_tk:
                    obv_index = observe_tk[kf_tk.index(kk)]
                    z_mat = np.mat([c_dynkf[obv_index],v_dynkf[obv_index]])
                    # p_mat = np.mat([[2, 0], [0, 2]])
                    
                else:
                    z_mat = np.mat([lastkf_c[kk],lastkf_v[kk]])

                p_mat = p_mat_list[kk]
                x_mat =  np.mat([lastkf_c[kk]-kf_dt*lastkf_v[kk],lastkf_v[kk]])

                q_mat = np.mat([[0.01, 0], [0, 0.01]])
                
                f_mat = np.mat([[1, kf_dt], [0, 1]])     # State transition matrix
              
#                                    q_mat = np.mat([[0.1, 0], [0, 0.1]])
               
                h_mat = np.mat([[1, 0],[0,1]])  #State observation matrix
                r_mat = np.mat([3])     #State observation noise Covariance matrix
 
                x_predict = f_mat * x_mat
                p_predict = f_mat * p_mat * f_mat.T + q_mat
                kalman = p_predict * h_mat.T / (h_mat * p_predict * h_mat.T + r_mat)
                x_mat = x_predict + kalman *(z_mat - h_mat * x_predict)
                p_mat = (np.eye(2) - kalman * h_mat) * p_predict
                
                
                if len(c_dyn_l) == 0 or min(np.linalg.norm(np.array(x_mat)[0]-np.array(c_dyn_l),axis=1)) > 0.3:
                    c_dyn_l.append(list(np.array(x_mat)[0]))
                    v_dyn_l.append(list(np.array(x_mat)[1]))
                    d_dyn_l.append(lastkf_d[kk])
                    p_mat_l.append(p_mat)
                    if kk in kf_tk:
                        c_dyn_tk.append(list(np.array(x_mat)[0]))
                        v_dyn_tk.append(list(np.array(x_mat)[1]))
                        d_dyn_tk.append(lastkf_d[kk])
                else:
                    del if_track[del_which]
                    t_obs = np.delete(t_obs,del_which)
               # print("KF is used!!")
#                                    c_dynkf=c_dyn_l
#                                    v_dynkf=v_dyn_l
                x_mat = []
                dyn_obs_reset=0
            # if len(c_dyn_l)>0:
            print('moving obj!!!!!!!!:',c_dyn_l,v_dyn_l,d_dyn_l,if_track)#,'pcl1:',pcl1)
            c_dyn1=np.array(c_dyn_l)
            v_dyn1=np.array(v_dyn_l)
            
            obsd=np.array(d_dyn_l)
            #obsd[np.where(obsd<0.5)]=0.5
            c_dyn11=[]
            dyn_v11=[]

            pcl1 = []
            if len(c_dyn1)>0:
                pcl1 = np.array([[1,2,3]])
                for kk in range(len(clts_center)):
                    if (np.linalg.norm(c_dyn1-clts_center[kk],axis = 1) < 0.2).any():
			pcl1 = np.r_[pcl1,np.array(clu_p2[kk][::2])]
                        continue
                  
                    pcl1 = np.r_[pcl1,np.array(clu_p2[kk])]
                pcl1 = pcl1[1::]
            elif len(pcl_h):
                pcl1 = pcl_h[-1] #pcl_c2 #[::4]
            if len(pcl1) > pub_pcl_size:
                
                pcl1 = pcl1[np.random.choice(len(pcl1), pub_pcl_size, replace=False)]
            if len(pcl1)>0:
               
                pcl1=pcl1-local_pos
                
                if len(pcl_sta_h) > 0:
                    
                    pcl1=np.r_[pcl1,np.array(pcl_sta_h[-pub_pcl_size::])[::5]-local_pos] #int(len(pcl_sta_h)/5)+1
                
                pcl1=convert.distance_filter(pcl1,8)
                pcl1=pcl1+local_pos
                
            if len(c_dyn1)>0 and len(obsd)>0 and len(pcl1)>0:
                pcl1=np.r_[pcl1,c_dyn1]
                pcl1=np.r_[pcl1,v_dyn1]
               
                pcl1=np.r_[pcl1,obsd]
            if len(pcl1)>0:
             #   if (pcl_c[:,2]<0.3).any():
                if len(pcl_c):    
                    pcl1 = np.r_[pcl1,pcl_c[pcl_c[:,2]<0.3][::10]]
                pcl1=np.r_[pcl1,np.array([[len(c_dyn1),0,0]])] #attach the number of dynamic obstacles
                
            else:
                pcl1=np.array([[0.0,0.0,0.0]])

            point_al=convert.xyz_array_to_pointcloud2(np.array(pcl1),'map',convert.pcl_timestamp)

            if len(pcl1) and point_al is not None:
                # print(point2)
                convert.alpub.publish(point_al)
                print("pcl_all length",len(pcl1))
            if len(pcl_c):
                point_global=convert.xyz_array_to_pointcloud2(np.array(pcl_c),'map',convert.pcl_timestamp)
                convert.globalpub.publish(point_global)
            point_sta=convert.xyz_array_to_pointcloud2(np.array(pcl_sta_h),'map',rospy.Time.now())
            if point_sta is not None:
              
                convert.sta_pub.publish(point_sta)

            protime=time.time()-starttime1
          
            print('protime,t_track:',protime,t_track)
            if point22 is not None:
             
                convert.octo_pub.publish(point22)
            if len(clts_center):
                convert.cluster_pub(clts_center)
            if c_dyn1 !=[] and v_dyn1 !=[] and len(obsd) != 0:
                print("if_track",if_track)
                convert.publish_dyn_obs(c_dyn1,v_dyn1,obsd,max_displ,if_track)
           #     convert.publish_dyn_obs_pyramids(c_dyn1,v_dyn1,obsd,max_displ,if_track,local_pos)
                #convert.pub_target(targets,matched)
            # if c_dyn !=[]:
            #     convert.publish_potential_dyn_obs(c_dyn,dyn_v,dyn_d)
            else:
                rate.sleep()

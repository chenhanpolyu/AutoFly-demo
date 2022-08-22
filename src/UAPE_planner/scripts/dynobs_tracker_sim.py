#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
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
class convert_pcl():
    def callback(self,data):
        #y.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        assert isinstance(data, PointCloud2)
        # global point2,pos,pub
        self.pcl = [list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)),list(point_cloud2.read_points(data, field_names=("rgb")))]
        self.pcl_time =data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        
    def pos_pcl(self,pcl,pos,vel):
        # self.pos=pos.pose
        self.pos=pos
        self.pos_time = pos.header.stamp.secs + pos.header.stamp.nsecs * 1e-9
#        assert isinstance(pcl, PointCloud2)
        # global point2,pos,pub
        self.pcl = [list(point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True)),list(point_cloud2.read_points(pcl, field_names=("rgb")))]
        self.pcl_time =pcl.header.stamp.secs + pcl.header.stamp.nsecs * 1e-9
        self.pcl_timestamp = pcl.header.stamp
        self.ang_vel = np.array([vel.twist.angular.x,vel.twist.angular.y,vel.twist.angular.z])
        self.line_vel = np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z])
        self.vel_time = vel.header.stamp.secs + vel.header.stamp.nsecs * 1e-9
        print("alighed",self.vel_time,self.pcl_time)
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
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        self.if_align = 0
        rospy.init_node('point_transfrer', anonymous=True)
        self.dyn_publisher = rospy.Publisher("/dyn", MarkerArray, queue_size=1)
        self.potential_dyn_publisher = rospy.Publisher("/potential_dyn", MarkerArray, queue_size=1)

    
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
            dynobs.type = Marker.SPHERE
            dynobs.pose.position.x = c_dyn1[m][0]
            dynobs.pose.position.y = c_dyn1[m][1]
            dynobs.pose.position.z = c_dyn1[m][2]
            dynobs.id=3*m
            dynobs.scale.x = obsd[m][0]
            dynobs.scale.y = obsd[m][1]
            dynobs.scale.z = obsd[m][2]
   
            dynobs.color.a = 0.6
            dynobs.color.r = 0.0
            dynobs.color.g = 0.9
            dynobs.color.b = 0.1  
       
            
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
            # dyn.markers.append(dynbbox)
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
    def ang_cos(self,c,b,a): #c-> the angle we need
        cb=np.linalg.norm(b-c)
        ca=np.linalg.norm(a-c)
        ab=np.linalg.norm(a-b)
        ang_cc=(cb**2+ca**2-ab**2)/(2*cb*ca)
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
if __name__ == '__main__':
    # global pub,rate,point2,pos

    convert=convert_pcl()
    convert.listener()
    # convert.vel=None
    convert.pos=None
    convert.octo_pcl=None
    convert.pcl=None
    convert.f=None
    convert.ang_vel=None
    rosrate=3
    point22=None
    c_dyn11=None
    dyn_v11=None
    local_pos=0
    octo_pcl=[]
    r0=0
    p0=0
    y0=0
    map_reso=0.2
    fac_dv=1.0  #dynamic obstacle velocity factor
    v_size = 0.15 #voxel size of the bounding box
    center_r = 0.3 # center ragion of the obstacle has higher weight 
    rate = rospy.Rate(rosrate) # 10hz
    dt=0.25
    max_obs_speed = 2.0
    n_p=12
    last_kftime = 0
    wt_pnum = 1.0
    wt_color = 4*1e39
    ft_vet_len = 2
    xb,yb,zb = 0.12,0,0.00
    static_pcl_len = 300
    kf_predtime = 0.5
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
#    c_dyn0=[]
#    dyn_v0=[]
#    d_dyn0=[]
    c_dynkf=[]
    v_dynkf=[]
    octo_pcl1=[]
    time22=0
    time_cap=0
    pre_ct=0
    dyn_obs_reset=1
    c_dyn_l,v_dyn_l,d_dyn_l=[],[],[]
    pcl_sta,pcl_sta_h=[],[]
    t_obs = []
    fake_obs_num = 20

    while not rospy.is_shutdown():
        c_dyn1 = (np.random.rand(fake_obs_num,3)+np.array([0,-0.5,0.5])) * np.array([20,10,0.2])
        v_dyn1 = (np.random.rand(fake_obs_num,3)+np.array([-0.5,-0.5,-0.5])) * np.array([5,5,0.1])
        obsd = np.array([[0.6,0.6,1.8]]*fake_obs_num)
        max_displ = 0.4
        if c_dyn1 !=[] and v_dyn1 !=[] and len(obsd) != 0:
            # print("if_track",if_track)
            # convert.publish_dyn_obs(c_dyn1,v_dyn1,obsd,max_displ,if_track)
            convert.publish_dyn_obs(c_dyn1,v_dyn1,obsd,max_displ,1)
        # if c_dyn !=[]:
        #     convert.publish_potential_dyn_obs(c_dyn,dyn_v,dyn_d)
        # else:
        rate.sleep()

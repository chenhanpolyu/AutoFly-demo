#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

nav_msgs::Path point()
{
    // point parameters
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    double h = 1.0;
    double scale = 0.5;
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 1.0;

    // /* loop 1 */
    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = 1.5;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = -1.5;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt); 

    // /* loop 2 */
    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = 1.5;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = -1.5;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt); 

    // /* loop 3 */
    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = 1.5;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt);  

    pt.pose.position.x = -1.5;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 1.0;
    waypoints.poses.push_back(pt); 

    // /* loop 1 */
    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 0;
    // pt.pose.position.z =  0.5;
    // waypoints.poses.push_back(pt);   

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 1.0;
    // pt.pose.position.z =  1.0;
    // waypoints.poses.push_back(pt);    

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 1.0;
    // pt.pose.position.z =  1.5;
    // waypoints.poses.push_back(pt);   

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 0.0;
    // pt.pose.position.z =  2.0;
    // waypoints.poses.push_back(pt);  

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * -1.0;
    // pt.pose.position.z =  1.5;
    // waypoints.poses.push_back(pt); 

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * -1.0;
    // pt.pose.position.z =  1.0;
    // waypoints.poses.push_back(pt); 

    // /* loop 2 */
    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 0;
    // pt.pose.position.z =  0.5;
    // waypoints.poses.push_back(pt);   

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 1.0;
    // pt.pose.position.z =  1.0;
    // waypoints.poses.push_back(pt);    

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 1.0;
    // pt.pose.position.z =  1.5;
    // waypoints.poses.push_back(pt);   

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 0.0;
    // pt.pose.position.z =  2.0;
    // waypoints.poses.push_back(pt);  

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * -1.0;
    // pt.pose.position.z =  1.5;
    // waypoints.poses.push_back(pt); 

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * -1.0;
    // pt.pose.position.z =  1.0;
    // waypoints.poses.push_back(pt); 

    // /* loop 3 */
    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 0;
    // pt.pose.position.z =  0.5;
    // waypoints.poses.push_back(pt);   

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 1.0;
    // pt.pose.position.z =  1.0;
    // waypoints.poses.push_back(pt);    

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 1.0;
    // pt.pose.position.z =  1.5;
    // waypoints.poses.push_back(pt);   

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 0.0;
    // pt.pose.position.z =  2.0;
    // waypoints.poses.push_back(pt);  

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * -1.0;
    // pt.pose.position.z =  1.5;
    // waypoints.poses.push_back(pt); 

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * -1.0;
    // pt.pose.position.z =  1.0;
    // waypoints.poses.push_back(pt); 

    // /* loop 4 */
    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * -0.0;
    // pt.pose.position.z =  0.5;
    // waypoints.poses.push_back(pt); 

    // pt.pose.position.y =  offset_y;
    // pt.pose.position.x =  scale * 1.0;
    // pt.pose.position.z =  1.0;
    // waypoints.poses.push_back(pt); 

    // Return
    return waypoints;
}

// Circle trajectory
nav_msgs::Path circle()
{
    double h = 1.0;
    double scale = 1.0;
    double offset_x = -1.0;
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);    
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);     
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);    
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 2.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y = 1.0 * scale;
    pt.pose.position.x = 1.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = 0.0 * scale;
    pt.pose.position.x = 0.0 * scale + offset_x;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);    

    // Return
    return waypoints;
}

// Figure 8 trajectory
nav_msgs::Path eight()
{
    // Circle parameters
    double offset_x = -1.0;
    double offset_y = 0.0;
    double offset_z = 1.0;
    double rx = 0.7;
    double ry = 1.0;
    double h = 0.0;
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);    

    for(int i=0; i< 3; ++i)
    {
        // First loop
        pt.pose.position.x =  rx + offset_x;
        pt.pose.position.y = -ry + offset_y;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  rx*2 + offset_x;
        pt.pose.position.y =  0 + offset_y ;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  rx*3 + offset_x;
        pt.pose.position.y =  ry + offset_y ;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  rx*4 + offset_x;
        pt.pose.position.y =  0 + offset_y ;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  rx*3 + offset_x;
        pt.pose.position.y = -ry + offset_y ;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  rx*2 + offset_x;
        pt.pose.position.y =  0 + offset_y ;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  rx + offset_x;
        pt.pose.position.y =  ry  + offset_y;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0  + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);
        // Second loop
        pt.pose.position.x =  rx + offset_x;
        pt.pose.position.y = -ry + offset_y;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  rx*2 + offset_x;
        pt.pose.position.y =  0 + offset_y ;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  rx*3 + offset_x;
        pt.pose.position.y =  ry + offset_y ;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  rx*4 + offset_x;
        pt.pose.position.y =  0 + offset_y ;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  rx*3 + offset_x;
        pt.pose.position.y = -ry + offset_y ;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  rx*2 + offset_x;
        pt.pose.position.y =  0 + offset_y ;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  rx + offset_x;
        pt.pose.position.y =  ry  + offset_y;
        pt.pose.position.z =  h+offset_z;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0  + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h/2+offset_z;
        waypoints.poses.push_back(pt);
        // Third loop
    }
    return waypoints;   
}  
#endif

/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/

/** 
 * @file
* @brief The documentation of this file is a responsibility of its current developer, Pedro Salvado.
*/

#include <atlascar_base/odometry.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/JointState.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

//SAVE FILE
#include <stdio.h>
#include <fstream>
#include <iostream>
//      FILE * pFile;

#define PFLN {printf("%s %d\n",__FILE__, __LINE__);}
bool new_status_message;
atlascar_base::AtlascarStatus base_status;
atlascar_base::AtlascarVelocityStatus base_velocity;
bool message_receive=false;
bool bool_init=true;
ros::Time t_i, t_i_1;
int contador_=0,contador_1=0;

void VelocityMessageHandler(const atlascar_base::AtlascarVelocityStatus& msg)
{
	base_velocity=msg;
       new_status_message=true;
       if(bool_init)
       {
              t_i_1=msg.header.stamp;
              t_i=msg.header.stamp;
              bool_init=false;
       }else
       {
              t_i_1=t_i;
              t_i=msg.header.stamp;
      }
}
void StatusMessageHandler(const atlascar_base::AtlascarStatus& msg)
{
       base_status=msg;
}

int main(int argc, char** argv)
{
       ros::init(argc, argv, "atlascar_odometry");
       ros::NodeHandle n;
       ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/vhc/odometry", 50);
//        ros::Publisher odom_pub_no_filter = n.advertise<nav_msgs::Odometry>("/atc/base/odometry_no_filter", 50);
       tf::TransformBroadcaster odom_broadcaster;

//	Subscriber subscribeStatusMessages = n.subscribe ("/atc/base/status/plc", 1, StatusMessageHandler);
       Subscriber subscribeStatusMessages = n.subscribe ("/vhc/plc/status", 1, StatusMessageHandler);
       Subscriber subscribeVelocityStatusMessages = n.subscribe ("/vhc/velocity/status", 1, VelocityMessageHandler);
//	Subscriber subscribeVelocityStatusMessages = n.subscribe ("/velocity_status", 1, VelocityMessageHandler);

       double x = 0.0;
       double y = 0.0;
       double yaw = 0.0;

       double vx = 0.0;
       double vy = 0.0;
       double vth = 0.0;

       double vl,phi;
       double l;

       n.param("wheel_base",l,DEFAULT_WHEEL_BASE);
       
       new_status_message=false;

       ros::Rate r(20.0);

       ROS_INFO("Starting to spin ...");

       //KALMAN FILTER
       non_holonomic_ekfilter filter;

       Vector z_i(2);
       Vector u(0);
       filter.SetWheelBase(l);

       nav_msgs::Odometry odom;

	   double phi_tmp=0;
	   
       while(n.ok())
       {
              spinOnce();   
              r.sleep();

              if(!new_status_message)
                     continue;

              new_status_message=false;
              message_receive=false;

              
              vl=base_velocity.velocity;
              phi=base_status.steering_wheel;

              
              if (phi ==0)
                     phi=phi_tmp;
              else
                     phi_tmp=phi;   
              
              
	  phi= phi +0.28801; //liceu

		if (phi <=0)
		{
			phi = phi*0.766;
			phi=phi*-1;
		}
		else if (phi>0)
		{
			phi= phi*0.770;
			phi=phi*-1;
		}
		//corrrection velocity car like robot
		double w=1.27;
		double instant_radious=fabs(l/tan(phi));
		double v_corrected;
		if(phi>=0)
			 v_corrected=vl*(instant_radious/(instant_radious-w/2));
		else	
			
			 v_corrected=vl*(instant_radious/(instant_radious+w/2));
              //kalmen input measures
              z_i(1)=v_corrected;
              z_i(2)=phi;



              //v
              double dt = (t_i - t_i_1).toSec();
              double delta_x;
              double delta_y;
              double delta_yaw;

             
		delta_x=cos(yaw)*cos(phi)*v_corrected*dt;
		delta_y=sin(yaw)*cos(phi)*v_corrected*dt;
		//delta_yaw=sin(phi/l)*v_corrected*dt; //erro no l
		delta_yaw=sin(phi)*v_corrected*dt/l;

              
              x += delta_x;
              y += delta_y;
              yaw += delta_yaw;

//  
              //KALMAN FILTER
              filter.SetTimeInterval(dt);
              filter.step(u,z_i);

              Vector x_estimated=filter.getX();
              Matrix covariance_matrix =filter.calculateP();
              //end KALMAN FILTER

              //since all odometry is 6DOF we'll need a quaternion created from yaw
              geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(x_estimated(3));
// 
//               //first, we'll publish the transform over tf
//               geometry_msgs::TransformStamped odom_trans;
//               odom_trans.header.stamp = t_i;
// 
//               odom_trans.header.frame_id = "/world";
//               odom_trans.child_frame_id = "/atc/vehicle/rear_axis";
// 
//               odom_trans.transform.translation.x = x_estimated(1);
//               odom_trans.transform.translation.y = x_estimated(2);
//               odom_trans.transform.translation.z = 0.0;
//               odom_trans.transform.rotation = odom_quat;

                              //file save to file

              
              //send the transform
//               odom_broadcaster.sendTransform(odom_trans);

              //next, we'll publish the odometry message over ROS
              nav_msgs::Odometry odom;
              odom.header.stamp =t_i;
              odom.header.frame_id = "/world";

              //set the position
              odom.pose.pose.position.x = x_estimated(1);
              odom.pose.pose.position.y = x_estimated(2);
              odom.pose.pose.position.z = 0.0;
              odom.pose.pose.orientation = odom_quat;
              odom.pose.covariance[0]=covariance_matrix(1,1);
              odom.pose.covariance[7]=covariance_matrix(2,2);
              odom.pose.covariance[35]=covariance_matrix(3,3);
              //cout << "possss      " << odom.pose.pose.position.x << endl; 
              
              //set the velocity
              odom.child_frame_id = "/atc/vehicle/rear_axis";
              odom.twist.twist.linear.x = vx;
              odom.twist.twist.linear.y = vy;
              odom.twist.twist.angular.z = vth;


              //publish the message
              odom_pub.publish(odom);
              
              
              
              
              /*
              //no filter
               nav_msgs::Odometry odom_no_filter;
              odom_no_filter.header.stamp =t_i;
              odom_no_filter.header.frame_id = "/world";

              //set the position
              odom_no_filter.pose.pose.position.x = x;
              odom_no_filter.pose.pose.position.y = y;
              odom_no_filter.pose.pose.position.z = 0.0;
              odom_no_filter.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
              odom_no_filter.pose.covariance[0]=covariance_matrix(1,1);
              odom_no_filter.pose.covariance[7]=covariance_matrix(2,2);
              odom_no_filter.pose.covariance[35]=covariance_matrix(3,3);
              
              
              //set the velocity
              odom.child_frame_id = "/atc/vehicle/rear_axis";
              odom.twist.twist.linear.x = vx;
              odom.twist.twist.linear.y = vy;
              odom.twist.twist.angular.z = vth;


              //publish the message
              odom_pub_no_filter.publish(odom_no_filter);*/
       }
}

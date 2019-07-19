#include <ros/ros.h> 
#include <stdio.h>   
#include <math.h>


#include <tf/tf.h> 
#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Odometry.h>   

#include <iostream> 
#include <fstream>  

#include <ros/time.h> 
#include <ros/duration.h>   
#include <ctime> 

#include <algorithm> 


using namespace std; 


nav_msgs::Odometry curPose;  









float waypoint_x[6] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00}; 
float waypoint_y[6] = {1.00, 2.00, 3.00, 4.00, 5.00, 6.00}; 


float deltaX, deltaY; 
float eucl_distance; 
float alpha;

double roll, pitch, yaw; 


FILE *myfile = fopen("data2.txt", "w");  



void odomCallback (const nav_msgs::Odometry::ConstPtr & msg){   

		tf::Quaternion q(

        msg->pose.pose.orientation.x,

        msg->pose.pose.orientation.y,

        msg->pose.pose.orientation.z,

        msg->pose.pose.orientation.w);

		tf::Matrix3x3 m(q);

		m.getRPY(roll, pitch, yaw);   

		cout<<""<<endl;

		cout<<"Seq: "<< msg->header.seq<<endl; 

		cout<<"Position-> X ="<< msg->pose.pose.position.x << " Y = "<< msg->pose.pose.position.y<< endl;

   	cout<<"Orientation:" << " = " << yaw << " radians" <<endl; 

		cout<<"Velocity: Linear.X = " <<msg->twist.twist.linear.x<< " Angular.Z = "<< 		msg->twist.twist.angular.z<< endl;  

		curPose.pose.pose.position.x = msg->pose.pose.position.x; 
	
		curPose.pose.pose.position.y =  msg->pose.pose.position.y; 

		curPose.pose.pose.orientation.z = yaw;  


		

	return;
	
} 




int main(int argc, char**argv){ 

	int i = 0;  

	int N = 5;


	ros::init(argc, argv, "movetoGoal2"); 

	ros::NodeHandle node;   

	ros::Subscriber sub = node.subscribe("odom", 1000, &odomCallback);    

	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 

	ros::Rate loop_rate(10); 

	geometry_msgs::Twist vel_msg; 

	fprintf(myfile, "TIME					X					Y	   Orient.Z     linError\n");


	while(ros::ok() && i <= N ){   

		ros::spinOnce(); 

		double secs = ros::Time::now().toSec();  


		fprintf(myfile, "%lf		%f			%f 			%f \n",secs, curPose.pose.pose.position.x, curPose.pose.pose.position.y, yaw);


		
		//computing the configuration error


		deltaX = waypoint_x[i] - curPose.pose.pose.position.x; 
	
		deltaY = waypoint_y[i] - curPose.pose.pose.position.y;  

		alpha = atan2f(deltaY, deltaX) - curPose.pose.pose.orientation.z;

		eucl_distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2)) * (cos(alpha));


		//send the velocity commands 


		vel_msg.linear.x = 0.5 * eucl_distance ; 

		vel_msg.angular.z = 0.5 * alpha ;  


			if(eucl_distance <= 0.1){ 

				i = i + 1 ;  

				printf("X = %d ", i);
			
		
		}  

			else{ 
			
				pub.publish(vel_msg); 

				}



		loop_rate.sleep();


		}




}









		


















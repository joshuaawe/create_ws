#include <ros/ros.h> 
#include <tf/tf.h> 
#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Odometry.h>   
#include <iostream> 
#include <fstream>  

#include <ros/time.h> 
#include <ros/duration.h>   
#include <ctime>


using namespace std; 


nav_msgs::Odometry curPose;  


float goalPose_x = 1.0000; //[m] 

float goalPose_y = -1.0000; //[m]  

float linError, angError;

float deltaX, deltaY, alpha;  

double roll, pitch, yaw;

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg);

float getAngError(nav_msgs::Odometry curpose); 

float getLinError(nav_msgs::Odometry curpose);   

clock_t start; 
double duration;  

double secs_1; 



 

FILE *myfile = fopen("data2.txt", "w");  



int main(int argc, char **argv){  

	ros::init(argc, argv, "movetoGoal1"); 

	ros::NodeHandle node;   

	ros::Subscriber sub = node.subscribe("odom", 1000, &odomCallback);    

	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 

	ros::Rate loop_rate(10); 

	geometry_msgs::Twist vel_msg;  
 
	fprintf(myfile, "TIME					X					Y					Orient.Z     linError\n");





	while (ros::ok() ){	 

		ros::spinOnce();  

		linError = getLinError(curPose); 
	
		angError = getAngError(curPose);  

		cout<<"Linear Error = " <<linError <<", Angular Error = " <<angError<< endl; 
	
		double secs = ros::Time::now().toSec();  


		fprintf(myfile, "%lf		%f			%f 			%f       %f \n",secs, curPose.pose.pose.position.x, curPose.pose.pose.position.y, yaw, linError);

		
		if (linError > 0.1){   


			vel_msg.linear.x = 0.8 * linError;  

			vel_msg.angular.z = 0.9 * angError; 
		
			pub.publish(vel_msg);


		} 


		else { 


			vel_msg.linear.x = 0;	

			vel_msg.angular.z = 0;
 
			pub.publish(vel_msg); 

			break;
			
			

		}   


		loop_rate.sleep(); 

	}


}   

void odomCallback (const nav_msgs::Odometry::ConstPtr & msg){   

	cout<<""<<endl;

	cout<<"Seq: "<< msg->header.seq<<endl; 

	cout<<"Position-> X ="<< msg->pose.pose.position.x << " Y = "<< msg->pose.pose.position.y<< endl;


	tf::Quaternion q(

        msg->pose.pose.orientation.x,

        msg->pose.pose.orientation.y,

        msg->pose.pose.orientation.z,

        msg->pose.pose.orientation.w);

	tf::Matrix3x3 m(q);

	//Converting to ROLL-PITCH-YAW (RADIANS)

	m.getRPY(roll, pitch, yaw);
   

	cout<<"Orientation:" << " = " <<yaw<< " radians" <<endl; 

	cout<<"Velocity -> Linear.X = " <<msg->twist.twist.linear.x<< " Angular.Z = "<< 		msg->twist.twist.angular.z<< endl;  

	curPose.pose.pose.position.x = msg->pose.pose.position.x; 
	
	curPose.pose.pose.position.y =  msg->pose.pose.position.y; 

	curPose.pose.pose.orientation.z = yaw;  


	return;
	
} 



float getAngError(nav_msgs::Odometry curpose) { // ANGULAR ERROR FOR P CONTROL

	curpose.pose.pose.orientation.z = yaw; 

	deltaX = goalPose_x - curpose.pose.pose.position.x; 
	
	deltaY = goalPose_y - curpose.pose.pose.position.y;

	alpha = atan2f(deltaY, deltaX) - curpose.pose.pose.orientation.z ;

	return alpha ;

}

float getLinError(nav_msgs::Odometry curpose){  	// LINEAR ERROR FOR P CONTROL

	curpose.pose.pose.orientation.z = yaw; 
	
	deltaX = goalPose_x - curpose.pose.pose.position.x; 
	
	deltaY = goalPose_y - curpose.pose.pose.position.y;

	alpha = atan2f(deltaY, deltaX) - curpose.pose.pose.orientation.z;

	float distance = sqrt( pow(deltaX, 2) + pow(deltaY, 2)) * (cos(alpha)); 

	return distance;

}







#include "ros/ros.h"
#include <cmath>
#include "my_odom.cpp"
#include "robot_msgs/Motor.h"
#include "geometry_msgs/Twist.h"
#define WHEEL 0.135/2
#define DIST 0.15
#define RPM_to_RADIAN 3.141592654*2/60
#define RADIAN_to_RPM 60/(3.141592654*2)
void CallBack_1(const robot_msgs::Motor &msg);
void CallBack_2(const geometry_msgs::Twist &msg);

geometry_msgs::Twist robot_state;

ros::Publisher *pub1,*pub2;
int main(int argc,char**argv){
	ros::init(argc,argv,"ODOME_NODE");
	ros::NodeHandle nh;
	ros::Subscriber sub1 = nh.subscribe("/Motor/speed",1,CallBack_1);
	ros::Subscriber sub2 = nh.subscribe("/cmd_vel",1,CallBack_2);
	ros::Publisher speed1 = nh.advertise<geometry_msgs::Twist>("/Motor/Twist/speed",1);
	ros::Publisher speed2 = nh.advertise<robot_msgs::Motor>("/Motor/speed_set",1);
	pub1 = &speed1;
	pub2 = &speed2;
	while(ros::ok()){
		ros::spinOnce();
	}
	return 0;
}

void CallBack_1(const robot_msgs::Motor &msg){
	double left = msg.left.data*RPM_to_RADIAN;
	double right = msg.right.data*RPM_to_RADIAN;
	robot_state.linear.x = WHEEL*left/2 + WHEEL*right/2;
	robot_state.linear.y = 0;
	robot_state.linear.z = 0;
	robot_state.angular.x = 0;
	robot_state.angular.y = 0;
	robot_state.angular.z = -WHEEL/(2*DIST)*left + WHEEL/(2*DIST)*right;
	pub1->publish(robot_state);
}

void CallBack_2(const geometry_msgs::Twist &msg){
	robot_msgs::Motor motor_msgs;
	double right = (DIST*msg.angular.z + msg.linear.x)/WHEEL;
	double left = right - 2*DIST/WHEEL*msg.angular.z;
	motor_msgs.right.data = right*RADIAN_to_RPM*4;
	motor_msgs.left.data = left*RADIAN_to_RPM*4;
	ROS_INFO("MOTOR SPEED \tLEFT : %f, RIGHT : %f",motor_msgs.left.data,motor_msgs.right.data);
	pub2->publish(motor_msgs);
}
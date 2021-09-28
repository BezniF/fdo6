#include <ros/ros.h>
#include <stdio.h>
#include <chrono>

#include "dhdc.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

geometry_msgs::WrenchStamped wrench;

bool pressed = 0;

void wrench_callback(const geometry_msgs::WrenchStamped msg){wrench = msg;}

int main(int argc, char** argv) {

	ros::init(argc, argv, "fdo6_driver");

	ros::NodeHandle public_node_handler;
	ros::NodeHandle private_node_handler("~");

	ros::Publisher pose_pub = public_node_handler.advertise<geometry_msgs::PoseStamped>("fdo6/pose", 1);
	ros::Publisher twist_pub = public_node_handler.advertise<geometry_msgs::TwistStamped>("fdo6/twist", 1);
	ros::Publisher button_pub = public_node_handler.advertise<std_msgs::Bool>("fdo6/button", 1);

	ros::Subscriber wrench_sub = public_node_handler.subscribe("fdo6/wrench_cmd", 10, &wrench_callback);

	std::string base_frame_name, ee_frame_name;
	double update_frequency;

	private_node_handler.param<std::string>("base_frame_name", base_frame_name, "base");
	private_node_handler.param<std::string>("ee_frame_name", ee_frame_name, "ee");
	private_node_handler.param<double>("update_frequency", update_frequency, 1000.0);

	ros::Rate loop_rate(update_frequency);

//Variables
	double px, py, pz;
	double R[3][3];
	tf::Quaternion 	q;

	tf::Transform pose_frame;
	tf::TransformBroadcaster broadcaster;

	double vx, vy, vz;
	double wx, wy, wz;

	geometry_msgs::PoseStamped pose;
	geometry_msgs::TwistStamped twist;
	std_msgs::Bool butt_msg;

	wrench.wrench.force.x = 0.0;
	wrench.wrench.force.y = 0.0;
	wrench.wrench.force.z = 0.0;

	int done = 0;

//Trying opening device
	if (dhdOpen () < 0) {
		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
		dhdSleep (2.0);
		return -1;
	}
	else{
//Message
		int major, minor, release, revision;
		dhdGetSDKVersion (&major, &minor, &release, &revision);
		printf ("\n");
		printf ("*******************************************************************\n");
		printf ("Force Dimension\n");
		printf ("SDK Version: %d.%d.%d.%d\n", major, minor, release, revision);
		printf ("Device detected %s\n", dhdGetSystemName());
		printf ("ROS driver started\n");
		printf ("*******************************************************************\n");
	}

//Enable force
	dhdEnableForce (DHD_ON);

//Haptic loop
	while ((!done) && (ros::ok())) {

//Pose
		if (dhdGetPositionAndOrientationFrame(&px, &py, &pz, R) < DHD_NO_ERROR){
			printf ("error: cannot read pose (%s)\n", dhdErrorGetLastStr());
			done = 1;
		}

		tf::Matrix3x3(R[0][0],R[0][1],R[0][2],R[1][0],R[1][1],R[1][2],R[2][0],R[2][1],R[2][2]).getRotation(q);

		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = base_frame_name;
		pose.pose.position.x = px;
		pose.pose.position.y = py;
		pose.pose.position.z = pz;
		pose.pose.orientation.x = q.x();
		pose.pose.orientation.y = q.y();
		pose.pose.orientation.z = q.z();
		pose.pose.orientation.w = q.w();

		pose_frame.setOrigin( tf::Vector3(px, py, pz) );
		pose_frame.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()) );

//Twist
		if((dhdGetLinearVelocity(&vx, &vy, &vz) < DHD_NO_ERROR) || (dhdGetAngularVelocityRad(&wx, &wy, &wz) < DHD_NO_ERROR)) {
			printf ("error: cannot read twist (%s)\n", dhdErrorGetLastStr());
			done = 1;
		}

		twist.header.stamp = ros::Time::now();
		twist.twist.linear.x = vx;
		twist.twist.linear.y = vy;
		twist.twist.linear.z = vz;
		twist.twist.angular.x = wx;
		twist.twist.angular.y = wy;
		twist.twist.angular.z = wz;

//Force
		if (dhdSetForceAndTorqueAndGripperForce (wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
			printf ("error: cannot set wrench (%s)\n", dhdErrorGetLastStr());
			done = 1;
		}

//Button state
    if((dhdGetButton (0)) && (pressed == false)){
        butt_msg.data = true;
		button_pub.publish(butt_msg);
    }
    pressed = dhdGetButton (0);


//Publish
		pose_pub.publish(pose);
		twist_pub.publish(twist);
		broadcaster.sendTransform(tf::StampedTransform(pose_frame, ros::Time::now(), base_frame_name, ee_frame_name));

		ros::spinOnce();
		loop_rate.sleep();
	}

//Close the connection
	dhdClose ();

	return 0;

}



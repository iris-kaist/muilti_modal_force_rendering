// Contact force Virtual Wall

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace std;

ros::Subscriber subEEPose_;
ros::Publisher pubWrench_;

geometry_msgs::WrenchStamped wrench_msg_;

double peg_rad_ = 0.0;
double peg_length_ = 0.0;

struct virtual_wall{
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double kp = 0.0;
	double kd = 0.0;
};

virtual_wall vw_left_;
virtual_wall vw_right_;


geometry_msgs::Vector3 ContactForceRendering(const geometry_msgs::Point &pos)
{
	static geometry_msgs::Vector3 contact_force;

    contact_force.x = 0.0;
    contact_force.z = 0.0;

	double contact_point_right = vw_right_.py - peg_rad_;
	double contact_point_left = vw_left_.py + peg_rad_;

	if(pos.y > contact_point_right){
		contact_force.y = vw_right_.kp * (contact_point_right - pos.y);
		// cout <<"Contact Force [Right VW]: " <<  contact_force.y << endl;
	}
	else if(pos.y < contact_point_left){
		contact_force.y = vw_left_.kp * (contact_point_left - pos.y);
		// cout <<"Contact Force [Left VW]: " <<  contact_force.y << endl;
	}
	else{
		contact_force.y = 0.0;
		// cout <<"Contact Force [No Contact]: " <<  contact_force.y << endl;
	}

	
	
    return contact_force;
}

void CallbackEEPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
    wrench_msg_.wrench.force = ContactForceRendering(msg->pose.position);
	
	wrench_msg_.header.stamp = msg->header.stamp;
	pubWrench_.publish(wrench_msg_);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "contact_force_rendering_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.getParam("/virtual_wall/left/position/x",vw_left_.px);
	pnh.getParam("/virtual_wall/left/position/y",vw_left_.py);
	pnh.getParam("/virtual_wall/left/position/z",vw_left_.pz);
	pnh.getParam("/virtual_wall/left/stiffness",vw_left_.kp);
	pnh.getParam("/virtual_wall/left/damping",vw_left_.kd);
	
	pnh.getParam("/virtual_wall/right/position/x",vw_right_.px);
	pnh.getParam("/virtual_wall/right/position/y",vw_right_.py);
	pnh.getParam("/virtual_wall/right/position/z",vw_right_.pz);
	pnh.getParam("/virtual_wall/right/stiffness",vw_right_.kp);
	pnh.getParam("/virtual_wall/right/damping",vw_right_.kd);

	pnh.getParam("/peg/dimension/length", peg_length_);
	pnh.getParam("/peg/dimension/radius", peg_rad_);

	subEEPose_ = nh.subscribe("/ee_pose", 1, CallbackEEPose); //topic que function
	pubWrench_ = nh.advertise<geometry_msgs::WrenchStamped>("/force/contact", 1); //topic que

	// init global variables
	wrench_msg_.header.frame_id = "end_effector";

	ros::spin();

	return 0;
}
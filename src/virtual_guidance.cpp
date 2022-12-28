// Virtual guidance generation

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

ros::Subscriber subEEPose_;
ros::Publisher pubWrench_;

geometry_msgs::WrenchStamped wrench_msg_;

struct Virtual_Guidance{
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double kp = 0.0;
	double kd = 0.0;
};
Virtual_Guidance vg_;

geometry_msgs::Vector3 VirtualGuidanceForceRendering(const geometry_msgs::Point &pos)
{
    static geometry_msgs::Vector3 vg_force;

    vg_force.x = 0.0;
    vg_force.y = -1.0 * vg_.kp * (pos.y - vg_.py);
    vg_force.z = 0.0;

    return vg_force;
}

void CallbackEEPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	// std::cout<<msg->state.pose.position<<std::endl;

    wrench_msg_.wrench.force = VirtualGuidanceForceRendering(msg->pose.position);
	
	wrench_msg_.header.stamp = msg->header.stamp;
	pubWrench_.publish(wrench_msg_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "contact_force_rendering_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

    pnh.getParam("/virtual_guidance/position/x",vg_.px);
	pnh.getParam("/virtual_guidance/position/y",vg_.py);
	pnh.getParam("/virtual_guidance/position/z",vg_.pz);
	pnh.getParam("/virtual_guidance/stiffness",vg_.kp);
	pnh.getParam("/virtual_guidance/damping",vg_.kd);



	subEEPose_ = nh.subscribe("/ee_pose", 1, CallbackEEPose); //topic que function
	pubWrench_ = nh.advertise<geometry_msgs::WrenchStamped>("/force/virtual_guidance", 1); //topic que

	// init global variables
	wrench_msg_.header.frame_id = "end_effector";

	ros::spin();

	return 0;
}
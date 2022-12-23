// Multi-modal force rendering algorithms

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <phantom_premium_msgs/TeleoperationDeviceStateStamped.h>

ros::Subscriber subInterfaceState_;
ros::Subscriber subContactForce_;
ros::Publisher pubWrench_;

phantom_premium_msgs::TeleoperationDeviceStateStamped MM_wrench_msg_;

geometry_msgs::Wrench contact_wrench_, virtual_guidance_wrench_;

void CallbackInterfaceState(const phantom_premium_msgs::TeleoperationDeviceStateStamped::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;

	MM_wrench_msg_.state.wrench.force.x = contact_wrench_.force.x + virtual_guidance_wrench_.force.x;
	MM_wrench_msg_.state.wrench.force.y = contact_wrench_.force.y + virtual_guidance_wrench_.force.y;
	MM_wrench_msg_.state.wrench.force.z = contact_wrench_.force.z + virtual_guidance_wrench_.force.z;
	
	pubWrench_.publish(MM_wrench_msg_);
}

void CallbackContactForce(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;

	contact_wrench_ = msg->wrench;
}

void CallbackVirtualGuidanceForce(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;

	virtual_guidance_wrench_ = msg->wrench;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "multi_modal_force_rendering_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	subInterfaceState_ = nh.subscribe("/master_control/input_master_state", 1, CallbackInterfaceState); //topic que function
	subContactForce_ = nh.subscribe("/force/contact", 1, CallbackContactForce); //topic que function
	subContactForce_ = nh.subscribe("/force/virtual_guidance", 1, CallbackVirtualGuidanceForce); //topic que function
	pubWrench_ = nh.advertise<geometry_msgs::WrenchStamped>("/force/", 1); //topic que

	// init global variables
	MM_wrench_msg_.header.frame_id = "tip";

	ros::spin();

	return 0;
}
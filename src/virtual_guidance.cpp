// Todo
// Virtual guidance generation
// single axis
// attraction force

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <phantom_premium_msgs/TeleoperationDeviceStateStamped.h>

ros::Subscriber subInterfaceState_;
ros::Publisher pubWrench_;

geometry_msgs::WrenchStamped wrench_msg_;

geometry_msgs::Vector3 VirtualGuidanceForceRendering(const geometry_msgs::Point &pos)
{
    static geometry_msgs::Vector3 contact_force;
    static geometry_msgs::Vector3 wall_pos;

    static float kp = 0.1;

    contact_force.x = kp * (wall_pos.x - pos.x);
    contact_force.y = kp * (wall_pos.y - pos.y);
    contact_force.z = kp * (wall_pos.z - pos.z);

    return contact_force;
}

void CallbackInterfaceState(const phantom_premium_msgs::TeleoperationDeviceStateStamped::ConstPtr& msg)
{
	// std::cout<<msg->state.pose.position<<std::endl;

    wrench_msg_.wrench.force = VirtualGuidanceForceRendering(msg->state.pose.position);
	
	pubWrench_.publish(wrench_msg_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "contact_force_rendering_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	subInterfaceState_ = nh.subscribe("/master_control/input_master_state", 1, CallbackInterfaceState); //topic que function
	pubWrench_ = nh.advertise<geometry_msgs::WrenchStamped>("/force/virtual_guidance", 1); //topic que

	// init global variables
	wrench_msg_.header.frame_id = "tip";

	ros::spin();

	return 0;
}
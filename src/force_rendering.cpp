// Multi-modal force rendering algorithms

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <phantom_premium_msgs/TeleoperationDeviceStateStamped.h>

ros::Subscriber subInterfaceState_;
ros::Subscriber subContactForce_;
ros::Subscriber subVGForce_;

ros::Publisher pubWrench_;
ros::Publisher pubMasterForceFeedback_;

phantom_premium_msgs::TeleoperationDeviceStateStamped MM_wrench_msg_;

geometry_msgs::Wrench contact_wrench_, virtual_guidance_wrench_;
geometry_msgs::WrenchStamped force_feedback_wrench_;

class HPF
{
private:
public:
	double dt = 1;
	double alpha = 1;
	double input_prev = 0;
	double output_prev = 0;
	
	bool isFirst = false;

	void init(double cutoff_freq, double dt_init = 0.001){
		dt = dt_init;
		double wc = 2*M_PI*cutoff_freq;
		double tau = 1/wc;
		alpha = tau/(tau + dt);

		isFirst = true;
		output_prev = 0;
	}
    double update(const double &input){
		if(isFirst){
			isFirst = false;
			input_prev = input;
		}
		output_prev = alpha * (output_prev + input - input_prev);
		input_prev = input;
		return output_prev;
	}
    double uni_update(const double &input){
		if(fabs(input_prev) < fabs(input)) update(input);
		else{
			input_prev = input;
			output_prev = input;
			output_prev = alpha * input;
			return output_prev;
		}
	}
};

typedef struct _HPF_Struct{
	HPF x;
	HPF y;
	HPF z;
} HPF_struct;

HPF_struct contactHPF;
HPF_struct guidanceHPF;

enum Rendering_Mode_{
	ERROR = 0,
	ADD_MODE = 1,
	CONTACT_HPF_MODE = 2,
	CONTACT_UD_HPF_MODE = 3,
};

int rendering_mode_ = ADD_MODE;

void CallbackInterfaceState(const phantom_premium_msgs::TeleoperationDeviceStateStamped::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;

	MM_wrench_msg_.state.wrench.force.x = contact_wrench_.force.x + virtual_guidance_wrench_.force.x;
	MM_wrench_msg_.state.wrench.force.y = contact_wrench_.force.y + virtual_guidance_wrench_.force.y;
	MM_wrench_msg_.state.wrench.force.z = contact_wrench_.force.z + virtual_guidance_wrench_.force.z;
	
	// pubMasterForceFeedback_.publish(MM_wrench_msg_);

	force_feedback_wrench_.header.frame_id = "end_effector";
	force_feedback_wrench_.header.stamp = ros::Time::now();
	switch(rendering_mode_){
		case ADD_MODE:
			force_feedback_wrench_.wrench.force.x = contact_wrench_.force.x + virtual_guidance_wrench_.force.x;
			force_feedback_wrench_.wrench.force.y = contact_wrench_.force.y + virtual_guidance_wrench_.force.y;
			force_feedback_wrench_.wrench.force.z = contact_wrench_.force.z + virtual_guidance_wrench_.force.z;
			break;
		case CONTACT_HPF_MODE:
			force_feedback_wrench_.wrench.force.x = contactHPF.x.update(contact_wrench_.force.x) + virtual_guidance_wrench_.force.x;
			force_feedback_wrench_.wrench.force.y = contactHPF.y.update(contact_wrench_.force.y) + virtual_guidance_wrench_.force.y;
			force_feedback_wrench_.wrench.force.z = contactHPF.z.update(contact_wrench_.force.z) + virtual_guidance_wrench_.force.z;
			break;
		case CONTACT_UD_HPF_MODE:
			force_feedback_wrench_.wrench.force.x = contactHPF.x.uni_update(contact_wrench_.force.x) + virtual_guidance_wrench_.force.x;
			force_feedback_wrench_.wrench.force.y = contactHPF.y.uni_update(contact_wrench_.force.y) + virtual_guidance_wrench_.force.y;
			force_feedback_wrench_.wrench.force.z = contactHPF.z.uni_update(contact_wrench_.force.z) + virtual_guidance_wrench_.force.z;
			break;
		default:
			force_feedback_wrench_.wrench.force.x = 0;
			force_feedback_wrench_.wrench.force.y = 0;
			force_feedback_wrench_.wrench.force.z = 0;
	}
	pubWrench_.publish(force_feedback_wrench_);

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

	double cutoff_freq = 10;
	pnh.getParam("cutoff_freq", cutoff_freq);
	pnh.getParam("mode", rendering_mode_);

	subInterfaceState_ = nh.subscribe("/master_control/input_master_state", 1, CallbackInterfaceState); //topic que function
	subContactForce_ = nh.subscribe("/force/contact", 1, CallbackContactForce); //topic que function
	subVGForce_ = nh.subscribe("/force/virtual_guidance", 1, CallbackVirtualGuidanceForce); //topic que function

	pubMasterForceFeedback_ = nh.advertise<phantom_premium_msgs::TeleoperationDeviceStateStamped>("/force/", 1); //topic que
	pubWrench_ = nh.advertise<geometry_msgs::WrenchStamped>("/force_feedback", 1); //topic que

	// init global variables
	MM_wrench_msg_.header.frame_id = "end_effector";

	double dt = 0.001;
	contactHPF.x.init(cutoff_freq, dt);
	contactHPF.y.init(cutoff_freq, dt);
	contactHPF.z.init(cutoff_freq, dt);
	// guidanceHPF.x.init(cutoff_freq, dt);
	// guidanceHPF.y.init(cutoff_freq, dt);
	// guidanceHPF.z.init(cutoff_freq, dt);
	ROS_WARN("contactHPF alpha: %f", contactHPF.x.alpha);
	// ROS_WARN("guidanceHPF alpha: %f", guidanceHPF.x.alpha);

	ros::spin();

	return 0;
}
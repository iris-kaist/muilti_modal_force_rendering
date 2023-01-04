// Contact force Virtual Wall

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace std;

ros::Subscriber subEEPose_;
ros::Publisher pubWrench_;

geometry_msgs::WrenchStamped wrench_msg_;
geometry_msgs::PoseStamped peg_pose_;
geometry_msgs::Pose hip_pose_;

double peg_rad_ = 0.0;
double peg_height_ = 0.0;

double max_force_ = 1.0; // [N]

enum ContactType{
	NO_CONTACT,// assigned 0
	LEFT,// assigned 1
	RIGHT,// assigned 2
	UP,// assigned 3
	DOWN,// assigned 4
	FRONT,// assigned 5
	BACK// assigned 6
};

enum PositionType{
	NULL_SIDE,
	LEFT_SIDE,
	RIGHT_SIDE,
	UP_SIDE,
	DOWN_SIDE,
	FRONT_SIDE,
	BACK_SIDE,
	IN_SIDE
};



int contact_state_[3] = {0,0,0}; // 0: No contact, 1: contact to the positive direction, -1: contact to the negative direction

struct Virtual_Wall{
	geometry_msgs::Point p;
	geometry_msgs::Point dim;
	double kp = 0.0;
	double kd = 0.0;
};
Virtual_Wall vw_left_;
Virtual_Wall vw_right_;

int left_contact_state_ = ContactType::NO_CONTACT;
int left_position_state_ = PositionType::NULL_SIDE;

int right_contact_state_ = ContactType::NO_CONTACT;
int right_position_state_ = PositionType::NULL_SIDE;

void PrintContactState(int contact_state){
	switch(contact_state){
		case ContactType::NO_CONTACT:
			std::cout << "No Contact";
			break;
		case ContactType::LEFT:
			std::cout << "Left Contact";
			break;
		case ContactType::RIGHT:
			std::cout << "Right Contact";
			break;
		case ContactType::UP:
			std::cout << "Up Contact";
			break;
		case ContactType::DOWN:
			std::cout << "Down Contact";
			break;
	}
}
void PrintPositionState(int position_state){
	switch(position_state){
		case PositionType::NULL_SIDE:
			std::cout << "NULL_SIDE";
			break;
		case PositionType::LEFT_SIDE:
			std::cout << "LEFT_SIDE";
			break;
		case PositionType::RIGHT_SIDE:
			std::cout << "RIGHT_SIDE";
			break;
		case PositionType::UP_SIDE:
			std::cout << "UP_SIDE";
			break;
		case PositionType::DOWN_SIDE:
			std::cout << "DOWN_SIDE";
			break;
		case PositionType::IN_SIDE:
			std::cout << "IN_SIDE";
			break;
	}
}


geometry_msgs::Vector3 ContactForceRendering(const geometry_msgs::Point &pos)
{
	static geometry_msgs::Vector3 contact_force;

    contact_force.x = 0.0;
	contact_force.y = 0.0;
    contact_force.z = 0.0;

	double contact_point_right = vw_right_.p.y - peg_rad_;
	double contact_point_left = vw_left_.p.y + peg_rad_;

	double contact_point_up = vw_right_.p.z + 0.5*vw_right_.dim.z + 0.5*peg_height_;
	
	// static int left_contact_state_prev = left_contact_state_;
	// static int right_contact_state_prev = right_contact_state_;
	
	static int left_position_state_prev = left_position_state_;
	static int right_position_state_prev = right_position_state_;


	// Peg Position State
	if(pos.y < contact_point_left){
		if(pos.z < contact_point_up){
			left_position_state_ = PositionType::IN_SIDE;
		}
		else{
			left_position_state_ = PositionType::UP_SIDE;
			right_position_state_ = PositionType::NULL_SIDE;

		}
	}
	else if(pos.y > contact_point_right){
		if(pos.z < contact_point_up){
			right_position_state_ = PositionType::IN_SIDE;
		}
		else{
			right_position_state_ = PositionType::UP_SIDE;
			left_position_state_ = PositionType::NULL_SIDE;
		}
	}
	else{
		if(pos.z < contact_point_up){
			left_position_state_ = PositionType::RIGHT_SIDE;
			right_position_state_ = PositionType::LEFT_SIDE;
		}
		else{
			left_position_state_ = PositionType::NULL_SIDE;
			right_position_state_ = PositionType::NULL_SIDE;
		}
	}



	// Contact State
	if(left_position_state_ == PositionType::IN_SIDE){
		if(left_position_state_prev == IN_SIDE){
			// left_contact_state_ = left_contact_state_prev; // Keep previous contact state
		}
		else if(left_position_state_prev == PositionType::RIGHT_SIDE) left_contact_state_ = ContactType::LEFT;
		else if(left_position_state_prev == PositionType::UP_SIDE) left_contact_state_ = ContactType::DOWN;
		else cout <<"Something Wrong" << endl;
	}
	else{
		left_contact_state_ = ContactType::NO_CONTACT;
	}

	if(right_position_state_ == PositionType::IN_SIDE){
		if(right_position_state_prev == IN_SIDE){
			// right_contact_state_ = right_contact_state_prev; // Keep previous contact state
		}
		else if(right_position_state_prev == PositionType::LEFT_SIDE) right_contact_state_ = ContactType::RIGHT;
		else if(right_position_state_prev == PositionType::UP_SIDE) right_contact_state_ = ContactType::DOWN;
		else cout <<"Something Wrong" << endl;
	}
	else{
		right_contact_state_ = ContactType::NO_CONTACT;
	}

	// Print State
	cout << "Position State: " << endl;
	PrintPositionState(left_position_state_);
	cout << ", ";
	PrintPositionState(right_position_state_);
	cout << endl;

	cout << "Contact State: " << endl;
	PrintContactState(left_contact_state_);
	cout << ", ";
	PrintContactState(right_contact_state_);
	cout << endl;
	cout << endl;

	left_position_state_prev = left_position_state_;
	right_position_state_prev = right_position_state_;

	// Horizontal Wall Reaction Force
	// if(pos.y > contact_point_right){
	// 	if(pos.z < contact_point_up){
	// 		contact_force.y = vw_right_.kp * (contact_point_right - pos.y);
	// 	}
	// }
	// else if(pos.y < contact_point_left){
	// 	if(pos.z < contact_point_up){
	// 		contact_force.y = vw_left_.kp * (contact_point_left - pos.y);
	// 	}
	// }
	// else{
	// 	contact_force.y = 0.0;
	// }

	/// Wall Reaction Force 23.01.04 by SS
	geometry_msgs::Vector3 contact_force_right;
	geometry_msgs::Vector3 contact_force_left;

	contact_force_right.x = 0.0;
	contact_force_right.y = 0.0;
	contact_force_right.z = 0.0;

	contact_force_left.x = 0.0;
	contact_force_left.y = 0.0;
	contact_force_left.z = 0.0;

	switch(left_contact_state_)
	{
		case ContactType::LEFT:
			contact_force_left.y = vw_left_.kp * (contact_point_left - pos.y);
			break;
		
		case ContactType::DOWN:
			contact_force_left.z = vw_left_.kp * (contact_point_up - pos.z);
	}

	switch(right_contact_state_)
	{
		case ContactType::RIGHT:
			contact_force_right.y = vw_right_.kp * (contact_point_right - pos.y);
			break;
		
		case ContactType::DOWN:
			contact_force_right.z = vw_right_.kp * (contact_point_up - pos.z);
	}

	contact_force.x = contact_force_left.x + contact_force_right.x ;
	contact_force.y = contact_force_left.y + contact_force_right.y ;
	contact_force.z = contact_force_left.z + contact_force_right.z ;


    return contact_force;
}

geometry_msgs::Vector3 ForceConstraint(const geometry_msgs::Vector3& force){
	static geometry_msgs::Vector3 constrained_force;

	if(force.x > max_force_) constrained_force.x = max_force_;
	else if(force.x <-max_force_) constrained_force.x =-max_force_;
	else constrained_force.x = force.x;

	if(force.y > max_force_) constrained_force.y = max_force_;
	else if(force.y <-max_force_) constrained_force.y =-max_force_;
	else constrained_force.y = force.y;

	if(force.z > max_force_) constrained_force.z = max_force_;
	else if(force.z <-max_force_) constrained_force.z =-max_force_;
	else constrained_force.z = force.z;

	return constrained_force;
}

void CallbackEEPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	peg_pose_.header = msg->header;
	hip_pose_ = msg->pose;

	static geometry_msgs::Vector3 contact_force;
	contact_force = ContactForceRendering(msg->pose.position);
	wrench_msg_.wrench.force = ForceConstraint(contact_force);
	
	wrench_msg_.header.stamp = msg->header.stamp;
	// std::cout << contact_force << std::endl << wrench_msg_.wrench.force << std::endl;
	pubWrench_.publish(wrench_msg_);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "contact_force_rendering_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.getParam("/virtual_wall/left/position/x",vw_left_.p.x);
	pnh.getParam("/virtual_wall/left/position/y",vw_left_.p.y);
	pnh.getParam("/virtual_wall/left/position/z",vw_left_.p.z);
	pnh.getParam("/virtual_wall/left/stiffness",vw_left_.kp);
	pnh.getParam("/virtual_wall/left/damping",vw_left_.kd);
	pnh.getParam("/virtual_wall/left/dimension/length",vw_left_.dim.x);
	pnh.getParam("/virtual_wall/left/dimension/width",vw_left_.dim.y);
	pnh.getParam("/virtual_wall/left/dimension/height",vw_left_.dim.z);

	pnh.getParam("/virtual_wall/right/position/x",vw_right_.p.x);
	pnh.getParam("/virtual_wall/right/position/y",vw_right_.p.y);
	pnh.getParam("/virtual_wall/right/position/z",vw_right_.p.z);
	pnh.getParam("/virtual_wall/right/stiffness",vw_right_.kp);
	pnh.getParam("/virtual_wall/right/damping",vw_right_.kd);
	pnh.getParam("/virtual_wall/right/dimension/length",vw_right_.dim.x);
	pnh.getParam("/virtual_wall/right/dimension/width",vw_right_.dim.y);
	pnh.getParam("/virtual_wall/right/dimension/height",vw_right_.dim.z);

	pnh.getParam("/peg/dimension/height", peg_height_);
	pnh.getParam("/peg/dimension/radius", peg_rad_);

	pnh.getParam("max_force", max_force_);

	subEEPose_ = nh.subscribe("/ee_pose", 1, CallbackEEPose); //topic que function
	pubWrench_ = nh.advertise<geometry_msgs::WrenchStamped>("/force/contact", 1); //topic que

	// init global variables
	wrench_msg_.header.frame_id = "end_effector";

	ros::spin();

	return 0;
}

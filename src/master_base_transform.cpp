// Master Base-Link End-effector Pose Stamped 

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <phantom_premium_msgs/TeleoperationDeviceStateStamped.h>


#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std;

using Eigen::Affine3d;
using Eigen::AngleAxisd;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;


ros::Subscriber subInterfaceState_;
ros::Subscriber subForceFeedback_;

ros::Publisher pubEEPoseStamped_; // world frame
ros::Publisher pubForceFeedback_;

phantom_premium_msgs::TeleoperationDeviceStateStamped send_to_phantom_;

double roll_wp_; // roll angle from world to phantom
double pitch_wp_; // pitch angle from world to phantom
double yaw_wp_; // yaw angle from world to phantom

Matrix3d R_pw_; // Rotation Matrix 

int publish_rate_;

void CallbackInterfaceState(const phantom_premium_msgs::TeleoperationDeviceStateStamped::ConstPtr& msg)
{
	send_to_phantom_.header.frame_id = msg->header.frame_id;

	// Coordinate Transform 
	Vector3d pm_phantom(msg->state.pose.position.x, msg->state.pose.position.y, msg->state.pose.position.z);
	Vector3d pm_world;

	pm_world = R_pw_*pm_phantom;

	// cout << "Pm_phantom: " << pm_phantom(0) << ", "<< pm_phantom(1) << ", "<< pm_phantom(2) << endl;
	// cout << "Pm___world: " << pm_world(0) << ", "<< pm_world(1) << ", "<< pm_world(2) << endl;

	geometry_msgs::PoseStamped ee_pose_stamped;
	ee_pose_stamped.header.frame_id = "world";
	ee_pose_stamped.header.stamp = msg->header.stamp;
	ee_pose_stamped.pose.position.x = pm_world(0);
	ee_pose_stamped.pose.position.y = pm_world(1);
	ee_pose_stamped.pose.position.z = pm_world(2);
	ee_pose_stamped.pose.orientation.w = 1.0;

	pubEEPoseStamped_.publish(ee_pose_stamped);


	// End-Effector TF BroadCasting

	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped tf_stamped;
	geometry_msgs::Transform transform;

    tf_stamped.header.stamp = msg->header.stamp;
    tf_stamped.header.frame_id = "world";
    tf_stamped.child_frame_id = "end_effector";

    transform.translation.x = pm_world(0);
    transform.translation.y = pm_world(1);
    transform.translation.z = pm_world(2);

	// TODO: Orientation update
    transform.rotation.x = 0.0;
	transform.rotation.y = 0.0;
	transform.rotation.z = 0.0;
	transform.rotation.w = 1;

    tf_stamped.transform = transform;

    br.sendTransform(tf_stamped);

}


void CallbackForceFeedback(const geometry_msgs::WrenchStampedConstPtr &msg){

	// Coordinate Transform
	Vector3d f_world(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
	Vector3d f_phantom;

	f_phantom = R_pw_.transpose()*f_world;

	// cout << "Force___World: " << f_world(0) << ", " << f_world(1) << ", " << f_world(2) << endl;
	// cout << "Force_Phantom: " << f_phantom(0) << ", " << f_phantom(1) << ", " << f_phantom(2) << endl;

	send_to_phantom_.state.wrench.force.x = f_phantom(0);
	send_to_phantom_.state.wrench.force.y = f_phantom(1);
	send_to_phantom_.state.wrench.force.z = f_phantom(2);
	send_to_phantom_.header.stamp = msg->header.stamp;

	pubForceFeedback_.publish(send_to_phantom_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "master_tf_transform_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param("publish_rate", publish_rate_, 1000);

	// Get Rotation ('world' frame to 'master'(phantom) frame)
	pnh.param("Roll_WtoP", roll_wp_, 0.0);
	pnh.param("Pitch_WtoP", pitch_wp_, 0.0);
	pnh.param("Yaw_WtoP", yaw_wp_, 0.0);
	
	AngleAxisd rollAngle(roll_wp_, Eigen::Vector3d::UnitX());
	AngleAxisd pitchAngle(pitch_wp_, Eigen::Vector3d::UnitY());
	AngleAxisd yawAngle(yaw_wp_, Eigen::Vector3d::UnitZ());
	
	Quaterniond q =  yawAngle * pitchAngle * rollAngle;

	R_pw_ = q.matrix();



    subInterfaceState_ = nh.subscribe("/master_control/input_master_state", 1, CallbackInterfaceState);
	subForceFeedback_ = nh.subscribe("/force_feedback", 1, CallbackForceFeedback); 

	pubEEPoseStamped_ = nh.advertise<geometry_msgs::PoseStamped>("ee_pose", 10); 
	pubForceFeedback_ = nh.advertise<phantom_premium_msgs::TeleoperationDeviceStateStamped>("/master_control/output_slave_state", 10); 


	ros::Rate loop_rate(publish_rate_);
	int count = 0;
	while (ros::ok())
	{
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}


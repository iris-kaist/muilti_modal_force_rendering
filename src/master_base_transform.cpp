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

ros::Publisher pubMasterPoseStamped_; // master device frame
ros::Publisher pubEEPoseStamped_; // base_link frame

phantom_premium_msgs::TeleoperationDeviceStateStamped msg_;

geometry_msgs::TransformStamped tf_stamped_;
phantom_premium_msgs::TeleoperationDeviceStateStamped interface_;


int publish_rate_;

Matrix3d rotvec_to_rotmat(const Vector3d &r, double epsilon = 1e-20) {
  AngleAxisd aa;
  aa.axis() = r.normalized();
  aa.angle() = r.norm();
  return aa.toRotationMatrix();
}


void UpdateTF(tf2_ros::TransformBroadcaster &br)
{
	static geometry_msgs::Transform transform;

    transform.translation.x = interface_.state.pose.position.x;
    transform.translation.y = interface_.state.pose.position.y;
    transform.translation.z = interface_.state.pose.position.z;
    transform.rotation.w = 1;

    tf_stamped_.header.stamp = ros::Time::now();
    tf_stamped_.header.frame_id = "master";
    tf_stamped_.child_frame_id = "end_effector";
    tf_stamped_.transform = transform;

    br.sendTransform(tf_stamped_);
}

void CallbackInterfaceState(const phantom_premium_msgs::TeleoperationDeviceStateStamped::ConstPtr& msg)
{
	interface_ = *msg;
	tf2_ros::TransformBroadcaster br;
	UpdateTF(br);

	// Transform master pose from 'master' frame to 'base_link' frame
	geometry_msgs::PoseStamped master_pose_stamped;
	master_pose_stamped.pose.position.x = msg->state.pose.position.x;
	master_pose_stamped.pose.position.y = msg->state.pose.position.y;
	master_pose_stamped.pose.position.z = msg->state.pose.position.z;
	master_pose_stamped.pose.orientation.w = 1.0;
	master_pose_stamped.header.stamp = msg->header.stamp;
	master_pose_stamped.header.frame_id = msg->header.frame_id;

	pubMasterPoseStamped_.publish(master_pose_stamped);


	Vector3d pm_m(msg->state.pose.position.x, msg->state.pose.position.y, msg->state.pose.position.z);
	Vector3d pm_b;


	double roll, pitch, yaw;
	roll = M_PI/2;
	pitch = 0.0;
	yaw = M_PI/2;

	AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	
	Quaterniond q =  yawAngle * pitchAngle * rollAngle;

	Matrix3d R_mb = q.matrix();

	pm_b = R_mb*pm_m;

	// cout << "Pmm: " << pm_m(0) << ", "<< pm_m(1) << ", "<< pm_m(2) << endl;
	// cout << "Pmb: " << pm_b(0) << ", "<< pm_b(1) << ", "<< pm_b(2) << endl;


	geometry_msgs::PoseStamped ee_pose_stamped;
	ee_pose_stamped.header.frame_id = "base_link";
	ee_pose_stamped.header.stamp = msg->header.stamp;
	ee_pose_stamped.pose.position.x = pm_b(0);
	ee_pose_stamped.pose.position.y = pm_b(1);
	ee_pose_stamped.pose.position.z = pm_b(2);
	ee_pose_stamped.pose.orientation.w = 1.0;

	pubEEPoseStamped_.publish(ee_pose_stamped);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "master_tf_transform_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param("publish_rate", publish_rate_, 1000);
	
	tf2_ros::TransformBroadcaster br;

    subInterfaceState_ = nh.subscribe("/master_control/input_master_state", 1, CallbackInterfaceState); //topic que function

	pubEEPoseStamped_ = nh.advertise<geometry_msgs::PoseStamped>("ee_pose_stamped", 10); 
	pubMasterPoseStamped_ = nh.advertise<geometry_msgs::PoseStamped>("master_pose_stamped", 10); 

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


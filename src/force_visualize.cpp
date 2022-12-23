// Visualize the virtual wall, guidance, force, HIP with RViZ

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>
#include <phantom_premium_msgs/TeleoperationDeviceStateStamped.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
ros::Subscriber subInterfaceState_;
ros::Publisher pubMarkerArray_;

phantom_premium_msgs::TeleoperationDeviceStateStamped msg_;
visualization_msgs::MarkerArray marker_array_msg_;

geometry_msgs::TransformStamped tf_stamped_;
phantom_premium_msgs::TeleoperationDeviceStateStamped interface_;

int publish_rate_;


void ForceMarkerInit(visualization_msgs::Marker *force)
{
	force->header.frame_id = "start_point";
	force->header.stamp = ros::Time();
	force->ns = "haptic_feedback";
	force->id = 0;
	force->type = visualization_msgs::Marker::CUBE;
	force->action = visualization_msgs::Marker::ADD;
	force->pose.position.x = 0;
	force->pose.position.y = 0;
	force->pose.position.z = 0;
	force->pose.orientation.x = 0.0;
	force->pose.orientation.y = 0.0;
	force->pose.orientation.z = 0.0;
	force->pose.orientation.w = 1.0;
	force->scale.x = 0.1;
	force->scale.y = 0.1;
	force->scale.z = 0.1;
	force->color.a = 1.0; // Don't forget to set the alpha!
	force->color.r = 1.0;
	force->color.g = 0.0;
	force->color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	// marker_mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}

void PublishMarkerArray()
{
	pubMarkerArray_.publish(marker_array_msg_);
}

void UpdateTF(tf2_ros::TransformBroadcaster &br)
{
	static geometry_msgs::Transform transform;

    transform.translation.x = interface_.state.pose.position.x;
    transform.translation.y = interface_.state.pose.position.y;
    transform.translation.z = interface_.state.pose.position.z;
    transform.rotation.w = 1;

    tf_stamped_.header.stamp = ros::Time::now();
    tf_stamped_.header.frame_id = "base";
    tf_stamped_.child_frame_id = "tip";
    tf_stamped_.transform = transform;

    br.sendTransform(tf_stamped_);
    // std::cout<<Odom->child_frame_id<<std::endl;

	PublishMarkerArray();

}

void CallbackInterfaceState(const phantom_premium_msgs::TeleoperationDeviceStateStamped::ConstPtr& msg)
{
	interface_ = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "force_visualize_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param("publish_rate", publish_rate_, 100);
	
	tf2_ros::TransformBroadcaster br;

    subInterfaceState_ = nh.subscribe("/master_control/input_master_state", 1, CallbackInterfaceState); //topic que function

	pubMarkerArray_ = nh.advertise<visualization_msgs::MarkerArray>("test_pub", 10); //topic que

	ros::Rate loop_rate(publish_rate_);
	int count = 0;
	while (ros::ok())
	{
		UpdateTF(br);

		PublishMarkerArray();

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
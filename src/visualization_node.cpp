// Visualize the virtual wall, guidance, force, HIP with RViZ

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


ros::Subscriber subEEPose_;

ros::Publisher pubEEMaker_;
ros::Publisher pubVGMaker_;
ros::Publisher pubVWLeftMaker_;
ros::Publisher pubVWRightMaker_;


visualization_msgs::Marker ee_marker_;
visualization_msgs::Marker vg_marker_;
visualization_msgs::Marker vw_left_marker_;
visualization_msgs::Marker vw_right_marker_;

geometry_msgs::TransformStamped tf_stamped_;

geometry_msgs::PoseStamped ee_pose_stamed_;

double vw_width_= 0.0;

double peg_rad_ = 0.0;
double peg_height_ = 0.0;

int publish_rate_;


struct Virtual_Wall{
	geometry_msgs::Point p;
	geometry_msgs::Point dim;
	double kp = 0.0;
	double kd = 0.0;
};
Virtual_Wall vw_left_;
Virtual_Wall vw_right_;


struct Virtual_Guidance{
	geometry_msgs::Point p;
	double kp = 0.0;
	double kd = 0.0;
};
Virtual_Guidance vg_;


void EEMarkerInit()
{
	ee_marker_.header.frame_id = "world";
	ee_marker_.header.stamp = ros::Time();
	ee_marker_.ns = "ee_marker";
	ee_marker_.id = 0;
	ee_marker_.type = visualization_msgs::Marker::CYLINDER;
	ee_marker_.action = visualization_msgs::Marker::ADD;
	ee_marker_.pose.position.x = 0;
	ee_marker_.pose.position.y = 0;
	ee_marker_.pose.position.z = 0;
	ee_marker_.pose.orientation.x = 0.0;
	ee_marker_.pose.orientation.y = 0.0;
	ee_marker_.pose.orientation.z = 0.0;
	ee_marker_.pose.orientation.w = 1.0;
	ee_marker_.scale.x = 2.0*peg_rad_;
	ee_marker_.scale.y = 2.0*peg_rad_;
	ee_marker_.scale.z = peg_height_;
	ee_marker_.color.a = 1.0; // Don't forget to set the alpha!
	ee_marker_.color.r = 0.0;
	ee_marker_.color.g = 1.0;
	ee_marker_.color.b = 0.0;
	//only if using a MESH_RESOURCE marker type:
	// marker_mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}

void VGMarkerInit()
{
	vg_marker_.header.frame_id = "world";
	vg_marker_.header.stamp = ros::Time();
	vg_marker_.ns = "vg_marker";
	vg_marker_.id = 0;
	vg_marker_.type = visualization_msgs::Marker::LINE_STRIP;
	vg_marker_.action = visualization_msgs::Marker::ADD;
	vg_marker_.pose.position.x = 0;
	vg_marker_.pose.position.y = 0;
	vg_marker_.pose.position.z = 0;
	vg_marker_.pose.orientation.x = 0.0;
	vg_marker_.pose.orientation.y = 0.0;
	vg_marker_.pose.orientation.z = 0.0;
	vg_marker_.pose.orientation.w = 1.0;
	vg_marker_.scale.x = 0.02;
	vg_marker_.scale.y = 0.1;
	vg_marker_.scale.z = 0.1;
	vg_marker_.color.a = 1.0; // Don't forget to set the alpha!
	vg_marker_.color.r = 0.0;
	vg_marker_.color.g = 0.0;
	vg_marker_.color.b = 1.0;
	//only if using a MESH_RESOURCE marker type:
	// marker_mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

	geometry_msgs::Point p1, p2;
	p1.x = vg_.p.x;
	p1.y = vg_.p.y;
	p1.z = vg_.p.z - 1.0;

	p2.x = vg_.p.x;
	p2.y = vg_.p.y;
	p2.z = vg_.p.z + 1.0;

	vg_marker_.points.push_back(p1);
	vg_marker_.points.push_back(p2);
}



void VW_LEFT_MarkerInit()
{
	vw_left_marker_.header.frame_id = "world";
	vw_left_marker_.header.stamp = ros::Time();
	vw_left_marker_.ns = "vw_left_marker";
	vw_left_marker_.id = 0;
	vw_left_marker_.type = visualization_msgs::Marker::CUBE;
	vw_left_marker_.action = visualization_msgs::Marker::ADD;
	vw_left_marker_.pose.position.x = vw_left_.p.x;
	vw_left_marker_.pose.position.y = vw_left_.p.y - vw_left_.dim.y*0.5;
	vw_left_marker_.pose.position.z = vw_left_.p.z;
	vw_left_marker_.pose.orientation.x = 0.0;
	vw_left_marker_.pose.orientation.y = 0.0;
	vw_left_marker_.pose.orientation.z = 0.0;
	vw_left_marker_.pose.orientation.w = 1.0;
	vw_left_marker_.scale.x = vw_left_.dim.x; 
	vw_left_marker_.scale.y = vw_left_.dim.y;
	vw_left_marker_.scale.z = vw_left_.dim.z;
	vw_left_marker_.color.a = 0.5; // Don't forget to set the alpha!
	vw_left_marker_.color.r = 0.8;
	vw_left_marker_.color.g = 0.8;
	vw_left_marker_.color.b = 0.8;
	//only if using a MESH_RESOURCE marker type:
	// marker_mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}

void VW_RIGHT_MarkerInit()
{
	vw_right_marker_.header.frame_id = "world";
	vw_right_marker_.header.stamp = ros::Time();
	vw_right_marker_.ns = "vw_right_marker";
	vw_right_marker_.id = 0;
	vw_right_marker_.type = visualization_msgs::Marker::CUBE;
	vw_right_marker_.action = visualization_msgs::Marker::ADD;
	vw_right_marker_.pose.position.x = vw_right_.p.x;
	vw_right_marker_.pose.position.y = vw_right_.p.y + vw_right_.dim.y*0.5;
	vw_right_marker_.pose.position.z = vw_right_.p.z;
	vw_right_marker_.pose.orientation.x = 0.0;
	vw_right_marker_.pose.orientation.y = 0.0;
	vw_right_marker_.pose.orientation.z = 0.0;
	vw_right_marker_.pose.orientation.w = 1.0;
	vw_right_marker_.scale.x = vw_right_.dim.x;
	vw_right_marker_.scale.y = vw_right_.dim.y;
	vw_right_marker_.scale.z = vw_right_.dim.z;
	vw_right_marker_.color.a = 0.5; // Don't forget to set the alpha!
	vw_right_marker_.color.r = 0.8;
	vw_right_marker_.color.g = 0.8;
	vw_right_marker_.color.b = 0.8;
	//only if using a MESH_RESOURCE marker type:
	// marker_mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}



void CallbackEEPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	ee_marker_.pose.position.x = msg->pose.position.x;
	ee_marker_.pose.position.y = msg->pose.position.y;
	ee_marker_.pose.position.z = msg->pose.position.z;

	// ee_marker_.header.stamp = msg->header.stamp;
	ee_marker_.header.stamp = ros::Time::now();

	pubEEMaker_.publish(ee_marker_);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "force_visualize_node");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	pnh.param("publish_rate", publish_rate_, 100);
	
	tf2_ros::TransformBroadcaster br;

    subEEPose_ = nh.subscribe("/ee_pose", 1, CallbackEEPose); //topic que function

	pubEEMaker_ = nh.advertise<visualization_msgs::Marker>("ee_marker", 1); //topic que
	pubVWLeftMaker_ = nh.advertise<visualization_msgs::Marker>("vw_left_marker", 10); //topic que
	pubVWRightMaker_ = nh.advertise<visualization_msgs::Marker>("vw_right_marker", 10); //topic que

	pubVGMaker_ = nh.advertise<visualization_msgs::Marker>("vg_marker", 10); //topic que

    pnh.getParam("/virtual_guidance/position/x",vg_.p.x);
	pnh.getParam("/virtual_guidance/position/y",vg_.p.y);
	pnh.getParam("/virtual_guidance/position/z",vg_.p.z);
	pnh.getParam("/virtual_guidance/stiffness",vg_.kp);
	pnh.getParam("/virtual_guidance/damping",vg_.kd);

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



	EEMarkerInit();
	VGMarkerInit();
	VW_LEFT_MarkerInit();
	VW_RIGHT_MarkerInit();
	

	ros::Rate loop_rate(publish_rate_);
	int count = 0;
	while (ros::ok())
	{
		
		// PublishMarkerArray();
		// ee_marker_.header.stamp = ros::Time::now();
		vg_marker_.header.stamp = ros::Time::now();
		vw_left_marker_.header.stamp = ros::Time::now();
		vw_right_marker_.header.stamp = ros::Time::now();

		pubVGMaker_.publish(vg_marker_);
		pubVWLeftMaker_.publish(vw_left_marker_);
		pubVWRightMaker_.publish(vw_right_marker_);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}

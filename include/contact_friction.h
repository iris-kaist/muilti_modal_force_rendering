#ifndef _CONTACT_FRICTION_H_
#define _CONTACT_FRICTION_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>

class ContactFriction
{
private:
    geometry_msgs::Point pos_;
    geometry_msgs::Point friction_proxy_;
    geometry_msgs::Vector3 stiffness_force_;
    geometry_msgs::Vector3 friction_force_; //return

    double stiffness_ = 0;

    double stiffness_force_norm_ = 0;

    double static_friction_force_ = 0;
    double kinetic_friction_force_ = 0;

    double peg_height_ = 0;
    double peg_rad_ = 0;

public:
    geometry_msgs::Point world_frame_proxy_;

    double contact_force_scalar_ = 0;

    double static_mu_ = 0.75; // [Friction Coeff.]
    double kinetic_mu_ = 0.5; // [Friction Coeff.]

    bool is_proxy_static_ = false;

    ContactFriction();

    void init();

    void updateFrictionParameter(const double &stiffness, const double &static_mu, const double &kinetic_mu);

    void updatePegSize(const double &peg_rad, const double &peg_height);

    int CheckContactDirection(const geometry_msgs::Point &input_pos, const geometry_msgs::Vector3 &contact_force, int &contact_direction);

    geometry_msgs::Vector3 inject(const geometry_msgs::Point &input_pos, const geometry_msgs::Vector3 &contact_force, int contact_direction = 0);
};

#endif // _CONTACT_FRICTION_H_
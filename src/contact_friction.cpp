#include <contact_friction.h>

ContactFriction::ContactFriction(){
    init();
}

void ContactFriction::init(){

    contact_force_scalar_ = 0;

    stiffness_force_norm_ = 0;

    static_friction_force_ = 0;
    kinetic_friction_force_ = 0;
}

void ContactFriction::updateFrictionParameter(const double &stiffness, const double &static_mu, const double &kinetic_mu){
    stiffness_ = stiffness;
    static_mu_ = static_mu;
    kinetic_mu_ = kinetic_mu;
}

void ContactFriction::updatePegSize(const double &peg_rad, const double &peg_height){
    peg_rad_ = peg_rad;
    peg_height_ = peg_height;
}

int ContactFriction::CheckContactDirection(const geometry_msgs::Point &input_pos, const geometry_msgs::Vector3 &contact_force, int &contact_direction){
    if(contact_force.x != 0 || contact_direction == 1){
        contact_direction = 1;

        pos_.x = input_pos.y;
        pos_.y = input_pos.z;
        pos_.z = input_pos.x;//contact direction

        if(contact_force_scalar_ == 0){ //CONTACT EVENT RISING: friction proxy init
            friction_proxy_ = pos_;
            
            if(contact_force.x > 0) friction_proxy_.z += peg_rad_;
            else friction_proxy_.z -= peg_rad_;
        }

        contact_force_scalar_ = fabs(contact_force.x);
    }else if(contact_force.y != 0 || contact_direction == 2){
        contact_direction = 2;

        pos_.x = input_pos.x;
        pos_.y = input_pos.z;
        pos_.z = input_pos.y;//contact direction

        if(contact_force_scalar_ == 0){ //CONTACT EVENT RISING: friction proxy init
            friction_proxy_ = pos_;
            
            if(contact_force.y < 0) friction_proxy_.z += peg_rad_;
            else friction_proxy_.z -= peg_rad_;
        }
        
        contact_force_scalar_ = fabs(contact_force.y);
    }else if(contact_force.z != 0 || contact_direction == 3){
        contact_direction = 3;

        pos_.x = input_pos.x;
        pos_.y = input_pos.y;
        pos_.z = input_pos.z;//contact direction

        if(contact_force_scalar_ == 0){ //CONTACT EVENT RISING: friction proxy init
            friction_proxy_ = pos_;

            if(contact_force.x > 0) friction_proxy_.z += peg_height_/2.;
            else friction_proxy_.z -= peg_height_/2.;
        }
        
        contact_force_scalar_ = fabs(contact_force.z);
    }else{
        contact_direction = 0;
        contact_force_scalar_ = 0;
    }
    
    return contact_direction;
}


geometry_msgs::Vector3 ContactFriction::inject(const geometry_msgs::Point &input_pos, const geometry_msgs::Vector3 &contact_force, int contact_direction){
    // check contact direction
    if(CheckContactDirection(input_pos, contact_force, contact_direction) == 0) return contact_force;


    //virtual stiffness
    stiffness_force_.x = stiffness_ * (friction_proxy_.x - pos_.x);
    stiffness_force_.y = stiffness_ * (friction_proxy_.y - pos_.y);
    stiffness_force_norm_ = sqrt(stiffness_force_.x*stiffness_force_.x + stiffness_force_.y*stiffness_force_.y); // norm([x, y])
    
    if(is_proxy_static_){
        static_friction_force_ = static_mu_ * contact_force_scalar_; // reserved
        if(stiffness_force_norm_ > static_friction_force_){
            friction_force_.x = stiffness_force_.x * static_friction_force_ / stiffness_force_norm_;
            friction_force_.y = stiffness_force_.y * static_friction_force_ / stiffness_force_norm_;

            friction_proxy_.x -= (stiffness_force_.x - friction_force_.x) / stiffness_;
            friction_proxy_.y -= (stiffness_force_.y - friction_force_.y) / stiffness_;

            is_proxy_static_ = false;
        }else{
            friction_force_.x = stiffness_force_.x;
            friction_force_.y = stiffness_force_.y;

            is_proxy_static_ = true;
        }
    }else{
        kinetic_friction_force_ = kinetic_mu_ * contact_force_scalar_; // threshold
        if(stiffness_force_norm_ > kinetic_friction_force_){
            friction_force_.x = stiffness_force_.x * kinetic_friction_force_ / stiffness_force_norm_;
            friction_force_.y = stiffness_force_.y * kinetic_friction_force_ / stiffness_force_norm_;

            friction_proxy_.x -= (stiffness_force_.x - friction_force_.x) / stiffness_;
            friction_proxy_.y -= (stiffness_force_.y - friction_force_.y) / stiffness_;

            is_proxy_static_ = false;
        }else{
            friction_force_.x = stiffness_force_.x;
            friction_force_.y = stiffness_force_.y;

            is_proxy_static_ = true;
        }
    }

    switch(contact_direction){
        case 1: //x
            friction_force_.z = friction_force_.y;
            friction_force_.y = friction_force_.x;
            friction_force_.x = contact_force.x;

            world_frame_proxy_.x = friction_proxy_.z;//
            world_frame_proxy_.y = friction_proxy_.x;
            world_frame_proxy_.z = friction_proxy_.y;
            break;
        case 2: //y
            // friction_force_.x = friction_force_.x;
            friction_force_.z = friction_force_.y;
            friction_force_.y = contact_force.y;

            world_frame_proxy_.x = friction_proxy_.x;
            world_frame_proxy_.y = friction_proxy_.z;//
            world_frame_proxy_.z = friction_proxy_.y;
            break;
        case 3: //z
            // friction_force_.x = friction_force_.x;
            // friction_force_.y = friction_force_.y;
            friction_force_.z = contact_force.z;

            world_frame_proxy_.x = friction_proxy_.x;
            world_frame_proxy_.y = friction_proxy_.y;
            world_frame_proxy_.z = friction_proxy_.z;//
            break;
    }

    return friction_force_;
}

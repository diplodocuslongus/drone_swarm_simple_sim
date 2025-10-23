/*********************************************************************************************************************
 * File : target.cpp                                                                                                 *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#include <GL/glut.h>

#include "target.h"
#include "gl_utils.h"
#include <iostream>


Target::Target(const Vec3f &position, UpdateSpeedFunc update_velocity_func) : MovingObject(position)
{
    boid_type_ = 2; // TARGET
    update_velocity_func_ = update_velocity_func;
    last_t_ = 0;
    update(0);
}
// Vec3f Target::get_exerted_proximity_force_object_only(const MovingObject &object) const
// {
//     if (object.get_id() == id_) // Can't repel itself
//         return Vec3f(0, 0, 0);
//
//     float tmp_target_attract_weight = MovingObject::getTargetAttractionWeight();
//     float tmp_target_velocity_align_weight = MovingObject::getTargetSpeedAlignmentWeight();
//     Vec3f posdif = position_ - object.get_position(); // position_ : target position, object pos: that of a boid
//     Vec3f force =  tmp_target_attract_weight * posdif +
//            tmp_target_velocity_align_weight * (velocity_ - object.get_velocity()); // Attract
//     if (posdif.norm() < 1.1){
//         // force = Vec3f(0.0,0.0,0.0); 
//         // std::cout << "force near target: "<< force.norm() <<std::endl;
//     }
//     return force;
// }

Vec3f Target::get_exerted_proximity_force(const MovingObject &object) const
{
    if (object.get_id() == id_) // Check for self-repulsion
        return Vec3f(0, 0, 0);

    Vec3f diff = position_ - object.get_position(); // Vector from Boid to Target
    float dist = diff.norm();
    
    Vec3f force(0, 0, 0);
    
    // Weights and constants depend on the mode
    float attract_weight;
    float velocity_align_weight;
    
    // Waypoint parameters (can be defined as static consts or member variables)
    const float WAYPOINT_MIN_RADIUS = 3.0f; 

    // Weights and Force Logic based on Target Mode
    if (mode_ == ATTRACT_AND_FLY_OVER) {
        // mainly for waypoints (could be used for regular target too, depending on intended behavior)
        
        // Waypoint-specific weights
        attract_weight     = MovingObject::getWaypointAttractionWeight();
        velocity_align_weight = MovingObject::getWaypointSpeedAlignmentWeight();
        
        // Alignment Force
        force += velocity_align_weight * (velocity_ - object.get_velocity());

        // Position Force: Attraction/Repulsion for fly-over
        if (dist > WAYPOINT_MIN_RADIUS) {
            // Attraction
            force += attract_weight * diff;
        } else {
            // Strong Repulsion (Fly-Over effect)
            float repel_magnitude = attract_weight * (WAYPOINT_MIN_RADIUS - dist) * 2.0f; 
            force -= repel_magnitude * diff.normalized(); 
        }

    } else { 
        // original target object
        
        // Target object weights
        attract_weight     = MovingObject::getTargetAttractionWeight();
        velocity_align_weight = MovingObject::getTargetSpeedAlignmentWeight();

        // Standard Attraction and Alignment (for stopping/hovering)
        force += attract_weight * diff;
        force += velocity_align_weight * (velocity_ - object.get_velocity());
    }

    return force;
}
void Target::update(float t)
{
    update_velocity_func_(t, velocity_);
    const float dt = (t - last_t_);
    last_t_ = t;
    position_ += velocity_ * dt;
}

void Target::update_no_thrust(float t)
{
}
void Target::update_no_ang_velocity_clamp(float t)
{
}
void Target::draw() const
{
    glPushMatrix();
    glTranslatef(position_[0], position_[1], position_[2]);
    glColor3f(1, 0, 0);
    glutSolidSphere(1, 5, 5);
    glPopMatrix();
}

/*********************************************************************************************************************
 * File : fence.cpp                                                                                               *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 * Defines a fence object type (area within which to evolve / fly)
 * Can be a spheroid fence boundary or a cuboid
 * The force is straightforward: if boid goes beyond the border, 
 * brings it toward the center with force opposite to forward direction
 *********************************************************************************************************************/

#include <GL/glut.h>
#include <iostream>
#include "fence.h"
#include "gl_utils.h"


Fence::Fence(const Vec3f &position, float radius) : MovingObject(position)
{
    type_ = SPHERE;
    boid_type_ = 4; // Fence

    radius_ = radius;

    rad_max_ = 0.8 * radius_;
}

// Initialize cuboid fence from 2 "vertices" corresponding to 
// min and max in x,y and z
Fence::Fence(const Vec3f &vertice1, const Vec3f &vertice2) : MovingObject((vertice1 + vertice2) / 2.0f)
{
    // Initialize properties for cuboid
    type_ = CUBOID;
    boid_type_ = 4; // Fence
    // Store the min and max coordinates
    min_x_ = std::min(vertice1.x(), vertice2.x());
    max_x_ = std::max(vertice1.x(), vertice2.x());
    min_y_ = std::min(vertice1.y(), vertice2.y());
    max_y_ = std::max(vertice1.y(), vertice2.y());
    min_z_ = std::min(vertice1.z(), vertice2.z());
    max_z_ = std::max(vertice1.z(), vertice2.z());
    radius_ = max_x_ - min_x_;
}


Vec3f Fence::get_exerted_proximity_force(const MovingObject &object) const
{
    if (object.get_id() == id_) // Can't repel itself
        return Vec3f(0, 0, 0);
    Vec3f repel_force(0,0,0);
    float tmp_fence_repel_weight = MovingObject::getFenceRepelWeight();
    if (type_ == SPHERE) {
        // Repel
        Vec3f diff = object.get_position() - position_;
        float dist = diff.norm();
        if (dist <= rad_max_)
            return Vec3f(0, 0, 0);

        float repel_strength = 1.0 / (dist - rad_max_); // Or similar...
        // std::cout << "fence " << repel_strength << " end fence";
        repel_force = tmp_fence_repel_weight * -diff.normalized() * repel_strength;
    }
    else if (type_ == CUBOID) {
        Vec3f force(0, 0, 0);
        Vec3f pos = object.get_position();
        // Check against x-axis boundaries
        // if (pos(0) < min_x_) {
        if (pos.x() < min_x_) {
            // Boid is outside on the left side, apply a force to the right
            float overlap_x = min_x_ - pos.x();
            force.x() += tmp_fence_repel_weight * overlap_x;
        } else if (pos.x() > max_x_) {
            // Boid is outside on the right side, apply a force to the left
            float overlap_x = pos.x() - max_x_;
            force.x() -= tmp_fence_repel_weight * overlap_x;
        }

        // Check against y-axis boundaries
        if (pos.y() < min_y_) {
            float overlap_y = min_y_ - pos.y();
            force.y() += tmp_fence_repel_weight * overlap_y;
        } else if (pos.y() > max_y_) {
            float overlap_y = pos.y() - max_y_;
            force.y() -= tmp_fence_repel_weight * overlap_y;
        }

        // Check against z-axis boundaries
        if (pos.z() < min_z_) {
            float overlap_z = min_z_ - pos.z();
            force.z() += tmp_fence_repel_weight * overlap_z;
        } else if (pos.z() > max_z_) {
            float overlap_z = pos.z() - max_z_;
            force.z() -= tmp_fence_repel_weight * overlap_z;
        }
        // return force;
        repel_force = force;

    }
    return repel_force;
}

void Fence::update(float t)
{
}
void Fence::update_no_ang_speed_clamp(float t)
{
}

void Fence::draw() const
{
    if (type_ == SPHERE) {
        glPushMatrix();
        glTranslatef(position_[0], position_[1], position_[2]);
        glColor3f(0.2, 0.3, 0.8);
        glutWireSphere(radius_, 20, 20);
        // glutSolidSphere(radius_, 20, 20);
        glPopMatrix();
    }
    else if (type_ == CUBOID) {
        glPushMatrix();
        glTranslatef(position_[0], position_[1], position_[2]);
        glColor3f(0.2, 0.3, 0.8);
        glutWireCube(1.2f*radius_); 
        glPopMatrix();
    }
}

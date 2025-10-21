/*********************************************************************************************************************
 * File : moving_object.cpp                                                                                          *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#include "moving_object.h"

// Initialize the static member variables outside the class.
// The default values here match those in `main.cpp` for consistency.
int MovingObject::s_boid_number = 50; 
float MovingObject::s_neighborhood_max_dist = 10.0f; //  
float MovingObject::s_separation_weight = 0.02f; //  separation_factor
float MovingObject::s_alignment_weight = 0.005f;
float MovingObject::s_cohesion_weight = 0.07f;
float MovingObject::s_vision_distance = 5.0f;
float MovingObject::s_vision_FOV = 90.0f;
float MovingObject::s_max_speed = 5.0f;
float MovingObject::s_target_attraction_weight = 0.02f;
float MovingObject::s_target_speed_alignment_weight = 0.03f;
float MovingObject::s_waypoint_attraction_weight = 0.02f;
float MovingObject::s_waypoint_speed_alignment_weight = 0.03f;
float MovingObject::s_fence_repel_weight = 50.0f;
float MovingObject::s_fence_size = 15.0f;
float MovingObject::s_min_cos_angle = 0.5f;
float MovingObject::s_force_randomness = 0.01f; // randomness to add to forces (can simulate wind)

float MovingObject::s_separation_min_dist = 1.0f;
// float MovingObject::separation_min_dist = 1;

// TODO command line param
float MovingObject::randomness_ = 0;

int MovingObject::next_id_ = 0;

// getter function so that other can access these parameters. 
int MovingObject::getBoidNumber() {
    return s_boid_number;
}
float MovingObject::getNeighborMaxDist() {
    return s_neighborhood_max_dist;
}
float MovingObject::getSeparationWeight() {
    return s_separation_weight;
}
float MovingObject::getSeparationMinDist() {
    return s_separation_min_dist;
}
float MovingObject::getCohesionWeight() {
    return s_cohesion_weight;
}
float MovingObject::getAlignmentWeight() {
    return s_alignment_weight;
}
float MovingObject::getMaxSpeed() {
    return s_max_speed;
}
float MovingObject::getTargetAttractionWeight() {
    return s_target_attraction_weight;
}
float MovingObject::getTargetSpeedAlignmentWeight() {
    return s_target_speed_alignment_weight;
}
float MovingObject::getWaypointAttractionWeight() {
    return s_target_attraction_weight;
}
float MovingObject::getWaypointSpeedAlignmentWeight() {
    return s_target_speed_alignment_weight;
}
float MovingObject::getMinCosAngle() {
    return s_min_cos_angle;
}
float MovingObject::getFenceRepelWeight() {
    return s_fence_repel_weight;
}
float MovingObject::getFenceSize() {
    return s_fence_size;
}
float MovingObject::getForceRandomness() {
    return s_force_randomness;
}

// static setter methods.
void MovingObject::setBoidNumber(int nbboids) {
    s_boid_number = nbboids;
}
void MovingObject::setNeighborMaxDist(float weight) {
    s_neighborhood_max_dist = weight;
}
void MovingObject::setSeparationWeight(float weight) {
    s_separation_weight = weight;
}
void MovingObject::setSeparationMinDist(float dist) {
    s_separation_min_dist = dist;
}

void MovingObject::setAlignmentWeight(float weight) {
    s_alignment_weight = weight;
}

void MovingObject::setCohesionWeight(float weight) {
    s_cohesion_weight = weight;
}
void MovingObject::setTargetAttractionWeight(float weight) {
    s_target_attraction_weight = weight;
}
void MovingObject::setTargetSpeedAlignmentWeight(float weight) {
    s_target_speed_alignment_weight = weight;
}
void MovingObject::setWaypointAttractionWeight(float weight) {
    s_target_attraction_weight = weight;
}
void MovingObject::setWaypointSpeedAlignmentWeight(float weight) {
    s_target_speed_alignment_weight = weight;
}
void MovingObject::setMinCosAngle(float value) {
    s_min_cos_angle = value;
}

void MovingObject::setFenceRepelWeight(float weight) {
    s_fence_repel_weight = weight;
}
void MovingObject::setFenceSize(float size) {
    s_fence_size = size;
}

void MovingObject::setMaxSpeed(float speed) {
    s_max_speed = speed;
}

// void MovingObject::set_velocity(const Vec3f& new_velocity) {
//         speed_ = new_velocity; 
//     }
void MovingObject::setForceRandomness(float value) {
    s_force_randomness = value;
}
bool MovingObject::are_at_safe_distance(const MovingObject &obj1, const MovingObject &obj2)
{
    float tmp_separation_min_dist = MovingObject::getSeparationMinDist();
    return (obj1.get_position() - obj2.get_position()).norm() > tmp_separation_min_dist;
}



bool MovingObject::are_neighbors(const MovingObject &left, const MovingObject &right)
{
    float temp_neighbor_max_dist = MovingObject::getNeighborMaxDist();
    return (left.get_position() - right.get_position()).norm() < temp_neighbor_max_dist;
}

MovingObject::MovingObject(const Vec3f &position, const Vec3f &speed) : id_(next_id_++),
                                                                        position_(position),
                                                                        speed_(speed),
                                                                        boid_type_(-1)
{
}

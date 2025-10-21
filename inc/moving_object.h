/*********************************************************************************************************************
 * File : moving_object.h                                                                                            *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#ifndef MOVING_OBJECT_H
#define MOVING_OBJECT_H

#include <Eigen/Dense>

using Vec3f = Eigen::Vector3f;

class MovingObject
{
private:
    // Static member variables to hold the boid parameters and weights for boid behavior.
    // could benefit refactor since not only boids are childs of moving objects... 
    static int s_boid_number;
    static float s_neighborhood_max_dist;
    static float s_separation_weight;
    static float s_separation_min_dist;
    static float s_alignment_weight;
    static float s_cohesion_weight;
    static float s_vision_distance;
    static float s_vision_FOV; // use min_cos_angle
    static float s_min_cos_angle; 
    static float s_max_speed;
    static float s_target_attraction_weight;
    static float s_target_speed_alignment_weight;
    static float s_waypoint_attraction_weight;
    static float s_waypoint_speed_alignment_weight;
    static float s_fence_repel_weight;
    static float s_fence_size; // fence radius for spheroid or side for cuboid
    static float s_force_randomness; // 

public:
    // static parameters getter
    static int getBoidNumber();
    static float getSeparationWeight();
    static float getSeparationMinDist();
    static float getNeighborMaxDist();
    static float getCohesionWeight();
    static float getAlignmentWeight();
    static float getMaxSpeed();
    static float getTargetAttractionWeight();
    static float getTargetSpeedAlignmentWeight();
    static float getWaypointAttractionWeight();
    static float getWaypointSpeedAlignmentWeight();
    static float getMinCosAngle();
    static float getFenceRepelWeight();
    static float getFenceSize();
    static float getForceRandomness();
    // Static setter methods to modify the weights.
    // These are what `main.cpp` will call to configure the simulation.
    static void setBoidNumber(int nbboids);
    static void setNeighborMaxDist(float weight);
    static void setSeparationWeight(float weight);
    static void setSeparationMinDist(float dist);
    static void setAlignmentWeight(float weight);
    static void setCohesionWeight(float weight);
    static void setMaxSpeed(float speed);
    static void setTargetAttractionWeight(float weight);
    static void setTargetSpeedAlignmentWeight(float weight);
    static void setWaypointAttractionWeight(float weight);
    static void setWaypointSpeedAlignmentWeight(float weight);
    static void setMinCosAngle(float value);
    static void setFenceRepelWeight(float weight);
    static void setFenceSize(float size);
    static void setForceRandomness(float value);

    MovingObject(const Vec3f &position, const Vec3f &speed = Vec3f(0, 0, 0));

    virtual ~MovingObject() = default;

    inline const Vec3f &get_position() const { return position_; }
    inline const Vec3f &get_speed() const { return speed_; }
    // for the waypoint manager
    void set_position(const Vec3f& new_position) { position_ = new_position; }
    void set_speed(const Vec3f& new_speed) { speed_ = new_speed; }
    
    inline int get_id() const { return id_; }
    inline int get_type() const { return boid_type_; }

    virtual Vec3f get_exerted_proximity_force(const MovingObject &boid) const = 0;
    virtual void update(float t) = 0;
    virtual void update_no_ang_speed_clamp(float t) = 0;
    virtual void draw() const = 0;
    // virtual void draw_boid() const = 0;

    // static float separation_min_dist_;
    static float randomness_;
    // static float min_cos_angle_;

    static bool are_neighbors(const MovingObject &left, const MovingObject &right);
    static bool are_at_safe_distance(const MovingObject &left, const MovingObject &right);

protected:
    static int next_id_;

    int id_;
    int boid_type_;

    Vec3f position_;
    Vec3f speed_;
};

#endif // MOVING_OBJECT_H

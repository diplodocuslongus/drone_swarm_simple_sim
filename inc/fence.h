/*********************************************************************************************************************
 * File : fence.h                                                                                                 *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#ifndef Fence_H
#define Fence_H

#include <functional>

#include "moving_object.h"

enum FenceType { SPHERE, CUBOID };

class Fence : public MovingObject
{
public:
    Fence(const Vec3f &position, float radius);
    Fence(const Vec3f &vertice1, const Vec3f &vertice2);

    virtual ~Fence() = default;

    Vec3f get_exerted_proximity_force(const MovingObject &object) const override;
    // Vec3f get_exerced_proximity_force_cub(const MovingObject &object) const override;

    void update(float t) override;
    void update_no_ang_velocity_clamp(float t) override;


    void draw() const override;

    // static float fence_repel_weight_;

private:
    FenceType type_; // spheroid or cuboid
    // for spheroid
    float radius_;
    float rad_max_;
    // for cuboid
    float min_x_,max_x_,min_y_,max_y_;
    float min_z_,max_z_;
};

#endif // Fence_H

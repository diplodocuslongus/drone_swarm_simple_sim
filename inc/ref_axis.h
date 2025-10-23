/*********************************************************************************************************************
 * File : ref_axis.h                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *                                                                                                                   *
 *********************************************************************************************************************/

#ifndef REF_AXIS_H
#define REF_AXIS_H

#include <functional>

#include "moving_object.h"

class RefAxis : public MovingObject
{
public:

    RefAxis(const Vec3f &vertice1, const Vec3f &vertice2);
    virtual ~RefAxis() = default;

    Vec3f get_exerted_proximity_force(const MovingObject &object) const override;

    void update(float t) override;
    void update_no_thrust(float t) override;
    void update_no_ang_velocity_clamp(float t) override;

    void draw() const override;

private:
    float last_t_;
    float min_x_,max_x_,min_y_,max_y_;
    float min_z_,max_z_;
};

#endif // 

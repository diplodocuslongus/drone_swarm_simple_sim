/*********************************************************************************************************************
 * File : target.h                                                                                                   *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#ifndef TARGET_H
#define TARGET_H

#include <functional>

#include "moving_object.h"

enum TargetMode {
    ATTRACT_AND_STOP,    // original target object behavior: Attract and hover over the point
    ATTRACT_AND_FLY_OVER // waypoint behavior: Attract then repel when close
};

class Target : public MovingObject
{
public:
    using UpdateSpeedFunc = std::function<void(float, Vec3f &)>;

    Target(
        const Vec3f &position,
        UpdateSpeedFunc update_speed_func = [](float, Vec3f &) {});

    virtual ~Target() = default;

    Vec3f get_exerted_proximity_force(const MovingObject &object) const override;
    // Vec3f get_exerted_proximity_force_object_only(const MovingObject &object) const override;
    void set_mode(TargetMode mode) { mode_ = mode; }

    void update(float t) override;
    void update_no_ang_speed_clamp(float t) override;

    void draw() const override;


private:
    float last_t_;
    UpdateSpeedFunc update_speed_func_;
    TargetMode mode_; 
};

#endif // TARGET_H

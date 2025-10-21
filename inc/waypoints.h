
/*********************************************************************************************************************
 * File : waypoints.h
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/
#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <vector>
#include "moving_object.h" // For Vec3f, various objects
#include "target.h"        // waypoint follows the force logic in target (waypointas are a kind of target)

class WaypointManager {
public:
    // constructor takes the list of points defining the path
    WaypointManager(const std::vector<Vec3f>& path);

    // returns a ref to the a single active target waypoint 
    Target& get_active_target() { return active_target_; }

    // logic to advance the path
    void update_path_progression(const Vec3f& swarm_center_position);
    void update_path_progression_per_boid(const Vec3f& swarm_center_position);

    // draw the whole path
    void draw_path() const;

    std::vector<Vec3f> g_formation_offsets; // not needed if update in systemEvolution
    bool has_swarm_stopped() const { return swarm_has_stopped_; }
private:
    std::vector<Vec3f> waypoints_; // full flight path
    int active_waypoint_index_;    // Current target index

    // The ONE object boids will interact with, placed at waypoints_[active_waypoint_index_]
    Target active_target_; 
    
    // threshold distance for switching to the next waypoint
    const float WAYPOINT_THRESHOLD = 5.0f; 

    // to check if last WP has been reached, if so, stop the drones
    bool swarm_has_stopped_ = false;
};

#endif // WAYPOINT_MANAGER_H

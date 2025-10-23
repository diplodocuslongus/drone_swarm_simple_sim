
// #define CONTINUE_AFTER_LAST // continues fly after last WP
// #define LOOP_BACK_AFTER_LAST // go back to 1st WP after last and loop
#define USE_3 // attracted back to last WP, breaks formation

// case where the whole swarm navigate to waypoint
// the same single waypoint acts like a target for each boids

#ifdef CONTINUE_AFTER_LAST
// works well for some WP configs
void WaypointManager::update_path_progression(const Vec3f& swarm_center_position) {

    // Entire path is finished. ---
    // The index is NOW pointing BEYOND the last valid waypoint index.
    if (active_waypoint_index_ >= waypoints_.size()) { 
        // We're done. Weights should remain zeroed.
        return; 
    }

    const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];
    float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();

    // Check for arrival at the current waypoint. ---
    if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
        
        // SWARM HAS REACHED current WAYPOINT, advance to the next one.
        active_waypoint_index_++; 

        // --- Check if the newly advanced index is BEYOND the path. ---
        if (active_waypoint_index_ >= waypoints_.size()) {
            // We just finished the final waypoint. Set final stop state.
            
            // Set the target's speed to zero (if it wasn't already).
            active_target_.set_velocity(Vec3f::Zero()); 

            // Turn off attraction and alignment forces.
            MovingObject::setWaypointAttractionWeight(0.0f); 
            MovingObject::setWaypointSpeedAlignmentWeight(0.0f);
            return; // Exit, the path is complete.
        }

        // Moving to the next (intermediate or final) waypoint. ---
        
        const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];
        active_target_.set_position(next_waypoint_pos);

        // --- Check if the new target is the final destination. ---
        if (active_waypoint_index_ == waypoints_.size() - 1) {
            
            // FINAL WP: Set speed to zero now, but KEEP the attraction/alignment 
            // active to maintain formation until the swarm reaches the final threshold.
            active_target_.set_velocity(Vec3f::Zero()); 
            
        } else {
            // INTERMEDIATE WP: Look ahead and cruise.
            
            const Vec3f& subsequent_waypoint_pos = waypoints_[active_waypoint_index_ + 1];
            Vec3f direction = (subsequent_waypoint_pos - next_waypoint_pos).normalized();
            active_target_.set_velocity(direction * 5.0f);  // no effect, WP target have no velocity here
            
            // only need 
            // to be reset if previously set to 0.0f.
            MovingObject::setWaypointAttractionWeight(0.02f); 
            MovingObject::setWaypointSpeedAlignmentWeight(0.03f);
        }
    }
}
#endif



#ifdef LOOP_BACK_AFTER_LAST
void WaypointManager::update_path_progression(const Vec3f& swarm_center_position) {

    // index shouldn't be larger than the path size since we reset it to loopback
    if (active_waypoint_index_ >= waypoints_.size()) { 
        // We're done. Weights should remain zeroed.
        std::cout << "we shouldn t be here\n"; // never called
        return; 
    }

    const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];
    float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();
    if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
        
        // 1. SWARM HAS REACHED current WAYPOINT, advance index.
        active_waypoint_index_++; 

        // 2. CHECK FOR LOOPING: If we're past the last index, loop back to 0.
        if (active_waypoint_index_ >= waypoints_.size()) {
            active_waypoint_index_ = 0;
        }

        // 3. SET THE TARGET: The target is always the new index.
        const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];
        active_target_.set_position(next_waypoint_pos);

        // 4. CALCULATE ALIGNMENT (Look-ahead)
        
        // Find the index of the waypoint *after* the next one.
        // If the next index is the LAST one, the subsequent index is 0.
        size_t subsequent_index;
        if (active_waypoint_index_ == waypoints_.size() - 1) {
            subsequent_index = 0; // Loop back
        } else {
            subsequent_index = active_waypoint_index_ + 1;
        }

        const Vec3f& subsequent_waypoint_pos = waypoints_[subsequent_index];
        Vec3f direction = (subsequent_waypoint_pos - next_waypoint_pos).normalized();
        
        // Use a reasonable cruising speed/velocity (e.g., 5.0f).
        // active_target_.set_velocity(direction * 5.0f); 

        // 5. RESTORE CRUSING WEIGHTS (Prevents acceleration/breakdown)
        // Ensure these are your base, stable values.
        MovingObject::setWaypointAttractionWeight(0.02f); 
        MovingObject::setWaypointSpeedAlignmentWeight(0.03f);
    }

    //  Check for arrival at the current waypoint. ---
    // if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
    //     
    //     // SWARM HAS REACHED current WAYPOINT, advance to the next one.
    //     active_waypoint_index_++; 
    //
    //     //  Moving to the next (intermediate or final) waypoint. ---
    //     
    //     const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];
    //     active_target_.set_position(next_waypoint_pos);
    //
    //     // final WP, loop back to first
    //     if (active_waypoint_index_ == waypoints_.size() - 1) {
    //         active_waypoint_index_ = 0;
    //         MovingObject::setWaypointAttractionWeight(1.02f); 
    //         MovingObject::setWaypointSpeedAlignmentWeight(1.03f);
    //         
    //         
    //     } else {
    //         // INTERMEDIATE WP: Look ahead and cruise.
    //         
    //         const Vec3f& subsequent_waypoint_pos = waypoints_[active_waypoint_index_ + 1];
    //         Vec3f direction = (subsequent_waypoint_pos - next_waypoint_pos).normalized();
    //         active_target_.set_velocity(direction * 1.0f); 
    //         
    //         // we assume the default static values are correct and only need 
    //         // to be reset if they were previously set to 0.0f.
    //         MovingObject::setWaypointAttractionWeight(0.02f); 
    //         MovingObject::setWaypointSpeedAlignmentWeight(0.03f);
    //     }
    // }
}
#endif

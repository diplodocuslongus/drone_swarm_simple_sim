
#ifdef USE_2

void WaypointManager::update_path_progression(const Vec3f& swarm_center_position) {
    if (active_waypoint_index_ >= waypoints_.size()  - 1) {
        // We are at the final waypoint. No progression needed.
        // TODO (?) set the active_target_ speed to zero here for a final hover effect.
             // MovingObject::setWaypointAttractionWeight(0.0); 
             // MovingObject::setWaypointSpeedAlignmentWeight(0.0);
        return;
    }

    const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];

    // Check if the center of the swarm has passed the waypoint
    float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();

    if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
        // SWARM HAS REACHED current WAYPOINT, advance to the next one.
        active_waypoint_index_++;

        const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];

        // Update the position of the physical Target object
        active_target_.set_position(next_waypoint_pos);

        // Calculate and set the Target's velocity (needed for alignment)
        // Set target speed toward the waypoint AFTER the next one 
        if (active_waypoint_index_ <= waypoints_.size() - 1) {
             Vec3f direction = (waypoints_[active_waypoint_index_ + 1] - next_waypoint_pos).normalized();
             // Set the target's speed to influence boid alignment
             active_target_.set_velocity(direction * 5.0f); 
             // MovingObject::setWaypointAttractionWeight(0.02f); 
             // MovingObject::setWaypointSpeedAlignmentWeight(0.03f);
        } else {
             // Final waypoint, set speed to zero (for a hovering/clustering effect)
             active_target_.set_velocity(Vec3f::Zero()); 
             // new: set the relevant attraction weights to zero, for all boids
             MovingObject::setWaypointAttractionWeight(0.0); 
             MovingObject::setWaypointSpeedAlignmentWeight(0.0);
        }
    }
}
#endif
#ifdef USE_3

void WaypointManager::update_path_progression(const Vec3f& swarm_center_position) {
    
    // Check 1: Has the swarm reached the FINAL waypoint? (active_waypoint_index_ is pointing at the last WP)
    if (active_waypoint_index_ == waypoints_.size() - 1) { 
        const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];
        float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();

        if (distance_to_waypoint < WAYPOINT_THRESHOLD / 1.0) {
            // SWARM HAS CROSSED THE FINAL THRESHOLD: INITIATE FULL STOP
            
            // Set index to indicate path is complete (will trigger return on next frame)
            active_waypoint_index_++; 
            
            // Turn off all waypoint forces globally.
            MovingObject::setWaypointAttractionWeight(0.0f); 
            MovingObject::setWaypointSpeedAlignmentWeight(0.0f);
            // active_target_.set_velocity(Vec3f::Zero()); // TODO: check, but seems unused Ensure the target itself is stopped

        }
    }
    
    // Check 2: If the path is COMPLETE (index has been incremented past the end), stop all processing.
    if (active_waypoint_index_ >= waypoints_.size() ){ //&& !swarm_hover_) {
    // if (active_waypoint_index_ >= waypoints_.size() && !swarm_hover_) {
        swarm_has_stopped_ = true; // new, to stop the drones over the last WP
        // if (!swarm_hover_){
        // // swarm_hover_ = true; // new, to stop the drones over the last WP
        // swarm_has_stopped_ = true; // new, to stop the drones over the last WP
        // }
        return;
    }

    // --- Normal Progression Logic ---
    const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];
    float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();

    // Check 3: Has the swarm reached the current INTERMEDIATE waypoint?
    if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
        // SWARM HAS REACHED current INTERMEDIATE WAYPOINT, advance to the next one.
        active_waypoint_index_++;
        
        const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];
        active_target_.set_position(next_waypoint_pos);

        // Only look ahead if there's a subsequent waypoint for smooth alignment.
        if (active_waypoint_index_ < waypoints_.size() - 1) {
             Vec3f direction = (waypoints_[active_waypoint_index_ + 1] - next_waypoint_pos).normalized();
             active_target_.set_velocity(direction * 5.0f); 
        } 
        // else {
        //      // We are now pointing at the final waypoint index (just set the position, speed will be handled by deceleration)
        //      active_target_.set_velocity(Vec3f::Zero()); // Set speed to zero NOW to start general slowdown
        // }
    }
    
    // --- Deceleration Logic  ---

    // Check if the current waypoint is the final destination
    if (active_waypoint_index_ >= waypoints_.size() - 1) {

        // Define the range over which the swarm will slow down (e.g., last 20 units)
        const float SLOWDOWN_DISTANCE = 100.0f; 
        const float MAX_SPEED = 5.1f; 

        // Calculate distance to the final waypoint
        const Vec3f& final_waypoint_pos = waypoints_[waypoints_.size() - 1];
        float final_distance = (swarm_center_position - final_waypoint_pos).norm();

        // Calculate the scale factor (0.0 when distance is 0, 1.0 when distance >= SLOWDOWN_DISTANCE)
        float speed_scale = std::min(final_distance / SLOWDOWN_DISTANCE, 1.0f);

        // Apply the reduced speed to the shared Target object
        // Note: We use the existing alignment direction

        // Find the direction (if it's not already set in the active_target_)
        Vec3f target_vel = active_target_.get_velocity().normalized();

        // Only apply if the current speed is non-zero (i.e., if the target is still moving)
            // std::cout << "lastWP: "<<  active_target_.get_velocity().norm() << " "; 
        if (active_target_.get_velocity().norm() > 0.1f) {
            active_target_.set_velocity(target_vel * MAX_SPEED * speed_scale);
            // std::cout << target_vel.norm() * MAX_SPEED * speed_scale << " "; 
        }
    }
    // if (active_waypoint_index_ == waypoints_.size() - 1) { 
    //     
    //     const float SLOWDOWN_DISTANCE = 130.0f; // Increase this to 30.0f or more for a smooth stop
    //     
    //     const Vec3f& final_waypoint_pos = waypoints_[waypoints_.size() - 1];
    //     float final_distance = (swarm_center_position - final_waypoint_pos).norm();
    //
    //     // The scale factor: 1.0 (full attraction) far away, decreases to 0.0 at the waypoint.
    //     float attraction_scale = std::min(final_distance / SLOWDOWN_DISTANCE, 1.0f);
    //
    //     // Apply the scaled attraction force: 
    //     // This smoothly turns OFF the "pull" toward the final waypoint.
    //     MovingObject::setWaypointAttractionWeight(0.02f * attraction_scale); 
    //
    //     // Keep alignment weight at full strength until the end for formation stability.
    //     MovingObject::setWaypointSpeedAlignmentWeight(0.03f);
    // } 
}

#endif

void WaypointManager::draw_path() const
{
    // glPushMatrix();
    // Vec3f curpos = waypoints_[0];
    // glTranslatef(curpos[0], curpos[1], curpos[2]);
    // glColor3f(0.5, 0.5, 0.5);
    // glutSolidSphere(1, 3, 3);
    // glPopMatrix();
    // Draw the entire path as a series of connected lines
    glColor3f(0.8f, 0.8f, 0.8f); // Light gray for the path line
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const Vec3f& pos = waypoints_[i];
        
        // Connect points with lines
        glVertex3f(pos[0], pos[1], pos[2]);
    }
    glEnd();
    glLineWidth(1.0f); // Reset line width

    // Draw the waypoints themselves as spheres, changing color based on status
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        const Vec3f& pos = waypoints_[i];
        
        glPushMatrix();
        glTranslatef(pos[0], pos[1], pos[2]);
        
        if (i < active_waypoint_index_) {
            // Passed Waypoint (e.g., Green)
            glColor3f(0.0f, 0.8f, 0.0f); 
        } else if (i == active_waypoint_index_) {
            // Active Waypoint (e.g., Yellow/Orange)
            glColor3f(1.0f, 0.6f, 0.0f); 
        } else {
            // Future Waypoint (e.g., White/Blue)
            glColor3f(0.5f, 0.5f, 1.0f); 
        }
        
        // Draw the sphere
        glutSolidSphere(1.0f, 10, 10); // Use a smaller radius, e.g., 1.0f
        glPopMatrix();
    }
}

/*********************************************************************************************************************
 * File : waypoints.cpp                                                                                                 *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#include <GL/glut.h>

#include "waypoints.h"
#include "gl_utils.h"
#include <iostream>

#define USE_3

WaypointManager::WaypointManager(const std::vector<Vec3f>& path) 
    // Initialize waypoints_ with the passed path
    : waypoints_(path), 
    
    // Initialize active_waypoint_index_
      active_waypoint_index_(0),
      
    // Initialize active_target_ using only the first waypoint's position.
    // The Target constructor will use its default (do-nothing) UpdateSpeedFunc.
      active_target_(path.at(0)) 
{
    // Ensure the path is not empty before proceeding
    if (path.empty()) {
        // TODO (if needed) Handle error 
        return; 
    }

    // Set initial mode for the first waypoint
    active_target_.set_mode(TargetMode::ATTRACT_AND_FLY_OVER);
    
    // Set target's initial speed toward next waypoint
    if (path.size() > 1) {
        Vec3f initial_direction = (path.at(1) - path.at(0)).normalized();
        // You should define a constant for the desired Boid flight speed
        const float DESIRED_FLIGHT_SPEED = 5.0f; 
        active_target_.set_speed(initial_direction * DESIRED_FLIGHT_SPEED);
    } else {
        // If only one waypoint, set speed to zero for an initial hover
        active_target_.set_speed(Vec3f::Zero()); 
    }

    // case where one wants to maintain formation shape
    // Assuming a grid spacing of 's' (e.g., s=5)
    // float s = 5.0f;
    // for (int row = 0; row < 3; ++row) {
    //     for (int col = 0; col < 3; ++col) {
    //         Vec3f offset;
    //         offset[0] = (col - 1) * s; // -s, 0, s
    //         offset[1] = 0.0f;          // Z-axis (or Y if 2D)
    //         offset[2] = (row - 1) * s; // -s, 0, s
    //         g_formation_offsets.push_back(offset);
    //     }
    // }
}

// alternative update: peform a per boid update
void WaypointManager::update_path_progression_per_boid(const Vec3f& swarm_center_position) {
    if (active_waypoint_index_ >= waypoints_.size() - 1) {
        // We are at the final waypoint. No progression needed.
        // TODO (?) set the active_target_ speed to zero here for a final hover effect.
         active_target_.set_speed(Vec3f::Zero()); 
        return;
    }

    const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];
    // const Vec3f& current_virtual waypoint_pos = current_waypoint_pos + active_target_.get_position(next_waypoint_pos);
    
    // Check if the center of the swarm has passed the waypoint
    float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();

    if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
        // SWARM HAS REACHED current WAYPOINT, advance to the next one.
        active_waypoint_index_++;
        
        const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];
        
        // Update the position of the physical Target object
        active_target_.set_position(next_waypoint_pos);
        
        // Calculate and set the Target's velocity (Crucial for alignment)
        // Set target speed toward the waypoint AFTER the next one (or just a look-ahead).
        if (active_waypoint_index_ < waypoints_.size() - 1) {
             Vec3f direction = (waypoints_[active_waypoint_index_ + 1] - next_waypoint_pos).normalized();
             // Set the target's speed to influence boid alignment
             active_target_.set_speed(direction * 5.0f); 
        } else {
             // Final waypoint, set speed to zero (for a hovering/clustering effect)
             active_target_.set_speed(Vec3f::Zero()); 
             // MovingObject::setWaypointAttractionWeight(0.0); 
             // MovingObject::setWaypointSpeedAlignmentWeight(0.0);
        }
    }
}
// case where the whole swarm navigate to waypoint
// the same single waypoint acts like a target for each boids

#ifdef USE_1
void WaypointManager::update_path_progression(const Vec3f& swarm_center_position) {

    // --- State 1: Entire path is finished. ---
    // The index is NOW pointing BEYOND the last valid waypoint index.
    if (active_waypoint_index_ >= waypoints_.size()) { 
        // We're done. Weights should remain zeroed.
        return; 
    }

    const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];
    float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();

    // --- State 2: Check for arrival at the current waypoint. ---
    if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
        
        // SWARM HAS REACHED current WAYPOINT, advance to the next one.
        active_waypoint_index_++; 

        // --- Check if the newly advanced index is BEYOND the path. ---
        if (active_waypoint_index_ >= waypoints_.size()) {
            // We just finished the final waypoint. Set final stop state.
            
            // Set the target's speed to zero (if it wasn't already).
            active_target_.set_speed(Vec3f::Zero()); 

            // Turn off attraction and alignment forces.
            MovingObject::setWaypointAttractionWeight(0.0f); 
            MovingObject::setWaypointSpeedAlignmentWeight(0.0f);
            return; // Exit, the path is complete.
        }

        // --- State 3: Moving to the next (intermediate or final) waypoint. ---
        
        const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];
        active_target_.set_position(next_waypoint_pos);

        // --- Check if the new target is the final destination. ---
        if (active_waypoint_index_ == waypoints_.size() - 1) {
            
            // FINAL WP: Set speed to zero now, but KEEP the attraction/alignment 
            // active to maintain formation until the swarm reaches the final threshold.
            active_target_.set_speed(Vec3f::Zero()); 
            
            // NOTE: DO NOT set weights to zero here yet. Keep them active so 
            // the virtual targets hold the formation in place as they slow down.
            
        } else {
            // INTERMEDIATE WP: Look ahead and cruise.
            
            const Vec3f& subsequent_waypoint_pos = waypoints_[active_waypoint_index_ + 1];
            Vec3f direction = (subsequent_waypoint_pos - next_waypoint_pos).normalized();
            active_target_.set_speed(direction * 5.0f); 
            
            // we assume the default static values are correct and only need 
            // to be reset if they were previously set to 0.0f.
            MovingObject::setWaypointAttractionWeight(0.02f); 
            MovingObject::setWaypointSpeedAlignmentWeight(0.03f);
        }
    }
}
#endif
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
             active_target_.set_speed(direction * 5.0f); 
             // MovingObject::setWaypointAttractionWeight(0.02f); 
             // MovingObject::setWaypointSpeedAlignmentWeight(0.03f);
        } else {
             // Final waypoint, set speed to zero (for a hovering/clustering effect)
             active_target_.set_speed(Vec3f::Zero()); 
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

        if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
            // SWARM HAS CROSSED THE FINAL THRESHOLD: INITIATE FULL STOP
            
            // Set index to indicate path is complete (will trigger return on next frame)
            active_waypoint_index_++; 
            
            // Turn off all waypoint forces globally.
            MovingObject::setWaypointAttractionWeight(0.0f); 
            MovingObject::setWaypointSpeedAlignmentWeight(0.0f);
            active_target_.set_speed(Vec3f::Zero()); // Ensure the target itself is stopped

            // return; // Path finished.
        }
        // swarm_has_stopped_ = true; // new, to stop the drones over the last WP
    }
    
    // Check 2: If the path is COMPLETE (index has been incremented past the end), stop all processing.
    if (active_waypoint_index_ >= waypoints_.size()) {
        return;
    }

    // --- Normal Progression Logic ---
    const Vec3f& current_waypoint_pos = waypoints_[active_waypoint_index_];
    float distance_to_waypoint = (swarm_center_position - current_waypoint_pos).norm();

    // Check 3: Has the swarm reached the current INTERMEDIATE waypoint?
    if (distance_to_waypoint < WAYPOINT_THRESHOLD) {
        // SWARM HAS REACHED current INTERMEDIATE WAYPOINT, advance to the next one.
        active_waypoint_index_++;
        
        // Safety check (should be redundant now, but harmless)
        // if (active_waypoint_index_ >= waypoints_.size()) {
        //     return; // Should have been caught by Check 1/3, but ensures safety.
        // }

        const Vec3f& next_waypoint_pos = waypoints_[active_waypoint_index_];
        active_target_.set_position(next_waypoint_pos);

        // Only look ahead if there's a subsequent waypoint for smooth alignment.
        if (active_waypoint_index_ < waypoints_.size() - 1) {
             Vec3f direction = (waypoints_[active_waypoint_index_ + 1] - next_waypoint_pos).normalized();
             active_target_.set_speed(direction * 5.0f); 
        } 
        // else {
        //      // We are now pointing at the final waypoint index (just set the position, speed will be handled by deceleration)
        //      active_target_.set_speed(Vec3f::Zero()); // Set speed to zero NOW to start general slowdown
        // }
    }
    
    // --- Deceleration Logic  ---

    // Check if the current waypoint is the final destination
    if (active_waypoint_index_ >= waypoints_.size() - 1) {

        // Define the range over which the swarm will slow down (e.g., last 20 units)
        const float SLOWDOWN_DISTANCE = 100.0f; 
        const float MAX_SPEED = 5.0f; 

        // Calculate distance to the final waypoint
        const Vec3f& final_waypoint_pos = waypoints_[waypoints_.size() - 1];
        float final_distance = (swarm_center_position - final_waypoint_pos).norm();

        // Calculate the scale factor (0.0 when distance is 0, 1.0 when distance >= SLOWDOWN_DISTANCE)
        float speed_scale = std::min(final_distance / SLOWDOWN_DISTANCE, 1.0f);

        // Apply the reduced speed to the shared Target object
        // Note: We use the existing alignment direction

        // Find the direction (if it's not already set in the active_target_)
        Vec3f target_vel = active_target_.get_speed().normalized();

        // Only apply if the current speed is non-zero (i.e., if the target is still moving)
            // std::cout << "lastWP: "<<  active_target_.get_speed().norm() << " "; 
        if (active_target_.get_speed().norm() > 0.1f) {
            active_target_.set_speed(target_vel * MAX_SPEED * speed_scale);
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

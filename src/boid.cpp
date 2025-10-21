/*********************************************************************************************************************
 * File : moving_object.cpp                                                                                          *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)

 *********************************************************************************************************************/

#include <GL/glut.h>

#include "boid.h"
#include "gl_utils.h"
#include <iostream>

#include <plog/Log.h>

// message latency
float Boid::s_message_latency = 0.01; 
// static setter methods.
void Boid::setMessageLatency(float latency) {
    s_message_latency = latency;
        std::cout << "mesg latency set to " << Boid::s_message_latency << std::endl;
}

float Boid::getMessageLatency() {
    return s_message_latency;
}

Boid::Boid(const Vec3f &position, const Vec3f &speed) : MovingObject(position, speed)
{
    boid_type_ = 1;

    last_t_ = 0;

    n_neighbors_ = 0;
    avg_position_ = Vec3f(0, 0, 0);
    avg_speed_ = Vec3f(0, 0, 0);
    proximity_force_ = Vec3f(0, 0, 0);

    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    color_ = Vec3f(r, g, b);
}

// void Boid::add_neighbor(const MovingObject &object)
// {
//     // PLOG_INFO << "boid type " << object.get_type();
//     if (boid_type_ == object.get_type()) // check that the object (target, boid, obstacle, fence) is a boid (boid_type_ = 1)
//     {
//         // Vision angle
//         const Vec3f diff = object.get_position() - position_;
//         float temp_min_cos_ang= MovingObject::getMinCosAngle();
//         if (diff.dot(speed_) >= temp_min_cos_ang * speed_.norm() * diff.norm())
//         // if (diff.dot(speed_) >= min_cos_angle_ * speed_.norm() * diff.norm())
//         {
//             n_neighbors_++;
//             // Cohesion
//             avg_position_ += object.get_position();
//             // Alignment
//             avg_speed_ += object.get_speed();
//         }
//     }
//
//     // Separation + Target attraction
//     proximity_force_ += object.get_exerted_proximity_force(*this);
//     if (object.get_type() == 2)
//         PLOG_INFO << "target force " << proximity_force_.norm();
//     else if (object.get_type() == 4)
//         PLOG_INFO << "fence force boid #" << proximity_force_.norm();
//     // std::cout << "force " << proximity_force_ << "  ";
// }
void Boid::add_neighbor(const MovingObject &object)
{
    // PLOG_INFO << "boid type " << object.get_type();
    if (boid_type_ == object.get_type()) // check that the object (target, boid, obstacle, fence) is a boid (boid_type_ = 1)
    {
        // Vision angle
        const Vec3f diff = object.get_position() - position_;
        float temp_min_cos_ang= MovingObject::getMinCosAngle();
        if (diff.dot(speed_) >= temp_min_cos_ang * speed_.norm() * diff.norm())
        {
            n_neighbors_++;
            // Cohesion
            avg_position_ += object.get_position();
            // Alignment
            avg_speed_ += object.get_speed();
        }
        // Separation 
        proximity_force_ += object.get_exerted_proximity_force(*this);
    }
    else{
        // fences, targets,....
        external_object_force_ += object.get_exerted_proximity_force(*this);
    }
    // if (object.get_type() == 2)
    //     PLOG_INFO << "target force " << proximity_force_.norm();
    // else if (object.get_type() == 4)
    //     PLOG_INFO << "\n\nfence force boid #" << external_object_force_.norm();
    // std::cout << "force " << proximity_force_ << "  ";
}

Vec3f Boid::get_exerted_proximity_force(const MovingObject &object) const
{
    if (object.get_id() == id_) // Can't repel itself
        return Vec3f(0, 0, 0);

    // Separation
    float tmp_separation_weight = MovingObject::getSeparationWeight();
    float tmp_separation_min_dist = MovingObject::getSeparationMinDist();
    const Vec3f pos_diff = object.get_position() - position_;
    float pos_diff_norm = pos_diff.norm();
    const auto dist = std::max(tmp_separation_min_dist, pos_diff_norm); // In case close to zero
    // std::cout << "DIST  = " << dist << std::endl;
    // increase proximity force if below min separation distance 
    if (pos_diff_norm < tmp_separation_min_dist)
        tmp_separation_weight *=10.0  ;
    return tmp_separation_weight * pos_diff / (dist * dist);
}

// process messages, compute forces and interpolate
// t: nowtime (current time in the simulation)
void Boid::processMessagesAndInterpolate(double t) {
    // Reset accumulators
    // n_neighbors_ = 0;
    // avg_position_ = Vec3f(0, 0, 0);
    // avg_speed_ = Vec3f(0, 0, 0);
    // proximity_force_ = Vec3f(0, 0, 0);

    // move released messages into neighbor_history_ (this is done by getVisibleMessages)
    (void)getVisibleMessages(t); // we can ignore returned vector; function already pushes into history

    // Snapshot neighbor IDs to avoid iterator invalidation
    std::vector<int> neighbor_ids;
    neighbor_ids.reserve(neighbor_history_.size());
    for (const auto &p : neighbor_history_) neighbor_ids.push_back(p.first);

    for (int neighbor_id : neighbor_ids) {
        NeighborState interp;
        if (!getInterpolatedNeighborState(neighbor_id, t, interp)) continue;

        const Vec3f neighbor_pos = interp.position;
        const Vec3f neighbor_speed = interp.velocity;

        const Vec3f pos_diff = neighbor_pos - position_;
        const float pos_diff_norm = pos_diff.norm();

        // vision cone check (handle near-zero speed)
        float temp_min_cos_ang = MovingObject::getMinCosAngle();
        bool in_vision = true;
        if (speed_.norm() > 1e-6f)
            in_vision = (pos_diff.dot(speed_) >= temp_min_cos_ang * speed_.norm() * pos_diff_norm);

        if (in_vision) {
            n_neighbors_++;
            neighbor_confidence_weight_sum_ +=  interp.state_confidence;
            avg_position_ += interp.state_confidence* neighbor_pos; // degrade position info based on the boid state confidence
            avg_speed_ += interp.state_confidence* neighbor_speed; // degrade speed info based on the boid state confidence
        }

        // separation: point away from neighbor -> negative pos_diff
        float tmp_separation_weight = MovingObject::getSeparationWeight();
        float tmp_separation_min_dist = MovingObject::getSeparationMinDist();
        const float dist = std::max(tmp_separation_min_dist, pos_diff_norm);
        if (pos_diff_norm < tmp_separation_min_dist) tmp_separation_weight *= 10.0f;
        proximity_force_ -=interp.state_confidence* tmp_separation_weight * pos_diff / (dist * dist);
    }

}


// process messages / update boid
// meant to replace add_neighbor when we use messaging
void Boid::processMessages(double t) {
    // Clear
    n_neighbors_ = 0;
    avg_position_ = Vec3f(0, 0, 0);
    avg_speed_ = Vec3f(0, 0, 0);
    proximity_force_ = Vec3f(0, 0, 0);
    auto msgs = getVisibleMessages(t);

    for (const auto& msg : msgs) {
        // Look up neighbor’s *perceived* position
        // this part is from add_neighbor
        Vec3f neighbor_pos = msg.position;
        Vec3f neighbor_speed = msg.velocity;

        const Vec3f pos_diff = neighbor_pos - position_;
        float pos_diff_norm = pos_diff.norm();
        float temp_min_cos_ang = MovingObject::getMinCosAngle();
        if (pos_diff.dot(speed_) >= temp_min_cos_ang * speed_.norm() * pos_diff_norm) {
            n_neighbors_++;
            // Cohesion
            avg_position_ += neighbor_pos;
            avg_speed_ += neighbor_speed;  
        }
        // Separation + target attraction
        // separation force must also use neighbor_pos
        // This section is from get_exerted_proximity_force
        float tmp_separation_weight = MovingObject::getSeparationWeight();
        float tmp_separation_min_dist = MovingObject::getSeparationMinDist();
        // std::cout << "pos_diff_norm " << pos_diff_norm << std::endl;
        const auto dist = std::max(tmp_separation_min_dist, pos_diff_norm);
        if (pos_diff_norm < tmp_separation_min_dist)
            tmp_separation_weight *= 10.0;
        // this is exactly what is returned by get_exerted_proximity_force
        proximity_force_ -= tmp_separation_weight * pos_diff / (dist * dist);
    }
}

void Boid::update_no_ang_speed_clamp(float t)
{
    Vec3f speed_incr(0, 0, 0);
    float cohesion_weight = MovingObject::getCohesionWeight();
    float align_weight = MovingObject::getAlignmentWeight();
    // Update speed
    if (n_neighbors_ != 0)
    {
        const Vec3f cohesion_force = avg_position_ / n_neighbors_ - position_;
        const Vec3f alignment_force = avg_speed_ / n_neighbors_ - speed_;

        speed_incr += cohesion_weight * cohesion_force;
        speed_incr += align_weight * alignment_force;
    }

    const float dv_x = -0.5 + static_cast<float>(std::rand()) / RAND_MAX;
    const float dv_y = -0.5 + static_cast<float>(std::rand()) / RAND_MAX;
    const float dv_z = -0.5 + static_cast<float>(std::rand()) / RAND_MAX;

    const Vec3f random_force = randomness_ * Vec3f(dv_x, dv_y, dv_z);
    speed_incr += random_force;

    speed_incr += proximity_force_;

    // Clamp speed if needed
    float tmp_max_speed = MovingObject::getMaxSpeed();
    speed_ += speed_incr;
    if (speed_.norm() > tmp_max_speed)
        speed_ = tmp_max_speed * speed_.normalized();

    // Update position
    const float dt = (t - last_t_);
    last_t_ = t;
    position_ += speed_ * dt;

    // Update color: Red if a lot of proximity forces
    const double cos_angle_x = 0.5 * (1 + speed_.dot(Vec3f::UnitX()) / speed_.norm());
    const double cos_angle_y = 0.5 * (1 + speed_.dot(Vec3f::UnitY()) / speed_.norm());
    const double cos_angle_z = 0.5 * (1 + speed_.dot(Vec3f::UnitZ()) / speed_.norm());
    color_ = Vec3f(cos_angle_x, cos_angle_y, cos_angle_z);

    // Clear
    n_neighbors_ = 0;
    avg_position_ = Vec3f(0, 0, 0);
    avg_speed_ = Vec3f(0, 0, 0);
    proximity_force_ = Vec3f(0, 0, 0);
}

void Boid::update(float t)
{
    Vec3f speed_incr(0, 0, 0);
    float max_angular_speed = 0.05f; // 0.05f  in deg/s, to avoid instantaneous flips
    float max_angular_speed_rad = max_angular_speed * 3.14/180.0; 

    // if (n_neighbors_ != 0)
    // {
    //     const Vec3f cohesion_force = avg_position_ / n_neighbors_ - position_;
    //     const Vec3f alignment_force = avg_speed_ / n_neighbors_ - speed_;
    if (neighbor_confidence_weight_sum_ > 0.0) {
        Vec3f cohesion_force  = (avg_position_ / neighbor_confidence_weight_sum_) - position_;
        Vec3f alignment_force = (avg_speed_ / neighbor_confidence_weight_sum_) - speed_;

        float cohesion_weight = MovingObject::getCohesionWeight();
        speed_incr += cohesion_weight * cohesion_force;
        float align_weight = MovingObject::getAlignmentWeight();
        speed_incr += align_weight * alignment_force;
    }

    const float dv_x = -0.5 + static_cast<float>(std::rand()) / RAND_MAX;
    const float dv_y = -0.5 + static_cast<float>(std::rand()) / RAND_MAX;
    const float dv_z = -0.5 + static_cast<float>(std::rand()) / RAND_MAX;

    float force_randomness = MovingObject::getForceRandomness();
    const Vec3f random_force = force_randomness * Vec3f(dv_x, dv_y, dv_z);
    speed_incr += random_force;
    PLOG_DEBUG << "random force in update: " << random_force;
    // std::cout << " speed inc (rand)" << speed_incr << " " ;

    speed_incr += proximity_force_;
    // std::cout << " speed inc" << speed_incr << " " ;

    // NEW:  add external forces directly
    // speed_incr += external_object_force_;
    float tmp_max_speed = MovingObject::getMaxSpeed();
    // time step
    const float dt = (t - last_t_);
    last_t_ = t;

    const Vec3f target_speed = speed_ + speed_incr;
    float current_speed_norm = speed_.norm();
    float target_speed_norm = target_speed.norm();

    // Handle boid becoming stationary or starting from rest
    if (current_speed_norm < 1e-6) {
        speed_ = target_speed;
    } else {
        const Vec3f current_direction = speed_.normalized();

        // Handle target speed close to zero
        if (target_speed_norm < 1e-6) {
            speed_ = Vec3f::Zero();
        } else {
            const Vec3f target_direction = target_speed.normalized();
            const Vec3f rotation_axis_unnormalized = current_direction.cross(target_direction);
            
            if (rotation_axis_unnormalized.norm() < 1e-6) {
                // The direction doesn't change, just the magnitude
                speed_ = target_direction * target_speed_norm;
            } else {
                const Vec3f rotation_axis = rotation_axis_unnormalized.normalized();
                
                // Clamp the dot product to the valid range [-1, 1] to prevent NaN
                float dot_product = current_direction.dot(target_direction);
                dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
                
                float angle = acos(dot_product);
                float clamped_angle = std::min(angle, max_angular_speed_rad);
                if (angle > max_angular_speed_rad){
                    // std::cout << "angle: "<< angle << "clamped_angle " << clamped_angle << std::endl;
                }

                Eigen::AngleAxisf rotation(clamped_angle, rotation_axis);
                Vec3f new_speed_direction = rotation * current_direction;
                
                speed_ = new_speed_direction * target_speed_norm;
            }
        }
    }

    // for contribution from non-boid objects, bypass the angle limitation
    // we expect a strong reaction to fence, target, but keep smooth sterring when
    // boid interact with one another (well TODO, except for when they almost collide!)
    speed_+= external_object_force_;
    // Clamp speed 
    if (speed_.norm() > tmp_max_speed)
        speed_ = tmp_max_speed * speed_.normalized();

    // Update position
    position_ += speed_ * dt;
    // std::cout << " speed: "<< position_ << std::endl;

    // Update color: Red if a lot of proximity forces
    const double cos_angle_x = 0.5 * (1 + speed_.dot(Vec3f::UnitX()) / speed_.norm());
    const double cos_angle_y = 0.5 * (1 + speed_.dot(Vec3f::UnitY()) / speed_.norm());
    const double cos_angle_z = 0.5 * (1 + speed_.dot(Vec3f::UnitZ()) / speed_.norm());
    color_ = Vec3f(cos_angle_x, cos_angle_y, cos_angle_z);

}

void Boid::resetAccumulators() {
    n_neighbors_ = 0;
    avg_position_ = Vec3f(0, 0, 0);
    avg_speed_ = Vec3f(0, 0, 0);
    proximity_force_ = Vec3f(0, 0, 0);
    external_object_force_ = Vec3f(0, 0, 0);
    neighbor_confidence_weight_sum_ = 0; 
}
void Boid::draw_wing() const
{
    glPushMatrix();
    glRotatef(80 * cosf(last_t_ / 0.2), 1, 0, 0);
    glColor3f(color_[0], color_[1], color_[2]);
    glBegin(GL_TRIANGLES);
    glNormal3f(0, 0, 1);
    glVertex3f(0.3, 0, 0);
    glVertex3f(-0.3, 0, 0);
    glVertex3f(0, 0.5, 0);
    glEnd();
    glPopMatrix();
}

// void Boid::draw_boid() const
// {
//     glPushMatrix();
//
//     glTranslatef(position_[0], position_[1], position_[2]);
//     GlUtils::align_view(speed_);
//     GlUtils::draw_box(Vec3f(0.6, 0.1, 0.1), color_);
//
//     bool wing_beat = true;
//     if (wing_beat)
//     {
//     glPushMatrix();
//     glTranslatef(0, 0.1, 0);
//     draw_wing();
//     glPopMatrix();
//
//     glPushMatrix();
//     glRotatef(180, 0, 0, 1);
//     glTranslatef(0, 0.1, 0);
//     draw_wing();
//     glPopMatrix();
//     }
//     glTranslatef(0.6, 0, 0);
//     GlUtils::draw_pyramid(Vec3f(0.4, 0.1, 0.1), Vec3f(1.0, 1.0, 0.0));
//     // GlUtils::draw_pyramid(Vec3f(1.0, 0.2, 0.2), color_);
//     glPopMatrix();
// }

// draw a "X" shape to model a drone
// could add color (red on the left, greeen on right and white at tail)
void Boid::draw() const
// void Boid::draw_drone() const
{
    glPushMatrix();

    // move boid in place and orient it properly
    glTranslatef(position_[0], position_[1], position_[2]);
    GlUtils::align_view(speed_);

    // draw first arm of the "X"
    glPushMatrix();
    glRotatef(-45, 0, 0, 1);
    GlUtils::draw_box(Vec3f(0.6, 0.1, 0.1), color_);
    glPopMatrix();
    
    // draw 2nd arm of the "X"
    glPushMatrix();
    glRotatef(45, 0, 0, 1);
    GlUtils::draw_box(Vec3f(0.6, 0.1, 0.1), color_);
    glPopMatrix();

    glPopMatrix();
}

void Boid::receiveMessage(const BoidMessage &msg) {
    inbox_.push_back(msg);
}

// std::vector<BoidMessage> Boid::getVisibleMessages(double current_time) {
//     std::vector<BoidMessage> visible;
//      
//     float tmp_msg_latency = Boid::getMessageLatency();
//     while (!inbox_.empty()) {
//         const auto &msg = inbox_.front();
//         if (current_time - msg.timestamp >= tmp_msg_latency) {
//             visible.push_back(msg);
//             inbox_.pop_front();
//         } else {
//             break;
//         }
//     }
//     return visible;
//     }


// messsages are read if they fall within the message latency (a parameter)
std::vector<BoidMessage> Boid::getVisibleMessages(double current_time) {
    std::vector<BoidMessage> visible;
    double tmp_msg_latency = Boid::getMessageLatency();

    while (!inbox_.empty()) {
        const BoidMessage msg = inbox_.front(); // copy, avoid referencing after pop
        if (current_time - msg.timestamp >= tmp_msg_latency) {
            visible.push_back(msg);
            inbox_.pop_front();

            // Save into neighbor_history_
            NeighborState state{ msg.position, msg.velocity, msg.timestamp };
            auto &hist = neighbor_history_[msg.sender_id];
            hist.push_back(state);

            // Prune stale entries older than a time window (e.g. keep last 2s)
            double keep_since = current_time - 2.0; // keep 2 seconds of history
            while (!hist.empty() && hist.front().timestamp < keep_since) {
                hist.pop_front();
            }
        } else {
            break;
        }
    }
    return visible;
}

     
// interpolate the state (position and velocity) of neighbors between
// query time (which is nowtime, i.e. current time) and when messages are updated (after latency delay)
bool Boid::getInterpolatedNeighborState(int neighbor_id, double query_time, NeighborState &out) {
    auto it = neighbor_history_.find(neighbor_id);
    if (it == neighbor_history_.end()) return false;
    auto &hist = it->second;
    if (hist.empty()) return false; // no message in the history of message, exit

    // target time: the time at which the neighbor's state (position, velocity, time stamp) was recorded (perceived time)
    double target_time = query_time - Boid::getMessageLatency();
    // std::cout << "mesg latency " << Boid::getMessageLatency() << std::endl;

    double dt = fabs(target_time);

    // clamp  target_time if it is beyond or below a time limit
    /*
    double min_look_back_time = query_time - 0.1; // don’t look further than 0.1s into the past
    double max_look_fwd_time = query_time + 0.1; // don’t look further than 0.1s into the past
    if (target_time < min_look_back_time) {
        target_time = min_look_back_time;
        // return false;
    }
    if (target_time > max_look_fwd_time) {
        target_time = min_look_back_time;
        // return false;
    }
    */
    // max time horizon for meaningful data
    // double horizon = 2.0 * Boid::getMessageLatency(); // or a fixed time, was 0.5
    double horizon = 0.5; //2.0 * Boid::getMessageLatency(); // or a fixed time, was 0.5

    // Confidence = 1 when fresh, decays to 0 at horizon
    double state_confidence = 0.1;
    // state_confidence = std::max(0.00, 1.0 - dt / horizon);
    // std::cout << "state conf " << state_confidence << std::endl;
    double tau = 0.1; // Boid::getMessageLatency(); // or a fixed value
    // floor value when stale
    double min_conf = 0.05;  

    if (dt < 1.0) {
        // exponential decay dominates at short horizons
        state_confidence = exp(-dt / tau);
    } else if (dt < 3.0) {
        // linear fade between [1s, 3s], gradually goes down to min_conf
        double alpha = (dt - 1.0) / (3.0 - 1.0);
        double lin_conf = (1.0 - alpha) * exp(-1.0 / tau) + alpha * min_conf;
        state_confidence = lin_conf;
    } else {
        // fully stale, but keep a tiny anchor
        state_confidence = min_conf;
    }


    if (state_confidence <= 0.0) {
        // std::cout << "state confidence < 0 \n\n" ;
        return false; // fully stale, ignore neighbor
    }
    // If we only have one sample, extrapolate with its velocity
    if (hist.size() == 1) {
        const auto &s = hist.front();
        double dt = target_time - s.timestamp;
        out.position = s.position + s.velocity * dt;
        out.velocity = s.velocity;
        out.timestamp = target_time;
        out.state_confidence = state_confidence;
        return true;
    }

    // If target_time is before earliest sample (i.e. the "first in" message), extrapolate backwards from first sample
    if (target_time <= hist.front().timestamp) {
        const auto &s = hist.front();
        double dt = target_time - s.timestamp;
        out.position = s.position + s.velocity * dt;
        out.velocity = s.velocity;
        out.timestamp = target_time;
        out.state_confidence = state_confidence;
        return true;
    }

    // If target_time is after latest sample, extrapolate forward from last sample
    if (target_time >= hist.back().timestamp) {
        const auto &s = hist.back();
        double dt = target_time - s.timestamp;
        out.position = s.position + s.velocity * dt;
        out.velocity = s.velocity;
        out.timestamp = target_time;
        out.state_confidence = state_confidence;
        return true;
    }

    // Otherwise, find the bracket s1 <= target_time <= s2 and linearly interpolate
    for (size_t i = 1; i < hist.size(); ++i) {
        const auto &s1 = hist[i - 1];
        const auto &s2 = hist[i];
        if (s1.timestamp <= target_time && target_time <= s2.timestamp) {
            double denom = (s2.timestamp - s1.timestamp);
            if (denom == 0.0) {
                out.position = s2.position;
                out.velocity = s2.velocity;
                out.timestamp = target_time;
                out.state_confidence = state_confidence;
                return true;
            }
            double alpha = (target_time - s1.timestamp) / denom;
            out.position = s1.position + (s2.position - s1.position) * alpha;
            out.velocity = s1.velocity + (s2.velocity - s1.velocity) * alpha;
            out.timestamp = target_time;
            out.state_confidence = state_confidence;
            return true;
        }
    }

    // If we got here then something weird happened; return the last sample
    out = hist.back();
    out.timestamp = target_time;
    out.state_confidence = state_confidence;
    return true;
}



const std::deque<BoidMessage>& Boid::peekInbox() const {
        return inbox_;
}

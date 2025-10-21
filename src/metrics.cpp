/*********************************************************************************************************************
 * File : metrics.cpp                                                                                          *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#include <GL/glut.h>

#include "metrics.h"
#include <iostream>
#include <iomanip> // For std::setw, std::left, std::right
#include <string>

#include <numeric> // For std::accumulate and other utilities


// Eigen::Vector3f calcStdDev(const Eigen::MatrixXf& positions);
Vec3f calcStdDev(const Eigen::MatrixXf& positions);
double calcStddDev(const Eigen::VectorXd& data);
Metrics::Metrics()
{
}
float calcStdDev(std::vector <float> &positions);

// float calcStdDev(float positions) {
float calcStdDev(std::vector <float> &positions){
// Eigen::Vector3f calcStdDev(const Eigen::MatrixXf& positions) {
    // TODO: check sanity
    // if (positions.cols() == 0) {
    //     return Eigen::Vector3f::Zero();
    // }
    //     if (myVector.empty()) {
    //     std::cout << "The vector is empty, cannot calculate mean." << std::endl;
    //     return 1; // Indicate an error
    // }
    float sum = std::accumulate(positions.begin(), positions.end(), 0.0f);
    float mean_pos = sum / positions.size();

    // squared differences
      double squaredDiffSum = 0.0;
    for (float value : positions) {
        squaredDiffSum += std::pow(value - mean_pos, 2);
    }
    float variance = squaredDiffSum / positions.size();

    return std::sqrt(variance);
}

Vec3f calcStdDev(const Eigen::MatrixXf& positions) {
// Eigen::Vector3f calcStdDev(const Eigen::MatrixXf& positions) {
    if (positions.cols() == 0) {
        return Eigen::Vector3f::Zero();
    }
    Eigen::Vector3f mean_pos = positions.rowwise().mean();

    // squared differences
    Eigen::MatrixXf diffs = positions.colwise() - mean_pos;
    Eigen::MatrixXf squared_diffs = diffs.array().square();

    //  mean of the squared differences (variance)
    Eigen::Vector3f variance = squared_diffs.rowwise().mean();

    return variance.array().sqrt();
}

double calcStddDev(const Eigen::VectorXd& data) {
    if (data.size() == 0) {
        return 0.0; 
    }
    double mean = data.mean();

    // for sample standard deviation, divide by (data.size() - 1)
    double std_dev = std::sqrt((data.array() - mean).square().sum() / data.size()); 

    return std_dev;
}

Vec3f Metrics::swarm_center_of_gravity(const std::vector<Boid>& boids)
{
    if (boids.empty()) {
        return Vec3f(0.0,0.0,0.0);
    }
    // boid positions into Eigen::MatrixXf
    Eigen::MatrixXf allboidpos(3, boids.size());
    Eigen::MatrixXf allboidpos_from_cog(3, boids.size());

    //  center of gravity
    Vec3f center_of_gravity = Vec3f(0,0,0);
    // range based loop :-)
    for (const auto& boid : boids) {
        center_of_gravity += boid.get_position();
    }
    center_of_gravity /= static_cast<float>(boids.size());
    return center_of_gravity;

}

double Metrics::average_confidence_weight(const std::vector<Boid>& boids)
{
    double avg_conf = 0.0;
    for (auto &b : boids) avg_conf += b.getNeihborConfidenceWeightSum();
    // std::cout << " n neighbor = "<< boids[0].get_n_neighbors() <<" " <<  boids.size() << std::endl;
    // std::cout << "  neighbor conf w sum = "<< boids[0].getNeihborConfidenceWeightSum()  << std::endl;
    avg_conf /= boids.size();
    return avg_conf ;
}

// raw, clamped [0,1] cohesion metric
float Metrics::meas_cohesion(const std::vector<Boid>& boids,  const SimulationContext simcxt)
// float Metrics::meas_cohesion(const std::vector<Boid>& boids,  float sim_time)
// float Metrics::meas_cohesion(const std::vector<Boid>& boids)
{
    if (boids.empty()) {
        return 0.0f;
    }
    // float safe_dist = 1.5f;
    float tmp_separation_min_dist = MovingObject::getSeparationMinDist();
    int cnt_unsafe = 0;
    // boid positions into Eigen::MatrixXf
    Eigen::MatrixXf allboidpos(3, boids.size());
    Eigen::MatrixXf allboidpos_from_cog(3, boids.size());
    // for (size_t i = 0; i < boids.size(); ++i) {
    //     positions.col(i) << boids[i].get_position().x, boids[i].get_position().y, boids[i].get_position().z;
    // }


    //  center of gravity
    Vec3f center_of_gravity = swarm_center_of_gravity(boids);

    /*
    Vec3f center_of_gravity = Vec3f(0,0,0);
    // range based loop :-)
    for (const auto& boid : boids) {
        center_of_gravity += boid.get_position();
    }
    // for (size_t i = 0; i < boids.size(); ++i) {
    //     const Vec3f boid_pos = boids[i].get_position();
    //     center_of_gravity += boid_pos;
    //     allboidpos.col(i) << boid_pos.x(), boid_pos.y(), boid_pos.z();
    //     // allboidpos.col(i) << boid_pos.x, boid_pos.y, boid_pos.z;
    // }
    center_of_gravity /= static_cast<float>(boids.size());
    */
    // get boid position relative to COG
    // index based loop :-)
    for (size_t i = 0; i < boids.size(); ++i) {
        const Vec3f boid_pos = boids[i].get_position();
        // Vec3f relative_pos = boid_pos - center_of_gravity;
        // allboidpos_from_cog.col(i) << relative_pos.x(), relative_pos.y(), relative_pos.z();
        allboidpos.col(i) << boid_pos.x(), boid_pos.y(), boid_pos.z();
        for (size_t j = i+1; j < boids.size(); ++j) {
            float inter_dist = (boids[j].get_position() - boid_pos).norm();
            if (inter_dist < tmp_separation_min_dist)
                cnt_unsafe++;
                // std::cout << "warning \n";
        }
    }
    // standard deviation of distances from the center
    float pos_stddevs = calcStdDev(allboidpos).norm();
    // Eigen::Vector3f pos_stddevs = calcStdDev(allboidpos);
    // Eigen::Vector3f pos_from_cog_stddevs = calcStdDev(allboidpos_from_cog);
    // float pos_from_cog_stddevs = calcStdDev(allboidpos_from_cog).norm();
    float nb_comb_boid_pairs =   (boids.size()*(boids.size()-1)/2.0);
    float ratio_unsafe_dist =  cnt_unsafe / nb_comb_boid_pairs;
    // std::cout << "ratio of boid pairs at unsafe distance : " << 
    //     ratio_unsafe_dist << "nb/tot=" << cnt_unsafe << "/" << nb_comb_boid_pairs 
    //     << std::endl;
    // std::cout << "std dev from COG : " << pos_from_cog_stddevs << std::endl;
    // std::cout << "std dev : " << pos_stddevs << std::endl;
    // count unsafe inter-boid distances (penalty)

    // Combine scores and normalize
    float c1=1.0,c2=1.0;
    // we want a metric proportional to cohesion (good cohesion -> large cohesion metric)
    float normalized_score =  c1/(pos_stddevs+0.001) - c2 * ratio_unsafe_dist;
    // float normalized_score =  c1*pos_stddevs - c2 * ratio_unsafe_dist;
    // std::cout << "COG : " << center_of_gravity << std::endl;
    return normalized_score;
}

// sigmoid , smoothed  cohesion metric
// float Metrics::meas_cohesion_v2(const std::vector<Boid>& boids,  float sim_time)
// float Metrics::meas_cohesion_v2(const std::vector<Boid>& boids)
float Metrics::meas_cohesion_v2(const std::vector<Boid>& boids,  const SimulationContext simcxt, std::ofstream& out_file)
{
    if (boids.empty()) return 0.0f;

    const float d_safe = MovingObject::getSeparationMinDist(); // TODO assume 4x the size of a boid
    const float d_collide = d_safe / 4.0;
    const float R_ref  = 20.0f;   // expected formation radius TODO:get it from take-off config 
                                  // could be = std dev of all dist to COG at take off
                                  // could also be a max acceptable encircling circle radius
    const float d_ref  = 3.0f;    // expected max average spacing TODO:get it from take-off config

    const float w1 = 0.4f, w2 = 0.4f, w3 = 0.1f, w4 = 0.1f;

    // --- gather positions ---
    Eigen::MatrixXf allpos(3, boids.size()); // x,y,z for all the drones (3 x nb_boid matx)
    for (size_t i = 0; i < boids.size(); ++i)
        allpos.col(i) << boids[i].get_position().x(), boids[i].get_position().y(), boids[i].get_position().z();

    // --- center of gravity ---
    Vec3f cog = swarm_center_of_gravity(boids);
    Eigen::Vector3f cog_eig(cog.x(), cog.y(), cog.z());

    // --- distances to COG ---
    std::vector<float> r;
    r.reserve(boids.size());
    for (size_t i = 0; i < boids.size(); ++i)
        r.push_back((allpos.col(i) - cog_eig).norm());
    // stddev of drone distances to COG
    float sigma_r = calcStdDev(r);

    // --- pairwise distances ---
    std::vector<float> dists;
    dists.reserve(boids.size() * (boids.size()-1) / 2);
    float unsafe_penalty_sum = 0.0f;
    float collided_penalty_sum = 0.0f;
    // see python script collision_penalty for differences
#ifdef COLLISION_PENALTY_DEF1    
    for (size_t i = 0; i < boids.size(); ++i) {
        for (size_t j = i+1; j < boids.size(); ++j) {
            float d = (allpos.col(i) - allpos.col(j)).norm(); // distances btwn drones
            dists.push_back(d);
            if (d <= d_safe & d > d_collide )
                unsafe_penalty_sum += std::exp(-0.5f * (d*d) / (d_safe*d_safe));
            else if (d <= d_collide )
                collided_penalty_sum += std::exp(-0.5f * (d*d) / (d_safe*d_safe));
        }
    }
#else
    for (size_t i = 0; i < boids.size(); ++i) {
        for (size_t j = i + 1; j < boids.size(); ++j) {
            float d = (allpos.col(i) - allpos.col(j)).norm();
            dists.push_back(d);

            if (d < d_safe) {
                // Distance-based weighting between unsafe and collided
                float alpha = std::clamp((d - d_collide) / (d_safe - d_collide), 0.0f, 1.0f);

                // α = 0 → full collision, α = 1 → just inside unsafe
                collided_penalty_sum += (1.0f - alpha) * std::exp(-0.5f * (d*d) / (d_collide*d_collide) );
                // collided_penalty_sum += (1.0f - alpha) * std::exp(-0.5f * (d_collide*d_collide) / (d_safe*d_safe));
                unsafe_penalty_sum   += alpha * std::exp(-0.5f * (d*d) / (d_safe*d_safe));
            }
        }
    }
#endif
    // stddev of all pairwise distances
    float sigma_d = calcStdDev(dists);
    float P_collided =  collided_penalty_sum / (boids.size() * (boids.size()-1)); // some drones collided
    float P_unsafe =  unsafe_penalty_sum / (boids.size() * (boids.size()-1)); // no collision but unsafe penalty

    // --- normalized cohesion ---
    float C_R_ref = w1 * (1.0f - sigma_r / (R_ref + 1e-6f));
    float C_d_sep = w2 * (1.0f - sigma_d / (d_ref + 1e-6f));
    float C_P_unsafe = - w3 *   P_unsafe;
    float C_P_collided = - w4  *  P_collided;
            // - w4 * P_collided;
    float C = C_R_ref
            + C_d_sep
            + C_P_unsafe
            + C_P_collided;
           
            // + w2 * (1.0f - sigma_d / (d_ref + 1e-6f));
            // - w3 * 0.1 *  P_unsafe
            // - w4 * P_collided;

    // float rawC = w1 * (1.0f - sigma_r / (R_ref + 1e-6f))
    //        + w2 * (1.0f - sigma_d / (d_ref + 1e-6f))
    //        - w3 * P_coll;
    // for sigmoid metric
    // float rawC = w1 * (1.0f - sigma_r / (R_ref + 1e-6f))
    //        + w2 * (1.0f - sigma_d / (d_ref + 1e-6f))
    //        - w3 * std::tanh(2.0f * P_coll); // tanh limits huge penalties

    float rawC = w1 * (1.0f - sigma_r / (R_ref + 1e-6f))
           + w2 * (1.0f - sigma_d / (d_ref + 1e-6f));
           // - w3 * std::tanh( P_unsafe) // tanh limits huge penalties
           // - w4 * 10*std::tanh(1.0f * P_collided); // tanh limits huge penalties

    /* Tunable parameters */
    const float s_scale = 3.0f;    // pre-scale multiplier (try 3..8)
    const float k_sigmoid = 3.0f;  // steepness (try 3..10)
    const float c0 = 0.0f;         // center/shift of sigmoid (try 0.0 or mean value)
    float alpha = 0.03f; // exponential moving average smoothing 

    metric_smoothed_ = alpha * rawC + (1.0f - alpha) * metric_smoothed_;

    // squash to [0,1] smoothly
    float k = 3.0f;
    float x = s_scale * metric_smoothed_ - c0;
    float C_sigmoid = 1.0f / (1.0f + expf(-k_sigmoid * x));
    // float C_sigmoid = 1.0f / (1.0f + std::expf(-k_sigmoid * x));

    // float C_sigmoid = 1.0f / (1.0f + std::exp(-k * metric_smoothed_));

    // Define column widths and precision for better looking output
    const int label_width = 15;
    const int value_width = 8;
    const int precision = 5;

    // Set formatting for floating-point numbers
    std::cout << std::fixed << std::setprecision(precision);
    // return std::clamp(C_sigmoid, 0.0f, 1.0f);
        // Set formatting for floating-point numbers
        std::cout << std::fixed << std::setprecision(precision);
        out_file << std::fixed << std::setprecision(precision);
        std::cout << std::left 
            << std::setw(value_width) <<  simcxt.sim_time << "\t"           
            << std::setw(value_width) <<  sigma_r << "\t"
            << std::setw(value_width) <<  sigma_d << "\t"
            << std::setw(value_width) <<  P_unsafe << "\t"
            << std::setw(value_width) <<  P_collided << "\t"
            << std::setw(value_width) <<  C_R_ref << "\t"
            << std::setw(value_width) <<  C_d_sep << "\t"
            << std::setw(value_width) <<  C_P_unsafe << "\t"
            << std::setw(value_width) <<  C_P_collided << "\t"
            << std::setw(value_width) <<  rawC << "\t"
            << std::setw(value_width) <<  C_sigmoid << "\t"
            << std::setw(value_width) <<  std::clamp(rawC,0.0f,1.0f) << "\t"
            << std::setw(value_width) <<  std::clamp(C,0.0f,1.0f) 
            << std::endl;
        out_file << std::left
            << std::setw(value_width)<<  simcxt.sim_time << "\t"           
            << std::setw(value_width)<<  sigma_r << "\t"
            << std::setw(value_width)<<  sigma_d << "\t"
            << std::setw(value_width)<<  P_unsafe << "\t"
            << std::setw(value_width)<<  P_collided << "\t"
            << std::setw(value_width)<<  C_R_ref << "\t"
            << std::setw(value_width)<<  C_d_sep << "\t"
            << std::setw(value_width)<<  C_P_unsafe << "\t"
            << std::setw(value_width)<<  C_P_collided << "\t"
            << std::setw(value_width)<<  rawC << "\t"
            << std::setw(value_width)<<  C_sigmoid << "\t"
            << std::setw(value_width)<<  std::clamp(rawC,0.0f,1.0f) << "\t"
            << std::setw(value_width)<<  std::clamp(C,0.0f,1.0f) 
            << std::endl;

    // Clamp to [0,1]
    return std::clamp(C, 0.0f, 1.0f);
}


// note: the length is determine by:
// PERMANENT_PATH_SKIP_FRAMES
// and the max length of the COG history MAX_HISTORY_POINTS
void Metrics::draw_permanent_path() const {
    // Use the GL_LINE_STRIP method
    glDisable(GL_LIGHTING);      
    glDisable(GL_DEPTH_TEST);
    
    glColor3f(0.3f, 0.3f, 0.9f); // Blue
    glLineWidth(1.0f);           // Thin line
    
    glBegin(GL_LINE_STRIP);
    for (const auto& pos : permanent_path_) {
        glVertex3f(pos[0], pos[1], pos[2]);
    }
    glEnd();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING); 
}
// void Metrics::draw_cog_trace() const
// {
//     // Ensure we have something to draw
//     if (cog_history_.size() < 1) return;
//
//     // --- Material Setup for History Trace ---
//     // Define the color for the persistent trace (Yellowish-Red)
//     GLfloat trace_color[] = {0.8f, 0.4f, 0.0f, 1.0f}; 
//     
//     // Set the material properties for the trace spheres
//     glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, trace_color);
//     
//     // Draw the PAST trace points (All but the last one, to avoid overlap)
//     size_t history_size = cog_history_.size();
//     for (size_t i = 0; i < history_size - 1; ++i) {
//         const Vec3f& pos = cog_history_[i];
//         std::cout << "pos[" << i << "].x" << pos[0] << " ";
//         
//         glPushMatrix();
//         glTranslatef(pos[0], pos[1], pos[2]);
//         
//         // CRITICAL: Use a very small radius (0.1f to 0.2f)
//         glutSolidSphere(5.1f, 4, 4); 
//         
//         glPopMatrix();
//     }
//     
//     // --- Material Setup for Current COG (Override with Bright Red) ---
//     /*
//     const Vec3f& current_cog = cog_history_.back();
//
//     GLfloat current_color[] = {1.0f, 0.0f, 0.0f, 1.0f}; // Bright Red
//     glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, current_color);
//     
//     glPushMatrix();
//     glTranslatef(current_cog[0], current_cog[1], current_cog[2]);
//     glutSolidSphere(0.5f, 5, 5); 
//     glPopMatrix();
//     
//     // Optional: Reset material color to white (default) or a known value
//     GLfloat default_color[] = {1.0f, 1.0f, 1.0f, 1.0f};
//     glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, default_color);
//     */
// }

// temporary trace (like a comet)
// note: the length is determine by:
// RECORD_SKIP_FRAMES
// and the max length of the COG history MAX_HISTORY_POINTS
void Metrics::draw_cog_trace() const
{
    // check history size
    size_t history_size = cog_history_.size();
    // std::cout << " COG history size = " << history_size << std::endl;
    // Draw the COG trace as a series of connected points 
    /*
    glColor3f(0.8f, 0.8f, 0.0f); 

    // The logic is now entirely outside the glBegin/glEnd structure
    for (const auto& pos : cog_history_) {

        // Save the current transformation matrix
        glPushMatrix();

        // Move to the recorded COG position
        glTranslatef(pos[0], pos[1], pos[2]);

        // Draw a very small sphere (e.g., radius 0.2f)
        glutSolidSphere(1.2f, 4, 4); 

        // Restore the previous matrix
        glPopMatrix();
    }
    */
    if (history_size < 1) return;

    // Draw the PAST trace points (up to the second-to-last one)
    for (size_t i = 0; i < history_size - 1; ++i) {
        const Vec3f& pos = cog_history_[i];

        glPushMatrix();
        glTranslatef(pos[0], pos[1], pos[2]);

        // Use a small radius for the trace dot (e.g., 0.1f)
        glutSolidSphere(0.1f, 4, 4); 

        glPopMatrix();
    }
    // Draw the COG trace as a series of connected lines (doesn't work...)
    /*
    glColor3f(1.0f, 0.0f, 0.0f); // Red for the trace
    if (cog_history_.size() < 2) {
        std::cout << "less than 2 points , return\n";
        return; // Don't draw lines unless we have points
    }

    glLineWidth(23.5f);
    glBegin(GL_LINE_STRIP);

    for (const auto& pos : cog_history_) {
        std::cout << pos[0] << " " <<  pos[1]  << " " <<  pos[2] << std::endl;  // position is indeed printed out
        glVertex3f(pos[0], pos[1], pos[2]);
    }

    glEnd();
    glLineWidth(1.0f); // Reset line width
    */
    // Draw a sphere at the current COG position
    if (!cog_history_.empty()) {
        const Vec3f& current_cog = cog_history_.back();
        glPushMatrix();
        glTranslatef(current_cog[0], current_cog[1], current_cog[2]);
        glColor3f(1.0f, 0.0f, 0.0f); // Bright red sphere
        glutSolidSphere(0.5f, 5, 5); // Smaller sphere to mark the center
        glPopMatrix();
    }
}

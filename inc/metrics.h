/*********************************************************************************************************************
 * File : metrics.h                                                                                                     *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#ifndef METRICS_H
#define METRICS_H


#include "moving_object.h"
#include "boid.h" 
#include "simulation_context.h"

// #define COLLISION_PENALTY_DEF1    // commment to use DEF2
class Metrics
{
public:
    Metrics();
    virtual ~Metrics() = default;
    float meas_cohesion(const std::vector<Boid>& boids,const SimulationContext simcxt);
    float meas_cohesion_v2(const std::vector<Boid>& boids,const SimulationContext simcxt, std::ofstream& out_file);
    // float meas_cohesion(const std::vector<Boid>& boids,float sim_time);
    // float meas_cohesion_v2(const std::vector<Boid>& boids,float sim_time);
    double average_confidence_weight(const std::vector<Boid>& boids);
    Vec3f swarm_center_of_gravity(const std::vector<Boid>& boids);

    void record_swarm_cog(const Vec3f& cog) {
        // for the limited trail (sliding window cog history)
        if (++record_counter_ % RECORD_SKIP_FRAMES == 0) {
            cog_history_.push_back(cog);
            // Optional: Limit history length to prevent memory issues
            if (cog_history_.size() > MAX_HISTORY_POINTS) {
                cog_history_.erase(cog_history_.begin());
            }
        }
        // for the whole trail (whole cog history)
        if (++whole_path_record_counter_ % PERMANENT_PATH_SKIP_FRAMES == 0) {
            permanent_path_.push_back(cog);
        }
    }
    
    void draw_cog_trace() const;
    void draw_permanent_path() const;
    const std::vector<Vec3f>& get_cog_history() const { return cog_history_; }

private:
    // MetricsInitPos init_pos_type_;
    float last_t_;
    std::vector<Vec3f> cog_history_; // sliding window history
    std::vector<Vec3f> permanent_path_;      // full cog history
    const size_t MAX_HISTORY_POINTS = 500; // keep last 500 COG points
    int record_counter_ = 0; // 
    int whole_path_record_counter_ = 0; // 
    const int RECORD_SKIP_FRAMES = 10; // record every nth frame
                                      // will determine the length of the cog trail (the comet like trail)
    const int PERMANENT_PATH_SKIP_FRAMES = 90; // for the whole history (need to skip more
                                               // to prevent too many points recorded for long simus)
    float metric_smoothed_ = 0.5; // initial value will influence consequent behaviour of the metric!

    // int n_neighbors_;
    // Vec3f avg_position_;    // Cohesion
    // Vec3f avg_velocity_;       // Alignment
    // Vec3f proximity_force_; // Separation + Target attraction
};

#endif // METRICS_H

/*********************************************************************************************************************
 * File : boid.h                                                                                                     *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#ifndef BOID_H
#define BOID_H

#include "moving_object.h"
#include "messages.h"


// enum BoidInitPos{ RANDOM_CENTER, ALIGNED, GRID };
class Boid : public MovingObject
{
public:
    Boid(const Vec3f &position, const Vec3f &speed = Vec3f(0, 0, 0));
    virtual ~Boid() = default;

    void add_neighbor(const MovingObject &object);

    Vec3f get_exerted_proximity_force(const MovingObject &object) const override;

    void update(float t) override;
    void update_no_ang_speed_clamp(float t) override;

    void draw() const override;
    // void draw_boid() const override;

    // for messaging
    void receiveMessage(const BoidMessage &msg);
    std::vector<BoidMessage> getVisibleMessages(double current_time);
    const std::deque<BoidMessage>& peekInbox() const;
    void processMessages(double t);
    void processMessagesAndInterpolate(double t);
    static float getMessageLatency();
    static void setMessageLatency(float latency);
    bool getInterpolatedNeighborState(int neighbor_id, double query_time, NeighborState &out);
    void resetAccumulators() ;
    // for debug
    int get_n_neighbors() const { return n_neighbors_; }
    const Vec3f &get_avg_position() const { return avg_position_; }
    const Vec3f &get_proximity_force() const { return proximity_force_; }
    // for metric & debug
    float getNeihborConfidenceWeightSum() const {return neighbor_confidence_weight_sum_ ;} ;

private:
    // BoidInitPos init_pos_type_;
    void draw_wing() const;

    float last_t_;

    Vec3f color_;

    int n_neighbors_;
    Vec3f avg_position_;    // Cohesion
    Vec3f avg_speed_;       // Alignment
    Vec3f proximity_force_; // Separation 
    Vec3f external_object_force_; // for non-boid object forces (ex fence, target,...)
    std::deque<BoidMessage> inbox_;
    // History of neighbor states after message are released with a latency
    std::unordered_map<int, std::deque<NeighborState>> neighbor_history_;
    static float s_message_latency; // latency now a parameter
    double neighbor_confidence_weight_sum_ = 0.0; // TODO find better names
};

#endif // BOID_H

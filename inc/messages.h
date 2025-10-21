/*********************************************************************************************************************
 * File : message.h                                                                                                     *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#ifndef MESSAGES_H
#define MESSAGES_H

// #include <queue>

#include "moving_object.h"   
#include <deque>
#include <vector>

struct BoidMessage {
    int sender_id;
    Vec3f position;
    Vec3f velocity;
    double timestamp;
};

struct NeighborState {
    Vec3f position;
    Vec3f velocity;
    double timestamp;
    float state_confidence; // to degrade influence of neighbors when latency increases
};
// pretty-print for debugging, thanks AI!
inline std::ostream& operator<<(std::ostream& os, const BoidMessage& msg) {
    os << "[id=" << msg.sender_id
       << " pos=(" << msg.position[0] << "," << msg.position[1] << "," << msg.position[2] << ")"
       << " velocity=(" << msg.velocity[0] << "," << msg.velocity[1] << "," << msg.velocity[2] << ")"
       << " t=" << msg.timestamp << "]";
    return os;
}

#endif // MESSAGES_H

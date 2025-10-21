/*********************************************************************************************************************
 * File : simulation_context.h                                                                                          *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#pragma once

#include <fstream>

class GlobalOutput {
public:
    static std::ofstream& getStream();
private:
    static std::ofstream out_file;
};
// shared state for the whole simulation
struct SimulationContext {
    float sim_time      = 0.0f;   // current simulation time [s]
    float dt        = 0.0f;   // timestep duration [s]
    int   step      = 0;      // iteration count
    float latency   = 0.0f;   // network latency [s]
    // TODO (if all this grows!) add more: wind_speed, num_boids, etc. 
};

// class GlobalOutput {
// public:
//     static std::ofstream& getStream() {
//         static std::ofstream out_file;
//         return out_file;
//     }
// };

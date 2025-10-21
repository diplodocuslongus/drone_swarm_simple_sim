
/*********************************************************************************************************************
 * File : simulation_context.cpp                                                                                          *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/
#include "simulation_context.h"

// Define the static member
std::ofstream GlobalOutput::out_file;

std::ofstream& GlobalOutput::getStream() {
    return out_file;
}

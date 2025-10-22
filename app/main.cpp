/*********************************************************************************************************************
 * File : main.cpp                                                                                                   *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#define USE_MESSAGES // boids get each others' position from messages
#define USE_WAYPOINT // define and use waypoints for navigation
#define WP_NAV_PRESERVE_FORMATION // preserve swarm formation when navigating with waypoints
#define WP_BACK_HOME // waypoint behavior: back to first waypoint after last (loop)

// #define SHOW_PATH    // show swarm COG sliding window and whole trajectory
// #define SHOW_SLIDE_WIN_PATH    // show swarm COG sliding window trajectory ( comet like trail)
#define SCREENSHOT_AT_X_SECONDS // captures screenshot at X seconds (set x where this define resides)

#include <vector>
#define FREEGLUT_EXT
#define ENABLE_GUI

#include <GL/freeglut.h>
#include "camera_trackball.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glut.h"
#include "imgui/imgui_impl_opengl2.h"

#include "boid.h"
#include "target.h"
#include "obstacle.h"
#include "fence.h"
#include "ref_axis.h"
#include "metrics.h"
#include "waypoints.h"

#include <memory>
#include <iostream>
#include <cmath>
#include <ctime>

// Logging
// #define PLOG_DISABLE_LOGGING // comment to enable logging

#include <plog/Log.h>
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include <fstream>

// for opengl screenshot
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <filesystem>
#include "simulation_context.h"

namespace fs = std::filesystem; // to save screenshot to file

enum BoidInitPos{ RANDOM_CENTER, ALIGNED, GRID };
const float FOVY = 60.0f;
const float NEARCLIP = 0.1f;
const float FARCLIP = 150.0f;
const int FPS = 10;
const int WINDOW_X = 700;
const int WINDOW_Y = 100;


SimulationContext simcxt;
// Inputs
int mouse_x = 0;
int mouse_y = 0;
int mouse_buttons[GLUT_NUM_MOUSE_BUTTONS];
int window_w = 1000; // 800
int window_h = 700; // 600
GLuint window_id ;
// Camera
CameraTrackball camera;

std::vector<Boid> boids_;
std::vector<Target> targets_;
std::vector<WaypointManager> waypoints_;
WaypointManager *crt_waypoint_;
Target *crt_target_;
int crt_target_id_ = 0;

std::vector<Obstacle> obstacles_;
std::vector<Fence> fences_;
std::vector<RefAxis> ref_axis_;

Metrics g_metrics; // for cohesion metric

// boids and simulation parameters
int boids_number = 50; // 100; //1000; 
float neighborhood_max_dist = 10.0f;
float boid_to_boid_min_dist = 1.0f;
float separation_weight = 0.02f;
float alignment_weight = 0.005f;
float cohesion_weight = 0.07f;
float vision_distance = 5.0f; // TODO
float vision_FOV = 90.0f; // TODO
float min_cos_FOV = 0.5f; // 0.5 -> +/- 45° FOV
float max_speed = 5.0f;
float target_attraction_weight = 0.02f;
float target_velocity_alignment_weight = 0.03f;
float fence_repel_weight = 50.0f;
float fence_size = 15.0f;
float force_randomness = 0.01f;// randomness to add to forces (can simulate wind)
float msg_latency = 0.01f;
float simulation_duration_s = 5.0;
bool g_show_full_path = false; // show full swarm COG path
bool g_show_log_metric_header = false; // mainly to debug the cohesion metric and log its details
float g_ogl_zoom_lvl = 70.0f; // large: far, small value: near (zoommed in)
float g_ogl_trans_x = 30.0f; // opengl camera view translate coord system along x axis
bool g_stop_simulation = false;
std::string metricfilename;

// save weight as a way to stop the swarm (make it hover) 
// and at the same time preserving the cohesion
struct SavedWeights {
    float cohesion = 0.0f;
    float separation = 0.0f;
    float alignment = 0.0f;
    float waypoint_attraction = 0.0f;
    float waypoint_alignment = 0.0f;
};
// extern SavedWeights g_original_weights;
SavedWeights g_original_weights;
// generate filename for coehsion metric logging
// takes relevant simulation parameters
std::string genCohesionMetricFilename(float randforce, float dronedist) {
    // Get current date as YYYY-MM-DD
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss_date;
    oss_date << std::put_time(&tm, "%Y-%m-%d");
    std::string date_str = oss_date.str();

    // Format parameters into the filename
    std::ostringstream oss;
    oss << "output_randforce" << randforce
        << "_dronedist" << dronedist
        << "_" << date_str
        << ".tsv";
    return oss.str();
}
void DEBUG_draw_simple_trace() {
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glColor3f(1.0f, 1.0f, 1.0f); // Bright White
    glPointSize(5.0f);

    glBegin(GL_POINTS);
    // test path 
    std::vector<Vec3f> g_path_history = {
        {-5.0f, 0.0f, 0.0f},
        {-3.0f, 1.0f, 0.0f},
        {-1.0f, 2.0f, 0.0f},
        { 1.0f, 1.0f, 0.0f},
        { 3.0f, 0.0f, 0.0f},
        { 5.0f, -1.0f, 0.0f}
    };

    // Use the global g_metrics to access the history
    const auto& history = g_metrics.get_cog_history(); // 
    // std::cout << "dbg hist size: " <<  history.size();
    // Draw ALL points in the history
    for (size_t i = 0; i < history.size(); ++i) {
        const Vec3f& pos = history[i];
        if (i == 0 || i == 499){
        std::cout << "posx[" <<i << "]" <<  pos[0] << ",";
        std::cout << "posy[" <<i << "]" <<  pos[1] << ",";
        std::cout << "posz[" <<i << "]" <<  pos[2] << "\n";
        }
        glVertex3f(pos[0], pos[1], pos[2]);
    }
    for (const auto& pos : history) {
        glPushMatrix();
        glTranslatef(pos[0], pos[1], pos[2]);
        glutSolidSphere(0.3f, 10, 10); // Draw small spheres
        glPopMatrix();
    }
    // show the test path
    for (const auto& pos : g_path_history) {
        glPushMatrix();
        glTranslatef(pos[0], pos[1], pos[2]);
        glutSolidSphere(0.3f, 10, 10); // Draw small spheres
        glPopMatrix();
    }

    
    glEnd();
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glPointSize(1.0f);
}

// help
void printHelp() {
    std::cout << "Usage: ./bin/./main [options]" << std::endl;
    std::cout << "Boid simulation." << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help  Display help and exit." << std::endl;
    std::cout << "--nb_boids    Number of boids in the simulation" << std::endl;
    std::cout << "--maxdist     Max dist btwn objects for them to be considered as neighbors (boids to boids, boids to object)    " << std::endl;
    std::cout << "--separation  Separation weight                " << std::endl;
    std::cout << "--alignment   Alignment weight                 " << std::endl;
    std::cout << "--cohesion    Cohesion weight                  " << std::endl;
    // std::cout << "--visdist     Vision distance range            " << std::endl; //TODO: ex if using a depth sensor
    std::cout << "--visfov      cos Vision FOV                   " << std::endl;
    std::cout << "--maxspeed    Maximum boid speed               " << std::endl;
    std::cout << "--targetattract Attraction to target weight    " << std::endl;
    std::cout << "--targetalign Speed of alignment to target" << std::endl;
    std::cout << "--fencerepel  Fence repel weight" << std::endl;
    std::cout << "--fencesz     Fence size" << std::endl;
    std::cout << "--randf     Force Randomness" << std::endl;
    std::cout << "--msglat     Message Latency" << std::endl;
    std::cout << "--simt       Simulation duration in s" << std::endl;
    std::cout << "--viewzoom       Simulation zoom level" << std::endl;
}
// --- SET THE WEIGHTS AND PARAMETERS ---
void set_boid_params(int nbboids,float maxdist,float mindist,
        float maxspeed,float targetattract, float targetvelocityalign,
        float mincosFOV, float fencesize,float fencerepel, float msglatency, float forcerandomness) {
    MovingObject::setBoidNumber(nbboids);
    MovingObject::setNeighborMaxDist(maxdist);
    MovingObject::setSeparationMinDist(mindist);
    MovingObject::setMaxSpeed(maxspeed);
    MovingObject::setTargetAttractionWeight(targetattract);
    MovingObject::setTargetSpeedAlignmentWeight(targetvelocityalign);
    MovingObject::setMinCosAngle(mincosFOV);
    MovingObject::setFenceSize(fencesize);
    MovingObject::setFenceRepelWeight(fencerepel);
    Boid::setMessageLatency(msglatency);
    MovingObject::setForceRandomness(forcerandomness);
}
void set_boid_weights(float sep, float align, float coh) {
    MovingObject::setSeparationWeight(sep);
    MovingObject::setAlignmentWeight(align);
    MovingObject::setCohesionWeight(coh);
}
// parse command-line arguments.
int parse_command_line_args(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp();
            return 1; // Exit 
        }
        else if (arg == "--nb_boids") {
            if (i + 1 < argc) {
                try {
                    boids_number = std::stoi(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --nb_boids requires an integer argument. Using default." << std::endl;
                }
            }
        } 
        // neighborhood  max distance
        else if (arg == "--maxdist") {
            if (i + 1 < argc) {
                try {
                    neighborhood_max_dist = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --maxdist requires a float argument. Using default." << std::endl;
                }
            }
        } 
        // boid to boid minimum safe distance
        else if (arg == "--mindist") {
            if (i + 1 < argc) {
                try {
                    boid_to_boid_min_dist = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --mindist requires a float argument. Using default." << std::endl;
                }
            }
        } 
        else if (arg == "--separation") {
            if (i + 1 < argc) {
                try {
                    separation_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --separation requires a float argument. Using default." << std::endl;
                }
            }
        } 
        else if (arg == "--alignment") {
            if (i + 1 < argc) {
                try {
                    alignment_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --alignment requires a float argument. Using default." << std::endl;
                }
            }
        } else if (arg == "--cohesion") {
            if (i + 1 < argc) {
                try {
                    cohesion_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --cohesion requires a float argument. Using default." << std::endl;
                }
            }
        } 
        else if (arg == "--maxspeed") {
            if (i + 1 < argc) {
                try {
                    max_speed = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --maxspeed requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--targetattract") {
            if (i + 1 < argc) {
                try {
                    target_attraction_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --targetattract requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--fov") {
            if (i + 1 < argc) {
                try {
                    float fov_deg = std::stof(argv[++i]);
                    min_cos_FOV = cos(fov_deg * 3.14/180.0);
                    // min_cos_FOV = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --cosangle: cosine(FOV),requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--fencerepel") {
            if (i + 1 < argc) {
                try {
                    fence_repel_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --fencerepel requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--fencesz") {
            if (i + 1 < argc) {
                try {
                    fence_size = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --targetattract requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--targetalign") {
            if (i + 1 < argc) {
                try {
                    target_velocity_alignment_weight = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --targetvelocityalign requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--msglat") {
            if (i + 1 < argc) {
                try {
                    msg_latency = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --msglat requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--simt") {
            if (i + 1 < argc) {
                try {
                    simulation_duration_s = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --simt requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--randf") {
            if (i + 1 < argc) {
                try {
                    force_randomness = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: --randf (force randomness) requires a float argument. Using default." << std::endl;
                }
            }
        }
        else if (arg == "--viewzoom") {
            if (i + 1 < argc) {
                try {
                    g_ogl_zoom_lvl = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: ." << std::endl;
                }
            }
        }
        else if (arg == "--viewxtrans") {
            if (i + 1 < argc) {
                try {
                    g_ogl_trans_x = std::stof(argv[++i]);
                } catch (const std::exception& e) {
                    std::cerr << "Error: ." << std::endl;
                }
            }
        }
    }
    return 0; // 
}

// screenshot related functions

std::string generateTimestampedFilename() {
    // Get the current time
    std::time_t now = std::time(nullptr);
    // Convert to a tm structure for the local time
    std::tm* local_time = std::localtime(&now);
    
    // Create a buffer to hold the formatted time string
    char buffer[80];
    
    // Format the time into a string. The format specifiers create a
    // string like "2025-09-22_10-53-45.png"
    std::strftime(buffer, sizeof(buffer), "screenshot_%Y-%m-%d_%H-%M-%S.png", local_time);
    
    return std::string(buffer);
}

void saveToImageFile(const std::string& directory_path, const GLubyte* pixels, int width, int height) {
    // std::string filename = "screenshot.png";
    // filename with a timestamp
    std::string ts_filename = generateTimestampedFilename();
    // Combine relative directory path and filename
    fs::path full_path = fs::path(directory_path) / ts_filename;
    
    // Create the directory if it doesn't exist
    // fs::create_directories(full_path.parent_path());
    
    // Save the file
    stbi_write_png(full_path.string().c_str(), width, height, 3, pixels, width * 3);
    

    // stbi_write_png(filename.c_str(), width, height, 3, pixels, width * 3);

    // std::cout << "Screenshot saved to " << filename << std::endl;
    std::cout << "Screenshot saved to " << full_path.string() << std::endl;
}

// Function to flip the image vertically
void flipPixelsVertically(GLubyte* pixels, int width, int height, int channels) {
    int row_size = width * channels;
    std::vector<GLubyte> temp_row(row_size);
    for (int i = 0; i < height / 2; ++i) {
        GLubyte* row1 = pixels + i * row_size;
        GLubyte* row2 = pixels + (height - 1 - i) * row_size;
        std::copy(row1, row1 + row_size, temp_row.begin());
        std::copy(row2, row2 + row_size, row1);
        std::copy(temp_row.begin(), temp_row.end(), row2);
    }
}

void captureScreenshot(){ // GLFWwindow* window) {
    // int width, height;
    // glfwGetFramebufferSize(window, &width, &height);
    int width = glutGet(GLUT_WINDOW_WIDTH);
    int height = glutGet(GLUT_WINDOW_HEIGHT);
    // Create a pixel buffer for the screen data
    GLubyte* pixels = new GLubyte[3 * width * height];

    // Set the packing alignment to 1
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    // Read the pixels from the front buffer (what's currently on screen)
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

    // The pixel data needs to be flipped vertically because
    // OpenGL's origin (0,0) is bottom-left, while most image formats
    // and viewers assume a top-left origin.
    flipPixelsVertically(pixels, width, height, 3);

    std::string my_screenshot_dir = "../screenshot/";
    saveToImageFile(my_screenshot_dir, pixels, width, height);
    // saveToImageFile(pixels, width, height);

    delete[] pixels;
}
// Boids are characterized by their position and speed
// set position and speed initial values
void add_boid(int b_i,int n_new_boids)
// void add_boid(int b_i) //,int b_nb)
// void add_boid()
{
    int scale = 1;
    float grid_stp = MovingObject::getSeparationMinDist() + 0.2f; // TODO check it is > boid size
    BoidInitPos init_pos_type_;
    init_pos_type_ = GRID; //ALIGNED; // RANDOM_CENTER;
    float x,y,z,u,v,w;
    if (init_pos_type_ == RANDOM_CENTER){

        x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        // speed
        u = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        v = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        w = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    }
    else if (init_pos_type_ == ALIGNED){
        x = static_cast<float>(0);
        y = static_cast<float>((b_i - n_new_boids/2)* grid_stp);
        z = static_cast<float>(0);
        // speed
        u = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        v = static_cast<float>(0);
        w = static_cast<float>(0.1);
    // std::cout << x << "," << y<< "," <<z << "  " ;
    }
    else if (init_pos_type_ == GRID){
        // nb of grid cells
        // int grid_w = static_cast<int>(ceil(sqrt(n_new_boids)));
        double grid_w = std::ceil(std::sqrt(n_new_boids));
        x = static_cast<float>((fmod(b_i,grid_w) - grid_w/2) * grid_stp);
        y = static_cast<float>(static_cast<int>(b_i) / static_cast<int>(grid_w) - grid_w/2)*grid_stp;
        z = static_cast<float>(0);
        // speed
        // add random forward speed, can also initial speed in other directions
        u = static_cast<float>(0.1+rand()) / static_cast<float>(RAND_MAX);
        // v = static_cast<float>(0.01+rand()) / static_cast<float>(RAND_MAX);
        v = static_cast<float>(0.01);
        w = static_cast<float>(0.0);
    }
    // std::cout << x << ", " << y<< ", " <<z << "\n" ;
    boids_.emplace_back(scale * Vec3f(x, y, z), 0.1 * Vec3f(u, v, w));
}

void init(void)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    // Init inputs
    for (int i(0); i < GLUT_NUM_MOUSE_BUTTONS; ++i)
        mouse_buttons[i] = GLUT_UP;

    // Init camera
    // can set here the level of zoom and view point
    // camera.init({20.0f, 0.0f, 0.0f}, 70.0f);
    camera.init({ g_ogl_trans_x, 0.0f, 0.0f}, g_ogl_zoom_lvl);

    // for (int j = 0; j < boids_number_; j++)
    for (int j = 0; j < boids_number; j++)
        add_boid(j,boids_number);

#ifdef USE_MESSAGES
    // initialization of boid messaging
    const float nowtime = (float)glutGet(GLUT_ELAPSED_TIME) * 0.001;
    for (int i = 0; i < boids_.size(); i++) {
        BoidMessage msg{ i, boids_[i].get_position(), boids_[i].get_velocity(), nowtime };
        for (int j = 0; j < boids_.size(); j++) {
            if (i != j) {
                boids_[j].receiveMessage(msg);
            }
        }
    }
#endif

    const int scale = 1;
#ifdef USE_WAYPOINT
    // define waypoints locations
    // define the whole path as a vector 
    std::vector<Vec3f> triangle_path = {
        Vec3f(20, 0, 0),      // Waypoint 1
        Vec3f(30, 10, 0),      // Waypoint 2
        Vec3f(40, 20, 0),      // Waypoint 3 
    };
    std::vector<Vec3f> four_point_path = {
        Vec3f(10, 0, 0),      // Waypoint 1
        Vec3f(10, 10, 0),      // Waypoint 2
        Vec3f(20, 0, 0),      // Waypoint 3 
        Vec3f(30, 10, 0)      // Waypoint 4 (The final hover point)
    };

    // WaypointManager
    waypoints_.emplace_back(four_point_path);
    // waypoints_.emplace_back(triangle_path);
    // waypoints_.emplace_back( Vec3f(10, 0, 0));
    // comment if want enable waypoint from keyboard
    crt_waypoint_ = &waypoints_.back();
#endif
    // define targets locations
    // different targets appears when pressing space
    // for (double i : {-2 * scale, 2 * scale})
    //     targets_.emplace_back(scale * Vec3f(i, 0, 0));
    //
    // for (double k : {-9, 9})
    //     targets_.emplace_back(scale * Vec3f(0, 0, k));
     
    // TODO (maybe): pass target locations from the command line
    // 1. Define the accelerating function (the movement strategy)
    //    dt is delta time, vel is the current velocity vector.
    auto gravity_velocity_update = [](float dt, Vec3f &vel) {
        // Gravitational acceleration (e.g., 9.8 m/s^2) applied over delta time.
        // Assuming Vec3f members are x, y, z.
        vel.y() -= 9.8f * dt;
    };

    // 2. Construct the Target by passing the function as the second argument.
    //    (Initial position: 20, 0, 2)
    targets_.emplace_back(
        Vec3f(20, 0, 2),        // The 'position' argument (first parameter)
        gravity_velocity_update    // The 'update_velocity_func' argument (second parameter)
    );
    // targets_.emplace_back( Vec3f(20, 0, 2));

    // define obstacle
    // for (double j : {-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7})
    //     for (double k : {-7, -6, -5, -4, -3, -2, 2, 3, 4, 5, 6, 7})
    //         obstacles_.emplace_back(scale * Vec3f(0, j, k), scale);
    //
    // for (double j : {-7, -6, -5, -4, -3, 0, 3, 4, 5, 6, 7})
    //     for (double k : {-1, 0, 1})
    //         obstacles_.emplace_back(scale * Vec3f(0, j, k), scale);

    // define a  fence
    // fences_.emplace_back(scale * Vec3f(0, 0, 0), fence_size); // spheroid
    // float cubdim = 3.0;
    fences_.emplace_back(scale * Vec3f(-fence_size, -fence_size, -0),scale * Vec3f(fence_size, fence_size, 2*fence_size)); // cuboid above the ground
    // fences_.emplace_back(scale * Vec3f(-fence_size, -fence_size, -fence_size),scale * Vec3f(fence_size, fence_size, fence_size)); // cuboid

    // define ref axis
    ref_axis_.emplace_back( Vec3f(0, 0, 0), Vec3f(0, 0, 0)); // 
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#ifdef ENABLE_GUI
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
    ImGui::Begin("Parameters");

    static float wfstep = 0.01f; // step change of weights
    // if (ImGui::SliderFloat("Foobar", &foobar, 0.0f, 3.0f, "%.0f")) {
    //     foobar = std::round(foobar / fstep) * fstep;
    // }
    int temp_boid_nb = MovingObject::getBoidNumber();
    // ImGui::SliderInt("Number of boids", &boids_number_, 0, 2000);
    ImGui::SliderInt("Number of boids", &boids_number, 0, 2000);
    MovingObject::setBoidNumber(temp_boid_nb);
    float temp_neighbor_max_dist = MovingObject::getNeighborMaxDist();
    if (ImGui::SliderFloat("Neigbr Max Dist", &temp_neighbor_max_dist, 0.0f, 12.f, "%.0f")) {
        temp_neighbor_max_dist = std::round(temp_neighbor_max_dist / 0.5) * 0.5;
    }
    MovingObject::setNeighborMaxDist(temp_neighbor_max_dist);
    float temp_separation_weight = MovingObject::getSeparationWeight();
    if (ImGui::SliderFloat("Separation", &temp_separation_weight, 0.0f, 0.1f, "%.2f")){
        temp_separation_weight = std::round(temp_separation_weight / wfstep) * wfstep;
    }
    MovingObject::setSeparationWeight(temp_separation_weight);
    float temp_cohesion_weight = MovingObject::getCohesionWeight();
    ImGui::SliderFloat("Cohesion", &temp_cohesion_weight, 0.0f, 0.3f);
    MovingObject::setCohesionWeight(temp_cohesion_weight);
    float temp_align_weight = MovingObject::getAlignmentWeight();
    ImGui::SliderFloat("Alignment", &temp_align_weight, 0.0f, 0.2f);
    // ImGui::SliderFloat("Alignment", &MovingObject::alignment_factor_, 0.0f, 0.02f);
    MovingObject::setAlignmentWeight(temp_align_weight);
    float temp_targt_attraction_weight = MovingObject::getTargetAttractionWeight();
    ImGui::SliderFloat("Target attraction", &temp_targt_attraction_weight, 0.0f, 1.f);
    MovingObject::setTargetAttractionWeight(temp_targt_attraction_weight);
    float temp_targt_velocity_align_weight = MovingObject::getTargetSpeedAlignmentWeight();
    ImGui::SliderFloat("Target speed attraction", &temp_targt_velocity_align_weight, 0.0f, 0.05f);
    MovingObject::setTargetSpeedAlignmentWeight(temp_targt_velocity_align_weight);
    ImGui::SliderFloat("Obstacle", &Obstacle::obstacle_factor_, 0.f, 200.f);
    float temp_force_randomness = MovingObject::getForceRandomness();
    ImGui::SliderFloat("Randomness", &temp_force_randomness, 0.0f, 2.f);
    MovingObject::setForceRandomness(temp_force_randomness);
    float temp_max_speed = MovingObject::getMaxSpeed();
    ImGui::SliderFloat("Max speed", &temp_max_speed, 0.0f, 20.f);
    MovingObject::setMaxSpeed(temp_max_speed);
    float temp_min_cos_ang = MovingObject::getMinCosAngle();
    if (ImGui::SliderFloat("Min cos angle", &temp_min_cos_ang, -1.f, 1.f,"%.1f")){

        temp_min_cos_ang = std::round(temp_min_cos_ang / wfstep) * wfstep;
    }
    MovingObject::setMinCosAngle(temp_min_cos_ang);

    ImGui::End();
#endif    

    //Camera setup
    camera.lookAt(); // default is trackball
    // camera.lookAt(CameraTrackball::TOP_DOWN);

    for (const auto &boid : boids_)
        boid.draw();

    if (crt_target_)
        crt_target_->draw();
#ifdef USE_WAYPOINT
    if (crt_waypoint_)
        crt_waypoint_->draw_path();
#endif
    g_metrics.draw_cog_trace();
    if (g_show_full_path) {
        g_metrics.draw_permanent_path(); // Draws the full, sparse path
    }
    // DEBUG_draw_simple_trace(); // debug tracer

    for (const auto &obstacle : obstacles_)
        obstacle.draw();

    for (const auto &fence : fences_)
        fence.draw();

    for (const auto &ref_axis : ref_axis_)
        ref_axis.draw();

    // this is to get the actual line width range 
    // GLfloat range[2];
    // glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, range);
    // printf("Supported line width range: %f - %f\n", range[0], range[1]);


    ////
float axisVertices[] = {
    0.0f, 0.0f, 0.0f,  // X-axis origin
    1.0f, 0.0f, 0.0f,  // X-axis end
    0.0f, 0.0f, 0.0f,  // Y-axis origin
    0.0f, 1.0f, 0.0f,  // Y-axis end
    0.0f, 0.0f, 0.0f,  // Z-axis origin
    0.0f, 0.0f, 1.0f   // Z-axis end
};

float axisColors[] = {
    1.0f, 0.0f, 0.0f,  // Red for X
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,  // Green for Y
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,  // Blue for Z
    0.0f, 0.0f, 1.0f
};
//     glBegin(GL_LINES);
//   glVertex3f(10.5f, -0.5f, 0.0f);
//   glVertex3f(-10.5f, -0.5f, -0.5f);
// glEnd();
    ///
    ///


#ifdef ENABLE_GUI
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
#endif
    glutSwapBuffers();
}

void reshape(int w, int h)
{
#ifdef ENABLE_GUI
    ImGui_ImplGLUT_ReshapeFunc(w, h);
#endif

    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(FOVY, (GLfloat)w / (GLfloat)h, NEARCLIP, FARCLIP);
}

void processKeys(unsigned char key, int x, int y)
{
    ImGui_ImplGLUT_KeyboardFunc(key, x, y);
    if (key == 32) // Space, add a target
    {
        crt_target_id_ = (crt_target_id_ + 1) % targets_.size();
        crt_target_ = &targets_[crt_target_id_];
    }
    else if (key == 'w')
    {
        crt_waypoint_ = &waypoints_.back();
    }
    else if (key == 'n')
    {
        boids_number+=10;
        // boids_number_++;
        // add_boid(-1);
        // add_boid(0,boids_number);
    }
    else if (key == 'h') // make the drone swarm to hover at its location
    {
        g_stop_simulation = !g_stop_simulation;
    }
    else if (key == 's')
    {
        captureScreenshot();
    }
    else if (key == 'b') // bird view
    {
        // camera.lookAt(CameraTrackball::TOP_DOWN);
        camera.setCamViewMode(CameraTrackball::TOP_DOWN);
        // 2. Request a redraw to apply the new view
        glutPostRedisplay(); 
    }
    else if (key == 'M') // latency becomes very large, similar to disabling all messages
    {
        Boid::setMessageLatency(6000);
        // std::cout << "mesg latency set to " << Boid::s_message_latency << std::endl;

    }
    else if (key ==  27) // Escape to close
    {
        glutDestroyWindow(window_id); // 
        exit(0);
    }
}


void systemEvolution()
{

    if (boids_number != boids_.size())
    {
        if (boids_number < boids_.size())
            boids_.erase(boids_.begin() + boids_number, boids_.end());
        else
        {
            const int n_new_boids = boids_number - boids_.size();
            for (size_t i = 0; i < n_new_boids; i++)
                add_boid(i,n_new_boids);
                // add_boid(i);
        }
    }
    // const float global_time = (float)glutGet(GLUT_ELAPSED_TIME) * 0.001;
    const float nowtime = (float)glutGet(GLUT_ELAPSED_TIME) * 0.001;
    static float  frozen_sim_time = 0.0f; //nowtime;
    static float delta_time_since_frozen = 0.0f;
    static bool is_frozen = false;
    if (g_stop_simulation && !is_frozen) {
        // Freeze sequence (executed ONCE when hover is ordered)
        is_frozen = true;
        frozen_sim_time = nowtime; //record current time to offset the subsequent simtime if resume fly
        // std::cout << "is frozen ="<<is_frozen << std::endl;
        
        // save original weights
        g_original_weights.cohesion = MovingObject::getCohesionWeight();
        g_original_weights.separation = MovingObject::getSeparationWeight();
        g_original_weights.alignment = MovingObject::getAlignmentWeight();
        // ... (TODO later Save waypoint weights ) ...

        // kill all force weights (prevents collapse)
        // MovingObject::setCohesionWeight(0.0f);
        // MovingObject::setSeparationWeight(0.0f);
        MovingObject::setAlignmentWeight(0.0f);
        // ... (Set all other active weights to 0.0f) ...

        // kill momentum or ... decelerate
        for (auto &boid : boids_) {
            boid.set_velocity(Vec3f::Zero()); 
        }
        // std::cout << "all boid speed to zero =" << std::endl;
    } 
    else if (!g_stop_simulation && is_frozen) { // to move again
        // unfreeze sequence (execute ONCE upon resume motion) presss h again
        // TODO: need to reset the force and all to a move forward direction, see the if(is_frozen) section
        is_frozen = false;
        std::cout << "is frozen 2 ="<<is_frozen << std::endl;
        
        // Restore Original Weights
        MovingObject::setCohesionWeight(g_original_weights.cohesion);
        MovingObject::setSeparationWeight(g_original_weights.separation);
        MovingObject::setAlignmentWeight(g_original_weights.alignment);
        for (auto &boid : boids_) {
            float u,v,w;
            // same as for the taking off config
            u = static_cast<float>(0.1+rand()) / static_cast<float>(RAND_MAX);
            v = static_cast<float>(0.01);
            w = static_cast<float>(0.0);
                
            Vec3f restart_velocity(u,v,w);
            boid.set_velocity(restart_velocity); 
            }
        // frozen_sim_time 
        // ... (Restore all other active weights) ...
    }
//  if (g_stop_simulation && is_frozen) {
 if (g_stop_simulation) {
    // std::cout << "is frozen 3 ="<<is_frozen << std::endl;
    // if (0){ // g_stop_simulation) {
    // else  if (g_stop_simulation) {
    // MovingObject::setAlignmentWeight(0.0f);
    // MovingObject::setAlignmentWeight(0.0f);
    // MovingObject::setAlignmentWeight(g_original_weights.alignment);
    // MovingObject::setCohesionWeight(g_original_weights.cohesion);
    // MovingObject::setSeparationWeight(g_original_weights.separation);
}
    if (is_frozen){
        // MovingObject::setCohesionWeight(g_original_weights.cohesion / 10);
        // MovingObject::setSeparationWeight(g_original_weights.separation / 1);
        // MovingObject::setAlignmentWeight(g_original_weights.alignment / 10);
        delta_time_since_frozen = nowtime - frozen_sim_time;
        for (auto &boid : boids_) {
            float r1 = ((float)rand() / RAND_MAX - 0.5f) * 0.0000f; 
            float r2 = ((float)rand() / RAND_MAX - 0.5f) * 0.0000f;
            float r3 = ((float)rand() / RAND_MAX - 0.5f) * 0.0000f;
            
            Vec3f random_nudge(r1, r2, r3);
            boid.set_velocity(random_nudge); 
        }
        // return;
    }
    // For each boid: reset, accumulate (messages OR direct neighbors), add static objects, then update
    for (auto &boid : boids_) {
        boid.resetAccumulators();

#ifdef USE_MESSAGES
        // note: now use a delta time to account for when the swarm hovers (otherway: use another global sim time)
        boid.processMessagesAndInterpolate(nowtime - delta_time_since_frozen); // populates avg_position_, avg_velocity_, proximity_force_, n_neighbors_
        // boid.processMessagesAndInterpolate(nowtime); // populates avg_position_, avg_velocity_, proximity_force_, n_neighbors_
        if (crt_target_) {
            boid.add_neighbor(*crt_target_);
            // if (boid.get_id() == 0)
            //     std::cout << "after add target: prox=" << boid.get_proximity_force().norm() << std::endl;
        }
        // always record swarm COG
        // get swarm COG from the swarm metrics
        Vec3f swarm_cog = g_metrics.swarm_center_of_gravity(boids_);
         // record the COG for the trace
        g_metrics.record_swarm_cog(swarm_cog); 
        if (crt_waypoint_) {
            // get swarm COG from the swarm metrics
            // Vec3f swarm_cog = g_metrics.swarm_center_of_gravity(boids_);
            // record the COG for the trace
            // g_metrics.record_swarm_cog(swarm_cog); 
      
            // update the path progression based on where the swarm (its COG) is w.r.t. the waypoint
            crt_waypoint_-> update_path_progression(swarm_cog); 
            // virtual waypoint position depends on the current boid's position w.r.t the swarm's COG
            Vec3f virtual_waypoint;

            // get the actual Target waypoint the boid should follow
            // (this target waypoint is a MovingObject)
            // each boid will have its own active target

            Target& active_target = crt_waypoint_->get_active_target();
            const Vec3f global_waypoint_pos = active_target.get_position();

            // current boid's  offset from COG (di = xi - R)
            Vec3f boid_position = boid.get_position();
            Vec3f current_offset = boid_position - swarm_cog;

            // boid's virtual target position (Ti = W + di)
            Vec3f virtual_waypoint_pos = global_waypoint_pos + current_offset;

            // temporary Target object at Ti.
            Target virtual_target(virtual_waypoint_pos); 

#ifdef WP_NAV_PRESERVE_FORMATION
            // when using virtual waypoints
            // formation is preserved
            boid.add_neighbor(virtual_target);
#else        
            // when using only one wapoint for everyone
            // waypoints will act like targets and break the formation
            boid.add_neighbor(active_target);
#endif
            if (crt_waypoint_->has_swarm_stopped()) {
                // stop every boid immediately only once
                // std::cout << "last WP, set speed to zero\n";
                for (auto &boid : boids_) {
                    if (boid.get_velocity().norm() > 0.01f) {
                    // boid.set_velocity(Vec3f::Zero()); 
                    }
                }
                // crt_waypoint_ -> reset_swarm_stopped_flag(); // set has stopped flag, will not change the speed again here
                // crt_waypoint_ -> set_swarm_hover_flag(); // hover flag, used in waypoint manager
            }
        }
#else
            // old immediate neighbor accumulation
            for (auto &other : boids_) {
                if (boid.get_id() != other.get_id()) {
                    if (MovingObject::are_neighbors(boid, other))
                        boid.add_neighbor(other);
                }
            }
#endif

            // always add non-boid objects (these will augment the accumulators)
            if (crt_target_) boid.add_neighbor(*crt_target_);
            for (const auto &obstacle : obstacles_) boid.add_neighbor(obstacle);
            for (const auto &fence : fences_) boid.add_neighbor(fence);
            for (const auto &ref_axis : ref_axis_) boid.add_neighbor(ref_axis);

            // (optional debug print to confirm accumulators contain contributions)
            // std::cout << "Boid " << boid.get_id() << " before update: n=" << boid.get_n_neighbors()
            //           << " prox=" << boid.get_proximity_force().norm()
            //           << " avgpos=" << boid.get_avg_position() << std::endl;
            boid.update(nowtime - delta_time_since_frozen);
            // boid.update(nowtime);
        }
    
#ifdef USE_MESSAGES
    // fill in each individual boid messages (its position, speed and corresponding time)
    for (int i = 0; i < boids_.size(); i++) {
        BoidMessage msg{ i, boids_[i].get_position(), boids_[i].get_velocity(), nowtime - delta_time_since_frozen};
        // BoidMessage msg{ i, boids_[i].get_position(), boids_[i].get_velocity(), nowtime };
        for (int j = 0; j < boids_.size(); j++) {
            if (i != j) {
                boids_[j].receiveMessage(msg);
            }
        }
    }
#endif
    // --------------
    // formation  metrics
    // --------------
    static float last_metric_time = 0.0f;
    const float current_time = (float)glutGet(GLUT_ELAPSED_TIME) * 0.001f;
    simcxt.sim_time = current_time;
    // Check if enough time has passed (0.5 seconds in this case)
    if (current_time - last_metric_time >= 0.5f) {
        // Call the meas_cohesion metric
        float cohesion_score = g_metrics.meas_cohesion(boids_,simcxt);
        float cohesion_score2 = g_metrics.meas_cohesion_v2(boids_,simcxt, GlobalOutput::getStream());
        // float cohesion_score2 = g_metrics.meas_cohesion_v2(boids_,simcxt, out_file);
        // float cohesion_score2 = g_metrics.meas_cohesion_v2(boids_,simcxt);
        // float cohesion_score = g_metrics.meas_cohesion(boids_,current_time);
        // float cohesion_score2 = g_metrics.meas_cohesion_v2(boids_,current_time);
        Vec3f swarm_cog = g_metrics.swarm_center_of_gravity(boids_);
        float swarm_travel_dist = swarm_cog.norm();
        double confweighsum = g_metrics.average_confidence_weight(boids_);
        // std::cout << nowtime << "\t"  << frozen_sim_time << "\t" << delta_time_since_frozen <<  std::endl;

    // if (crt_target_)
    //     std::cout << crt_target_->get_exerted_proximity_force(*this);
        // timestamp coehsion travel_distance average_confidence_weight
        // std::cout << nowtime << "\t"  << swarm_travel_dist << "\t" << confweighsum <<  std::endl;
        // std::cout << nowtime << "\t" << cohesion_score << "\t" << swarm_travel_dist << "\t" << confweighsum <<  std::endl;
        // timestamp coehsion travel_distance
        // std::cout << nowtime << "\t" << cohesion_score << "\t"<< cohesion_score2 << "\t" << swarm_travel_dist <<  std::endl;
        // std::cout << nowtime << "\t" <<  swarm_travel_dist <<  std::endl;
        // std::cout << nowtime << "\t" << cohesion_score << std::endl;

        // Update the last metric time to the current time
        last_metric_time = current_time;
    }

}


void mouseButton(int button, int state, int x, int y)
{
#ifdef ENABLE_GUI
    ImGui_ImplGLUT_MouseFunc(button, state, x, y);
    if (ImGui::GetIO().WantCaptureMouse)
        return;
#endif

    mouse_buttons[button] = state;

    // Update camera
    camera.zoom(mouse_buttons[3] - mouse_buttons[4]);
}

void mousePassiveMotion(int x, int y)
{
    mouse_x = x;
    mouse_y = y;
}

void mouseMotion(int x, int y)
{
#ifdef ENABLE_GUI
    ImGui_ImplGLUT_MotionFunc(x, y);
    if (ImGui::GetIO().WantCaptureMouse)
        return;
#endif

    int mouse_dx = mouse_x - x;
    int mouse_dy = mouse_y - y;
    float dxn = static_cast<float>(mouse_dx) / static_cast<float>(window_w);
    float dyn = -static_cast<float>(mouse_dy) / static_cast<float>(window_h);
    mouse_x = x;
    mouse_y = y;

    // Update camera
    if (mouse_buttons[GLUT_LEFT_BUTTON] == GLUT_DOWN)
        camera.rotate(dxn, dyn);
    if (mouse_buttons[GLUT_RIGHT_BUTTON] == GLUT_DOWN)
        camera.pan(dxn, dyn, 0.0f);
}
// Mouse wheel handler (requires freeglut)
void mouseWheel(int wheel, int direction, int x, int y) {
    std::cout << "mouseWheel: wheel=" << wheel
              << " direction=" << direction
              << " x=" << x << " y=" << y << std::endl;
    float step = 5.0f; //  zoom step
    if (direction > 0) {
        camera.zoom(-step); // zoom in
    } else {
        camera.zoom(step);  // zoom out
    }
    glutPostRedisplay();
}

void timer(int v)
{
    glutPostRedisplay();
    glutTimerFunc(1000 / FPS, timer, 0);
}
// ends the simulation after some time and capture screenshot at that time
// This is called after the specified time
void timerCallback(int value) {
        // captureScreenshot();
        glutLeaveMainLoop(); // Exits the glutMainLoop
    }

// just called to make a screenshot at a given time 
void timerScreenshotCallback(int value) {
        captureScreenshot();
    }

// Main function: GLUT runs as a console application
int main(int argc, char **argv)
{
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    // plog::init(plog::debug, &consoleAppender); // set severity: verbose, debug, info, ...
    plog::init(plog::info, &consoleAppender); // set severity: verbose, debug, info, ...

    PLOG_INFO << "First log, using message";
    // starting message
#ifdef USE_MESSAGES
    std::cout << "Using MESSAGES!." << std::endl;
#else 
    std::cout << "No message." << std::endl;
#endif
    // fixed seed
    srand(10); // or srand(time(10)); need ctime
    // srand(unsigned int 10);
    if (argc < 2) {
        std::cout << "Using default parameters. Use -h or --help for usage." << std::endl;
        // return 1;
    }
    if (parse_command_line_args(argc, argv) == 1) return 0;
    set_boid_params(boids_number,neighborhood_max_dist,
            boid_to_boid_min_dist,max_speed,target_attraction_weight ,
            target_velocity_alignment_weight,min_cos_FOV,fence_size,
            fence_repel_weight,
            msg_latency,force_randomness);
    set_boid_weights( separation_weight, alignment_weight, cohesion_weight);

    //-----------------------
    // Init GLUT and create window
    //-----------------------
    glutInit(&argc, argv);
    // will enable to run the simulation for a fixed duration,
    // see timer callback
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutInitWindowSize(window_w, window_h);
    glutInitWindowPosition(WINDOW_X, WINDOW_Y);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glViewport(0, 0, window_w, window_h);
    window_id = glutCreateWindow("drone boid");

    // prepare filename to log the cohesion metric
    float randforce = 1.9f;
    float dronedist = 2.0f;
    metricfilename = genCohesionMetricFilename(randforce, dronedist);

     // out_file(metricfilename);
    std::ofstream out_file(metricfilename);
        GlobalOutput::getStream().open(metricfilename);
    if (!GlobalOutput::getStream()) {
        std::cerr << "Failed to open output file: " << metricfilename << std::endl;
        return 1;
    }

    // Write command line and headers
    GlobalOutput::getStream() << "# Command: ";
    for (int i = 0; i < argc; ++i) GlobalOutput::getStream() << argv[i] << " ";
    GlobalOutput::getStream() << std::endl;
    GlobalOutput::getStream() << "t(s)\tsigma_r\t..." << std::endl;

    // if (!out_file) {
    //     std::cerr << "Failed to open output file: " << metricfilename << std::endl;
    //     return 1;
    // }
        const int value_width = 8;
        const int precision = 5;

        // // Set formatting for floating-point numbers
        std::cout  << std::setprecision(precision);
        // std::cout << std::fixed << std::setprecision(precision);
        if (!g_show_log_metric_header )
        {
        std::cout << std::left << std::setw(value_width) << "METRIC DETAILS" << std::endl;
        std::cout << std::left << std::setw(value_width) << "t(s)" << "\t"     
                   << std::setw(value_width) << "sigma_r" << "\t"     
                   << std::setw(value_width) << "sigma_d" << "\t"     
                   << std::setw(value_width) << "P_unsafe" << "\t"    
                   << std::setw(value_width) << "P_collided" << "\t"  
                   << std::setw(value_width) << "C_R_ref" << "\t"     
                   << std::setw(value_width) << "C_d_sep" << "\t"     
                   << std::setw(value_width) << "C_P_unsafe" << "\t"  
                   << std::setw(value_width) << "C_P_collide" << "\t" 
                   << std::setw(value_width) << "rawC" << "\t"        
                   << std::setw(value_width) << "C_sigmoid" << "\t"   
                   << std::setw(value_width) << "rawC_clampd" << "\t"
                   << std::setw(value_width) << "C_clamped"   
                  << std::endl;
        g_show_log_metric_header = true;
        }
        // Print header labels for the metric log
    // GlobalOutput::getStream() << "t(s)" << "\t"
    //          << "sigma_r" << "\t"
    //          << "sigma_d" << "\t"
    //          << "P_unsafe" << "\t"
    //          << "P_collided" << "\t"
    //          << "C_R_ref" << "\t"
    //          << "C_d_sep" << "\t"
    //          << "C_P_unsafe" << "\t"
    //          << "C_P_collide" << "\t"
    //          << "rawC" << "\t"
    //          << "C_sigmoid" << "\t"
    //          << "rawC_clampd" << "\t"
    //          << "C_clamped"
    //          << std::endl;
    GlobalOutput::getStream() << "t(s)" << "\t"
             << std::setw(value_width)<< "sigma_r" << "\t"
             << std::setw(value_width)<< "sigma_d" << "\t"
             << std::setw(value_width)<< "P_unsafe" << "\t"
             << std::setw(value_width)<< "P_collided" << "\t"
             << std::setw(value_width)<< "C_R_ref" << "\t"
             << std::setw(value_width)<< "C_d_sep" << "\t"
             << std::setw(value_width)<< "C_P_unsafe" << "\t"
             << std::setw(value_width)<< "C_P_collide" << "\t"
             << std::setw(value_width)<< "rawC" << "\t"
             << std::setw(value_width)<< "C_sigmoid" << "\t"
             << std::setw(value_width)<< "rawC_clampd" << "\t"
             << std::setw(value_width)<< "C_clamped"
             << std::setw(value_width)<< std::endl;
    init();

    // Register callbacks
    glutDisplayFunc(display);
    glutIdleFunc(systemEvolution);
    glutTimerFunc(1000 / FPS, timer, 0);

    // Setup Dear ImGui context
#ifdef ENABLE_GUI
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
#endif

    // Replace some handlers after ImGui_ImplGLUT_InstallFuncs() sets its own
    // our impls will call the Imgui impls internally
    glutPassiveMotionFunc(mousePassiveMotion);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseButton);
    glutKeyboardFunc(processKeys);
    glutMotionFunc(mouseMotion);
#ifdef FREEGLUT
    glutMouseWheelFunc(mouseWheel);
#endif
 
    // simulation duration 
    // int fixed_duration_ms = 5000; // TODO add as a CLI parameter 
#ifdef SCREENSHOT_AT_X_SECONDS
    // capture screenshot after x seconds
    glutTimerFunc(6*1000.0, timerScreenshotCallback, 0);
#endif
    glutTimerFunc(simulation_duration_s*1000.0, timerCallback, 0);
    // glutTimerFunc(fixed_duration_ms, timerCallback, 0);

    glutMainLoop();

#ifdef ENABLE_GUI
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();
    ImGui::DestroyContext();
#endif
    // metricfilename
    // out_file.close();
    GlobalOutput::getStream().close();
    return 0;
}

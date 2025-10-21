/*********************************************************************************************************************
 * File : camera_trackball.cpp                                                                                       *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#include "camera_trackball.h"

#include <GL/glut.h>
#include <Eigen/Dense>

const float CameraTrackball::DEFAULT_EULER_YAW = 1.0f;
const float CameraTrackball::DEFAULT_EULER_PITCH = 0.6f;

CameraTrackball::CameraTrackball()
    : eye_{0.0f, 0.0f, 0.0f},
      center_{0.0f, 0.0f, 0.0f},
      front_{0.0f, 0.0f, 1.0f},
      left_{0.0f, 0.0f, 1.0f},
      up_{0.0f, 0.0f, 1.0f},
      r_{1.0f},
      yaw_{3.14f},
      pitch_{1.0f}
{
}

void CameraTrackball::init(const Vec3f &center, float r0)
{
    center_ = center;
    r_ = r0;
    yaw_ = DEFAULT_EULER_YAW;
    pitch_ = DEFAULT_EULER_PITCH;
}

    // Setter to change the mode
    void CameraTrackball::setCamViewMode(CameraMode mode) { 
        current_mode_ = mode; 
    }
    void CameraTrackball::lookAt( )
    // void CameraTrackball::lookAt(CameraMode mode )
    {
    Vec3f eye, up;
    Vec3f center = center_;

    if (current_mode_== TOP_DOWN) {
    // if (mode == TOP_DOWN) {
        // Position the camera high above the scene (along Z-axis)
        // we can use a fixed distance or a relative one based on `r_`
        eye = center_ + Vec3f(0.0, 0.0, r_); 
        // Look down at the center of the scene
        center = center_;
        // For Z-up, the 'up' vector is in the Y direction
        up = Vec3f(0.0, 1.0, 0.0);
    } else { // Standard trackball mode
        front_ = (-(cos(pitch_) * cos(yaw_) * Vec3f::UnitX() +
                    cos(pitch_) * sin(yaw_) * Vec3f::UnitY() +
                    sin(pitch_) * Vec3f::UnitZ()))
                     .normalized();
        left_ = (Vec3f::UnitZ().cross(front_)).normalized();
        up_ = (front_.cross(left_)).normalized();
        eye = center_ - r_ * front_;
        up = Vec3f(0.0, 0.0, 1.0); // Your existing up vector for trackball
    }
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye[0], eye[1], eye[2],
              center[0], center[1], center[2],
              up[0], up[1], up[2]);
}


// void CameraTrackball::lookAt()
// {
//     front_ = (-(cos(pitch_) * cos(yaw_) * Vec3f::UnitX() +
//                 cos(pitch_) * sin(yaw_) * Vec3f::UnitY() +
//                 sin(pitch_) * Vec3f::UnitZ()))
//                  .normalized();
//
//     left_ = (Vec3f::UnitZ().cross(front_)).normalized();
//     up_ = (front_.cross(left_)).normalized();
//
//     eye_ = center_ - r_ * front_;
//
//     glMatrixMode(GL_MODELVIEW);
//     glLoadIdentity();
//     gluLookAt(eye_[0], eye_[1], eye_[2],
//               center_[0], center_[1], center_[2],
//               0.0, 0.0, 1.0);
// }

void CameraTrackball::rotate(float right_input, float up_input)
{
    static const float HALFPI_F = static_cast<float>(0.5 * M_PI);

    yaw_ += HALFPI_F * right_input;
    pitch_ += HALFPI_F * up_input;

    // Clamp pitch to ]-pi/2, pi/2[
    static const float PITCH_MIN = -HALFPI_F + 0.001f;
    static const float PITCH_MAX = HALFPI_F - 0.001f;
    pitch_ = std::min(std::max(pitch_, PITCH_MIN), PITCH_MAX);
}

void CameraTrackball::pan(float right_input, float up_input, float front_input)
{
    center_ +=
        -right_input * r_ * left_ +
        up_input * r_ * up_ +
        front_input * r_ * front_;
}

void CameraTrackball::zoom(int zoom_input)
{
    r_ *= std::pow(1.1f, zoom_input);
}

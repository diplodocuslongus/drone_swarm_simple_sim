/*********************************************************************************************************************
 * File : ref_axis.cpp                                                                                               *
 *                                                                                                                   *
 * 2025 diplodocuslongus (Ludo)
 *********************************************************************************************************************/

#include <GL/glut.h>
#include <iostream>
#include "ref_axis.h"
#include "gl_utils.h"



// Initialize cuboid fence from 2 "vertices" corresponding to 
// min and max in x,y and z
RefAxis::RefAxis(const Vec3f &vertice1, const Vec3f &vertice2) : MovingObject((vertice1 + vertice2) / 2.0f)
{
    // Initialize properties for cuboid
    boid_type_ = 5; // RefAxis
    // Store the min and max coordinates
    min_x_ = std::min(vertice1.x(), vertice2.x());
    max_x_ = std::max(vertice1.x(), vertice2.x());
    min_y_ = std::min(vertice1.y(), vertice2.y());
    max_y_ = std::max(vertice1.y(), vertice2.y());
    min_z_ = std::min(vertice1.z(), vertice2.z());
    max_z_ = std::max(vertice1.z(), vertice2.z());
}


Vec3f RefAxis::get_exerted_proximity_force(const MovingObject &object) const
{
        Vec3f force(0, 0, 0);
        return force;
}

void RefAxis::update(float t)
{
}

void RefAxis::update_no_ang_velocity_clamp(float t)
{
}
void drawGrid(float size, int divisions) {
    float step = size / divisions;
    glColor3f(0.7f, 0.7f, 0.7f);  // gray

    glBegin(GL_LINES);
    for (int i = -divisions; i <= divisions; i++) {
        float pos = i * step;
        // lines parallel to X
        glVertex3f(-size, pos, 0);
        glVertex3f( size, pos, 0);

        // lines parallel to Y
        glVertex3f(pos, -size, 0);
        glVertex3f(pos,  size, 0);
    }
    glEnd();
}

void RefAxis::draw() const
{
    glPushMatrix();
    // glTranslatef(0.0,0.0,0.0);
    // glColor3f(0.2, 0.9, 0.8);
    glLineWidth(3.0f);   // width in pixels, try 2.0–5.0
    GLUquadric* quad = gluNewQuadric();

    // X axis
    glColor3f(1, 0, 0);
    glRotatef(90, 0, 1, 0);        // orient cylinder along X
    gluCylinder(quad, 0.1, 0.1, 5.0, 16, 16);
    glTranslatef(0, 0, 5.0);
    gluCylinder(quad, 0.3, 0.0, 2.0, 16, 16); // arrowhead
    glPopMatrix();
    // --- Y axis ---
    glPushMatrix();
        glColor3f(0, 1, 0);
        glRotatef(-90, 1, 0, 0);
        gluCylinder(quad, 0.1, 0.1, 5.0, 16, 16);
        glTranslatef(0, 0, 5.0);
        gluCylinder(quad, 0.3, 0.0, 2.0, 16, 16);
    glPopMatrix();

    // --- Z axis ---
    glPushMatrix();
        glColor3f(0, 0, 1);
        gluCylinder(quad, 0.1, 0.1, 5.0, 16, 16);
        glTranslatef(0, 0, 5.0);
        gluCylinder(quad, 0.3, 0.0, 2.0, 16, 16);
    glPopMatrix();
    glBegin(GL_LINES);
        // X axis (red)
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(5, 0, 0);

        // Y axis (green)
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 5, 0);

        // Z axis (blue)
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 5);
    glEnd();
    glPushMatrix();
    drawGrid(5.0f, 2);

    glPopMatrix();
}

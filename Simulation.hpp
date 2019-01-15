#pragma once

#include "Path.hpp"

#include <functional>
#include <iostream>

#include <gtkmm.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>

#include <glm/glm.hpp>

/// Container for mouse input
struct MouseInfo{
    /// Normalized X Coordinate in [-1, 1] from left to right
    float x;
    /// Normalized Y Coordinate in [-1, 1] from top to bottom
    float y;
    /// Amount the scroll wheel is turning
    double scroll;
    /// Left button pressed
    bool leftDown;
    /// Right button pressed
    bool rightDown;
    /// Mouse is on the screen
    bool onScreen;
};

class Simulation{
public:
    /// \return the trajectory the car is currently following
    virtual const Trajectory& get_trajectory() = 0;
    /// \return the transformation matrix of the car's current position and orientation
    virtual glm::mat4 get_transform() = 0;
    /// \return the transformation matrix of the car's current orientation
    virtual glm::mat4 get_rotation() = 0;

    //TODO Probably should use quaternions, but only have rotation about one axis now anyway.
    /// Gets the car's current state information
    /// \param position The car's 3D position
    /// \param velocity The car's 3D velocity
    /// \param orientation The XYZ Euler angles that define the car's orientation
    /// \param angular_velocity The angular velocity about each axis
    virtual void get_state(glm::vec3 &position, glm::vec3 &velocity, glm::vec3 &orientation, glm::vec3 &angular_velocity) = 0;

    /// Advances the simulation a certain amount of time
    /// \param keyPressed A function to return the state of each key
    /// \param mouse The current mouse inputs
    /// \param dt The amount of time to simulate
    virtual void update(std::function<bool(guint key)> keyPressed, MouseInfo& mouse, Float dt) = 0;

    /// Overlays the view with debug information
    /// \param camera The camera matrix
    /// \param mvpUniform The id of the OpenGL uniform for the MVP matrix
    /// \param colorUniform The id of the OpenGL uniform for the color to draw
    virtual void draw_debug(glm::mat4 camera, GLint mvpUniform, GLint colorUniform){}

    /// Overlays the view with other information
    /// \param camera The camera matrix
    /// \param mvpUniform The id of the OpenGL uniform for the MVP matrix
    /// \param colorUniform The id of the OpenGL uniform for the color to draw
    virtual void draw_info(glm::mat4 camera, GLint mvpUniform, bool pause) {}
};

/// Creates the simulation to run
/// \return A pointer to the simulation
std::unique_ptr<Simulation> create_simulation();

/// float PI constant
constexpr float PI = static_cast<float>(M_PI);
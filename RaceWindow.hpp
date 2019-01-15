#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <utility>
#include <vector>

#include <gtkmm.h>
#include <gtkmm/button.h>
#include <gtkmm/window.h>
#include <giomm/resource.h>
#include <gdk/gdkkeysyms.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "RgbImage.hpp"
#include "GlModel.hpp"
#include "OpenGlWrappers.hpp"
#include "Simulation.hpp"

constexpr bool DRAW_DEBUG = true;
constexpr bool SAVE_CONES = false;

using std::cerr;
using std::endl;
using std::string;

/// Class for storing a cone, including its position and type
struct Cone{
    /// The four types of cones
    enum class Type{
        Blue,
        Yellow,
        Orange,
        TallOrange
    };

    /// Creates a cone
    /// \param x X coordinate
    /// \param z Z coordinate
    /// \param angle Orientation of the cone
    /// \param stripeAngle The angle of the text on the stripe
    /// \param type The type of cone
    Cone(float x, float z, float angle, float stripeAngle, Type type) noexcept
            : x(x), z(z), angle(angle), stripeAngle(stripeAngle), type(type) {}

    float x;
    float z;
    float angle;
    float stripeAngle;
    Type type;
};

/// Main class that updates and renders the simulation
class RaceWindow : public Gtk::Window
{

public:
    RaceWindow();

    ~RaceWindow() override {
        gtk_widget_remove_tick_callback(Widget::gobj(), tickCallback);
    }

private:
    Gtk::Box windowBow {Gtk::Orientation::ORIENTATION_VERTICAL, false};
    Gtk::GLArea simulationArea;
    Gtk::Box sliderBox {Gtk::Orientation::ORIENTATION_HORIZONTAL, false};
    Gtk::TextView timeText;
    Glib::RefPtr<Gtk::TextBuffer> timeTextBuffer;

    std::unique_ptr<Program> cone_program;
    GLint mvpUniform = -1;
    GLint coneColorUniform = -1;
    GLint lightDirectionUniform = -1;
    GLint stripeStart1Uniform = -1;
    GLint stripeStart2Uniform = -1;
    GLint stripeEnd1Uniform = -1;
    GLint stripeEnd2Uniform = -1;
    GLint stripeAngleUniform =  -1;
    GLint textureUniform = -1;

    std::unique_ptr<Program> solid_color_program;
    GLint solidMvpUniform = -1;
    GLint solidColorUniform = -1;

    std::unique_ptr<Program> any_color_program;
    GLint anyMvpUniform = -1;

    GLuint black_cone_texture = 0;
    GLuint white_cone_texture = 0;

    guint tickCallback = 0;

    std::unique_ptr<GlModel> coneModel;
    std::unique_ptr<GlModel> carModel;
    std::vector<Cone> cones;

    float zoom = 1.f;
    bool lastPausePress = false;
    bool pause = false;

    std::unique_ptr<Simulation> _simulation = create_simulation();

    std::array<float, 3> viewAngles{};

    /// Updates a float variable with the value from an adjustment
    /// \param dest A pointer to where it should be stored
    /// \param a The adjustment to get the value from
    void updateFromAdjustment(float *dest, const Glib::RefPtr<Gtk::Adjustment> &a);

    /// Initializes OpenGL resources
    void initOpenGl();
    /// Releases OpenGL resources
    void deinitOpenGl();
    /// Renders a frame of the simulation
    /// \param context The OpenGL context
    /// \return true if it should continue getting called
    bool render(const Glib::RefPtr<Gdk::GLContext>& context);

    /// Draws everything that should have motion blur.
    /// \param camera Camera matrix
    void drawWorld(const glm::mat4& camera);
    /// Draws everything that should have motion blur.
    /// \param camera Camera matrix
    void drawNoBlur(const glm::mat4 &camera);

    /// Draws the specified lines with the transformation applied
    /// \param vector A vector of lines stored as two end points combined into a single vector
    /// \param mvp The model view perspective matrix
    void drawLines(const std::vector<glm::vec4> &vector, glm::mat4 &mvp);

    /// Called to trigger the next render
    /// \param frame_clock The frame clock
    /// \return True if rendering should continue (always true)
    gboolean tick(GdkFrameClock *frame_clock);

    /// Helper function to call tick on a certain RaceWindow that is compatible with Gtkmm
    /// \param widget Not used
    /// \param frame_clock The clock to pass to tick
    /// \param user_data A pointer to the RaceWindow
    /// \return The result of tick
    static gboolean staticTick(GtkWidget *widget, GdkFrameClock *frame_clock, gpointer user_data);

    static void destroyNotify(gpointer data){}

    std::unordered_map<guint, bool> _keyboard_map;
    /// Function to check if a key is pressed
    /// \param key The ID of the key to check (GDK_KEY_X)
    /// \return True if it's pressed
    bool key_pressed(guint key);

    bool on_key_press(GdkEventKey *event);

    MouseInfo _mouse;
    gboolean on_mouse_move(GdkEventMotion *event);
    gboolean on_mouse_scroll(GdkEventScroll *event);
};
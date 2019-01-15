#include "Simulation.hpp"

#include <memory>
#include <valarray>

#include <eigen3/Eigen/Dense>

#include <glm/gtc/matrix_transform.hpp>

#include "gurobi_c++.h"

///Gravitational constant in m/s^2
constexpr Float gravity = 9.81;

/// The ratio between the maximum acceleration of the trajectory and the maximum
constexpr Float plan_margin = .9;

//A veriety of vehicle parameters. They're mostly the best estimates I can find without spending a ton of time on it
/// Mass of the racecar in kg, between weight with and without driver...
constexpr Float car_mass = 225;
/// Moment of inertia in kg*m^2
constexpr Float moment_of_inertia = car_mass * 1.25f*1.25f; //TODO get a better number
/// Coefficient of friction
constexpr Float mu = 1.6f;
/// downforce/(mass*velocity^2)
constexpr Float aero_accel_coef = 1.f/(30.f*30.f);
/// Maximum power the car will try to use (75kW). Rules limit to 80kW, but need some margin.
constexpr Float max_power = 75000;
/// Maximum steer angle
constexpr Float steer_limit = PI/4;

//Car dimensions
/// Distance between front and back wheels in m
constexpr Float wheelbase = 1.5291f;
/// Distance between left and right wheels in m
constexpr Float wheel_track = 1.1662f;
/// Height of the center of mass in m
constexpr Float center_of_mass_height = .31f;
/// Position of the front right wheel in 2D relative to the center of mass
const Vector2 fr_position(wheelbase/2, -wheel_track/2);
/// Position of the front left wheel in 2D relative to the center of mass
const Vector2 fl_position(wheelbase/2, wheel_track/2);
/// Position of the rear left wheel in 2D relative to the center of mass
const Vector2 rl_position(-wheelbase/2, wheel_track/2);
/// Position of the rear right wheel in 2D relative to the center of mass
const Vector2 rr_position(-wheelbase/2, -wheel_track/2);

/// Time it takes the lateral suspension to
constexpr Float suspension_lateral_time_constant = .3f;
constexpr Float suspension_longitudinal_time_constant = .3f;

/// Tune this to change the tire model, which affects how aggressively the racecar can accelerate
constexpr Float lateral_longitudinal_factor = 3;

/// Container for storing vertex positions and colors
struct ColorVertex{
    ColorVertex(const glm::vec4 &position, const glm::vec4 &color) : position(position), color(color) {}

    glm::vec4 position;
    glm::vec4 color;
};
static_assert(sizeof(ColorVertex)==32);

/// Storage class for fixed size arrays with named indices
/// An implementation that used Eigen::Matrix is commented out because std::valarray
/// works better with GDB and is faster
/// \tparam Vars An enum with the names for the indices. Must have a Length as the last element
template<typename Vars>
struct NamedVector : std::valarray<Float>{ //Eigen::Matrix<Float, static_cast<int>(Vars::Length), 1>{
    //typedef Eigen::Matrix<Float, static_cast<int>(Vars::Length), 1> BaseType;
    typedef std::valarray<Float> BaseType;

    NamedVector() : BaseType(static_cast<size_t>(Vars::Length)){} //= default;

    explicit NamedVector(BaseType &&other)
            : BaseType(other) {}

    Float& operator[](Vars var){
        int index = static_cast<int>(var);
        if(index < 0 || index >= static_cast<int>(Vars::Length))
            throw std::out_of_range("Tried to access index out of range");
        return BaseType::operator[](index);
    }

    const Float& operator[](Vars var) const{
        int index = static_cast<int>(var);
        if(index < 0 || index >= static_cast<int>(Vars::Length))
            throw std::out_of_range("Tried to access index out of range");
        return BaseType::operator[](index);
    }

    Vector2 get(Vars var1, Vars var2) const{
        return Vector2(this[var1], this[var2]);
    }

    Eigen::Vector3f get(Vars var1, Vars var2, Vars var3) const{
        return Eigen::Vector3f(this[var1], this[var2], this[var3]);
    }

    void set(Vars var1, Vars var2, Vector2 vals){
        this[var1] = vals[0];
        this[var2] = vals[1];
    }

    void set(Vars var1, Vars var2, Vars var3, Eigen::Vector3f vals){
        this[var1] = vals[0];
        this[var2] = vals[1];
        this[var3] = vals[2];
    }
};

/// Runs a simulation a certain number of iterations
/// \tparam TState The type of the state vector
/// \tparam TCommand The type of the command vector
/// \param initial The initial state
/// \param dynamics A function that computes the derivative of the state vector given the current state and command
/// \param controller Determines the control command given the state
/// \param integrator The integration function to use
/// \param controllerPeriod The period at which the control input is updated
/// \param controller_updates The number of controller updates to run the simulation for
/// \param iters_per_update The number of iterations to simulate within a controller update
/// \param log A vector to store all the states in or nullptr if not needed
/// \return The final state after advancing the simulation
template<typename TState, typename TCommand>
TState run(TState initial, std::function<TState(TState, TCommand)> dynamics,
           std::function<TCommand(TState)> controller, std::function<TState(decltype(dynamics), TState, TCommand, Float)> integrator,
           Float controllerPeriod, unsigned controller_updates, unsigned iters_per_update,
           std::vector<TState>* log = nullptr){
    TState state = initial;
    Float dt = controllerPeriod/iters_per_update;
    for(int i = 0; i< controller_updates; i++){
        TCommand command = controller(state);
        for(int j = 0; j < iters_per_update; j++){
            state = integrator(dynamics, state, command, dt);
            if(log)
                log->push_back(state);
            //std::cout << state[CircleModel::X] << ',' << state[CircleModel::Z] << std::endl;
        }
    }
    return state;
}

/// Performs Euler integration. Note that time is not kept track of separately. If needed,
/// it can be added to the state vector and dynamics should always return 1 for its derivative
/// \tparam TState The type of the state vector
/// \tparam TCommand The type of the command vector
/// \param dynamics A function that computes the derivative of the state vector given the current state and command
/// \param state The initial state
/// \param command The command at this instant
/// \param dt The time step size
/// \return The new state after dt
template<typename TState, typename TCommand>
TState Euler(std::function<TState(const TState&, const TCommand&)> dynamics, const TState& state, const TCommand& command, Float dt){
    return TState(state + dt * dynamics(state, command));
}

/// Performs RK4 integration. Note that time is not kept track of separately. If needed,
/// it can be added to the state vector and dynamics should always return 1 for its derivative
/// \tparam TState The type of the state vector
/// \tparam TCommand The type of the command vector
/// \param dynamics A function that computes the derivative of the state vector given the current state and command
/// \param state The initial state
/// \param command The command at this instant
/// \param dt The time step size
/// \return The new state after dt
template<typename TState, typename TCommand>
TState RK4(std::function<TState(const TState&, const TCommand&)> dynamics, const TState& state, const TCommand& command, Float dt){
    //Moved dt calculations from standard equations to improve performance (~10% faster)
    TState k1(dynamics(state, command));
    TState k2(dynamics(TState(state + k1*dt/2), command));
    TState k3(dynamics(TState(state + k2*dt/2), command));
    TState k4(dynamics(TState(state + k3*dt), command));
    return TState(state + (k1 + 2*(k2 + k3) + k4)*dt/6);
}

/// The state vector enum for the CircleSimulation
enum class CircleModel : unsigned{
    X,
    Z,
    VX,
    VZ,
    Length
};

/// The command vector enum for the CircleSimulation
enum class CircleControl : unsigned{
    Force,
    Length
};

/// A simulation of a car moving with constand lateral acceleration, forming a circle
/// Just for testing other parts of the simulation
class CircleSimulation : public Simulation{
    typedef CircleModel Model;
    typedef NamedVector<Model> State;
    typedef NamedVector<CircleControl> Command;

public:
    explicit CircleSimulation(State &&state)
        : state(state){
    }

    glm::mat4 get_rotation() override{
        return glm::rotate(glm::mat4(1.f), static_cast<float>(std::atan2(state[Model::VZ], state[Model::VX])), glm::vec3(0, 1.f, 0));
    }


    glm::mat4 get_transform() override{
        return glm::rotate(glm::translate(glm::mat4(1.f),glm::vec3(state[Model::X], 0.f, -state[Model::Z])),
                           static_cast<float>(std::atan2(state[Model::VZ], state[Model::VX])), glm::vec3(0, 1.f, 0));

    }

    void get_state(glm::vec3 &position, glm::vec3 &velocity, glm::vec3 &orientation, glm::vec3 &angular_velocity) override{
        position[0] = static_cast<float>(state[CircleModel::X]);
        position[2] = static_cast<float>(state[CircleModel::Z]);
        velocity[0] = static_cast<float>(state[CircleModel::VX]);
        velocity[2] = static_cast<float>(state[CircleModel::VZ]);
    }


    void update(std::function<bool(guint key)> keyPressed, MouseInfo& mousa, Float dt) override {
        dt += last_dt;
        while(dt > 0){
            if(next_controller <= 0){
                command = controller(state);// * ((keyPressed(GDK_KEY_Left) ? 1.f : 0.f) + (keyPressed(GDK_KEY_Right) ? -1.f : 0.f)));
                next_controller = controller_period;
            }

            state = RK4<State, Command>(dynamics, state, command, time_step);

            next_controller -= time_step;
            dt -= time_step;
        }
        last_dt = dt;
    }

    const Trajectory& get_trajectory() override {
        return trajectory;
    }

private:
    static constexpr Float controller_period = .1f;
    static constexpr Float time_step = 1/6000.f;

    Float next_controller = 0;
    Float last_dt = 0;
    State state;
    Command command;
    Trajectory trajectory{std::make_shared<Arc>(Vector2(5, 0), 5, 0, 2*PI),
            time_step, 100, 14.547027, 0};

    static State dynamics(const State &st, const Command &c){
        State derivative;
        Float vx = st[Model::VX];
        Float vz = st[Model::VZ];
        derivative[Model::X] = vx;
        derivative[Model::Z] = vz;
        //Vector2 force(vz, -vx);
        Float force_factor = c[CircleControl::Force]/std::hypot(vx, vz);
        derivative[Model::VX] = force_factor * vz;//force[0];
        derivative[Model::VZ] = force_factor * -vx;//force[1];
        return derivative;
    }

    static Command controller(const State &st){
        Command c;
        c[CircleControl::Force] = 14.547027f;
        return c;
    }

    void test(){
        state = run<State, Command>(state, dynamics, controller, Euler<State, Command>, .0628f, 100, 1, nullptr);
    }
};

/// The state vector enum used for all the racecar simulations
enum class NonlinearModel : unsigned{
    X,
    Z,
    VX,
    VZ,
    Heading,
    AngularVelocity,
    SteeringAngle,
    FrontSuspension,
    LeftSuspension,
    Length
};

/// The command vector enum used for the racecar simulations
enum class FourWheelControl : unsigned{
    TFL,
    TFR,
    TRL,
    TRR,
    Steer,
    Length
};

/// 2D vector cross product
/// \param v1 The first vector
/// \param v2 The second vector
/// \return The cross product
Float cross2(const Vector2 &v1, const Vector2 &v2) {
    return v1[0]*v2[1] - v1[1]*v2[0];
}

//TODO add a way to account for controller latency (Run forward dynamics with inputs that will be used, calculate
//TODO new control signals and save old inputs)

/// Computes the current force on a tire
/// \param torque_request The normalized torque request for the motor on this wheel in the [-1, 1] range
/// \param max_force The maximum force this tire can exert in any direction
/// \param ground_velocity The velocity of the ground relative to the spinning tread of the wheel
/// \param direction A unit vector in the direction the tire is pointing
/// \return The force the tire applies on the racecar
Vector2 TireForce(Float torque_request, Float max_force, const Vector2 &ground_velocity, const Vector2 &direction){
    Float ground_speed_squared = ground_velocity.squaredNorm();
    if(ground_speed_squared < .00000001f){
        return torque_request * std::max(static_cast<Float>(0), max_force) * direction;
    }

    Vector2 normal_direction(-direction[1], direction[0]);
    Float ground_speed_on_axis = ground_velocity.dot(direction);
    Float ground_speed_off_axis = ground_velocity.dot(normal_direction);
    Float denominator = (ground_speed_on_axis * ground_speed_on_axis * std::abs(torque_request) + lateral_longitudinal_factor*ground_speed_off_axis*ground_speed_off_axis);
    if(denominator < .00000000000000001f){
        return Vector2(0,0);
    }
    return max_force * (ground_speed_on_axis * ground_speed_on_axis * torque_request* std::abs(torque_request) * direction
          + lateral_longitudinal_factor*ground_speed_off_axis * std::abs(ground_speed_off_axis) * normal_direction)/denominator;
}

/// Computes the current force on a tire. Same as the previous implementation, except it's
/// a bit simpler and slower because it uses an angle instead of a unit vector.
/// \param torque_request The normalized torque request for the motor on this wheel in the [-1, 1] range
/// \param max_force The maximum force this tire can exert in any direction
/// \param ground_velocity The velocity of the ground relative to the spinning tread of the wheel
/// \param angle The angle of the tire in radians
/// \return The force the tire applies on the racecar
Vector2 TireForce(Float torque_request, Float max_force, const Vector2 &ground_velocity, Float angle){
    Float ground_speed_squared = ground_velocity.squaredNorm();
    if(ground_speed_squared < .00000001f){
        return torque_request * std::max(static_cast<Float>(0), max_force) * Vector2(std::cos(angle), std::sin(angle));
    }

    Vector2 rotated_ground_v = Rotation2D(-angle) * ground_velocity;

    return max_force/ground_speed_squared * (Rotation2D(angle) *
        (rotated_ground_v.array() * Array2(torque_request * rotated_ground_v[0], std::abs(rotated_ground_v[0]))).matrix());
}

///// Unfinished common helper class to simplify Simulation classes
///// \tparam Model Enum type of the state vector
///// \tparam State Type of the state vector
///// \tparam Command Type of the command vector
//template<typename Model, typename State, typename Command>
//class SimulationHelper : public Simulation{
//public:
//    SimulationHelper(const Float controller_period, const Float time_step, State state)
//            : _controller_period(controller_period), _time_step(_time_step), _state(state) {
//
//    }
//
//    glm::mat4 get_transform() override{
//        return glm::translate(glm::rotate(glm::translate(glm::mat4(1.f),glm::vec3(_state[Model::X], 0.f, -_state[Model::Z])),
//                                          _state[Model::Heading], glm::vec3(0, 1.f, 0)), glm::vec3(-wheelbase/2, 0, 0));
//    }
//
//private:
//    const Float _controller_period;
//    const Float _time_step;
//
//    Float _next_controller = 0;
//    Float _last_dt = 0;
//    State _state;
//    Command _last_command;
//
//};


class NonlinearSimulation : public Simulation{
public:
    typedef NonlinearModel Model;
    typedef NamedVector<Model> State;
    typedef NamedVector<FourWheelControl> Command;

    /// Creates a racecar simulation
    /// \param state The initial racecar state
    /// \param path The path for the racecar to follow
    NonlinearSimulation(State &&state, std::shared_ptr<PathSegment> path)
        : state(state),
          trajectory(std::move(path), 50*time_step, max_power/car_mass, mu * gravity * plan_margin, aero_accel_coef * plan_margin){
    }

    /// Frees the OpenGL resources
    ~NonlinearSimulation(){
        glDeleteBuffers(1, &trajectory_vbo);
        glDeleteVertexArrays(1, &trajectory_vao);
    }

    glm::mat4 get_rotation() override{
        return glm::rotate(glm::mat4(1.f), static_cast<float>(state[Model::Heading]), glm::vec3(0, 1.f, 0));
    }

    glm::mat4 get_transform() override{
        return glm::translate(glm::rotate(glm::translate(glm::mat4(1.f),glm::vec3(static_cast<float>(state[Model::X]), 0.f, -static_cast<float>(state[Model::Z]))),
                           static_cast<float>(state[Model::Heading]), glm::vec3(0, 1.f, 0)), glm::vec3(-wheelbase/2, 0, 0));
    }

    void get_state(glm::vec3 &position, glm::vec3 &velocity, glm::vec3 &orientation, glm::vec3 &angular_velocity) override{
        position[0] = static_cast<float>(state[Model::X]);
        position[2] = static_cast<float>(state[Model::Z]);
        velocity[0] = static_cast<float>(state[Model::VX]);
        velocity[2] = static_cast<float>(state[Model::VZ]);
        orientation[1] = static_cast<float>(state[Model::Heading]);
        angular_velocity[1] = static_cast<float>(state[Model::AngularVelocity]);
    }

    /// Draws red vectors showing the forces on each tire
    void draw_debug(glm::mat4 camera, GLint mvpUniform, GLint colorUniform) override {
        glm::mat4 mvp(camera);
        glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, reinterpret_cast<float*>(&mvp));
        glm::vec4 red(1, 0, 0, 1);
        glUniform4fv(colorUniform, 1, reinterpret_cast<float*>(&red));

        glLineWidth(5);

        glm::mat4 pm(glm::rotate(glm::translate(glm::mat4(1.f),glm::vec3(static_cast<float>(state[Model::X]), 0.f, static_cast<float>(-state[Model::Z]))),
                    static_cast<float>(state[Model::Heading]), glm::vec3(0, 1.f, 0)));

//Want to rotate start positions, not forces
        constexpr Float scale = 5/(mu * car_mass * gravity /4);
        std::vector<glm::vec4> vertices{
            {fr_position[0], .5f, fr_position[1], 1.f},
            {fr_position[0], .5f, fr_position[1], 1.f},
            {fl_position[0], .5f, fl_position[1], 1.f},
            {fl_position[0], .5f, fl_position[1], 1.f},
            {rr_position[0], .5f, rr_position[1], 1.f},
            {rr_position[0], .5f, rr_position[1], 1.f},
            {rl_position[0], .5f, rl_position[1], 1.f},
            {rl_position[0], .5f, rl_position[1], 1.f},
            {0, .5f, 0, 1.f},
            {0, .5f, 0, 1.f}};

        for(auto& vec : vertices) {
            vec = pm * vec;
        }

        vertices[1] += glm::vec4(frForce[0]*scale, 0.f, -frForce[1]*scale, 1.f);
        vertices[3] += glm::vec4(flForce[0]*scale, 0.f, -flForce[1]*scale, 1.f);
        vertices[5] += glm::vec4(rrForce[0]*scale, 0.f, -rrForce[1]*scale, 1.f);
        vertices[7] += glm::vec4(rlForce[0]*scale, 0.f, -rlForce[1]*scale, 1.f);
        vertices[9] += glm::vec4(acceleration[0]*scale*car_mass, 0.f, -acceleration[1]*scale*car_mass, 1.f);

//        {rl_position[0] + rlForce[0]/scale, .5f, rl_position[1] + rlForce[1]/scale}};

        GLuint vao;
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        GLuint vertex_buffer;
        glGenBuffers(1, &vertex_buffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(vertices[0]), vertices.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, nullptr);

        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
        glDisableVertexAttribArray(0);

        glDeleteBuffers(1, &vertex_buffer);
        glDeleteVertexArrays(1, &vao);
    }

    /// Draws a colored path showing where the racecar has been and how fast it was going
    /// on a scale where red is slow and green is fast.
    void draw_info(glm::mat4 camera, GLint mvpUniform, bool pause) override {
        glm::mat4 mvp(camera);
        glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, reinterpret_cast<float*>(&mvp));


        Float min = 6;
        Float max = 25;

        constexpr int vertex_count = 1000;
        if(trajectory_vao == 0){
            std::vector<ColorVertex> vertices;

            for (int i = 0; i < vertex_count; ++i) {
                Float x = static_cast<Float>(i)/(vertex_count-1);
                Vector2 position = trajectory.path().position(x);
                float value = static_cast<float>(trajectory.get_speed(x));//.path().curvature(x);
                float ratio = static_cast<float>((value - min) / (max - min));
                vertices.emplace_back(glm::vec4(position[0], 0, -position[1], 1),
                                      ratio*glm::vec4(0.f, 1.f, 0, 1.f) + (1-ratio)*glm::vec4(1.f, 0.f, 0.f, 1.f));
            }

            glGenVertexArrays(1, &trajectory_vao);
            glBindVertexArray(trajectory_vao);

            glGenBuffers(1, &trajectory_vbo);
            glBindBuffer(GL_ARRAY_BUFFER, trajectory_vbo);
            glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(vertices[0]), vertices.data(), GL_STATIC_DRAW);

            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);

            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(vertices[0]), (char *) 0);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(vertices[0]), (char *) 16);

            glBindVertexArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);


            glGenVertexArrays(1, &actual_vao);
            glBindVertexArray(actual_vao);

            glGenBuffers(1, &actual_vbo);
            glBindBuffer(GL_ARRAY_BUFFER, actual_vbo);
            glBufferData(GL_ARRAY_BUFFER, vertex_count * sizeof(actual_vertices[0]), nullptr, GL_STATIC_DRAW);

            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);

            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(actual_vertices[0]), (char *) 0);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(actual_vertices[0]), (char *) 16);

            glBindVertexArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);
        }

        glLineWidth(5);
        glBindVertexArray(trajectory_vao);
        glDrawArrays(trajectory.path().is_loop() ? GL_LINE_LOOP : GL_LINE_STRIP, 0, static_cast<GLsizei>(vertex_count));
        glBindVertexArray(actual_vao);

        glLineWidth(2.5f);

        if(!pause){
            Vector2 position(state[NonlinearModel::X], state[NonlinearModel::Z]);
            float value = static_cast<float>(Vector2(state[NonlinearModel::VX], state[NonlinearModel::VZ]).norm());
            float ratio = static_cast<float>((value - min) / (max - min));
            actual_vertices.emplace_back(glm::vec4(position[0], 0, -position[1], 1),
                                         ratio*glm::vec4(0.f, 1.f, 0.f, 1.f) + (1-ratio)*glm::vec4(1.f, 0.f, 0.f, 1.f));
        }

        int actual_vertex_count = std::min(static_cast<int>(actual_vertices.size()), vertex_count);
        glNamedBufferSubData(actual_vbo, 0, actual_vertex_count * sizeof(actual_vertices[0]),
                &actual_vertices[std::max(static_cast<size_t>(0), actual_vertices.size() - actual_vertex_count)]);
        glDrawArrays(GL_LINE_STRIP, 0, actual_vertex_count);
        glBindVertexArray(0);
    }

    /// Shifts an angle into the range [-pi, pi]
    /// \param angle The angle to shift
    /// \return The angle in the [-pi, pi] range
    static constexpr Float angle_near_zero(Float angle){
        angle = std::fmod(angle, 2*PI);
        if(angle > PI)
            return angle - 2*PI;
        return angle;
    }

    /// Main update function that does the Model Predictive Control (MPC)
    void update(std::function<bool(guint key)> keyPressed, MouseInfo& mouse, Float dt) override {
        dt += last_dt;
        while(dt > 0){
            if(next_controller_time <= 0){
                last_x = trajectory.path().nearest_on_path(Vector2(state[Model::X], state[Model::Z]), last_x);

                //TODO start using async futures to run this in parallel for 100ms
                //Check if it's time to do MPC
                if(next_mpc_count == 0){
                    next_mpc_count = mpc_controller_periods;

                    int commands = 5;
                    int state_size = 6;
                    int variables_size = commands + state_size;
                    int n = mpc_states_per_period * mpc_periods_ahead * variables_size;

                    GRBModel model(gurobi);

                    GRBQuadExpr obj;
                    //Initial state to make the code consistent between time steps
                    GRBVar old_x = model.addVar(0, 0, 0.0, GRB_CONTINUOUS);
                    GRBVar old_z = model.addVar(0, 0, 0.0, GRB_CONTINUOUS);
                    GRBVar old_vx = model.addVar(0, 0, 0.0, GRB_CONTINUOUS);
                    GRBVar old_vz = model.addVar(0, 0, 0.0, GRB_CONTINUOUS);
                    GRBVar old_h = model.addVar(0, 0, 0.0, GRB_CONTINUOUS);
                    GRBVar old_w = model.addVar(0, 0, 0.0, GRB_CONTINUOUS);

                    State future_state = state;
                    Command future_command = command;
                    Float future_x = last_x;
                    //Loop through all the time steps, adding variables to the model as we go
                    for (int i = 0; i < mpc_states_per_period * mpc_periods_ahead; ++i) {
                        Float mpc_dt = controller_period * mpc_controller_periods/mpc_states_per_period;
                        Eigen::MatrixXd A = linearized_dynamics(future_state, future_command, mpc_dt);
                        Eigen::MatrixXd B = linearized_command(future_state, future_command, mpc_dt);
                        //TODO consider just constructing states assuming on path
                        future_x = trajectory.path().nearest_on_path(Vector2(future_state[Model::X], future_state[Model::Z]), future_x);
                        future_command = optimal_trajectory_command(future_state, future_command, trajectory, future_x);
                        constexpr int mpc_dynamics_count = 10;
                        for (int j = 0; j < mpc_dynamics_count ; ++j) {
                            future_state = RK4<State, Command>(dynamics, future_state, future_command, mpc_dt/mpc_dynamics_count);
                        }

                        if(i < mpc_commands.size()){
                            mpc_commands[i] = future_command;
                        }

                        //TODO consider optimizing to want less power (obj = -?)

                        //Command variables for this time step
                        GRBVar us = model.addVar(-steer_limit - future_command[FourWheelControl::Steer], steer_limit - future_command[FourWheelControl::Steer], 0.0, GRB_CONTINUOUS);
                        GRBVar ufl = model.addVar(-1.0 - future_command[FourWheelControl::TFL], 1.0 - future_command[FourWheelControl::TFL], 0.0, GRB_CONTINUOUS);
                        GRBVar ufr = model.addVar(-1.0 - future_command[FourWheelControl::TFR], 1.0 - future_command[FourWheelControl::TFR], 0.0, GRB_CONTINUOUS);
                        GRBVar url = model.addVar(-1.0 - future_command[FourWheelControl::TRL], 1.0 - future_command[FourWheelControl::TRL], 0.0, GRB_CONTINUOUS);
                        GRBVar urr = model.addVar(-1.0 - future_command[FourWheelControl::TRR], 1.0 - future_command[FourWheelControl::TRR], 0.0, GRB_CONTINUOUS);

                        //State variables for this time step
                        GRBVar x = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
                        GRBVar z = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
                        GRBVar vx = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
                        GRBVar vz = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
                        GRBVar h = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);
                        GRBVar w = model.addVar(-10000.0, 10000.0, 0.0, GRB_CONTINUOUS);

                        //Linearized Dynamics Constraints
                        model.addConstr(x == us * B(0,0) + ufl * B(0, 1) + ufr * B(0, 2) + url * B(0, 3) + urr * B(0, 4)
                            + old_x * A(0,0) + old_z * A(0, 1) + old_vx * A(0, 2) + old_vz * A(0, 3) + old_h * A(0, 4) + old_w * A(0, 5));
                        model.addConstr(z == us * B(1,0) + ufl * B(1, 1) + ufr * B(1, 2) + url * B(1, 3) + urr * B(1, 4)
                            + old_x * A(1,0) + old_z * A(1, 1) + old_vx * A(1, 2) + old_vz * A(1, 3) + old_h * A(1, 4) + old_w * A(1, 5));
                        model.addConstr(vx == us * B(2,0) + ufl * B(2, 1) + ufr * B(2, 2) + url * B(2, 3) + urr * B(2, 4)
                            + old_x * A(2,0) + old_z * A(2, 1) + old_vx * A(2, 2) + old_vz * A(2, 3) + old_h * A(2, 4) + old_w * A(2, 5));
                        model.addConstr(vz == us * B(3,0) + ufl * B(3, 1) + ufr * B(3, 2) + url * B(3, 3) + urr * B(3, 4)
                            + old_x * A(3,0) + old_z * A(3, 1) + old_vx * A(3, 2) + old_vz * A(3, 3) + old_h * A(3, 4) + old_w * A(3, 5));
                        model.addConstr(h == us * B(4,0) + ufl * B(4, 1) + ufr * B(4, 2) + url * B(4, 3) + urr * B(4, 4)
                            + old_x * A(4,0) + old_z * A(4, 1) + old_vx * A(4, 2) + old_vz * A(4, 3) + old_h * A(4, 4) + old_w * A(4, 5));
                        model.addConstr(w == us * B(5,0) + ufl * B(5, 1) + ufr * B(5, 2) + url * B(5, 3) + urr * B(5, 4)
                            + old_x * A(5,0) + old_z * A(5, 1) + old_vx * A(5, 2) + old_vz * A(5, 3) + old_h * A(5, 4) + old_w * A(5, 5));

                        //Update the previous state error variables
                        old_x = x;
                        old_z = z;
                        old_vx = vx;
                        old_vz = vz;
                        old_h = h;
                        old_w = w;

                        //Determine where we want the racecar to be
                        Vector2 target_position = trajectory.path().position(future_x);
                        Vector2 derivative = trajectory.path().derivative(future_x);
                        Float path_angle = std::atan2(derivative[1], derivative[0]);
                        Vector2 target_velocity = derivative * trajectory.get_speed(future_x)/derivative.norm();
                        Float target_angular_velocity = target_velocity.norm()*trajectory.path().curvature(future_x);

                        //Compute future error without MPC control inputs
                        Float x_error = future_state[NonlinearModel::X] - target_position[0];
                        Float z_error = future_state[NonlinearModel::Z] - target_position[1];
                        Float vx_error = future_state[NonlinearModel::VX] - target_velocity[0];
                        Float vz_error = future_state[NonlinearModel::VZ] - target_velocity[1];
                        Float h_error = angle_near_zero(future_state[NonlinearModel::Heading] - path_angle);
                        Float w_error = future_state[NonlinearModel::AngularVelocity] - target_angular_velocity;

                        //Update the objective function, weighting the position and heading over the velocities.
                        obj += 10*(x + x_error)*(x + x_error) + 10*(z+z_error)*(z+z_error)
                             + (vx + vx_error)*(vx + vx_error) + (vz+vz_error)*(vz+vz_error)
                             + 20*(h+h_error)*(h+h_error) + (w+w_error)*(w+w_error);
                    }

                    // Optimize model

                    model.setObjective(obj);
                    model.set(GRB_IntParam_OutputFlag, 0);
                    model.optimize();

                    //std::cout << x.get(GRB_StringAttr_VarName) << " "
                    //          << x.get(GRB_DoubleAttr_X) << std::endl;
                    //std::cout << y.get(GRB_StringAttr_VarName) << " "
                    //          << y.get(GRB_DoubleAttr_X) << std::endl;
                    //std::cout << z.get(GRB_StringAttr_VarName) << " "
                    //          << z.get(GRB_DoubleAttr_X) << std::endl;

                    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

                    for (int j = 0; j < mpc_commands.size(); ++j) {
                        mpc_commands[j][FourWheelControl::Steer] += static_cast<Float>(
                                model.getVar(state_size + j * variables_size + 0).get(GRB_DoubleAttr_X));
                        //std::cout << mpc_commands[j][FourWheelControl::Steer] << " ";
                        mpc_commands[j][FourWheelControl::TFL] += static_cast<Float>(
                                model.getVar(state_size + j * variables_size + 1).get(GRB_DoubleAttr_X));
                        mpc_commands[j][FourWheelControl::TFR] += static_cast<Float>(
                                model.getVar(state_size + j * variables_size + 2).get(GRB_DoubleAttr_X));
                        mpc_commands[j][FourWheelControl::TRL] += static_cast<Float>(
                                model.getVar(state_size + j * variables_size + 3).get(GRB_DoubleAttr_X));
                        mpc_commands[j][FourWheelControl::TRR] += static_cast<Float>(
                                model.getVar(state_size + j * variables_size + 4).get(GRB_DoubleAttr_X));
                        ///std::cout << model.getVar(state_size + j * variables_size + 0).get(GRB_DoubleAttr_X);
                    }
                }

                //Look up the next command to execute
                command = Command(//optimal_trajectory_command(state, command, trajectory, last_x)
                                mpc_commands[mpc_commands.size() - next_mpc_count]
                                );

                command[FourWheelControl::Steer] = std::clamp(command[FourWheelControl::Steer], -steer_limit, steer_limit);
                command[FourWheelControl::TFL] = std::clamp(command[FourWheelControl::TFL], static_cast<Float>(-1.f), static_cast<Float>(1.f));
                command[FourWheelControl::TFR] = std::clamp(command[FourWheelControl::TFR], static_cast<Float>(-1.f), static_cast<Float>(1.f));
                command[FourWheelControl::TRL] = std::clamp(command[FourWheelControl::TRL], static_cast<Float>(-1.f), static_cast<Float>(1.f));
                command[FourWheelControl::TRR] = std::clamp(command[FourWheelControl::TRR], static_cast<Float>(-1.f), static_cast<Float>(1.f));

                //command = mpc_commands[mpc_commands.size() - next_mpc_count];
                next_mpc_count--;



                constexpr bool keyboard = false;
                //Override the commands with user control inputs
                if(keyboard){
                    //Keyboard control
                    //command[FourWheelControl::Steer] = (keyPressed(GDK_KEY_Left) ? 1.f : 0.f) + (keyPressed(GDK_KEY_Right) ? -1.f : 0.f);
                    //Mouse control
                    //command[FourWheelControl::Steer] = (std::abs(mouse.x) > 1.f ? 0 :
                    //        (std::abs(mouse.x) > 1.f ? std::copysign(1.f, -mouse.x) : -mouse.x))*steer_limit;// - (state[Model::SteeringAngle] / steer_limit);

                    //Vector2 on_path = trajectory.path().position(last_x);
                    //std::cout << last_x << ", (" << on_path[0] << ", " << on_path[1] << "), (" << position[0] << ", " << position[1] << ")" << std::endl;

                    command[FourWheelControl::TFL] = command[FourWheelControl::TFR] = command[FourWheelControl::TRL] = command[FourWheelControl::TRR] =
                            (keyPressed(GDK_KEY_Up) ? 1.f : 0.f) + (keyPressed(GDK_KEY_Down) ? -1.f : 0.f) + 0.f;
                }

                next_controller_time = controller_period;
            }

            state = RK4<State, Command>(dynamics, state, command, time_step);

            next_controller_time -= time_step;
            dt -= time_step;
        }
        last_dt = dt;

//        std::cout << dt << ',' << state[NonlinearModel::VX] << ','<< state[NonlinearModel::VZ] << ','  << state[NonlinearModel::Heading] << std::endl;
    }

    /// Nominal controller that determines the desired force and moment and then
    /// uses gradient descent to find what commands are required to make the car follow the path
    /// \param state The racecar state
    /// \param last_command The previous command
    /// \param trajectory The target trajectory
    /// \param last_x The previous point on the trajectory's path
    /// \return the optimal command to follow the desired trajectory
    static Command optimal_trajectory_command(const State& state, const Command& last_command, const Trajectory& trajectory, const Float& last_x){
        Command command = last_command;

        Vector2 position(state[Model::X], state[Model::Z]);
        Vector2 velocity(state[Model::VX], state[Model::VZ]);
        //position += controller_period * Vector2(state[Model::VX], state[Model::VZ]);
        Float next_x = trajectory.path().nearest_on_path(position + controller_period * velocity, last_x);

        //command = controller(state);


        //Gradient descent tends find a bad angle, because it can achieve 0 cost at
        //most steering angles with the right torque vectoring and it only becomes a problem
        //when the car's heading gets so far off there's no good angle and the car veers off course
        //Two initial steering options before optimization to help with this:
        //1. take direction of tangent and steer so average wheel angle matches
        //1.5 Same as 1. except use velocity instead of heading
        //2. steer so radius of curvature matches current curvature
        //Combining both seems to produce the best results

        //1.
        Vector2 derivative = trajectory.path().derivative(last_x);
        Float path_angle = std::atan2(derivative[1], derivative[0]);
        command[FourWheelControl::Steer] = angle_near_zero(
                + .25f* angle_near_zero(path_angle - state[Model::Heading])
                + .75f* angle_near_zero(path_angle - std::atan2(state[Model::VZ], state[Model::VX])) //1.5
        );

        //2.
        constexpr Float curvature_constant = 1.f;//2.f;//1.2f;
        command[FourWheelControl::Steer] += std::atan(curvature_constant * std::tan(std::asin(wheelbase * trajectory.path().curvature(last_x))));
        command[FourWheelControl::Steer] =  std::clamp(command[FourWheelControl::Steer], -steer_limit, steer_limit);

        Float next_curvature = trajectory.path().curvature(next_x);
        Float last_curvature = trajectory.path().curvature(last_x);
        Float speed_sqr = velocity.squaredNorm();
        Float speed = std::sqrt(speed_sqr);
        Vector2 path_derivative = trajectory.path().derivative(next_x);
        path_derivative.normalize();
        Float max_acceleration = plan_margin * std::sqrt(
                std::pow(mu * gravity * (1.f + aero_accel_coef * speed_sqr), 2.f)
                - std::pow(speed_sqr * (next_curvature + last_curvature)/2.f, 2.f));
        if(std::isnan(max_acceleration))
            max_acceleration = 1;
        Float trajectory_speed = trajectory.path().is_loop() || next_x < 1.f ? trajectory.get_speed(next_x) : 0;
        Vector2 next_velocity = path_derivative * (speed + controller_period *
                                                                   std::clamp((trajectory_speed - speed)/controller_period, -max_acceleration, max_acceleration));
        Vector2 v_now = velocity; // speed * path_derivative b
        Vector2 target_force = car_mass / controller_period * (next_velocity - v_now);
        //std::cout << last_curvature << " " << next_curvature << " ";
        Float max_moment = 1000;
        Float target_moment = std::clamp(moment_of_inertia *
                                         (next_curvature * next_velocity.norm() - last_curvature * speed),
                                         -max_moment, max_moment);
        //command[FourWheelControl::TFL] = command[FourWheelControl::TFR] = command[FourWheelControl::TRL] = command[FourWheelControl::TRR] =
        //        std::clamp((trajectory.get_speed(next_x) - speed)/controller_period/max_acceleration, -1.f, 1.f);
        return optimal_command(state, command, target_force, target_moment);
        //command = optimal_command(state, command, {}, 100 * std::sin(state[Model::Heading]));
    }

    const Trajectory& get_trajectory() override {
        return trajectory;
    }

private:
    //Parameters to tune MPC
    static constexpr int mpc_controller_periods = 20;
    static constexpr int mpc_periods_ahead = 10;//Number of periods to predict ahead
    static constexpr int mpc_states_per_period = 20;//Number of time steps to simulate per MPC period
    static constexpr Float controller_period = 1/100.f;
    static constexpr Float time_step = 1/6000.f;

    std::array<Command, mpc_states_per_period> mpc_commands;

    GRBEnv gurobi;

    Float next_controller_time = 0;
    int next_mpc_count = 0;

    Float last_dt = 0;
    Float last_x = 0;
    State state;
    Command command;
    Trajectory trajectory;
    GLuint trajectory_vao = 0;
    GLuint trajectory_vbo = 0;
    GLuint actual_vao = 0;
    GLuint actual_vbo = 0;
    std::vector<ColorVertex> actual_vertices;

    static Vector2 flForce;
    static Vector2 frForce;
    static Vector2 rlForce;
    static Vector2 rrForce;
    static Vector2 acceleration;

    /// Computes the net acceleration and moment on the racecar from all the tire forces
    /// \param state The state of the racecar
    /// \param c The current command
    /// \return A pair of the acceleration vector and moment
    static std::pair<Vector2, Float> accelerations(const State &state, const Command &c){
        Vector2 velocity(state[Model::VX], state[Model::VZ]);
        Rotation2D car_rotation(state[Model::Heading]);
        Vector2 direction(std::cos(state[Model::Heading]), std::sin(state[Model::Heading]));
        Vector2 front_tire_direction(std::cos(state[Model::Heading] + state[Model::SteeringAngle]),
                                             std::sin(state[Model::Heading] + state[Model::SteeringAngle]));

        Vector2 fr_rotated = car_rotation*fr_position;
        Vector2 fl_rotated = car_rotation*fl_position;
        Vector2 rr_rotated = car_rotation*rr_position;
        Vector2 rl_rotated = car_rotation*rl_position;

        //TODO AERO can also apply lateral forces
        //TODO get actual CL. Now just assuming drives upside down at 30m/s
        const Float average_wheel_weight = gravity * car_mass/4 * (1 + velocity.squaredNorm() * aero_accel_coef);
        flForce = TireForce(c[FourWheelControl::TFL], mu * average_wheel_weight * state[Model::FrontSuspension] * state[Model::LeftSuspension],
                            -velocity - state[Model::AngularVelocity] * Vector2(-fl_rotated[1], fl_rotated[0]), front_tire_direction);//state[Model::Heading] + state[Model::SteeringAngle]);
        frForce = TireForce(c[FourWheelControl::TFR], mu * average_wheel_weight * state[Model::FrontSuspension] * (2-state[Model::LeftSuspension]),
                            -velocity - state[Model::AngularVelocity] * Vector2(-fr_rotated[1], fr_rotated[0]), front_tire_direction);//state[Model::Heading] + state[Model::SteeringAngle]);
        rlForce = TireForce(c[FourWheelControl::TRL], mu * average_wheel_weight* (2-state[Model::FrontSuspension]) * state[Model::LeftSuspension],
                            -velocity - state[Model::AngularVelocity] * Vector2(-rl_rotated[1], rl_rotated[0]), direction);//state[Model::Heading]);
        rrForce = TireForce(c[FourWheelControl::TRR], mu * average_wheel_weight * (2-state[Model::FrontSuspension]) * (2-state[Model::LeftSuspension]),
                            -velocity - state[Model::AngularVelocity] * Vector2(-rr_rotated[1], rr_rotated[0]), direction);// state[Model::Heading]);
        Vector2 acceleration = (flForce + frForce + rlForce + rrForce)/car_mass;
        //if(std::isnan(acceleration[0])){
        //    std::cout << acceleration << std::endl;
        //}

        Float angular_acceleration = (cross2(fl_rotated, flForce) + cross2(fr_rotated, frForce) +
                                      cross2(rl_rotated, rlForce) + cross2(rr_rotated, rrForce))/ moment_of_inertia;
        return std::make_pair(acceleration, angular_acceleration);
    }

    /// Dynamics derivative function
    /// \param state Initial state
    /// \param c Command
    /// \return The derivative of the state
    static State dynamics(State state, const Command &c){
        state[Model::SteeringAngle] = c[FourWheelControl::Steer];

        auto a = accelerations(state, c);
        acceleration = a.first;
        Float angular_acceleration = a.second;

        State derivative;
        derivative[Model::X] = state[Model::VX];
        derivative[Model::Z] = state[Model::VZ];
        derivative[Model::VX] = acceleration[0];
        derivative[Model::VZ] = acceleration[1];
        derivative[Model::Heading] = state[Model::AngularVelocity];
        derivative[Model::AngularVelocity] = angular_acceleration;
        if((state[Model::SteeringAngle] > steer_limit && c[FourWheelControl::Steer] > 0) ||
                (state[Model::SteeringAngle] < -steer_limit && c[FourWheelControl::Steer] < 0)){
            derivative[Model::SteeringAngle] = 0;
        }
        else{
            derivative[Model::SteeringAngle] = c[FourWheelControl::Steer];
        }
        Vector2 acceleration_car_frame = Rotation2D(-state[Model::Heading]) * acceleration;
        //Decided to leave out suspension, because it didn't affect the simulation much and it slowed things down.
        //Probably worth adding back at some point.
        derivative[Model::FrontSuspension] = 0;//-acceleration_car_frame[0] * center_of_mass_height/ wheelbase - (state[Model::FrontSuspension] - 1)/suspension_longitudinal_time_constant;
        derivative[Model::LeftSuspension] = 0;//acceleration_car_frame[1] * center_of_mass_height/ wheelbase - (state[Model::LeftSuspension] - 1)/suspension_lateral_time_constant;
        return derivative;
    }

    /// Computes the linearized command matrix by computing the gradients of each control input
    /// \param state State to linearize at
    /// \param c Command to linearize about
    /// \param dt Time step size
    /// \return The linearized command matrix
    static Eigen::MatrixXd linearized_command(State state, const Command &c, Float dt){
        state[Model::SteeringAngle] = c[FourWheelControl::Steer];

        Vector2 velocity(state[Model::VX], state[Model::VZ]);
        Rotation2D car_rotation(state[Model::Heading]);
        Vector2 direction(std::cos(state[Model::Heading]), std::sin(state[Model::Heading]));

        Vector2 fr_rotated = car_rotation*fr_position;
        Vector2 fl_rotated = car_rotation*fl_position;
        Vector2 rr_rotated = car_rotation*rr_position;
        Vector2 rl_rotated = car_rotation*rl_position;

        const Float average_wheel_weight = gravity * car_mass/4 * (1 + velocity.squaredNorm() * aero_accel_coef);

        Float flWeight = mu * average_wheel_weight * state[Model::FrontSuspension] * state[Model::LeftSuspension];
        Float frWeight = mu * average_wheel_weight * state[Model::FrontSuspension] * (2-state[Model::LeftSuspension]);
        Float rlWeight = mu * average_wheel_weight * (2-state[Model::FrontSuspension]) * state[Model::LeftSuspension];
        Float rrWeight = mu * average_wheel_weight * (2-state[Model::FrontSuspension]) * (2-state[Model::LeftSuspension]);

        auto flVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-fl_rotated[1], fl_rotated[0]);
        auto frVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-fr_rotated[1], fr_rotated[0]);
        auto rlVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-rl_rotated[1], rl_rotated[0]);
        auto rrVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-rr_rotated[1], rr_rotated[0]);

        Vector2 front_tire_direction(std::cos(state[Model::Heading] + c[FourWheelControl::Steer]),
                                             std::sin(state[Model::Heading] + c[FourWheelControl::Steer]));

        //No great option, because tire dynamics are very nonlinear.
        constexpr Float gradient = .1;
        Vector2 front_tire_direction_gradient(
                std::cos(state[Model::Heading] + c[FourWheelControl::Steer] + gradient),
                std::sin(state[Model::Heading] + c[FourWheelControl::Steer] + gradient));

        Float flCommand = c[FourWheelControl::TFL];
        Float frCommand = c[FourWheelControl::TFR];
        Float rlCommand = c[FourWheelControl::TRL];
        Float rrCommand = c[FourWheelControl::TRR];

        Float flCommand_gradient = c[FourWheelControl::TFL] + gradient;
        Float frCommand_gradient = c[FourWheelControl::TFR] + gradient;
        Float rlCommand_gradient = c[FourWheelControl::TRL] + gradient;
        Float rrCommand_gradient = c[FourWheelControl::TRR] + gradient;

        Vector2 flForce = TireForce(flCommand, flWeight, flVelocity, front_tire_direction);
        Vector2 frForce = TireForce(frCommand, frWeight, frVelocity, front_tire_direction);
        Vector2 rlForce = TireForce(rlCommand, rlWeight, rlVelocity, direction);
        Vector2 rrForce = TireForce(rrCommand, rrWeight, rrVelocity, direction);


        Vector2 flForce_steering_gradient = TireForce(flCommand, flWeight, flVelocity, front_tire_direction_gradient);
        Vector2 frForce_steering_gradient = TireForce(frCommand, frWeight, frVelocity, front_tire_direction_gradient);
        Vector2 flForce_gradient = TireForce(flCommand_gradient, flWeight, flVelocity, front_tire_direction);
        Vector2 frForce_gradient = TireForce(frCommand_gradient, frWeight, frVelocity, front_tire_direction);
        Vector2 rlForce_gradient = TireForce(rlCommand_gradient, rlWeight, rlVelocity, direction);
        Vector2 rrForce_gradient = TireForce(rrCommand_gradient, rrWeight, rrVelocity, direction);


        acceleration = (flForce + frForce + rlForce + rrForce)/car_mass;
        //if(std::isnan(acceleration[0])){
        //    std::cout << acceleration << std::endl;
        //}

        Float moment = cross2(fl_rotated, flForce) + cross2(fr_rotated, frForce) +
                       cross2(rl_rotated, rlForce) + cross2(rr_rotated, rrForce);


        // U = [Steer, FL, FR, RL, RR]
        // X = [X, Z, VX, VZ, Heading, Angular Velocity]

        Eigen::MatrixXd B(6,5);
        B << //X
             0, 0, 0, 0, 0,
             //Z
             0, 0, 0, 0, 0,
             //VX
             (flForce_steering_gradient - flForce + frForce_steering_gradient - frForce)[0]/car_mass*dt,
             (flForce_gradient - flForce)[0]/car_mass*dt,
             (frForce_gradient - frForce)[0]/car_mass*dt,
             (rlForce_gradient - rlForce)[0]/car_mass*dt,
             (rrForce_gradient - rrForce)[0]/car_mass*dt,
             //VZ
             (flForce_steering_gradient - flForce + frForce_steering_gradient - frForce)[1]/car_mass*dt,
             (flForce_gradient - flForce)[1]/car_mass*dt,
             (frForce_gradient - frForce)[1]/car_mass*dt,
             (rlForce_gradient - rlForce)[1]/car_mass*dt,
             (rrForce_gradient - rrForce)[1]/car_mass*dt,
             //Heading
             0, 0, 0, 0, 0,
             //Angular Velocity
             (cross2(fl_rotated, flForce_steering_gradient - flForce) + cross2(fr_rotated, frForce_steering_gradient - frForce))/ moment_of_inertia*dt,
              cross2(fl_rotated, flForce_gradient - flForce)/ moment_of_inertia*dt,
              cross2(fr_rotated, frForce_gradient - frForce)/ moment_of_inertia*dt,
              cross2(rl_rotated, rlForce_gradient - rlForce)/ moment_of_inertia*dt,
              cross2(rr_rotated, rrForce_gradient - rrForce)/ moment_of_inertia*dt;

        return B/gradient;
    }

    /// Computes the linearized dynamics matrix by computing the gradients of each control input
    /// \param state State to linearize at
    /// \param c Command to linearize about
    /// \param dt Time step size
    /// \return The linearized dynamics matrix
    static Eigen::MatrixXd linearized_dynamics(State state, const Command &c, Float dt){
        //No great option, because tire dynamics are very nonlinear.
        constexpr Float gradient = 1.f;
        constexpr Float heading_gradient = 1.f;

        auto nominal = accelerations(state, c);

        Float old_vx = state[Model::VX];
        state[Model::VX] += gradient;
        auto gradient_vx = accelerations(state, c);
        Float daxdvx = (gradient_vx.first[0] - nominal.first[0])/gradient;
        Float dazdvx = (gradient_vx.first[1] - nominal.first[1])/gradient;
        Float daldvx = (gradient_vx.second - nominal.second)/gradient;
        state[Model::VX] = old_vx;

        Float old_vz = state[Model::VZ];
        state[Model::VZ] += gradient;
        auto gradient_vz = accelerations(state, c);
        Float daxdvz = (gradient_vz.first[0] - nominal.first[0])/gradient;
        Float dazdvz = (gradient_vz.first[1] - nominal.first[1])/gradient;
        Float daldvz = (gradient_vz.second - nominal.second)/gradient;
        state[Model::VZ] = old_vz;

        Float old_h = state[Model::Heading];
        state[Model::Heading] += heading_gradient;
        auto gradient_h = accelerations(state, c);
        Float daxdh = angle_near_zero(gradient_h.first[0] - nominal.first[0])/heading_gradient;
        Float dazdh = angle_near_zero(gradient_h.first[1] - nominal.first[1])/heading_gradient;
        Float daldh = angle_near_zero(gradient_h.second - nominal.second)/heading_gradient;
        state[Model::Heading] = old_h;

        Float old_w = state[Model::AngularVelocity];
        state[Model::AngularVelocity] += heading_gradient;
        auto gradient_w = accelerations(state, c);
        Float daxdw = (gradient_w.first[0] - nominal.first[0])/heading_gradient;
        Float dazdw = (gradient_w.first[1] - nominal.first[1])/heading_gradient;
        Float daldw = (gradient_w.second - nominal.second)/heading_gradient;
        state[Model::AngularVelocity] = old_w;

        //X acceleration ax, Z acceleration az, Angular Acceleration al
        // X = [X (x), Z (z), VX (vx), VZ (vz), Heading (h), Angular Velocity (w)]
        Eigen::MatrixXd A(6,6);
        A <<//X
            1, 0, dt, 0, 0, 0,
            //Z
            0, 1, 0, dt, 0, 0,
            //VX
            0, 0, 1 + daxdvx * dt, daxdvz * dt, daxdh * dt, daxdw * dt,
            //VZ
            0, 0, dazdvx * dt, 1 + dazdvz * dt, dazdh * dt, dazdw * dt,
            //Heading
            0, 0, 0, 0, 1, dt,
            //Angular Velocity
            0, 0, daldvx * dt, daldvz * dt, daldh * dt, 1 + daldw * dt;

        return A;
    }

    /// Helper for optimal_trajectory_command that does the gradient descent
    /// \param state The racecar state
    /// \param guess Initial command guess
    /// \param target_force The desired force
    /// \param target_moment The desired moment
    /// \param iterations Number of gradient descent iterations to perform
    /// \param gamma Parameter to tune gradient descent speed
    /// \return The command that mimimizes the cost
    static Command optimal_command(State state, Command guess, const Vector2 &target_force, Float target_moment, int iterations = 50, Float gamma = .0001){
        Vector2 velocity(state[Model::VX], state[Model::VZ]);
        Rotation2D car_rotation(state[Model::Heading]);
        Vector2 direction(std::cos(state[Model::Heading]), std::sin(state[Model::Heading]));

        Vector2 fr_rotated = car_rotation*fr_position;
        Vector2 fl_rotated = car_rotation*fl_position;
        Vector2 rr_rotated = car_rotation*rr_position;
        Vector2 rl_rotated = car_rotation*rl_position;

        const Float average_wheel_weight = gravity * car_mass/4 * (1 + velocity.squaredNorm() * aero_accel_coef);

        Float flWeight = mu * average_wheel_weight * state[Model::FrontSuspension] * state[Model::LeftSuspension];
        Float frWeight = mu * average_wheel_weight * state[Model::FrontSuspension] * (2-state[Model::LeftSuspension]);
        Float rlWeight = mu * average_wheel_weight * (2-state[Model::FrontSuspension]) * state[Model::LeftSuspension];
        Float rrWeight = mu * average_wheel_weight * (2-state[Model::FrontSuspension]) * (2-state[Model::LeftSuspension]);

        auto flVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-fl_rotated[1], fl_rotated[0]);
        auto frVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-fr_rotated[1], fr_rotated[0]);
        auto rlVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-rl_rotated[1], rl_rotated[0]);
        auto rrVelocity = -velocity - state[Model::AngularVelocity] * Vector2(-rr_rotated[1], rr_rotated[0]);

        //The cost function
        auto cost = [&](Float flCommand, Float frCommand, Float rlCommand, Float rrCommand, Vector2 &front_tire_direction,
                Vector2 &flForce, Vector2 &frForce, Vector2 &rlForce, Vector2 &rrForce){
            Vector2 force = flForce + frForce + rlForce + rrForce;
            Float moment = cross2(fl_rotated, flForce) + cross2(fr_rotated, frForce) +
                     cross2(rl_rotated, rlForce) + cross2(rr_rotated, rrForce);

            //Experimented some with other parameters. Definitely room for improvement here.
            return (force - target_force).squaredNorm() //+ 10*std::pow(moment - target_moment, 2)
                 //+ flVelocity.dot(front_tire_direction) * flCommand + frVelocity.dot(front_tire_direction) * frCommand
                 //+ rlVelocity.dot(direction) * rlCommand + rrVelocity.dot(direction) * rrCommand
                 ;
        };

        //std::cout << target_force[0] << ", " << target_force[1] << ", " << target_moment << " ";
        for (int i = 0; i < iterations; ++i) {
            Vector2 front_tire_direction(std::cos(state[Model::Heading] + guess[FourWheelControl::Steer]),
                                                 std::sin(state[Model::Heading] + guess[FourWheelControl::Steer]));
            constexpr Float gradient = .0001;
            Vector2 front_tire_direction_gradient(
                    std::cos(state[Model::Heading] + guess[FourWheelControl::Steer] + gradient),
                    std::sin(state[Model::Heading] + guess[FourWheelControl::Steer] + gradient));

            Float flCommand = guess[FourWheelControl::TFL];
            Float frCommand = guess[FourWheelControl::TFR];
            Float rlCommand = guess[FourWheelControl::TRL];
            Float rrCommand = guess[FourWheelControl::TRR];

            Float flCommand_gradient = guess[FourWheelControl::TFL] + gradient;
            Float frCommand_gradient = guess[FourWheelControl::TFR] + gradient;
            Float rlCommand_gradient = guess[FourWheelControl::TRL] + gradient;
            Float rrCommand_gradient = guess[FourWheelControl::TRR] + gradient;

            Vector2 flForce = TireForce(flCommand, flWeight, flVelocity, front_tire_direction);
            Vector2 frForce = TireForce(frCommand, frWeight, frVelocity, front_tire_direction);
            Vector2 rlForce = TireForce(rlCommand, rlWeight, rlVelocity, direction);
            Vector2 rrForce = TireForce(rrCommand, rrWeight, rrVelocity, direction);


            Vector2 flForce_steering_gradient = TireForce(flCommand, flWeight, flVelocity, front_tire_direction_gradient);
            Vector2 frForce_steering_gradient = TireForce(frCommand, frWeight, frVelocity, front_tire_direction_gradient);
            Vector2 flForce_gradient = TireForce(flCommand_gradient, flWeight, flVelocity, front_tire_direction);
            Vector2 frForce_gradient = TireForce(frCommand_gradient, frWeight, frVelocity, front_tire_direction);
            Vector2 rlForce_gradient = TireForce(rlCommand_gradient, rlWeight, rlVelocity, direction);
            Vector2 rrForce_gradient = TireForce(rrCommand_gradient, rrWeight, rrVelocity, direction);

            Float guess_cost = cost(flCommand, frCommand, rlCommand, rrCommand, front_tire_direction,
                                    flForce, frForce, rlForce, rrForce);

            if(guess_cost < 1)
                break;

            Command gradients(std::valarray<Float>(guess_cost, static_cast<size_t>(FourWheelControl::Length)));
            gradients[FourWheelControl::Steer] -= cost(flCommand, frCommand, rlCommand, rrCommand, front_tire_direction_gradient,
                                       flForce_steering_gradient, frForce_steering_gradient, rlForce, rrForce);
            gradients[FourWheelControl::TFL] -= cost(flCommand_gradient, frCommand, rlCommand, rrCommand, front_tire_direction,
                                 flForce_gradient, frForce, rlForce, rrForce);
            gradients[FourWheelControl::TFR] -= cost(flCommand, frCommand_gradient, rlCommand, rrCommand, front_tire_direction,
                                 flForce, frForce_gradient, rlForce, rrForce);
            gradients[FourWheelControl::TRL] -= cost(flCommand, frCommand, rlCommand_gradient, rrCommand, front_tire_direction,
                                 flForce, frForce, rlForce_gradient, rrForce);
            gradients[FourWheelControl::TRR] -= cost(flCommand, frCommand, rlCommand, rrCommand_gradient, front_tire_direction,
                                 flForce, frForce, rlForce, rrForce_gradient);

            Float oldSteer = guess[FourWheelControl::Steer];
            guess += gradients * gamma;
            guess[FourWheelControl::Steer] = std::clamp(guess[FourWheelControl::Steer], -steer_limit, steer_limit);
            guess[FourWheelControl::TFL] = std::clamp(guess[FourWheelControl::TFL], static_cast<Float>(-1.f), static_cast<Float>(1.f));
            guess[FourWheelControl::TFR] = std::clamp(guess[FourWheelControl::TFR], static_cast<Float>(-1.f), static_cast<Float>(1.f));
            guess[FourWheelControl::TRL] = std::clamp(guess[FourWheelControl::TRL], static_cast<Float>(-1.f), static_cast<Float>(1.f));
            guess[FourWheelControl::TRR] = std::clamp(guess[FourWheelControl::TRR], static_cast<Float>(-1.f), static_cast<Float>(1.f));
            //std::cout << i << " " << guess_cost << " ";
        }
        //std::cout << " " << guess_cost << std::endl;
        return guess;
    }

    /*static Command controller(const State &st){
        Command c;
        return c;
    }*/
};

std::unique_ptr<Simulation> create_simulation() {
    //Test functions
    //Vector2 v1 = TireForce(0, 47, Vector2(-1, 0), Vector2(1, 0));
    //Vector2 v2 = TireForce(0, 47, Vector2(0, -1), Vector2(0, 1));
    //Vector2 v3 = TireForce(1, 47, Vector2(-1, 0), Vector2(1, 0));
    //Vector2 v4 = TireForce(-1, 47, Vector2(0, -1), Vector2(0, 1));
    //Vector2 v5 = TireForce(0, 47, Vector2(-1, 0), Vector2(0, 1));
    //Vector2 v6 = TireForce(1, 47, Vector2(-1, 0), Vector2(0, 1));

    //NamedVector<CircleModel> state;
    //state[CircleModel::X] = -9.25f;
    //state[CircleModel::Z] = 0;
    //state[CircleModel::VX] = 0;
    //state[CircleModel::VZ] = 11.6f;
    //return std::unique_ptr<Simulation>(new CircleSimulation(std::move(state)));

    //Three paths currently: Circle, Oval and Squiggle

    Arc circle(Vector2(), 15, 0, 2*PI);

    Float y_scale = 3;
    Float length = 30;
    Float width = 10;
    auto oval = std::make_shared<Path>(std::vector<std::shared_ptr<PathSegment>>{
        std::make_shared<Line>(Vector2(), Vector2(0, length)),
        std::make_shared<Spline>(Polynomial(width * (Eigen::VectorXf(4) << -2, 3, 0, 0).finished()),
                                 Polynomial((Eigen::VectorXf(3) << -width * y_scale, width * y_scale, length).finished()), 1024),
        std::make_shared<Line>(Vector2(width, length), Vector2(width, 0)),
        std::make_shared<Spline>(Polynomial(width * (Eigen::VectorXf(4) << 2, -3, 0, 1).finished()),
                                 Polynomial((Eigen::VectorXf(3) << width * y_scale, -width * y_scale, 0).finished()), 1024),
    });

    auto squiggle = std::make_shared<Spline>(
            Polynomial(.15*(Eigen::VectorXf(6) << 2500/3.f, 0, -500/3.f, 0, 10, 0).finished()),
            Polynomial((Eigen::VectorXf(3) << 0, 30, 0).finished()), 1024);

    auto& path = oval;
    //Initialize state to something reasonable
    NamedVector<NonlinearModel> state;
    state[NonlinearModel::X] = path->position(0)[0]+1.f;
    state[NonlinearModel::Z] = path->position(0)[1];
    state[NonlinearModel::VX] = 0;
    state[NonlinearModel::VZ] = 0;
    state[NonlinearModel::Heading] = std::atan2(path->derivative(0)[1], path->derivative(0)[0]);
    state[NonlinearModel::AngularVelocity] = 0;
    state[NonlinearModel::SteeringAngle] = 0;
    state[NonlinearModel::FrontSuspension] = 1;
    state[NonlinearModel::LeftSuspension] = 1;
    return std::unique_ptr<Simulation>(new NonlinearSimulation(std::move(state), path));
}

Vector2 NonlinearSimulation::flForce;
Vector2 NonlinearSimulation::frForce;
Vector2 NonlinearSimulation::rlForce;
Vector2 NonlinearSimulation::rrForce;
Vector2 NonlinearSimulation::acceleration;
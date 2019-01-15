#pragma once

#include <map>
#include <memory>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <iostream>

//typedefs to easily switch between floats and doubles for the simulation
typedef Eigen::Vector2d Vector2;
typedef Eigen::Array2d Array2;
typedef double Float;
typedef Eigen::Rotation2Dd Rotation2D;


/// An immutable continuous and differentiable set of points parameterized by a coordinate x in the range [0,1]
/// The behavior of functions with coordinates outside this range is unspecified
class PathSegment{
public:
    /// Gets the position at the specified coordinate
    virtual Vector2 position(Float x) const = 0;
    /// Gets the derivative at the specified coordinate
    virtual Vector2 derivative(Float x) const = 0;
    /// Gets the second derivative at the specified coordinate
    virtual Vector2 second_derivative(Float x) const = 0;
    /// Gets the distance along the path at the specified coordinate
    virtual Float distance(Float x) const = 0;
    /// Gets the curvature at the specified coordinate
    virtual Float curvature(Float x) const = 0;

    /// Finds the coordinate of the path at the point nearest the specified point.
    /// Uses gradient descent, so the initial reference guess should be good
    /// \param point The point to start at
    /// \param x_ref The coordinate the start searching at
    /// \param gamma Can be tuned to improve the performance of gradient descent
    /// \param iterations The number of gradient descent iterations
    /// \return The coodinate on the path
    Float nearest_on_path(Vector2 point, Float x_ref = 0.5f, Float gamma = .00001f, unsigned iterations = 100) const;

    /// Finds a point a certain distance along the path from another point
    /// \param x The point to start from
    /// \param distance_away The distance to the point we are looking for (may be negative)
    /// \param dx0 The initial coordinate step size. Should only affect convergence time
    /// \param max_error The maximum distance error
    /// \return The coordinate of the located point
    Float next_point(Float x, Float distance_away, Float dx0 = .00001, Float max_error = .0001) const;

    /// Checks whether the path starts and ends at the same point
    /// \return true if it is a loop
    bool is_loop() const;

    virtual ~PathSegment() = default;
};

/// Determines and stores the optimal speeds along a given path
class Trajectory{
public:
    /// Calculates a trajectory for the given path. Units of all the parameters must be compatible. O(1/dt)
    /// \param path A pointer to the path
    /// \param dt The time step size to use.
    /// \param power_to_mass The car's power to mass ratio.
    /// \param max_acceleration The traction limit of the vehicle. Assumed to be the same in every direction.
    /// \param aero_accel_coef downforce/(mass*velocity^2)
    explicit Trajectory(std::shared_ptr<PathSegment> path, Float dt,
                        Float power_to_mass, Float max_acceleration, Float aero_accel_coef);

    /// Uses linear interpolation to find the speed at a given point. O(log(1/dt))
    /// \param x The coordinate along the path
    /// \return The speed at the point
    Float get_speed(Float x) const;

    /// Note: this function is approximate
    /// \return The total time the trajectory takes
    Float total_time() const;

    /// \return The path the trajectory follows
    const PathSegment& path() const;

private:
    /// A pointer to the path the trajectory follows. Doesn't really need to be stored here, but it's convenient
    std::shared_ptr<PathSegment> _path;
    /// A mapping from points along path to speeds.
    std::map<Float, Float> _speeds;
    /// The step size
    Float _dt;
};

///A PathSegment made up of multiple other PathSegments.
///Note that it reparameterizes them to [0,1/n), [1/n,2/n) ... [(n-1)/n,1] if there are n segments
class Path : public PathSegment{
public:
    /// \param segments The segments the path. Must be nonempty and the segments should be continuous.
    explicit Path(std::vector<std::shared_ptr<PathSegment>>&& segments);

    /// Copy constructor
    Path(const Path& path);

    /// Move constructor
    Path(Path&& path) noexcept;

    Vector2 position(Float x) const override;
    Vector2 derivative(Float x) const override;
    Vector2 second_derivative(Float x) const override;
    Float distance(Float x) const override;
    Float curvature(Float x) const override;

private:
    ///All the segments that made up this Path
    std::vector<std::shared_ptr<PathSegment>> _segments;
    ///Store precomputed distances for O(1) lookups
    std::vector<Float> _distance_sums;

    /// Helper that executes a function on the correct subsegment with the correcteed coordinate
    /// \tparam TReturn the function return type
    /// \param f The PathSegment function to call that accepts a float
    /// \param x The coordinate on this Path.
    /// \return the result of the function call
    template<typename TReturn>
    TReturn evaluate(TReturn (PathSegment::* f)(Float) const, Float x) const{
        if(x >= 1)
            return ((_segments.end()-1)->get()->*f)(1.f);
        if(x <= 0)
            return (_segments.begin()->get()->*f)(0.f);
        Float mid_index = x*_segments.size();
        int segment_index = static_cast<int>(mid_index);
        return (_segments[segment_index].get()->*f)(mid_index - segment_index);
    }
};

/// A line segment between two points
class Line : public PathSegment{
public:
    /// Creates a line segment from the end points
    /// \param start The intial position (x=0)
    /// \param end The final position (x=1)
    Line(Vector2 start, Vector2 end);

    Vector2 position(Float x) const override;
    Vector2 derivative(Float x) const override;
    Vector2 second_derivative(Float x) const override;
    Float distance(Float x) const override;
    Float curvature(Float x) const override;

private:
    Vector2 _start;
    Vector2 _end;
};

/// An arc. May be a circle or even multiple loops
class Arc : public PathSegment{
public:
    /// Creates an arc from a center, radius and angles
    /// \param center The center of the arc
    /// \param radius The radius
    /// \param start_angle The angle from the center to the first point. Does not have to be in [0,2pi].
    /// \param end_angle The angle from the center to the last point. Does not have to be in [0,2pi].
    Arc(Vector2 center, Float radius, Float start_angle, Float end_angle);

    Float get_angle(Float x) const;
    Vector2 position(Float x) const override;
    Vector2 derivative(Float x) const override;
    Vector2 second_derivative(Float x) const override;
    Float distance(Float x) const override;
    Float curvature(Float x) const override;

private:
    Vector2 _center;
    Float _radius;
    Float _start_angle;
    Float _end_angle;
};

/// Stores a polynomial with a list of coefficients
class Polynomial{
public:
    /// Creates a polynomial with the specified coefficients
    /// \param _data A vector of coefficients
    explicit Polynomial(Eigen::VectorXf _data) : _data(std::move(_data)) {}

    /// Evaluates the polynomial at a given point after taking a certain number of derivatives
    /// \tparam derivative The number of derivatives to take. 0 means no derivatives
    /// \param x The point to compute the value at
    /// \return The computed value
    template<unsigned derivative>
    Float evaluate(Float x) const{
        long long int degree = _data.size() - 1;
        Float derivative_factor = 1;
        for (int j = 0; j < derivative; ++j) {
            derivative_factor *= degree - j;
        }
        Float total = 0;
        for (int i = 0; i < 1 + degree - derivative; ++i) {
            total = x*total + derivative_factor * _data[i];
            derivative_factor *= static_cast<Float>(degree - i - derivative)/(degree - i);
        }
        return  total;
    }

private:
    Eigen::VectorXf _data;
};

/// A PathSegment with coordinates defined by polynomials
class Spline : public PathSegment{
public:
    /// Creates a spline from the specified
    /// \param xpoly The polynomial for x coordiantes
    /// \param ypoly The polynomial for y coordinates
    /// \param distance_lookup_size The size of the distance lookup table. distance is O(log(distance_lookup_size))
    Spline(Polynomial xpoly, Polynomial ypoly, unsigned int distance_lookup_size = 1024);

    Vector2 position(Float x) const override;
    Vector2 derivative(Float x) const override;
    Vector2 second_derivative(Float x) const override;
    Float distance(Float x) const override;
    Float curvature(Float x) const override;

private:
    Polynomial _xpoly;
    Polynomial _ypoly;
    std::vector<Float> distance_lookup;
};

//Not currently used, but may be useful in the future.
// class RotateTranslateSpline : public Spline{
// public:
//     RotateTranslateSpline(const Polynomial &xpoly, const Polynomial &ypoly, unsigned int distance_lookup_size,
//                           Vector2 translation, Float angle)
//         : Spline(xpoly, ypoly, distance_lookup_size),
//           _translation(std::move(translation)), _rotation(angle) {}
//
//     Vector2 position(Float x) const override {
//         return _translation + _rotation * Spline::position(x);
//     }
//
//     Vector2 derivative(Float x) const override {
//         return _rotation * Spline::derivative(x);
//     }
//
//     Vector2 second_derivative(Float x) const override {
//         return _rotation * Spline::second_derivative(x);
//     }
//
// private:
//     Vector2 _translation;
//     Rotation2D _rotation;
// };
#include "Path.hpp"

Float PathSegment::nearest_on_path(Vector2 point, Float x_ref, Float gamma, unsigned int iterations) const {
    Float x = x_ref;
    bool loop = is_loop();
    for (int i = 0; i < iterations; ++i) {
        Vector2 next = position(x);
        Vector2 tangent = derivative(x);

        x += (point - next).dot(tangent) * gamma;

        if(x > 1)
            x = loop ? 0 : 1;
        if(x < 0)
            x = loop ? 1 : 0;
    }
    return x;
}

Float PathSegment::next_point(Float x, Float distance_away, Float dx0, Float max_error) const {
    Float start_distance = distance(x);
    Float dx = std::copysign(dx0, distance_away);
    while((distance(x) - start_distance)/distance_away < 1){
        if(dx > 0 ? x > 1 : x < 0)
            return dx > 0 ? 1 : 0;
        dx *= 2;
        x += dx;
    }

    Float error;
    while(max_error < std::abs(error = distance(x) - start_distance - distance_away)){
        dx *= .5f;
        x -= std::copysign(dx, error);
    }

    return x;
}

bool PathSegment::is_loop() const {
    return (position(1.f) - position(0)).norm() < .01;
}

Trajectory::Trajectory(std::shared_ptr<PathSegment> path, Float dt, Float power_to_mass, Float max_acceleration,
                       Float aero_accel_coef)
        : _path(std::move(path)),
          _dt(dt){

    bool loop = _path->is_loop();

    //Forward accelerating velocity pass
    {
        Float last_x = 0;
        Float last_speed = 20; //Start at 20m/s. This will be recalculated, so isn't that important.
        _speeds[last_x] = last_speed;
        auto itr = _speeds.begin();
        while(true){
            Float last_x_in_range = last_x > 1 ? last_x - 1 : last_x;
            Float curvature = std::abs(_path->curvature(last_x_in_range));
            Float v2 = std::pow(last_speed, 2.f);
            Float lateral_accel = v2 * curvature;
            Float max_accel = max_acceleration * (1+aero_accel_coef*v2);
            if(lateral_accel > max_accel){
                last_speed = std::sqrt(max_acceleration/curvature);
                itr->second = last_speed;
                lateral_accel = max_accel;
            }
            if(last_x >= (loop ? 2.f : 1.f))
                break;

            Float longitudinal_accel = std::sqrt(std::pow(max_accel, 2.f) - std::pow(lateral_accel, 2.f));
            if(longitudinal_accel * last_speed > power_to_mass){
                longitudinal_accel = power_to_mass / last_speed;
            }

            Float distance_along = last_speed *dt + longitudinal_accel * dt*dt/2;
            Float offset = last_x - last_x_in_range;
            last_x = _path->next_point(last_x_in_range, distance_along);
            if(last_x >= 1.f && loop){
                last_x = 1.f + _path->next_point(last_x - 1, distance_along + _path->distance(last_x_in_range)
                                                             - _path->distance(1.f) - _path->distance(last_x - 1));
            }
            last_x += offset;

            last_speed += longitudinal_accel * dt;
            itr = _speeds.insert(itr, std::make_pair(last_x, last_speed));
        }
    }

    auto itr = _speeds.end();
    itr--;
    //Backward pass for braking
    while(itr != _speeds.begin()){
        auto previous = itr;
        previous--;
        Float speed = itr->second;
        if(speed < previous->second){
            //Was going too fast
            Float x = itr->first;
            Float x_in_range = std::fmod(x, 1.f);

            Float curvature = std::abs(_path->curvature(x_in_range));
            Float v2 = std::pow(speed, 2.f);
            Float lateral_accel = v2 * curvature;
            Float max_accel = max_acceleration * (1+aero_accel_coef*v2);
            Float longitudinal_accel = -std::sqrt(std::pow(max_accel, 2.f) - std::pow(lateral_accel, 2.f));
            Float distance_along = speed*-dt + longitudinal_accel * dt*dt/2;
            Float offset = x - x_in_range;
            x = _path->next_point(x_in_range, distance_along);
            if(x < 0.f){
                x = -1.f + _path->next_point(x + 1, distance_along - _path->distance(x)
                                                    + _path->distance(1.f) + _path->distance(x_in_range));
            }
            x += offset;
            Float new_x = x;
            Float new_speed = speed + longitudinal_accel * -dt;

            while(previous->first >= new_x){
                previous = _speeds.erase(previous);
                if(previous == _speeds.begin())
                    break;
                previous--;
            }

            //Handle the case where we're at the point with the maximum speed nicely
            if(previous->second < new_speed)
                new_speed = previous->second;

            itr = _speeds.insert(itr, std::make_pair(new_x, new_speed));
        }
        else{
            itr = previous;
        }
    }

    if(loop){
        //delete 0-.5, copy 1-1.5 to 0-.5 and delete 1-2, ensuring the path is valid for multiple loops
        _speeds.erase(_speeds.begin(), _speeds.upper_bound(.5f));
        itr = _speeds.upper_bound(1.f);
        auto second_lap_half = _speeds.upper_bound(1.5f);
        auto insert_itr = _speeds.begin();
        while(itr != second_lap_half){
            insert_itr = _speeds.insert(insert_itr, std::make_pair(itr->first-1, itr->second));
            insert_itr++;
            itr++;
        }
        _speeds.erase(_speeds.upper_bound(1.f), _speeds.end());
    }
}

Float Trajectory::get_speed(Float x) const {
    auto itr = _speeds.lower_bound(x);
    if(itr == _speeds.end()){
        itr--;
        return itr->second;
    }
    if(itr == _speeds.begin())
        return itr->second;

    auto before = itr;
    before--;

    Float ratio = (x-before->first)/(itr->first - before->first);
    return ratio * itr->second + (1-ratio)*before->second;
}

Float Trajectory::total_time() const {
    return _speeds.size() * _dt;
}

const PathSegment &Trajectory::path() const {
    return *_path;
}

Path::Path(std::vector<std::shared_ptr<PathSegment>> &&segments)
        : _segments(std::move(segments)) {
    Float total = 0;
    for (auto& segment : _segments) {
        _distance_sums.push_back(total);
        total += segment->distance(1.f);
    }
}

Path::Path(Path &&path) noexcept
        : _segments(std::move(path._segments)),
          _distance_sums(std::move(path._distance_sums)){
}

Path::Path(const Path &path)
        : _segments(path._segments),
          _distance_sums(path._distance_sums){
}

Vector2 Path::position(Float x) const {
    return evaluate(&PathSegment::position, x);
}

Vector2 Path::derivative(Float x) const {
    return evaluate(&PathSegment::derivative, x);
}

Vector2 Path::second_derivative(Float x) const {
    return evaluate(&PathSegment::second_derivative, x);
}

Float Path::distance(Float x) const {
    if(x >= 1)
        return *(_distance_sums.end()-1)+(_segments.end()-1)->get()->distance(1.f);
    if(x <= 0)
        return 0;
    Float mid_index = x*_segments.size();
    int segment_index = static_cast<int>(mid_index);
    return _distance_sums[segment_index] + _segments[segment_index].get()->distance(mid_index - segment_index);
}

Float Path::curvature(Float x) const {
    return evaluate(&PathSegment::curvature, x);
}

Line::Line(Vector2 start, Vector2 end)
        :   _start(std::move(start)),
            _end(std::move(end)){}

Vector2 Line::position(Float x) const {
    return x * _end + (1-x)*_start;
}

Vector2 Line::derivative(Float x) const {
    return _end - _start;
}

Vector2 Line::second_derivative(Float x) const {
    return Vector2();
}

Float Line::distance(Float x) const {
    return x * ((_end - _start).norm());
}

Float Line::curvature(Float x) const {
    return 0;
}

Arc::Arc(Vector2 center, Float radius, Float start_angle, Float end_angle)
        : _center(std::move(center)),
          _radius(radius),
          _start_angle(start_angle),
          _end_angle(end_angle) {}

Float Arc::get_angle(Float x) const {
    return x * _end_angle + (1-x)*_start_angle;
}

Vector2 Arc::position(Float x) const {
    Float angle = get_angle(x);
    return Vector2(std::cos(angle), std::sin(angle)) * _radius + _center;
}

Vector2 Arc::derivative(Float x) const {
    Float angle = get_angle(x);
    return Vector2(-std::sin(angle), std::cos(angle)) * _radius * (_end_angle - _start_angle);
}

Vector2 Arc::second_derivative(Float x) const {
    Float angle = get_angle(x);
    return Vector2(-std::cos(angle), -std::sin(angle)) * _radius * std::pow(_end_angle - _start_angle, 2);
}

Float Arc::distance(Float x) const {
    return (_end_angle - _start_angle)*_radius*x;
}

Float Arc::curvature(Float x) const {
    return std::copysign(1/_radius, _end_angle - _start_angle);
}

Spline::Spline(Polynomial xpoly, Polynomial ypoly, unsigned int distance_lookup_size)
        : _xpoly(std::move(xpoly)),
          _ypoly(std::move(ypoly)){
    distance_lookup.push_back(0);
    for (int i = 1; i < distance_lookup_size; ++i) {
        Float x = (i - 0.5f)/distance_lookup_size;
        distance_lookup.push_back(distance_lookup[i-1] + derivative(x).norm()/distance_lookup_size);
    }
}

Vector2 Spline::position(Float x) const {
    return Vector2(_xpoly.evaluate<0>(x), _ypoly.evaluate<0>(x));
}

Vector2 Spline::derivative(Float x) const {
    return Vector2(_xpoly.evaluate<1>(x), _ypoly.evaluate<1>(x));
}

Vector2 Spline::second_derivative(Float x) const {
    return Vector2(_xpoly.evaluate<2>(x), _ypoly.evaluate<2>(x));
}

Float Spline::distance(Float x) const {
    if(x >= 1)
        return *(distance_lookup.end()-1);
    if(x <= 0)
        return *distance_lookup.begin();
    Float mid_index = x*(distance_lookup.size() - 1);
    int low_index = static_cast<int>(mid_index);
    Float ratio = mid_index - low_index;
    return ratio * distance_lookup[low_index + 1] + (1-ratio) * distance_lookup[low_index];
}

Float Spline::curvature(Float x) const {
    Float xp = _xpoly.evaluate<1>(x);
    Float yp = _ypoly.evaluate<1>(x);
    Float xpp = _xpoly.evaluate<2>(x);
    Float ypp = _ypoly.evaluate<2>(x);

    return (xp * ypp - yp * xpp) / std::pow(xp*xp + yp*yp, 1.5f);
}

#include "mm_spring_damper.h"
#include "mm_math/mm_math.h"

MMSpringDamper* MMSpringDamper::singleton = nullptr;

void MMSpringDamper::_bind_methods() {
    ClassDB::bind_method(D_METHOD("spring_damper_velocity", "current_velocity", "acceleration", "target_velocity", "halflife", "delta"), &MMSpringDamper::spring_damper_velocity);
}

MMSpringDamper::MMSpringDamper() {
    singleton = this;
}

MMSpringDamper* MMSpringDamper::get_singleton() {
    return singleton;
}

Dictionary MMSpringDamper::spring_damper_velocity(const Vector3& current_veloctiy, const Vector3& acceleration, const Vector3& target_velocity, const float& halflife, float delta) {
    Dictionary result;
    Vector3 x = Vector3(0, 0, 0);
    Vector3 v = current_veloctiy;
    Vector3 a = acceleration;
    MMMath::spring_velocity_match<Vector3>(x, v, a, target_velocity, halflife, delta);
    result["location"] = x;
    result["velocity"] = v;
    result["acceleration"] = a;
    Array future_trajectory;
    for(int i = 0; i != motion_matching::FeatureVector::trajectory_point_cnt; ++i) {
        Vector3 future_loc = Vector3(0, 0, 0), future_velocity = current_veloctiy, future_acceleration = acceleration;
        MMMath::spring_velocity_match<Vector3>(future_loc, future_velocity, future_acceleration, target_velocity, halflife, motion_matching::FeatureVector::intervals[i]);
        future_trajectory.push_back(future_loc);
        future_trajectory.push_back(future_velocity);
    }
    result["trajectory"] = future_trajectory;
    result["trajectory_cnt"] = motion_matching::FeatureVector::trajectory_point_cnt;
    return result;
}
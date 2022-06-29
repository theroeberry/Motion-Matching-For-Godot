#include "mm_math.h"

float MMMath::fast_atan(float x) {
    float z = fabs(x);
    float w = z > 1.0f ? 1.0f / z : z;
    float y = (Math_PI / 4.0f) * w - w * (w - 1) * (0.2447f + 0.0663f * w);
    return copysign(z > 1.0f ? Math_PI / 2.0f - y : y, x);
}

float MMMath::fast_rsqrt(float x) noexcept {
    float y = x;
    int i = *(int*) &y;
    i = 0x5f3759df - (i >> 1); // calculating log(y^(-1/2))
    y = *(float*) &i;
    return  y * (1.5f - (x * y * y * .5f));
}

float MMMath::angle_lerp(float la, float ra, float ratio) noexcept {
    if(fabs(ra - la) < Math_PI) return Math::lerp(la, ra, ratio);
    if(la < ra) la += Math_TAU;
    else ra += Math_TAU;
    la = Math::lerp(la, ra, ratio);
    return la > Math_TAU ? la - Math_TAU : la;
}

float MMMath::get_y_rotation(const Quat& quat) { 
    float result = acos(CLAMP(quat.w, -1.f, 1.f));
    auto axis = Vector3(quat.x, quat.y, quat.z) / sin(result);
    result *= 2;
    if(axis.y < 0)
        result = -result;
    if(result < 0)
        result += Math_TAU; 
    return result;
}

float MMMath::delta_angle(float angle, float dangle) {
    float result = dangle - angle;
    if(result < -Math_PI)
        result += Math_TAU; // Rotate counter-clockwise
    else if(result > Math_PI)
        result -= Math_TAU; // Rotate clockwise
    return result;
}

Vector3 MMMath::quat_to_angular_velocity(const Quat& quat) {
    if(Math::is_equal_approx(fabs(quat.w), 1.)) return Vector3(0.f, 0.f, 0.f); // No rotation
    float angle = acos(CLAMP(quat.w, -1.f, 1.f));
    Vector3 axis = Vector3(quat.x, quat.y, quat.z);
    axis /= sin(angle);
    angle *= 2.f;
    return axis * angle;
}

Quat MMMath::quat_from_angular_velocity(const Vector3& vec) {
    float angle = vec.length();
    angle *= .5f;
    auto axis = vec.normalized();
    axis *= sin(angle);
    return Quat(axis.x, axis.y, axis.z, cos(angle));
}

void MMMath::simple_spring_damper_implicit_quat(Quat& x, Vector3& v, const Quat& goal, float halflife, float delta) {
    float y = halflife_to_damping(halflife) / 2.f;
    Vector3 j0 = quat_to_angular_velocity(x * goal.inverse());
    Vector3 j1 = v + j0 * y;
    float eydt = fast_negexp(y * delta);

    x = quat_from_angular_velocity(eydt * (j0 + j1 * delta)) * goal;
    v = eydt * (v - j1 * y * delta);
}

void MMMath::inertialize_update_quat(
    Quat& out_quat, Vector3& out_velocity, 
    Quat& off_quat, Vector3& off_velocity, 
    const Quat& in_quat, const Vector3& in_velocity, 
    float halflife, float delta, float eydt) {
    decay_spring_damper_implicit_quat(off_quat, off_velocity, halflife, delta, eydt);
    out_quat = off_quat * in_quat;
    out_velocity = in_velocity + off_velocity;
}

void MMMath::decay_spring_damper_implicit_quat(Quat& quat, Vector3& velocity, float halflife, float delta, float eydt) {
    float y = halflife_to_damping(halflife) / 2.0f;	
    Vector3 j0 = quat_to_angular_velocity(quat);
    Vector3 j1 = velocity + j0 * y;

    quat = quat_from_angular_velocity(eydt * (j0 + j1 * delta));
    velocity = eydt * (velocity - j1 * y * delta);
}

// float MMMath::inertialization(float a, float b, float c, float t, float a0, float v0, float x0) {
//     float t_squared = t * t;
//     float t_forth_power = t_squared * t_squared;
//     return a * t * t_forth_power + b * t_forth_power + c * t * t_squared + a0 * t_squared * .5f + v0 * t + x0;
// }

float MMMath::deviation(const Vector<float>& data) {
    return deviation(data.ptr(), data.size());
}

float MMMath::deviation(const float* data, int size) {
    if(size < 2) return 0.f;
    float average = 0.f;
    for(int i = 0; i != size; ++i) {
        average += data[i];
    }
    average /= size;
    float deviation = 0.f;
    for(int i = 0; i != size; ++i) {
        float temp = data[i] - average;
        deviation += temp * temp;
    }
    return deviation / (size - 1);
}

float MMMath::standard_deviation(const Vector<float>& data) {
    float d = deviation(data);
    if(d == 0.f) return d;
    return d * fast_rsqrt(d);
}

Quat MMMath::minial_angle_quat(const Quat& quat) {
    return quat.w > 0 ? quat : quat * -1.f;
}
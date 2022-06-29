#ifndef MM_MATH_H
#define MM_MATH_H

#include "core/math/math_defs.h"
#include "core/math/math_funcs.h"
#include "core/math/quat.h"
#include "core/math/vector3.h"
#include "core/vector.h"

class MMMath {

public:

    static inline float eps = 1e-5f;

    /**
     * @brief exp(-x)
    */ 
    _FORCE_INLINE_ static float fast_negexp(float x) { return 1.f / (1.f + x + 0.48f * x * x + 0.235f * x * x * x); }

    static float fast_atan(float x);

    _FORCE_INLINE_ static float sqrf(float x) noexcept { return x * x; }

    template <typename T> static void critical_spring_damper_implicit(T& x, T& v, const T& x_goal, const T& v_goal, float halflife, float delta, float eydt = NAN) {
        float d = halflife_to_damping(halflife);
        T c = x_goal + (d * v_goal) / (sqrf(d) / 4.f);
        float y = d / 2.f;
        T j0 = x - c;
        T j1 = v + j0 * y;
        if(!(eydt == eydt))
            eydt = fast_negexp(y * delta);

        x = eydt * (j0 + j1 * delta) + c;
        v = eydt * (v - j1 * y * delta);
    }

    /**
     * @brief Critical spring damper that decay location to zero
     * @param x location
     * @param v velocity
    */ 
    template <typename T> static void decay_spring_damper_implicit(T& x, T& v, float halflife, float delta, float eydt = NAN) {
        float y = halflife_to_damping(halflife) / 2.0f;	
        T j1 = v + x * y;
        if(!(eydt == eydt))
            eydt = fast_negexp(y * delta);

        x = eydt * (x + j1 * delta);
        v = eydt * (v - j1 * y * delta);
    }

    /**
     * @brief Critical spring damper on velocity, position is calculated as time integral of velocity
     * @param x Location, time integral of velocity
     * @param v Velocity
     * @param a Acceleration
     * @param v_goal Target velocity
     * @param halflife Approx. half of the time needed to reach target velocity
    */
    template <typename T> static void spring_velocity_match(T& x, T& v, T& a, const T& v_goal, float halflife, float delta, float eydt = NAN) {
        float y = halflife_to_damping(halflife) * .5f;
        T j0 = v - v_goal;
        T j1 = a + j0 * y;
        if(!(eydt == eydt))
            eydt = fast_negexp(y * delta);

        x = eydt * ((j1 * delta) / y) + delta * v_goal + x; // Time intergral of velocity
        v = eydt * (j0 + j1 * delta) + v_goal;
        a = eydt * (a - j1 * y * delta);
    }

    static float fast_rsqrt(float x) noexcept;

    static float angle_lerp(float la, float ra, float ratio) noexcept;

    /**
     * @brief Calculate angle in radian that represents counter-clockwise rotation around Vector3::UP
    */ 
    static float get_y_rotation(const Quat& quat);

    static float delta_angle(float angle, float dangle);

    _FORCE_INLINE_ static float halflife_to_damping(float halflife) {
        return (4.f * Math_LN2) / (halflife + eps);
    }

    // First convert to angle axis then scale the axis by the angle
    // Quat must be normalized before
    static Vector3 quat_to_angular_velocity(const Quat& quat);

    static Quat quat_from_angular_velocity(const Vector3& vec);

    static void simple_spring_damper_implicit_quat(Quat& x, Vector3& v, const Quat& goal, float halflife, float delta);

    // Angular velocity is represented as rotation axis scaled by rotation angle
    // With eydt as a parameter for fast calculation
    static void inertialize_update_quat(Quat& out_quat, Vector3& out_velocity, Quat& off_quat, Vector3& off_velocity, const Quat& in_quat, const Vector3& in_velocity, float halflife, float delta, float eydt);

    template <typename T> static void inertialize_update(T& out_loc, T& out_velocity, T& off_loc, T& off_velocity, const T& in_loc, const T& in_velocity, float halflife, float delta, float eydt) {
        decay_spring_damper_implicit<T>(off_loc, off_velocity, halflife, delta, eydt);
        out_loc = in_loc + off_loc;
        out_velocity = in_velocity + off_velocity;
    }

    // Decay a given quat to identity quat
    // With eydt as a parameter for fast calculation
    static void decay_spring_damper_implicit_quat(Quat& quat, Vector3& velocity, float halflife, float delta, float eydt);

    // static float inertialization(float a, float b, float c, float t, float a0, float v0, float x0);

    static float deviation(const Vector<float>& data);
    static float deviation(const float* data, int size);
    static float standard_deviation(const Vector<float>& data);

    static Quat minial_angle_quat(const Quat& quat);

};

#endif // MM_MATH_H
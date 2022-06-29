#include "mm_helper.h"

String to_string(const Vector3& vec) {
    return "x:" + String::num_real(vec.x) + " y:" + String::num_real(vec.y) + " z:" + String::num_real(vec.z);
}

String to_string(const Quat& quat) {
    return "x:" + String::num_real(quat.x) + " y:" + String::num_real(quat.y) + " z:" + String::num_real(quat.z) + " w:" + String::num_real(quat.w);
}
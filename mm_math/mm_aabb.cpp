#include "mm_aabb.h"

MMAABB MMAABB::aabb_union(const MMAABB& a, const MMAABB& b) {
    return MMAABB(min_vector(a.lower_bound, b.lower_bound), max_vector(a.upper_bound, b.upper_bound), MIN(a.lower_index, b.lower_index), MAX(a.upper_index, b.upper_index));
}

Vector<float> MMAABB::min_vector(const Vector<float>& a, const Vector<float>& b) {
    _ASSERT(a.size() == b.size());
    Vector<float> result;
    for(int i = 0; i != a.size(); ++i) {
        result.push_back(a[i] < b[i] ? a[i] : b[i]);
    }
    return result;
}

Vector<float> MMAABB::max_vector(const Vector<float>& a, const Vector<float>& b) {
    _ASSERT(a.size() == b.size());
    Vector<float> result;
    for(int i = 0; i != a.size(); ++i) {
        result.push_back(a[i] > b[i] ? a[i] : b[i]);
    }
    return result;
}
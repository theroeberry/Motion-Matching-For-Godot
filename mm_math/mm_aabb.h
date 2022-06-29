#ifndef MM_AABB_H
#define MM_AABB_H

#include "core/vector.h"

class MMAABB {

public:

    Vector<MMAABB> children;

    Vector<float> lower_bound;
    Vector<float> upper_bound;
    int lower_index;
    int upper_index;

    MMAABB() {}
    MMAABB(const Vector<float>& lb, const Vector<float>& ub, int li, int ui) : lower_bound(lb), upper_bound(ub), lower_index(li), upper_index(ui) { _ASSERT(lb.size() == ub.size()); }

    static MMAABB aabb_union(const MMAABB& a, const MMAABB& b);
    static Vector<float> min_vector(const Vector<float>& a, const Vector<float>& b);
    static Vector<float> max_vector(const Vector<float>& a, const Vector<float>& b);
};

#endif // MM_AABB_H
#ifndef MM_HELPER_H
#define MM_HELPER_H

#include "core/ustring.h"
#include "core/math/quat.h"
#include "core/math/vector3.h"

class MMHelper {

public:

    static String to_string(const Vector3& vec);
    static String to_string(const Quat& quat);

};

#endif // MM_HELPER_H
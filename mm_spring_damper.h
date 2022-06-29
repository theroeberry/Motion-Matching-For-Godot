#ifndef MM_SPRING_DAMPER_H
#define MM_SPRING_DAMPER_H

#include "core/class_db.h"
#include "scene/main/node.h"
#include "core/vector.h"
#include "mm_math/mm_data_types.h"

class MMSpringDamper final : public Node {

    GDCLASS(MMSpringDamper, Node);

private:

    static MMSpringDamper* singleton;

    static void _bind_methods(); 


public:

    MMSpringDamper(const MMSpringDamper&) = delete;

    MMSpringDamper();

    Dictionary spring_damper_velocity(const Vector3& current_veloctiy, const Vector3& acceleration, const Vector3& target_velocity, const float& halflife, float delta);

    static MMSpringDamper* get_singleton();

};

#endif // MM_SPRING_DAMPER_H
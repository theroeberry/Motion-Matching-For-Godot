#ifndef MM_DATA_TYPES_H
#define MM_DATA_TYPES_H

#include "core/vector.h"
#include "core/ustring.h"
#include "core/math/vector3.h"
#include "core/math/quat.h"
#include "core/map.h"

namespace motion_matching {

struct InertializationParameter {
public:
    float x0;
    float v0;
    float a0;
    float cofa;
    float cofb;
    float cofc;
    Vector3 q0;
};

struct JointData {
     
public:

    Vector3 position;
    Vector3 velocity;
    Quat quat;
    Quat delta_quat;
    Vector3 angular_velocity;
    int track_index;

};

struct TrajectoryPoint {

public:
    Vector3 offset;
    Vector3 direction;
};

struct PoseVector {

public:
    String anim_name;
    float anim_time;
    int root_bone_track_index;
};


struct FeatureVector {

public:

    static constexpr int trajectory_point_cnt = 3;
    static constexpr float intervals[trajectory_point_cnt] = { .2f, .5f, 1.f};

    Vector3 root_velocity;
    float y_rotation;
    float y_rotational_velocity;
    float y_rotation_projection; // Project angle to [0, 1] for cost calculation
    TrajectoryPoint trajectory[trajectory_point_cnt];
    Vector<JointData> joint_data;
    Quat quat;
};

struct AnimData {

public:
    static inline int bone_cnt;
    static inline int root_bone_index;
    static inline int hip_bone_index;
    static inline Map<String, int> bone_name_id_map;
    static  inline int dimension;

    PoseVector pose_vector;
    Vector<float> feature_vector;
    // When true
    // This frame will be ignored during matching process but will be played
    // This is because after matching is complete and a best match is calculated, the animation will keep playing from best match for a given time period
    // This prevents exceeding current animation file and jump into another
    bool ignored = false;
};

}

#endif // MM_DATA_TYPES_H
#ifndef MOTION_MATCHING_H
#define MOTION_MATCHING_H

#include "scene/main/node.h"
#include "core/class_db.h"
#include "core/vector.h"
#include "mm_math/mm_data_types.h"
#include "scene/resources/animation.h"
#include "mm_math/mm_math.h"
#include "mm_math/mm_aabb.h"

class MotionMatching : public Node {

    GDCLASS(MotionMatching, Node);

protected:

    // struct CostFuncClass {
    //     float responsivity;
    //     float operator()(float* fa, const Vector<float>& fv) {
    //         float cost = 0.f;
    //         for(int i = 0; i != fv.size(); ++i) {
    //             cost += distance_to_axis(fa[i], fv[i], i);
    //         }
    //         return cost;
    //     }
    //     float distance_to_axis(float a, float b, int axis) {
    //         float temp = (a - b) * (a - b);
    //         if(axis - 5 >= 0 && axis - 5 < 18) temp *= responsivity;
    //         if(axis < 3) temp *= responsivity;
    //         return temp;
    //     }
    // };

    // Exposed variables
    NodePath skeleton_node_path;
    NodePath animation_player_path;
    Array bones_to_track;
    float responsivity = 1.f;
    float speed_scale = 1.f;
    // Exposed variables

    class AnimationPlayer* animation_player;
    int animation_index;
    Vector<motion_matching::AnimData> animation_array;
    // KDTree<int, CostFuncClass> kd_tree;
    Vector<MMAABB> aabb_array;
    float* standard_deviations = nullptr;

    // Animation Status
    motion_matching::FeatureVector current_feature_vector;
    motion_matching::FeatureVector goal;
    Vector<motion_matching::JointData> offset_pose_array;
    String cur_anim_name;
    float cur_anim_time;
    bool should_blend = false;
    float blend_time = 0.f; // Elapsed time period since blend begin
    int match_cnt = 0;
    bool pending = false;
    bool activated = false;
    Ref<Animation> pending_anim;
    float pending_anim_pos = false;
    bool stoping = false;
    // Animation status

    static constexpr float sample_step = .1f;
    static constexpr float time_delta = 1.f / 30.f; // For calculating velocity
    static constexpr float anim_blend_time = .4f;
    static constexpr float blend_halflife = .1f;

    class KinematicBody* parent;
    class Skeleton* skeleton = nullptr;

    void flip_flop(Ref<Animation> animation, float anim_pos);

    void generate_skeleton_map() const;

    void initialize_bones_to_track();

    // motion_matching::InertializationParameter MotionMatching::inertialization_precalculation(const Quat& q0, const Quat& q1);

    /**
     * @brief Set joint pose for a given animation and track at time
    */
    void set_joint_pose_track(Ref<Animation> animation, float time, int32_t track_index);

    // Set skeleton pose from a given pose array
    void set_skeleton_pose(const Vector<motion_matching::JointData>& pose_array);

    // Interpolate feature vector for pose matching
    Vector<float> calculate_feature_vector(float delta) const;

    Vector<motion_matching::JointData> calculate_pose_array(int index, float time_offset) const;
    Vector<motion_matching::JointData> calculate_pose_array(Ref<Animation> animation, float position) const;

    int find_best_match_index(Vector<float>& cfv, float delta);

    /**
     * @brief Calculate offset and add to pose_array
     * @param delta frame time
    */
    void blend_animation(Vector<motion_matching::JointData>& pose_array, float delta);

    /**
     * @brief Calculate pose offset between current pose and target pose for blending
     * @param src_index Index of animation array to blend from
     * @param src_time_offset Time offset from pose_vector.anim_time
     * @param dst_index Target index of animation array to blend into
    */ 
    void blend_pose_offset(int src_index, float src_time_offset, int dst_index);
    void blend_pose_offset(Ref<Animation> src_anim, float src_anim_position, Ref<Animation> dest_anim, float dest_anim_position);
    
    static void _bind_methods();

    float* feature_vector_to_float_array(const motion_matching::FeatureVector& fv);

    float* vector3_to_float_array(const Vector3& vec3);

    Vector<float> feature_vector_to_float_vector(const motion_matching::FeatureVector& fv);

    // Generate aabb enclosing up to 64 frames and up to 4 children aabbs
    MMAABB generate_aabb(int from);
    // Generate aabb enclosing up to 16 frames and no children
    MMAABB generate_aabb(int from, int to);

    int search(const Vector<float>& fv);

public:
    MotionMatching(); 
    ~MotionMatching();

    NodePath get_skeleton_node_path() const;
    void set_skeleton_node_path(const NodePath& np);
    NodePath get_animation_player_path() const;
    void set_animation_player_path(const NodePath& np);
    Array get_bones_to_track() const;
    void set_bones_to_track(const Array& array);
    float get_responsivity() const;
    void set_responsivity(float new_responsivity);
    float get_speed_scale() const;
    void set_speed_scale(float new_speed_scale);

    void anim_update(float delta);
    void process_animation();
    void set_goal(const Variant& trajectory);
    
    static String to_string(const Quat& quat);
    static String to_string(const Vector3& vec);
};

#endif // MOTION_MATCHING_H
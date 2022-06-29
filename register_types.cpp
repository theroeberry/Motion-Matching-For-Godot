//
//   █████▒█    ██  ▄████▄   ██ ▄█▀       ██████╗ ██╗   ██╗ ██████╗
// ▓██   ▒ ██  ▓██▒▒██▀ ▀█   ██▄█▒        ██╔══██╗██║   ██║██╔════╝
// ▒████ ░▓██  ▒██░▒▓█    ▄ ▓███▄░        ██████╔╝██║   ██║██║  ███╗
// ░▓█▒  ░▓▓█  ░██░▒▓▓▄ ▄██▒▓██ █▄        ██╔══██╗██║   ██║██║   ██║
// ░▒█░   ▒▒█████▓ ▒ ▓███▀ ░▒██▒ █▄       ██████╔╝╚██████╔╝╚██████╔╝
//  ▒ ░   ░▒▓▒ ▒ ▒ ░ ░▒ ▒  ░▒ ▒▒ ▓▒       ╚═════╝  ╚═════╝  ╚═════╝
//  ░     ░░▒░ ░ ░   ░  ▒   ░ ░▒ ▒░
//  ░ ░    ░░░ ░ ░ ░        ░ ░░ ░
//           ░     ░ ░      ░  ░
//   
#include "register_types.h"
#include "core/class_db.h"
#include "motion_matching.h"
#include "core/engine.h"
#include "mm_spring_damper.h"
#include "editor/plugins/animation_blend_tree_editor_plugin.h"

static MMSpringDamper* _mm_spring_damper = nullptr;

void register_motion_matching_types() {

    // register and initialization

    ClassDB::register_class<MotionMatching>();
    ClassDB::register_class<MMSpringDamper>();
    
    _mm_spring_damper = memnew(MMSpringDamper);

    Engine::get_singleton()->add_singleton(Engine::Singleton("MMSpringDamper", MMSpringDamper::get_singleton()));

}

void unregister_motion_matching_types() {
    // Clean up
    memdelete(_debug_draw_3d);
    memdelete(_mm_spring_damper);
    memdelete(_animation_solver);
}
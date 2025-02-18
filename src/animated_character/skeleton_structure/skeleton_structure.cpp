#include "skeleton_structure.hpp"

using namespace cgp;

int skeleton_structure::size() const{
    return joint_name.size();   
}

void skeleton_structure::update_joint_matrix_local_to_global() {
    int const N = size();
    if(N==0){ return; }
    if(joint_matrix_global.size()!=N) {
        joint_matrix_global.resize(N);
    }
    assert_cgp(parent_index.size()==size(), "Incoherent size of skeleton structure");
    assert_cgp(joint_matrix_local.size()==size(), "Incoherent size of skeleton structure");

    joint_matrix_global[0] = joint_matrix_local[0];
    for (int k = 1; k < N; ++k)
        joint_matrix_global[k] = joint_matrix_global[parent_index[k]] * joint_matrix_local[k];
}


void skeleton_structure::update_joint_matrix_local_to_global_except_ids_r(numarray<int> exceptions, int id_parent) {
    numarray<int> children = child(id_parent);
    for (int k=0 ; k<children.size();k++) {
        int id_joint = children[k];
        if (! exceptions.contains(id_joint)) {
            joint_matrix_global[id_joint] = joint_matrix_global[parent_index[id_joint]] * joint_matrix_local[id_joint];
            update_joint_matrix_local_to_global_except_ids_r(exceptions, id_joint);
        }
    }
}


void skeleton_structure::update_joint_matrix_local_to_global_except_ids(numarray<int> exceptions) {
    int const N = size();
    if(N==0){ return; }
    if(joint_matrix_global.size()!=N) {
        joint_matrix_global.resize(N);
    }
    assert_cgp(parent_index.size()==size(), "Incoherent size of skeleton structure");
    assert_cgp(joint_matrix_local.size()==size(), "Incoherent size of skeleton structure");

    joint_matrix_global[0] = joint_matrix_local[0];

    numarray<int> children = child(0);
    for (int k=0 ; k<children.size();k++) {
        int id_joint = children[k];
        if (! exceptions.contains(id_joint)) {
            joint_matrix_global[id_joint] = joint_matrix_global[parent_index[id_joint]] * joint_matrix_local[id_joint];
            update_joint_matrix_local_to_global_except_ids_r(exceptions, id_joint);
        }
    }
}


void skeleton_structure::update_joint_matrix_local_to_global_from_id(int id_parent) {
    
    joint_matrix_global[id_parent] = joint_matrix_global[parent_index[id_parent]] * joint_matrix_local[id_parent];
    numarray<int> children = child(id_parent);
    for (int k=0 ; k<children.size();k++) {
        update_joint_matrix_local_to_global_from_id(children[k]);
    }
}


void skeleton_structure::update_joint_matrix_global_to_local() {
    int const N = size();
    if(N==0){ return; }
    if(joint_matrix_local.size()!=N) {
        joint_matrix_local.resize(N);
    }
    assert_cgp(parent_index.size()==size(), "Incoherent size of skeleton structure");
    assert_cgp(joint_matrix_global.size()==size(), "Incoherent size of skeleton structure");

    joint_matrix_local[0] = joint_matrix_global[0];
    for (int k = 1; k < N; ++k)
        joint_matrix_local[k] = joint_matrix_global[parent_index[k]].inverse_assuming_rigid_transform() * joint_matrix_global[k];
}

void skeleton_structure::update_joint_matrix_global_to_local_from_id(int id_parent) {
    if(id_parent == 0) {
        joint_matrix_local[0] = joint_matrix_global[0];
    } else {
        joint_matrix_local[id_parent] = joint_matrix_global[parent_index[id_parent]].inverse_assuming_rigid_transform() * joint_matrix_global[id_parent];
    }
    numarray<int> children = child(id_parent);
    for (int k=0 ; k<children.size();k++) {
        update_joint_matrix_global_to_local_from_id(children[k]);
    }
}

std::string skeleton_structure::parent_name(int joint_index) const 
{
    int parent = parent_index[joint_index];
    if(parent==-1) {
        return "GlobalFrame";
    }
    return joint_name[parent];
}
std::string skeleton_structure::parent_name(std::string const& name) const
{
    int N = size();
    for(int k=0; k<N; ++k) {
        if(name==joint_name[k]) {
            return parent_name(k);
        }
    }
    return "None";
}
cgp::numarray<int> skeleton_structure::child(int joint_index) const
{
    cgp::numarray<int> children_index;
    int N = size();
    for(int k=0; k<N; ++k) {
        if(parent_index[k]==joint_index) {
            children_index.push_back(k);
        }
    }
    return children_index;
}

void skeleton_structure::apply_translation_recursive(int joint_index, vec3 translation)
{
    joint_matrix_global[joint_index].apply_translation(translation);

    numarray<int> children_index = child(joint_index);
    int n = children_index.size();
    for (int i=0 ; i < n ; i++) {
        int child_id = children_index[i];
        apply_translation_recursive(child_id, translation);
    }
}

void skeleton_structure::init_constraints()
{
    int N_joints = joint_matrix_global.size();
    constraint_angles.resize(N_joints);
    constraint_orientation.resize(N_joints);

    // all to -1 (= not defined) 
    for (int i=0; i<N_joints; i++)
    {
        constraint_angles[i] = vec4(-1.f, -1.f, -1.f, -1.f);
        constraint_orientation[i] = 2.f*Pi;
    }

    // angle constraints

    // epaule droite | id = 17
    /*constraint_angles[17].x = Pi/100.f; 
    constraint_angles[17].y= Pi/4.f;
    constraint_angles[17].z = 3.f*Pi/8.f;
    constraint_angles[17].w = Pi/16.f; */

    // coude droit | id = 24
    constraint_angles[24].x = 0.45f*Pi;
    constraint_angles[24].y = 0.1f*Pi;
    constraint_angles[24].z = 0.45f*Pi; 
    constraint_angles[24].w = 0.45f*Pi; 
    //constraint_orientation[24] = Pi;

    // epaule gauche | id = 16
    /*constraint_angles[16].x = Pi/4.f;
    constraint_angles[16].y = Pi/100.f; 
    constraint_angles[16].z = Pi/16.f;
    constraint_angles[16].w = 3.f*Pi/8.f; */

    // coude gauche | id = 23
    constraint_angles[23].x = 0.45f*Pi; 
    constraint_angles[23].y = 0.1f*Pi;
    constraint_angles[23].z = 0.45f*Pi;
    constraint_angles[23].w = 0.45f*Pi; 
    //constraint_orientation[23] = Pi;

    // torse et buste | ids 1, 4 et 7
    //constraint_angles[1].x = Pi;
    //constraint_angles[1].y= Pi; 
    //constraint_angles[1].z = Pi;
    //constraint_angles[1].w = Pi/2.f; 
    //constraint_orientation[1] = 3.f*Pi/4.f;
    //constraint_orientation[4] = 3.f*Pi/4.f;

    // pieds | ids 8 et 9
    //droit
    /*constraint_angles[8].x = Pi/6.f;
    constraint_angles[8].y = 0.45f*Pi; 
    constraint_angles[8].z = Pi/6.f;
    constraint_angles[8].w = Pi/6.f; 
    //gauche
    constraint_angles[9].x = 0.45f*Pi; 
    constraint_angles[9].y = Pi/6.f;
    constraint_angles[9].z = Pi/6.f; 
    constraint_angles[9].w = Pi/6.f; */

    // genoux | ids 5 et 6
    //droit
    constraint_angles[6].x = 0.45f*Pi;
    constraint_angles[6].y = Pi/8.f; 
    constraint_angles[6].z = 0.45f*Pi; 
    constraint_angles[6].w = 0.45f*Pi; 
    constraint_orientation[6] = Pi/2.f; //ok

    //gauche
    constraint_angles[5].x = 0.45f*Pi;
    constraint_angles[5].y = Pi/8.f;
    constraint_angles[5].z = 0.45f*Pi;
    constraint_angles[5].w = 0.45f*Pi; 
    constraint_orientation[5] = Pi/2.f; //ok

    /*constraint_angles[7].x = 0.48f*Pi;
    constraint_angles[7].y = 0.48f*Pi; 
    constraint_angles[7].z = 0.48f*Pi;
    constraint_angles[7].w = 0.48f*Pi;*/
    
}
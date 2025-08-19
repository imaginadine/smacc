import bpy
import sys
import os
import numpy as np

def mat4_to_flatlist(m):
    return [f for row in m for f in row]

def get_args():
    argv = sys.argv
    if "--" in argv:
        idx = argv.index("--")
        return argv[(idx + 1):]
    return []

def export_mesh(obj, output_dir):
    import bpy

def export_mesh(mesh_obj, output_dir):
    import os

    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = mesh_obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()

    output_path = os.path.join(output_dir, "mesh_ascii.txt")
    with open(output_path, "w") as f:
        f.write("vertices\n")
        for v in mesh.vertices:
            f.write(f"{v.co.x} {v.co.y} {v.co.z}\n")

        f.write("\nnormals\n")
        for v in mesh.vertices:
            f.write(f"{v.normal.x} {v.normal.y} {v.normal.z}\n")

        f.write("\nfaces\n")
        for poly in mesh.polygons:
            f.write(f"{len(poly.vertices)} " + " ".join(str(i) for i in poly.vertices) + "\n")

    # Release memory
    eval_obj.to_mesh_clear()


def export_skeleton(arm_obj, output_dir):
    bones = arm_obj.data.bones
    joint_names = []
    parent_indices = []
    bind_matrices = []

    bone_index = {bone.name: i for i, bone in enumerate(bones)}

    for bone in bones:
        joint_names.append(bone.name)
        parent_idx = -1 if bone.parent is None else bone_index[bone.parent.name]
        parent_indices.append(parent_idx)
        mat = bone.matrix_local
        bind_matrices.append(mat)

    with open(os.path.join(output_dir, "skeleton_joint_name.txt"), "w") as f:
        for name in joint_names:
            f.write(name + "\n")
    with open(os.path.join(output_dir, "skeleton_parent_index.txt"), "w") as f:
        for idx in parent_indices:
            f.write(str(idx) + " ")
    with open(os.path.join(output_dir, "skeleton_joint_matrix.txt"), "w") as f:
        for mat in bind_matrices:
            for row in mat:
                for v in row:
                    f.write(f"{v} ")
            f.write("\n")

def export_skinning(obj, arm_obj, output_dir):
    mod = next((m for m in obj.modifiers if m.type == 'ARMATURE'), None)
    if not mod:
        return
    vg_indices = {vg.index: vg.name for vg in obj.vertex_groups}

    bones = arm_obj.data.bones
    bone_index = {bone.name: i for i, bone in enumerate(bones)}

    skin_joint_names = []
    skin_rig_to_skel = []
    inverse_bind_matrices = []

    for i, bone in enumerate(bones):
        skin_joint_names.append(bone.name)
        skin_rig_to_skel.append(i)
        mat = bone.matrix_local.inverted()
        inverse_bind_matrices.append(mat)

    with open(os.path.join(output_dir, "skinning_joint_name_dependence.txt"), "w") as f:
        for name in skin_joint_names:
            f.write(name + "\n")
    with open(os.path.join(output_dir, "skinning_joint_rig_index_to_skeleton_index.txt"), "w") as f:
        for idx in skin_rig_to_skel:
            f.write(f"{idx} ")
    with open(os.path.join(output_dir, "skinning_inverse_bind_matrix.txt"), "w") as f:
        for mat in inverse_bind_matrices:
            for row in mat:
                for v in row:
                    f.write(f"{v} ")
            f.write("\n")

    joints = []
    weights = []
    for v in obj.data.vertices:
        vj = []
        vw = []
        for g in v.groups:
            bone_name = vg_indices.get(g.group)
            if bone_name in bone_index:
                vj.append(bone_index[bone_name])
                vw.append(g.weight)
        joints.append(vj)
        weights.append(vw)

    with open(os.path.join(output_dir, "skinning_joint.txt"), "w") as f:
        for js in joints:
            f.write(" ".join(str(j) for j in js) + "\n")
    with open(os.path.join(output_dir, "skinning_weight.txt"), "w") as f:
        for ws in weights:
            f.write(" ".join(str(w) for w in ws) + "\n")


def export_global_bind_matrix(output_dir):
    for obj in bpy.data.objects:
        if obj.type == 'MESH':
            mesh_obj = obj
            arm_mod = next((m for m in obj.modifiers if m.type == 'ARMATURE'), None)
            if arm_mod is None:
                continue
            arm_obj = arm_mod.object
            if arm_obj is None or arm_obj.type != 'ARMATURE':
                continue

            bind_matrix = arm_obj.matrix_world.inverted() @ mesh_obj.matrix_world

            mesh_dir = os.path.join(output_dir, f"mesh-{mesh_obj.name.lower()}")
            os.makedirs(mesh_dir, exist_ok=True)
            filepath = os.path.join(mesh_dir, "skinning_global_bind_matrix.txt")
            with open(filepath, 'w') as f:
                for row in bind_matrix:
                    for val in row:
                        f.write(f"{val:.6f} ")


def export_animation(arm_obj, output_dir):
    action = arm_obj.animation_data.action if arm_obj.animation_data else None
    if not action:
        return

    bones = arm_obj.pose.bones
    bone_index = {bone.name: i for i, bone in enumerate(bones)}
    joint_keys = []
    joint_times = []

    for bone in bones:
        fcurves = [f for f in action.fcurves if f.data_path.endswith(f'["{bone.name}"]')]
        key_times = set()
        for f in fcurves:
            key_times.update([kp.co[0] for kp in f.keyframe_points])
        key_times = sorted(key_times)
        matrices = []
        for t in key_times:
            bpy.context.scene.frame_set(int(t))
            matrices.append(bone.matrix.copy())
        joint_keys.append(matrices)
        joint_times.append(key_times)

    anim_dir = os.path.join(output_dir, "animation")
    os.makedirs(anim_dir, exist_ok=True)

    with open(os.path.join(anim_dir, "skeleton_animation_joint_index.txt"), "w") as f:
        for i in range(len(bones)):
            f.write(f"{i} ")
    with open(os.path.join(anim_dir, "skeleton_animation_timing.txt"), "w") as f:
        for times in joint_times:
            f.write(" ".join(str(t) for t in times) + "\n")
    with open(os.path.join(anim_dir, "skeleton_animation_matrix.txt"), "w") as f:
        for matrices in joint_keys:
            for mat in matrices:
                for row in mat:
                    for v in row:
                        f.write(f"{v} ")
            f.write("\n")

def main():
    fbx_path = "/home/alegrand/Documents/Amandine/AnimationWithLines/character_animation/assets/octopus.fbx"
    output_dir = "/home/alegrand/Documents/Amandine/AnimationWithLines/character_animation/assets/OUT"

    # Ensure output directory exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # --- Step 1: Import FBX ---
    import_success = False
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for region in area.regions:
                if region.type == 'WINDOW':
                    override = bpy.context.copy()
                    override['area'] = area
                    override['region'] = region
                    override['window'] = bpy.context.window
                    override['screen'] = bpy.context.screen
                    try:
                        bpy.ops.import_scene.fbx(override, filepath=fbx_path)
                        import_success = True
                        print("FBX import succeeded (with override)")
                    except Exception as e:
                        print(f"Error during import with override: {e}")
                    break
        if import_success:
            break

    if not import_success:
        try:
            bpy.ops.import_scene.fbx(filepath=fbx_path)
            print("FBX import succeeded (fallback)")
        except Exception as e:
            print(f"Fallback import failed: {e}")
            return

    # --- Step 2: Find Armature and Mesh ---
    arm_obj = None
    mesh_obj = None

    for obj in bpy.context.scene.objects:
        if obj.type == 'ARMATURE':
            arm_obj = obj
        elif obj.type == 'MESH':
            mesh_obj = obj

    # --- Step 3: Export ---
    if mesh_obj:
        print(f"Found mesh: {mesh_obj.name}")
        export_mesh(mesh_obj, output_dir)
    else:
        print("No mesh object found.")

    if arm_obj:
        print(f"Found armature: {arm_obj.name}")
        export_skeleton(arm_obj, output_dir)
    else:
        print("No armature object found.")

    if mesh_obj and arm_obj:
        export_skinning(mesh_obj, arm_obj, output_dir)
        export_global_bind_matrix(output_dir)
        export_animation(arm_obj, output_dir)

if __name__ == "__main__":
    main()

import os.path

import trimesh
import numpy as np

def transform_mesh(mesh_file: str, translation, out_put_name: str):
    mesh = trimesh.load(mesh_file)
    mesh.apply_translation(translation)
    output_file = os.path.join(os.path.dirname(mesh_file), out_put_name)
    mesh.export(output_file)
    print(f"Translated mesh mesh saved to {output_file}")

if __name__ == '__main__':
    mesh_files = [
        "industreal_tray_insert_round_peg_8mm.obj",
        "industreal_round_peg_8mm.obj"
    ]
    mesh_file_outputs = [
        "car_hood_left_top.stl",
        "car_logo_left_top.stl",
    ]
    translation = np.array([0.013, -0.013, -0.041])
    for mesh_file, mesh_file_output in zip(mesh_files, mesh_file_outputs):
        transform_mesh(mesh_file, translation, mesh_file_output)
    mesh_file_outputs = [
        "car_hood_right_bottom.stl",
        "car_logo_right_bottom.stl",
    ]
    translation = np.array([-0.013, 0.013, -0.041])
    for mesh_file, mesh_file_output in zip(mesh_files, mesh_file_outputs):
        transform_mesh(mesh_file, translation, mesh_file_output)

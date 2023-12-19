"""
AWP3D File

AWP3D is a very simple but effective animated 3D File format.
An AWP3D file is simply a ZIP file. Every frame of an animation in Blender
is exported as a Wavefront OBJ file and mtl and added inside the Zip.

Payton loads each frame from the file and uses them for key frames

This Add-On works with Blender 4.0.x +

With Blender 4.0.x the Wavefront exporter is moved under bpy.ops.wm with 
different flags. Maybe it's time to support glb format for Payton.

Author:
Sinan Islekdemir - sinan@islekdemir.com
"""

import os
from zipfile import ZipFile

import bpy
from bpy.props import StringProperty
from bpy_extras.io_utils import ExportHelper

bl_info = {
    "name": "AWP Export",
    "author": "Sinan Islekdemir",
    "version": (0, 0, 0, 1),
    "blender": (2, 80, 0),
}


class ExportAnimatedWavefrontPackage(bpy.types.Operator, ExportHelper):
    """Export animation frames as mesh files."""

    bl_idname = "frame.export_mesh_awp3d"
    bl_label = "Export selected animated object"
    bl_options = {'REGISTER'}

    filename_ext = ".awp3d"

    filter_glob: StringProperty(default="*.awp3d", options={"HIDDEN"}, maxlen=255)

    def execute(self, context):
        scene = context.scene

        path = os.path.dirname(os.path.realpath(self.filepath))
        files = []

        with ZipFile(self.filepath, 'w') as zipf:
            for frame in range(scene.frame_start, scene.frame_end):
                scene.frame_set(frame)
                context.view_layer.update()
                fname = os.path.join(path, f"{frame}.obj")
                bpy.ops.wm.obj_export(
                    filepath=fname, export_selected_objects=True, export_triangulated_mesh=True, forward_axis="NEGATIVE_Y", up_axis="Z"
                )
                zipf.write(
                    fname,
                    arcname=f"{frame}.obj",
                )
                zipf.write(fname.replace(".obj", ".mtl"), arcname=f"{frame}.mtl")
                files.append(fname)
                files.append(fname.replace(".obj", ".mtl"))

        for f in files:
            os.remove(f)

        return {'FINISHED'}


# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(ExportAnimatedWavefrontPackage.bl_idname, text="AWP3D Export (Payton 3D)")


def register():
    bpy.utils.register_class(ExportAnimatedWavefrontPackage)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(ExportAnimatedWavefrontPackage)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()

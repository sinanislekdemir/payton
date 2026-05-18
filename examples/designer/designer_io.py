import json
from pathlib import Path

from payton.scene.geometry.wavefront import export as export_wavefront
from payton.tools.mesh.geometry import merge_mesh

DEFAULT_SCENE_FILE = "designer_scene.json"
DEFAULT_EXPORT_FILE = "designer_export.obj"


def save_scene(specs, filename):
    path = Path(filename).expanduser()
    path.write_text(json.dumps({"objects": specs}, indent=2))
    return path


def load_scene(filename):
    path = Path(filename).expanduser()
    data = json.loads(path.read_text())
    objects = data.get("objects")
    if not isinstance(objects, dict):
        raise ValueError("Scene file must contain an 'objects' dictionary.")
    return path, objects


def export_scene(meshes, filename):
    if not meshes:
        raise ValueError("There are no objects to export.")

    merged = meshes[0]
    for mesh in meshes[1:]:
        merged = merge_mesh(merged, mesh)
    merged.fix_normals()

    path = Path(filename).expanduser()
    export_wavefront(merged, str(path))
    return path

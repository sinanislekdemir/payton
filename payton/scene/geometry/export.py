from payton.scene.geometry.mesh import Mesh


def export_json(mesh: Mesh, filename: str, **kwargs):
    data = mesh.to_json(**kwargs)
    open(filename, "w").write(data)


def import_json(filename: str) -> Mesh:
    data = open(filename, "r").read()
    return Mesh.from_json(data)

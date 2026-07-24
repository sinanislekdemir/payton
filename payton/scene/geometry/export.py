"""Export Import all to JSON."""

from typing import Any

from payton.scene.geometry.mesh import Mesh


def export_json(mesh: Mesh, filename: str, **kwargs: Any) -> None:
    """Export the given mesh as JSON.

    Keyword arguments:
    mesh -- Mesh to export
    filename -- Output filename
    """
    data = mesh.to_json(**kwargs)
    with open(filename, "w") as f:
        f.write(data)


def import_json(filename: str) -> Mesh:
    """Import the given file as a Mesh.

    Keyword arguments:
    filename -- Filename to import
    """
    with open(filename) as f:
        data = f.read()
    return Mesh.from_json(data)

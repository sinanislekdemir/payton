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
    open(filename, "w").write(data)


def import_json(filename: str) -> Mesh:
    """Import the given file as a Mesh.

    Keyword arguments:
    filename -- Filename to import
    """
    data = open(filename, "r").read()
    return Mesh.from_json(data)

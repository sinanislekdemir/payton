"""Machine-readable, AI-ready access to Payton's own documentation.

This module lets AI assistants, agents and editor tooling learn Payton's public
API without scanning the whole source tree:

* :func:`context` -- the bundled Markdown primer (:file:`payton/llms.txt`).
* :func:`catalog` -- the generated public-API catalog as a dictionary.
* :func:`catalog_json` -- the same catalog serialized as JSON text.
* :func:`examples` -- names of the curated examples bundled with the package.
* :func:`example` -- the source text of one bundled example.
* :func:`version` -- the installed Payton version.

Example
-------
>>> import payton.ai
>>> print(payton.ai.context())            # doctest: +SKIP
>>> api = payton.ai.catalog()             # doctest: +SKIP
>>> print(payton.ai.examples())           # doctest: +SKIP
"""

import json
from importlib.metadata import version as _distribution_version
from importlib.resources import files
from typing import Any, cast

__all__ = ["catalog", "catalog_json", "context", "example", "examples", "version"]


def version() -> str:
    """Return the installed Payton version string.

    Falls back to ``"unknown"`` when distribution metadata is unavailable
    (for example when running straight from a source checkout that was never
    installed).
    """
    try:
        return _distribution_version("Payton")
    except Exception:  # noqa: BLE001
        return "unknown"


def context() -> str:
    """Return the bundled Markdown primer that teaches Payton to an AI agent."""
    return files("payton").joinpath("llms.txt").read_text(encoding="utf-8")


def catalog() -> dict[str, Any]:
    """Return the generated public-API catalog as a nested dictionary.

    The catalog is produced by ``tools/gen_ai_catalog.py`` and lists every
    public module, class and function with its signature and a docstring
    summary.
    """
    data = files("payton.ai").joinpath("api.json").read_text(encoding="utf-8")
    return cast(dict[str, Any], json.loads(data))


def catalog_json(indent: int | None = 2) -> str:
    """Return the API catalog serialized as JSON text."""
    return json.dumps(catalog(), indent=indent)


def examples() -> list[str]:
    """Return the file names of the curated examples bundled with Payton.

    These are self-contained, verbatim copies of selected repository examples.
    Read one with :func:`example`.
    """
    directory = files("payton").joinpath("examples")
    return sorted(
        entry.name
        for entry in directory.iterdir()
        if entry.name.endswith(".py")
    )


def example(name: str) -> str:
    """Return the source text of a bundled example.

    Parameters
    ----------
    name : str
        File name returned by :func:`examples`, with or without the ``.py``
        suffix (for example ``"basics_04_clock"``).

    Raises
    ------
    KeyError
        If no bundled example matches ``name``.
    """
    if not name.endswith(".py"):
        name = f"{name}.py"
    available = examples()
    if name not in available:
        raise KeyError(
            f"unknown example {name!r}; available: {', '.join(available)}"
        )
    return files("payton").joinpath("examples", name).read_text(encoding="utf-8")

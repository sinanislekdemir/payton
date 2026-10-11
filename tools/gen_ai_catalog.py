"""Generate ``payton/ai/api.json`` -- a machine-readable catalog of the public API.

The catalog is consumed by :mod:`payton.ai` so that AI assistants and tooling can
load Payton's public surface (classes, signatures, docstring summaries) without
parsing the whole source tree.

Usage::

    python tools/gen_ai_catalog.py

The output is deterministic (no timestamps) so it diffs cleanly in git.
"""

import importlib
import inspect
import json
from pathlib import Path
from typing import Any

MODULES = [
    "payton",
    "payton.math",
    "payton.scene",
    "payton.scene.camera",
    "payton.scene.clock",
    "payton.scene.collision",
    "payton.scene.geometry",
    "payton.scene.grid",
    "payton.scene.gui",
    "payton.scene.light",
    "payton.scene.material",
    "payton.scene.shader",
    "payton.tools.mesh",
]

_JSON_TYPES = (int, float, bool, str, list, tuple)


def _version() -> str:
    try:
        from importlib.metadata import version

        return version("Payton")
    except Exception:  # noqa: BLE001
        return "unknown"


def _summary(doc: str | None) -> str:
    """Return the first paragraph of a docstring, collapsed to one line."""
    if not doc:
        return ""
    paragraph: list[str] = []
    for line in doc.strip().splitlines():
        stripped = line.strip()
        if not stripped:
            if paragraph:
                break
            continue
        paragraph.append(stripped)
    return " ".join(paragraph)


def _signature(obj: Any) -> str:
    try:
        return str(inspect.signature(obj))
    except (TypeError, ValueError):
        return ""


def _describe_callable(obj: Any) -> dict[str, Any]:
    return {
        "kind": "function",
        "signature": _signature(obj),
        "summary": _summary(obj.__doc__),
    }


def _describe_class(cls: type) -> dict[str, Any]:
    properties: dict[str, str] = {}
    methods: dict[str, dict[str, str]] = {}

    for name, member in inspect.getmembers(cls):
        if name.startswith("_"):
            continue
        if isinstance(member, property):
            properties[name] = _summary(member.__doc__)
        elif inspect.isfunction(member):
            methods[name] = {
                "signature": _signature(member),
                "summary": _summary(member.__doc__),
            }

    return {
        "kind": "class",
        "signature": _signature(cls),
        "summary": _summary(cls.__doc__),
        "properties": properties,
        "methods": dict(sorted(methods.items())),
    }


def _describe_symbol(obj: Any) -> dict[str, Any] | None:
    if inspect.isclass(obj):
        return _describe_class(obj)
    if inspect.isfunction(obj) or inspect.isbuiltin(obj):
        return _describe_callable(obj)
    if isinstance(obj, _JSON_TYPES) and not inspect.ismodule(obj):
        try:
            json.dumps(obj)
        except (TypeError, ValueError):
            return None
        return {"kind": "constant", "value": obj}
    return None


def collect_module(name: str) -> dict[str, Any]:
    module = importlib.import_module(name)
    exported = getattr(module, "__all__", None)
    declared = exported is not None
    if exported is None:
        exported = [n for n in vars(module) if not n.startswith("_")]

    symbols: dict[str, Any] = {}
    for symbol_name in sorted(exported):
        obj = getattr(module, symbol_name, None)
        if obj is None or inspect.ismodule(obj):
            continue
        # Without an explicit __all__ we can only trust symbols that are
        # actually defined in this module, not imported from elsewhere.
        if not declared and (inspect.isclass(obj) or inspect.isfunction(obj)):
            if getattr(obj, "__module__", None) != name:
                continue
        described = _describe_symbol(obj)
        if described is not None:
            symbols[symbol_name] = described

    return {
        "doc": _summary(module.__doc__),
        "symbols": symbols,
    }


def build() -> dict[str, Any]:
    catalog: dict[str, Any] = {
        "package": "payton",
        "version": _version(),
        "modules": {},
    }
    for name in MODULES:
        catalog["modules"][name] = collect_module(name)
    return catalog


def main() -> int:
    target = Path(__file__).resolve().parent.parent / "payton" / "ai" / "api.json"
    catalog = build()
    target.write_text(json.dumps(catalog, indent=2) + "\n", encoding="utf-8")
    symbol_count = sum(
        len(m["symbols"]) for m in catalog["modules"].values()
    )
    print(f"Wrote {target} ({len(catalog['modules'])} modules, {symbol_count} symbols)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

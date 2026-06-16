import json
import math
from copy import deepcopy
from pathlib import Path

import sdl2

from payton.math.functions import distance
from payton.math.matrix import IDENTITY_MATRIX
from payton.scene import Scene, THEME_STUDIO
from payton.scene.controller import BaseController
from payton.scene.geometry import Cube, Cylinder, Plane, Sphere

from designer_io import export_scene, load_scene, save_scene
from ui import build_designer_ui


PRIMITIVE_DEFAULTS = {
    "cube": {
        "kind": "cube",
        "params": {"width": 1.0, "depth": 1.0, "height": 1.0},
        "position": [0.0, 0.0, 0.5],
        "rotation": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "color": [0.4, 0.7, 1.0],
    },
    "sphere": {
        "kind": "sphere",
        "params": {"radius": 0.5, "parallels": 12, "meridians": 12},
        "position": [0.0, 0.0, 0.5],
        "rotation": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "color": [1.0, 0.6, 0.3],
    },
    "cylinder": {
        "kind": "cylinder",
        "params": {"bottom_radius": 0.5, "top_radius": 0.5, "height": 1.0, "meridians": 16},
        "position": [0.0, 0.0, 0.5],
        "rotation": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "color": [0.5, 0.9, 0.5],
    },
    "plane": {
        "kind": "plane",
        "params": {"width": 1.5, "height": 1.5},
        "position": [0.0, 0.0, 0.02],
        "rotation": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
        "color": [0.9, 0.9, 0.9],
    },
}


class DesignerController(BaseController):
    def keyboard(self, event, scene):
        if event.type != sdl2.SDL_KEYUP:
            return False

        key = event.key.keysym.sym
        if key == sdl2.SDLK_1:
            scene.add_cube()
            return True
        if key == sdl2.SDLK_2:
            scene.add_sphere()
            return True
        if key == sdl2.SDLK_3:
            scene.add_cylinder()
            return True
        if key == sdl2.SDLK_4:
            scene.add_plane()
            return True
        if key in [sdl2.SDLK_DELETE, sdl2.SDLK_BACKSPACE]:
            scene.delete_selected()
            return True
        if scene._ctrl_down and key == sdl2.SDLK_s:
            scene.save_scene_file()
            return True
        if scene._ctrl_down and key == sdl2.SDLK_l:
            scene.load_scene_file()
            return True
        if scene._ctrl_down and key == sdl2.SDLK_e:
            scene.export_scene_file()
            return True
        return False

    def mouse(self, event, scene):
        return False


class DesignerScene(Scene):
    def __init__(self):
        super().__init__(width=1440, height=900, on_select=self.select_from_hits, theme=THEME_STUDIO)
        self.grid.visible = True
        self.active_camera.position = [8.0, -10.0, 8.0]
        self.active_camera.target = [0.0, 0.0, 1.0]

        self.design_specs = {}
        self.primitive_counts = {name: 0 for name in PRIMITIVE_DEFAULTS}
        self.selected_name = None

        self.controller.add_controller(DesignerController())
        self.hud, self.widgets = build_designer_ui(self)
        self.add_object("designer_ui", self.hud)
        self.set_status("Ready")
        self.update_inspector()

    def add_cube(self):
        self.add_primitive("cube")

    def add_sphere(self):
        self.add_primitive("sphere")

    def add_cylinder(self):
        self.add_primitive("cylinder")

    def add_plane(self):
        self.add_primitive("plane")

    def set_status(self, message):
        if len(message) > 30:
            message = message[:27] + "..."
        self.widgets["status"].label = message

    def _next_name(self, kind):
        index = self.primitive_counts[kind]
        while f"{kind}_{index}" in self.design_specs:
            index += 1
        self.primitive_counts[kind] = index + 1
        return f"{kind}_{index}"

    def _spawn_position(self, kind):
        if self.selected_name in self.design_specs:
            current = self.design_specs[self.selected_name]["position"]
            z = current[2]
            if kind == "plane":
                z = 0.02
            return [current[0] + 1.75, current[1], z]

        count = len(self.design_specs)
        col = count % 4
        row = count // 4
        z = 0.5 if kind != "plane" else 0.02
        return [-3.0 + (col * 2.0), -3.0 + (row * 2.0), z]

    def _apply_spec_transform(self, obj, spec):
        obj.matrix = deepcopy(IDENTITY_MATRIX)
        obj.scale(*spec["scale"])
        obj.rotate_around_x(math.radians(spec["rotation"][0]))
        obj.rotate_around_y(math.radians(spec["rotation"][1]))
        obj.rotate_around_z(math.radians(spec["rotation"][2]))
        obj.position = list(spec["position"])
        obj.material.color = list(spec["color"])

    def _build_object(self, spec):
        kind = spec["kind"]
        params = dict(spec["params"])
        if kind == "cube":
            obj = Cube(**params)
        elif kind == "sphere":
            obj = Sphere(**params)
        elif kind == "cylinder":
            obj = Cylinder(**params)
        elif kind == "plane":
            obj = Plane(**params)
        else:
            raise ValueError(f"Unsupported primitive type: {kind}")

        self._apply_spec_transform(obj, spec)
        return obj

    def _store_object(self, name, spec):
        self.design_specs[name] = deepcopy(spec)
        obj = self._build_object(spec)
        self.add_object(name, obj)

    def add_primitive(self, kind):
        spec = deepcopy(PRIMITIVE_DEFAULTS[kind])
        spec["position"] = self._spawn_position(kind)
        name = self._next_name(kind)
        self._store_object(name, spec)
        self.select_object(name)
        self.set_status(f"Created {name}")

    def _refresh_selection_visuals(self):
        for name, spec in self.design_specs.items():
            obj = self.objects.get(name)
            if obj is None:
                continue
            color = list(spec["color"])
            if name == self.selected_name:
                color = [1.0, 0.85, 0.2]
            obj.material.color = color

    def select_from_hits(self, hits):
        candidates = [obj for obj in hits if obj.name in self.design_specs]
        if not candidates:
            return

        nearest = min(
            candidates,
            key=lambda obj: distance(obj.position, self.active_camera.position),
        )
        self.select_object(nearest.name)

    def select_object(self, name):
        if name not in self.design_specs:
            self.clear_selection()
            return
        self.selected_name = name
        self._refresh_selection_visuals()
        self.update_inspector()
        self.set_status(f"Selected {name}")

    def clear_selection(self):
        self.selected_name = None
        self._refresh_selection_visuals()
        self.update_inspector()
        self.set_status("Selection cleared")

    def delete_selected(self):
        if self.selected_name is None:
            self.set_status("Nothing is selected.")
            return

        name = self.selected_name
        if name in self.objects:
            del self.objects[name]
        if name in self.design_specs:
            del self.design_specs[name]
        self.selected_name = None
        self._refresh_selection_visuals()
        self.update_inspector()
        self.set_status(f"Deleted {name}")

    def _selected_spec(self):
        if self.selected_name is None:
            self.set_status("Select an object first.")
            return None
        return self.design_specs[self.selected_name]

    def _parse_vector(self, text, label):
        parts = [part.strip() for part in text.split(",")]
        if len(parts) != 3:
            raise ValueError(f"{label} must contain exactly three comma-separated numbers.")
        return [float(part) for part in parts]

    def _apply_selected_spec(self):
        spec = self._selected_spec()
        if spec is None:
            self.update_inspector()
            return
        self._apply_spec_transform(self.objects[self.selected_name], spec)
        self._refresh_selection_visuals()
        self.update_inspector()

    def set_position_text(self, text):
        spec = self._selected_spec()
        if spec is None:
            return
        try:
            spec["position"] = self._parse_vector(text, "Position")
        except ValueError as exc:
            self.set_status(str(exc))
            self.update_inspector()
            return
        self._apply_selected_spec()
        self.set_status(f"Updated position for {self.selected_name}")

    def set_rotation_text(self, text):
        spec = self._selected_spec()
        if spec is None:
            return
        try:
            spec["rotation"] = self._parse_vector(text, "Rotation")
        except ValueError as exc:
            self.set_status(str(exc))
            self.update_inspector()
            return
        self._apply_selected_spec()
        self.set_status(f"Updated rotation for {self.selected_name}")

    def set_scale_text(self, text):
        spec = self._selected_spec()
        if spec is None:
            return
        try:
            scale = self._parse_vector(text, "Scale")
        except ValueError as exc:
            self.set_status(str(exc))
            self.update_inspector()
            return

        if any(value <= 0 for value in scale):
            self.set_status("Scale values must all be greater than zero.")
            self.update_inspector()
            return

        spec["scale"] = scale
        self._apply_selected_spec()
        self.set_status(f"Updated scale for {self.selected_name}")

    def set_color_text(self, text):
        spec = self._selected_spec()
        if spec is None:
            return
        try:
            color = self._parse_vector(text, "Color")
        except ValueError as exc:
            self.set_status(str(exc))
            self.update_inspector()
            return

        if any(value < 0 or value > 1 for value in color):
            self.set_status("Color channels must stay between 0.0 and 1.0.")
            self.update_inspector()
            return

        spec["color"] = color
        self._apply_selected_spec()
        self.set_status(f"Updated color for {self.selected_name}")

    def _set_spec_axis(self, key, axis, value):
        spec = self._selected_spec()
        if spec is None:
            return
        spec[key][axis] = value
        self._apply_selected_spec()

    def set_position_x(self, val):
        self._set_spec_axis("position", 0, val)

    def set_position_y(self, val):
        self._set_spec_axis("position", 1, val)

    def set_position_z(self, val):
        self._set_spec_axis("position", 2, val)

    def set_rotation_x(self, val):
        self._set_spec_axis("rotation", 0, val)

    def set_rotation_y(self, val):
        self._set_spec_axis("rotation", 1, val)

    def set_rotation_z(self, val):
        self._set_spec_axis("rotation", 2, val)

    def set_scale_x(self, val):
        self._set_spec_axis("scale", 0, val)

    def set_scale_y(self, val):
        self._set_spec_axis("scale", 1, val)

    def set_scale_z(self, val):
        self._set_spec_axis("scale", 2, val)

    def _format_vector(self, values):
        return ", ".join(f"{value:.3f}" for value in values)

    def _update_slider_values(self, spec):
        for axis, idx in [("x", 0), ("y", 1), ("z", 2)]:
            self.widgets[f"slider_p_{axis}"].value = spec["position"][idx]
            self.widgets[f"slider_r_{axis}"].value = spec["rotation"][idx]
            self.widgets[f"slider_s_{axis}"].value = spec["scale"][idx]

    def update_inspector(self):
        if self.selected_name is None:
            self.widgets["selected_name"].label = "None"
            self.widgets["selected_type"].label = "-"
            self.widgets["position"].value = "0.000, 0.000, 0.500"
            self.widgets["rotation"].value = "0.000, 0.000, 0.000"
            self.widgets["scale"].value = "1.000, 1.000, 1.000"
            self.widgets["color"].value = "0.400, 0.700, 1.000"
            for axis in ["x", "y", "z"]:
                self.widgets[f"slider_p_{axis}"].value = 0.0
                self.widgets[f"slider_r_{axis}"].value = 0.0
                self.widgets[f"slider_s_{axis}"].value = 1.0
            return

        spec = self.design_specs[self.selected_name]
        self.widgets["selected_name"].label = self.selected_name
        self.widgets["selected_type"].label = spec["kind"]
        self.widgets["position"].value = self._format_vector(spec["position"])
        self.widgets["rotation"].value = self._format_vector(spec["rotation"])
        self.widgets["scale"].value = self._format_vector(spec["scale"])
        self.widgets["color"].value = self._format_vector(spec["color"])
        self._update_slider_values(spec)

    def _scene_filename(self):
        return self.widgets["scene_file"].value.strip()

    def _export_filename(self):
        return self.widgets["export_file"].value.strip()

    def save_scene_file(self):
        try:
            path = save_scene(self.design_specs, self._scene_filename())
        except OSError as exc:
            self.set_status(f"Save failed: {exc}")
            return
        self.set_status(f"Saved {Path(path).name}")

    def _clear_objects(self):
        for name in list(self.design_specs):
            if name in self.objects:
                del self.objects[name]
        self.design_specs = {}
        self.selected_name = None

    def _rebuild_counts(self):
        self.primitive_counts = {name: 0 for name in PRIMITIVE_DEFAULTS}
        for name, spec in self.design_specs.items():
            kind = spec["kind"]
            prefix = f"{kind}_"
            if name.startswith(prefix):
                suffix = name[len(prefix) :]
                if suffix.isdigit():
                    self.primitive_counts[kind] = max(self.primitive_counts[kind], int(suffix) + 1)
                    continue
            self.primitive_counts[kind] += 1

    def load_scene_file(self):
        try:
            path, objects = load_scene(self._scene_filename())
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.set_status(f"Load failed: {exc}")
            return

        self._clear_objects()
        for name, spec in objects.items():
            self._store_object(name, spec)
        self._rebuild_counts()
        self._refresh_selection_visuals()
        self.update_inspector()
        self.set_status(f"Loaded {Path(path).name}")

    def export_scene_file(self):
        meshes = []
        for spec in self.design_specs.values():
            meshes.append(self._build_object(spec))

        try:
            path = export_scene(meshes, self._export_filename())
        except (OSError, ValueError) as exc:
            self.set_status(f"Export failed: {exc}")
            return
        self.set_status(f"Exported {Path(path).name}")

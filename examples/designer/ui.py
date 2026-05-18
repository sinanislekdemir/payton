from payton.scene.gui import Button, EditBox, Hud, Window, WindowAlignment

from designer_io import DEFAULT_EXPORT_FILE, DEFAULT_SCENE_FILE


def build_designer_ui(scene):
    hud = Hud()
    widgets = {}

    actions = Window(
        "Designer",
        width=280,
        height=900,
        align=WindowAlignment.LEFT,
        theme=scene.ui_theme,
    )
    widgets["actions_window"] = actions

    top = 40
    step = 40
    button_width = 250

    buttons = [
        ("add_cube", "Add Cube", scene.add_cube),
        ("add_sphere", "Add Sphere", scene.add_sphere),
        ("add_cylinder", "Add Cylinder", scene.add_cylinder),
        ("add_plane", "Add Plane", scene.add_plane),
        ("clear_selection", "Clear Selection", scene.clear_selection),
        ("delete_selected", "Delete Selected", scene.delete_selected),
    ]

    for name, label, callback in buttons:
        actions.add_child(
            name,
            Button(
                label,
                width=button_width,
                height=30,
                left=15,
                top=top,
                on_click=callback,
            ),
        )
        top += step

    actions.add_child(
        "scene_file_label",
        Button("Scene File", width=button_width, height=30, left=15, top=top),
    )
    top += step
    widgets["scene_file"] = EditBox(
        DEFAULT_SCENE_FILE,
        width=button_width,
        height=30,
        left=15,
        top=top,
    )
    actions.add_child("scene_file", widgets["scene_file"])
    top += step

    actions.add_child(
        "save_scene",
        Button("Save Scene", width=button_width, height=30, left=15, top=top, on_click=scene.save_scene_file),
    )
    top += step
    actions.add_child(
        "load_scene",
        Button("Load Scene", width=button_width, height=30, left=15, top=top, on_click=scene.load_scene_file),
    )
    top += step

    actions.add_child(
        "export_file_label",
        Button("OBJ Export File", width=button_width, height=30, left=15, top=top),
    )
    top += step
    widgets["export_file"] = EditBox(
        DEFAULT_EXPORT_FILE,
        width=button_width,
        height=30,
        left=15,
        top=top,
    )
    actions.add_child("export_file", widgets["export_file"])
    top += step
    actions.add_child(
        "export_scene",
        Button("Export OBJ", width=button_width, height=30, left=15, top=top, on_click=scene.export_scene_file),
    )
    top += step

    actions.add_child(
        "status_label",
        Button("Status", width=button_width, height=30, left=15, top=top),
    )
    top += step
    widgets["status"] = Button(
        "Ready",
        width=button_width,
        height=30,
        left=15,
        top=top,
    )
    actions.add_child("status", widgets["status"])
    top += step

    shortcut_lines = [
        ("shortcuts_1", "1-4 Add  |  Del Delete"),
        ("shortcuts_2", "Ctrl+S Save  |  Ctrl+L Load"),
        ("shortcuts_3", "Ctrl+E Export OBJ"),
    ]
    for name, label in shortcut_lines:
        widgets[name] = Button(
            label,
            width=button_width,
            height=30,
            left=15,
            top=top,
        )
        actions.add_child(name, widgets[name])
        top += step

    inspector = Window(
        "Selection",
        width=320,
        height=520,
        align=WindowAlignment.RIGHT,
        theme=scene.ui_theme,
    )
    widgets["inspector_window"] = inspector

    inspector.add_child(
        "selected_label",
        Button("Selected Object", width=290, height=30, left=15, top=40),
    )
    widgets["selected_name"] = Button(
        "None",
        width=290,
        height=30,
        left=15,
        top=80,
    )
    inspector.add_child("selected_name", widgets["selected_name"])

    inspector.add_child(
        "selected_type_label",
        Button("Primitive Type", width=290, height=30, left=15, top=120),
    )
    widgets["selected_type"] = Button(
        "-",
        width=290,
        height=30,
        left=15,
        top=160,
    )
    inspector.add_child("selected_type", widgets["selected_type"])

    inspector.add_child(
        "position_label",
        Button("Position (x, y, z)", width=290, height=30, left=15, top=200),
    )
    widgets["position"] = EditBox(
        "0.000, 0.000, 0.500",
        width=290,
        height=30,
        left=15,
        top=240,
        on_change=scene.set_position_text,
    )
    inspector.add_child("position", widgets["position"])

    inspector.add_child(
        "rotation_label",
        Button("Rotation Degrees (x, y, z)", width=290, height=30, left=15, top=280),
    )
    widgets["rotation"] = EditBox(
        "0.000, 0.000, 0.000",
        width=290,
        height=30,
        left=15,
        top=320,
        on_change=scene.set_rotation_text,
    )
    inspector.add_child("rotation", widgets["rotation"])

    inspector.add_child(
        "scale_label",
        Button("Scale (x, y, z)", width=290, height=30, left=15, top=360),
    )
    widgets["scale"] = EditBox(
        "1.000, 1.000, 1.000",
        width=290,
        height=30,
        left=15,
        top=400,
        on_change=scene.set_scale_text,
    )
    inspector.add_child("scale", widgets["scale"])

    inspector.add_child(
        "color_label",
        Button("Color RGB 0-1", width=290, height=30, left=15, top=440),
    )
    widgets["color"] = EditBox(
        "0.400, 0.700, 1.000",
        width=290,
        height=30,
        left=15,
        top=480,
        on_change=scene.set_color_text,
    )
    inspector.add_child("color", widgets["color"])

    hud.add_child("actions", actions)
    hud.add_child("inspector", inspector)
    return hud, widgets

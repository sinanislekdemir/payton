from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.shader import Shader

fragment_shader = """
#version 330 core
out vec4 FragColor;

/* Below uniforms are redundant,
 they exists to suppress the error logs */
uniform vec3 light_pos[100]; // assume 100 lights max.
uniform vec3 light_color[100];
uniform int LIGHT_COUNT;

uniform vec3 object_color;
uniform int material_mode;
uniform float opacity;

uniform sampler2D tex_unit;

void main() {
    FragColor = vec4(1.0, 0.0, 0.0, 0.5);
}
"""

scene = Scene()
cube = Cube()
cube2 = Cube()
shader = Shader(fragment=fragment_shader)
cube.material.shader = shader
scene.add_object("cube", cube)
scene.add_object("cube2", cube2)

cube2.position = [0, 0, 2]
scene.run()

import ctypes
import logging
from typing import Any, Dict, List, Optional

import numpy as np  # type: ignore
from OpenGL.GL import (
    GL_FALSE,
    GL_FRAGMENT_SHADER,
    GL_GEOMETRY_SHADER,
    GL_PROGRAM_POINT_SIZE,
    GL_TRUE,
    GL_VERTEX_SHADER,
    glDisable,
    glEnable,
    glGetUniformLocation,
    glUniform1f,
    glUniform1i,
    glUniform3fv,
    glUniform4fv,
    glUniformMatrix4fv,
    glUseProgram,
    shaders,
)

DEFAULT_SHADER = "default"
PARTICLE_SHADER = "particle"
SHADOW_SHADER = "depth"

depth_fragment_shader = """
#version 330 core
in vec4 FragPos;

uniform float far_plane;
uniform vec3 light_pos[100];

void main()
{
    float lightDistance = length(FragPos.xyz - light_pos[0]);
    lightDistance = lightDistance / far_plane;
    gl_FragDepth = lightDistance;
}
"""

depth_vertex_shader = """
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;

void main()
{
    gl_Position = model * vec4(aPos, 1.0);
}
"""

depth_geometry_shader = """
#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices=18) out;

uniform mat4 shadowMatrices[6];

out vec4 FragPos; // FragPos from GS (output per emitvertex)

void main()
{
    for(int face = 0; face < 6; ++face)
    {
        gl_Layer = face;
        for(int i = 0; i < 3; ++i) // for each triangle's vertices
        {
            FragPos = gl_in[i].gl_Position;
            gl_Position = shadowMatrices[face] * FragPos;
            EmitVertex();
        }
        EndPrimitive();
    }
}
"""


particle_geometry_shader = """
#version 330 core

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform int view_mode;
uniform float particle_size;

layout (points) in;
layout (triangle_strip, max_vertices=4) out;

out vec2 varTex;

void main() {
    vec4 center = view * model * gl_in[0].gl_Position;
    varTex = vec2( 0, 0 );
    gl_Position = projection * (center + vec4(particle_size, particle_size, 0, 0));
    EmitVertex();

    varTex = vec2( 1, 0  );
    gl_Position = projection * (center + vec4(particle_size, -particle_size, 0, 0));
    EmitVertex();

    varTex = vec2( 0, 1 );
    gl_Position = projection * (center + vec4(-particle_size, particle_size, 0, 0));
    EmitVertex();

    varTex = vec2( 1, 1 );
    gl_Position = projection * (center + vec4(-particle_size, -particle_size, 0, 0));
    EmitVertex();
}

"""


particle_fragment_shader = """
#version 330

uniform sampler2D tex_unit;
uniform vec3 object_color;
uniform float opacity;

in vec2 varTex;

layout(location = 0) out vec4 outFragColor;

void main()
{
    vec4 tex_color;
    tex_color = texture( tex_unit, varTex );

    outFragColor = tex_color * vec4(object_color, opacity);
}
"""


particle_vertex_shader = """
#version 330 core
layout ( location = 0 ) in vec3 position;
layout ( location = 1 ) in vec3 normal;
layout ( location = 2 ) in vec2 texCoords;
layout ( location = 3 ) in vec3 colors; // optional

out vec2 tex_coords;
out vec3 frag_color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform int view_mode;

out vec3 l_fragpos;
out vec3 l_normal;

void main()
{
    gl_Position = vec4(position, 1.0f);
}
"""  # type: str


default_fragment_shader = """
#version 330 core
out vec4 FragColor;

in vec2 tex_coords;
in vec3 frag_color;

in vec3 l_fragpos;
in vec3 l_normal;

uniform vec3 light_pos[100]; // assume 100 lights max.
uniform vec3 light_color[100];
uniform vec3 camera_pos;
uniform bool shadow_enabled;
uniform int samples;

uniform int LIGHT_COUNT;

uniform vec3 object_color;
uniform int material_mode;
uniform float opacity;
uniform float far_plane;
uniform int lit;

uniform sampler2D tex_unit;
uniform samplerCube depthMap;

// array of offset direction for sampling
vec3 gridSamplingDisk[20] = vec3[]
(
   vec3(1, 1,  1), vec3( 1, -1,  1), vec3(-1, -1,  1), vec3(-1, 1,  1),
   vec3(1, 1, -1), vec3( 1, -1, -1), vec3(-1, -1, -1), vec3(-1, 1, -1),
   vec3(1, 1,  0), vec3( 1, -1,  0), vec3(-1, -1,  0), vec3(-1, 1,  0),
   vec3(1, 0,  1), vec3(-1,  0,  1), vec3( 1,  0, -1), vec3(-1, 0, -1),
   vec3(0, 1,  1), vec3( 0, -1,  1), vec3( 0, -1, -1), vec3( 0, 1, -1)
);

float ShadowCalculation(vec3 fragPos)
{
    vec3 fragToLight = fragPos - light_pos[0];
    float currentDepth = length(fragToLight);
    float shadow = 0.0;
    float bias = 0.15;

    float viewDistance = length(camera_pos - fragPos);
    float diskRadius = (1.0 + (viewDistance / far_plane)) / 25.0;

    for(int i = 0; i < samples; ++i)
    {
        float closestDepth = texture(depthMap, fragToLight +
                                     gridSamplingDisk[i] * diskRadius).r;
        closestDepth *= far_plane;
        if(currentDepth - bias > closestDepth)
             shadow += 1.0;
    }

    return shadow / float(samples);
}

void main()
{
    vec3 color;
    float alpha;
    if (material_mode == 0 || material_mode == 2) {
        color = object_color;
        alpha = opacity;
    }
    if (material_mode == 1 || material_mode == 3) {
        color = texture(tex_unit, tex_coords).rgb;
        alpha = texture(tex_unit, tex_coords).a;
    }
    if (material_mode == 4) {
        color = frag_color;
        alpha = opacity;
    }

    if (lit == 0) {
        FragColor = vec4(color, alpha);
    } else {
        vec3 norm = normalize(l_normal);
        for (int i = 0; i < LIGHT_COUNT; i++) {
            vec3 ambient = 0.3 * light_color[i];

            // diffuse
            vec3 norm = normalize(l_normal);
            vec3 lightDir = normalize(light_pos[i] - l_fragpos);
            float diff = max(dot(norm, lightDir), 0.0);
            vec3 diffuse = diff * light_color[i];
            float spec = 0.0;
            vec3 halfwayDir = normalize(lightDir + camera_pos);
            spec = pow(max(dot(norm, halfwayDir), 0.0), 64.0);

            vec3 specular = spec * light_color[i];
            float shadow = shadow_enabled ? ShadowCalculation(l_fragpos) : 0.0;
            vec3 lighting = (ambient + (1.0 - shadow) *
                             (diffuse + specular)) * color;

            FragColor = vec4(lighting, alpha);
        }
    }
}"""  # type: str


default_vertex_shader = """
#version 330 core
layout ( location = 0 ) in vec3 position;
layout ( location = 1 ) in vec3 normal;
layout ( location = 2 ) in vec2 texCoords;
layout ( location = 3 ) in vec3 colors; // optional

out vec2 tex_coords;
out vec3 frag_color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform int view_mode;

out vec3 l_fragpos;
out vec3 l_normal;

void main()
{
    l_fragpos = vec3(model * vec4(position, 1.0));
    l_normal = mat3(transpose(inverse(model))) * normal;

    if (view_mode == 0) {
        gl_Position = projection * view * model * vec4(position, 1.0f);
    }
    if (view_mode == 1) {
        gl_Position = (projection * (vec4(position, 1.0f)
                        + vec4(model[3].xyz, 0.0f)));
    }

    gl_PointSize = 5.0;

    tex_coords = texCoords;
    frag_color = colors;
}
"""  # type: str


background_vertex_shader = """
#version 330 core
out vec2 v_uv;

void main()
{
  // uint idx = gl_VertexID;
  gl_Position = (vec4( gl_VertexID & 1, gl_VertexID >> 1, 0.0, 0.5 )
                 * 4.0 - 1.0);
  v_uv = vec2( gl_Position.xy * 0.5 + 0.5 );
}
"""  # type: str

background_fragment_shader = """
#version 330 core
uniform vec4 top_color;
uniform vec4 bot_color;
in vec2 v_uv;

out vec4 frag_color;
void main()
{
  frag_color = top_color * (1 - v_uv.y) + bot_color * v_uv.y;
}
"""  # type: str


class Shader(object):
    NO_LIGHT_COLOR = 0  # type: int
    NO_LIGHT_TEXTURE = 1  # type: int
    LIGHT_COLOR = 2  # type: int
    LIGHT_TEXTURE = 3  # type: int
    PER_VERTEX_COLOR = 4  # type:int
    HUD = 5  # type: int
    DEPTH_CALC = 6  # type: int

    def __init__(
        self,
        fragment: str = default_fragment_shader,
        vertex: str = default_vertex_shader,
        geometry: str = "",
        variables: Optional[List[str]] = None,
        **kwargs: Any,
    ):
        self.fragment_shader_source = fragment
        self.vertex_shader_source = vertex
        self.geometry_shader_source = geometry

        self.variables: List[str] = [] if variables is None else variables
        self._stack: Dict[str, int] = {}  # Variable stack.
        self._mode: int = self.NO_LIGHT_COLOR  # Lightless color material
        self._depth_shader = False

        self.program: int = -1

    def build(self) -> int:
        vertex_shader = shaders.compileShader(self.vertex_shader_source, GL_VERTEX_SHADER)
        fragment_shader = shaders.compileShader(self.fragment_shader_source, GL_FRAGMENT_SHADER)
        geometry_shader = None
        if self.geometry_shader_source != "":
            geometry_shader = shaders.compileShader(self.geometry_shader_source, GL_GEOMETRY_SHADER)

        if geometry_shader is not None:
            self.program = shaders.compileProgram(vertex_shader, fragment_shader, geometry_shader, validate=False)
        else:
            self.program = shaders.compileProgram(vertex_shader, fragment_shader, validate=False)

        return self.program

    def use(self) -> bool:
        if self.program == -1:
            logging.error("Shader not compiled")
            return False
        glUseProgram(self.program)
        glEnable(GL_PROGRAM_POINT_SIZE)
        return True

    def end(self) -> None:
        glUseProgram(0)
        glDisable(GL_PROGRAM_POINT_SIZE)

    def set_matrix4x4_np(self, variable: str, value: np.ndarray, transpose: bool = False) -> bool:
        g_transpose = GL_TRUE if transpose else GL_FALSE
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniformMatrix4fv(
            location, 1, g_transpose, value,
        )
        return True

    def get_location(self, variable: str) -> int:
        if variable in self._stack:
            return self._stack[variable]
        location = glGetUniformLocation(self.program, variable)

        self._stack[variable] = location
        return location

    def set_matrix4x4(self, variable: str, value: List[List[float]], transpose: bool = False) -> None:
        self.set_matrix4x4_np(variable, np.array(value, np.float32), transpose)

    def set_vector3_array_np(self, variable: str, value: np.ndarray, count: int) -> bool:
        value = value.flatten()
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform3fv(location, count, value)
        return True

    def set_vector3_np(self, variable: str, value: np.ndarray) -> bool:
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform3fv(location, 1, value)
        return True

    def set_vector4_np(self, variable: str, value: np.ndarray) -> bool:
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform4fv(location, 1, value)
        return True

    def set_vector3(self, variable: str, value: List[float]) -> None:
        self.set_vector3_np(variable, np.array(value, dtype=np.float32))

    def set_int(self, variable: str, value: int) -> bool:
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniform1i(location, ctypes.c_int(value))
        return True

    def set_float(self, variable: str, value: float) -> bool:
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniform1f(location, ctypes.c_float(value))
        return True

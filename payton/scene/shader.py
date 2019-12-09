"""Payton Shader

Payton Shader is an OpenGL 3.3 Compatible GLSL (330) module. Basic principle
is to be be able to compile vertex and fragment shaders and hold basic
ready-to-use shaders.

A usual user should not be aware of this shading system. Basic shaders defined
should be enough for simple simulations. Our intention is not to create a
graphics library. We want to create a simulation platform and let developers
to focus on mathematics rather than OpenGL Stuff.

Also, keep in mind that, this is NOT a generic-purpose shader class which can
be used at common OpenGL apps. This shader class is just enough of what is
essential to Payton Library. That is all.

"""

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
        closestDepth *= 100.0;
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
  frag_color = bot_color * (1 - v_uv.y) + top_color * v_uv.y;
}
"""  # type: str


class Shader(object):
    """Payton shader class. Creates shaders / programs

    For better performance, Shader aims to reduce the number of
    `glGetUniformlocation` function calls. For this purpose, `__init__` and
    `build` functions accept a list of variable names as `variables` argument.

    If variable not found in `self.variables` then the system will try to
    locate the variable location and store it in `self.variables` for
    future reference.

    This way, Shader goes through this list of variables after compiling the
    shader program and stores their locations in memory for future use.

    If you provide the list of variable names in build function call then
    it will overwrite the existing list.
    """

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
        """Initialize Shader.

        Args:
          fragment: Fragment shader code
          vertex: Vertex shader code
          variables: List of in/out/uniform variable names.
        """
        self.fragment_shader_source = fragment
        self.vertex_shader_source = vertex
        self.geometry_shader_source = geometry

        self.variables: List[str] = [] if variables is None else variables
        self._stack: Dict[str, int] = {}  # Variable stack.
        self._mode: int = self.NO_LIGHT_COLOR  # Lightless color material
        self._depth_shader = False

        self.program: int = -1

    def build(self) -> int:
        """Build GLSL Shader
        Compile shaders and compile glsl program
        Args:
          variables: List of in/out/uniform variable names.
        Return:
            self.program
        """
        vertex_shader = shaders.compileShader(
            self.vertex_shader_source, GL_VERTEX_SHADER
        )
        fragment_shader = shaders.compileShader(
            self.fragment_shader_source, GL_FRAGMENT_SHADER
        )
        geometry_shader = None
        if self.geometry_shader_source != "":
            geometry_shader = shaders.compileShader(
                self.geometry_shader_source, GL_GEOMETRY_SHADER
            )

        if geometry_shader is not None:
            self.program = shaders.compileProgram(
                vertex_shader, fragment_shader, geometry_shader
            )
        else:
            self.program = shaders.compileProgram(
                vertex_shader, fragment_shader
            )

        return self.program

    def use(self) -> bool:
        """Use GLSL Program.
        This method should be called before setting any variables or rendering
        any entities
        """
        if self.program == -1:
            logging.error("Shader not compiled")
            return False
        glUseProgram(self.program)
        glEnable(GL_PROGRAM_POINT_SIZE)
        return True

    def end(self) -> None:
        """Set the active GLSL program to 0
        """
        glUseProgram(0)
        glDisable(GL_PROGRAM_POINT_SIZE)

    def set_matrix4x4_np(
        self, variable: str, value: np.ndarray, transpose: bool = False
    ) -> bool:
        """Set 4x4 Numpy matrix value

        Some elements like Observer and Light can pass their matrices directly
        as numpy array to reduce number of object conversions. This is the
        ideal way if possible.

        Args:
          variable: Variable name to set
          value: Matrix to set. (Numpy matrix)
          transpose: Transpose matrix.
        """
        g_transpose = GL_TRUE if transpose else GL_FALSE
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniformMatrix4fv(
            location,
            1,
            g_transpose,
            np.asfortranarray(value, dtype=np.float32),
        )
        return True

    def get_location(self, variable: str) -> int:
        """Get location of a given variable within the shader"""
        if variable in self._stack:
            return self._stack[variable]
        location = glGetUniformLocation(self.program, variable)

        self._stack[variable] = location
        return location

    def set_matrix4x4(
        self, variable: str, value: List[List[float]], transpose: bool = False
    ) -> None:
        """Set 4x4 Matrix value

        This method will simply convert `value` into numpy array and call
        `payton.scene.shader.Shader.set_matrix4x4_np` method.

        Args:
          variable: Variable name to set
          value: Matrix to set. (Numpy matrix)
          transpose: Transpose matrix.
        """
        self.set_matrix4x4_np(variable, np.array(value, np.float32), transpose)

    def set_vector3_array_np(
        self, variable: str, value: np.ndarray, count: int
    ) -> bool:
        """Set array of vec3 as numpy array value"""
        value = value.flatten()
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform3fv(location, count, value)
        return True

    def set_vector3_np(self, variable: str, value: np.ndarray) -> bool:
        """Set Vector 3 as numpy array value

        Some elements like Light or pre-set materials can pass their
        vertices directly as numpy array to reduce number of object
        conversions.

        Args:
          variable: Variable name to set
          value: Vector 3 to set. (Numpy array with 3 elemenets)
        """
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform3fv(location, 1, value)
        return True

    def set_vector4_np(self, variable: str, value: np.ndarray) -> bool:
        """Set Vector 4 as numpy array value

        Some elements like Light or pre-set materials can pass their vertices
        directly as numpy array to reduce number of object conversions.

        Args:
          variable: Variable name to set
          value: Vector 4 to set. (Numpy array with 4 elemenets)
        """
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform4fv(location, 1, value)
        return True

    def set_vector3(self, variable: str, value: List[float]) -> None:
        """Set Vector 3 as array value

        This method will simply convert `value` into numpy array and call
        `payton.scene.shader.Shader.set_vector3_np` method.

        Args:
          variable: Variable name to set
          value: Vector 3 to set. (Numpy array with 3 elemenets)
        """
        self.set_vector3_np(variable, np.array(value, dtype=np.float32))

    def set_int(self, variable: str, value: int) -> bool:
        """Set Integer value

        Args:
          variable: Variable name to set
          value: Integer value to set
        """
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniform1i(location, ctypes.c_int(value))
        return True

    def set_float(self, variable: str, value: float) -> bool:
        """Set Float value

        Args:
          variable: Variable name to set
          value: Floaf value to set
        """
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniform1f(location, ctypes.c_float(value))
        return True

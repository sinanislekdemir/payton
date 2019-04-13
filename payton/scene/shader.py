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

import numpy as np # We need C floats
import logging

from OpenGL.GL import (GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, glEnable,
                       glDisable, GL_PROGRAM_POINT_SIZE,
                       glGetUniformLocation, glUseProgram, GL_TRUE, GL_FALSE,
                       glUniformMatrix4fv, glUniform3fv, glUniform4fv)

from OpenGL.GL import shaders

default_fragment_shader = """
#version 330 core
out vec4 FragColor;

in vec3 l_normal;
in vec3 l_fragpos;

uniform vec3 light_pos;
uniform vec3 light_color;
uniform vec3 object_color;

void main()
{
    // ambient
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * light_color;

    // diffuse
    vec3 norm = normalize(l_normal);

    vec3 lightDir = normalize(light_pos - l_fragpos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * light_color;

    vec3 result = (ambient + diffuse) * object_color;
    FragColor = vec4(result, 1.0);
}"""

lightless_fragment_shader = """
#version 330 core
out vec4 FragColor;

in vec3 l_normal; // Not used
in vec3 l_fragpos; // Not used

uniform vec3 light_pos; // Not used
uniform vec3 light_color; // Not used
uniform vec3 object_color;

void main()
{
    FragColor = vec4(object_color, 1.0);
}
"""


default_vertex_shader = """
#version 330 core
layout ( location = 0 ) in vec3 position;
layout ( location = 1 ) in vec3 normal;
layout ( location = 2 ) in vec2 texCoords;

out vec2 TexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 l_fragpos;
out vec3 l_normal;

void main()
{
    l_fragpos = vec3(model * vec4(position, 1.0));
    l_normal = mat3(transpose(inverse(model))) * normal;

    gl_Position = projection * view * model * vec4(position, 1.0f);
    gl_PointSize = 2.0;

    TexCoords = texCoords;
}
"""

background_vertex_shader = """
#version 330 core
out vec2 v_uv;

void main()
{
  // uint idx = gl_VertexID;
  gl_Position = vec4( gl_VertexID & 1, gl_VertexID >> 1, 0.0, 0.5 ) * 4.0 - 1.0;
  v_uv = vec2( gl_Position.xy * 0.5 + 0.5 );
}
"""

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
"""


class Shader(object):
    """Payton shader class. Creates shaders / programs

    For an improved performance, Shader aims to reduce the number of
    `glGetUniformlocation` function calls. For this purpose, `__init__` and
    `build` functions accept a list of variable names as `variables` argument.

    By this way, Shader goes through this list of variables after compiling the
    shader program and stores their locations in memory for future use.

    If you provide the list of variable names in build function call then
    it will overwrite existing list.
    """
    def __init__(self, **args):
        """Initialize Shader.

        Args:
          fragment: Fragment shader code
          vertex: Vertex shader code
          variables: List of in/out/uniform variable names.
        """
        global default_fragment_shader, default_vertex_shader
        self.fragment_shader_source = args.get('fragment',
                                               default_fragment_shader)
        self.vertex_shader_source = args.get('vertex',
                                             default_vertex_shader)
        self.variables = args.get('variables', [])
        self._stack = {} # Variable stack.

        self.program = None

    def build(self, variables=None):
        """Build GLSL Shader
        Compile shaders and compile glsl program
        Args:
          variables: List of in/out/uniform variable names.
        Return:
            self.program
        """
        vertex_shader = shaders.compileShader(self.vertex_shader_source,
                                                    GL_VERTEX_SHADER)
        fragment_shader = shaders.compileShader(self.fragment_shader_source,
                                                GL_FRAGMENT_SHADER)
        self.program = shaders.compileProgram(vertex_shader,
                                              fragment_shader)
        if variables:
            self.variables = variables

        for v in self.variables:
            location = glGetUniformLocation(self.program, v)
            self._stack[v] = location

        return self.program

    def use(self):
        """Use GLSL Program.
        This method should be called before setting any variables or rendering
        any entities
        """
        if not self.program:
            logging.error('Shader not compiled')
            return False
        glUseProgram(self.program)
        glEnable(GL_PROGRAM_POINT_SIZE)
        return True

    def end(self):
        """Set the active GLSL program to 0
        """
        glUseProgram(0)
        glDisable(GL_PROGRAM_POINT_SIZE)

    def set_matrix4x4_np(self, variable, value, transpose=False):
        """Set 4x4 Numpy matrix value

        Some elements like Observer and Light can pass their matrices directly as
        numpy array to reduce number of object conversions. This is the ideal way
        if possible.

        If variable not found in `self.variables` then system will try to locate
        the variable location and store it in `self.variables` for future
        reference.

        Args:
          variable: Variable name to set
          value: Matrix to set. (Numpy matrix)
          transpose: Transpose matrix.
        """
        transpose = GL_TRUE if transpose else GL_FALSE
        if variable not in self.variables:
            location = glGetUniformLocation(self.program, variable)
            if not location:
                logging.error('Variable not found in program [{}]'.format(
                    variable))
                return False
            self._stack[variable] = location

        glUniformMatrix4fv(self._stack[variable], 1, transpose,
                           np.asfortranarray(value, dtype=np.float32))
        return True

    def set_matrix4x4(self, variable, value, transpose=False):
        """Set 4x4 Matrix value

        This method will simply convert `value` into numpy array and call
        `payton.scene.shader.Shader.set_matrix4x4_np` method.

        Args:
          variable: Variable name to set
          value: Matrix to set. (Numpy matrix)
          transpose: Transpose matrix.
        """
        self.set_matrix4x4_np(variable, value, transpose)

    def set_vector3_np(self, variable, value):
        """Set Vector 3 as numpy array value

        If variable not found in `self.variables` then system will try to locate
        the variable location and store it in `self.variables` for future
        reference.

        Some elements like Light or pre-set materials can pass their vertices
        directly as numpy array to reduce number of object conversions.

        Args:
          variable: Variable name to set
          value: Vector 3 to set. (Numpy array with 3 elemenets)
        """
        if variable not in self.variables:
            location = glGetUniformLocation(self.program, variable)
            if not location:
                logging.error('Variable not found in program [{}]'.format(
                    variable))
                return False
            self._stack[variable] = location
        glUniform3fv(self._stack[variable], 1, value)

    def set_vector4_np(self, variable, value):
        """Set Vector 4 as numpy array value

        If variable not found in `self.variables` then system will try to locate
        the variable location and store it in `self.variables` for future
        reference.

        Some elements like Light or pre-set materials can pass their vertices
        directly as numpy array to reduce number of object conversions.

        Args:
          variable: Variable name to set
          value: Vector 4 to set. (Numpy array with 4 elemenets)
        """
        if variable not in self.variables:
            location = glGetUniformLocation(self.program, variable)
            if not location:
                logging.error('Variable not found in program [{}]'.format(
                    variable))
                return False
            self._stack[variable] = location
        glUniform4fv(self._stack[variable], 1, value)

    def set_vector3(self, variable, value):
        """Set Vector 3 as array value
        If variable not found in `self.variables` then system will try to locate
        the variable location and store it in `self.variables` for future
        reference.

        This method will simply convert `value` into numpy array and call
        `payton.scene.shader.Shader.set_vector3_np` method.

        Args:
          variable: Variable name to set
          value: Vector 3 to set. (Numpy array with 3 elemenets)
        """
        self.set_vector3_np(variable, value)

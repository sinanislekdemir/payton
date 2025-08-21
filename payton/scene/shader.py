import ctypes
import logging
from typing import Any, Dict, List, Optional

import numpy as np
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
    glUniform2fv,
    glUniform3fv,
    glUniform4fv,
    glUniformMatrix4fv,
    glUseProgram,
    shaders,
)

from payton.math.matrix import Matrix
from payton.math.vector import Vector3D

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
out vec3 particle_world_pos;

void main() {
    vec4 world_pos = model * gl_in[0].gl_Position;
    particle_world_pos = world_pos.xyz;
    vec4 center = view * world_pos;
    
    // Create proper billboard quad
    varTex = vec2(0, 0);
    gl_Position = projection * (center + vec4(-particle_size, -particle_size, 0, 0));
    EmitVertex();

    varTex = vec2(1, 0);
    gl_Position = projection * (center + vec4(particle_size, -particle_size, 0, 0));
    EmitVertex();

    varTex = vec2(0, 1);
    gl_Position = projection * (center + vec4(-particle_size, particle_size, 0, 0));
    EmitVertex();

    varTex = vec2(1, 1);
    gl_Position = projection * (center + vec4(particle_size, particle_size, 0, 0));
    EmitVertex();
    
    EndPrimitive();
}
"""


particle_fragment_shader = """
#version 330

uniform sampler2D tex_unit;
uniform vec3 object_color;
uniform float opacity;

in vec2 varTex;
in vec3 particle_world_pos;

layout(location = 0) out vec4 outFragColor;

void main()
{
    vec4 tex_color = texture(tex_unit, varTex);
    
    // Simple circular particle shape with smooth edges
    vec2 center = vec2(0.5, 0.5);
    float dist_from_center = length(varTex - center);
    float circle_alpha = 1.0 - smoothstep(0.3, 0.5, dist_from_center);
    
    vec3 final_color = tex_color.rgb * object_color;
    float final_alpha = tex_color.a * opacity * circle_alpha;
    
    outFragColor = vec4(final_color, final_alpha);
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
out vec3 l_fragpos;
out vec3 l_normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform int view_mode;
uniform float time;

void main()
{
    // Basic particle animation - you can enhance this based on your needs
    vec3 animated_pos = position;
    
    // Optional: Add some movement or rotation based on time
    // animated_pos.y += sin(time + position.x) * 0.1;
    
    gl_Position = vec4(animated_pos, 1.0f);
    
    l_fragpos = vec3(model * vec4(animated_pos, 1.0));
    l_normal = mat3(transpose(inverse(model))) * normal;
    
    tex_coords = texCoords;
    frag_color = colors;
}
"""  # type: str


default_fragment_shader = """
#version 330 core
out vec4 FragColor;

in vec2 tex_coords;
in vec3 frag_color;
in vec3 l_fragpos;
in vec3 l_normal;
in vec3 view_pos;
in vec4 frag_pos_light_space;

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

// Material properties for enhanced shading (with defaults to maintain compatibility)
uniform float metallic;
uniform float roughness;
uniform float ao;

uniform sampler2D tex_unit;
uniform samplerCube depthMap;

// Improved sampling pattern for shadows
vec3 gridSamplingDisk[20] = vec3[]
(
   vec3(1, 1,  1), vec3( 1, -1,  1), vec3(-1, -1,  1), vec3(-1, 1,  1),
   vec3(1, 1, -1), vec3( 1, -1, -1), vec3(-1, -1, -1), vec3(-1, 1, -1),
   vec3(1, 1,  0), vec3( 1, -1,  0), vec3(-1, -1,  0), vec3(-1, 1,  0),
   vec3(1, 0,  1), vec3(-1,  0,  1), vec3( 1,  0, -1), vec3(-1, 0, -1),
   vec3(0, 1,  1), vec3( 0, -1,  1), vec3( 0, -1, -1), vec3( 0, 1, -1)
);

// Enhanced shadow calculation with better bias
float ShadowCalculation(vec3 fragPos)
{
    if (LIGHT_COUNT == 0) return 0.0;
    
    vec3 fragToLight = fragPos - light_pos[0];
    float currentDepth = length(fragToLight);
    float shadow = 0.0;
    
    // Improved bias calculation
    vec3 normal = normalize(l_normal);
    vec3 lightDir = normalize(light_pos[0] - fragPos);
    float bias = max(0.05 * (1.0 - dot(normal, lightDir)), 0.005);

    float viewDistance = length(camera_pos - fragPos);
    float diskRadius = (1.0 + (viewDistance / far_plane)) / 25.0;

    int valid_samples = min(samples, 20);
    for(int i = 0; i < valid_samples; ++i)
    {
        float closestDepth = texture(depthMap, fragToLight +
                                     gridSamplingDisk[i] * diskRadius).r;
        closestDepth *= far_plane;
        if(currentDepth - bias > closestDepth)
             shadow += 1.0;
    }

    return shadow / float(valid_samples);
}

void main()
{
    vec3 color;
    float alpha;
    
    // Material color determination
    if (material_mode == 0 || material_mode == 2) {
        color = object_color;
        alpha = opacity;
    }
    else if (material_mode == 1 || material_mode == 3) {
        vec4 tex_sample = texture(tex_unit, tex_coords);
        color = tex_sample.rgb;
        alpha = tex_sample.a * opacity;
    }
    else if (material_mode == 4) {
        color = frag_color;
        alpha = opacity;
    }
    else {
        color = object_color;
        alpha = opacity;
    }

    if (lit == 0 || LIGHT_COUNT == 0) {
        // Unlit or no lights - just show the color
        FragColor = vec4(color, alpha);
        return;
    }
    
    // Lighting calculations
    vec3 norm = normalize(l_normal);
    
    // Check for invalid normals
    if (length(l_normal) < 0.1) {
        FragColor = vec4(color * 0.5, alpha); // Fallback for invalid normals
        return;
    }
    
    vec3 viewDir = normalize(camera_pos - l_fragpos);
    vec3 result = vec3(0.0);
    
    for (int i = 0; i < min(LIGHT_COUNT, 100); i++) {
        vec3 lightDir = normalize(light_pos[i] - l_fragpos);
        float distance = length(light_pos[i] - l_fragpos);
        
        // Enhanced attenuation
        float attenuation = 1.0 / (1.0 + 0.045 * distance + 0.0075 * distance * distance);
        
        // Ambient
        vec3 ambient = 0.15 * color * ao;
        
        // Diffuse
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 diffuse = diff * light_color[i] * color;
        
        // Specular (Blinn-Phong)
        vec3 halfwayDir = normalize(lightDir + viewDir);
        float spec_strength = mix(0.5, 1.0, metallic); // Use metallic to control specular strength
        float shininess = mix(32.0, 128.0, 1.0 - roughness); // Use roughness to control shininess
        float spec = pow(max(dot(norm, halfwayDir), 0.0), shininess);
        vec3 specular = spec_strength * spec * light_color[i];
        
        // Shadow calculation (only for first light to keep performance)
        float shadow = (i == 0 && shadow_enabled) ? ShadowCalculation(l_fragpos) : 0.0;
        
        // Combine results
        vec3 lighting = ambient + (1.0 - shadow) * (diffuse + specular);
        result += lighting * attenuation;
    }
    
    // Simple tone mapping
    result = result / (result + vec3(1.0));
    
    // Gamma correction
    result = pow(result, vec3(1.0/2.2));
    
    FragColor = vec4(result, alpha);
}"""  # type: str


default_vertex_shader = """
#version 330 core
layout ( location = 0 ) in vec3 position;
layout ( location = 1 ) in vec3 normal;
layout ( location = 2 ) in vec2 texCoords;
layout ( location = 3 ) in vec3 colors; // optional

out vec2 tex_coords;
out vec3 frag_color;
out vec3 l_fragpos;
out vec3 l_normal;
out vec3 view_pos;
out vec4 frag_pos_light_space;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 light_space_matrix;
uniform int view_mode;

void main()
{
    vec4 world_pos = model * vec4(position, 1.0);
    l_fragpos = world_pos.xyz;
    
    // Transform normal to world space with proper handling of non-uniform scaling
    mat3 normal_matrix = mat3(transpose(inverse(model)));
    l_normal = normalize(normal_matrix * normal);
    
    // Calculate view position for PBR calculations
    view_pos = (view * world_pos).xyz;
    
    // Calculate light space position for shadow mapping
    frag_pos_light_space = light_space_matrix * world_pos;

    if (view_mode == 0) {
        gl_Position = projection * view * world_pos;
    }
    if (view_mode == 1) {
        gl_Position = projection * (vec4(position, 1.0f) + vec4(model[3].xyz, 0.0f));
    }

    // Enhanced point size with distance attenuation
    float distance_to_camera = length(view_pos);
    gl_PointSize = max(1.0, 10.0 / (1.0 + 0.1 * distance_to_camera));

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
uniform float time;
uniform vec2 resolution;
in vec2 v_uv;

out vec4 frag_color;

void main()
{
    // Enhanced gradient with subtle animation
    float gradient_factor = smoothstep(0.0, 1.0, v_uv.y);
    
    // Add subtle color variation over time (optional)
    float time_variation = sin(time * 0.1) * 0.02 + 1.0;
    
    vec4 mixed_color = mix(bot_color, top_color, gradient_factor) * time_variation;
    
    // Add subtle noise for more natural look
    float noise = fract(sin(dot(v_uv, vec2(12.9898, 78.233))) * 43758.5453) * 0.02 - 0.01;
    mixed_color.rgb += noise;
    
    // Ensure colors stay in valid range
    mixed_color = clamp(mixed_color, 0.0, 1.0);
    
    frag_color = mixed_color;
}
"""  # type: str


class Shader:
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
        """Initialize the shader

        Keyword arguments:
        fragment -- Fragment shader code
        vertex -- Vertex shader code
        geometry -- Geometry shader code
        variables -- Shader variables (Optional but defining them beforehand can increase the warm-up performance)
        """
        self.fragment_shader_source = fragment
        self.vertex_shader_source = vertex
        self.geometry_shader_source = geometry

        # Enhanced default variables for improved visual quality
        default_vars = [
            "model",
            "view",
            "projection",
            "light_space_matrix",
            "object_color",
            "opacity",
            "metallic",
            "roughness",
            "ao",
            "camera_pos",
            "light_pos",
            "light_color",
            "LIGHT_COUNT",
            "material_mode",
            "lit",
            "shadow_enabled",
            "samples",
            "far_plane",
            "time",
            "resolution",
            "top_color",
            "bot_color",
            "particle_size",
            "tex_unit",
            "depthMap",
        ]

        self.variables: List[str] = (
            default_vars if variables is None else variables + default_vars
        )
        self._stack: Dict[str, int] = {}  # Variable stack.
        self._mode: int = self.NO_LIGHT_COLOR  # Lightless color material
        self._depth_shader = False

        self.program: int = -1

    def build(self) -> int:
        """Build the shader

        Compile the shader codes and compile the program
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
                vertex_shader, fragment_shader, geometry_shader, validate=False
            )
        else:
            self.program = shaders.compileProgram(
                vertex_shader, fragment_shader, validate=False
            )

        return self.program

    def use(self) -> bool:
        """Use the shader (activate it in the pipeline)"""
        if self.program == -1:
            logging.error("Shader not compiled")
            return False
        glUseProgram(self.program)
        glEnable(GL_PROGRAM_POINT_SIZE)
        return True

    def end(self) -> None:
        """End using the shader in the pipeline"""
        glUseProgram(0)
        glDisable(GL_PROGRAM_POINT_SIZE)

    def set_matrix4x4_np(
        self, variable: str, value: np.ndarray, transpose: bool = False
    ) -> bool:
        """Set 4x4 Matrix data in the shader as Numpy Array

        Keyword arguments:
        variable -- Variable name
        value -- Numpy array as the value
        transpose -- GL_TRANSPOSE variable if needed (glUniformMatrix4fv)
        """
        g_transpose = GL_TRUE if transpose else GL_FALSE
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniformMatrix4fv(
            location,
            1,
            g_transpose,
            value,
        )
        return True

    def get_location(self, variable: str) -> int:
        """Get the memory location of the given variable in the shader

        Keyword arguments:
        variable -- Variable name
        """
        if variable in self._stack:
            return self._stack[variable]
        location = glGetUniformLocation(self.program, variable)

        self._stack[variable] = location
        return location

    def set_matrix4x4(
        self, variable: str, value: Matrix, transpose: bool = False
    ) -> None:
        """Set 4x4 Matrix data in the shader

        Keyword arguments:
        variable -- Variable name
        value -- Matrix value
        transpose -- GL_TRANSPOSE variable if needed (glUniformMatrix4fv)
        """
        self.set_matrix4x4_np(variable, np.array(value, np.float32), transpose)

    def set_vector3_array_np(
        self, variable: str, value: np.ndarray, count: int
    ) -> bool:
        """Set the given vectors as value in the shader

        Keyword arguments:
        variable -- Variable name
        value -- Value as numpy array
        count -- Number of elements
        """
        value = value.flatten()
        location = self.get_location(variable)
        if location == -1:
            return False
        # Send count * 3 values (3 components per vector)
        glUniform3fv(location, count, value[: count * 3])
        return True

    def set_vector3_np(self, variable: str, value: np.ndarray) -> bool:
        """Set the given vector value in the shader

        Keyword arguments:
        variable -- Variable name
        value -- Numpy array value
        """
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform3fv(location, 1, value[:3])
        return True

    def set_vector4_np(self, variable: str, value: np.ndarray) -> bool:
        """Set the given vector as 4 elements in the shader

        Keyword arguments:
        variable -- Variable name
        value -- Numpy array value
        """
        location = self.get_location(variable)
        if location == -1:
            return False
        glUniform4fv(location, 1, value)
        return True

    def set_vector3(self, variable: str, value: Vector3D) -> None:
        """Set the given vector in the shader

        Keyword arguments:
        variable -- Variable name
        value -- Value as Vector3D
        """
        self.set_vector3_np(variable, np.array(list(value), dtype=np.float32))

    def set_int(self, variable: str, value: int) -> bool:
        """Set integer in the shader

        Keyword arguments:
        variable -- Variable name
        value -- Value as integer
        """
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniform1i(location, ctypes.c_int(value))
        return True

    def set_float(self, variable: str, value: float) -> bool:
        """Set float in the shader

        Keyword arguments:
        variable -- Variable name
        value -- Value as float
        """
        location = self.get_location(variable)
        if location == -1:
            return False

        glUniform1f(location, ctypes.c_float(value))
        return True

    def set_material_properties(
        self, metallic: float = 0.0, roughness: float = 0.5, ao: float = 1.0
    ) -> None:
        """Set PBR material properties

        Keyword arguments:
        metallic -- Metallic factor (0.0 = dielectric, 1.0 = metallic)
        roughness -- Surface roughness (0.0 = mirror, 1.0 = completely rough)
        ao -- Ambient occlusion factor (0.0 = fully occluded, 1.0 = no occlusion)
        """
        # Ensure the uniforms exist and set safe defaults
        if self.get_location("metallic") == -1:
            # If uniform doesn't exist, shader might not use PBR
            return

        self.set_float("metallic", metallic)
        self.set_float("roughness", max(0.04, roughness))  # Prevent roughness of 0
        self.set_float("ao", ao)

    def set_default_material_properties(self) -> None:
        """Set default material properties for backward compatibility"""
        self.set_material_properties(0.0, 0.5, 1.0)

    def set_time(self, time: float) -> bool:
        """Set time uniform for animated shaders

        Keyword arguments:
        time -- Time value in seconds
        """
        return self.set_float("time", time)

    def set_resolution(self, width: float, height: float) -> bool:
        """Set screen resolution for shaders that need it

        Keyword arguments:
        width -- Screen width
        height -- Screen height
        """
        location = self.get_location("resolution")
        if location == -1:
            return False

        import numpy as np

        resolution = np.array([width, height], dtype=np.float32)
        glUniform2fv(location, 1, resolution)
        return True

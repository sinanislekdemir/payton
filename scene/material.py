"""
What is a material?

Materials define how your scene entities look like. Their colors, shininess,
or displaying them as solid objects or wireframes, all are defined inside
object materials. This also effects if your object will respond to light
sources or not.

Materials are currently only OpenGL 1.1 definitions.
"""

from OpenGL.GL import (glEnable, glDisable, glColorMaterial, glMaterialfv,
                       glBlendFunc, glColor4fv, GL_LIGHTING, GL_COLOR_MATERIAL,
                       GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GL_FRONT,
                       GL_SHININESS, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,
                       GL_BLEND, GL_SPECULAR)

SOLID = 0
WIREFRAME = 1

class Material(object):
    """
    Material is a basic
    """
    def __init__(self, **args):
        """
        Initialize Material

        Each material has a color, display mode, specular, shininess
        and lighting.

        Color and Specular are a list or tuple of 4 floats.
        [1.0, 1.0, 1.0, 1.0] which are Red, Green, Blue and Alpha.
        Each element of color is a float between 0 and 1.
        (0 - 255 respectively)
        
        Display Mode has 2 modes. Solid and Wireframe. Wireframe is
        often rendered in a faster way. Also good for debugging your
        object.
        Display mode changes requires object to be rebuilt.

        Shininess is the amount of `shininess` between 0 and 1.

        When Lighting is False, the object is not effected by the
        light and will not be shaded.

        Default variables:

            {'color': [1.0, 1.0, 1.0, 1.0],
             'display': SOLID,
             'specular': [1.0, 1.0, 1.0, 1.0],
             'shininess': 1.0,
             'lighting': True}
        """

        self.color = args.get('color', [1.0, 1.0, 1.0, 1.0])
        self.display = args.get('display', SOLID)
        self.specular = args.get('specular', [1.0, 1.0, 1.0, 1.0])
        self.shininess = args.get('shininess', 1.0)
        self.lighting = args.get('lighting', True)

    def begin_render(self):
        """
        Begin render should be called before drawing the object geometry.
        """
        if self.lighting:
            glEnable(GL_LIGHTING)
            glEnable(GL_COLOR_MATERIAL)
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
            glMaterialfv(GL_FRONT, GL_SPECULAR, self.specular)
            glMaterialfv(GL_FRONT, GL_SHININESS, [self.shininess])
        else:
            glDisable(GL_LIGHTING)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_BLEND)
        glColor4fv(self.color)
    
    def end_render(self):
        """
        End render should be called after drawing the geometry. Otherwise,
        next object drawing calls in the pipeline can get effected by
        previous objects material.
        """
        if self.lighting:
            glDisable(GL_LIGHTING)
            glDisable(GL_COLOR_MATERIAL)
        glDisable(GL_BLEND)

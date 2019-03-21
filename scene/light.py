"""Payton Lights.

Support for scene lights.
"""

import logging
from OpenGL.GL import (glLightfv, GL_LIGHTING, GL_POSITION, GL_MAX_LIGHTS,
                       GL_AMBIENT, GL_SPECULAR, glEnable,
                       GL_LIGHT0, GL_LIGHT1, GL_LIGHT2, GL_LIGHT3, GL_LIGHT4,
                       GL_LIGHT5, GL_LIGHT6, GL_LIGHT7)

LIGHT_INDEX = 0

class Light(object):
    """Main Light object
    """
    LIGHTS = [GL_LIGHT0, GL_LIGHT1, GL_LIGHT2, GL_LIGHT3, GL_LIGHT4, GL_LIGHT5,
          GL_LIGHT6, GL_LIGHT7]
    def __init__(self, **args):
        """Initialize Payton Light
        Note that, for every newly created light, a GL_LIGHTx will be
        consumed. OpenGL 2.1 supports only 8 light sources. So, if you try to
        create more than 8 lights, system will log an error and will not add
        new light.
        """
        global LIGHT_INDEX
        self.position = args.get('position', [0.0, 0.0, 0.0, 1.0])
        self.ambient = args.get('ambient', [0.0, 0.0, 0.0, 1.0])
        self.specular = args.get('specular', [1.0, 1.0, 1.0, 1.0])

        self.active = True
        self.index = LIGHT_INDEX
        LIGHT_INDEX += 1
        if LIGHT_INDEX >= 8:
            self.active = False
            self.index = -1
            logging.error('Can not create additional lights in the scene')

    def render(self):
        """Render light
        """
        return False
        if not self.active:
            return False

        glEnable(self.LIGHTS[self.index])
        glLightfv(self.LIGHTS[self.index], GL_POSITION, self.position)
        glLightfv(self.LIGHTS[self.index], GL_AMBIENT, self.ambient)
        glLightfv(self.LIGHTS[self.index], GL_SPECULAR, self.specular)

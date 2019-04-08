"""Payton Lights.

Support for scene lights.
"""

import logging

LIGHT_INDEX = 0

class Light(object):
    """Main Light object
    """
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
        pass

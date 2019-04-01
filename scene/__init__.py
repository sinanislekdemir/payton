"""Payton Scene.

Python scene can be thought as a real movie scene, where you have your
actors, camera and lights. So, in a simulation, there can be
only one Scene running the show. *(Multiple scenes can be achieved by
multi-threading but due to Python limitations like Global Interpreter
Lock, there will not be much benefit to that)*

Scene has many items elements:

- Objects (`self.objects = {}`)
- Observers (`self.observers = []`)
- Clocks (`self.clocks = {}`)
- Grid (`self.grid = Grid()`)
- window (SDL)

**Objects** hold the main scene actors.

**Grid** is the default grid in 3D Scene. It is easier to follow the
motion of objects when there is a grid.

**Clocks** give you actual clocks-timers you can program. If you want to
move your objects one step at a time, you need clocks. A motion can
not happen without `time` and as Albert Einstein said, *"time is what the
clock measures"*
"""
import math
import ctypes
import sdl2
import logging
import numpy as np
import pyrr

from OpenGL.GL import *

from payton.scene.grid import Grid
from payton.scene.observer import Observer, BUTTON_LEFT, BUTTON_RIGHT
from payton.scene.clock import Clock
from payton.scene.material import Material, SOLID
from payton.scene.light import Light
from payton.scene.shader import Shader, lightless_fragment_shader


class Object(object):
    """Main Payton Object.

    This is an abstract class to define common properties between
    Mesh / Particle / Virtual objects.

    Objects are not actually built as a 3D object unless they are
    being rendered. Render function calls `build` function
    which then creates the opengl display list of the object.
    Display list is a static data so, once the object is built,
    changing vertices or indices won't help with the geometry
    of the object.

    To change the geometry, you need to call `build` function once
    more. (Also, display mode changes need a rebuild)
    """
    def __init__(self):
        """
        Initialize the basic object properties here.
        This is important as to keep track of all self object
        properties and avoid any assumptions on whether an object
        property is set or not.

        So if anyhow, you are adding an object property, please
        do not forget to define its default here.

        Each object can have several children.
        And each child can have its own children as well.
        So we have an object tree, which is suitable for complex
        systems like a solar system.
        A Star has planets and each planet can have moons or satellites.
        Each child in the list has their own local object coordinate system.
        To get the absolute coordinates of a local coordinate in the universe
        you can use to_absolute function.
        """
        self.children = {}
        self.material = Material()
        self.static = True
        self.matrix = [1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 0.0, 1.0]
        self._vertices = []
        self._normals = []
        self._texcoords = []
        self._indices = []
        self._vertex_count = 0
        self._model_matrix = None

        # Default object uses default shaders.
        variables = ['model', 'view', 'projection',
                     'light_pos', 'light_color', 'object_color']
        self._shader = Shader(variables=variables)

        # Vertex Array Object pointer
        self._vao = None

    def destroy(self):
        """
        Destroy objects self
        """
        if self._vao:
            glDeleteVertexArrays(1, [self._vao])
        return True

    def render(self, proj, view, light_pos, light_color):
        """
        Virtual function for rendering the object.

        Args:
          proj: Camera projection matrix.
          view: Camera location/view matrix.
          light_pos: Light position
          light_color: Light color

        """

        if not self._vao:
            self.build()

        # Setup shader arguments
        self._shader.use()
        self._model_matrix = np.array(self.matrix, dtype=np.float32)

        self._shader.set_matrix4x4_np('model', self._model_matrix)
        self._shader.set_matrix4x4_np('view', view)
        self._shader.set_matrix4x4_np('projection', proj)
        self._shader.set_vector3_np('light_pos', light_pos)
        self._shader.set_vector3_np('light_color', light_color)
        self._shader.set_vector3('object_color', np.array(self.material.color,
                                                          dtype=np.float32))

        if glIsVertexArray(self._vao):
            glBindVertexArray(self._vao)
            pmode = GL_LINE
            if self.material.display == SOLID:
                pmode = GL_FILL
            glPolygonMode(GL_FRONT_AND_BACK, pmode)
            glDrawElements(GL_TRIANGLES, self._vertex_count,
                           GL_UNSIGNED_INT, ctypes.c_void_p(0))
            if pmode != GL_FILL:
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glBindVertexArray(0)

        # render
        self._shader.end()

        for child in self.children:
            self.children[child].render()


    def set_position(self, pos):
        """
        Shortcut function for explicitly modifying matrix indices.

        Basically just sets 12, 13, 14 = x, y, z
        """
        self.matrix[12] = pos[0]
        self.matrix[13] = pos[1]
        self.matrix[14] = pos[2]

    def get_position(self):
        return self.matrix[12:15]


    def to_absolute(self, coordinates):
        """
        Return local coordinates (tuple, list) into absolute coordinates in
        space.
        """
        pass

    def to_local(self, coordinates):
        """
        Return absolute coordinates (tuple, list) into local coordinates
        """
        pass

    def build(self):
        """
        Build OpenGL Vertex Array for the object
        This function gets automatically called if `self._vao` does not
        exists in the first render cycle. Once the display list is built,
        geometry changes or material display mode changes will not be
        automatically effected. So, in every geometry or display mode
        change, a build call is necessary.

        if `self.static` is `True`, then system assumes that another build
        call is not expected, thus frees `_normals', `_textcoords`,
        `_vertices` and `_indices` lists to free memory.
        So in this case, calling `build` function twice will result with
        an invisible object (will not be drawn)

        """
        self._vao = glGenVertexArrays(1)
        vbos = glGenBuffers(4)
        glBindVertexArray(self._vao)

        self._shader.build()
        vertices = np.array(self._vertices, dtype=np.float32)
        normals = np.array(self._normals, dtype=np.float32)
        texcoords = np.array(self._texcoords, dtype=np.float32)
        indices = np.array(self._indices, dtype=np.int32)

        glBindBuffer(GL_ARRAY_BUFFER, vbos[0])
        glEnableVertexAttribArray(0) # shader layout location
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes,
                     vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ARRAY_BUFFER, vbos[1])
        glEnableVertexAttribArray(1) # shader layout location
        glVertexAttribPointer(1, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        glBufferData(GL_ARRAY_BUFFER, normals.nbytes, normals, GL_STATIC_DRAW)

        glBindBuffer(GL_ARRAY_BUFFER, vbos[2])
        glEnableVertexAttribArray(2) # shader layout location
        glVertexAttribPointer(2, 2, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        glBufferData(GL_ARRAY_BUFFER, texcoords.nbytes, texcoords,
                     GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos[3])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)
        self._vertex_count = len(indices)

        glBindVertexArray(0)
        # glDisableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDeleteBuffers(4, vbos)

        if self.static:
            # we can clear this data to free some more memory
            self._normals = []
            self._texcoords = []
            self._vertices = []
            self._indices = []

        return True


class Scene(object):
    """
    Main Payton scene.
    """
    def __init__(self):
        """
        Initialize the Payton Scene
        There is no parameters here. Every class property must be explicitly
        defined.

        Simplest form of usage:

            a = Scene()
            a.run()

        """
        # All objects list
        self.objects = {}
        # List of observers (cameras) in the scene. There can be only
        # one active observer at a time
        self.observers = []
        self.observers.append(Observer(active=True))
        # Instead of looping through observers to find the active
        # observer, we are keeping the known index to avoid redundant
        # loops.
        self._active_observer = 0

        self.lights = []

        # List of all clocks. Clocks are time based function holders which
        # animate objects in the scene or do other stuff.
        self.clocks = {}
        self.grid = Grid()

        # SDL Related Stuff
        self.window = None
        self.window_width = 800
        self.window_height = 600
        self._context = None
        self._mouse = [0, 0]
        self._shift_down = False
        self._ctrl_down = False
        self._rotate = False

        # Main running state
        self.running = False

    def _render(self):
        """
        Render the whole scene here. Note that, this is a private function and
        should not be overriden unless you really know what you are dealing
        with.
        """
        # Disable Depth Test to draw background at the very back of the scene
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        # TODO: Grid should be optional.
        # Observer location or light location does not change during a frame
        # render. To avoid redundant calls to re-calculate and convert locations
        # for each object in the scene, we diretly pass their Numpy array
        # values to render pipeline.
        proj, view = self.observers[self._active_observer].render()
        light_pos = np.array([3.0, 6.0, 9.0], dtype=np.float32)
        light_color = np.array([1.0, 1.0, 1.0], dtype=np.float32)

        self.grid.render(proj, view, light_pos, light_color)

        for object in self.objects:
            self.objects[object].render(proj, view, light_pos, light_color)

        return 0

    def add_object(self, name, obj):
        """
        Add object to the scene. Object must be an instance of
        `payton.scene.Object`

        Example usage:

            from payton.scene import Scene
            from payton.scene.geometry import Cube

            my_scene = Scene()
            cube = Cube()
            my_scene.add_object(cube)
            my_scene.run()
        """
        if not isinstance(obj, Object):
            logging.error("Given object is not an instance of `scene.Object`")
            return False

        if name in self.objects:
            logging.error('Given object name [{}] already exists'.format(name))
            return False

        self.objects[name] = obj

    def add_observer(self, obj):
        """
        Add observer to the scene. Observer must be an instance of
        `payton.scene.observer.Observer`
        Generally this is not needed as scene has already a default observer.
        """
        if not isinstance(obj, Observer):
            logging.error("Given observer is not an instance of `scene.Observer`")
            return False

        self.observers.append(obj)

    def create_observer(self):
        """
        Create a new observer in the scene.
        """
        self.observers.append(Observer())

    def create_clock(self, name, period, callback):
        """
        Creates a clock in the scene. This is the preffered way to create a clock
        in the scene as it binds the clock to the scene by itself.

        Note that, name should be unique within the scene so any duplicating
        clock names will be logged as an error and will not be added.

        Example usage:

            from payton.scene import Scene
            from payton.scene.clock import Clock


            def time_counter_callback(name, scene, period, total):
                print("Total time passed since beginning {}secs".format(total))

            my_scene = Scene()
            my_scene.create_clock('time_counter', 1.0, time_counter_callback)
            my_scene.run()

        Result output to console:

            Total time passed since beginning 0secs
            Total time passed since beginning 1.0secs
            Total time passed since beginning 2.0secs
            Total time passed since beginning 3.0secs
            Total time passed since beginning 4.0secs
            Total time passed since beginning 5.0secs
            Total time passed since beginning 6.0secs
            Total time passed since beginning 7.0secs
            Total time passed since beginning 8.0secs
            Total time passed since beginning 9.0secs
            Total time passed since beginning 10.0secs
            ...
        """
        if name in self.clocks:
            logging.error("A clock named {} already exists".format(name))
            return False

        c = Clock(name, period, self, callback)
        self.clocks[name] = c

    def run(self):
        """
        Run scene.

        This is a complex function and needs some refactoring.
        Here is the logic and flow behind this function:

        1. Initialize SDL
        2. Create SDL window with given parameters enabling OpenGL Pipeline
        3. Create and set active context as OpenGL Context
        4. Set running flag.
        5. Fix aspect ratios of observers *aka cameras* in the scene.
        6. Start scene loop.
            1. Poll for SDL events
            2. React to SDL events *(quit, key press, key up, mouse motion
               etc)*
            3. Render scene, turn to step 6.1 if not quit
        7. Destroy objects and clear memory.
        8. Destroy window and such.
        """
        if sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO) != 0:
            return -1
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_MAJOR_VERSION, 3)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_MINOR_VERSION, 3)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_PROFILE_MASK,
                                 sdl2.SDL_GL_CONTEXT_PROFILE_CORE)

        self.window = sdl2.SDL_CreateWindow(b"Payton Scene",
                                            sdl2.SDL_WINDOWPOS_UNDEFINED,
                                            sdl2.SDL_WINDOWPOS_UNDEFINED,
                                            self.window_width,
                                            self.window_height,
                                            sdl2.SDL_WINDOW_OPENGL)

        if not self.window:
            return -1

        self._context = sdl2.SDL_GL_CreateContext(self.window)
        self.event = sdl2.SDL_Event()
        self.running = True

        # Fix aspect ratios of observers
        for observer in self.observers:
            observer.aspect_ratio = (self.window_width /
                                     self.window_height * 1.0)

        for clock in self.clocks:
            self.clocks[clock].start()

        while self.running:
            while sdl2.SDL_PollEvent(ctypes.byref(self.event)) != 0:
                if self.event.type == sdl2.SDL_QUIT:
                    logging.debug('Quit SDL Scene')
                    self.running = False
                    for clock in self.clocks:
                        c = self.clocks[clock]
                        logging.debug('Kill clock [{}]'.format(clock))
                        c.kill()
                        c.join()

                if self.event.type == sdl2.SDL_KEYDOWN:
                    key = self.event.key.keysym.sym
                    if key == sdl2.SDLK_LSHIFT:
                        self._shift_down = True
                    if key == sdl2.SDLK_LCTRL:
                        self._ctrl_down = True

                if self.event.type == sdl2.SDL_KEYUP:
                    key = self.event.key.keysym.sym
                    if key == sdl2.SDLK_LSHIFT:
                        self._shift_down = False
                    if key == sdl2.SDLK_LCTRL:
                        self._ctrl_down = False

                    if key == sdl2.SDLK_c:
                        self.observers[0].perspective = not self.observers[0].perspective
                        logging.debug('Observer[0] Perspective = {}'.format(
                            'True' if self.observers[0].perspective else 'False'))

                    if key == sdl2.SDLK_g:
                        self.grid.visible = not self.grid.visible

                    if key == sdl2.SDLK_p:
                        for clock in self.clocks:
                            c = self.clocks[clock]
                            logging.debug('Pause clock [{}]'.format(clock))
                            c.pause()
                    if key == sdl2.SDLK_ESCAPE:
                        self.running = False
                    if key in [sdl2.SDLK_F2, sdl2.SDLK_F3]:
                        active = 0
                        for i in range(len(self.observers)):
                            if self.observers[i].active:
                                active = i
                        if key == sdl2.SDLK_F2:
                            active -= 1
                        if key == sdl2.SDLK_F3:
                            active += 1
                        if active < 0:
                            active = len(self.observers) - 1
                        if active > len(self.observers) - 1:
                            active = 0
                        self._active_observer = active
                        for i in range(len(self.observers)):
                            self.observers[i].active = (i == active)

                if self.event.type == sdl2.SDL_MOUSEMOTION:
                    button = -1
                    if self.event.motion.state == sdl2.SDL_BUTTON_LMASK:
                        button = BUTTON_LEFT
                    if self.event.motion.state == sdl2.SDL_BUTTON_RMASK:
                        button = BUTTON_RIGHT
                    for o in self.observers:
                        o.mouse(button, self._shift_down, self._ctrl_down,
                                self.event.motion.x,
                                self.event.motion.y,
                                self.event.motion.xrel,
                                self.event.motion.yrel)

            self._render()
            sdl2.SDL_GL_SwapWindow(self.window)
            sdl2.SDL_Delay(10)

        for object in self.objects:
            self.objects[object].destroy()

        sdl2.SDL_GL_DeleteContext(self._context)
        sdl2.SDL_DestroyWindow(self.window)
        self.window = None
        sdl2.SDL_Quit()
        return 0

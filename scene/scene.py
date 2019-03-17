"""
Payton scene module.

Please see the `payton.scene` module documentation for a wider
information on Scene. This documentation is for the class methods and
basics.
"""
import ctypes
import sdl2
import logging

from OpenGL.GL import (glClear, glEnable, glDisable, glMatrixMode, glPushMatrix,
                       glLoadIdentity, glBegin, glEnd, glColor3f, glVertex2f,
                       glLightfv, GL_COLOR_BUFFER_BIT, GL_DEPTH_TEST,
                       GL_PROJECTION, GL_MODELVIEW, GL_LIGHTING, GL_QUADS,
                       GL_LIGHT0, GL_POSITION, GL_DEPTH_BUFFER_BIT)

from payton.scene.grid import Grid
from payton.scene.observer import Observer, BUTTON_LEFT, BUTTON_RIGHT
from payton.scene.clock import Clock
from payton.scene.object import Object


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
        self.objects = []
        # List of observers. Currently there will be only one default observer
        self.observers = []
        self.observers.append(Observer())

        # List of all clocks. Clocks are time based function holders which
        # animate objects in the scene or do other stuff.
        self.clocks = {}
        self.grid = Grid()
        self.grid.build()

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
        glDisable(GL_DEPTH_TEST)

        # Draw gradient background
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Gradient background
        # TODO: Remove hardcoded color codes from here. Give
        #       developers a chance to change the background.
        glDisable(GL_LIGHTING)
        glBegin(GL_QUADS)
        glColor3f(0.16015625, 0.1796875, 0.19140625)
        glVertex2f(-1.0, -1.0)
        glVertex2f(1.0, -1.0)
        glColor3f(0.03515625, 0.15625, 0.26171875)
        glVertex2f(1.0, 1.0)
        glVertex2f(-1.0, 1.0)
        glEnd()

        for observer in self.observers:
            observer.render()

        # TODO: Grid should be optional.
        self.grid.render()

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)

        glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 10.0, 1.0])

        for object in self.objects:
            object.render()

        glDisable(GL_LIGHT0)
        glDisable(GL_LIGHTING)

        return 0

    def add_object(self, obj):
        """
        Add object to the scene. Object must be an instance of
        `payton.scene.object.Object`

        Example usage:

            from payton.scene.scene import Scene
            from payton.scene.geometry import Cube

            my_scene = Scene()
            cube = Cube()
            my_scene.add_object(cube)
            my_scene.run()
        """
        if not isinstance(obj, Object):
            logging.error("Given object is not an instance of `scene.Object`")
            return False

        self.objects.append(obj)

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

            from payton.scene.scene import Scene
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
                    if self.event.key.keysym.sym == sdl2.SDLK_LSHIFT:
                        self._shift_down = True
                    if self.event.key.keysym.sym == sdl2.SDLK_LCTRL:
                        self._ctrl_down = True

                if self.event.type == sdl2.SDL_KEYUP:
                    if self.event.key.keysym.sym == sdl2.SDLK_LSHIFT:
                        self._shift_down = False
                    if self.event.key.keysym.sym == sdl2.SDLK_LCTRL:
                        self._ctrl_down = False

                    if self.event.key.keysym.sym == sdl2.SDLK_c:
                        self.observers[0].perspective = not self.observers[0].perspective
                        logging.debug('Observer[0] Perspective = {}'.format(
                            'True' if self.observers[0].perspective else 'False'))

                    if self.event.key.keysym.sym == sdl2.SDLK_g:
                        self.grid.visible = not self.grid.visible

                    if self.event.key.keysym.sym == sdl2.SDLK_p:
                        for clock in self.clocks:
                            c = self.clocks[clock]
                            logging.debug('Pause clock [{}]'.format(clock))
                            c.pause()

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
            object.destroy()

        sdl2.SDL_GL_DeleteContext(self._context)
        sdl2.SDL_DestroyWindow(self.window)
        self.window = None
        sdl2.SDL_Quit()
        return 0

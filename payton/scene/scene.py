"""#Payton Scene Module

##Main Scene Module:

* SDL2 Window
  * Scene (`payton.scene`)
    * Geometry (`payton.scene.geometry`)
    * Grid (`payton.scene.grid`)
    * Background (`payton.scene.scene.Background`)
    * Clock (`payton.scene.clock`)
    * Light (`payton.scene.light.Light`)
    * Collision Test (`payton.scene.collision.CollisionTest`)
    * Observer (`payton.scene.observer.Observer`)
    * Wavefront (`payton.scene.wavefront.Wavefront`)

"""
import ctypes
import sdl2
import logging
import time
import numpy as np  # type: ignore
from typing import Dict, Any, List, Type, Callable, TypeVar

from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_BUFFER_BIT,
    glEnable,
    GL_DEPTH_TEST,
    GL_LESS,
    glGenVertexArrays,
    glBindVertexArray,
    glDisable,
    glDrawArrays,
    glClear,
    glDepthFunc,
    GL_TRIANGLES,
)

from payton.scene.controller import Controller
from payton.scene.grid import Grid
from payton.scene.geometry import Object
from payton.scene.light import Light
from payton.scene.observer import Observer
from payton.scene.clock import Clock
from payton.scene.collision import CollisionTest
from payton.scene.gui import Hud, Shape2D
from payton.scene.receiver import Receiver

from payton.scene.shader import (
    Shader,
    background_fragment_shader,
    background_vertex_shader,
)

S = TypeVar("S", bound="Scene")


class Scene(Receiver):
    """
    Main Payton scene.
    """

    def __init__(self, **args: Any) -> None:
        """
        Initialize the Payton Scene
        There is no parameters here. Every class property must be explicitly
        defined.

        Simplest form of usage:

            from payton.scene import Scene
            a = Scene()
            a.run()

        on_select sample:

            from payton.scene import Scene
            from payton.scene.geometry import Cube

            def select(objects):
                print(objects)
                for obj in objects:
                    obj.material.color = [1.0, 0.0, 0.0]

            scene = Scene(on_select=select)
            cube1 = Cube()

            scene.add_object('cube1', cube1)

            scene.run()

        Args:
          width: Window width
          height: Window height
          on_select: On object select callback function. Controller passes
        selected objects in a list as the first parameter of the function.
        """
        # All objects list
        self.objects: Dict[str, Object] = {}
        # All Huds (Heads Up Display)
        self.huds: Dict[str, Hud] = {}
        self.__fps_counter = 0
        self.fps = 0

        self.__timer = -1.0
        # List of observers (cameras) in the scene. There can be only
        # one active observer at a time
        self.observers: List[Observer] = []
        self.observers.append(Observer(active=True))
        self.hudcam = Observer(
            active=True,
            position=[0, 0, 1.0],
            up=[0, 1.0, 0],
            perspective=False,
        )
        # Instead of looping through observers to find the active
        # observer, we are keeping the known index to avoid redundant
        # loops.
        self._active_observer = 0

        self.lights: List[Light] = []
        self.lights.append(Light())

        # List of all clocks. Clocks are time based function holders which
        # animate objects in the scene or do other stuff.
        self.clocks: Dict[str, Clock] = {}
        self.grid = Grid()
        self.controller = Controller()
        self._background = Background()

        # SDL Related Stuff
        self.window = None
        self.window_width = args.get("width", 800.0)
        self.window_height = args.get("height", 600.0)
        self._context = None
        self._mouse = [0, 0]
        self._shift_down = False
        self._ctrl_down = False
        self._rotate = False
        self._collision_detectors: List[CollisionTest] = []

        self.on_select = args.get("on_select", None)
        # Main running state
        self.running = False
        self._render_lock = False

    def _render(self) -> None:
        """
        Render the whole scene here. Note that, this is a private function and
        should not be overriden unless you really know what you are dealing
        with.
        """
        # Disable Depth Test to draw background at the very back of the scene
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # type: ignore
        self._background.render()
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        proj, view = self.active_observer.render()

        self.grid.render(proj, view, self.lights)

        self._render_lock = True
        for object in self.objects:
            self.objects[object].render(proj, view, self.lights)

        for object in self.huds:
            self.huds[object].render(proj, view, self.lights)
        self._render_lock = False

        for test in self._collision_detectors:
            test.check()

        self.__fps_counter += 1
        if self.__timer == -1:
            self.__timer = time.time()
        diff = time.time() - self.__timer
        if diff > 1:
            self.__timer = time.time()
            self.fps = self.__fps_counter
            self.__fps_counter = 0

    def add_collision_test(self, tester: CollisionTest) -> None:
        """Add collision test to the scene

        See `payton.scene.collision` module for details on collision

        Args:
          tester: Instance of `payton.scene.collision.CollisionTest`
        """
        if not isinstance(tester, CollisionTest):
            logging.error("tester must be an instance of CollisionTest")
            return

        self._collision_detectors.append(tester)

    def add_object(self, name: str, obj: Object) -> bool:
        """
        Add object to the scene root.

        Args:
          name: Name of the object, must be unique within its' scope.
          obj: Object (must be an instance of `payton.scene.geometry.Object`

        Return:
          bool: False in case of an error.

        Example usage:

            from payton.scene import Scene
            from payton.scene.geometry import Cube

            my_scene = Scene()
            cube = Cube()
            my_scene.add_object('cube', cube)
            my_scene.run()
        """
        if not isinstance(obj, Object):
            logging.error("Given object is not an instance of `scene.Object`")
            return False

        if name in self.objects:
            logging.error(f"Given object name [{name}] already exists")
            return False

        if isinstance(obj, Shape2D):
            logging.error("2D Shapes can't be added directly to the scene")
            return False

        if isinstance(obj, Hud):
            """Huds must be rendered in a different loop after rendering
            all objects"""
            if self._render_lock:
                while self._render_lock:
                    # Wait for render loop to release the lock
                    # TODO: This can cause a possible deadlock.
                    continue
            self.huds[name] = obj
            obj.name = name
            obj.set_size(self.window_width, self.window_height)
            return True

        if self._render_lock:
            while self._render_lock:
                # Wait for render loop to release the lock
                # TODO: This can cause a possible deadlock.
                continue
        self.objects[name] = obj
        obj.name = name
        return True

    def add_observer(self, obj: Observer) -> bool:
        """
        Add observer to the scene. Observer must be an instance of
        `payton.scene.observer.Observer`
        Generally this is not needed as scene has already a default observer.

        Args:
          obj: Observer object, instance of `payton.scene.observer.Observer`

        Return:
          bool: False in case of an error.
        """
        if not isinstance(obj, Observer):
            logging.error("Observer is not an instance of `scene.Observer`")
            return False

        self.observers.append(obj)
        return True

    @property
    def active_observer(self) -> Observer:
        """Return active observer

        Returns:
          observer
        """
        return self.observers[self._active_observer]

    def create_observer(self) -> None:
        """
        Create a new observer in the scene and add to scene observers.
        """
        self.observers.append(Observer())

    def create_clock(
        self,
        name: str,
        period: float,
        callback: Callable[[str, Type[Receiver], float, float], None],
    ) -> None:
        """
        Creates a clock in the scene. This is the preffered way to create a
        clock in the scene as it binds the clock to the scene by itself.

        Args:
          name: Name of the clock. Must be unique within the scene.
          period: Period of the clock (in seconds), time between each iteration
          callback: Callback function to call in each iteration. Callback
        function must have these arguments:
            name: Name of the clock that triggers callback function.
            scene: Scene reference to the callback function.
            period: Period of the clock (time difference between iterations)
            total: Total time elapsed from the beginning.

        Example usage:

            from payton.scene import Scene
            from payton.scene.clock import Clock


            def time_counter_callback(name, scene, period, total):
                print("Total time passed since beginning {}secs".format(total))

            my_scene = Scene()
            my_scene.create_clock('time_counter', 1.0, time_counter_callback)
            print("Hit space to start clock")
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
            logging.error(f"A clock named {name} already exists")
            return

        c = Clock(name, period, self, callback)  # type: ignore
        self.clocks[name] = c

    def run(self) -> int:
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
        sdl2.SDL_GL_SetAttribute(
            sdl2.SDL_GL_CONTEXT_PROFILE_MASK, sdl2.SDL_GL_CONTEXT_PROFILE_CORE
        )
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_MULTISAMPLEBUFFERS, 1)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_MULTISAMPLESAMPLES, 16)
        self.window = sdl2.SDL_CreateWindow(
            b"Payton Scene",
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            int(self.window_width),
            int(self.window_height),
            sdl2.SDL_WINDOW_OPENGL,
        )  # type: ignore

        if not self.window:
            return -1

        self._context = sdl2.SDL_GL_CreateContext(self.window)
        self.event = sdl2.SDL_Event()
        self.running = True

        # Fix aspect ratios of observers
        for observer in self.observers:
            observer.aspect_ratio = (
                self.window_width / self.window_height * 1.0
            )

        for clock in self.clocks:
            self.clocks[clock].start()

        while self.running:
            while sdl2.SDL_PollEvent(ctypes.byref(self.event)) != 0:
                self.controller.keyboard(self.event, self)
                self.controller.mouse(self.event, self)

            self._render()

            sdl2.SDL_GL_SwapWindow(self.window)
            sdl2.SDL_Delay(1)

        for obj in self.objects:
            self.objects[obj].destroy()
        for clock in self.clocks:
            self.clocks[clock].kill()

        sdl2.SDL_GL_DeleteContext(self._context)
        sdl2.SDL_DestroyWindow(self.window)
        self.window = None
        sdl2.SDL_Quit()
        return 0


class Background(object):
    """Background is a special object

    Only used by Scene and used for once. It has a special place in the render
    pipeline. This is the first object to be rendered in each cycle and before
    rendering this object, Scene disables depth test. So, every other element
    in the scene can be drawn on top of this background.

    (Shader code and idea derived from the original work of:
    https://www.cs.princeton.edu/~mhalber/blog/ogl_gradient/)
    """

    def __init__(self, **args: Dict[str, Any]):
        """Initialize background

        Args:
          top_color: Color at the top of the scene screen
          bottom_color: Color at the bottom of the screen
        """
        self.top_color = args.get("top_color", [0.0, 0.1, 0.2, 1.0])
        self.bottom_color = args.get("bottom_color", [0.1, 0.1, 0.1, 1.0])
        variables = ["top_color", "bot_color"]
        self._shader = Shader(
            fragment=background_fragment_shader,
            vertex=background_vertex_shader,
            variables=variables,
        )
        self._vao = None
        self.visible = True

    def render(self) -> None:
        """Render background of the scene"""
        if not self.visible:
            return

        if not self._vao:
            self._vao = glGenVertexArrays(1)
            glBindVertexArray(self._vao)
            self._shader.build()
            glBindVertexArray(0)

        glDisable(GL_DEPTH_TEST)

        self._shader.use()
        self._shader.set_vector4_np(
            "top_color", np.array(self.top_color, dtype=np.float32)
        )
        self._shader.set_vector4_np(
            "bot_color", np.array(self.bottom_color, dtype=np.float32)
        )
        glBindVertexArray(self._vao)
        glDrawArrays(GL_TRIANGLES, 0, 3)
        glBindVertexArray(0)
        self._shader.end()
        glEnable(GL_DEPTH_TEST)

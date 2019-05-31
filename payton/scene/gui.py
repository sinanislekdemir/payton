"""Graphical User Interface support

GUI Module contains several GUI elements which are all instance of
`payton.scene.geometry.Mesh` and mainly, their projection matrices
differ in pipeline.

Example code:

    from payton.scene import Scene
    from payton.scene.gui import Hud, Rectangle, Text

    scene = Scene()
    hud = Hud()
    rectangle = Rectangle(position=(10, 20), size=(300, 300))
    hud.add_child("rect", rectangle)

    text = Text(label="Hello World!")
    rectangle.add_child("label", text)
    hud.set_font("/Library/Fonts/Arial.ttf") # Path for Mac OS
    scene.add_object("hud", hud)

    scene.run()

"""

from PIL import Image, ImageDraw, ImageFont

from OpenGL.GL import glEnable, GL_DEPTH_TEST, glDisable

from payton.scene.geometry import Mesh, Object
from payton.math.matrix import ortho


class Shape2D(Mesh):
    """Shape2D is the base of GUI primitives

    An instance of `payton.scene.geometry.Mesh` so as a result of this,
    you can use the material information to change the color and texture
    of the shape.
    """

    def draw(self):
        """Placeholder for draw function"""
        pass

    def __init__(self, **args):
        """Initialize Shape2D

        Args:
          opacity: Opacity of the shape. (1: transparent)
          position: Position of the shape in screen. (0, 0 by default.)
        """
        super(Shape2D, self).__init__(**args)
        self.material.opacity = args.get("opacity", 0.5)
        self._position = args.get("position", (0, 0, 0))
        self.set_position(self._position)


class Rectangle(Shape2D):
    """Rectangle primitive, an instance of `payton.scene.gui.Shape2D`

    Args:
      size: Size of the rectangle.
    """

    def __init__(self, **args):
        super(Rectangle, self).__init__(**args)
        self.size = args.get("size", (100, 100))
        self._init = False
        self.draw()

    def draw(self):
        """Draw the rectangle.

        Basically, this method gets called right after the initialization
        to build the rectangle buffer.

        Also, if you change the size of rectangle, you have to call this
        method again.
        """
        x, y = 0, 0
        w, h = self.size
        self.clear_triangles()
        if not self._init:
            self.add_triangle(
                [[x, y, 1], [x + w, y, 1], [x + w, y + h, 1]],
                texcoords=[[0, 0], [1, 0], [1, 1]],
            )
            self.add_triangle(
                [[x, y, 1], [x + w, y + h, 1], [x, y + h, 1]],
                texcoords=[[0, 0], [1, 1], [0, 1]],
            )
            self._init = True


class Text(Rectangle):
    """Text object.

    This is an instance of `payton.scene.gui.Rectangle` and uses Pillow
    library to generate a texture with a text and assigns the texture
    to own material
    """

    def __init__(self, **args):
        """Initialize Text

        Args:
          label: Label of the text. (Text to be written)
        """
        super(Text, self).__init__(**args)
        self.label = args.get("label", "lorem")
        self._font = None
        self.draw_text()

    def set_font(self, font):
        """Set font of the text

        Args:
          font: An instance of `PIL.ImageFont`
        """
        self._font = font
        self.draw_text()

    def draw_text(self):
        """Draw text

        Create an empty transparent image with the rectangle size
        and draw the text on it. Then, assign the image to the material
        """
        img = Image.new("RGBA", self.size, color=(0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        if self._font is not None:
            d.text((5, 5), self.label, fill=(0, 0, 0), font=self._font)
        else:
            d.text((5, 5), self.label, fill=(0, 0, 0))

        self.material._image = img
        self.material.refresh()


def assign_font(obj, font):
    """Assign font to given object and children recursively.

    Args:
      obj: Object to assing the font
      font: Font to be assigned. (Instance of `PIL.ImageFont`)
    """
    if isinstance(obj, Text):
        obj.set_font(font)
    for name in obj.children:
        assign_font(obj.children[name], font)


class Hud(Object):
    """Main 2D Hud

    HUD stands for Heads Up Display and it is the Graphical User Interface
    layer on top of your 3D Scene. HUDs are rendered after your 3D Scene by
    disabling Depth Test. So, order of drawing is an important factor here.

    You can create a HUD for debugging reasons. If you want to use any 2D
    primitives, you need to add it into a HUD.

    One HUD is enough to handle all your 2D drawings but it is not restricted
    to one in the scene.
    """

    def __init__(self, **args):
        """Initialize HUD

        Args:
          width: Window width (overriden by Scene on add_object)
          height: Window height (overriden by Scene on add_object)
          font: Path to font file (See:
            https://pillow.readthedocs.io/en/3.1.x/reference/ImageFont.html)
          font_size: Font size
        """

        super(Hud, self).__init__(**args)
        self.width = args.get("width", 800)
        self.height = args.get("height", 600)
        self.font = args.get("font", "")
        self.font_size = args.get("font_size", "")
        self._font = None
        self._projection_matrix = None

    def render(self, *_args):
        """Render HUD

        Disables depth test and renders child primitives
        """
        glDisable(GL_DEPTH_TEST)
        if self._projection_matrix is None:
            self._projection_matrix = ortho(
                0, self.width, self.height, 0, True
            )

        for child in self.children:
            self.children[child].render(self._projection_matrix, None, [])
        glEnable(GL_DEPTH_TEST)

    def set_size(self, w, h):
        """Set size of the HUD view

        Args:
          w: Width of the viewport
          h: Height of the viewport
        """
        self.width = w
        self.height = h

    def set_font(self, font_name, font_size=15):
        """Set font of the HUD

        Args:
          font_name: Path to the font file. TrueType or OpenType fonts are
            also supported.
          font_size: Font size.
        """
        self._font = ImageFont.truetype(font_name, font_size)
        for name in self.children:
            assign_font(self.children[name], self._font)

from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.scrollview import ScrollView
from kivy.graphics import Color, Line
from gfx_font import GFXFont

mylcdfonts = [
    GFXFont("assets/u8x8_font_chroma48medium8_r.json"),
    GFXFont("assets/u8x8_font_7x14B_1x2_f.json"),
]

myfonts = [
    GFXFont("assets/basefont.json"),
    GFXFont("assets/Terminal11x16.json"),
    GFXFont("assets/Terminal11x16.json"),
    GFXFont("assets/Terminal11x16.json"),
    GFXFont("assets/Terminal11x16.json"),
    GFXFont("assets/FreeSans9pt7b.json"),
    GFXFont("assets/FreeSans12pt7b.json"),
    GFXFont("assets/Picopixel.json"),
    GFXFont("assets/FreeSans18pt7b.json"),
    GFXFont("assets/FreeMono9pt8b.json"),
    GFXFont("assets/FreeMono12pt8b.json"),
    GFXFont("assets/ttgologo.json"),
]


border = 3

class DrawingContainer(BoxLayout):
    def __init__(self, parser, ttgodata, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        self.scroll_view = ScrollView(size_hint=(1, 1), bar_width=10)

        # FloatLayout to hold the fixed-size object
        self.float_layout = FloatLayout(size_hint=(None, None))
        self.scroll_view.add_widget(self.float_layout)

        # Add ScrollView to the main layout
        self.add_widget(self.scroll_view)

        # Fixed-size object
        self.drawing_area = DrawingArea(parser, ttgodata)
        self.float_layout.add_widget(self.drawing_area)

        # Bind to size changes of the drawing container
        self.bind(size=self.update_layout)

    def update_size(self, width, height, scale):
        """
        Update the size of the drawing area and redraw it.
        """
        self.drawing_area.update_drawing_size(width, height, scale)
        self.update_layout()

    def redraw(self, block_name):
        """
        Redraw the content in the drawing area.
        """
        self.drawing_area.redraw(block_name)

    def update_layout(self, *args):
        """
        Update the layout to center the fixed-size object in the available space.
        """
        # Make the FloatLayout size match the DrawingArea or the container size
        self.float_layout.size = (
            max(self.drawing_area.width, self.width),
            max(self.drawing_area.height, self.height),
        )

        # Center the DrawingArea within the FloatLayout
        self.drawing_area.center_x = self.float_layout.width / 2
        self.drawing_area.center_y = self.float_layout.height / 2


class DrawingArea(Widget):
    def __init__(self, parser, ttgodata, **kwargs):
        super().__init__(**kwargs)
        self.parser = parser
        self.ttgodata = ttgodata
        self.current_block = None
        self.width = 240
        self.height = 176
        self.islcd = False
        self.size_hint = (None, None)
        self.scale = 1
        self.block_name = None

        # Bind to size and position changes to update the frame
        self.bind(size=self.update_frame, pos=self.update_frame)

    def update_drawing_size(self, width, height, scale):
        """
        Update the size of the drawing area.
        """
        self.width = width * scale + 2 * border
        self.height = height * scale + 2 * border
        self.size = (self.width, self.height)
        self.scale = scale
        if self.width // self.scale < 176:
            print("Using LCD fonts")
            self.islcd = True
        else:
            print(f"Using TFT fonts {self.width} {self.scale} {self.width//self.scale}")
            self.islcd = False
        self.update_frame()

    def update_frame(self, *args):
        """
        Update the frame surrounding the drawing area.
        """
        self.redraw(self.block_name)

    def redraw(self, block_name):
        """
        Redraw the content within the drawing area.
        """
        self.canvas.before.clear()
        with self.canvas.before:
            Color(0.5, 0.5, 0.5)  # Gray frame color
            Line(rectangle=(
                self.x + border,
                self.y + border,
                self.width - 2 * border,
                self.height - 2 * border
            ), width=border)

        if not block_name:
            return

        self.block_name = block_name

        # Parse and render objects
        objects = self.parser.parse_objects(block_name)
        for obj in objects:
            obj.render(
                self.canvas.before,
                state=self.ttgodata,
                area_height=self.height,
                scale=self.scale,
                islcd = self.islcd,
                offset_x= self.x + border + 1,
                offset_y=- self.y + border,
            )


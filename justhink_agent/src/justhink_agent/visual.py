import importlib_resources

import pyglet
from pyglet.window import key

import rospy

from justhink_world.tools.graphics import Graphics
from justhink_world.agent.visual import MentalScene
from justhink_world.tools.read import load_image_from_reference


class RoboticAgentWindow(pyglet.window.Window):
    def __init__(self, worlds, width=1920, height=1080, screen_no=0):
        scenes = dict()
        for name, world in worlds.items():
            scenes[name] = MentalScene(world, width, height)
        self._scenes = scenes

        self._cur_activity_name = None

        self.says_text = ''
        self.does_text = ''
        self.observes_text = ''

        self._init_graphics(width, height)

        style = pyglet.window.Window.WINDOW_STYLE_BORDERLESS
        super().__init__(
            width, height, caption="JUSThink Robot", 
            style=style, visible=False)

        image_container = importlib_resources.files(
            'justhink_agent.resources.images')
        icon1 = load_image_from_reference(
            image_container.joinpath('robot_256x256.png'))
        icon2 = load_image_from_reference(
            image_container.joinpath('robot_128x128.png'))
        self.set_icon(icon1, icon2)

        rospy.on_shutdown(self.on_close)

        self.register_event_type('on_update')
        self.set_handler('on_update', self.on_update)

        self.screen_no = screen_no

        self.update()

    def move_window(self):
        # Move the window to a screen in possibly a dual-monitor setup.
        offset = (1920, 0)
        display = pyglet.canvas.get_display()
        screens = display.get_screens()
        # 0 for the laptop screen, e.g. 1 for the external screen
        active_screen = screens[self.screen_no]
        self.set_location(active_screen.x+offset[0], active_screen.y+offset[1])

    @property
    def cur_activity_name(self):
        """Name of the current scene being rendered."""
        return self._cur_activity_name

    @cur_activity_name.setter
    def cur_activity_name(self, value):
        if value in self._scenes:
            self._cur_activity_name = value
        else:
            self._cur_activity_name = None

    @property
    def cur_scene(self):
        """Current scene being rendered."""
        if self._cur_activity_name is not None:
            return self._scenes[self._cur_activity_name]
        else:
            return None

    # GUI methods.

    def on_draw(self):
        self.clear()
        if self.cur_scene is not None:
            self.cur_scene.on_draw()
        self.graphics.batch.draw()

    def on_key_press(self, symbol, modifiers):
        if symbol == key.ESCAPE and modifiers & key.MOD_CTRL:
            rospy.signal_shutdown('Agent window is closed by CTRL+ESCAPE.')
            self.close()

    def on_close(self):
        rospy.loginfo("Closing robot cognition node & its window...")

        rospy.signal_shutdown('Robot cognition node & window is closed.')

        pyglet.app.exit()

    # Custom public methods.

    def update(self):
        pyglet.clock.schedule_once(self.update_callback, 0)

    def update_callback(self, dt):
        self.on_update()

    def on_update(self):
        self.switch_to()
        # Update the scene.
        if self.cur_scene is not None:
            self.cur_scene.on_update()
        try:
            self.graphics.says_label.text = self.says_text
            self.graphics.does_label.text = self.does_text
            if self.cur_scene is not None:
                self.cur_scene.graphics.observes_label.text = self.observes_text
        except Exception as e:
            rospy.logerr(e)

    def _init_graphics(self, width, height, batch=None, font_size=20):
        graphics = Graphics(width, height, batch=batch)
        batch = graphics.batch

        padding = (50, 50, 40, 60)  # left, right, top, bottom
        x_pad, y_pad = (300, 100)

        # Initialize "says" texts.
        y = height - padding[2]
        graphics.says_heading_label = pyglet.text.Label(
            '(robot says)', x=padding[0], y=y, font_name='Sans',
            font_size=font_size, align='center', batch=batch)
        graphics.says_label = pyglet.text.Label(
            self.says_text, x=padding[0]+x_pad, y=y, font_name='Sans',
            multiline=True, font_size=font_size, width=1500, batch=batch)

        # Initialize "does" texts.
        y = height - padding[2] - y_pad
        graphics.does_heading_label = pyglet.text.Label(
            '(robot does)', x=padding[0], y=y, font_name='Sans',
            font_size=font_size, batch=batch)
        graphics.does_label = pyglet.text.Label(
            self.does_text, x=padding[0]+x_pad, y=y, align='center',
            font_name='Sans', font_size=font_size, batch=batch)

        self.graphics = graphics

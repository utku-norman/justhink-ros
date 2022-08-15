import pyglet

from pyglet.window import key


from justhink_world.tools.graphics import Scene
from justhink_world.visual import WorldScene, CollaborativeWorldScene, \
    HumanIndividualWorldScene


from justhink_world.tools.graphics import ButtonWidget, TextWidget, Graphics, \
    BLACKA
from justhink_world.tools.read import load_image_from_reference


class CoverScene(Scene):
    def __init__(self, name, width, height, **kwargs):
        super().__init__(name=name, width=width, height=height)

        self.label = pyglet.text.Label(
            'Welcome!', font_name='Sans', font_size=48,
            anchor_x='center', anchor_y='center',
            color=(0, 0, 0, 255), x=width//2, y=height//2)

    def on_draw(self):
        pyglet.gl.glClearColor(1, 1, 1, 1)
        self.label.draw()

    def on_key_press(self, symbol, modifiers, win):
        if symbol == key.S and modifiers & key.MOD_CTRL:
            win.scene_no = win.scene_no + 1


class WelcomeScene(Scene):
    def __init__(self, name, width, height, **kwargs):
        super().__init__(name=name, width=width, height=height)

        self.label = pyglet.text.Label(
            'Press to begin!', x=width//2, y=height//2, font_name='Sans',
            font_size=48, anchor_x='center', anchor_y='center',
            color=(0, 0, 0, 255))

    def on_draw(self):
        pyglet.gl.glClearColor(1, 1, 1, 1)
        self.label.draw()

    def on_mouse_press(self, x, y, button, modifiers, win):
        robot_text_height = win.graphics.robot_text_rect.height
        in_robot_text = y > win.graphics.height - robot_text_height

        if not in_robot_text:  # and not in_help_button:
            win.scene_no = win.scene_no + 1


class DrawingScene(WorldScene):

    @WorldScene.draw_from.setter
    def draw_from(self, value):
        is_changed = self.set_draw_from(value)
        if is_changed:
            self.win.publish_drawing_change(self.draw_from, self.draw_to)

    @WorldScene.draw_to.setter
    def draw_to(self, value):
        is_changed = self.set_draw_to(value)
        if is_changed:
            self.win.publish_drawing_change(self.draw_from, self.draw_to)


class DebriefingScene(DrawingScene, HumanIndividualWorldScene):
    pass


class TestScene(DrawingScene, HumanIndividualWorldScene):

    def on_mouse_release(self, x, y, button, modifiers, win):
        super().on_mouse_release(x, y, button, modifiers, win)

        if self._state.is_terminal:
            win.scene_no = win.scene_no + 1


class CollaborativeScene(DrawingScene, CollaborativeWorldScene):

    def on_mouse_release(self, x, y, button, modifiers, win):
        super().on_mouse_release(x, y, button, modifiers, win)


class ProfileScene(Scene):
    """profile: Enter name and year of birth."""

    def __init__(self, name, width, height, **kwargs):
        super().__init__(name=name, width=width, height=height)
        self._init_graphics()

    def on_draw(self):
        self.graphics.batch.draw()

    def _init_graphics(self, width, height, image_container, batch=None):
        graphics = Graphics(width, height, batch=batch)
        batch = graphics.batch

        groups = [pyglet.graphics.OrderedGroup(i) for i in range(10)]

        ref = image_container.joinpath('profile_background.jpg')
        image = load_image_from_reference(ref)
        graphics.bg_sprite = pyglet.sprite.Sprite(
            image, batch=batch, group=groups[0])

        # Initialize the buttons.
        button_pad = 200
        c = image_container
        paths = {
            ButtonWidget.ENABLED: c.joinpath('profile_submit_enabled.png'),
            ButtonWidget.DISABLED: c.joinpath('profile_submit_disabled.png'),
            ButtonWidget.SELECTED: c.joinpath('profile_submit_selected.png'),
        }
        graphics.submit_button = ButtonWidget(
            width-button_pad, height-button_pad, paths=paths, scale=0.4,
            state=ButtonWidget.DISABLED, batch=batch, group=groups[1])

        paths = {
            ButtonWidget.ENABLED: c.joinpath('male_avatar_enabled.png'),
            ButtonWidget.SELECTED: c.joinpath('male_avatar_selected.png'),
        }
        graphics.male_button = ButtonWidget(
            500, 330, paths=paths, state=ButtonWidget.ENABLED, scale=0.25,
            batch=batch, group=groups[1])

        paths = {
            ButtonWidget.ENABLED: c.joinpath('female_avatar_enabled.png'),
            ButtonWidget.SELECTED: c.joinpath('female_avatar_selected.png'),
        }
        graphics.female_button = ButtonWidget(
            200, 330, paths=paths, state=ButtonWidget.ENABLED, scale=0.25,
            batch=batch, group=groups[1])

        # Initialize the text fields.
        def valid_name(text): return text.isalpha() \
            and len(text) > 2

        def valid_name_entry(char): return char.isalpha()

        def valid_age(text): return text.isdigit() \
            and int(text) < 2020 and int(text) > 1940

        def valid_age_entry(char): return char.isdigit()

        pad = 600
        font_size = 48
        graphics.name_field = TextWidget(
            '?', pad, height-280, 350, 'Name', font_size=font_size,
            valid_func=valid_name, entry_func=valid_name_entry,
            batch=batch, group=groups[1])
        graphics.year_field = TextWidget(
            '0', pad, height-500, 350, 'Year', font_size=font_size,
            valid_func=valid_age, entry_func=valid_age_entry,
            batch=batch, group=groups[1])

        # Initialize the text labels.
        graphics.name_label = pyglet.text.Label(
            'Name:', x=100, y=height-280, font_name='Sans',
            font_size=font_size, color=BLACKA, batch=batch, group=groups[1])

        graphics.year_label = pyglet.text.Label(
            'Year of birth:', x=100, y=height-500, font_name='Sans',
            font_size=font_size, color=BLACKA, batch=batch, group=groups[1])

        self.graphics = graphics

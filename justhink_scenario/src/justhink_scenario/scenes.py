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

        # in_help_button = win.graphics.help_button.check_hit(x, y)

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

    # @WorldScene.draw_from.setter
    # def draw_from(self, value):
    #     if self._draw_from != value:
    #         self._draw_from = value
    #         self.win.publish_drawing_change(self._draw_from, self._draw_to)

    # @WorldScene.draw_to.setter
    # def draw_to(self, value):
    #     if self._draw_to != value:
    #         self._draw_to = value
    #         self.win.publish_drawing_change(self._draw_from, self._draw_to)


class TestScene(DrawingScene, HumanIndividualWorldScene):

    def on_mouse_release(self, x, y, button, modifiers, win):
        super().on_mouse_release(x, y, button, modifiers, win)

        if self._state.is_terminal:
            win.scene_no = win.scene_no + 1


class CollaborativeScene(DrawingScene, CollaborativeWorldScene):

    def on_mouse_release(self, x, y, button, modifiers, win):
        super().on_mouse_release(x, y, button, modifiers, win)

        # if self._state.is_terminal:
        #     win.scene_no = win.scene_no + 1


class ProfileScene(Scene):
    """profile: Enter name and year of birth."""

    def __init__(self, name, width, height, **kwargs):
        super().__init__(name=name, width=width, height=height)
        self._init_graphics()
        # self.label = pyglet.text.Label(
        #     'Press to begin!', x=width//2, y=height//2, font_name='Sans',
        #     font_size=48, anchor_x='center', anchor_y='center',
        #     color=(0, 0, 0, 255))

    def on_draw(self):
        self.graphics.batch.draw()

    # def __init__(self, width, height, **kwargs):
    #     super().__init__(width=width, height=height)

    #     # self._images_dir = pl.Path(images_dir)

    #     batch, graphics = init_profile_graphics(self._images_dir)
    #     self._batch = batch
    #     for name, obj in graphics.items():
    #         setattr(self, name, obj)

    #     self._name_field.document.text = ''
    #     self._year_field.document.text = ''
    #     self._submit_button.set_state('disabled')
    #     self._male_button.set_state('enabled')
    #     self._female_button.set_state('enabled')

    #     self.focus = None
    #     self.set_focus(self._name_field)

    #     self.profile_pub = rospy.Publisher(
    #         '~profile',
    #         justhink_msgs.msg.Profile,
    #         queue_size=10)

    # def on_draw(self):
    #     self._batch.draw()

    # def on_mouse_press(self, x, y, button, modifiers, win):
    #     for widget in [self._name_field, self._year_field]:
    #         if widget.check_hit(x, y):
    #             self.set_focus(widget)
    #             break

    #     if self.focus:
    #         self.focus.caret.on_mouse_press(x, y, button, modifiers)

    #     # Check if submit button is pressed.
    #     if self._submit_button.check_hit(x, y) and self.check_done():
    #         self.publish_profile()
    #         self._submit_button.set_state('selected')
    #     elif self._female_button.check_hit(x, y):
    #         self._female_button.set_state('selected')
    #         self._male_button.set_state('enabled')
    #         self.update_submit_button()
    #     elif self._male_button.check_hit(x, y):
    #         self._male_button.set_state('selected')
    #         self._female_button.set_state('enabled')
    #         self.update_submit_button()

    # def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers, win):
    #     if self.focus:
    #         self.focus.caret.on_mouse_drag(x, y, dx, dy, buttons, modifiers)

    # def publish_profile(self):
    #     message = self.make_profile_message()
    #     self.profile_pub.publish(message)
    #     rospy.loginfo(message)

    # def set_focus(self, focus):
    #     if self.focus:
    #         self.focus.caret.visible = False
    #         self.focus.caret.mark = 0
    #         self.focus.caret.position = 0

    #     self.focus = focus
    #     if self.focus:
    #         self.focus.caret.visible = True
    #         self.focus.caret.mark = 0
    #         self.focus.caret.position = len(self.focus.document.text)

    # def change_focus(self, dir=1):
    #     widgets = [self._name_field, self._year_field]
    #     if self.focus in widgets:
    #         i = widgets.index(self.focus)
    #     else:
    #         i = 0
    #         dir = 0
    #     offset = 1
    #     widget = widgets[(i + dir * offset) % len(widgets)]
    #     self.set_focus(widget)

    # def on_text(self, text):
    #     if self.focus:
    #         if self.focus.caret.position == 0:
    #             text = text.upper()

    #         if self.focus.entry_func(text):
    #             self.focus.caret.on_text(text)

    #         self.update_submit_button()

    # def on_text_motion(self, motion):
    #     if self.focus:
    #         self.focus.caret.on_text_motion(motion)

    #         self.update_submit_button()

    # def on_text_motion_select(self, motion):
    #     if self.focus:
    #         self.focus.caret.on_text_motion_select(motion)

    # def on_key_press(self, symbol, modifiers, win):
    #     if symbol == key.UP:
    #         self.change_focus(-1)
    #     elif symbol == key.DOWN or symbol == key.TAB:
    #         self.change_focus(1)

    # def update_submit_button(self):
    #     if self.check_done():
    #         self._submit_button.set_state('enabled')
    #     else:
    #         self._submit_button.set_state('disabled')

    # def check_done(self):
    #     # Check the validity of the text widgets.
    #     for widget in [self._name_field, self._year_field]:
    #         if not widget.check_valid():
    #             return False

    #     # Check validity of gender selection.
    #     if self._male_button.state != 'selected' and \
    #             self._female_button.state != 'selected':
    #         return False

    #     return True

    def _init_graphics(self, width, height, image_container, batch=None):
        graphics = Graphics(width, height, batch=batch)
        batch = graphics.batch

        groups = [pyglet.graphics.OrderedGroup(i) for i in range(10)]

        # background
        ref = image_container.joinpath('profile_background.jpg')
        image = load_image_from_reference(ref)
        graphics.bg_sprite = pyglet.sprite.Sprite(
            image, batch=batch, group=groups[0])

        # Initialise the buttons.
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

        # Initialise the text fields.
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

        # Initialise the text labels.
        graphics.name_label = pyglet.text.Label(
            'Name:', x=100, y=height-280, font_name='Sans',
            font_size=font_size, color=BLACKA, batch=batch, group=groups[1])

        graphics.year_label = pyglet.text.Label(
            'Year of birth:', x=100, y=height-500, font_name='Sans',
            font_size=font_size, color=BLACKA, batch=batch, group=groups[1])

        self.graphics = graphics

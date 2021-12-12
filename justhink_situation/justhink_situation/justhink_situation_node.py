from justhink_world import create_world
from justhink_world.visual import WorldWindow, WorldScene, RobotIndividualWorldScene
from justhink_world.domain.action import PickAction

import justhink_interfaces.msg

import pyglet
from pyglet.window import mouse, key

import rclpy
from rclpy.node import Node

from .messages import make_action_message


class Situation(Node):

    def __init__(self, name='robot-individual-1'):
        super().__init__('justhink_situation')

        self.get_logger().info('Initialising publishers...')
        self._init_ros()

        screen_index = -1
        drawing_mode = 'click'

        # Create a world.
        self.name = name
        self.get_logger().info('Creating the world {}...'.format(self.name))
        world = create_world(self.name)

        class DrawingScene(WorldScene):
            @WorldScene.draw_from.setter
            def draw_from(scene, value):
                is_changed = scene.set_draw_from(value)
                if is_changed:
                    self.publish_drawing_change(scene.draw_from, scene.draw_to)

            @WorldScene.draw_to.setter
            def draw_to(scene, value):
                is_changed = scene.set_draw_to(value)
                if is_changed:
                    self.publish_drawing_change(scene.draw_from, scene.draw_to)

        class IndividualScene(RobotIndividualWorldScene, DrawingScene):
            pass

        # Visualise the world on the last attached screen (by default).
        # It is non-blocking.
        self.get_logger().info('Creating the window...')
        window = WorldWindow(
            world, screen_index=screen_index, drawing_mode=drawing_mode,
            scene_type=IndividualScene)

        @window.event
        def on_mouse_motion(x, y, dx, dy):
            self._publish_mouse_motion(x, y, dx, dy)

        @window.event
        def on_mouse_press(x, y, button, modifiers):
            self._publish_mouse_press(x, y, button)
            self.get_logger().info('Mouse pressed at x:{}, y:{}'.format(x, y))

        @window.event
        def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
            self._publish_mouse_drag(x, y, buttons, dx=dx, dy=dy)
            # self.get_logger().info(
            #   'Mouse dragged at x:{}, y:{}'.format(x, y))

        @window.event
        def on_mouse_release(x, y, button, modifiers):
            self._publish_mouse_release(x, y, button)
            self.get_logger().info('Mouse released at x:{}, y:{}'.format(x, y))

        @window.event
        def on_key_press(symbol, modifiers):
            self._publish_key_press(symbol, modifiers)

            self.get_logger().info(
                make_key_info_text(symbol, modifiers, 'pressed'))

            if symbol == key.ESCAPE:
                window.on_close()
            if symbol == key.A:
                action = window.get_agent_action()
                self.publish_agent_intention(action)

        @window.event
        def on_key_release(symbol, modifiers):
            self._publish_key_release(symbol, modifiers)

            self.get_logger().info(
                make_key_info_text(symbol, modifiers, 'released'))

            # if symbol == key.ESCAPE:
            #     window.on_close()

        @window.event
        def on_close():
            raise KeyboardInterrupt
            # self.get_logger().info("Closing application window...")
            # rclpy.shutdown()
            # pyglet.app.exit()

        self.world = world
        self.window = window

        self.get_logger().info('Ready!')

    # ROS methods.

    def _init_ros(self):
        # Initialize the publishers.
        # Mouse events.
        self.mouse_motion_pub = self.create_publisher(
            justhink_interfaces.msg.Mouse, '~/mouse_motion', 10)
        self.mouse_press_pub = self.create_publisher(
            justhink_interfaces.msg.Mouse, '~/mouse_press', 10)
        self.mouse_drag_pub = self.create_publisher(
            justhink_interfaces.msg.Mouse, '~/mouse_drag', 10)
        self.mouse_release_pub = self.create_publisher(
            justhink_interfaces.msg.Mouse, '~/mouse_release', 10)
        # Keyboard events.
        self.key_press_pub = self.create_publisher(
            justhink_interfaces.msg.Key, '~/key_press', 10)
        self.key_release_pub = self.create_publisher(
            justhink_interfaces.msg.Key, '~/key_release', 10)

        # Other events.
        self.agent_intention_pub = self.create_publisher(
            justhink_interfaces.msg.Action, '~/agent_intention', 10)
        self.drawing_change_pub = self.create_publisher(
            justhink_interfaces.msg.EdgeDrawing, '~/drawing_change', 10)

        # Initialise messages.
        self.mouse_message = justhink_interfaces.msg.Mouse()
        self.key_message = justhink_interfaces.msg.Key()

    def _make_mouse_message(self, x, y, buttons=mouse.LEFT, dx=0, dy=0):
        self.mouse_message.header.frame_id = self.name
        self.mouse_message.header.stamp = self.get_clock().now().to_msg()
        self.mouse_message.x = x
        self.mouse_message.y = y
        self.mouse_message.dx = dx
        self.mouse_message.dy = dy
        self.mouse_message.left = bool(buttons & mouse.LEFT)
        self.mouse_message.right = bool(buttons & mouse.RIGHT)

    def _make_key_message(self, symbol, modifiers):
        self.key_message.header.frame_id = self.name
        self.key_message.header.stamp = self.get_clock().now().to_msg()
        self.key_message.symbol = str(symbol)
        self.key_message.modifiers = str(modifiers)

    def _make_drawing_message(self, current, u, v) \
            -> justhink_interfaces.msg.EdgeDrawing:
        """TODO"""
        message = justhink_interfaces.msg.EdgeDrawing()

        message.header.frame_id = current
        message.header.stamp = self.get_clock().now().to_msg()

        edge = justhink_interfaces.msg.Edge()
        edge.u = u if u is not None else -1
        edge.v = v if v is not None else -1
        message.edge = edge

        return message

    def _publish_mouse_motion(self, x, y, dx, dy):
        self._make_mouse_message(x, y, dx=dx, dy=dy)
        self.mouse_motion_pub.publish(self.mouse_message)

    def _publish_mouse_press(self, x, y, button):
        self._make_mouse_message(x, y, button)
        self.mouse_press_pub.publish(self.mouse_message)

    def _publish_mouse_drag(self, x, y, buttons, dx, dy):
        self._make_mouse_message(x, y, buttons, dx=dx, dy=dy)
        self.mouse_drag_pub.publish(self.mouse_message)

    def _publish_mouse_release(self, x, y, button):
        self._make_mouse_message(x, y, button)
        self.mouse_release_pub.publish(self.mouse_message)

    def _publish_key_press(self, symbol, modifiers):
        self._make_key_message(symbol, modifiers)
        self.key_press_pub.publish(self.key_message)

    def _publish_key_release(self, symbol, modifiers):
        self._make_key_message(symbol, modifiers)
        self.key_release_pub.publish(self.key_message)

    def publish_drawing_change(self, from_data, to_data):
        message = self._make_drawing_message(self.name, from_data, to_data)
        self.drawing_change_pub.publish(message)
        self.get_logger().info('Drawing changed to ({}, {})'.format(
            self.window.scene.draw_from, self.window.scene.draw_to))

    def publish_agent_intention(self, action):
        action_message = make_action_message(action)
        self.agent_intention_pub.publish(action_message)

        if isinstance(action, PickAction):
            u, v = self.world.env.state.network.get_edge_name(action.edge)
            action = action.__class__(edge=(u, v), agent=action.agent)
        self.get_logger().info('Robot intends to: {}'.format(action))


def make_mouse_info_text(pos, action_text):
    """Construct the text to inform about a mouse position change event."""
    if pos is None:
        s = 'Not {} yet.'.format(action_text)
    else:
        s = 'Last {} x: {:4d}, y:{:4d}'.format(
            action_text, pos[0], pos[1])
    return s


def make_key_info_text(symbol, modifiers, action_text):
    """Construct the text to inform about a key event."""
    s = 'Key {} {}'.format(key.symbol_string(symbol), action_text)
    mod_s = key.modifiers_string(modifiers)
    if len(mod_s) > 0:
        s = s + ' with modifiers {}'.format(mod_s)
    return s


def main(args=None):
    rclpy.init(args=args)

    situation = Situation()

    # pyglet.app.run()

    # Enter the main event loop.
    try:
        while True:
            pyglet.clock.tick()

            for window in pyglet.app.windows:
                window.switch_to()
                window.dispatch_events()
                window.dispatch_event('on_draw')
                window.flip()

            rclpy.spin_once(situation, timeout_sec=0.0)
    except (KeyboardInterrupt, RuntimeError):
        situation.window.close()
        print('Window is closed.')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    situation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

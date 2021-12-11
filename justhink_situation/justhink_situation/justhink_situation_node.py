from justhink_world import create_world
from justhink_world.visual import WorldWindow
from justhink_world.domain.action import PickAction, SuggestPickAction, \
    AgreeAction, DisagreeAction, ClearAction, ResetAction, \
    AttemptSubmitAction, ContinueAction, SubmitAction
from justhink_world.agent import Agent

import justhink_interfaces.msg

import pyglet
from pyglet.window import mouse, key

import rclpy
from rclpy.node import Node


class Situation(Node):

    def __init__(self):
        super().__init__('justhink_situation')

        self.get_logger().info('Initialising publishers...')
        self._init_ros()

        screen_index = -1
        drawing_mode = 'click'

        # Create a world.
        self.name = 'pretest-1'  # an individual world
        self.get_logger().info('Creating the world {}...'.format(self.name))
        world = create_world(self.name)

        # Visualise the world on the last attached screen (by default).
        # It is non-blocking.
        self.get_logger().info('Creating the window...')
        window = WorldWindow(
            world, screen_index=screen_index, drawing_mode=drawing_mode)

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
            elif symbol == key.A:
                action = window.get_agent_action()
                action_message = make_action_message(action)
                self.agent_intention_pub.publish(action_message)

        @window.event
        def on_key_release(symbol, modifiers):
            self._publish_key_release(symbol, modifiers)

            self.get_logger().info(
                make_key_info_text(symbol, modifiers, 'released'))

            if symbol == key.ESCAPE:
                window.on_close()

        @window.event
        def on_close():
            self.get_logger().info("Closing application window...")
            rclpy.shutdown()

            pyglet.app.exit()

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


def make_action_message(action) -> justhink_interfaces.msg.Action:
    """Construct a ROS message for an action."""
    message = justhink_interfaces.msg.Action()

    message.agent = action.agent
    u, v = -1, -1

    if isinstance(action, SuggestPickAction):
        message.type = justhink_interfaces.msg.Action.TYPE_SUGGEST_PICK
        u, v = action.edge
    elif isinstance(action, PickAction):
        message.type = justhink_interfaces.msg.Action.TYPE_PICK
        u, v = action.edge
    # elif isinstance(action, UnpickAction):
    #     message.type = justhink_interfaces.msg.Action.TYPE_UNPICK
    #     u, v = action.edge

    elif isinstance(action, AttemptSubmitAction):
        message.type = justhink_interfaces.msg.Action.TYPE_ATTEMPT_SUBMIT
    elif isinstance(action, ContinueAction):
        message.type = justhink_interfaces.msg.Action.TYPE_CONTINUE
    elif isinstance(action, SubmitAction):
        message.type = justhink_interfaces.msg.Action.TYPE_SUBMIT

    elif isinstance(action, AgreeAction):
        message.type = justhink_interfaces.msg.Action.TYPE_AGREE
    elif isinstance(action, DisagreeAction):
        message.type = justhink_interfaces.msg.Action.TYPE_DISAGREE

    elif isinstance(action, ClearAction):
        message.type = justhink_interfaces.msg.Action.TYPE_CLEAR
    else:
        print('making action message failed for {}'.format(action))

    message.edge = justhink_interfaces.msg.Edge(u=u, v=v)

    return message


def decode_action_message(data):
    """TODO: docstring for decode_action_message"""
    if data.agent == Agent.HUMAN:
        agent = Agent.HUMAN
    elif data.agent == Agent.ROBOT:
        agent = Agent.ROBOT
    else:
        raise ValueError

    edge = (data.edge.u, data.edge.v)

    if data.type == justhink_interfaces.msg.Action.TYPE_SUGGEST_PICK:
        action = SuggestPickAction(edge, agent=agent)
    elif data.type == justhink_interfaces.msg.Action.TYPE_PICK:
        action = PickAction(edge, agent=agent)
    # elif data.type == justhink_interfaces.msg.Action.TYPE_UNPICK:
    #     action = UnpickAction(edge, agent=agent)

    elif data.type == justhink_interfaces.msg.Action.TYPE_ATTEMPT_SUBMIT:
        action = AttemptSubmitAction(agent=agent)
    elif data.type == justhink_interfaces.msg.Action.TYPE_CONTINUE:
        action = ContinueAction(agent=agent)
    elif data.type == justhink_interfaces.msg.Action.TYPE_SUBMIT:
        action = SubmitAction(agent=agent)

    elif data.type == justhink_interfaces.msg.Action.TYPE_AGREE:
        action = AgreeAction(agent=agent)
    elif data.type == justhink_interfaces.msg.Action.TYPE_DISAGREE:
        action = DisagreeAction(agent=agent)

    elif data.type == justhink_interfaces.msg.Action.TYPE_CLEAR:
        action = ClearAction(agent=agent)

    elif data.type == justhink_interfaces.msg.Action.TYPE_RESET:
        action = ResetAction(agent=agent)
    else:
        raise ValueError

    return action


def main(args=None):
    rclpy.init(args=args)

    situation = Situation()

    # pyglet.app.run()

    # Enter the main event loop.
    while True:
        pyglet.clock.tick()

        for window in pyglet.app.windows:
            window.switch_to()
            window.dispatch_events()
            window.dispatch_event('on_draw')
            window.flip()

        rclpy.spin_once(situation, timeout_sec=0.0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    situation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

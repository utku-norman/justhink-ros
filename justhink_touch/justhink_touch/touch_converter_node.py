from justhink_world import create_world
from justhink_world.visual import WorldWindow

from justhink_world.domain.action import PickAction, SuggestPickAction, \
    AgreeAction, DisagreeAction, ClearAction, \
    AttemptSubmitAction, ContinueAction, SubmitAction

from justhink_situation.messages import decode_action_message


import rclpy
from rclpy.node import Node

import justhink_interfaces.msg


class TouchConverter(Node):

    def __init__(self, name='robot-individual-1'):
        super().__init__('justhink_touch')

        self.name = name
        self.get_logger().info('Creating the world {}...'.format(self.name))
        # A world to get node positions.
        self.world = create_world(self.name)
        # A hidden window to get button positions.
        self.window = WorldWindow(self.world, visible=False)

        # Subscribe to the agent intention topic.
        self.subscription = self.create_subscription(
            justhink_interfaces.msg.Action,
            '/justhink_situation/agent_intention',
            self.intention_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a touch pixel publisher.
        self.intended_points_pub = self.create_publisher(
            justhink_interfaces.msg.PointDrawing, '~/intended_points', 10)

        self.get_logger().info('Ready!')

    def intention_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)

        action = decode_action_message(msg)

        # Info.
        if isinstance(action, PickAction):
            u, v = self.world.env.state.network.get_edge_name(action.edge)
            a = action.__class__(edge=(u, v), agent=action.agent)
        self.get_logger().info('I heard robot intends to do: {}'.format(a))

        if isinstance(action, PickAction) \
                or isinstance(action, SuggestPickAction):
            u, v = action.edge
            pos_u = self.get_node_position(u)
            pos_v = self.get_node_position(v)
            self.publish_intended_points(pos_u, pos_v)

        elif isinstance(action, AttemptSubmitAction) \
                or isinstance(action, SubmitAction):
            button = self.window.scene.graphics.submit_button
            pos = button.x, button.y
            self.publish_intended_points(pos, pos)

    def get_node_position(self, node):

        node_data = self.world.cur_state.network.graph.nodes
        try:
            assert node in node_data
        except Exception as e:
            print(e)
            print('Node {} not found in node data {}'.format(node, node_data))

        return node_data[node]['x'], node_data[node]['y']

    def publish_intended_points(self, from_point, to_point):
        message = justhink_interfaces.msg.PointDrawing()

        message.from_point.x, message.from_point.y = from_point
        message.to_point.x, message.to_point.y = to_point

        self.get_logger().info('Robot intends to points: {}->{}'.format(
            from_point, to_point))

        self.intended_points_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)

    converter = TouchConverter()

    rclpy.spin(converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

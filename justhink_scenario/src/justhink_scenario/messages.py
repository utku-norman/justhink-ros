import rospy

import justhink_msgs.msg
import justhink_msgs.srv

from justhink_world.domain.state import NetworkState, EnvState
from justhink_world.domain.action import PickAction, SuggestPickAction, \
    AgreeAction, DisagreeAction, ClearAction, ResetAction, \
    AttemptSubmitAction, ContinueAction, SubmitAction

from justhink_world.agent import Agent


# ROBOT_SPEAKING = '·····'
ROBOT_SPEAKING = '(. . .)'


def decode_state_message(data, graph) -> EnvState:
    """TODO"""
    network_state = NetworkState(graph)
    for edge in data.network.edges:
        network_state.subgraph.add_edge(edge.u, edge.v)

    u = data.network.suggested_edge.u
    v = data.network.suggested_edge.v
    if u == -1 and v == -1:
        network_state.suggested_edge = None
    else:
        network_state.suggested_edge = (u, v)

    agents = set()
    for agent in data.agents:
        if agent == Agent.HUMAN:
            agents.add(Agent.HUMAN)
        elif agent == Agent.ROBOT:
            agents.add(Agent.ROBOT)
    agents = frozenset(agents)
    state = EnvState(network_state, agents=agents)

    state.attempt_no = data.attempt_no
    state.max_attempts = data.max_attempts
    if data.max_attempts == -1:
        state.max_attempts = None
    else:
        state.max_attempts = data.max_attempts

    state.is_submitting = data.is_submitting
    state.is_paused = data.is_paused
    state.is_terminal = data.is_terminal

    state.step_no = data.step_no

    return state


def decode_action_message(data):
    """TODO"""
    if data.agent == Agent.HUMAN:
        agent = Agent.HUMAN
    elif data.agent == Agent.ROBOT:
        agent = Agent.ROBOT
    else:
        raise ValueError

    edge = (data.edge.u, data.edge.v)

    if data.type == justhink_msgs.msg.Action.TYPE_SUGGEST_PICK:
        action = SuggestPickAction(edge, agent=agent)
    elif data.type == justhink_msgs.msg.Action.TYPE_PICK:
        action = PickAction(edge, agent=agent)
    # elif data.type == justhink_msgs.msg.Action.TYPE_UNPICK:
    #     action = UnpickAction(edge, agent=agent)

    elif data.type == justhink_msgs.msg.Action.TYPE_ATTEMPT_SUBMIT:
        action = AttemptSubmitAction(agent=agent)
    elif data.type == justhink_msgs.msg.Action.TYPE_CONTINUE:
        action = ContinueAction(agent=agent)
    elif data.type == justhink_msgs.msg.Action.TYPE_SUBMIT:
        action = SubmitAction(agent=agent)

    elif data.type == justhink_msgs.msg.Action.TYPE_AGREE:
        action = AgreeAction(agent=agent)
    elif data.type == justhink_msgs.msg.Action.TYPE_DISAGREE:
        action = DisagreeAction(agent=agent)

    elif data.type == justhink_msgs.msg.Action.TYPE_CLEAR:
        action = ClearAction(agent=agent)

    elif data.type == justhink_msgs.msg.Action.TYPE_RESET:
        action = ResetAction(agent=agent)
    else:
        raise ValueError

    return action


def make_drawing_message(current, u, v) -> justhink_msgs.msg.Drawing:
    """TODO"""
    message = justhink_msgs.msg.Drawing()

    message.header.frame_id = current
    message.header.stamp = rospy.Time.now()
    message.header.seq += 1
    message.u = u if u is not None else -1
    message.v = v if v is not None else -1

    return message


def make_button_message(current, name, state) -> justhink_msgs.msg.Button:
    """TODO"""
    message = justhink_msgs.msg.Button()

    message.header.frame_id = current
    message.header.stamp = rospy.Time.now()
    message.header.seq += 1
    message.name = name
    message.state = state

    return message


def make_action_message(action) -> justhink_msgs.msg.Action:
    """Construct a ROS message for an action."""
    message = justhink_msgs.msg.Action()

    message.agent = action.agent
    u, v = -1, -1

    if isinstance(action, SuggestPickAction):
        message.type = justhink_msgs.msg.Action.TYPE_SUGGEST_PICK
        u, v = action.edge
    elif isinstance(action, PickAction):
        message.type = justhink_msgs.msg.Action.TYPE_PICK
        u, v = action.edge
    # elif isinstance(action, UnpickAction):
    #     message.type = justhink_msgs.msg.Action.TYPE_UNPICK
    #     u, v = action.edge

    elif isinstance(action, AttemptSubmitAction):
        message.type = justhink_msgs.msg.Action.TYPE_ATTEMPT_SUBMIT
    elif isinstance(action, ContinueAction):
        message.type = justhink_msgs.msg.Action.TYPE_CONTINUE
    elif isinstance(action, SubmitAction):
        message.type = justhink_msgs.msg.Action.TYPE_SUBMIT

    elif isinstance(action, AgreeAction):
        message.type = justhink_msgs.msg.Action.TYPE_AGREE
    elif isinstance(action, DisagreeAction):
        message.type = justhink_msgs.msg.Action.TYPE_DISAGREE

    elif isinstance(action, ClearAction):
        message.type = justhink_msgs.msg.Action.TYPE_CLEAR
    else:
        rospy.logfatal('making action message failed for {}'.format(action))

    message.edge = justhink_msgs.msg.Edge(u=u, v=v)

    return message


def make_state_message(state) -> justhink_msgs.msg.EnvState:
    """Construct a ROS message for an environment state."""
    message = justhink_msgs.msg.EnvState()

    message.network = justhink_msgs.msg.NetworkState()

    # Add the selected edges.
    message.network.edges = list()
    for u, v in state.network.subgraph.edges():
        e = justhink_msgs.msg.Edge()
        e.u = u
        e.v = v
        message.network.edges.append(e)

    # Add the suggested edge if any, (-1, -1) otherwise.
    e = justhink_msgs.msg.Edge()
    if state.network.suggested_edge is not None:
        e.u, e.v = state.network.suggested_edge
    else:
        e.u, e.v = -1, -1
    message.network.suggested_edge = e

    message.agents = list(state.agents)
    message.attempt_no = state.attempt_no
    if state.max_attempts is None:
        message.max_attempts = -1
    else:
        message.max_attempts = state.max_attempts
    message.is_submitting = state.is_submitting
    message.is_paused = state.is_paused
    message.is_terminal = state.is_terminal

    message.step_no = state.step_no

    return message


def make_activity_transition_message(
        current, next,
        step=0, seq=0) -> justhink_msgs.msg.ActivityTransition:
    """Construct a ROS message for activity changes."""
    message = justhink_msgs.msg.ActivityTransition()

    message.header.frame_id = current
    message.header.stamp = rospy.Time.now()
    message.header.seq = 0

    message.current = current
    message.next = next

    return message


def make_state_transition_message(
        state, action, next_state,
        current='') -> justhink_msgs.msg.StateTransition:
    """Construct a ROS message for state transitions."""
    message = justhink_msgs.msg.StateTransition()

    message.header.frame_id = current
    message.header.stamp = rospy.Time.now()
    # message.header.seq = action_no

    message.state = make_state_message(state)
    message.action = make_action_message(action)
    message.next_state = make_state_message(next_state)

    return message


# def make_profile_message(self):
#     message = justhink_msgs.msg.Profile()

#     message.header.frame_id = 'profile'
#     message.header.stamp = rospy.Time.now()
#     message.header.seq = message.header.seq + 1

#     message.valid = self.check_done()

#     message.name = self._name_field.get_content()
#     message.year = str(self._year_field.get_content())
#     if self._male_button.state == 'selected':
#         message.gender = 'male'
#     elif self._female_button.state == 'selected':
#         message.gender = 'female'
#     else:
#         message.gender = '?'

#     return message

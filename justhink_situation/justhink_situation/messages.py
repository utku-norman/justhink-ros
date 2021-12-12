from justhink_world.domain.action import PickAction, SuggestPickAction, \
    AgreeAction, DisagreeAction, ClearAction, \
    AttemptSubmitAction, ContinueAction, SubmitAction

from justhink_world.agent import Agent

import justhink_interfaces.msg


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

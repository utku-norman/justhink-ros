import random
import pprint

import networkx as nx

import rospy

# Standard ROS messages
import std_msgs.msg
# Custom ROS messages and services
import justhink_msgs.msg
import justhink_msgs.srv

from justhink_world import create_all_worlds
from justhink_world.domain.action import SuggestPickAction, \
    AgreeAction, DisagreeAction, ClearAction, SubmitAction, AttemptSubmitAction
from justhink_world.domain.observation import Observation
from justhink_world.agent import Agent
from justhink_world.agent.reasoning import BetterThanExplanation, \
    ConnectedExplanation

from justhink_scenario.logging import log_publish, log_heard, \
    log_service_response, log_service_call
from justhink_scenario.messages import decode_state_message, \
    decode_action_message, make_action_message, ROBOT_SPEAKING

from .visual import RoboticAgentWindow
from .tools import make_service_proxy


def decision(probability):
    """https://stackoverflow.com/a/5887040"""
    return random.random() < probability


class RoboticAgent(object):
    """TODO: docstring for RoboticAgent"""

    def __init__(self):
        super().__init__()

        seed = 5
        random.seed(seed)

        param_name = '~with_robot'
        self.with_robot = rospy.get_param(param_name, 'auto')
        if rospy.has_param(param_name):
            rospy.delete_param(param_name)

        param_name = '~instruct'
        self.is_instructing = rospy.get_param(param_name, True)
        if rospy.has_param(param_name):
            rospy.delete_param(param_name)
        # print()
        if self.is_instructing:
            rospy.logwarn('Robot will introduce the next activity.')
        else:
            rospy.logwarn('Robot will NOT introduce the next activity.')
        print()

        param_name = '~lang'
        self.lang = rospy.get_param(param_name, 'en-US')
        if rospy.has_param(param_name):
            rospy.delete_param(param_name)

        self.is_disagreeing = False

        self.cur_world = None

        s = ("Connect the gold mines to each other,"
             " so that miners can go from any mine to any other"
             "\nby some path: try to spend as little as you can.")
        self.goal_help_text = s

        s = ('I made a suggestion. Either agree with me by pressing check,'
             '\nto connect it, or disagree by pressing the cross.')
        self.agreement_help_text = s

        self.last_speech_text = None
        self.cur_activity = None

        self.help_text = 'Hello!'

        worlds = create_all_worlds()  # verbose=True)
        self.world = worlds['collaboration-1']
        self.world2 = worlds['collaboration-2']
        self.world_tutorial = worlds['tutorial']

        trackeds = ['collaboration-1', 'collaboration-2', 'tutorial']
        tracked_worlds = {k: v for k, v in worlds.items() if k in trackeds}
        window = RoboticAgentWindow(tracked_worlds)

        self.window = window

        # Wait for the human app to become online.
        self.init_ros()

        # Configure the robot speech.
        if self.with_robot:
            try:
                opts = {'speed': 82, 'language': self.lang}
                resp = self.call_configure_speech(**opts)
                if resp.status:
                    s = 'Robot speech configured: speed={}, lang={}'.format(
                        opts['speed'], opts['language'])
                    rospy.loginfo(s)
                else:
                    s = ('Failed to configure robot speech:'
                         ' speed={}, language={}').format(
                        opts['speed'], opts['language'])
                    rospy.loginfo(s)
            except rospy.ServiceException as e:
                rospy.logerr("Service did not process request: {}".format(e))

        # # Observe the activity.
        # try:
        #     resp = self.observe_activity()
        #     self.observe_activity(resp.current)
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service did not process request: {}".format(e))

        self.window.set_visible()
        self.window.move_window()

        rospy.loginfo('Robot cognition window is ready!')
        print()

    # Action taking ###########################################################

    def make_explanation_utterance(self, explanation, agent_cur_name):
        expl_s = ''
        state = self.cur_world.env.state

        best_count = len(explanation.best)
        other_count = len(explanation.others)
        if isinstance(explanation, BetterThanExplanation):
            # if random.randrange(1, 2+1) == 1:
            if decision(0.5):  # 50% chance : from explanation.
                if best_count > 1:  # multiple greedy optimal
                    opt_s = random.choice(['among the', *3*['one of the']])
                else:
                    opt_s = 'the'
                if other_count == 0:
                    oth_s = 'only one'
                else:
                    oth_s = 'best'

                base = agent_cur_name
                if state.step_no == 1:
                    base = 'there'
                options = [
                    'Because it is {} cheapest from {}!'.format(opt_s, base),
                    'Because it is {} {} from {}!'.format(opt_s, oth_s, base),
                    'Because from {}, it is {} {}!'.format(base, opt_s, oth_s),
                ]
                if other_count > 0:
                    options += [
                        'Because from {}, it is {} best option{}!'.format(
                            base, opt_s, 's' if best_count > 1 else ''),
                    ]

                expl_s = random.choice(options)
            else:   # 50% chance : to explanation.
                nodes = {u for a in explanation.others for u in a.edge}
                named_nodes = {state.network.get_node_name(u) for u in nodes}
                excludeds = {state.network.get_node_name(u)
                             for a in explanation.best for u in a.edge}
                named_others = named_nodes.difference(excludeds)
                if len(named_others) > 0:
                    expl_s = 'Because it is better than going to {}.'.format(
                        ' or '.join(named_others))
                else:
                    expl_s = 'Because there is nowhere else to go.'
        elif isinstance(explanation, ConnectedExplanation):
            base = agent_cur_name
            options = [
                'Because there is nowhere else to go from {}.'.format(base),
                'Because we can not go anwhere else from {}.'.format(base),
                'Because there is nowhere else to go.',
                'Because we can not go anywhere else.',
            ]
            expl_s = random.choice(options)

        if self.is_instructing and state.step_no == 1:
            expl_s = ''

        return expl_s

    def act(self):
        """TODO: docstring for act"""

        world = self.cur_world
        state = world.env.state

        # Plan for the next action as if solving alone.
        explanation = world.agent.planner.last_explanation
        planned_action = world.agent.planner.last_plan

        # Get the current node info.
        mental_state = world.agent.cur_state
        agent_cur_name = world.env.state.network.get_node_name(
            mental_state.cur_node)

        # is_contradictory = False

        # Generate an explanation.
        best_count = len(explanation.best)
        expl_s = self.make_explanation_utterance(explanation, agent_cur_name)

        # The robot has a plan to connect.
        if isinstance(planned_action, SuggestPickAction):
            u, v = world.env.state.network.get_edge_name(planned_action.edge)
            next_name = u if u != agent_cur_name else v

            # If there is no current suggestion: suggest something.
            if state.network.suggested_edge is None:
                if self.is_disagreeing:
                    self.is_disagreeing = False

                    edge_str = '{} to {}'.format(u, v)
                    options = [
                        'I think we should rather pick {}.'.format(
                            edge_str),
                        'I think we should go from {} instead.'.format(
                            edge_str),
                        "Shouldn't we go from {} instead?".format(
                            edge_str),
                        ('Rather than that,'
                         ' maybe we go from {}?').format(edge_str),
                        ('Instead of that,'
                         ' what about going from {}?').format(edge_str),
                    ]
                    s = random.choice(options)

                # If the very first suggestion.
                elif state.step_no == 1:
                    if self.is_instructing \
                            and self.cur_activity == 'collaboration-1':
                        s = ("Let's start from {}. Shall we go to {}? Now, it"
                             " is your turn. If you think we must").format(
                            agent_cur_name, next_name)
                        s += ("\nconnect them, agree with me by pressing the"
                              " check. Otherwise, disagree with the cross!")
                    else:
                        s = "Let's start from {}. Shall we go to {}?".format(
                            agent_cur_name, next_name)

                # For the later suggestions.
                else:
                    options = [
                        "Let's pick {} to {}.".format(u, v),
                        "Then, shall we go from {} to {}?".format(u, v),
                        "Umm, what about from {} to {}?".format(u, v),
                    ]
                    s = random.choice(options)

                # if decision(1.0):  # 100% chance: intentional comment.
                if decision(0.75):  # 75% chance: intentional comment.
                    if len(expl_s) > 0:
                        s += '\n' + expl_s

                # Enact suggesting.
                self.help_text = self.agreement_help_text
                self.express('point_self')
                self.home_head()
                rospy.sleep(0.5)
                self.say(s)
                self.express('point_human')
                self.emote('smile')
                # if random.randrange(1, 3+1) == 1:
                if decision(.33):  # 33% chance
                    self.express('head_scratch', is_blocking=True)
                else:
                    rospy.sleep(2)
                self.execute_action(planned_action)

            # There is a current suggestion by the human: Agree or disagree.
            else:
                suggested_edge = state.network.suggested_edge
                planned_edge = planned_action.edge

                agreable_edges = {planned_edge}
                agreable_edges.update({a.edge for a in explanation.best})

                is_agree = False
                for edge in agreable_edges:
                    if set(suggested_edge) == set(edge):
                        is_agree = True

                if is_agree:  # Will agree
                    options = [
                        "I agree with you!",
                        "I definitely agree!",
                        "I agree!",
                        "Okay, I think it is a good one too!",
                        "Definitely, that's a good choice!",
                        "Very well, let's connect them.",
                    ]
                    s = random.choice(options)

                    # if random.randrange(1, 3) == 1:  # 50% chance
                    if random.randrange(1, 2) == 1:  # 100% chance
                        if len(expl_s) > 0:
                            s += '\n' + expl_s

                    gesture = 'yes'
                    action = AgreeAction(agent=Agent.ROBOT)
                else:  # Will disagree
                    agent = self.cur_world.agent
                    prev_ms = agent.get_state(agent.state_no-1)
                    u, v = suggested_edge
                    # If I think so and you don't think so:
                    my_beliefs = prev_ms.beliefs['me']['world']
                    your_beliefs = prev_ms.beliefs['me']['you']['world']
                    if your_beliefs[u][v]['is_optimal'] == 1.0 \
                            and my_beliefs[u][v]['is_optimal'] == 0.0:
                        # Decide what to say.
                        options = [
                            "Okay, if you really want it so much.",
                            "Okay if you really want.",
                            "If you insist, okay.",
                            ("I see that you really want to connect them:"
                                " fine."),
                            ("What can I say,"
                             " if you want it so much! Okay."),
                        ]
                        s = random.choice(options)
                        # Decide which gesture to perform.
                        # gesture = 'bored'
                        # if random.randrange(1, 5+1) == 1:  # 20% chance
                        gesture = 'protect'
                        action = AgreeAction(agent=Agent.ROBOT)
                    else:
                        # To carry over to the next utterance.
                        self.is_disagreeing = True
                        # Decide what to say.
                        options = [
                            "I disagree with you!",
                            "I disagree.",
                            "I don't think so!",
                            "I don't think it is a good connection!",
                            "I think we shouldn't do that.",
                        ]
                        s = random.choice(options)
                        if best_count == 1:
                            options = [
                                "There is a better option.",
                                "",
                                "",
                            ]
                        else:
                            options = [
                                "There are better options.",
                                "There are better connections!",
                                "There are cheaper ones!",
                            ]
                        ss = random.choice(options)
                        if len(ss) > 0:
                            s += '\n' + ss
                        # Decide which gesture to perform.
                        gesture = 'no'
                        # Decide on the action.
                        action = DisagreeAction(agent=Agent.ROBOT)

                # Enact agreeing or disagreeing.
                # self.set_pause(True)
                self.express(gesture)
                rospy.sleep(0.5)
                self.home_head()
                self.say(s)
                if random.randrange(1, 5+1) == 1:  # 20% chance
                    self.express('rappel')
                elif random.randrange(1, 3) == 1:  # 40% chance overall
                    self.express('so')
                self.emote('smile')
                rospy.sleep(2)
                self.execute_action(action)
                # self.set_pause(False)

        elif isinstance(planned_action, AttemptSubmitAction):
            s = ("Let's submit! The system will check "
                 "if our connection is the cheapest possible.")

            # Enact submission.
            self.express('point_self')
            rospy.sleep(1)
            self.home_head()
            self.say(s)
            self.express('point_human')
            self.execute_action(AttemptSubmitAction(agent=Agent.ROBOT))
            rospy.sleep(1)

    # Process state transition observations ###################################

    def state_transition_callback(self, data):
        """Process state transition observations from a transition message."""
        log_heard(self.state_trans_sub, data)
        if self.cur_world is not None:
            activity = data.header.frame_id
            graph = self.cur_world.env.state.network.graph

            state = decode_state_message(data.state, graph)
            action = decode_action_message(data.action)
            next_state = decode_state_message(data.next_state, graph)

            # if not isinstance(action, SetPauseAction):
            self.state_transition_observation(
                activity, state, action, next_state)

    def state_transition_observation(
            self, activity, state, action, next_state):
        """Process state transition observations from the context."""
        if activity == 'tutorial':
            self.tutorial_state_observation(state, action, next_state)
        elif 'collaboration' in activity:
            self.collaboration_state_observation(
                activity, state, action, next_state)

    def tutorial_state_observation(self, state, action, next_state):
        """Process state transition observation for the tutorial."""

        # If first step: to draw.
        if state.step_no == 1 and next_state.step_no == 2:
            # Say the cost.
            self.set_pause(True)
            self.express('point_activity')
            s = (
                "This railway costs 3 francs!"
                " In this game we consider the costs of the tracks:"
                "\nevery time we build a track, it costs something."
                " We want to spend as little as we can.")
            self.say(s)
            self.emote('happy')
            self.express('happy', is_blocking=True, speed=0.5)
            delay = 2 if self.with_robot else 6
            rospy.sleep(delay)

            # Instruct to erase.
            self.help_text = ('Press the eraser button to'
                              ' remove all of the connections.')
            s = ('Now, imagine you want to remove the connections and'
                 ' start connecting again from \nscratch. For this, you'
                 ' can press the eraser button on the right. Try it!')
            self.say(s)
            delay = 0 if self.with_robot else 2
            rospy.sleep(delay)

            # Point to the screen.
            self.express('point_human', is_blocking=True)
            self.set_pause(False)

        # If second step: to erase.
        elif state.step_no == 2 and next_state.step_no == 3:
            # Explain no connection.
            self.set_pause(True)
            self.set_robot_text('')
            self.help_text = ('Connect the mountains and '
                              'submit your connection.')
            self.express('yes', is_blocking=True)
            s = ("Currently, you see there is no connection between"
                 " the mountains:\nminers can't go between them.")
            self.say(s, is_setting_text=False)
            delay = 0 if self.with_robot else 3
            rospy.sleep(delay)

            # Instruct to submit.
            s = ("Let's connect the mountains again and when you are done,"
                 "\nsuggest your connection by pressing submit button!")
            self.say(s)
            delay = 0 if self.with_robot else 3
            rospy.sleep(delay)
            self.express('point_human')
            self.set_pause(False)

        # If third and final step: to submit.
        if next_state.is_terminal:  # If done with the tutorial.
            # Explain that the human is done with this activity.
            self.express('happy')
            s = ('Perfect! You got a hold of it. We can finally begin!')
            self.say(s)

            # Set the activity to the next.
            rospy.sleep(3)
            self.set_activity('pretest-1')

    def update_current_world(self, activity):
        if activity == 'collaboration-1':
            self.cur_world = self.world
        elif activity == 'collaboration-2':
            self.cur_world = self.world2
        elif activity == 'tutorial':
            self.cur_world = self.world_tutorial

    def collaboration_state_observation(
            self, activity, state, action, next_state):
        """Process state transition for a collaboration activity."""
        self.help_text = None

        cur_node = self.cur_world.agent.cur_state.cur_node

        self.cur_world.act(action)

        # Update scene.
        self.window.cur_scene.state = self.cur_world.agent.cur_state

        # Logging/info.
        print()
        rospy.loginfo('Belief update (on event observation):')
        if isinstance(action, SuggestPickAction):
            u, v = state.network.get_edge_name(action.edge)
            verbose_action = action.__class__(edge=(u, v), agent=action.agent)
        else:
            verbose_action = action
        rospy.loginfo('Action: {}'.format(verbose_action))
        rospy.loginfo('Observation: {}'.format(Observation(next_state)))
        new_cur_node = self.cur_world.agent.cur_state.cur_node

        beliefs = self.cur_world.agent.cur_state.get_beliefs()
        if new_cur_node != cur_node:
            rospy.loginfo('I believed that we were at {} (Node {})'.format(
                state.network.get_node_name(cur_node), cur_node))
            rospy.loginfo('I now believe that we are at {} (Node {})'.format(
                next_state.network.get_node_name(new_cur_node), new_cur_node))
        belief = 'I believe that we are at {}.'.format(
            next_state.network.get_node_name(new_cur_node))
        beliefs.insert(0, belief)
        rospy.loginfo('Beliefs:\n{}'.format(pprint.pformat(beliefs)))
        print()

        if action.agent == Agent.HUMAN:
            self.human_action_observation(state, action, next_state)
        elif action.agent == Agent.ROBOT:
            self.robot_action_observation(state, action, next_state)

        self.window.observes_text = str(next_state)

        # Take an action if it is robot's turn.
        if Agent.ROBOT in next_state.agents:
            # print('#### robot will act')
            rospy.sleep(2)
            self.act()

    def human_action_observation(self, state, action, next_state):
        """TODO: Docstring for human_action_observation"""

        # React to suggestions.
        if isinstance(action, SuggestPickAction):
            u, v = next_state.network.suggested_edge
            u_name = self.cur_world.env.state.network.get_node_name(u)
            v_name = self.cur_world.env.state.network.get_node_name(v)
            edge_str = '{} to {}'.format(u_name, v_name)
            options = [
                "I see, you want to connect {}.".format(
                    edge_str),
                "Hmm, you would like to go from {}!".format(
                    edge_str),
                "Then, you think we should go from {}.".format(
                    edge_str),
            ]
            # 33% chance: verbalise human choice.
            if random.randrange(1, 3+1) == 1:
                s = random.choice(options)
            else:
                s = ''

            # Note if it is surprising.
            # Surprise.
            surp_s = ''
            world = self.cur_world
            agent = world.agent
            if agent.state_no > 1:
                prev_mental_state = agent.get_state(agent.state_no-1)
                u, v = action.edge
                beliefs = prev_mental_state.beliefs['me']['you']['world']
                p = beliefs[u][v]['is_optimal']
                if p == 0.0:
                    options = [
                        'Oh, what a surprise!',
                        'Oh, really!',
                        'Oh, really?',
                        'Surprising!',
                        'No way!',
                        'Really?',
                    ]
                    surp_s = random.choice(options)
                    # if p == 0.0:
                    options = [
                        " I thought you didn't think it was a good choice!",
                        " I thought you didn't think we should connect them!",
                        " I thought you didn't think we should do that!",
                    ]
                    surp_s += random.choice(options)

                # # Move as a "disagree" surprise earlier
                # elif p == 1.0:
                #     options = [
                #         " I thought you thought it was a good choice!",
                #         " I thought you thought we should connect them!",
                #         " I thought you thought we should do that!",
                #     ]
                #     surp_s += random.choice(options)

            self.set_pause(True)

            if len(surp_s) > 0:
                # if random.randrange(1, 2+1) == 1:  # 50% chance
                if random.randrange(1, 1+1) == 1:  # 100% chance
                    self.say(surp_s)
                    rospy.sleep(2)

            if random.randrange(1, 5+1) == 1:  # 20% chance
                gesture = 'rappel'
            else:   # 80% chance
                gesture = 'so'
            self.express(gesture)
            if len(s) > 0:
                self.say(s)
            self.home_head()
            self.set_pause(False)

        # React to yes.
        elif isinstance(action, AgreeAction):
            self.express('happy')
            if state.step_no == 1 and self.is_instructing \
                    and self.cur_world.name == 'collaboration-1':
                s = ("Great, you agree."
                     " Then, what should we pick next?"
                     " Go ahead and suggest the"
                     "\nconnection we should pick, by clicking on"
                     " a mine and dragging to another.")
            else:
                options = [
                    "Great to see that you agree.",
                    "Very well, you agree.",
                    "Good, we agree.",
                    "Good, you agree!",
                    "Great! You agree!",
                    "Great, you agree with me!",
                ]
                s = random.choice(options)
                options = [
                    "Then, you think it is the right connection.",
                    "Then, you think it is a correct choice.",
                    "So, you think it is the right one.",
                    "So, you think we should do that too.",
                ]
                if random.randrange(1, 4+1) == 1:  # 25% chance
                    s += ' ' + random.choice(options)
                options = [
                    "Now it is your turn: what should we do?",
                    "Now, you go: which ones should we connect?",
                    "Then, it is your turn: which mines shall we connect?",
                    "So, what should we do now?",
                ]
                s += '\n' + random.choice(options)

            self.set_pause(True)
            self.say(s)
            emotion = None
            if random.randrange(1, 5+1) == 1:  # 20% chance
                gesture = 'point_human'
                emotion = 'happy'
            elif random.randrange(1, 3) == 1:  # 40% chance overall
                gesture = 'shy'
                emotion = 'happy'
            elif random.randrange(1, 3) == 1:  # 10% chance overall
                gesture = 'afraid'
            elif random.randrange(1, 3) == 1:  # 5% chance overall
                gesture = 'yawn'
            else:  # 5% chance overall
                gesture = 'right_arm_up_down'
            self.emote(emotion)
            self.express(gesture, is_blocking=False)
            # rospy.sleep(1)
            self.set_pause(False)

        # React to no.
        elif isinstance(action, DisagreeAction):
            self.express('sad')
            # Decide what to say.
            options = [
                "Okay, you disagree.",
                "Oh, you don't think so.",
                "Okay, you don't think so.",
                "I see you do not agree.",
                "Okay, you do not agree.",
            ]
            s = random.choice(options)
            options = [
                "Then you don't think it is a good connection.",
                "Then you think it is a wrong one.",
                "",
            ]
            s += ' ' + random.choice(options)
            options = [
                "Then, what do you think we should do?",
                "Then, what shall we do?",
                "What would you like to do, then?",
                "Then, what is best to do now?",
                "So, what should we do now?",
            ]
            s += '\n' + random.choice(options)

            self.set_pause(True)
            self.say(s)
            delay = 0 if self.with_robot else 2  # 0.5
            rospy.sleep(delay)
            self.set_pause(False)

        # React to clearing.
        elif isinstance(action, ClearAction):
            self.set_pause(True)
            # Decide what to say.
            options = [
                ("Oh I see you want to start connecting again."
                 "\nOkay, please go ahead!"),
                "Okay, let's start again."]
            s = random.choice(options)
            self.say(s)
            # Wait briefly and unpause.
            # rospy.sleep(0.5)
            self.set_pause(False)

        # React to attempting to submit.
        elif isinstance(action, AttemptSubmitAction):
            self.set_pause(True)
            # Decide what to say.
            options = [
                ('I see that you think we are done,'
                 ' and you want to submit.'),
                ('Okay, then you think this is the cheapest possible,'
                 ' and you want to submit.'),
            ]
            s = random.choice(options)
            self.say(s)  # , is_setting_text=False)
            # Wait briefly and unpause.
            # rospy.sleep(0.5)
            self.set_pause(False)

        # React to submit.
        elif isinstance(action, SubmitAction):
            # If the activity is completed.
            if next_state.is_terminal:
                # Thank the human.
                self.set_pause(True)
                s = ('Perfect! Congratulations!\nWe connected '
                     'them while spending as little as possible!')
                self.express('hoora')
                self.say(s)

                # Set the next activity.
                if self.cur_world.name == 'collaboration-1':
                    rospy.sleep(3)
                    self.set_activity('collaboration-2')
                elif self.cur_world.name == 'collaboration-2':
                    rospy.sleep(3)
                    self.set_activity('posttest-1')
                self.set_pause(False)
            else:
                # Report the result/failure.
                self.express('sad')
                options = [
                    ('Oh, the system checked our connection: '
                     'it seems there is a better one!'),
                    ('Ah, the system checked our connection: '
                     'It seems we can do better!'),
                ]
                s = random.choice(options)
                self.say(s)

                # Motivate the human to try again.
                delay = 1 if self.with_robot else 4
                rospy.sleep(delay)
                self.express('happy')
                s = "Let's try again!"
                self.say(s)
                delay = 0 if self.with_robot else 2
                rospy.sleep(delay)
                self.set_pause(False)

    def robot_action_observation(self, state, action, next_state):
        """TODO: docstring"""
        if isinstance(action, SubmitAction):
            if next_state.is_terminal:
                self.set_pause(True)
                self.express('hoora')
                s = ('Perfect! Congratulations!'
                     '\nWe connected all of them while'
                     ' spending as little as possible!')
                self.say(s)
                delay = 0 if self.with_robot else 1
                rospy.sleep(delay)
                self.set_pause(False)
                self.set_activity('collaboration-2')
            else:
                self.set_pause(True)
                self.express('sad')
                options = [
                    'Oh, the system checked our connection: '
                    'it seems we can maybe spend less!',
                    'The system says we can do better!',
                ]
                s = random.choice(options)
                self.say(s)

                delay = 0.5 if self.with_robot else 4
                rospy.sleep(delay)

                s = "Let's try again!"
                self.express('happy')
                self.say(s)

                rospy.sleep(4)

                self.set_pause(False)

    # Process activity transition observations ################################

    def activity_transition_callback(self, data):
        """Process an ROS Observe an activity transition.

        Attributes:
            activity (str):
                name of the activity.
        """
        log_heard(self.activity_trans_sub, data)
        activity = data.next

        if data.next != data.current:
            rospy.loginfo('Activity transition from {} to {}'.format(
                data.current, data.next))
            self.is_instructing = True
            rospy.logwarn('Will instruct from this point on.')

        self.cur_activity = activity
        self.window.cur_activity_name = activity

        self.activity_observation(activity)

    def activity_observation(self, activity):
        """Observe an activity transition.

        Attributes:
            activity (str):
                name of the activity.
        """
        self.update_current_world(activity)

        if activity == 'cover':
            self.cover_observation()
        elif activity == 'welcome' and self.is_instructing:
            self.welcome_observation()
        elif activity == 'profile':
            self.profile_observation()
        elif activity == 'intro' and self.is_instructing:
            self.intro_observation()
        elif activity == 'tutorial' and self.is_instructing:
            self.tutorial_observation()
        elif 'pretest' in activity and self.is_instructing:
            self.pretest_observation(activity)
        elif 'collaboration' in activity:
            self.collaboration_observation(activity)
        elif 'posttest' in activity:
            self.posttest_observation(activity)
        elif 'debriefing' in activity:
            self.goodbye_observation()

    def cover_observation(self):
        """React to the start of the cover activity."""
        self.set_robot_text('')
        self.help_text = 'Hello!'

    def welcome_observation(self):
        """React to the start of the hello/start activity."""
        self.set_pause(True)
        self.emote('happy_blinking')
        self.express('hi', is_blocking=True, speed=1.4)
        s = ("Hello! My name is Q.T.!"  # " Nice to meet you."
             # , and here is my friend Utku." \\pau=500\\
             " I can't wait to play with you."
             "\nLet's start when you are ready!")
        # disp_s = ("Hello! My name is Q.T., and here is my friend Utku."
        #           # "Hello! My name is Q.T. I can't wait to play with you."
        #           "\nLet's start when you are ready!")  # \\pau=300\\
        self.say(s)
        self.express('point_activity')

        self.help_text = 'Click anywhere in the white area to begin.'

        rospy.sleep(1)

        self.set_pause(False)

    def profile_observation(self):
        """React to the start of the profile-filling activity.
        TODO: test"""
        self.express('hi')
        s = ('Could you write down your name,'
             ' year of birth and choose your gender?'
             "\n I can't wait to start playing with you!")
        self.say(s)

    def intro_observation(self):
        """React to the start of the introduction activity."""
        self.help_text = None
        self.set_pause(True)

        # Introduce the environment.
        self.express('point_activity')
        rospy.sleep(1)
        s = ('Look at this! This is a map of Switzerland.'
             ' You see many mountains,'
             ' and in all of\nthe mountains there is gold!'
             ' You also see buildings, but they are not important.')
        self.say(s)

        # Introduce the goal of spanningness.
        self.express('swipe_right', is_blocking=True)
        delay = 1 if self.with_robot else 5
        rospy.sleep(delay)
        s = ('In this game, our goal is to build a railway network'
             ' to\nhelp the miners go from any mine to any other.')
        self.say(s)

        # Introduce the goal of optimality.
        self.express('challenge', is_blocking=True)
        delay = 1 if self.with_robot else 5
        rospy.sleep(delay)
        s = ('For this goal, we should try to spend\n'
             'as little money as possible to build these railways!')
        self.say(s)
        self.express('clap', is_blocking=True, speed=1.5)
        delay = 2 if self.with_robot else 5
        rospy.sleep(delay)
        self.set_pause(False)

        # Initiate the next activity: the tutorial.
        self.set_activity('tutorial')

    def tutorial_observation(self):
        """React to the start of the tutorial activity."""
        self.help_text = None
        self.set_pause(True)

        # Introduce the mines and the tracks.
        self.express('point_activity')
        s = ("Before we play together, let's see how to interact"
             " with the game. You see here\ntwo gold mines, with "
             "a possible connection between them, shown as a gray track.")
        self.say(s, is_setting_text=False)
        delay = 1 if self.with_robot else 4
        rospy.sleep(delay)

        # Introduce the goal.
        self.express('swipe_left')
        s = ("Let's connect these two gold mines to each other,"
             " so that the miners can travel\nbetween them. For this, "
             "click on one mine, and go to the other without releasing!")
        self.say(s)
        delay = 0.5 if self.with_robot else 4
        rospy.sleep(delay)
        self.set_pause(False)

    def pretest_observation(self, activity):
        """React to the start to a pretest activity.

        Attributes:
            activity (str):
                name of the activity.
        """
        self.help_text = self.goal_help_text

        # First pretest activity.
        if '1' in activity:
            # Give the goal.
            self.set_pause(True)
            self.express('point_activity')
            s = ("Now, let’s start the game! Go ahead and connect "
                 "all the gold mines to each other\nso that miners can go"
                 " from any mine to any other mine by some path!")
            self.say(s)

            # Motivate and signal for the upcoming collaborative activity.
            delay = 1 if self.with_robot else 3
            rospy.sleep(delay)
            self.express(
                'point_human', is_blocking=True)
            s = ("Please give it a try by yourself, later, "
                 "we will do it together!")
            self.say(s)
            self.express('clap')
            self.set_pause(False)

        # Second pretest activity.
        elif '2' in activity:
            # Give the goal.
            self.set_pause(True)
            self.express('curious')
            s = ("Okay, so, let's try to connect these different "
                 "gold mines to help the miners go\nbetween all of them. "
                 "Try to spend as few francs as you can. Give it a try!")
            self.say(s)
            self.express('left_arm_up_pull_down')
            delay = 1 if self.with_robot else 3
            rospy.sleep(delay)
            self.set_pause(False)

        # Third pretest activity.
        elif '3' in activity:
            # Give the goal.
            self.set_pause(True)
            self.express('point_human')
            s = ("So you are done with the second one."
                 " What about these gold mines? Let's connect"
                 "\nthem as cheaply as possible, while miners can travel"
                 " between them. Go ahead!")
            self.say(s)
            self.express('challenge')
            delay = 1 if self.with_robot else 3
            rospy.sleep(delay)
            self.set_pause(False)

        # Fourth pretest activity.
        elif '4' in activity:
            # Give the goal.
            self.set_pause(True)
            self.express('hard_nod', is_blocking=True)
            s = ("Okay, let's try for this one now."
                 " Connect them while spending as little as possible,"
                 "\nmake sure miners can travel between all of them."
                 " Soon, we will do it together. Good luck!")
            self.say(s)
            self.express('curious')
            delay = 1 if self.with_robot else 3
            rospy.sleep(delay)
            self.set_pause(False)

        # Fifth pretest activity.
        elif '5' in activity:
            # Give the goal.
            self.set_pause(True)
            self.express('right_arm_up_down')
            s = ("So, for this last time, please connect these different"
                 " mines as\ncheaply as you can, by yourself. "
                 "Then, we will play together!")
            self.say(s)
            self.express('curious')
            delay = 1 if self.with_robot else 3
            rospy.sleep(delay)
            self.set_pause(False)

    def posttest_observation(self, activity):
        """React to the start to a posttest activity.

        Attributes:
            activity (str):
                name of the activity.
        """
        self.help_text = None

        # First posttest activity.
        if '1' in activity:
            self.express('point_activity')
            s = ('It was a great experience for me to work with you!'
                 ' Now, please connect all of\nthe mines to each other'
                 ' by some path, and spend as little as possible, '
                 'by yourself.')
            self.say(s)

        # Second posttest activity.
        elif '2' in activity:
            self.express('point_human')
            s = ("So, let's try this one. Make sure miners can go from"
                 " any mine to any other, through\nthe tracks. And, try"
                 " to spend as few francs as you can!")
            self.say(s)

        # Third posttest activity.
        elif '3' in activity:
            self.express('curious')
            s = ("Okay, now, another one. Miners should be able to go"
                 "between all\nthe mines. And, please try to spend as"
                 " few francs as you can!")
            self.say(s)

        # Fourth posttest activity.
        elif '4' in activity:
            self.express('curious')
            s = ("Now, what about these gold mines? "
                 "Be sure that miners can go from any mine\nto any other."
                 " Also, try to spend as little as possible."
                 " Give it a try!")
            self.say(s)

        # Fifth posttest activity.
        elif '5' in activity:
            self.express('curious')
            s = ("So, I have a final one for you. Miners need to "
                 "be able to go between all the mines:\ntry to find the"
                 " cheapest such connection. Let me see how you do it!")
            self.say(s)

    def collaboration_observation(self, activity):
        """React to the start to a collaboration activity.

        Attributes:
            activity (str):
                name of the activity.
        """
        if activity == 'collaboration-1':
            if self.is_instructing:
                self.help_text = None

                # Repeat the goal.
                self.set_pause(True)
                self.express('point_activity')
                s = ("Now, we do it together! We need to connect the mines,"
                     " so that miners can travel from\nany mine to any other."
                     " Our goal is to spend as few francs as possible to build"
                     " tracks!")
                self.say(s)
                self.express('challenge', is_blocking=True)

                # Give the contraints.
                delay = 0.5 if self.with_robot else 3
                rospy.sleep(delay)
                self.set_robot_text('')
                s = ("We have got only four attempts to find the cheapest"
                     " connection, so let's try our best!\nIf you think "
                     "we achieved our goal, press submit: the system will "
                     "give feedback.")
                self.say(s)
                self.express('arms_open_close', is_blocking=True)
                delay = 0.5 if self.with_robot else 3
                rospy.sleep(delay)
                self.set_pause(False)

        elif activity == 'collaboration-2':
            # Repeat the goal.
            self.help_text = None
            self.express('point_activity')
            self.set_pause(True)
            s = ("Now we play again, this time for different connections!"
                 "\nOur goal is the same: connecting them while spending"
                 " as little as we can.")
            self.say(s)

            # ...
            delay = 1 if self.with_robot else 5
            rospy.sleep(delay)
            s = "Let me start."
            self.say(s)

            self.set_pause(False)

        # Take an action.
        resp = self.observe_state()
        # print('#########', resp)
        graph = self.cur_world.env.state.network.graph
        state = decode_state_message(resp.state, graph)
        if Agent.ROBOT in state.agents:
            self.act()

    def goodbye_observation(self):
        """React to the start of the goodbye activity."""
        self.help_text = None

        s = ('Its time to say goodbye....'
             '\nI loved playing with you, I hope you enjoyed it too! '
             ' Goodbye!')
        self.say(s)

        delay = 1 if self.with_robot else 4
        rospy.sleep(delay)

        s = ('Oh, before you go, could you please answer some questions'
             '\nthat my friend Utku would like to ask you?'
             ' Thank you very much!')
        self.say(s)

        self.express('wave')
        self.emote('sad')

        self.express('send_kiss')
        self.emote('kiss')

    # Generate robot speech, gestures and emotions #########################

    def say(self, speech_text, display_text=None, is_setting_text=True):
        """TODO: docstring"""
        self.set_robot_text(ROBOT_SPEAKING)

        if display_text is None:
            display_text = speech_text
        speech_text = str(speech_text).replace('\n', ' ')

        rospy.loginfo("Robot says: {}".format(speech_text))
        self.log_say_pub.publish(speech_text)
        self.window.says_text = display_text

        self.window.update()

        if self.with_robot and speech_text is not None:
            log_service_call(self.call_say, data=speech_text)
            try:
                resp = self.call_say(speech_text)
            except Exception as e:
                rospy.logerr(e)
                resp = False
            rospy.logdebug('Response: {}'.format(resp))
        else:
            rospy.sleep(1)
            resp = False

        if is_setting_text:
            self.last_speech_text = display_text
            self.set_robot_text(display_text)

        return resp

    def express(self, gesture, is_blocking=False, speed=1):
        """TODO: docstring for express"""
        rospy.loginfo('Robot expresses: "{}"{}{}'.format(
            gesture, ' (speed: {}{})' if speed != 1 else '',
            '' if is_blocking else ' (non-blocking)'))

        resp = True
        if gesture is not None:
            self.log_express_pub.publish(str(gesture))
            if not is_blocking:
                self.express_pub.publish(gesture)
                log_publish(self.express_pub, gesture)
            else:
                if self.with_robot:
                    log_service_call(self.call_express, data=gesture)
                    try:
                        resp = self.call_express(name=gesture, speed=speed)
                    except Exception as e:
                        rospy.logerr(e)
                        resp = False
                    rospy.logdebug('Response: {}'.format(resp))
                else:
                    resp = False

        return resp

    def emote(self, emotion, is_blocking=False):
        """TODO: docstring for emote"""
        rospy.loginfo('Robot emotes: "{}"{}'.format(
            emotion, '' if is_blocking else ' (non-blocking)'))
        self.log_emote_pub.publish(str(emotion))

        resp = True
        if emotion is not None:
            if not is_blocking:
                self.emote_pub.publish(emotion)
                log_publish(self.emote_pub, emotion)
            else:
                log_service_call(self.call_emote, data=emotion)
                try:
                    resp = self.call_emote(name=emotion, speed=1)
                except Exception as e:
                    rospy.logerr(e)
                    resp = False
                rospy.logdebug('Response: {}'.format(resp))
        else:
            resp = False

        return resp

    def home_head(self):
        """TODO: docstring"""
        if self.with_robot:
            data = ['HeadPitch']
            log_service_call(self.call_home, data)
            self.log_emote_pub.publish('home_head_via_service')
            try:
                self.call_home(data)
            except Exception as e:
                rospy.logerr(e)

        rospy.loginfo('Robot homes head.')

    # Communicate with the application ########################################

    # Make service calls to the application

    def execute_action(self, action):
        # Does.
        if hasattr(action, 'edge') and action.edge is not None:
            u, v = self.cur_world.env.state.network.get_edge_name(
                action.edge)
            action_text = str(action.__class__(
                edge=(u, v), agent=action.agent))
        else:
            action_text = str(action)
        self.window.does_text = action_text

        # Update the window.
        self.window.update()

        # Execute.
        data = make_action_message(action)
        log_service_call(self.call_act, data=data)
        self.log_act_pub.publish(data)
        try:
            resp = self.call_act(data)
        except Exception as e:
            rospy.logerr(e)
            resp = False
        rospy.logdebug('Response: {}'.format(resp))
        return resp

    def set_robot_text(self, text):
        """TODO: docstring"""
        self.log_set_robot_text_pub.publish(text)
        log_service_call(self.call_set_robot_text, data=text)
        try:
            resp = self.call_set_robot_text(text)
        except Exception as e:
            rospy.logerr(e)
            resp = False
        rospy.logdebug('Response: {}'.format(resp))
        return resp

    def observe_activity(self):
        log_service_call(self.call_observe_activity)
        try:
            resp = self.call_observe_activity()
            self.cur_activity = resp.current
            self.log_observe_activity_pub.publish(resp.current)
        except Exception as e:
            rospy.logerr(e)
            resp = False
        rospy.logdebug('Response: {}'.format(resp))
        return resp

    def set_activity(self, name):
        self.log_set_activity_pub.publish(name)
        log_service_call(self.call_set_activity)
        try:
            resp = self.call_set_activity(name)
        except Exception as e:
            rospy.logerr(e)
            resp = False
        rospy.logdebug('Response: {}'.format(resp))
        return resp

    def observe_state(self):
        log_service_call(self.call_observe_state)
        try:
            resp = self.call_observe_state()
            if hasattr(self, 'log_observe_state_pub'):
                self.log_observe_state_pub.publish(resp.state)
        except Exception as e:
            rospy.logerr(e)
            resp = False
        rospy.logdebug('Response: {}'.format(resp))
        return resp

    def set_pause(self, is_paused):
        """Pause or unpause the application."""
        self.log_pause_pub.publish(is_paused)
        try:
            resp = self.call_set_pause(is_paused)
        except Exception as e:
            rospy.logerr(e)
            resp = False
            # rospy.logerr('Service call to {} failed {}'.format(
            #     self.log_pause_pub.resolved_name, e))
        rospy.logdebug('Response: {}'.format(resp))
        return resp
    # Respond calls from the application.

    def reply_repeat_speech_request(self, data):
        if self.last_speech_text is None:
            return False
        else:
            self.say(self.last_speech_text)
            log_service_response(self.repeat_speech_service, data, True)
            return True

    def reply_help_request(self, data):
        if self.help_text is None:
            resp = False
            log_service_response(self.help_service, data, resp)
        else:
            self.set_pause(True)
            self.say(self.help_text)
            self.set_pause(False)
            resp = True
        log_service_response(self.help_service, data, resp)
        return resp

    # ROS initialisation ######################################################

    def init_ros(self, timeout=2):
        robot_node_name = '/agent/embodiment'
        # Activate with robot if in 'auto' mode and the service is available.
        name = '{}/say'.format(robot_node_name)
        if self.with_robot == 'auto':
            s = 'Robot mode: auto.'
            s += ' Checking if the robot {} is available...'.format(
                robot_node_name)
            rospy.loginfo(s)
            s = 'Waiting for {} service...'.format(name)
            if timeout is not None:
                s += ' (timeout in {}s)'.format(timeout)
            rospy.loginfo(s)
            try:
                rospy.wait_for_service(name, timeout=timeout)
                s = ('Run WITH a physical robot'
                     ' ({} service is available).').format(name)
                rospy.logwarn(s)
                self.with_robot = True
            except Exception as e:
                rospy.loginfo(str(e).title())
                s = ('Run WITHOUT a physical robot'
                     ' ({} service not available).').format(name)
                rospy.logwarn(s)
                self.with_robot = False
        elif self.with_robot:
            rospy.logwarn('Run WITH a physical robot (Robot mode is True).')
            rospy.loginfo('Waiting for {} service...'.format(name))
            rospy.wait_for_service(name)
        elif not self.with_robot:
            rospy.logwarn(
                'Run WITHOUT a physical robot (Robot mode is False).')
        else:
            rospy.logerr('Unknown robot mode {}.'.format(self.with_robot))

        # Initialise the services to offer.
        self.repeat_speech_service = rospy.Service(
            'cognition/repeat_speech', justhink_msgs.srv.RepeatSpeech,
            self.reply_repeat_speech_request)
        self.help_service = rospy.Service(
            'cognition/request_help', justhink_msgs.srv.RequestHelp,
            self.reply_help_request)

        # Initialise the services to request from a robot.
        if self.with_robot:
            self.call_say = make_service_proxy(
                '/agent/embodiment/say',
                justhink_msgs.srv.Say)
            self.call_express = make_service_proxy(
                '/agent/embodiment/express',
                justhink_msgs.srv.Express)
            self.call_emote = make_service_proxy(
                '/agent/embodiment/emote',
                justhink_msgs.srv.Emote)
            self.call_home = make_service_proxy(
                '/agent/embodiment/home',
                justhink_msgs.srv.Home)
            self.call_configure_speech = make_service_proxy(
                '/agent/embodiment/configure_speech',
                justhink_msgs.srv.ConfigureSpeech)
            self.call_stop_say = make_service_proxy(
                '/agent/embodiment/stop_say',
                justhink_msgs.srv.StopSay)
            self.call_stop_emote = make_service_proxy(
                '/agent/embodiment/stop_emote',
                justhink_msgs.srv.StopEmote)
            self.call_stop_express = make_service_proxy(
                '/agent/embodiment/stop_express',
                justhink_msgs.srv.StopExpress)

            rospy.on_shutdown(self.call_stop_say)
            rospy.on_shutdown(self.call_stop_emote)
            rospy.on_shutdown(self.call_stop_express)

        # Initialise the publishers for a robot.
        opts = {'data_class': std_msgs.msg.String, 'queue_size': 10}
        self.say_pub = rospy.Publisher('/agent/embodiment/say', **opts)
        self.express_pub = rospy.Publisher('/agent/embodiment/express', **opts)
        self.emote_pub = rospy.Publisher('/agent/embodiment/emote', **opts)

        # Initialise the services to request from the scenario.
        print()
        rospy.loginfo('Initialising connection with human application node...')
        self.call_act = make_service_proxy(
            '/env/situation/act',
            justhink_msgs.srv.Act)
        self.call_observe_activity = make_service_proxy(
            '/env/situation/observe_activity',
            justhink_msgs.srv.ObserveActivity)
        self.call_set_pause = make_service_proxy(
            '/env/situation/pause',
            justhink_msgs.srv.Pause)
        self.call_observe_state = make_service_proxy(
            '/env/situation/observe_state',
            justhink_msgs.srv.ObserveState)
        self.call_set_activity = make_service_proxy(
            '/env/situation/set_activity',
            justhink_msgs.srv.SetActivity)
        self.call_set_robot_text = make_service_proxy(
            '/env/situation/set_robot_text',
            justhink_msgs.srv.SetRobotText)

        # Initialise the publishers for the app.
        self.act_pub = rospy.Publisher(
            '/env/situation/act', justhink_msgs.msg.Action, queue_size=10)

        # Initialise the subscribers.
        self.activity_trans_sub = rospy.Subscriber(
            '/env/situation/outset',
            justhink_msgs.msg.ActivityTransition,
            self.activity_transition_callback)
        self.state_trans_sub = rospy.Subscriber(
            '/env/situation/event',
            justhink_msgs.msg.StateTransition,
            self.state_transition_callback)

        # Initialise the publishers for logging.
        opts = {'data_class': std_msgs.msg.String, 'queue_size': 10}
        self.log_say_pub = rospy.Publisher(
            'cognition/log_say', **opts)
        self.log_express_pub = rospy.Publisher(
            'cognition/log_express', **opts)
        self.log_emote_pub = rospy.Publisher(
            'cognition/log_emote', **opts)
        self.log_set_robot_text_pub = rospy.Publisher(
            'cognition/log_set_robot_text', **opts)
        self.log_set_activity_pub = rospy.Publisher(
            'cognition/log_set_activity', **opts)
        self.log_observe_activity_pub = rospy.Publisher(
            'cognition/log_observe_activity', **opts)
        opts = {'data_class':  justhink_msgs.msg.EnvState, 'queue_size': 10}
        self.log_observe_state_pub = rospy.Publisher(
            'cognition/log_observe_state', **opts)
        opts = {'data_class':  std_msgs.msg.Bool, 'queue_size': 10}
        self.log_pause_pub = rospy.Publisher(
            'cognition/log_pause', **opts)
        opts = {'data_class': justhink_msgs.msg.Action, 'queue_size': 10}
        self.log_act_pub = rospy.Publisher(
            'cognition/log_act', **opts)

        rospy.loginfo('Robot cognition node is ready!')
        print()

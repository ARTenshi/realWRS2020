#!/usr/bin/env python

import rospy
import smach
import traceback
import time

from common import speech


class AskName(smach.State):
    def __init__(self, name_candidates, say_fn=None,
                 ask_msg="Excuse me. Could I have your name?",
                 ask_again_msg="Could you tell me your name again?",
                 confirm_msg="Your name is {}?",
                 confirm_again_msg="Please say yes or no.",
                 success_msg="Thank you.",
                 timeout=30.,
                 sphinx=None):
        smach.State.__init__(self, input_keys=['person_name'],
                                   outcomes=['success', 'timeout', 'failure'],
                                   output_keys=['person_name'])
        self.ask_msg = ask_msg
        self.confirm_msg = confirm_msg
        self.ask_again_msg = ask_again_msg
        self.confirm_again_msg = confirm_again_msg
        self.success_msg = success_msg
        self.say_fn = say_fn if say_fn else speech.DefaultTTS().say
        self.name_candidates = map(lambda x: x.lower(), name_candidates)
        self.timeout = timeout
        self.sphinx = sphinx
        if sphinx is None:
            from pocketsphinx_jsgf import PocketSphinxClient
            self.sphinx = PocketSphinxClient()

    def execute(self, userdata):
        try:
            msg_dict = dict(userdata)
            history = {}
            name_validity = False
            loop_count = 0
            while not rospy.is_shutdown():
                if len(self.name_candidates) > 0:
                    self.sphinx.set_single_rule(self.name_candidates)
                if loop_count <= 0:
                    self.say_fn(self.ask_msg.format(**msg_dict))
                else:
                    self.say_fn(self.ask_again_msg.format(msg_dict))
                rospy.loginfo('Name candidates: {}'.format(self.name_candidates))
                time.sleep(1)
                try:
                    name = self.sphinx.next_speech(timeout=self.timeout)
                except:
                    return 'timeout'
                rospy.loginfo('Name: {}'.format(name))
                if len(self.name_candidates) > 0:
                    if name in self.name_candidates:
                        name_validity = True
                else:
                    name_validity = True
                if name_validity is True:
                    if name in history.keys():
                        history[name] += 1
                    else:
                        history[name] = 1
                    userdata.person_name = name
                    msg_dict['person_name'] = name
                    for k, v in history.items():
                        if v >= 3:
                            return 'success'

                    self.sphinx.set_yes_or_no()
                    self.say_fn(self.confirm_msg.format(name))
                    time.sleep(1)
                    try:
                        confirm = self.sphinx.next_speech(timeout=self.timeout)
                    except:
                        return 'timeout'
                    while confirm not in ['yes', 'no']:
                        self.sphinx.set_yes_or_no()
                        self.say_fn(self.confirm_again_msg.format(msg_dict))
                        time.sleep(1)
                        try:
                            confirm = self.sphinx.next_yes_or_no(
                                timeout=self.timeout)
                        except:
                            return 'timeout'
                    if confirm == 'yes':
                        self.say_fn(self.success_msg.format(msg_dict))
                        return 'success'
                    else:
                        self.say_fn('I\'m sorry.')
                        del history[name]
                loop_count += 1
            rospy.logerr("Failed to get the person's name.")
            return 'failure'
        except:
            rospy.logerr(traceback.format_exc())
            return 'failure'

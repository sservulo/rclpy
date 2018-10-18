# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from transitions import Machine


class Action:

    states = ['Accepted', 'Executing', 'Canceling', 'Succeeded', 'Aborted', 'Canceled']

    def __init__(self, id):
        self.id = id
        self.result = None

        # Initialize the state machine
        self.machine = Machine(model=self, states=Action.states, initial='Accepted')

        # Define transitions
        # From Accepted
        self.machine.add_transition(trigger='execute', source='Accepted', dest='Executing')
        self.machine.add_transition(trigger='cancel_goal', source='Accepted', dest='Canceling')
        # From Executing
        self.machine.add_transition(trigger='set_succeeded', source='Executing', dest='Succeeded')
        self.machine.add_transition(trigger='set_aborted', source='Executing', dest='Aborted')
        self.machine.add_transition(trigger='cancel_goal', source='Executing', dest='Canceling')
        # From Canceling
        self.machine.add_transition(trigger='set_succeeded', source='Canceling', dest='Succeeded')
        self.machine.add_transition(trigger='set_aborted', source='Canceling', dest='Aborted')
        self.machine.add_transition(trigger='set_canceled', source='Canceling', dest='Canceled')


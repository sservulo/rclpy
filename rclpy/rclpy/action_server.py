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

from example_interfaces.srv import CancelFibonacci
from example_interfaces.srv import SendFibonacci
from example_interfaces.srv import ResultFibonacci

import rclpy
from rclpy.action import Action
from rclpy.node import Node


import threading
import uuid


class ActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')

        self.actions = {}
        self.threads = {}
        self.lock = threading.Lock()

        self.accept_service = self.create_service(SendFibonacci,
                                                  'send_fibonacci',
                                                  self.accept_action)
        
        self.cancel_service = self.create_service(CancelFibonacci,
                                                  'cancel_fibonacci',
                                                  self.cancel_action)

        self.result_service = self.create_service(ResultFibonacci,
                                                  'result_fibonacci',
                                                  self.get_result)

        self.get_logger().info("Fibonacci action server initialized.")

    def accept_action(self, request, response):
        with self.lock:
            action = Action(request.action_id)
            self.actions[request.action_id] = action
            
            thread = threading.Thread(target = self.execute, args = (action, request.order,))
            self.threads[request.action_id] = thread
            
            self.get_logger().info("Received action with id %s." % request.action_id)
            thread.start()

        response.accepted = True
        return response

    def cancel_action(self, request, response):
        action_id = request.action_id
        #TODO: race condition when canceling already executing action
        if action_id in self.actions and self.actions[action_id].state in ['Accepted', 'Executing']:
            self.actions[action_id].cancel_goal()
            response.canceled = True
        else:
            response.canceled = False

        return response

    def get_result(self, request, response):
        action_id = request.action_id
        if action_id in self.actions and self.actions[action_id].state is 'Succeeded':
            response.result = self.actions[action_id].result
        return response

    def feedback(self):

        return 0

    def execute(self, action, order):
        if action.state is 'Accepted':
            action.execute()

            #TODO: improve error handling
            number = self.fibonacci_sequence(order)
            action.result = number
            
            action.set_succeeded()

    def fibonacci_sequence(self, order):
        n = 1
        n_1 = 1
        for i in range(1, order):
            next = n + n_1
            n = n_1
            n_1 = next
        return n_1

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
from rclpy.action_server import ActionServer
from rclpy.node import Node

import uuid

class ActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        
        self.send = self.create_client(SendFibonacci, 'send_fibonacci')
        self.cancel = self.create_client(CancelFibonacci, 'cancel_fibonacci')
        self.result = self.create_client(ResultFibonacci, 'result_fibonacci')

        self.send_future = None
        self.cancel_future = None
        self.result_future = None

        self.action_id = None

        self.get_logger().info("Fibonacci action client initialized.")

    def send_action(self):
        while not self.send.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        req = SendFibonacci.Request()
        req.action_id = 'test' #TODO: use str(uuid.uuid4())
        req.order = 10

        self.action_id = req.action_id

        self.send_future = self.send.call_async(req)

        # opt 1
        rclpy.spin_until_future_complete(self, self.send_future)

        if self.send_future.result() is not None:
            self.get_logger().info('Action sent.')
        else:
            self.get_logger().info('Service call failed %r.' % (self.send_future.exception(),))

        ## opt 2
        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #     if self.send_future.done():
        #         if self.send_future.result() is not None:
        #             self.get_logger().info('Action sent.')
        #         else:
        #             self.get_logger().info('Service call failed %r.' % (future.exception(),))
        #         break

    def cancel_action(self, action_id):
        while not self.cancel.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        req = CancelFibonacci.Request()
        req.action_id = action_id

        self.action_id = req.action_id

        self.cancel_future = self.cancel.call_async(req)

        # opt 1
        rclpy.spin_until_future_complete(self, self.cancel_future)

        if self.cancel_future.result() is not None:
            response = self.cancel_future.result()
            if(response.canceled):
                self.get_logger().info('Action canceled.')
            else:
                self.get_logger().info('Action could not be canceled.')
        else:
            self.get_logger().info('Service call failed %r.' % (self.cancel_future.exception(),))


    def get_result(self, action_id):
        while not self.result.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        req = ResultFibonacci.Request()
        req.action_id = action_id

        self.action_id = req.action_id

        self.result_future = self.result.call_async(req)

        # opt 1
        rclpy.spin_until_future_complete(self, self.result_future)

        if self.result_future.result() is not None:
            response = self.result_future.result()
            self.get_logger().info('Fibonacci sequence last number: %d' % response.result)
        else:
            self.get_logger().info('Service call failed %r.' % (self.result_future.exception(),))

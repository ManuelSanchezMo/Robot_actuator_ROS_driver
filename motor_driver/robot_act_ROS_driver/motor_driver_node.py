# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from typing import Optional

import rclpy

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
import motor_driver 
import std_msgs.msg
from motor_interfaces.srv import Config1,Config2,Config3,SendTransition
from motor_interfaces.msg import OutElec, OutMec, SendComm


class LifecycleMotorNode(Node):
    """Our lifecycle talker node."""

    def __init__(self, node_name, **kwargs):
        """Construct the node."""
        self._count: int = 0
        self._pub: Optional[Publisher] = None
        self._timer: Optional[Timer] = None
        self._motor = motor_driver.motor_driver(cobid = 10, channel='vcan0')
        self._motor_ns = '/motor_0/'
        #self._publisher_mec_out = self.create_publisher(OutElec, self._motor_ns + 'elec_out', 10)
        #self._publisher_elec_out = self.create_publisher(OutElec,  self._motor_ns + 'mec_out', 10)

        super().__init__(node_name, **kwargs)


    def config_1_callback(self, request, response):
        print('config1')
        response = self._motor.send_config_1(P_controller=request.p_control, 
                                  I_controller=request.I_control, D_controller=request.D_control)
        return response
    
    def config_2_callback(self, request, response):
        print('config2')
        response = self._motor.send_config_2(P_vel=request.p_vel,I_vel=request.i_vel,
                                             D_vel=request.d_vel, Vel_lim=request.vel_lim)
        return response
    
    def config_3_callback(self, request, response):
        print('config2')
        response = self._motor.send_config_3(Volt_lim=request.volt_lim, V_aling=request.v_aling,
                                             Calibrate=request.calibrate, Zero_angle_elec=request.zero_angle_elec)
        return response   
    
    def send_transition_callback(self, request, response):
        response = self._motor.send_transition(transition = response.transition)
        return response 
      
    def command_callback(self, msg):
        print('comm')

    def publish_motor_ou(self):
        """Publish a new message when enabled."""
        msg = std_msgs.msg.String()
        msg.data = 'Lifecycle HelloWorld #' + str(self._count)
        self._count += 1

        # Print the current state for demo purposes
        if self._pub is None or not self._pub.is_activated:
            self.get_logger().info('Lifecycle publisher is inactive. Messages are not published.')
        else:
            self.get_logger().info(f'Lifecycle publisher is active. Publishing: [{msg.data}]')

        # We independently from the current state call publish on the lifecycle
        # publisher.
        # Only if the publisher is in an active state, the message transfer is
        # enabled and the message actually published.
        if self._pub is not None and self._pub.is_activated:
            msg_out_mec = OutMec()
            msg_out_mec.shaft_angle = self._motor.motor_mec_out['shaft_angle']
            msg_out_mec.shaft_angle_sp = self._motor.motor_mec_out['shaft_angle_sp']
            msg_out_mec.shaft_velocity = self._motor.motor_mec_out['shaft_velocity']
            self._publisher_mec_out.publish(msg_out_mec)
            msg_out_elec = OutElec()
            msg_out_elec.current = self._motor.motor_elec_out['current']
            msg_out_elec.electrical_angle = self._motor.motor_elec_out['electrical_angle']
            msg_out_elec.ua = self._motor.motor_elec_out['Ua']
            msg_out_elec.ub = self._motor.motor_elec_out['Ub']
            self._publisher_elec_out.publish(msg_out_elec)


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.

        :return: The state machine either invokes a transition to the "inactive" state or stays
            in "unconfigured" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "inactive".
            TransitionCallbackReturn.FAILURE transitions to "unconfigured".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self._pub = self.create_lifecycle_publisher(std_msgs.msg.String, 'lifecycle_chatter', 10)
        self._publisher_mec_out = self.create_lifecycle_publisher(OutMec, 'elec_out', 10)
        self._publisher_elec_out = self.create_lifecycle_publisher(OutElec,   'mec_out', 10)
        self._timer = self.create_timer(1.0, self.publish_motor_ou)
        self._config1_srv = self.create_service(Config1, 'Motor_config_1', self.config_1_callback)
        self._config2_srv = self.create_service(Config2, 'Motor_config_2', self.config_2_callback)
        self._config3_srv = self.create_service(Config3, 'Motor_config_3', self.config_3_callback)
        self._send_transition_srv = self.create_service(SendTransition, 'send_transition', self.send_transition_callback)
        self._commad_suscriber = self.create_subscription(SendComm, "command_angle", self.command_callback, 10)
        
        self.get_logger().info('on_configure() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the
        # inactive and enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks,
        # and we don't need to write on_activate()/on_deactivate() callbacks.

        # Log, only for demo purposes
        self.get_logger().info('on_activate() is called.')

        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info('on_deactivate() is called.')
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested.

        on_cleanup callback is being called when the lifecycle node
        enters the "cleaning up" state.

        :return: The state machine either invokes a transition to the "unconfigured" state or stays
            in "inactive" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
            TransitionCallbackReturn.FAILURE transitions to "inactive".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.

        :return: The state machine either invokes a transition to the "finalized" state or stays
            in the current state depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
            TransitionCallbackReturn.FAILURE transitions to "inactive".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS


# A lifecycle node has the same node API
# as a regular node. This means we can spawn a
# node, give it a name and add it to the executor.

def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecycleMotorNode('motor_driver')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()

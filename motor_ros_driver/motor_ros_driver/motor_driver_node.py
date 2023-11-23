#!/usr/bin/env python3
import rclpy
import os
import logging

from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.timer import Timer
from ament_index_python.packages import get_package_share_directory
from motor_interfaces.srv import Config1, Config2, Config3, SendTransition
from motor_interfaces.msg import OutElec, OutMec, SendComm
from .motor_driver import MotorDriver

class LifecycleMotorNode(Node):
    """ROS2 Lifecycle Node for motor control."""

    def __init__(self, node_name, **kwargs):
        """Construct the node."""
        super().__init__(node_name, **kwargs)
        share_directory = get_package_share_directory('motor_ros_driver')
        self._can_parser_dbc_param = self.declare_parameter('can_parser_dbc', os.path.join(share_directory, 'can_parser.dbc'))
        self._timer_duration_param = self.declare_parameter('timer_duration', 0.05)

        # Initialize parameters with default values
        self._channel_param = self.declare_parameter('channel', rclpy.Parameter.Type.STRING)
        self._node_id_param = self.declare_parameter('node_id', rclpy.Parameter.Type.INTEGER)
        self._motor_namespace = self.get_namespace()

        self._motor = None
        self._publisher_mec_out = None
        self._publisher_elec_out = None
        self._timer = None
        self._config1_srv = None
        self._config2_srv = None
        self._config3_srv = None
        self._send_transition_srv = None
        self._command_subscriber = None

        # Logger setup
        self._logger = self.get_logger()

    def _create_publishers(self):
        """Create ROS2 publishers for motor data."""
        self._publisher_mec_out = self.create_lifecycle_publisher(OutMec, f'{self._motor_namespace}/elec_out', 10)
        self._publisher_elec_out = self.create_lifecycle_publisher(OutElec, f'{self._motor_namespace}/mec_out', 10)

    def _create_services(self):
        """Create ROS2 services for motor configuration and transitions."""
        self._config1_srv = self.create_service(Config1, f'{self._motor_namespace}/motor_config_1', self.config_1_callback)
        self._config2_srv = self.create_service(Config2, f'{self._motor_namespace}/motor_config_2', self.config_2_callback)
        self._config3_srv = self.create_service(Config3, f'{self._motor_namespace}/motor_config_3', self.config_3_callback)
        self._send_transition_srv = self.create_service(SendTransition, f'{self._motor_namespace}/send_transition', self.send_transition_callback)

    def _create_subscriber(self):
        """Create ROS2 subscriber for motor commands."""
        self._command_subscriber = self.create_subscription(SendComm, f'{self._motor_namespace}/command_angle', self.command_callback, 10)

    def initialize_motor_components(self):
        """Initialize motor-related ROS2 components."""
        self._create_publishers()
        self._create_services()
        self._create_subscriber()

    def config_1_callback(self, request, response):
        """Callback for motor configuration service (Config1)."""
        response = self._motor.send_config_1(P_controller=request.p_control,
                                             I_controller=request.I_control, D_controller=request.D_control)
        return response

    def config_2_callback(self, request, response):
        """Callback for motor configuration service (Config2)."""
        response = self._motor.send_config_2(P_vel=request.p_vel, I_vel=request.i_vel,
                                             D_vel=request.d_vel, Vel_lim=request.vel_lim)
        return response

    def config_3_callback(self, request, response):
        """Callback for motor configuration service (Config3)."""
        response = self._motor.send_config_3(Volt_lim=request.volt_lim, V_aling=request.v_aling,
                                             Calibrate=request.calibrate, Zero_angle_elec=request.zero_angle_elec)
        return response

    def send_transition_callback(self, request, response):
        """Callback for motor send transition service."""
        self._motor.send_transition(transition=request.transition)
        return response

    def command_callback(self, msg):
        """Callback for motor command subscriber."""
        self._motor.send_command(msg.angle)

    def publish_motor_output(self):
        """Publish motor state data."""

        if self._publisher_mec_out.is_activated:
            msg_out_mec = OutMec()
            msg_out_mec.shaft_angle = self._motor.motor_mec_out['shaft_angle']
            msg_out_mec.shaft_angle_sp = self._motor.motor_mec_out['shaft_angle_sp']
            msg_out_mec.shaft_velocity = self._motor.motor_mec_out['shaft_velocity']
            self._publisher_mec_out.publish(msg_out_mec)

        if self._publisher_elec_out.is_activated:
            msg_out_elec = OutElec()
            msg_out_elec.current = self._motor.motor_elec_out['current']
            msg_out_elec.electrical_angle = self._motor.motor_elec_out['electrical_angle']
            msg_out_elec.ua = self._motor.motor_elec_out['Ua']
            msg_out_elec.ub = self._motor.motor_elec_out['Ub']
            self._publisher_elec_out.publish(msg_out_elec)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Callback for the configure transition."""
        self.initialize_motor_components()
        self._timer = self.create_timer(self._timer_duration_param.value, self.publish_motor_output)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Callback for the activate transition."""
        self._logger.info(f"Activated with parameter value: {self._channel_param.value}")
        self._logger.info(f"Activated with parameter value: {self._node_id_param.value}")
        self._motor = MotorDriver(cobid=self._node_id_param.value, channel=self._channel_param.value, dicfile=self._can_parser_dbc_param.value)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Callback for the deactivate transition."""
        self._motor.shutdown()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Callback for the cleanup transition."""
        self.destroy_timer(self._timer)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Callback for the shutdown transition."""
        self.destroy_timer(self._timer)
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
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

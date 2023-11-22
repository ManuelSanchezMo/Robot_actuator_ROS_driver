# Robotic Actuator ROS 2 Driver

This ROS 2 package acts as a bridge between ROS 2 and the [3D printed robotic actuator](https://github.com/ManuelSanchezMo/Robotic_actuator). It simplifies the integration process by offering a ROS 2 node with communication interfaces and services.

## Driver Overview

![Motor Finite State Machine](./imgs/Motor_FSM.png)

### Dependencies

To use this ROS 2 driver, make sure the following dependencies are installed:

- `can`: The CAN (Controller Area Network) protocol library.
- `can_tools`: Tools for working with CAN messages.

## ROS 2 Interface

### Published Topics

#### Input:

- **/motor_1/command_angle** (*motor_interfaces/SendComm.msg*): Input angle command.
- **/motor_1/elec_out** (*motor_interfaces/OutElec.msg*): Output electrical state of the motor.
- **/motor_1/mec_out** (*motor_interfaces/OutMec.msg*): Output mechanical state of the motor.

### Provided Services

The driver offers the following services for configuration and control:

- **/motor_1/motor_config_1** (*motor_interfaces/Config1.srv*): Configuration service for motor PID parameters.
- **/motor_1/motor_config_2** (*motor_interfaces/Config2.srv*): Configuration service for motor PID_vel parameters.
- **/motor_1/motor_config_3** (*motor_interfaces/Config3.srv*): Configuration service for general motor parameters.
- **/motor_1/send_transition** (*motor_interfaces/SendTransition.srv*): Service to send transitions for the internal Finite State Machine.

## Usage Example

Here's a basic example of how to use the ROS 2 node in Python:

```python
import motor_driver 
import numpy as np
if __name__ == '__main__':
    motor = motor_driver.MotorDriver(cobid = 10, channel='vcan0')
    print(motor.cobid)
    motor.send_config_1(P_controller=300.0, I_controller=0.1,D_controller=0.0)
    motor.send_config_2( P_vel = 2.0, I_vel = 0.01, D_vel = 0, Vel_lim  = 20.0)  
    motor.send_config_3( Volt_lim = 20.0, V_aling = 7.0, Calibrate = 0, Zero_angle_elec  = 0.0)
    motor.send_transition(1)
    motor.send_transition(3)
    x = np.linspace(1, 10, num=10)
    y = np.sin(x)
    for n in range(y.size):
        print(n)
        motor.send_command(y[n])
        print(motor.motor_mec_out)
        print(motor.motor_elec_out)
    print('ended')
    motor.shutdown()
```
# MIT_Mini_Cheeta_Motor_ROS2_Driver

This is a ROS2 control hardware interface for controlling MIT mini cheetah motors written at California Polytechnic State University, San Luis Obispo for the Switch robotic dog masters thesis seen at https://github.com/jack33001/CP-Quadruped-Control. 

This driver was forked from the driver https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-can developed at the Underactuated Lab in Robotics Innovation Center at DFKI GmbH, Bremen. 

## Supported Motors

Motors currently supported:

  AK80_6_V1
  AK80_6_V1p1
  AK80_6_V2
  AK80_9_V1p1
  AK80_9_V2
  AK70_10V1p1
  AK10_9_V1p1
  GIM8108

### Adding Motors

Adding a new motor configuration, of commercial or custom motors to the driver can be accomplished through modification of several files. 

1. Add motor parameters to MotorDriver.hpp. These limits are the translation from position, velocity, ect, entered by the user to the actual rotation of the motor. 

Motor parameter entries should follow the following template. All enteries must include, position limits, velocity limits, torque limits, PD gain limits for Proportional, and Derivative, and axis direction. Note axis direction may be configured in the ROS hardware interface. 

    // Working parameters for Steadywin GIM8108 firmware
    const motorParams GIM8108_params = {
        -15*2*3.14159265359,  // P_MIN
        15*2*3.14159265359,   // P_MAX
        -45,    // V_MIN
        45,     // V_MAX
        -18,  // T_MIN
        18,   // T_MAX
        0,    // KP_MIN
        500.0,  // KP_MAX
        0,      // KD_MIN
        5,      // KD_MAX
        1       // AXIS_DIRECTION
    };

2. Add the motor parameter to the motor type enum in MotorDriver.hpp. This is used to verify a supported motor is being accessed.

    enum MotorType {
    AK80_6_V1,
    AK80_6_V1p1,
    AK80_6_V2,
    AK80_9_V1p1,
    AK80_9_V2,
    AK70_10V1p1,
    AK10_9_V1p1,
    GIM8108
    };

3. Add motor type to the stringToMotorType function in MotorDriver.cpp. This helper function converts the motor type from the URDF from a string to one of the recognized motor types. 

    if (motor_type_str == "GIM8108") {
        return motor_driver::MotorType::GIM8108;
    }

4. Test new motor configuration carefully and thouroughly. Errors in parameters may lead to unintended large commands in position, velocity, or torque. These can cause unexpected and dangerous movements which can lead to damage or injury. As a precaution keep gains low, and be prepared to power off the motor. 

## Setup

1. Add the mit_mini_cheeta_motor_driver directory and it's contents to your ROS2 project in the src directory. 

2. Add motor configurations for each motor in a ROS2 control URDF file. Parameters include:

    joint_name: the joint name from the robot URDF
    motor_type: The supported motor type from the enum list in MotorDriver.hpp
    can_bus: The CAN bus the motor is connected to (usually can0 or can1 on Jetson)
    can_id: The CAN ID of the motor
    flip: set True or False to flip the positive direction of the motor
    effort_limit: Limit in Newton-M that if exceeded the motor will cut effort and go into safety standby mode
    command_type: radians or degrees for command type
    frequency: motor loop frequency of the motor control thread in hz
    zero_position_offset: A position offset to be added to the motors position command. 
    min_position_limit: Lower position limit expressed in the unit declared by command_type.
    max_position_limit: Upper position limit expressed in the unite decared by command_type. 
              

   <ros2_control name="mit_mini_cheeta_motor_driver/CANMotorDriver_FRH" type="system" > 
        <hardware>
            <plugin>mit_mini_cheeta_motor_driver/CANMotor</plugin>
            <param name="joint_name">fr_hip</param> 
            <param name="motor_type">GIM8108</param>
            <param name="can_bus" >can0</param>
            <param name="can_id" >10</param>
            <param name="flip" >true</param>
            <param name="effort_limit" >2</param>
            <param name="command_type" >radian</param>     
            <param name="frequency" >50.0</param>   
            <param name="zero_position_offset" >0</param>
            <param name="min_position_limit" >-3.13</param>
            <param name="max_position_limit" >0.531</param>

        </hardware>

        <joint name="fr_hip">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <command_interface name="effort"/>
            <command_interface name="kp"/>
            <command_interface name="kd"/>
            <command_interface name="m_state"/>
            <command_interface name="flip"/>
            <state_interface   name="position"/>
            <state_interface   name="velocity"/>
            <state_interface   name="effort"/>
            <state_interface   name="m_state"/>
            <state_interface   name="flip"/>
        </joint>
    </ros2_control>

3. Build and launch the ROS2 project. Make sure to publish the robot description linked to the ROS2 control URDF file. The controller manager needs the ROS2 URDF published to launch the motor hardware interfaces. 

## Control

Control of the motor is achieved through command interfaces and state information is expressed through state interfaces. The command interfaces are as follows.

    position: commanded position 
    velocity: commanded velocity
    effort: commanded torque
    kp: proportional gain
    kd: derivative gain
    m_state: motor state (see below)
    flip: flip motor postive direction (true or false)

The m_state command is the variable that controls the state machine of the motor. By setting m_state to a integer the mode of the motor is changed.
 
    0: CMD: Send standard position velocity, PD, torque command. Stay on m_state 0.
    1: ENABLE: Send enable command then return to m_state 0.
    2: DISABLE: Send disable command. Stay on m_state 2.
    3: ZERO: Send zero command if safe, power cycle will run. After go to m_state 0(If the absolute postion is less than 0.2 or if kp and kd are 0 so the motor doesn't jump).
    4: E-STOP: Send move cmd (0,0,0,0.5,0), position 0 with small damping. Stay on m_state 4. 


State interfaces return the latest state information published from the motor thread to the state interface. For these values to update, the controller manager must be running and available. 

    position: position at the time the last command was received
    velocity: velocity at the time the last command was received
    effort: commanded torque at the time the last command was received
    kp: proportional gain at the time the last command was received
    kd: derivative gain at the time the last command was received
    m_state: current motor state 
    flip: current flip motor postive direction (true or false)


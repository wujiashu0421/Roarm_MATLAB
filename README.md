# Roarm_MATLAB
 MATLAB backend scripts for Roarm Experiment

# Useful Links:

- Roarm overview: https://www.waveshare.com/wiki/RoArm-M2-S#How_to_Use
- All JSON Commands for robotic arm could be found here: https://www.waveshare.com/wiki/RoArm-M2-S_Robotic_Arm_Control
- Python UART example: https://www.waveshare.com/wiki/RoArm-M2-S_Python_UART_Communication

# Dependencies
Install SPART by navigating to the SPART source code folder and execute the following command in MATLAB command window:
'SPART2path'

# Structure:

MATLAB Backend: Responsible of gathering the inputs from user and feedback information from robotic arm, compute optimal tracjectory for the robotic arm to move.
ROS Middleware: Take commands from MATLAB, instruct robot arm to move. Receive feedback information from the robotic arm, provide that information back to MATLAB.
Actual Arm/Rviz: Operation or simulation of the arm.
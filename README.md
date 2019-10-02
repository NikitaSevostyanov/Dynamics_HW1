# Dynamics_HW1
Solving forward and inverse kinematic for AR-601M
MATLAB executable files:
1. FK.m - function for calculating forward kinematics using DH convention
2. AR_601M.m - file for MATLAB, where I develop a kinematic model of the robot in plot and use functions for calculating inverse and direct kinematic problems
How to open:
1. Download AR_601M.m and FK.m files in one folder and manually run it from MATLAB
2. Run AR_601M.m project 
3. In file AR_601M.m you can set different angles theta by adding needed angles in matrix q0.
4. You can gain the result with command line,just write "T" for viewing the result of my method and "Direct" for viewing the result of MATLAB function fkine.
# How to Run
1. Open directory, where you are going to work
2. git clone https://github.com/NikitaSevostyanov/Dynamics_HW1.git
3. ./matlab
4. Open the folder Dynamics_HW1
5. Run “AR_601M” ! You must have Robotics ToolBox (http://petercorke.com/wordpress/toolboxes/robotics-toolbox)

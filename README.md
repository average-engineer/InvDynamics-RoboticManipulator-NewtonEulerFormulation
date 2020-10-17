# InvDynamics-RoboticManipulator-NewtonEulerFormulation
Inverse dynamics of a robotic manipulator of any DOF using recursive Newton Euler formulation (outward iteration from base to tip for finding link angular and linear velocities and accelerations and inward iteration from tip to base for finding joint torques and forces). The code is made on the basis of proximal DH parameters of the manipulator, but the code takes the input in form of the distal DH parameters of the manipulator (which are them converted to proximal by the code itself). 
The example of a simple 2 link manipulator has been taken for the code.

Explanation of the included scripts in the repository:

1). 

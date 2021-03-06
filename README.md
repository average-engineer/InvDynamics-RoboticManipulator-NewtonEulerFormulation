# InvDynamics-RoboticManipulator-NewtonEulerFormulation
Inverse dynamics of a robotic manipulator of any DOF using _**Recursive Newton Euler formulation**_ (J.Y.S Luh, M.W. Walker and R.P.C. Paul, “On-Line Computational Scheme for Mechanical Manipulators”, Journal of Dynamic Systems, Measurement and Control, Vol. 102, No. 6, pp. 69 76, 1980). This formulation involves outward iteration from base to tip for finding link angular and linear velocities and accelerations and inward iteration from tip to base for finding joint torques and forces. The code is made on the basis of proximal DH parameters of the manipulator, but the code takes the input in form of the distal DH parameters of the manipulator (which are them converted to proximal by the code itself). 
The example of a simple **2 link manipulator** has been taken for the code. The distal DH parameters of the 2 link manipulator need to be input into the text file `dhParameters_2link.txt`.

The code structure is given by:
![RNEA Code Structure](https://github.com/average-engineer/InvDynamics-RoboticManipulator-NewtonEulerFormulation/blob/main/NewtonEulerDynamicModel.PNG)

Explanation of the included scripts in the repository:

1. `Inv_Dynamics_NE.m` is the main script which computes the joint axis forces and torques for any given manipulator. All the main computations and the inward and outward iterations are done in this script only. First the outward iterations are carried out to find the link angular and linear velocties and accelerations. The linear accclerations of the link centre of masses with respect to the frames at the proximal joints of the links are also computed. The starting point for outward iterations is usually the fact that the base of the manipulator doesn't have any speed but has a pseudo acceleration of `9.81 m/s^2` due to the effect of gravity. Next, the inward iteraions are carried out to calculate the interlink forces and torques (forces and torques applied on one link by the previous link). The starting point for inward iteration is the fact that for an open chain manipulator moving the end effector freely in space without any contact with the environment (assumption for this code), the last link carrying the end effector doesn't apply any force and torque as there is no link to apply any force or torque on. Once, the forces and torques have been calculated, their components along the joint axes will be the joint torques.

2. `ArmMatrix.m` is a function which parses the distal DH table of the manipulator from the text file and converts the distal DH parameters to proximal DH parameters. It also outputs the homogenous tranformation matrix from one frame to the adjacent frame.

3. `inertia_matrix_COM.m` is a function which calculates the inertia tensor for each link. Note that in this case, contrary to the euler langrange formulation, the mass moment of inertias are computed with respect to the frames which are located at the centre of mass of the links (the COM frames will have the same orientation as that of the frames at the proximal joints of the links).

This dynamic formulation is often more suited for real world applications like real time testing of controller design. It has a computational complexity of `O(N)` thus making it much more computationally efficient than the _**Euler Lagrange Formulation**_.

The validation of the dynamic model using the example of a 3 axis SCARA manipulator can be accessed from my [Projects Website](https://average-engineer.github.io/Projects-Website-Ashutosh-Mukherjee/)

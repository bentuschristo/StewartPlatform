# Stewart Platform MATLAB Model and Simulations

This repository contains the MATLAB programs (.m files) that contains the kinematic model, dynamic model and simulation programs of a 6-DOF Stewart platform.

Instructions:
1. The kinematic.m file contains the kinematic model of the Stewart platform. From this file, you can edit the dimensions of the base and platform. Inverse kinematics analysis was used to made this model, which means that the position and orientation of the platform are known, while the length of each legs are determined through various calculations.
2. The kinesim.m file contains the kinematic model simulation of the Stewart platform. You can change the position and orientation input in order to see how each legs changes in order to achieve a certain position.

I am currently working on the dynamic model of the Stewart platform. I will keep updating the code as long as I find something new.

## References
[1] H. Guo and H. Li, “Dynamic analysis and simulation of a six degree of freedom stewart platform manipulator,” Proceedings of the Institution of Mechanical Engineers, Part C: Journal of Mechanical Engineering Science, vol. 220, no. 1, pp. 61–72, 2006.

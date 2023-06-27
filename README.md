# ArmTraj

### What does it do?
This program is made specifically for two jointed arms and it helps with achieving smooth motion.  
It creates a smooth interpolation for the positions and uses Trajectory Parametrization to calculate velocity.  

### Getting Started
1. Download Processing <a href = "https://processing.org/">here</a>.
2. Install the library ControlP5 which is used for the UI. (Sketch -> Import Library -> Manage Libraries) <img src ="https://github.com/Teddy-Mengistie/ArmTraj/assets/55628361/dfc6381b-ad84-4817-8e27-68d25759642c" height = 350 width = 300>
3. Search for "ControlP5" and click on download
4. Copy over the code from the "Arm_Visual.pde" file on this repository into your processing program.
5. Press Run

### Costraints
![image](https://github.com/Teddy-Mengistie/ArmTraj/assets/55628361/de8d4952-5468-4cd3-b4ee-e15b7c61ea38)
  
- Acceleration(Blue)/Deceleration(Red) constraints: The units are in rad/s/s and the joints never exceed this amount of angular acceleration throughout the motion. The color blue along the path denotes acceleration of the end effector, and the color red denotes deceleration.
- Velocity Constraint(White): The units are in rad/s and the joints never exceed this amount of angular velocity throughout the motion. The color white alont the path denotes the end effector has reached the maximum possible velocity.
- Curvature Constraint(Green): The user does not have control over this constraint and it automatically constrains the velocity when there is a lot of curvature in the path.



   

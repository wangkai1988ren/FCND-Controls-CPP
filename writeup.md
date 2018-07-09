# FCND - Building a Controller
![Quad Image](./misc/enroute.png)

# Content
1、Implemented body rate control in C++.
2、Implement roll pitch control in C++.
3、Implement altitude controller in C++.
4、Implement lateral position control in C++.
5、Implement yaw control in C++.
6、Implement calculating the motor commands given commanded thrust and moments in C++.




### Starter Code

The whole C++ code implementing controll function is contained in source file QuadController.cpp, and the tuned parameters are contained in text file QuadControlParams.txt.


### Control Architecture
![Quad Image](./ca.png)
#### 1. Implemented body rate control in C++.The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.



#### 2. Implement roll pitch control in C++.The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.


#### 3. Implement altitude controller in C++.The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.



#### 4. Implement lateral position control in C++.The controller should use the local NE position and velocity to generate a commanded local acceleration.



#### 5. Implement yaw control in C++.The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).



#### 6. Implement calculating the motor commands given commanded thrust and moments in C++.The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.



### Execute the flight
#### 1. Does it work?
It works!

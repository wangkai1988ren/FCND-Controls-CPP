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
![Quad Image](./misc/ca.png)

#### 1. Implemented body rate control in C++.The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.
In the line 105~107 I use the difference value of pqrCmd and pqr(estimated body rates), mutiply the inerita and kpPQR rate to get the momentCmd.


#### 2. Implement roll pitch control in C++.The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.
In the line 136~146, I take several steps to implement the roll pitch control:
step 1: Dividing collThrustCmd by mass to get collective accelerate.
step 2: constrain target_R13/R23 in -maxTiltAngle and maxTiltAngle.target_R13/R23 is accelCmd divding collective accelerate.
step 3: Using a P method to command b_x_command_dot and b_y_command_dot.
step 4: Rotating the command b_x_command_dot and b_y_command_dot to get pqrCmd
![Top Down View](./misc/rp.png)

#### 3. Implement altitude controller in C++.The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

In the line 178 to 183, I implement a PID altitude control.
```
this->integratedAltitudeError += (posZCmd - posZ)* dt;
velZCmd = CONSTRAIN(velZCmd, -1 * maxAscentRate, maxDescentRate);
float velCmd_bar = kpPosZ * (posZCmd - posZ)+ velZCmd;
float a_z_desire = accelZCmd + kpVelZ * (velCmd_bar - velZ) + KiPosZ * this->integratedAltitudeError;
thrust = mass * ( -a_z_desire + 9.81f) / R(2, 2);
```
![Top Down View](./misc/at.png)

#### 4. Implement lateral position control in C++.The controller should use the local NE position and velocity to generate a commanded local acceleration.

In the line 220 to 230， I implement a PD lateral position control.
```
if (velCmd.mag() > maxSpeedXY)
 {
   velCmd *= maxSpeedXY / velCmd.mag();
 }
 V3F VelXYCmd_bar = kpPosXY * (posCmd - pos) + velCmd;
 accelCmd +=  + kpVelXY * (VelXYCmd_bar - vel);
 if (accelCmd.mag() > maxAccelXY)
 {
   accelCmd = accelCmd * maxAccelXY / accelCmd.mag();
 }  
 accelCmd.z = 0;
```

#### 5. Implement yaw control in C++.The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).
In the line 251 to 261， I implement a P yaw control. the yaw error should be constrain in -pi to pi.
```
float yaw_error = yawCmd - yaw;
  yaw_error = fmodf(yaw_error, 2 * M_PI);
  if (yaw_error >  M_PI)
  {
	  yaw_error -= 2 * M_PI;
  }
  if (yaw_error < -M_PI)
  {
	  yaw_error += 2 * M_PI;
  }
  yawRateCmd = kpYaw * yaw_error;
```



#### 6. Implement calculating the motor commands given commanded thrust and moments in C++.The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

In the 78 to 81, I implement a 4IN-4OUT linear equation,which convert the collThrustCmd, Mx, My, Mz to the four motor thrust.
```
cmd.desiredThrustsN[0] = collThrustCmd / 4.f + momentCmd.x / L * M_SQRT2 / 4.f + momentCmd.y / L * M_SQRT2 / 4.f - momentCmd.z / kappa / 4.f; // front left
cmd.desiredThrustsN[1] = collThrustCmd / 4.f - momentCmd.x / L * M_SQRT2 / 4.f + momentCmd.y / L * M_SQRT2 / 4.f + momentCmd.z / kappa / 4.f; // front right
cmd.desiredThrustsN[2] = collThrustCmd / 4.f + momentCmd.x / L * M_SQRT2 / 4.f - momentCmd.y / L * M_SQRT2 / 4.f + momentCmd.z / kappa / 4.f; // rear left
cmd.desiredThrustsN[3] = collThrustCmd / 4.f - momentCmd.x / L * M_SQRT2 / 4.f - momentCmd.y / L * M_SQRT2 / 4.f - momentCmd.z / kappa / 4.f; // rear right

```


### Execute the controler
#### 1. Does it work?
It works!

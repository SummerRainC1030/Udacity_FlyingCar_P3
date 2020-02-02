# FCND-Control-of-a-3D-Quadrotor Readme #

This is the readme for FCND-Control-of-a-3D-Quadrotor.



### Body rate and roll/pitch control (scenario 2) ###

First, you will implement the body rate and roll / pitch control.  For the simulation, you will use `Scenario 2`.  In this scenario, you will see a quad above the origin.  It is created with a small initial rotation speed about its roll axis.  Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.


1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`
      float l = L / sqrt(2.f); // perpendicular distance to axes
      float p_bar = momentCmd.x / l; // p_bar = (I_x * U_p) / (k_f * l) = F1 - F2 + F3 - F4
      float q_bar = momentCmd.y / l; // q_bar = (I_y * U_q) / (k_f * l) = F1 + F2 - F3 - F4
      float r_bar = - momentCmd.z / kappa; // drag/thrust ratio (k_m/k_f);  r_bar = (I_z * U_r) / k_m = F1 - F2 - F3 + F4
      float c_bar = collThrustCmd; // c_bar = c * m / k_f = F1 + F2 + F3 + F4

      cmd.desiredThrustsN[0] = (p_bar + q_bar + r_bar + c_bar) / 4.f; // front left 1
      cmd.desiredThrustsN[1] = (-p_bar + q_bar - r_bar + c_bar) / 4.f; // front right 2
      cmd.desiredThrustsN[2] = (p_bar - q_bar - r_bar + c_bar) / 4.f; // rear left 3
      cmd.desiredThrustsN[3] = (-p_bar - q_bar + r_bar + c_bar) / 4.f; // rear right 4


 - implement the code in the function `BodyRateControl()`
        
      V3F MI;
      MI.x = Ixx;
      MI.y = Iyy;
      MI.z = Izz;  
      momentCmd = MI * kpPQR * (pqrCmd - pqr);  

 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot

       kpPQR = 76, 23, 5 // 23, 23, 5


2. Implement roll / pitch control

 - implement the code in the function `RollPitchControl()`
      float c = -collThrustCmd / mass;
      float b_x_cmd = accelCmd.x / c;
      
      float b_x_err = b_x_cmd - R(0,2); // b_x_a = R13
      float b_x_p_term = kpBank * b_x_err; // P-controller
   
      float b_y_cmd = accelCmd.y / c;      
      
      float b_y_err = b_y_cmd - R(1,2); // b_y_a = R23
      float b_y_p_term = kpBank * b_y_err; // P-controller
        
      pqrCmd.x = (R(1,0) * b_x_p_term - R(0,0) * b_y_p_term) / R(2,2);
      pqrCmd.y = (R(1,1) * b_x_p_term - R(0,1) * b_y_p_term) / R(2,2);
      pqrCmd.z = 0;


 - Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot

      kpBank = 8  // 5


3. Performance Metrics
  - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
  - roll rate should less than 2.5 radian/sec for 0.75 seconds


### Position/velocity and yaw angle control (scenario 3) ###

Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

1. Implement LateralPositionControl

 - implement the code in the function `LateralPositionControl()`
      V3F kpPosXY0 = kpPosXY;
      kpPosXY0.z = 0;

      V3F kpVelXY0 = kpVelXY;
      kpVelXY0.z = 0;

      if (velCmd.mag() > maxSpeedXY) velCmd = velCmd.norm() * maxSpeedXY;
  
     V3F pos_err = posCmd - pos;
      V3F pos_err_dot = velCmd - vel;

      V3F pos_dot_dot_command = kpPosXY0 * pos_err + kpVelXY0 * pos_err_dot + accelCmd;

      accelCmd = pos_dot_dot_command;
      if (accelCmd.mag() > maxAccelXY) accelCmd = accelCmd.norm() * maxAccelXY;

2. Implement AltitudeControl

 - implement the code in the function `AltitudeControl()`
      float z_err = posZCmd - posZ;
      float z_err_dot = velZCmd - velZ;
      float b_z = R(2, 2);


      float u_1_bar = (kpPosZ * z_err) + (kpVelZ * z_err_dot) + accelZCmd; //+i_term : KiPosZ * integratedAltitudeError

      float c = (u_1_bar - CONST_GRAVITY) / b_z;

      thrust = -mass * c;

 - tune parameters `kpPosXY` and `kpPosZ`
    kpPosXY = 30         //1
    kpPosZ = 30          //1


 - tune parameters `kpVelXY` and `kpVelZ`
    kpVelXY = 10         //4
    kpVelZ = 8          //4

3. Implement YawControl

 - implement the code in the function `YawControl()`
    if (yawError > F_PI)
    {
      yawError -= 2.f * F_PI;
    }
    else if (yawError < -F_PI)
    {
      yawError += 2.f * F_PI;
    }
    yawRateCmd = kpYaw * yawError;


 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`
    kpYaw = 3         #1
    kpPQR = 76, 76, 6         

4. Performance Metrics
    - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
    - Quad2 yaw should be within 0.1 of the target for at least 1 second



### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:
    kpPosXY = 30   
    kpPosZ = 65       
    KiPosZ = 20
    kpVelXY = 12.5          #4
    kpVelZ = 9  

### Tracking trajectories ###

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory.  For this simulation, you will use `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out **Extra Challenge 1** below!

How well is your drone able to follow the trajectory?  It is able to hold to the path fairly well?


### Extra Challenge 1 (Optional) ###

You will notice that initially these two trajectories are the same. Let's work on improving some performance of the trajectory itself.

1. Inspect the python script `traj/MakePeriodicTrajectory.py`.  Can you figure out a way to generate a trajectory that has velocity (not just position) information?

2. Generate a new `FigureEightFF.txt` that has velocity terms
Did the velocity-specified trajectory make a difference? Why?

With the two different trajectories, your drones' motions should look like this:



### Extra Challenge 2 (Optional) ###

For flying a trajectory, is there a way to provide even more information for even better tracking?

How about trying to fly this trajectory as quickly as possible (but within following threshold)!


## Evaluation ##

To assist with tuning of your controller, the simulator contains real time performance evaluation.  We have defined a set of performance metrics for each of the scenarios that your controllers must meet for a successful submission.

There are two ways to view the output of the evaluation:

 - in the command line, at the end of each simulation loop, a **PASS** or a **FAIL** for each metric being evaluated in that simulation
 - on the plots, once your quad meets the metrics, you will see a green box appear on the plot notifying you of a **PASS**


### Performance Metrics ###

The specific performance metrics are as follows:

 - scenario 2
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds

 - scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second


 - scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

 - scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
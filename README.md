# ACADO-MPC Package 
## About the project  

This project implements ACADO-generated Model Predictive Control (MPC) for an mini autonous vehicle based on MuSHR Car Autonomy Stack from University of Washington. The controller is able to track a pre-defined trajectory with 10^-3 s level computation time, and this package have user friendly interfaces for modification of trajectories and controller formulations. We demonstrate the trajectory tracking performance in simulation and real-time onboard experiment. The exact workflow of the system, the formulation of the controller, and details for implementing the controllers are documented below.

This project is advised by [Professor Xiangru Xu](https://directory.engr.wisc.edu/me/Faculty/Xu_Xiangru/), at University of Wisconsin-Madison, and contributed by William Dong, Wenjia Li, Yingwei Song, Harry Zhang, and Kevin Mo in [Autonomous & Resilient Control Lab](https://xu.me.wisc.edu).

## Setup

### MuSHR
In this project, [MuSHR project](https://mushr.io/about/) developed by [Personal Lab](https://personalrobotics.cs.washington.edu/people/) at University of Washington was used as the hardware platform for experiment. For the detailed instruction to setup the hardware platform, please check the link for MuSHR Project, and please finish up to [teleop control configuration](https://mushr.io/tutorials/first_steps/) before implementing this package.

### ACADOtoolkit
In addition, this package also requires setup of ACADO Toolkit, which is a environment for dymnamic optimization and control, with a user-friendly framework for different formulations. We use this package to test optimization formulation, generate real-time MPC computation package for simulations and experiments. Before following the instruction below, please setup [ACADO package with Matlab interface](https://acado.github.io/matlab_overview.html) on your base station following the instruction in the link. (For the base station, it is recommended to use laptop or desktop computer with intel CPU. Single-board computers, often used as onboard computers, like Jetson-nano, might not able to generate real-time computation package with accurate results.)

### MoCap
Lastly, this package also requires motion capture to provide feedback state of the vehicle. 
  1. Open Motive software on the base station, and if icons of all cameras are not in their appropriate positions, please re-calibrate the camera positions with following [the tutorial](https://docs.optitrack.com/motive/calibration). 
  2. Make sure the body for the vehicle is built with appropirate orientations, and this assest is on in the space.
  3. Make sure the base station is connected to the appropriate network, same for the onboard computer. Here we strongly recommend to use ARCLab_WIFI instead of UWNet to avoid network issues. Also, in the streaming and network settings, please set the correct Local Interface IP. If you are not sure, please do `ip config` in Windows Command Prompt to check the different IP addresses for different networks.
  4. Build the ROS interface package to get data onto ROS. Here we strongly recommend to build the motion capture ROS interface onto the onboard computer, which has lower risks on data transmission and network issues. The detailed tutorial is [here](http://wiki.ros.org/mocap_optitrack).
  5. DEBUG: There could be several issues when you try to run this code according to the tutorial, and since there are network isssues getting involved, and the package itself is already obsolete, it is hard to find a starting point to debug. A good way to look into it is by get things figured out level by level, instead of trying randomly. Firstly, please make sure you have a trustworthy network to deliver your data, and having incorrect network configurations or inappropriate networks will block the workflow. Then, make sure you are editting the correct config file, and this file is correctly loaded as rosparams, and please read the returned message carefully when you run the program (does it change when you modify your config file, or it is just not loaded yet). Lastly, make sure there is data inside the published rostopics, and, if not, please check issues with hostnames and firewall issues.
  6. Run the program by `roslaunch mocap_optitrack mocap.launch`, and you should see states have been published correctly in designated rostopics.

## Workflow

Shown in the image below, real-time MPC control of the MuSHR Car is carried out by running several aspects.

<p align="center">
  <img src="https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/images/ACADO-MPC%20Workflow.png" width=70% height=70% alt>
</p>

Base Station with Motive software will process data from motion capture system and feed the state of the vehicle to the Mocap ROS node on the onboard computer ROS environment, which will publish the vehicle state as a ROS topic. Then the MPC solver node will subscribe the state topic and optimize over horizons with dynamics and input constraints to get a array of optimal inputs in the prospective time to minimize the cost function. The first of input array, the immediate one to be performed, will be published as another ROS topic. The input topic will be subscribed by the autonomy stack of the MuSHR Car, and will drive the drivetrain to turn the steering and power the wheel in reality. This workflow will iterate in real-time by ROS, and new inputs will be computed based on updated state of the vehicle. To control the onboard computer remotely and visualize real-time data, multi-machine ROS master is used on the base station as a human interface.

## Formulation
The controller is formulated as below, in equation (1) to (4), where $x_t$ is the actual state of the robot at real time $t$, and $x_{i|t}$ and $u_{i|t}$ is the prediction states and inputs of the robot at horizon $i$ and real time $t$. In addition, $x_{r{i|t}}$ and $u_{r{i|t}}$ are the reference states and inputs of the robot. Equation (1) is the cost function of the controller to minimize, (2) is the robot dynamics constraint and (3) is the input constraint. Equation (4) states the first state over horizons is the current state in real time. The intuition is to get the optimal input over $h$ future horizons based on the constraints, and take the first input $u_{t|t}$ as the actual input $u(x(t)) = u_{t|t}$.

<p align="center">
  <img src="https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/images/formulation_1.png" width=70% height=70% alt>
</p>

The constriants of dynamics in equation (2) is detailed in equation (5) to (7) below, input constraint (3) is detailed in equation (8) and (9), and the weight matrix in cost function (1) is designed in equation (10) below, and prediction time is designed to be 1 second.

<p align="center">
  <img src="https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/images/formulation_2.png" width=40% height=40% alt>
</p>

## Code Generation

Real-time onboard qpOASES interface package for MPC solver need to be generated by ACADO-Toolkit.

  1. On your base station, download optimization formulation for ACADO, [mushr_mpc_continuous.cpp](https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/formulation/mushr_mpc_continuous.cpp), and move it to `ACADOtoolkit/examples/code-generation/mpc_mhe/` inside the ACADO package. You may modify the name of the file and formulation based on your need. Please check [tutorials](https://acado.sourceforge.net/doc/html/db/d4e/tutorial.html) for ACADOtoolkit to see how to modify the formulation.

  2. Generate code generator on your base station:
```
cd ACADOtoolkit/build/
cmake ..
make
```
  3. Now you should see a execuatable program named "code_generation_mushr_mpc_continuous" in `ACADOtoolkit/examples/code-generation/mpc_mhe/`. We also provide a [similar one](https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/formulation/code_generation_mushr_mpc_continuous) in formulation folder. If you named your formulation in a different name like XXX, the generator executable will be named as "code_generation_XXX". Please run the executable by:
```
./code_generation_mushr_mpc_continuous
```
  4. Now you should have the qpOASES interface package in the same folder, named as "mushr_mpc_continuous". Lastly, please move a copy of `qpoases` folder from `ACADOtoolkit/external_packages` into the "mushr_mpc_continuous" directory.
  5. Compile the `test.c` in this folder by,
```
make clean
make
```
  6. Run the optimization by run the compiled executable `test` program. 
```
./test
```

## Simulation

We designed several simulation testers to check the performance of the controller. In these testers, the vehicle with bicycle model will try to track a designed reference trajectory. You may change different trajectories to see the tracking performance. The ACADO-generated OCP solver is runed each time in a new step to imitate the MPC scenrio.

To implement the simulation tester, please download matlab filed from [simulation](https://github.com/wisc-arclab/arclab_vehicles/tree/ACADO-MPC/simulation) directory in this branch. Move the package into the generated qpOASES interface package, and run the matlab program to see the result.

A tracked circle in simulation is shown below:

<p align="center">
  <img src="https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/images/conti_sim.jpg" width=50% height=50% alt>
</p>

## Experiment

This section will talk about how to make the real-time controller implemented onto the onboard computer (Jetson-nano) on the MuSHR platform.

  1. Move the generated qpOASES interface package into `~/catkin_ws` worskapce on the onboard computer. Remove the `qpoases` package, `test.c`, and 'Makefile'.
  2. Download [replacement_package](https://github.com/wisc-arclab/arclab_vehicles/tree/ACADO-MPC/package_replacement) in this repo, and move the three replacements into the qpOASES interface package. (It is better to use the qpOASES package in the ACADO package on your own onboard computer instead of the one in this folder, and you may find it at `ACADOtoolkit/external_packages`)
  3. Re-compile the package by running the following commands in the package:
```
make clean
make
```
 
In this way, the ROS-embedded ACADO solver is built. To modify the trajectory for the controller to track, see [#TODO items](https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/mushr_mpc_continuous/test.c) in `test.c` file. To test the tracking performance by not deploying the vehicle on the ground, change the subscribed topic for vehicle state to "/car/car_pose", and the feedback state of the vehicle will be approximated state by encoder, instead of motion capture. After changing the `test.c` file, please remember to recompile it using `make` command.
  
Below are the instructions to run the experiment.
  
  4. Connect your base station and On the base station, `ssh` into the onboard computer on the MuSHR. Have at least three onboard terminals and two base station terminals avaliable.
  5. On all terminals, run the following command to share the same ROS master.
```
export ROS_MASTER_URI=http://your.mushr.ip.address:11311
```
  6. In the first onboard terminal, run the following command to start autonomy stack:
```
roslaunch mushr_base teleop.launch
```
  7. In the second onboard terminal, run the following command to start motion capture:
```
rosparam load directory_to_mocap/config/mocap.yaml /mocap_node
rosrun mocap_optitrack mocap_node
```
  8. In the last onboard terminal, run the following command to run the ACADO controller for trajectory tracking:
```
cd directory_to_mushr_mpc_continuous/
./test
```
Now, MuSHR should start moving following the predefined trajectory, as this [recorded video](https://drive.google.com/file/d/1sUGZVHpW2Q1ipanKkKN7wSeb59a1EsZc/view?usp=sharing) shows.
<p align="center">
  <img src="https://github.com/wisc-arclab/arclab_vehicles/blob/ACADO-MPC/images/experiment.PNG" width=50% height=50% alt>
</p>


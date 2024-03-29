# CARRIE_EE421

Simulation source code for path following control of a 4-wheeled autonomous robot.

## Picture

<img width="1318" alt="image" src="https://user-images.githubusercontent.com/16526959/231543480-4c57d332-ee93-475b-859f-68bc603f5f96.png">

## Source

Forked from: https://github.com/pgh79/PythonRobotics-master

Can cite for a paper from here: https://arxiv.org/abs/1808.10703

## How to use

### Running a simulation
To perform the simulation run the simulator.py file. This file initializes the course, vehicle, simulation, and blackbox objects. During the simulation, press 'q' to quit and close the window. 

During the simulation, the elapsed time and vehicle speed are displayed in the title while the vehicle velocity is plotted as a vector. Various markers show the vehicle COM and other points used for the position controller (talked about later)

When the simulation is complete, a plot will appear displaying the course and the vehicle trajectory, pressing 'shift+s' will save the "blackbox" i.e. trajectory, vehicle inputs, and controller parameters in a new file in the saved_simulations folder. These blackboxes can be reopened and reviewed by running the blackbox.py file with the filename as a terminal argument. Pressing 'q' will close this plot.

More plots will appear showing the lateral acceleration, trajectory-course deviation, speed, acceleration, etc... These plots are generated by the blackbox. Look at blackbox.py to add or modify these plots. For this reason, the values for all of these plots are stored in the blackbox file and can be recovered later if previously saved.

### General modifications

To modify or add a different target path/course, open course.py and make a new function that uses the PathPlanning module.

The physical parameters of the vehicle can be added to or modified in the parameters/vehicle_parameters.py file. Running this file will generate a plot of the vehicle geometry with a marker at the COM and a sample velocity vector.

The simulation parameters (duration & time-step) can be modified in the parameters/simulation_parameters.py file.

### Controllers

The required functions and properties of a controller class is in the controllers/controller.py file. To initialize the controller a vehicle state and course object need to be passed. The only required property is .name. 

Use the command() function to return an Input() object, which includes desired acceleration and steering angle. (This will need to be modified to output motor voltages in the future). 

The plot_realtime() function will plot any relevant geometry during the realtime data. For purepursuit this will plot the lookahead point/vectory (see in PP.py). Do not call plt.show() or make a new plot in this function. Just call plt.plot() or similar.

The export_params() function is used to save any relative variables (PID gains, time-step horizons, cost-matrices, etc...) to the blackbox files for later review. Return a dictionary with string keys of the variable name and then the variable object itself (see PP.py for example). 

### 

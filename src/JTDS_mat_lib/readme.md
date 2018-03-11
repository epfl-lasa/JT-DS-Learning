


## How to Train a JT-DS Model (Outdated -- need to update some of these insstructions!)

This page will walk through an example of how to train a JT-DS model on a series of joint-space demonstrations.

The first thing you need to train a JT-DS model is joint position demonstration data. Import your data into MATLAB in the format described by on demonstration data, prepare your joint position demonstrations in the input format described in `preprocess_demos_jseds.m`.

`[demos, time] = load('path/to/demonstrations.mat')`

Now, we'll need to create a kinematic model for our robot. We'll create a robot model using Peter Corke's [robotics-toolbox](http://petercorke.com/Robotics_Toolbox.html), by calling

`robot = initialize_robot(A,D,Alpha,Qmin,Qmax);`

where `A, D, Alpha, Qmin,` and `Qmax` are the Denavit-Hartenberg parameters of our system and its joint position limits.

Next, we'll create a RobotPlant object, which encodes the basic dynamics of the robot. If our task-space is the Cartesian end-effector position, the second argument should be `end_trans`. (To encode more complex task-spaces, see `RobotPlant.m`).

`robotplant = RobotPlant(robot, 'end_trans');`

Now we preprocess our joint position demonstrations so that they can be used in the learning algorithm. `tol_cutting` defines a velocity threshold which we use to trim the beginning and end of our demonstrations.

`[Data, index] = preprocess_demos_jtds(robotplant, demos, time, tol_cutting);`

Now we're ready to begin training. Specify the options you'd like for your JT-DS training (a full description of the available options exists at `JTDS_Solver.m`). Now you can run the training algorithm!

`[Priors, Mu, Sigma, As, q_to_z] = JTDS_Solver(Data,robotplant,options);`

The outputs of this method are our JT-DS model parameters. `Priors, Mu,` and `Sigma` are the priors, means, and variances of each mode in the GMM. `As` represents the joint behavior augmentation matrices of each region, and `q_to_z` is a function mapping from the joint space to a lower-dimensional latent space. (By default, the mapping uses PCA. To extract the PCA matrix `M`, simply call `M = q_to_z(1)`.)

You can now export your model parameters to a file using the following command:

`export2JSEDS_Cpp_lib('name/of/destination/file.txt',Priors, Mu, Sigma, robot, As, M);`

This same file can then be used to execute the learned system using [the JT-DS C++ controller library](https://github.com/epfl-lasa/JT-DS-lib).

## Simulating the system in MATLAB
We've also implemented a JT-DS controller/simulator in MATLAB to quickly understand the behavior of a system by leveraging Peter Corke's robotics-toolbox visualization suite. To create a JT-DS controller, create a MotionGeneratorBounded object. 

`motion_generator = MotionGeneratorBounded(robotplant, Mu, Sigma, Priors, As, q_to_z);`

You can now specify initial conditions and target task positions for your system, and then integrate the model forward using an ODE solver to get the generated trajectory. `q_initial` is the starting joint-space configuration, `x_targets` is an array of target task-space configurations, `goal_tolerance` denotes how close the end-effector needs to be to the target to accept, and `max_trajectory_duration` specifies how long to integrate each subtrajectory forward.

`[Q_traj, T_traj] = computeFullTrajectory(q_initial, x_targets, motion_generator, goal_tolerance, max_trajectory_duration);`

The outputs `Q_traj` and `T_traj` are matrices denoting the joint configuration and time at each point in the trajectory.

To visualize the trajectory in a 2D/3D simulator, call PlaybackTrajectory:
`PlaybackTrajectory(robotplant, Q_traj, T_traj);`

## Generating Cross-Validation and Hyper-Parameter Search Results 
We've provided an extensive evaluation of different dimensionality reduction schemes to group the local joint behaviors in synergy space. We evaluated 'None','PCA' and 'KPCA', for a comparison of the first two:

run ``crossvalidation_dr_kuka.m`` the expected results are stored in ./cv_data

For a CV with hyper-parameter search of Kernel PCA with Gaussian kernel:

run ``hyperparam_kpca_kuka.m``

Statistics of the results can be found here:
[Link to Google Sheet](https://docs.google.com/spreadsheets/d/1UTTw4sNdMJ5lkdNBWetGq6xl7DyO_NYe_IA_3OfSpLc/edit?usp=sharing)

### Other Useful Functions

* `GraphRobot.m` - create an overlayed series of freezeframes of your robot's configurations throughout its trajectory
* `initialize_robot_figure.m` - instantiates a robotics-toolbox robot figure, which can then be used to visualize robot motions
* `Siimulation.m` - run a MotionGeneratorBounded system forward in real time from a given initial configuration.

# Augmented Joint-space Task-oriented Dynamical Systems (JT-DS) Training Library

## 1. Introduction
The following library provides a framework for learning the parameters of a Joint-space Task-directed Dynamical System. The training component of the algorithm is primary written in MATLAB. The parameters of the model can then be exported, and imported to [the C++ code](https://github.com/epfl-lasa/JT-DS-lib) to run the learned JT-DS on a real robot.

### 1.1 What can this package be used for?
The JT-DS learning framework can be used to extract the underlying behaviors from a series of demonstrated motions, and to encode those motions in a dynamical system. In particular, the demonstrations are joint-space position trajectories for a manipulator moving towards a target task-space position. These demonstrations are converted to a series of locally-exhibited behaviors "explaining" the joint motions in exhibited in each demonstration.

## 2. Requirements and Dependencies
To run the JT-DS learning code, you'll need to have MATLAB installed. (Any version after 2016a will certainly work, though most older versions should too).
The code also relies on [YALMIP](https://yalmip.github.io/), a semidefinite optimization framework, and [Peter Corke's robotics-toolbox framework](http://petercorke.com/Robotics_Toolbox.html) though you don't need to worry about these - versions of YALMIP and robotics-toolbox are included in the "include/" folder.

## 3. Installation and set up
Make sure that `include/` and `src/` are on your MATLAB path. That's it!

## 4. Quick Start
For a brief walkthrough on how to utilize the JTDS-Learning library to train your own model, see [quickstart.md](https://github.com/epfl-lasa/JTDS-Learning/blob/master/src/JTDS_mat_lib/quickstart.md).

### 4.1 Example Implementations
To see a thoroughly documented example of the system at work on a toy problem, check out [demo_footstep.m](https://github.com/epfl-lasa/JTDS-Learning/blob/master/src/JTDS_mat_lib/demo_footstep.m).
To see an implementation of the learning algorithm for the KUKA LWR 4+ robot arm, check out [demo_kuka.m](https://github.com/epfl-lasa/JTDS-Learning/blob/master/src/JTDS_mat_lib/demo_kuka.m) (note that this requires you to also have [rtk-JTDS](https://github.com/epfl-lasa/rtk_JT_DS​) installed).
To see some nice side-by-side comparisons of different A matrices and their effects on motion, check out [demo_varying_A.m](https://github.com/epfl-lasa/JTDS-Learning/blob/master/src/JTDS_mat_lib/demo_varying_A.m).

## 5. Controller Implementation
To run the trained model on a real robot, it is recommended that you use [the C++ JT-DS library](https://github.com/epfl-lasa/JT-DS-lib). The parameter file that you export from the learning process can be fed directly into that library, and used to control your robot using your learned behaviors. For an example of using that library to control a KUKA LWR 4+, see [rtk_JT_DS](https://github.com/epfl-lasa/rtk_JT_DS).

## 6. Tips \& Tricks

### Creating Demonstrations
When creating your own training demonstrations, it's important to understand the fundamental assumptions of the system. Specifically:
* The behaviors are always learned with respect to the final task-position (it's assumed to be the target) so make sure to end your demonstration where you hope the target to be.
* The JT-DS controller, which by default sets P = 1, is always nondecreasing in task-space distance to the target task position. This means that, e.g. if your goal is in front of you and you want your arm to first move backwards and then forwards to the target, no JT-DS can ever encode this. Instead, break this motion up into two behaviors: one where the target is backwards, and the other (beginning from the backwards-position) where the target is the original forwards target.
* Similar joint positions lead to similar joint velocities, so don't give two demonstrations that have the same final position but behave very differently at a shared earlier joint position.

### Computation Time
The largest bottleneck in terms of computation time for `JTDS_Solver.m` is setting up a YALMIP SDP which includes all the GMM components for every datapoint. The optimization, run on a 3.4GHz i7 PC, takes around 3 minutes for ~1000 points and 3 GMM components, and around 15m for ~10000 points and 5 GMM components.

### Fitting a good GMM
When training a JT-DS, the most important subtask is that the GMM is learned correctly, as this will guarantee a good segmentation of local behaviors. There are a number of options you can specify in the `options` struct passed into `JTDS_learning.m` to guide how the GMM will be trained:
* `options.GMM_sigma_type` - Specifies whether the variance matrix is diagonal or full. Full variance matrices generally lead to better fitted Gaussians.
* `options.maximize_BIC` - If true, varies the number of GMM components and selects the best one. This should be used if you're not sure how many sub-behaviors your motion has.
** `options.max_gaussians` - The maximum number of sub-behaviors. The higher this number is, the more likely the GMM will overfit and erroneously encode too many small regions as behaviors.
** `options.BIC_regularization` - The Bayesian Information Criterion score is used to select between GMMs to determine the best-fitted one. By regularizing the BIC to penalize higher numbers of Gaussians, we push the system towards a more compact GMM distribution. The precise regularization equation is available in the option description.
* `options.fixed_num_gaussians` - If you already know the exact number of behaviors you believe your system should encode, you can directly pre-specify it. In this case, remember to set `options.maximize_BIC` to false!

### Simulating the System
When using [computeFullTrajectory.m](https://github.com/epfl-lasa/JTDS-Learning/blob/master/src/JTDS_mat_lib/simulate/computeFullTrajectory.m), the system may end up being poorly behaved and integrate in an unstable manner. You may want to switch the ODE solver used by MATLAB (between ode45, ode15s, or [others](https://www.mathworks.com/help/matlab/math/choose-an-ode-solver.html)) if you find yourself getting NaN values as the output of the integration.

## 7. References
The library is based on the paper "Learning Augmented Joint-Space Task-Oriented Dynamical Systems: A Linear Parameter Varying and Synergetic Control Approach", by Yonadav Shavit,  Nadia Figueroa, Seyed Sina Mirrazavi Salehian and Aude Billard.
For further assistance with the learning, contact yonadav at mit dot edu. or {Nadia dot Figueroa, Sina dot Mirrazavi} at epfl dot ch 
We hope you and your robot enjoy learning together!


# Final Year Project - Distributed Optimisation for Exploration and Trajectory Planning of Unmanned Aerial Vehicles

## Description
My Final Year Project masters thesis. This project consists of a hierarchical control approach of a mutli-MAV system in the exploration of an environment. We seek to maximise the area encompassed by a set of circles; this represents the sensor regions of our MAVs, where the (x,y) position of a circle is the (x,y) position of an MAV, and the radius of a circle is related to the altitude of an MAV. Once the optimal circle positions, i.e. MAV positions, are obtained, we construct the MAV trajectories from their initial positions to the optimal positions using a Nonlinear Model Predictive Control scheme.

### Greens_Method.jl
A function to obtain the union of the area of N intersecting circles. This algorithm works by identifying the boundaries of the circles, and calculates the union area of these circles based on Greens Theorem. This function is inspired by the StackOverflow posts by Ants Aasma & Timothy Shields @ https://stackoverflow.com/questions/1667310/combined-area-of-overlapping-circles

### Position_Optimization.jl
A module for maximising the objective function Greens_Method, i.e. optimises Greens_Method to obtain the position of the circles that maximises the union area encompassed by the circles. The optimisation is performed using an algorithm of the Mesh Adaptive Direct Search class developed by Imperial College London, found @ https://github.com/ImperialCollegeLondon/DirectSearch.jl

### Trajectory_Optimization.jl
A module for performing trajectory optimisation using the Altro solver, found @ https://github.com/RoboticExplorationLab/Altro.jl; the initial MAV positions are supplied and the final MAV positions is the output from Position_Optimization. We construct the trajectories using an Nonlinear Model Predictive Control scheme, with non-linear dynamics for the MAVs. Collision avoidance is implemented using spherical constraints and an algorithm that allows communication between MAVs only when they are within a "danger distance" from each other.

### MonteCarlo_Method.jl
A function to obtain the union of the area of N intersecting circles using a MonteCarlo approximation method.

### Base_Functions.jl
Contains all the base functions used.

### Plotter.jl
Plots our described problem for visualisation.

### Script_MADS.jl
Script to run the positional optimization
1) Generate N random circles in our defined domain
2) Use MADS (objective function defined by Greens_Method.jl) to find optimum position of circles that maximise area

### Script_Altro.jl
Script to run the trajectory optimization
Given the initial and final positions from Script_MADS.jl, find the optimal trajectories from initial to final positions

## How to run problem
Run Script_MADS.jl, then without clearing the workspace, run Script_Altro.jl
include("TDM_Functions.jl")
include("Circle_Domain.jl")
include("TDM_Trajectory_Opt.jl")

using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics

using TrajectoryOptimization
using Altro

# Drone Parameters
mass = 0.5                                       # mass of quadrotor # default 0.5
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix # default Diagonal(@SVector[0.0023, 0.0023, 0.004])
# gravity = SVector(0,0,-3.721)                    # gravity vector # default SVector(0,0,-9.81) 
gravity = SVector(0,0,-9.81)
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u) # default 1.0
km = 0.0245                                      # motor torque constant (motor torque = km*u) # default 0.0245

r_max = 100 * tand(80/2)
N = 5
cir_domain = Circle_Domain.Domains_Problem([], 20.0, 2.0) # 
# global MADS_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max) # should be recovered 20230810
# x_start = TDM_TRAJECTORY_opt.MADS_to_ALTRO(MADS_input )                         # should be recovered 20230810


# MPC optimization
hor = 3.0           # Prediction horizon length         (3 seconds in total)
dt = 0.1            # Time-step length per horizon      (want to separate 3 seconds into 30 pieces)
Nf = Int(hor/dt)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps

tf = hor     # for testing on 20230809
x_start_4test = RBState([-7.77469514777312, 0.7173085987290898, 7.545026699377439, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# x_start_4test = [-7.77469514777312, 0.7173085987290898, 7.545026699377439, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}() # ----- testing on just first UAV on 0802
for i in 1:N
    # push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start[i]))
    push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start_4test)) # test on 20230810
end


```
Enum Altro.TerminationStatus:
UNSOLVED = 0
LINESEARCH_FAIL = 1
SOLVE_SUCCEEDED = 2
MAX_ITERATIONS = 3
MAX_ITERATIONS_OUTER = 4
MAXIMUM_COST = 5
STATE_LIMIT = 6
CONTROL_LIMIT = 7
NO_PROGRESS = 8
COST_INCREASE = 9
```




# prob = nothing
# obj = nothing

# for ind in eachindex(x_start)
include("TDM_Trajectory_Opt.jl")
ind = 1
MAV = MAVs[ind]; # local in for loop
XF = 1189.0
X, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, XF);
println("Distance: $XF; Solver status: ", solver_dBar.stats.status) 


global critical_dis = 0.0

global X = nothing
global solver_dBar = nothing
global prob_dBar = nothing
global obj_dBar = nothing



# might add a while loop outside to iteratively increase the upper boundary until the results would converge
for i in 1.0:0.1:100.0
    X, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, i); 
    println("Distance: $i; Solver status: ", solver_dBar.stats.status)
    # println("Index: $i")
    if Int(solver_dBar.stats.status)!= 2
        global critical_dis = i
        break
    end
end


X, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, 13.6); 



# Document the data and make plots
traj = zeros(Nf, 3) # local in for loop
for i in eachindex(X)
    traj[i,1] = X[i][1]
    traj[i,2] = X[i][2]
    traj[i,3] = X[i][3]
end

# plot!(traj[:,1], traj[:,2], traj[:,3],  markershape=:none, xlims=(-20,20), ylims=(-20,20), zlims=(-20,20), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
using DelimitedFiles
writedlm("Traj_$ind.csv", traj, ", ")


# end
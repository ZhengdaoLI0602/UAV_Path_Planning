include("TDM_Functions.jl")
include("Circle_Domain.jl")
include("TDM_Trajectory_Opt.jl")

using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics

using TrajectoryOptimization
using Altro

# Drone Parameters
mass = 0.4                                       # mass of quadrotor # default 0.5
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix # default Diagonal(@SVector[0.0023, 0.0023, 0.004])
# gravity = SVector(0,0,-3.721)                    # gravity vector # default SVector(0,0,-9.81) 
gravity = SVector(0,0,-9.81)
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u) # default 1.0
km = 0.0245                                      # motor torque constant (motor torque = km*u) # default 0.0245

r_max = 100 * tand(80/2)
N = 5
cir_domain = Circle_Domain.Domains_Problem([], 20.0, 2.0)
global MADS_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max)
x_start = Trajectory_Optimization.MADS_to_ALTRO(MADS_input) #----- testing on 0802 (with 5 UAVs in total)


# MPC optimization
hor = 3.0           # Prediction horizon length         (3 seconds in total)
dt = 0.1            # Time-step length per horizon      (want to separate 3 seconds into 30 pieces)
Nf = Int(hor/dt)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps



MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}() # ----- testing on just first UAV on 0802
for i in eachindex(x_start)
    push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start[i]))
end


for ind in eachindex(x_start)
    local MAV = MAVs[ind];
    local X = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf)

    # Document the data and make plots
    local traj = zeros(Nf,3) 
    for i in eachindex(X)
        traj[i,1] = X[i][1]
        traj[i,2] = X[i][2]
        traj[i,3] = X[i][3]
    end

    # plot!(traj[:,1], traj[:,2], traj[:,3],  markershape=:none, xlims=(-20,20), ylims=(-20,20), zlims=(-20,20), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    using DelimitedFiles
    writedlm("Traj_$ind.csv", traj, ", ")
    
end
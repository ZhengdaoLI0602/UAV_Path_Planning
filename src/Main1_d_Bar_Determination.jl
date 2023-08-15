include("TDM_Functions.jl")
include("TDM_Trajectory_Opt.jl")

using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using TrajectoryOptimization
using Altro

# Drone Parameters
mass = 0.5                                       # mass of quadrotor # default 0.5
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix # default Diagonal(@SVector[0.0023, 0.0023, 0.004])
gravity = SVector(0,0,-9.81)                     # gravity vector # default SVector(0,0,-9.81) 
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u) # default 1.0
km = 0.0245                                      # motor torque constant (motor torque = km*u) # default 0.0245
r_max = 100 * tand(80/2)
# N = 5                                            # number of UAVs


# MPC optimization
hor = 3.0           # Prediction horizon length         (3 seconds in total)
dt = 0.1            # Time-step length per horizon      (want to separate 3 seconds into 30 pieces)
Nf = Int(hor/dt)+1  # Number of timesteps per horizon
tf = hor            # for testing on 20230809


# Enum Altro.TerminationStatus:
# UNSOLVED = 0
# LINESEARCH_FAIL = 1
# SOLVE_SUCCEEDED = 2
# MAX_ITERATIONS = 3
# MAX_ITERATIONS_OUTER = 4
# MAXIMUM_COST = 5
# STATE_LIMIT = 6
# CONTROL_LIMIT = 7
# NO_PROGRESS = 8
# COST_INCREASE = 9


# cir_domain = TDM_Functions.Domains_Problem([], 20.0, 2.0) # 
# global MADS_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max) # may be recovered 20230814
# x_start = TDM_TRAJECTORY_opt.MADS_to_ALTRO(MADS_input)                         # may be recovered 20230814


# x_start_4test = RBState([-7.77469514777312, 0.7173085987290898, 7.545026699377439, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}() # ----- testing on just first UAV on 0802
# for i in 1:N
#     push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start[i]))
#     # push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start_4test)) # test on 20230810
# end





x_start_dBar = RBState([0.0,0.0,0.0,  1.0,0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0])
MAV = TDM_TRAJECTORY_opt.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start_dBar)


# include("TDM_Trajectory_Opt.jl")
# XF = 7.5
# X, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, XF);
# solver = solver_dBar
# println("Distance: $XF; Solver status: ", solver.stats.status) 
# println("Number of iterations: ", iterations(solver))
# println("Final cost: ", cost(solver))
# println("Final constraint satisfaction: ", max_violation(solver))

# critical_dis = zeros(N, 1)
critical_dis = 0.0
traj_s = []

# for ind in 1:N
    # ind = 1
    # MAV = MAVs[ind]; # local in for loop

    # global X = nothing
    # global solver_dBar = nothing
    # global prob_dBar = nothing
    # global obj_dBar = nothing

    sin_phi, cos_theta, sin_theta = √2/√3 , √2/2 ,√2/2
    sin_phi, cos_theta, sin_theta = 1 , 1 , 0

    for d in 0.0:0.1:100.0
        X, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, d, sin_phi, cos_theta, sin_theta); 
        println("Distance: $d; Solver status: ", solver_dBar.stats.status)
        if Int(solver_dBar.stats.status)!= 2    
        # Not successful
            # global critical_dis[ind, 1] = d-0.1  
            global critical_dis = d-0.1 
            # The previous trial of d is the last d makes problem converge
            break
        end
    end

    X, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, critical_dis, sin_phi, cos_theta, sin_theta); 
    # Document the data and make plots
    traj = zeros(Nf, 3) # local in for loop
    for i in eachindex(X)
        traj[i,1] = X[i][1]
        traj[i,2] = X[i][2]
        traj[i,3] = X[i][3]
    end
    push!(traj_s, traj)

    # plot!(traj[:,1], traj[:,2], traj[:,3],  markershape=:none, xlims=(-20,20), ylims=(-20,20), zlims=(-20,20), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    using DelimitedFiles
    # writedlm("Traj_$ind.csv", traj, ", ")
    writedlm("Traj_1.csv", traj, ", ")

# end

# a = Vector{Vector{Any}}(undef, 5)

# global b = Array{Vector{Any}}(undef, 4,3)

# b[1,1]=[1]

# for j in 1:4
#     for k in 1:3
#         global b[j,k]= [3]
#     end
# end

# b[1:end]

# b[1,:]





# include("TDM_TRAJECTORY_opt.jl")
# #using Trajectory_Optimization
# using StaticArrays, Rotations, LinearAlgebra
# using RobotZoo: Quadrotor
# using RobotDynamics

# # Drone Parameters
# mass = 0.4                                       # mass of quadrotor
# J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix
# gravity = SVector(0,0,-3.721)                    # gravity vector
# motor_dist = 0.1750                              # distance between motors
# kf = 1.0                                         # motor force constant (motor force = kf*u)
# km = 0.0245                                      # motor torque constant (motor torque = km*u)

# # MPC optimization
# hor = 3.0           # Prediction horizon length
# dt = 0.1            # Time-step length per horizon
# Nt = Int(hor/dt)+1  # Number of timesteps per horizon
# R_D = 10.0          # Danger radius
# R_C = 1.0           # Collision radius
# Nm = 5              # Number of applied time-steps


# times =[]
# for i in 1:100    # d ranges from (0.02: 0.02: 2)
#     global d = i/50
#     global x_start = TDM_TRAJECTORY_opt.MADS_to_ALTRO([0.0, 0.0, 0.0])
#     global x_final = TDM_TRAJECTORY_opt.MADS_to_ALTRO([d, 0.0, 0.0])
#     global MAV = TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[1],x_final[1])


#     global total_converge = false
#     time_recorder = []
#     while total_converge == false
#         println("-----------------------------Unconverged iteration-----------------------------") # for testing
#         global total_converge = true
        
#         # Optimize if MAV has not converged to final position
#         if TDM_TRAJECTORY_opt.converge(MAV) > 0.1  # may change later on
#             global total_converge = false
#             t = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt,Nm)
#             push!(time_recorder,t)  
#         end        
#     end
#     push!(times, time_recorder)

# end


include("TDM_Functions.jl")

a = TDM_Functions.Domains_Problem([],20.0,2.0)
a.domain_expand()



















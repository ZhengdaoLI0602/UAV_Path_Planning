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


# include("TDM_Functions.jl")

# a = TDM_Functions.Domains_Problem([],20.0,2.0)
# a.domain_expand()



include("../../FYP_Optimization-main/Base_Functions.jl")
# using .Base_Functions
include("../../FYP_Optimization-main/Greens_Method.jl")
# using .Greens_Method
include("../../FYP_Optimization-main/Plotter.jl")
using Random
include("TDM_Functions.jl")



M = 5               # (user-defined) Total number of map updates 
N = 5               # (user-defined) Total number of UAVs 
R1 = 100.0            # (user-defined) Initial radius 
ΔR = 5.0              # Expanding rate: 2m/update
FOV = 80/180*π      # FOV in radians
h_min = 1           # (user-defined, replaced later) Flying altitude lower bound (exclude initialization)
h_max = 20         # Flying altitude upper bound
r_min = h_min * tan(FOV/2) # (user-defined, replaced later)
# r_min = 0           # (user-defined, replaced later)
r_max = h_max * tan(FOV/2) 
d_lim = 17.5 #critical_dis           # (user-defined) limitations on displacement of group UAV induced from optimization 
N_iter = 100        # (use-defined) set the limit of iterations for coverage maximization



global STATIC_input = TDM_Functions.allocate_even_circles(5.0, N, 5.0)

circles_pool = Base_Functions.make_circles(STATIC_input) 
using Plots
plotlyjs()
plot()
TDM_Functions.show_epoch(circles_pool, cir_domain.Domain_History[1])



global count = 0
global success = true
for angle in 1:1:360
    success = true

    movement = 0.9*d_lim
    this_x = 0.0 + movement * cosd(angle)
    this_y = 0.0 + movement * sind(angle)
    this_r = 5.0
    global this_circle = Base_Functions.make_circles([this_x, this_y, this_r])

    for m in eachindex(circles_pool)
        if Base_Functions.intersection(this_circle[1], circles_pool[m]) !== nothing ||
            Base_Functions.contained(this_circle[1], circles_pool[m]) !== nothing
            success =  false
            break
        end
    end

    if success == true
        println("Success!")
        count += 1
    end
end

global success = true
for i =1:2
    success = false
end
println(success)









# random five circles
global STATIC_input = [-0.39202819916964404,34.872393940656,5.5239352511433015,-29.73691435996299,-22.73383564724028,
                    3.113693723318892,10.794042832857249,-2.3783477662587384,27.63667034017559,-1.1279665973334239,
                    5.025207348818531,4.676120462178847,8.538346582322927,1.6090792603854056,2.0937907421476356]


                    